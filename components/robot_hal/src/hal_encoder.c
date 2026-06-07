/**
 * @file hal_encoder.c
 * @brief L2 — quadrature encoder bring-up (Phase 3). PCNT x4 hardware decode +
 *        glitch filter. The 16-bit HW counter is auto-extended to a full 32-bit
 *        value by the driver (flags.accum_count + watch points), so reads never
 *        wrap and need no task or mutex — get_count is a spinlock-guarded read.
 */
#include "hal_encoder.h"
#include "bsp_pins.h"
#include "bsp_params.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "hal_encoder";

/* The ESP32-S3 PCNT counter is 16-bit signed. We ring it at these limits and
 * let the driver accumulate the limit into a 32-bit value inside the watch-point
 * ISR (enabled by flags.accum_count). Max edge rate is ~7.3 kcnt/s, so a limit
 * crossing — and thus an ISR — happens only every few seconds. */
#define ENC_PCNT_HIGH_LIMIT   (32000)
#define ENC_PCNT_LOW_LIMIT    (-32000)
/* Hall edges are >100 us apart even at max RPM; 1 us rejects contact noise. */
#define ENC_GLITCH_NS         (1000)

typedef struct {
    int     gpio_a;
    int     gpio_b;
    int32_t sign;   /* +1 native; -1 if the wheel reads negative when driven forward */
} enc_cfg_t;

static const enc_cfg_t s_cfg[MOTOR_COUNT] = {
    [MOTOR_LEFT]  = { BSP_ENC_L_A_GPIO, BSP_ENC_L_B_GPIO, BSP_ENC_L_SIGN },
    [MOTOR_RIGHT] = { BSP_ENC_R_A_GPIO, BSP_ENC_R_B_GPIO, BSP_ENC_R_SIGN },
};

static bool               s_init = false;
static pcnt_unit_handle_t s_unit[MOTOR_COUNT];
static int32_t            s_prev[MOTOR_COUNT];
static float              s_omega_filt[MOTOR_COUNT];   /* velocity LPF state */

/* Configure one PCNT unit for x4 quadrature decode on its A/B pins. */
static esp_err_t enc_setup_unit(motor_id_t id) {
    const enc_cfg_t *c = &s_cfg[id];

    pcnt_unit_config_t ucfg = {
        .high_limit = ENC_PCNT_HIGH_LIMIT,
        .low_limit  = ENC_PCNT_LOW_LIMIT,
        .flags.accum_count = 1,     /* HW 16-bit -> SW-extended 32-bit on overflow */
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &s_unit[id]), TAG, "new_unit[%d]", id);

    pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = ENC_GLITCH_NS };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_unit[id], &fcfg), TAG, "filter[%d]", id);

    /* Two channels = x4 decode: each edge of BOTH phases counts, and the level
     * of the *other* phase selects count direction. */
    pcnt_channel_handle_t chan_a, chan_b;
    pcnt_chan_config_t a_cfg = { .edge_gpio_num = c->gpio_a, .level_gpio_num = c->gpio_b };
    pcnt_chan_config_t b_cfg = { .edge_gpio_num = c->gpio_b, .level_gpio_num = c->gpio_a };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit[id], &a_cfg, &chan_a), TAG, "chan_a[%d]", id);
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit[id], &b_cfg, &chan_b), TAG, "chan_b[%d]", id);

    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE), TAG, "a_edge");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "a_level");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE), TAG, "b_edge");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "b_level");

    /* Watch points at the ring limits drive the accumulate-on-overflow ISR. */
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s_unit[id], ENC_PCNT_HIGH_LIMIT), TAG, "wp_hi");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s_unit[id], ENC_PCNT_LOW_LIMIT),  TAG, "wp_lo");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_unit[id]),      TAG, "enable[%d]", id);
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_unit[id]), TAG, "clear[%d]", id);
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_unit[id]),       TAG, "start[%d]", id);
    return ESP_OK;
}

esp_err_t hal_encoder_init(void) {
    if (s_init) return ESP_OK;
    for (motor_id_t id = 0; id < MOTOR_COUNT; ++id) {
        s_prev[id] = 0;
        s_omega_filt[id] = 0.0f;
        ESP_RETURN_ON_ERROR(enc_setup_unit(id), TAG, "setup unit %d", id);
    }
    ESP_LOGI(TAG, "init x4 CPR=%d glitch=%dns sign[L,R]=%d,%d",
             BSP_ENC_COUNTS_PER_REV, ENC_GLITCH_NS,
             (int)s_cfg[MOTOR_LEFT].sign, (int)s_cfg[MOTOR_RIGHT].sign);
    s_init = true;
    return ESP_OK;
}

esp_err_t hal_encoder_get_count(motor_id_t id, int32_t *out_count) {
    if (id >= MOTOR_COUNT || !out_count) return ESP_ERR_INVALID_ARG;
    if (!s_init) return ESP_ERR_INVALID_STATE;
    int raw = 0;
    esp_err_t err = pcnt_unit_get_count(s_unit[id], &raw);   /* spinlock-atomic vs ISR */
    if (err != ESP_OK) return err;
    *out_count = s_cfg[id].sign * (int32_t)raw;
    return ESP_OK;
}
    
float hal_encoder_get_omega(motor_id_t id, float dt_s) {
    if (id >= MOTOR_COUNT || dt_s <= 0.0f) return 0.0f;
    int32_t now = 0;
    hal_encoder_get_count(id, &now);
    const int32_t d = now - s_prev[id];   /* int32 delta stays correct across HW wrap */
    s_prev[id] = now;
    /* M-method: omega = 2*pi * (delta / CPR) / dt. Coarsely quantized at 200 Hz
     * (0.952 rad/s/count), so smooth it with a 1st-order LPF. */
    const float omega_raw = (2.0f * (float)M_PI * (float)d) / ((float)BSP_ENC_COUNTS_PER_REV * dt_s);

    const float fc = BSP_ENC_OMEGA_LPF_FC_HZ;
    if (fc <= 0.0f) return omega_raw;                       /* filter disabled */
    const float tau = 1.0f / (2.0f * (float)M_PI * fc);
    const float a = dt_s / (tau + dt_s);                    /* dt-aware alpha */
    s_omega_filt[id] += a * (omega_raw - s_omega_filt[id]);
    return s_omega_filt[id];
}
