/**
 * @file hal_motor.c
 * @brief L2 — motor abstraction over MCPWM + DIR GPIO (MDD10A sign-magnitude).
 *        One 20 kHz timer drives both channels; caller works in normalized duty
 *        [-1,1] (sign -> DIR pin, magnitude -> PWM compare). PWM ticks & deadzone
 *        comp are hidden here.
 */
#include "hal_motor.h"
#include "bsp_pins.h"
#include "bsp_params.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "hal_motor";

/* 10 MHz timebase divides 160 MHz (PLL_F160M) cleanly; 500 ticks @ 20 kHz gives
 * 0.2 %/step duty resolution — ample for a speed loop. */
#define MOTOR_PWM_RES_HZ        (10 * 1000 * 1000)
#define MOTOR_PWM_PERIOD_TICKS  (MOTOR_PWM_RES_HZ / BSP_PWM_FREQ_HZ)

typedef struct {
    int pwm_gpio;
    int dir_gpio;
    int dir_invert;   /* 1 = flip DIR so +duty drives the robot forward */
} motor_cfg_t;

static const motor_cfg_t s_cfg[MOTOR_COUNT] = {
    [MOTOR_LEFT]  = { BSP_MOTOR_L_PWM_GPIO, BSP_MOTOR_L_DIR_GPIO, BSP_MOTOR_L_DIR_INVERT },
    [MOTOR_RIGHT] = { BSP_MOTOR_R_PWM_GPIO, BSP_MOTOR_R_DIR_GPIO, BSP_MOTOR_R_DIR_INVERT },
};

static bool                s_init = false;
static mcpwm_cmpr_handle_t s_cmp[MOTOR_COUNT];   /* per-motor compare -> duty */

/* Map normalized [0,1] magnitude through the dead band so small duty still moves
 * the wheel. With BSP_MOTOR_DEADZONE_DUTY = 0 this is the identity (linear). */
static float apply_deadzone(float mag) {
    if (mag <= 1e-3f) return 0.0f;
    const float dz = BSP_MOTOR_DEADZONE_DUTY;
    return dz + (1.0f - dz) * mag;
}

esp_err_t hal_motor_init(void) {
    if (s_init) return ESP_OK;

    /* Single timer, shared by both operators (both motors run the same 20 kHz). */
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t tcfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MOTOR_PWM_RES_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = MOTOR_PWM_PERIOD_TICKS,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&tcfg, &timer), TAG, "new_timer");

    for (motor_id_t id = 0; id < MOTOR_COUNT; ++id) {
        /* DIR = plain push-pull GPIO; start LOW (reverse) — PWM is 0 so motor off. */
        gpio_config_t dir = {
            .pin_bit_mask = 1ULL << s_cfg[id].dir_gpio,
            .mode         = GPIO_MODE_OUTPUT,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&dir), TAG, "dir_gpio[%d]", id);
        gpio_set_level(s_cfg[id].dir_gpio, 0);

        mcpwm_oper_handle_t oper = NULL;
        mcpwm_operator_config_t ocfg = { .group_id = 0 };
        ESP_RETURN_ON_ERROR(mcpwm_new_operator(&ocfg, &oper), TAG, "new_oper[%d]", id);
        ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(oper, timer), TAG, "connect[%d]", id);

        mcpwm_comparator_config_t ccfg = { .flags.update_cmp_on_tez = true };  /* glitch-free duty update */
        ESP_RETURN_ON_ERROR(mcpwm_new_comparator(oper, &ccfg, &s_cmp[id]), TAG, "cmp[%d]", id);
        ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_cmp[id], 0), TAG, "cmp0[%d]", id);

        mcpwm_gen_handle_t gen = NULL;
        mcpwm_generator_config_t gcfg = { .gen_gpio_num = s_cfg[id].pwm_gpio };
        ESP_RETURN_ON_ERROR(mcpwm_new_generator(oper, &gcfg, &gen), TAG, "gen[%d]", id);

        /* Edge-aligned PWM: HIGH at period start (counter empty), LOW at compare. */
        ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)),
            TAG, "act_t[%d]", id);
        ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_cmp[id], MCPWM_GEN_ACTION_LOW)),
            TAG, "act_c[%d]", id);
    }

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(timer), TAG, "timer_enable");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP), TAG, "timer_start");

    s_init = true;
    ESP_LOGI(TAG, "init MCPWM %d Hz (%d ticks/period) deadzone=%.2f",
             BSP_PWM_FREQ_HZ, (int)MOTOR_PWM_PERIOD_TICKS, BSP_MOTOR_DEADZONE_DUTY);
    return ESP_OK;
}

esp_err_t hal_motor_set_duty(motor_id_t id, float duty) {
    if (!s_init) return ESP_ERR_INVALID_STATE;
    if (id >= MOTOR_COUNT) return ESP_ERR_INVALID_ARG;
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;

    int dir = (duty >= 0.0f) ? 1 : 0;                  /* MDD10A: 1=fwd, 0=rev (raw) */
    if (s_cfg[id].dir_invert) dir = !dir;              /* mirror-mounted motor */
    const float mag = apply_deadzone(fabsf(duty));     /* [0,1] */
    uint32_t ticks = (uint32_t)lroundf(mag * (float)MOTOR_PWM_PERIOD_TICKS);
    if (ticks > MOTOR_PWM_PERIOD_TICKS) ticks = MOTOR_PWM_PERIOD_TICKS;

    gpio_set_level(s_cfg[id].dir_gpio, dir);
    return mcpwm_comparator_set_compare_value(s_cmp[id], ticks);
}

/* MDD10A sign-magnitude has no active brake line; PWM=0 lets the motor coast. */
esp_err_t hal_motor_brake(motor_id_t id) { return hal_motor_set_duty(id, 0.0f); }

esp_err_t hal_motor_stop_all(void) {
    esp_err_t e1 = hal_motor_set_duty(MOTOR_LEFT,  0.0f);
    esp_err_t e2 = hal_motor_set_duty(MOTOR_RIGHT, 0.0f);
    return (e1 != ESP_OK) ? e1 : e2;
}
