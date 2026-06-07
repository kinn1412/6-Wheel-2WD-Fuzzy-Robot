#include "app_tasks.h"
#include "app_internal.h"
#include "bsp_params.h"
#include "controller_pid.h"
#include "pid.h"
#include "hal_motor.h"
#include "hal_encoder.h"
#include "hal_imu.h"
#include "hal_uart_link.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "app";

/* ── Bring-up mode — exactly one. Wheels OFF ground for MOTOR_SWEEP/SPEED_STEP. ─
 *   NORMAL      : production — Pi commands via mailbox + FSM (Phase 8+)
 *   MOTOR_SWEEP : Phase-4 open-loop motor characterization
 *   SPEED_STEP  : Phase-5 closed-loop speed-PID step-response tuning
 *   IMU_TEST    : Phase-6 BNO085 verification (rotate by hand, watch yaw)       */
#define APP_MODE_NORMAL       0
#define APP_MODE_MOTOR_SWEEP  1
#define APP_MODE_SPEED_STEP   2
#define APP_MODE_IMU_TEST     3
#define APP_MODE              APP_MODE_NORMAL

/* ---- shared object definitions ---- */
robot_state_t    g_state;
QueueHandle_t    g_cmd_mailbox;
_Atomic uint64_t g_last_cmd_us;
controller_if_t  g_ctrl_l, g_ctrl_r;

#if APP_MODE == APP_MODE_NORMAL || APP_MODE == APP_MODE_SPEED_STEP
/* PID storage (static — no heap in control path). Only the closed-loop modes
 * bind controllers, so guard these out of the open-loop bring-up builds. */
static pid_t s_pid_l, s_pid_r;

/* Speed-loop gains. kff from Phase-4 sweep (omega ~= 34*duty -> kff ~= 1/34).
 * kp/ki are starting points — tune from the SPEED_STEP step response. */
static const pid_config_t PID_CFG = {
    .kp = 0.05f, .ki = 0.20f, .kd = 0.0f, .kff = 0.029f,
    .dt = BSP_CTRL_LOOP_DT_S, .out_min = -1.0f, .out_max = 1.0f, .kaw = 1.0f,
};
#endif

void app_tasks_start(void) {
    ESP_ERROR_CHECK(hal_motor_init());
    ESP_ERROR_CHECK(hal_encoder_init());
    if (hal_imu_init() != ESP_OK)           /* IMU is non-critical: never brick the robot */
        ESP_LOGW(TAG, "hal_imu_init failed — yaw/yaw_rate unavailable");
    ESP_ERROR_CHECK(hal_uart_link_init());

#if APP_MODE == APP_MODE_MOTOR_SWEEP
    xTaskCreatePinnedToCore(task_motor_bringup, "mtr_bringup", 4096, NULL, 6, NULL, 1);
    ESP_LOGW(TAG, "MODE=MOTOR_SWEEP — control stack down");
    return;
#elif APP_MODE == APP_MODE_IMU_TEST
    xTaskCreatePinnedToCore(task_imu_test, "imu_test", 4096, NULL, 6, NULL, 0);
    ESP_LOGW(TAG, "MODE=IMU_TEST — rotate robot by hand, watch yaw/yaw_rate");
    return;
#else
    rs_init(&g_state);
    g_cmd_mailbox = xQueueCreate(1, sizeof(cmd_packet_t));   /* mailbox */
    configASSERT(g_cmd_mailbox);
    atomic_store(&g_last_cmd_us, 0);

    /* Bind PID into the Strategy interface. Swap to MPC = swap these two lines. */
    pid_init(&s_pid_l, &PID_CFG);
    pid_init(&s_pid_r, &PID_CFG);
    g_ctrl_l = controller_pid_make(&s_pid_l);
    g_ctrl_r = controller_pid_make(&s_pid_r);

  #if APP_MODE == APP_MODE_SPEED_STEP
    /* Core 1: closed-loop step-response harness (drives the real PID). */
    xTaskCreatePinnedToCore(task_speed_test, "spd_test", 4096, NULL, 15, NULL, 1);
    ESP_LOGW(TAG, "MODE=SPEED_STEP — closed-loop step test (ctrl=%s)", g_ctrl_l.name);
  #else /* APP_MODE_NORMAL */
    /* Core 1 (APP_CPU): hard-real-time control, isolated from WiFi/BT on Core 0. */
    xTaskCreatePinnedToCore(task_speed_ctrl, "speed_ctrl", 4096, NULL, 15, NULL, 1);
    /* Core 0 (PRO_CPU): I/O-bound tasks tolerate jitter. */
    xTaskCreatePinnedToCore(task_imu,        "imu_spi",    4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(task_comm,       "comm",       4096, NULL,  8, NULL, 0);
    xTaskCreatePinnedToCore(task_telemetry,  "telemetry",  3072, NULL,  5, NULL, 0);
    ESP_LOGI(TAG, "tasks started (ctrl=%s)", g_ctrl_l.name);
  #endif
#endif
}
