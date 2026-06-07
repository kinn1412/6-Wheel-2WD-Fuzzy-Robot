/* Core 1, prio 15, 200 Hz. The only hard-real-time task. */
#include "app_internal.h"
#include "robot_fsm.h"
#include "bsp_params.h"
#include "hal_encoder.h"
#include "hal_imu.h"
#include "hal_motor.h"
#include "safety.h"
#include "esp_timer.h"

void task_speed_ctrl(void *arg) {
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / BSP_CTRL_LOOP_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    robot_state_e fsm = ROBOT_BOOT;
    uint64_t prev_us = esp_timer_get_time();

    for (;;) {
        vTaskDelayUntil(&last_wake, period);   /* fixed-frequency cadence */
        const uint64_t now = esp_timer_get_time();
        const float dt = (float)(now - prev_us) * 1e-6f;
        prev_us = now;

        /* --- inputs --- */
        const float wl = hal_encoder_get_omega(MOTOR_LEFT,  dt);
        const float wr = hal_encoder_get_omega(MOTOR_RIGHT, dt);
        const float yaw = hal_imu_get_yaw();
        const float yawr = hal_imu_get_yaw_rate();

        cmd_packet_t cmd = {0};
        xQueuePeek(g_cmd_mailbox, &cmd, 0);   /* latest command, non-blocking */

        /* --- watchdog + FSM --- */
        unsigned faults = FAULT_NONE;
        if (safety_cmd_timed_out(atomic_load(&g_last_cmd_us), now))
            faults |= FAULT_CMD_TIMEOUT;
        fsm = robot_fsm_step(fsm, faults, cmd.mode);

        float duty_l = 0.0f, duty_r = 0.0f;
        if (fsm == ROBOT_RUN) {
            ctrl_input_t in_l = { cmd.omega_ref_l, wl, yaw, yawr, dt };
            ctrl_input_t in_r = { cmd.omega_ref_r, wr, yaw, yawr, dt };
            duty_l = controller_update(&g_ctrl_l, &in_l).duty;
            duty_r = controller_update(&g_ctrl_r, &in_r).duty;
            /* TODO Phase 2: yaw-stability supervisory bias on duty_l/duty_r here. */
        } else {
            controller_reset(&g_ctrl_l);
            controller_reset(&g_ctrl_r);
            hal_motor_stop_all();
        }

        if (fsm == ROBOT_RUN) {
            hal_motor_set_duty(MOTOR_LEFT,  duty_l);
            hal_motor_set_duty(MOTOR_RIGHT, duty_r);
        }

        /* --- publish snapshot (seqlock) --- */
        robot_telemetry_t t = {
            .omega_meas_l = wl, .omega_meas_r = wr,
            .yaw = yaw, .yaw_rate = yawr,
            .duty_l = duty_l, .duty_r = duty_r,
            .fault_flags = faults, .timestamp_us = now,
        };
        rs_publish(&g_state, &t);
    }
}
