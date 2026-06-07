/**
 * @file robot_fsm.h
 * @brief L5 — bring-up phase flags + runtime state machine.
 * Enable exactly ONE phase flag while bringing the robot up incrementally.
 */
#pragma once

/* ---- Phase bring-up flags (compile-time gate) ---- */
#define PHASE2_ENCODER_ONLY    0
#define PHASE3_IMU_SPI         0
#define PHASE4_MOTOR_OPENLOOP  0
#define PHASE5_SPEED_CLOSED    1   /* default: full closed-loop speed control */

typedef enum {
    ROBOT_BOOT = 0,
    ROBOT_IDLE,
    ROBOT_RUN,
    ROBOT_FAULT,
    ROBOT_ESTOP,
} robot_state_e;

robot_state_e robot_fsm_step(robot_state_e cur, unsigned fault_flags, int cmd_mode);
const char   *robot_fsm_name(robot_state_e s);
