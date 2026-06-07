/**
 * @file bsp_params.h
 * @brief L1/BSP — physical / electrical design constants and loop rates.
 *
 * These are PLANT parameters, not algorithm gains. PID gains live in the
 * control layer config so the control component stays plant-agnostic.
 */
#pragma once

/* -------- Drivetrain geometry -------- */
#define BSP_GEARBOX_RATIO        30.0f
#define BSP_ENC_PPR_MOTOR        11        /* pulses/rev at motor shaft, 1 channel */
#define BSP_ENC_DECODE_MULT      4         /* PCNT x4 */
#define BSP_ENC_COUNTS_PER_REV   (BSP_ENC_PPR_MOTOR * BSP_ENC_DECODE_MULT * 30) /* 1320 */
/* Encoder count polarity — Phase-4 bring-up calibration (measured 06/2026).
 * Sign is chosen so a FORWARD-driven wheel reads POSITIVE omega. It MUST agree
 * with motor direction, else the Phase-5 speed PID closes as POSITIVE feedback
 * and runs away. Mirror-mounted motors + identical wiring make the two sides
 * asymmetric: here LEFT needed -1, RIGHT stays +1 (RIGHT motor is DIR-inverted
 * below instead — see BSP_MOTOR_*_DIR_INVERT). */
#define BSP_ENC_L_SIGN           (-1)
#define BSP_ENC_R_SIGN           (+1)
/* Wheel-speed estimate low-pass cutoff (Hz). M-method gives 0.952 rad/s/count at
 * 200 Hz; this 1st-order LPF smooths that quantization dither. Keep it well above
 * the ~10 Hz closed-loop bandwidth so it doesn't eat phase margin. <=0 disables. */
#define BSP_ENC_OMEGA_LPF_FC_HZ  30.0f
#define BSP_WHEEL_DIAMETER_M     0.065f
#define BSP_WHEEL_RADIUS_M       (BSP_WHEEL_DIAMETER_M * 0.5f)
#define BSP_WHEEL_BASE_M         0.290f

/* -------- Actuation -------- */
#define BSP_PWM_FREQ_HZ          20000
/* Normalized [0,1] duty below which the wheel does not turn. 0 = linear duty->PWM
 * (honest baseline). Measure it with the Phase-4 motor sweep; in closed loop the
 * PID integral + feedforward usually cover the low end, so keep 0 unless the
 * open-loop dead band proves large. */
#define BSP_MOTOR_DEADZONE_DUTY  0.0f
/* DIR polarity — Phase-4 bring-up. 1 = invert so +duty drives the robot FORWARD.
 * The right motor is mirror-mounted: +duty spins it backward, so it is inverted. */
#define BSP_MOTOR_L_DIR_INVERT   0
#define BSP_MOTOR_R_DIR_INVERT   1

/* -------- Loop timing -------- */
#define BSP_CTRL_LOOP_HZ         200
#define BSP_CTRL_LOOP_DT_S       (1.0f / (float)BSP_CTRL_LOOP_HZ)
#define BSP_COMM_LOOP_HZ         100
#define BSP_TELEM_LOOP_HZ        50

/* -------- Safety -------- */
#define BSP_CMD_TIMEOUT_US       200000ULL /* lose Pi command > 200 ms -> stop */

/* -------- Power -------- */
#define BSP_VBAT_CELLS           3
#define BSP_VBAT_NOMINAL_V       11.1f
