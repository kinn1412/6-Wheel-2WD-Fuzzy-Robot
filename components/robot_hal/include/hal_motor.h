/**
 * @file hal_motor.h
 * @brief L2 — motor abstraction over MCPWM + DIR GPIO (MDD10A sign-magnitude).
 * Caller works in normalized duty [-1,1]; deadzone comp & PWM ticks hidden here.
 */
#pragma once
#include "esp_err.h"

typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT = 1, MOTOR_COUNT } motor_id_t;

esp_err_t hal_motor_init(void);
/** duty in [-1.0,+1.0]: sign -> DIR pin, magnitude -> PWM (deadzone applied). */
esp_err_t hal_motor_set_duty(motor_id_t id, float duty);
esp_err_t hal_motor_brake(motor_id_t id);
esp_err_t hal_motor_stop_all(void);
