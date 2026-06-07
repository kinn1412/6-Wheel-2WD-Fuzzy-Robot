/**
 * @file hal_encoder.h
 * @brief L2 — quadrature encoder via PCNT (x4 hardware decode). Count read is a
 *        single atomic 32-bit access; no task or mutex required.
 */
#pragma once
#include "esp_err.h"
#include "hal_motor.h"   /* reuse motor_id_t for left/right indexing */
#include <stdint.h>

esp_err_t hal_encoder_init(void);
/** Raw accumulated signed count since boot. */
esp_err_t hal_encoder_get_count(motor_id_t id, int32_t *out_count);
/** Wheel angular speed (rad/s) from delta-count over dt; updates internal prev. */
float     hal_encoder_get_omega(motor_id_t id, float dt_s);
