/**
 * @file feedforward.h
 * @brief L4 — static feedforward map omega_ref -> nominal duty (kff*ref + deadzone
 *        compensation handled in HAL). Stub: extend with a measured lookup table.
 */
#pragma once
static inline float feedforward_duty(float omega_ref, float kff) {
    return kff * omega_ref;
}
