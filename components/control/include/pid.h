/**
 * @file pid.h
 * @brief L4 — Wheel-speed PID: positional form, feedforward, derivative on
 *        measurement, back-calculation anti-windup. Plant-agnostic, no I/O.
 */
#pragma once
#include <stdbool.h>

typedef struct {
    float kp, ki, kd;   /**< gains */
    float kff;          /**< feedforward gain on omega_ref */
    float dt;           /**< nominal sample time (s) */
    float out_min;      /**< saturation lower bound (e.g. -1.0) */
    float out_max;      /**< saturation upper bound (e.g. +1.0) */
    float kaw;          /**< back-calculation anti-windup gain   */
} pid_config_t;

typedef struct {
    pid_config_t cfg;
    /* runtime state */
    float integ;        /**< integrator accumulator */
    float prev_meas;    /**< for derivative-on-measurement */
    bool  initialized;
} pid_t;

void  pid_init(pid_t *s, const pid_config_t *cfg);
void  pid_reset(pid_t *s);
/** @return saturated control output. `dt_actual<=0` => use cfg.dt. */
float pid_update(pid_t *s, float ref, float meas, float dt_actual);
