#include "pid.h"

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

void pid_init(pid_t *s, const pid_config_t *cfg) {
    s->cfg = *cfg;
    pid_reset(s);
}

void pid_reset(pid_t *s) {
    s->integ = 0.0f;
    s->prev_meas = 0.0f;
    s->initialized = false;
}

float pid_update(pid_t *s, float ref, float meas, float dt_actual) {
    const float dt = (dt_actual > 0.0f) ? dt_actual : s->cfg.dt;
    const float err = ref - meas;

    /* derivative on measurement (avoids derivative kick on setpoint steps) */
    if (!s->initialized) { s->prev_meas = meas; s->initialized = true; }
    const float deriv = -(meas - s->prev_meas) / dt;
    s->prev_meas = meas;

    const float ff   = s->cfg.kff * ref;
    const float prop = s->cfg.kp  * err;
    const float intg = s->cfg.ki  * s->integ;
    const float u    = ff + prop + intg + s->cfg.kd * deriv;

    const float u_sat = clampf(u, s->cfg.out_min, s->cfg.out_max);

    /* back-calculation anti-windup: bleed integrator by the saturation error */
    s->integ += (err + s->cfg.kaw * (u_sat - u)) * dt;

    return u_sat;
}
