#include "controller_if.h"
#include "pid.h"

static ctrl_output_t pid_update_trampoline(void *ctx, const ctrl_input_t *in) {
    pid_t *p = (pid_t *)ctx;
    float duty = pid_update(p, in->omega_ref, in->omega_meas, in->dt);
    ctrl_output_t out = {
        .duty = duty,
        .saturated = (duty <= p->cfg.out_min || duty >= p->cfg.out_max),
    };
    return out;
}

static void pid_reset_trampoline(void *ctx) { pid_reset((pid_t *)ctx); }

controller_if_t controller_pid_make(pid_t *pid) {
    controller_if_t iface = {
        .reset  = pid_reset_trampoline,
        .update = pid_update_trampoline,
        .name   = "pid",
        .ctx    = pid,
    };
    return iface;
}
