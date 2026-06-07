/**
 * @file controller_if.h
 * @brief L4 — Strategy interface. PID, Feedforward, Fuzzy, MPC all implement
 *        this same vtable, so the control task is decoupled from the algorithm.
 */
#pragma once
#include <stdbool.h>

typedef struct {
    float omega_ref;    /**< rad/s desired wheel angular speed */
    float omega_meas;   /**< rad/s measured (from PCNT)        */
    float yaw;          /**< rad   (BNO085 Game Rotation Vector) */
    float yaw_rate;     /**< rad/s (BNO085 calibrated gyro Z)    */
    float dt;           /**< s     actual measured loop period   */
} ctrl_input_t;

typedef struct {
    float duty;         /**< [-1.0, +1.0] normalized motor command */
    bool  saturated;    /**< true if output hit a limit this step  */
} ctrl_output_t;

/** Strategy vtable. Concrete state lives behind `ctx` (opaque pointer). */
typedef struct controller_if {
    void          (*reset)(void *ctx);
    ctrl_output_t (*update)(void *ctx, const ctrl_input_t *in);
    const char   *name;     /**< "pid" | "fuzzy" | "mpc" — for telemetry */
    void         *ctx;      /**< concrete controller state */
} controller_if_t;

static inline ctrl_output_t controller_update(const controller_if_t *c,
                                               const ctrl_input_t *in) {
    return c->update(c->ctx, in);
}
static inline void controller_reset(const controller_if_t *c) {
    if (c->reset) c->reset(c->ctx);
}
