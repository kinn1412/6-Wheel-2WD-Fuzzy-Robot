/**
 * @file robot_state.h
 * @brief L3 — lock-free shared telemetry (SEQLOCK).
 *
 * Single writer (speed_ctrl_task, Core 1), multiple readers (comm/telemetry,
 * Core 0). Writer is wait-free (never blocks the 200 Hz loop); readers retry
 * only if they race an in-progress write. Use this when multiple fields must be
 * read as ONE consistent snapshot. For a single scalar, a plain _Atomic float
 * is cheaper — do not over-engineer.
 */
#pragma once
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    float    omega_meas_l, omega_meas_r;  /* rad/s */
    float    yaw, yaw_rate;               /* rad, rad/s */
    float    duty_l, duty_r;              /* [-1,1] */
    float    vbat;                        /* V */
    uint32_t fault_flags;
    uint32_t seq_no;                      /* monotonic packet counter */
    uint64_t timestamp_us;
} robot_telemetry_t;

typedef struct {
    _Atomic uint32_t  seq;   /* even = stable, odd = write in progress */
    robot_telemetry_t data;
} robot_state_t;

static inline void rs_init(robot_state_t *s) {
    atomic_store_explicit(&s->seq, 0u, memory_order_relaxed);
    memset(&s->data, 0, sizeof(s->data));
}

/* WRITER — wait-free */
static inline void rs_publish(robot_state_t *s, const robot_telemetry_t *src) {
    uint32_t v = atomic_load_explicit(&s->seq, memory_order_relaxed);
    atomic_store_explicit(&s->seq, v + 1u, memory_order_relaxed);   /* odd */
    atomic_thread_fence(memory_order_release);
    s->data = *src;
    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&s->seq, v + 2u, memory_order_relaxed);   /* even */
}

/* READER — retry on race */
static inline void rs_read(robot_state_t *s, robot_telemetry_t *dst) {
    uint32_t before, after;
    do {
        before = atomic_load_explicit(&s->seq, memory_order_acquire);
        while (before & 1u)
            before = atomic_load_explicit(&s->seq, memory_order_acquire);
        atomic_thread_fence(memory_order_acquire);
        memcpy(dst, &s->data, sizeof(*dst));
        atomic_thread_fence(memory_order_acquire);
        after = atomic_load_explicit(&s->seq, memory_order_acquire);
    } while (before != after);
}
