/**
 * @file odometry.h
 * @brief L3 — convert wheel angular speeds -> body twist (v, omega_body).
 *        Pure math; plant geometry comes from bsp_params.h.
 */
#pragma once

typedef struct {
    float v;          /**< m/s   body linear speed     */
    float omega_body; /**< rad/s body yaw rate (kinematic) */
} body_twist_t;

/** Differential-drive forward kinematics. wheel speeds in rad/s. */
body_twist_t odom_forward(float omega_l, float omega_r);

/** Inverse: body twist -> wheel angular speed references (rad/s). */
void odom_inverse(float v, float omega_body, float *omega_l_ref, float *omega_r_ref);
