#include "odometry.h"
#include "bsp_params.h"

body_twist_t odom_forward(float omega_l, float omega_r) {
    const float r = BSP_WHEEL_RADIUS_M;
    const float L = BSP_WHEEL_BASE_M;
    body_twist_t t;
    t.v          = r * (omega_r + omega_l) * 0.5f;
    t.omega_body = r * (omega_r - omega_l) / L;
    return t;
}

void odom_inverse(float v, float omega_body, float *omega_l_ref, float *omega_r_ref) {
    const float r = BSP_WHEEL_RADIUS_M;
    const float L = BSP_WHEEL_BASE_M;
    *omega_l_ref = (v - omega_body * L * 0.5f) / r;
    *omega_r_ref = (v + omega_body * L * 0.5f) / r;
}
