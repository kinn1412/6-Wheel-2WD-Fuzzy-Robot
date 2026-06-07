/* Host unit test for the control layer (no hardware). Build with Unity/Ceedling.
 * Demonstrates the payoff of keeping `control` hardware-free: replay logged data. */
#include "pid.h"
/* #include "unity.h"  // provided by your host test harness */

/* Example skeleton (pseudo):
 *   pid_t p; pid_config_t c = { .kp=.05f,.ki=.2f,.kd=0,.kff=.02f,
 *                               .dt=0.005f,.out_min=-1,.out_max=1,.kaw=1 };
 *   pid_init(&p,&c);
 *   for each row in recorded_step_response.csv:
 *       float u = pid_update(&p, row.ref, row.meas, 0.005f);
 *       TEST_ASSERT_FLOAT_WITHIN(...);
 */
