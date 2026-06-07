/**
 * @file task_imu_test.c
 * @brief Phase-6 BRING-UP. IMU verification — poll the BNO085 and log yaw +
 *        yaw_rate. No motors. Rotate the robot BY HAND: yaw should track heading
 *        (sweep toward +/-180 deg) and yaw_rate should spike with the rotation,
 *        return to ~0 when still. Build with APP_MODE = APP_MODE_IMU_TEST.
 */
#include "app_internal.h"
#include "hal_imu.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "imu_test";

void task_imu_test(void *arg) {
    (void)arg;
    ESP_LOGW(TAG, "=== IMU TEST: rotate robot by hand; yaw should track heading ===");
    int n = 0, miss = 0;
    for (;;) {
        if (hal_imu_wait_sample(100)) {        /* blocks on data_available, latches */
            miss = 0;
            if (++n % 10 == 0)                 /* ~10 Hz log when data ~100 Hz */
                ESP_LOGI(TAG, "yaw=%+7.1f deg  yaw_rate=%+6.2f rad/s",
                         hal_imu_get_yaw() * 180.0f / (float)M_PI, hal_imu_get_yaw_rate());
        } else {
            if (++miss <= 3 || (miss % 25) == 0)
                ESP_LOGW(TAG, "no IMU data (check SPI/CS/INT/RST wiring, PSx ties, power)");
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}
