/* Core 0, prio 10, INT-driven (~200 Hz). Wakes on BNO085 data-ready. */
#include "app_internal.h"
#include "hal_imu.h"

void task_imu(void *arg) {
    (void)arg;
    hal_imu_set_notify_task(xTaskGetCurrentTaskHandle());  /* ISR -> notify us */
    for (;;) {
        /* Real backend blocks on the data-ready notification; stub sleeps. */
        if (!hal_imu_wait_sample(20 /*ms*/)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        /* hal_imu_* internally latched yaw/yaw_rate; control task reads them.
           (Single-scalar volatile floats are atomic on Xtensa LX7 -> no lock.) */
    }
}
