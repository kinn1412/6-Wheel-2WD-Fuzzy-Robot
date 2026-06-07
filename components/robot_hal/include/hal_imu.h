/**
 * @file hal_imu.h
 * @brief L2 — BNO085 (SPI) abstraction. Exposes yaw + yaw_rate in SI units.
 *        Real backend gated by HAL_IMU_USE_BNO08X; otherwise stub (yaw=0).
 */
#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {            /* implemented in C++ (hal_imu.cpp), called from C */
#endif

esp_err_t hal_imu_init(void);
/** Register the task to notify from the data-ready ISR (INT pin). */
void      hal_imu_set_notify_task(TaskHandle_t t);
/** Block until next sample or timeout; pulls latest report from driver. */
bool      hal_imu_wait_sample(uint32_t timeout_ms);
float     hal_imu_get_yaw(void);       /* rad, wrapped (-pi, pi] */
float     hal_imu_get_yaw_rate(void);  /* rad/s, gyro Z (native) */
bool      hal_imu_is_stale(uint64_t now_us);

#ifdef __cplusplus
}
#endif
