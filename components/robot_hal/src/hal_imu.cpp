/**
 * @file hal_imu.cpp
 * @brief L2 — BNO085 (SPI) abstraction over the esp32_BNO08x C++ driver, exposed
 *        behind the plain-C hal_imu.h API. Real backend gated by
 *        HAL_IMU_USE_BNO08X (set in CMakeLists); otherwise a yaw=0 stub.
 *
 * The driver runs its own internal tasks (pinned to Core 0 via sdkconfig). Our
 * task_imu polls hal_imu_wait_sample(), which blocks on data_available() and
 * latches yaw/yaw_rate into volatile floats — single-float reads are atomic on
 * the LX7, so the Core-1 control loop reads them lock-free.
 */
#include "hal_imu.h"
#include "bsp_pins.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "hal_imu";

/* Latched outputs — written by task_imu (Core 0), read by control loop (Core 1). */
static volatile float    s_yaw = 0.0f;
static volatile float    s_yaw_rate = 0.0f;
static volatile uint64_t s_last_us = 0;
static TaskHandle_t      s_notify = nullptr;
static bool              s_init = false;

#ifdef HAL_IMU_USE_BNO08X
#include <new>
#include "BNO08x.hpp"

#define IMU_REPORT_PERIOD_US  10000U    /* 100 Hz: Game Rotation Vector + Calibrated Gyro */
#define IMU_STALE_US          100000ULL /* no sample in 100 ms => stale                   */

static BNO08x *s_imu = nullptr;

esp_err_t hal_imu_init(void) {
    if (s_init) return ESP_OK;

    /* Pins from BSP (single source of truth) override the library Kconfig defaults. */
    bno08x_config_t cfg(
        BSP_IMU_SPI_HOST,
        static_cast<gpio_num_t>(BSP_IMU_MOSI_GPIO),   /* DI  */
        static_cast<gpio_num_t>(BSP_IMU_MISO_GPIO),   /* SDA / DO */
        static_cast<gpio_num_t>(BSP_IMU_SCK_GPIO),    /* SCL */
        static_cast<gpio_num_t>(BSP_IMU_CS_GPIO),
        static_cast<gpio_num_t>(BSP_IMU_INT_GPIO),    /* HINT */
        static_cast<gpio_num_t>(BSP_IMU_RST_GPIO),
        BSP_IMU_SPI_CLK_HZ,                           /* <= 3 MHz; drop to 2 MHz if flaky */
        true /* install GPIO ISR service */);

    s_imu = new (std::nothrow) BNO08x(cfg);
    if (!s_imu) return ESP_ERR_NO_MEM;

    if (!s_imu->initialize()) {
        ESP_LOGE(TAG, "BNO08x initialize() failed — check SPI/CS/INT/RST wiring + PSx/BOOTN ties");
        return ESP_FAIL;
    }
    if (!s_imu->rpt.rv_game.enable(IMU_REPORT_PERIOD_US) ||
        !s_imu->rpt.cal_gyro.enable(IMU_REPORT_PERIOD_US)) {
        ESP_LOGE(TAG, "BNO08x enable reports failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BNO08x ready: GameRV + CalGyro @ %u Hz", 1000000U / IMU_REPORT_PERIOD_US);
    s_init = true;
    return ESP_OK;
}

void hal_imu_set_notify_task(TaskHandle_t t) { s_notify = t; }

bool hal_imu_wait_sample(uint32_t timeout_ms) {
    (void)timeout_ms;                       /* driver uses its own data_available timeout */
    if (!s_init || !s_imu) return false;
    if (!s_imu->data_available()) return false;

    bool got = false;
    if (s_imu->rpt.rv_game.has_new_data()) {
        const bno08x_euler_angle_t e = s_imu->rpt.rv_game.get_euler(false /*radians*/);
        s_yaw = e.z;                        /* heading about Z, (-pi, pi] */
        got = true;
    }
    if (s_imu->rpt.cal_gyro.has_new_data()) {
        const bno08x_gyro_t g = s_imu->rpt.cal_gyro.get();
        s_yaw_rate = g.z;                   /* gyro Z, rad/s */
        got = true;
    }
    if (got) s_last_us = esp_timer_get_time();
    return got;
}

float hal_imu_get_yaw(void)      { return s_yaw; }
float hal_imu_get_yaw_rate(void) { return s_yaw_rate; }
bool  hal_imu_is_stale(uint64_t now_us) { return (now_us - s_last_us) > IMU_STALE_US; }

#else  /* ---------------- stub (yaw = 0) ---------------- */

esp_err_t hal_imu_init(void) {
    if (s_init) return ESP_OK;
    ESP_LOGI(TAG, "init (stub: yaw=0). Define HAL_IMU_USE_BNO08X for the real driver.");
    s_init = true;
    return ESP_OK;
}
void  hal_imu_set_notify_task(TaskHandle_t t) { s_notify = t; }
bool  hal_imu_wait_sample(uint32_t timeout_ms) { (void)timeout_ms; return false; }
float hal_imu_get_yaw(void)      { return s_yaw; }
float hal_imu_get_yaw_rate(void) { return s_yaw_rate; }
bool  hal_imu_is_stale(uint64_t now_us) { (void)now_us; return false; }

#endif
