/**
 * @file hal_uart_link.c
 * @brief L2 — raw byte transport over UART2 to the Pi. Framing (COBS+CRC) is L3.
 */
#include "hal_uart_link.h"
#include "bsp_pins.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "hal_uart";
static bool s_init = false;

#define UART_RX_BUF  2048   /* >> per-cycle traffic at 921600 baud */
#define UART_TX_BUF  512

esp_err_t hal_uart_link_init(void) {
    if (s_init) return ESP_OK;

    const uart_config_t cfg = {
        .baud_rate  = BSP_PI_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(BSP_PI_UART_NUM, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0),
                        TAG, "driver_install");
    ESP_RETURN_ON_ERROR(uart_param_config(BSP_PI_UART_NUM, &cfg), TAG, "param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(BSP_PI_UART_NUM, BSP_PI_UART_TX_GPIO, BSP_PI_UART_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "set_pin");
    ESP_LOGI(TAG, "UART%d up: %d baud TX=%d RX=%d", (int)BSP_PI_UART_NUM, BSP_PI_UART_BAUD,
             BSP_PI_UART_TX_GPIO, BSP_PI_UART_RX_GPIO);
    s_init = true;
    return ESP_OK;
}

int hal_uart_link_read(uint8_t *buf, size_t cap, uint32_t timeout_ms) {
    if (!s_init || !buf) return -1;
    return uart_read_bytes(BSP_PI_UART_NUM, buf, cap, pdMS_TO_TICKS(timeout_ms));
}

int hal_uart_link_write(const uint8_t *buf, size_t len) {
    if (!s_init || !buf) return -1;
    return uart_write_bytes(BSP_PI_UART_NUM, buf, len);
}
