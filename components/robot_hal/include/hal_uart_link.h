/**
 * @file hal_uart_link.h
 * @brief L2 — raw byte transport over UART2 to the Pi. Framing is L3 (protocol).
 */
#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

esp_err_t hal_uart_link_init(void);
int       hal_uart_link_read(uint8_t *buf, size_t cap, uint32_t timeout_ms);
int       hal_uart_link_write(const uint8_t *buf, size_t len);
