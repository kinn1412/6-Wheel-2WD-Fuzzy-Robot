/**
 * @file bsp_pins.h
 * @brief L1/BSP — single source of truth for ESP32-S3 pin assignment.
 *
#pragma once

/* -------- Motor (MDD10A, sign-magnitude: PWM + DIR) -------- */
#define BSP_MOTOR_L_PWM_GPIO   4
#define BSP_MOTOR_L_DIR_GPIO   5
#define BSP_MOTOR_R_PWM_GPIO   6
#define BSP_MOTOR_R_DIR_GPIO   7

/* -------- Encoder (quadrature AB, PCNT x4 hardware decode) -------- */
#define BSP_ENC_L_A_GPIO       9
#define BSP_ENC_L_B_GPIO       10   /* GREEN wire (NOT white) */
#define BSP_ENC_R_A_GPIO       11
#define BSP_ENC_R_B_GPIO       12   /* GREEN wire (NOT white) */

/* -------- IMU BNO085 (SPI2 / FSPI) -------- */
#define BSP_IMU_SPI_HOST       SPI2_HOST
#define BSP_IMU_SCK_GPIO       14
#define BSP_IMU_MOSI_GPIO      21
#define BSP_IMU_MISO_GPIO      47
#define BSP_IMU_CS_GPIO        38   /* 10K pull-up to 3V3 required */
#define BSP_IMU_INT_GPIO       18   /* active-low data-ready */
#define BSP_IMU_RST_GPIO       8    /* active-low; 10K pull-up to 3V3 required */
#define BSP_IMU_SPI_CLK_HZ     (3 * 1000 * 1000)  /* <= 3 MHz */

/* -------- UART2 link to Raspberry Pi 4 (Phase 6+) -------- */
#define BSP_PI_UART_NUM        UART_NUM_2
#define BSP_PI_UART_TX_GPIO    17
#define BSP_PI_UART_RX_GPIO    16
#define BSP_PI_UART_BAUD       921600

/* -------- Status / misc -------- */
#define BSP_STATUS_LED_GPIO    48   /* WS2812 RGB (RMT). Try 38 if dark. */
#define BSP_ESTOP_GPIO         13   /* reserve, Phase 6+ */
