/**
 * @file safety.h
 * @brief L3 — fault model + command watchdog policy.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FAULT_NONE          = 0,
    FAULT_CMD_TIMEOUT   = 1u << 0,  /* lost Pi command */
    FAULT_IMU_STALE     = 1u << 1,
    FAULT_ENC_INVALID   = 1u << 2,
    FAULT_VBAT_LOW      = 1u << 3,
    FAULT_ESTOP         = 1u << 4,
} fault_bit_t;

/** True if the last accepted command is older than BSP_CMD_TIMEOUT_US. */
bool safety_cmd_timed_out(uint64_t last_cmd_us, uint64_t now_us);
