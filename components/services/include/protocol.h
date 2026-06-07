/**
 * @file protocol.h
 * @brief L3 — ESP<->Pi framing: COBS + CRC16. Phase 6+.
 * Frame: [0x7E][COBS(payload)][CRC16-LE][0x7E]
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

/* Frame delimiter. MUST be 0x00 because cobs_encode() eliminates 0x00 from the
 * body — so 0x00 is the only byte guaranteed never to appear inside a frame.
 * (0x7E would be unsafe: COBS output can contain 0x7E.) The Pi side must match. */
#define PROTO_DELIM   0x00

typedef struct __attribute__((packed)) {
    float    omega_ref_l, omega_ref_r;  /* rad/s */
    uint8_t  mode;                        /* 0=idle 1=run 2=estop */
    uint16_t seq;
} cmd_packet_t;

typedef struct __attribute__((packed)) {
    float    omega_meas_l, omega_meas_r;
    float    yaw, yaw_rate;
    float    pwm_l, pwm_r;
    float    vbat;
    uint16_t fault_flags;
    uint16_t seq;
} telem_packet_t;

/* CRC16-CCITT (poly 0x1021). */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/* COBS. dst must hold >= len + (len/254) + 2. Returns encoded length, 0 on error. */
size_t cobs_encode(const uint8_t *src, size_t len, uint8_t *dst, size_t dst_cap);
size_t cobs_decode(const uint8_t *src, size_t len, uint8_t *dst, size_t dst_cap);
