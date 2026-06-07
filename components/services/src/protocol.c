#include "protocol.h"

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

size_t cobs_encode(const uint8_t *src, size_t len, uint8_t *dst, size_t dst_cap) {
    if (dst_cap < len + 2) return 0;
    size_t read = 0, write = 1, code_idx = 0;
    uint8_t code = 1;
    while (read < len) {
        if (src[read] == 0) {
            dst[code_idx] = code; code = 1; code_idx = write++;
        } else {
            dst[write++] = src[read]; code++;
            if (code == 0xFF) { dst[code_idx] = code; code = 1; code_idx = write++; }
        }
        read++;
    }
    dst[code_idx] = code;
    return write;
}

size_t cobs_decode(const uint8_t *src, size_t len, uint8_t *dst, size_t dst_cap) {
    size_t read = 0, write = 0;
    while (read < len) {
        uint8_t code = src[read++];
        if (code == 0) return 0;
        for (uint8_t i = 1; i < code && read < len; i++) {
            if (write >= dst_cap) return 0;
            dst[write++] = src[read++];
        }
        if (code != 0xFF && read < len) {
            if (write >= dst_cap) return 0;
            dst[write++] = 0;
        }
    }
    return write;
}
