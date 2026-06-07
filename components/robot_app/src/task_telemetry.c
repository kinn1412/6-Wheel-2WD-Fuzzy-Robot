/* Core 0, prio 5, 50 Hz. Seqlock snapshot -> telem_packet_t -> COBS+CRC -> UART.
 *
 * Frame on the wire: [0x00][ COBS( telem_packet_t || crc16_le ) ][0x00].
 * Also prints a ~1 Hz line to the USB console (UART0) for local sanity.
 */
#include "app_internal.h"
#include "bsp_params.h"
#include "hal_uart_link.h"
#include "protocol.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "telem";

#define TLEN ((size_t)sizeof(telem_packet_t))   /* 32 (packed) */

void task_telemetry(void *arg) {
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / BSP_TELEM_LOOP_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint16_t seq = 0;
    int log_div = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        robot_telemetry_t t;
        rs_read(&g_state, &t);                       /* consistent multi-field snapshot */

        const telem_packet_t tp = {
            .omega_meas_l = t.omega_meas_l,
            .omega_meas_r = t.omega_meas_r,
            .yaw          = t.yaw,
            .yaw_rate     = t.yaw_rate,
            .pwm_l        = t.duty_l,                 /* normalized duty [-1,1] */
            .pwm_r        = t.duty_r,
            .vbat         = t.vbat,
            .fault_flags  = (uint16_t)t.fault_flags,
            .seq          = seq++,
        };

        /* payload || crc16_le, then COBS, then 0x00-delimit both ends. */
        uint8_t pc[TLEN + 2];
        memcpy(pc, &tp, TLEN);
        const uint16_t crc = crc16_ccitt(pc, TLEN);
        pc[TLEN]     = (uint8_t)(crc & 0xFF);
        pc[TLEN + 1] = (uint8_t)(crc >> 8);

        uint8_t out[TLEN + 8];
        out[0] = PROTO_DELIM;
        const size_t enc = cobs_encode(pc, sizeof(pc), &out[1], sizeof(out) - 2);
        if (enc) {
            out[1 + enc] = PROTO_DELIM;
            hal_uart_link_write(out, 1 + enc + 1);
        }

        if (++log_div >= BSP_TELEM_LOOP_HZ) {        /* ~1 Hz to console */
            log_div = 0;
            ESP_LOGI(TAG, "wl=%.2f wr=%.2f yaw=%.2f yr=%.2f dL=%.2f dR=%.2f flt=0x%02x",
                     t.omega_meas_l, t.omega_meas_r, t.yaw, t.yaw_rate,
                     t.duty_l, t.duty_r, (unsigned)t.fault_flags);
        }
    }
}
