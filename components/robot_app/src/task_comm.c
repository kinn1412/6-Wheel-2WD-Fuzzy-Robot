/* Core 0, prio 8. UART <- Pi: byte-stream framer -> verify -> cmd mailbox.
 *
 * Frame on the wire: [0x00][ COBS( cmd_packet_t || crc16_le ) ][0x00].
 * COBS removes 0x00 from the body, so 0x00 unambiguously delimits frames. CRC16
 * (CCITT, poly 0x1021, init 0xFFFF) is over the packed struct, appended LE.
 */
#include "app_internal.h"
#include "hal_uart_link.h"
#include "protocol.h"
#include "esp_timer.h"
#include <string.h>

#define CMD_LEN    ((size_t)sizeof(cmd_packet_t))   /* 11 (packed) */
#define FRAME_MAX  64

static uint8_t s_acc[FRAME_MAX];   /* COBS bytes accumulated between delimiters */
static size_t  s_acc_len = 0;

/* Decode one frame body, verify CRC, publish to the command mailbox. */
static void handle_frame(const uint8_t *body, size_t len) {
    uint8_t dec[FRAME_MAX];
    const size_t dlen = cobs_decode(body, len, dec, sizeof(dec));
    if (dlen != CMD_LEN + 2) return;                                /* wrong size */
    const uint16_t crc_rx = (uint16_t)dec[CMD_LEN] | ((uint16_t)dec[CMD_LEN + 1] << 8);
    if (crc16_ccitt(dec, CMD_LEN) != crc_rx) return;               /* bad CRC */

    cmd_packet_t cmd;
    memcpy(&cmd, dec, CMD_LEN);
    xQueueOverwrite(g_cmd_mailbox, &cmd);                          /* latest-only mailbox */
    atomic_store(&g_last_cmd_us, (uint64_t)esp_timer_get_time());  /* feed the watchdog */
}

void task_comm(void *arg) {
    (void)arg;
    uint8_t rx[128];
    for (;;) {
        const int n = hal_uart_link_read(rx, sizeof(rx), 20 /*ms*/);
        for (int i = 0; i < n; i++) {
            const uint8_t b = rx[i];
            if (b == PROTO_DELIM) {
                if (s_acc_len > 0) { handle_frame(s_acc, s_acc_len); s_acc_len = 0; }
            } else if (s_acc_len < FRAME_MAX) {
                s_acc[s_acc_len++] = b;
            } else {
                s_acc_len = 0;   /* overflow -> drop, resync at next delimiter */
            }
        }
    }
}
