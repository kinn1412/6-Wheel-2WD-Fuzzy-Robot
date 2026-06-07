# CONTEXT – Probot: Robot Vi Sai Differential-Drive
## Phiên bản: Phase 6 — UART ESP↔Pi đang triển khai (cập nhật 04/06/2026)

---

## 5. Phase tiếp theo — Việc cần làm cụ thể

Toàn bộ protocol layer (COBS + CRC16) và packet struct đã implement sẵn trong `services/protocol.h` và `services/protocol.c`. Chỉ cần **điền logic vào 2 task**:

### 5.1 task_comm.c — Parse frame nhận từ Pi

Hiện tại là stub. Cần implement:
1. `hal_uart_link_read()` đọc raw bytes từ UART2
2. Tách frame theo delimiter `0x7E`
3. `cobs_decode()` → `crc16_ccitt()` verify → nếu hợp lệ → `memcpy` vào `cmd_packet_t`
4. `xQueueOverwrite(g_cmd_mailbox, &cmd)` — mailbox publish
5. `atomic_store(&g_last_cmd_us, esp_timer_get_time())` — watchdog timestamp

### 5.2 task_telemetry.c — Đóng gói + gửi telemetry lên Pi

Hiện tại chỉ ESP_LOGI. Cần thêm:
1. `rs_read(&g_state, &t)` — consistent snapshot (seqlock)
2. Đóng gói `telem_packet_t` từ snapshot
3. `cobs_encode()` + append `crc16_ccitt()` + delimiters `0x7E`
4. `hal_uart_link_write()` gửi lên Pi

### 5.3 hal_uart_link.c — Điền TODO

Implement `uart_driver_install()`, `uart_param_config()`, `uart_set_pin()` cho UART2 @ 921600 baud, GPIO TX=17, RX=16.

### 5.4 Packet structs (đã có trong protocol.h)

```c
typedef struct __attribute__((packed)) {
    float    omega_ref_l, omega_ref_r;  // rad/s
    uint8_t  mode;                       // 0=idle 1=run 2=estop
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
```

---

## 6. IPC Design (không đổi)

| Ranh giới | Primitive | Lý do |
|---|---|---|
| IMU INT → imu_spi_task | Task Notification | Nhanh nhất, zero-alloc |
| Telemetry cross-core (1W/NR) | Seqlock (robot_state.h) | Writer wait-free, control 200 Hz không block |
| Command Pi→ESP | Queue len=1 + xQueueOverwrite | Mailbox: chỉ giữ lệnh mới nhất |
| Watchdog | _Atomic uint64_t g_last_cmd_us | Control loop tự kiểm mỗi chu kỳ |
| System flags | Event Group | Nhiều cờ, nhiều task chờ |

**Task layout**:

| Task | Hz | Core | Prio | Phase 6 status |
|---|---|---|---|---|
| speed_ctrl_task | 200 | Core 1 | 15 | ✅ Done (PID) |
| imu_spi_task | INT | Core 0 | 10 | ✅ Done |
| task_comm | 100 | Core 0 | 8 | ⬜ Cần điền |
| task_telemetry | 50 | Core 0 | 5 | ⬜ Cần điền |

---

## 7. Pi-side Architecture (Phase 7+, chưa triển khai)

```
probot_pi/
├── bsp/
│   └── params.py              # mirror bsp_params.h
├── hal/
│   ├── serial_link.py         # UART + COBS + CRC16
│   └── robot_state.py         # latest telemetry
├── control/
│   ├── fuzzy_yaw.py           # Block 1: eψ, r → Δω_yaw (25 luật)
│   ├── fuzzy_traction.py      # Block 2: σ_err, |r_err| → λ (9 luật)
│   └── supervisor.py          # combine 2 blocks, modulate setpoints
├── services/
│   ├── kinematics.py          # inverse/forward diff-drive
│   └── expected.py            # σ_expected, r_ref từ command
├── app/
│   ├── main_loop.py           # 100 Hz: read→fuzzy→write
│   ├── dashboard.py           # Flask + WebSocket → browser
│   └── logger.py              # CSV log (backup)
└── main.py
```

---

## 8. Fuzzy Design — Tóm tắt đã chốt

**2 khối Mamdani song song, pre-PID setpoint injection:**

| Khối | Input | MFs | Output | Luật |
|---|---|---|---|---|
| Yaw stability | eψ [-30,30]°, r [-120,120]°/s | 5×5 | Δω_yaw | 25 |
| Traction | σ_err [0,1], \|r_err\| (3 MF) | 3×3 | λ [0.4,1.0] | 9 |

**Công thức injection**: `ω_ref_L' = λ·ωE_L − Δω_yaw`, `ω_ref_R' = λ·ωE_R + Δω_yaw` → gửi sang ESP qua UART.

**σ_err** = σ_measured − σ_expected (loại false positive khi cua chủ đích).

**Demo**: toggle fuzzy ON/OFF trên dashboard (mode field trong cmd_packet: 0=PID-only, 1=PID+Fuzzy).

---

## 9. ESP32 Pin Map (tham khảo nhanh)

| Chức năng | GPIO | Ghi chú |
|---|---|---|
| Motor L PWM | IO4 | MCPWM, 20 kHz |
| Motor L DIR | IO5 | |
| Motor R PWM | IO6 | MCPWM, 20 kHz |
| Motor R DIR | IO7 | |
| Encoder L A/B | IO9 / IO10 | PCNT unit 0, x4 |
| Encoder R A/B | IO11 / IO12 | PCNT unit 1, x4 |
| IMU SCK | IO14 | SPI2, ≤3 MHz |
| IMU MOSI | IO21 | |
| IMU MISO | IO47 | |
| IMU CS | IO38 | Pull-up 10K |
| IMU INT | IO18 | Active-low |
| IMU RST | IO8 | Pull-up 10K |
| **UART2 TX (→ Pi RX)** | **IO17** | **921600 baud** |
| **UART2 RX (← Pi TX)** | **IO16** | **Phase 6 target** |
| Status LED | IO48 | WS2812 RGB |

---

## 10. Config Key Values (bsp_params.h)

```
GEARBOX_RATIO         30.0f
ENC_COUNTS_PER_REV    1320
WHEEL_DIAMETER_M      0.065f
WHEEL_BASE_M          0.180f      // ĐÃ ĐO
PWM_FREQ_HZ           20000
CTRL_LOOP_HZ          200
COMM_LOOP_HZ          100
TELEM_LOOP_HZ         50
MOTOR_DEADZONE_DUTY   0.12f
CMD_TIMEOUT_US        200000
PI_UART_BAUD          921600
```

---

## 11. Nguyên tắc không đổi

- Dependency Rule: control (L4) KHÔNG phụ thuộc phần cứng. Vi phạm = build fail.
- Ownership: mỗi peripheral thuộc đúng 1 task.
- Không dùng Mutex cho sensor data cross-core (dùng seqlock).
- Không phát biểu quá mức về slip tuyệt đối → dùng σ_err.
- Pi 4 KHÔNG làm low-level controller. ESP32 KHÔNG chạy fuzzy.

---

## 12. Ngữ cảnh rút gọn cho Claude Code

> Project `probot`, ESP-IDF v5.x, ESP32-S3. Layered architecture 5 lớp đã build: bsp → robot_hal → services → control → robot_app. Components đã rename: hal→robot_hal, app→robot_app.
> Phase 1–5 DONE (hardware, encoder PCNT, motor MCPWM, IMU BNO085 SPI, PID speed closed-loop).
> Phase 6 IN PROGRESS: cần implement UART2 link ESP↔Pi (921600 baud, COBS+CRC16). Protocol layer đã sẵn (protocol.h/c). Cần điền: hal_uart_link.c (uart_driver_install), task_comm.c (parse incoming cmd → xQueueOverwrite mailbox), task_telemetry.c (rs_read seqlock → encode → uart_write). Packet structs: cmd_packet_t và telem_packet_t đã define trong protocol.h.
> Mục tiêu sau Phase 6: Pi 4 chạy Fuzzy supervisor (Python, 2-block Mamdani 34 luật, pre-PID setpoint injection) gửi ω_ref modulated xuống ESP qua UART. Dashboard Flask+WebSocket trên Pi, view từ laptop browser qua WiFi.
> Pin UART2: TX=IO17, RX=IO16. Baud 921600. Frame: [0x7E][COBS(payload)][CRC16-LE][0x7E].
