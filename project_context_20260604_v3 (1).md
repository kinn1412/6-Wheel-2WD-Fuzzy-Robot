# CONTEXT – Probot: Robot Vi Sai Differential-Drive
## Phiên bản: Phase 7 DONE (cần verify) → chuẩn bị Phase 8 Fuzzy on Pi (cập nhật 04/06/2026)

---

## 1. Danh tính & mục tiêu

- **Tên project firmware**: `probot` (ESP-IDF v5.x, FreeRTOS, build trên "ESP-IDF 5.x CMD")
- **Đề tài**: Fuzzy-Based Integrated Traction and Stability Control for a Differential-Drive Robot
- **Kiến trúc điều khiển đã chốt**: ESP32-S3 chạy PID speed loop (low-level, 200 Hz) + **Raspberry Pi 4 chạy Fuzzy supervisor** (Python + scikit-fuzzy, ~100 Hz) + dashboard WiFi
- Fuzzy là 2 khối Mamdani song song (yaw stability 25 luật + traction 9 luật = 34 luật tổng), suy luận min-max, giải mờ centroid
- **Mục tiêu giai đoạn hiện tại**: verify UART link ESP↔Pi (Phase 7, đã implement bởi Claude Code), sau đó chuyển sang Phase 8: Fuzzy supervisor trên Pi (Python + scikit-fuzzy)

---

## 2. Tiến độ

| Phase | Mục tiêu | Trạng thái |
|---|---|---|
| Phase 1 | Phần cứng + wiring + perfboard | ✅ DONE |
| Phase 2 | Firmware scaffold + layered architecture + build | ✅ DONE |
| Phase 3 | Encoder bring-up (PCNT x4 decode) | ✅ DONE |
| Phase 4 | Motor open-loop (MCPWM + DIR) + deadzone measurement | ✅ DONE |
| Phase 5 | PID speed closed-loop + tuning | ✅ DONE |
| Phase 6 | IMU bring-up (BNO085 SPI, Game RV + Calibrated Gyro) | ✅ DONE (Claude Code) |
| Phase 7 | UART comm ESP↔Pi (COBS+CRC16, hal_uart_link + task_comm + task_telemetry) | ✅ DONE (Claude Code) — **CẦN VERIFY trước khi tiếp** |
| **Phase 8** | **Fuzzy supervisor trên Pi (Python + scikit-fuzzy)** | **⬜ TIẾP THEO** |
| Phase 9 | Online integration Pi→ESP + tuning fuzzy | ⬜ |
| Phase 10 | Dashboard WiFi + demo setup | ⬜ |
| Phase 11 | Test T1–T4, thu thập kết quả, báo cáo | ⬜ |

---

## 3. Kiến trúc hệ thống tổng thể

```
┌──────────────────────────────────────────────────────┐
│ Pi 4 — FUZZY SUPERVISOR (Python, ~100 Hz)            │
│  Lệnh (v, ω) → Inverse Kinematics → ωE_L, ωE_R     │
│  Tính σ_err, r_err (loại false positive khi cua)     │
│  Fuzzy 2-block → λ (traction), Δω_yaw (yaw bias)    │
│  ω_ref_L' = λ·ωE_L − Δω_yaw                         │
│  ω_ref_R' = λ·ωE_R + Δω_yaw                         │
│  Dashboard: Flask + WebSocket → laptop browser        │
└──────────────┬───────────────────────────────────────┘
               │ UART2 921600 baud, COBS+CRC16, 100 Hz
               │ Cmd  (Pi→ESP): {ω_ref_L', ω_ref_R', mode, seq}
               │ Telem (ESP→Pi): {ω_meas_L, ω_meas_R, yaw, r, duty_L, duty_R, vbat, faults, seq}
               ▼
┌──────────────────────────────────────────────────────┐
│ ESP32-S3 — LOW-LEVEL (FreeRTOS, 200 Hz)              │
│  PID speed loop từng bánh (Core 1, prio 15)          │
│  Encoder PCNT, IMU BNO085 SPI, MCPWM                │
│  Watchdog: mất cmd > 200ms → hal_motor_stop_all()    │
└──────────────────────────────────────────────────────┘
```

---

## 4. Firmware ESP32 — Cấu trúc (đã build, đã rename)

```
probot/
├── CMakeLists.txt
├── sdkconfig.defaults
├── components/
│   ├── bsp/                         ← L1: pin map + params
│   │   ├── include/bsp_pins.h         (GPIO assignments)
│   │   └── include/bsp_params.h       (WHEEL_RADIUS, GEARBOX, CTRL_HZ...)
│   │
│   ├── robot_hal/                   ← L2: hardware abstraction (renamed từ hal)
│   │   ├── include/hal_motor.h        (duty [-1,1] → MCPWM+DIR, deadzone)
│   │   ├── include/hal_encoder.h      (PCNT x4 → ω rad/s)
│   │   ├── include/hal_imu.h          (BNO085 SPI → yaw, yaw_rate — ✅ DONE)
│   │   ├── include/hal_uart_link.h    (UART2 raw byte I/O — ✅ DONE)
│   │   └── src/hal_*.c
│   │
│   ├── control/                     ← L4: thuật toán THUẦN (host-testable)
│   │   ├── include/controller_if.h    (Strategy vtable: PID/MPC/Fuzzy swap)
│   │   ├── include/controller_pid.h   (factory)
│   │   ├── include/pid.h              (positional + FF + anti-windup)
│   │   └── src/pid.c, controller_pid.c
│   │
│   ├── services/                    ← L3: middleware
│   │   ├── include/robot_state.h      (seqlock — lock-free shared telemetry)
│   │   ├── include/odometry.h         (diff-drive kinematics)
│   │   ├── include/protocol.h         (COBS + CRC16 + cmd/telem packet structs)
│   │   ├── include/safety.h           (fault flags, watchdog)
│   │   └── src/protocol.c             (cobs_encode/decode + crc16_ccitt — DONE)
│   │
│   ├── robot_app/                   ← L5: orchestration (renamed từ app)
│   │   ├── include/app_tasks.h
│   │   ├── include/robot_fsm.h        (phase flags + state machine)
│   │   ├── src/app_internal.h         (g_state, g_cmd_mailbox, g_last_cmd_us, g_ctrl_L/R)
│   │   ├── src/app_tasks.c            (init + spawn tasks on cores)
│   │   ├── src/task_speed_ctrl.c      (Core 1, 200 Hz — PID loop)
│   │   ├── src/task_imu.c             (Core 0, INT-driven)
│   │   ├── src/task_comm.c            (Core 0, 100 Hz — ✅ DONE Claude Code)
│   │   ├── src/task_telemetry.c       (Core 0, 50 Hz — ✅ DONE Claude Code)
│   │   └── src/robot_fsm.c
│   │
│   └── esp32_BNO08x/               ← third-party (đã clone)
│
├── main/
│   └── main.c                       (app_main → app_tasks_start)
└── test/
```

**Dependency rule bắt buộc**: `control` KHÔNG REQUIRES `robot_hal`. `#include "hal_*.h"` trong control = BUILD FAIL.

---

## 5. Trạng thái Phase 6–7 và việc tiếp theo

### 5.1 Phase 6 (IMU) — ✅ DONE bởi Claude Code

- BNO085 SPI đã init, enable SH2_GAME_ROTATION_VECTOR + SH2_GYROSCOPE_CALIBRATED.
- hal_imu.c đã implement đầy đủ (không còn stub).
- imu_spi_task hoạt động INT-driven trên Core 0.

### 5.2 Phase 7 (UART ESP↔Pi) — ✅ DONE bởi Claude Code, CẦN VERIFY

Đã implement nhưng **cần Claude Code check trước khi chuyển Phase 8**:

- **hal_uart_link.c**: `uart_driver_install()`, `uart_param_config()`, `uart_set_pin()` cho UART2 @ 921600 baud, GPIO TX=17, RX=16.
- **task_comm.c**: đọc UART → tách frame 0x7E → `cobs_decode()` → `crc16_ccitt()` verify → `xQueueOverwrite(g_cmd_mailbox)` → update `g_last_cmd_us`.
- **task_telemetry.c**: `rs_read()` seqlock → đóng gói `telem_packet_t` → `cobs_encode()` + CRC16 + delimiters → `hal_uart_link_write()`.

**Checklist verify:**
- [ ] Build clean (no warning)
- [ ] Loopback test: ESP TX→RX nối tắt, gửi telem → nhận lại → so CRC
- [ ] Struct alignment: `cmd_packet_t` và `telem_packet_t` packed đúng size
- [ ] Watchdog: mất frame > 200ms → FAULT_CMD_TIMEOUT → motor stop
- [ ] Không race condition: task_comm (Core 0) ghi mailbox, speed_ctrl (Core 1) peek — đã dùng xQueueOverwrite

### 5.3 Phase 8 (TIẾP THEO) — Fuzzy supervisor trên Pi

Sau khi verify Phase 7 xong:
1. Setup Pi: Python 3.11+, `pip install scikit-fuzzy numpy pyserial flask`
2. Implement `probot_pi/hal/serial_link.py` (UART + COBS + CRC16 — mirror ESP protocol)
3. Implement `probot_pi/control/fuzzy_yaw.py` + `fuzzy_traction.py` (skfuzzy.control API)
4. Implement `probot_pi/app/main_loop.py` (100 Hz: read telem → compute fuzzy → send cmd)
5. Test offline trước với CSV log từ ESP

### 5.4 Packet structs (reference — đã có trong protocol.h)

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

| Task | Hz | Core | Prio | Status |
|---|---|---|---|---|
| speed_ctrl_task | 200 | Core 1 | 15 | ✅ Done (PID) |
| imu_spi_task | INT | Core 0 | 10 | ✅ Done (Claude Code) |
| task_comm | 100 | Core 0 | 8 | ✅ Done (Claude Code) — cần verify |
| task_telemetry | 50 | Core 0 | 5 | ✅ Done (Claude Code) — cần verify |

---

## 7. Pi-side Architecture (Phase 7+, chưa triển khai)

**Stack đã chốt**: Python 3.11+, **scikit-fuzzy (skfuzzy)** cho Mamdani inference, numpy, pyserial (UART), Flask + WebSocket (dashboard), matplotlib (offline plot).

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

**2 khối Mamdani song song, pre-PID setpoint injection. Implement bằng scikit-fuzzy (`skfuzzy.control.Antecedent/Consequent/Rule/ControlSystem`):**

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
> Phase 1–7 DONE (hardware, encoder PCNT, motor MCPWM, PID speed closed-loop, IMU BNO085 SPI, UART2 ESP↔Pi COBS+CRC16). Phase 6 (IMU) và Phase 7 (UART comm) đã implement bởi Claude Code. Phase 7 cần verify trước khi tiếp (loopback test, struct alignment, watchdog behavior).
> Phase 8 TIẾP THEO: implement Fuzzy supervisor trên Pi 4 (Python + scikit-fuzzy, 2-block Mamdani 34 luật, pre-PID setpoint injection). Pi gửi ω_ref modulated xuống ESP qua UART, nhận telemetry ngược lại. Dashboard Flask+WebSocket, view từ laptop browser qua WiFi.
> Packet structs: cmd_packet_t {omega_ref_l, omega_ref_r, mode, seq} và telem_packet_t {omega_meas_l/r, yaw, yaw_rate, pwm_l/r, vbat, fault_flags, seq} đã define trong protocol.h.
> Pin UART2: TX=IO17, RX=IO16. Baud 921600. Frame: [0x7E][COBS(payload)][CRC16-LE][0x7E].
