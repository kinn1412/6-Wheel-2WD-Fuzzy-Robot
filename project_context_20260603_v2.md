# CONTEXT – Probot: Robot Vi Sai Differential-Drive Project
## Phiên bản: sau build thành công firmware scaffold (cập nhật 03/06/2026)

---

## 1. Danh tính dự án

- **Tên đề tài**: Fuzzy-Based Integrated Traction and Stability Control for a Differential-Drive Robot
- **Bản chất**: Robot vi sai 2WD, chống trượt bánh + ổn định hướng bằng logic mờ tích hợp
- **Kiến trúc điều khiển**: phân tầng — ESP32-S3 (low-level PID speed loop) + Raspberry Pi 4 (fuzzy supervisor + dashboard)
- **Không bị khóa vào fuzzy**: có thể chuyển PID+feedforward, observer, MPC sau này
- **Tên project firmware**: `probot` (ESP-IDF v5.x, FreeRTOS, build trên Windows qua "ESP-IDF 5.x CMD")

---

## 2. Phần cứng đã chốt

| Hạng mục | Model cụ thể | Ghi chú |
|---|---|---|
| MCU low-level | ESP32-S3 DevKitC-1 (WROOM-1) | ESP-IDF v5.x, FreeRTOS, PCNT + MCPWM |
| Upper computer | Raspberry Pi 4 | Host fuzzy logic, dashboard, ROS 2 (Phase 6+) |
| Motor | 2× JGB37-520 12V 333rpm (gearbox 1:30, encoder 11 PPR) | Encoder Hall AB, 6 dây, decode x4 = 1320 cnt/rev |
| Motor driver | Cytron MDD10A | Sign-magnitude: PWM + DIR, 10A cont / 30A peak |
| IMU | BNO085 chế độ **SPI** | SPI2 qua GPIO matrix, ≤3 MHz, Game Rotation Vector + Calibrated Gyro |
| Bánh xe | 65mm đường kính | Wheel base chưa đo chính xác (ước ~180mm) |
| Pin | LiPo 3S 2Ah | 9.0–12.6V, cấp trực tiếp motor qua MDD10A + 2 buck |
| Buck ESP | XY-3606 (5.2V fixed, 5A) | Hàn dây trực tiếp vào pad PCB, bypass USB + barrel jack |
| Buck Pi | DFRobot 60W Adjustable | VẶN biến trở ra 5.1V trước khi cắm Pi. Cấp qua GPIO pin 2 |
| Perfboard | 8×12 cm | Interface board cho ESP32, đặt tầng trên bên trái |

---

## 3. Cơ khí – Layout 2 tầng

```
Tầng trên (upper tier):
  Bên trái:  Perfboard 8×12cm (ESP32-S3 cắm trên)
  Chính giữa: BNO085 IMU
  Bên phải:  Raspberry Pi 4

  Khoảng cách giữa 2 tầng: 10cm

Tầng dưới (lower tier):
  Mặt trên: Pin 3S + đế pin, Buck XY-3606 (ESP), Buck DFRobot (Pi), MDD10A driver
  Mặt dưới: 2× JGB37-520 motor bắt vào gá
```

---

## 4. Wiring đã chốt – Color code theo shop (KHÔNG phải generic)

### 4.1 Motor cable JGB37-520 – 6 dây (theo documentation shop thegioiic)

| Wire màu | Chức năng | Voltage | Nối vào |
|---|---|---|---|
| **Đỏ (Red)** | Motor + | 12V | MDD10A M1A (L) / M2A (R) |
| **Trắng (White)** | Motor − | 12V | MDD10A M1B (L) / M2B (R) |
| **Vàng (Yellow)** | Encoder signal Phase A | 3.3V logic | ESP IO9 (L) / IO11 (R) |
| **Xanh lá (Green)** | Encoder signal Phase B | 3.3V logic | ESP IO10 (L) / IO12 (R) |
| **Xanh dương (Blue)** | Encoder VCC (+) | 3.3–5V | ESP 3.3V pin |
| **Đen (Black)** | Encoder GND (−) | 0V | ESP GND pin |

⚠ CHÚ Ý: Wiring này KHÁC với nhiều source generic trên internet. Nguồn authoritative: shop bán motor.
⚠ Encoder VCC = 3.3V, TUYỆT ĐỐI KHÔNG cấp 12V vào dây xanh dương.
⚠ Phase B = XANH LÁ (Green), KHÔNG phải White. White = Motor − = 12V.

### 4.2 Motor cable đã được cắt và chia 3 cặp xoắn

Mỗi motor cable cắt cách thân motor 8–10cm, chia thành:
- **Cặp A (motor power)**: Đỏ + Trắng → xoắn → MDD10A screw terminal (cùng tầng dưới)
- **Cặp B (encoder power)**: Xanh dương + Đen → xoắn → lên tầng trên → perfboard 3.3V + GND
- **Cặp C (encoder signal)**: Vàng + Xanh lá → xoắn → lên tầng trên → perfboard GPIO

Cặp B + C bó chung thành 1 bundle đi lên tầng trên (Bundle B2 cho motor L, Bundle B3 cho motor R).
Cặp A đi riêng, KHÔNG chung bundle với B+C.

### 4.3 ESP32-S3 Pin Map (theo số chân trên board — cột No.)

| Pin No. | Name (silk) | Nhóm | Chức năng | Nối vào | Hướng | Ghi chú |
|---|---|---|---|---|---|---|
| 1 | GND | Power | GND chung | Star GND tại cực âm pin 3S | — | Rail GND perfboard |
| 2 | 3V3 | Power | LDO 3.3V output | Encoder VCC ×2, BNO085 VCC, BNO085 PS1 tie | OUT | Không cấp tải >100mA |
| 3 | EN | System | Chip Enable | NC — có pull-up onboard | — | KHÔNG dùng làm GPIO |
| 4 | IO4 | Motor | MCPWM Motor L PWM | MDD10A PWM1 (Header 1×5 pin 1) | OUT | 20kHz; 10K pull-down tại MDD10A |
| 5 | IO5 | Motor | GPIO Motor L DIR | MDD10A DIR1 (Header 1×5 pin 2) | OUT | 0=reverse 1=forward; 10K pull-down optional |
| 6 | IO6 | Motor | MCPWM Motor R PWM | MDD10A PWM2 (Header 1×5 pin 3) | OUT | 20kHz; 10K pull-down tại MDD10A |
| 7 | IO7 | Motor | GPIO Motor R DIR | MDD10A DIR2 (Header 1×5 pin 4) | OUT | 0=reverse 1=forward; 10K pull-down optional |
| 8 | IO15 | Spare | DỰ PHÒNG | NC | — | Chân trống chính |
| 9 | IO16 | Pi UART | UART2 RX | Pi 4 GPIO14 (TX) — cross | IN | Baud 921600, COBS+CRC16 |
| 10 | IO17 | Pi UART | UART2 TX | Pi 4 GPIO15 (RX) — cross | OUT | Header 1×3 pin 1 |
| 11 | IO18 | IMU SPI | BNO085 INT (H_INTN) | BNO085 chân INT | IN | Active-low data-ready; pull-up 10K optional |
| 12 | IO8 | IMU SPI | BNO085 RST | BNO085 chân RST | OUT | Active-low; PULL-UP 10K lên 3.3V bắt buộc |
| 13 | IO19 | Avoid | USB D− native | KHÔNG NỐI | — | USB flash/monitor |
| 14 | IO20 | Avoid | USB D+ native | KHÔNG NỐI | — | USB flash/monitor |
| 15 | IO3 | Avoid | Strapping pin | KHÔNG NỐI | — | JTAG source select |
| 16 | IO46 | Avoid | Strapping pin | KHÔNG NỐI | — | Boot mode |
| 17 | IO9 | Encoder | PCNT Encoder L Phase A | Motor L dây VÀNG (Yellow) | IN | Hall 3.3V; PCNT unit 0 |
| 18 | IO10 | Encoder | PCNT Encoder L Phase B | Motor L dây XANH LÁ (Green) | IN | ⚠ KHÔNG phải White! |
| 19 | IO11 | Encoder | PCNT Encoder R Phase A | Motor R dây VÀNG (Yellow) | IN | Hall 3.3V; PCNT unit 1 |
| 20 | IO12 | Encoder | PCNT Encoder R Phase B | Motor R dây XANH LÁ (Green) | IN | ⚠ KHÔNG phải White! |
| 21 | IO13 | Reserve | E-stop reserve | NC (Phase 6+ nút dừng) | IN | Future pull-up + nút nhấn |
| 22 | IO14 | IMU SPI | SPI2 SCK | BNO085 chân SCK | OUT | ≤3MHz; J2 liền kề ↓ |
| 23 | IO21 | IMU SPI | SPI2 MOSI | BNO085 chân DI (MOSI) | OUT | J2 liền kề ↓ |
| 24 | IO47 | IMU SPI | SPI2 MISO | BNO085 chân DO (MISO) | IN | J2 liền kề — gom 3 bus cùng cụm |
| 25 | IO48 | LED | WS2812 RGB LED | Onboard DevKitC | — | RMT driver; nếu không sáng thử IO38 |
| 26 | IO45 | Avoid | Strapping pin | KHÔNG NỐI | — | VDD_SPI voltage select |
| 27 | IO0 | Avoid | Strapping pin | KHÔNG NỐI | — | Boot mode select |
| 28 | IO35 | Avoid | PSRAM | KHÔNG NỐI | — | Octal SPI PSRAM bus |
| 29 | IO36 | Avoid | PSRAM | KHÔNG NỐI | — | Octal SPI PSRAM bus |
| 30 | IO37 | Avoid | PSRAM | KHÔNG NỐI | — | Octal SPI PSRAM bus |
| 31 | IO38 | IMU SPI | SPI2 CS | BNO085 chân CS | OUT | Active-low; PULL-UP 10K lên 3.3V bắt buộc |
| 32 | IO39 | JTAG | JTAG MTCK | NC (spare nếu không debug JTAG) | — | Dùng được nếu cần thêm GPIO |
| 33 | IO40 | JTAG | JTAG MTDO | NC (spare) | — | |
| 34 | IO41 | JTAG | JTAG MTDI | NC (spare) | — | |
| 35 | IO42 | JTAG | JTAG MTMS | NC (spare) | — | |
| 36 | RXD0 (IO44) | Console | UART0 Console RX | USB-UART bridge onboard | — | KHÔNG DÙNG |
| 37 | TXD0 (IO43) | Console | UART0 Console TX | USB-UART bridge onboard | — | KHÔNG DÙNG |
| 38 | IO2 | Spare | Spare GPIO | NC (available) | — | ADC1_CH1 |
| 39 | IO1 | Spare | Spare GPIO | NC (available) | — | ADC1_CH0 |
| 40 | GND | Power | GND | Star GND | — | |
| 41 | EPAD | Power | Exposed pad GND | GND | — | Thermal pad |

### 4.4 BNO085 SPI — Static Ties & Connector

| Chân BNO085 | Nối vào | Lý do |
|---|---|---|
| PS1 | **3.3V (HIGH)** | Chọn SPI mode |
| PS0 / WAKE | **3.3V (HIGH)** | Chọn SPI mode; không dùng sleep → tie cứng |
| BOOTN | **3.3V (HIGH)** | Normal mode (không bootloader) |
| VCC | 3.3V rail ESP32 | VDDIO = 3.3V |
| GND | GND rail | Common ground |

Connector IMU trên perfboard: **JST 8P hoặc 2× header**.
Thứ tự dây: VCC, GND, SCK, MOSI, MISO, CS, INT, RST.

### 4.5 Pull Resistors

| Vị trí | Giá trị | Loại | Mức độ |
|---|---|---|---|
| MDD10A PWM1 → GND | 10K | Pull-down | BẮT BUỘC — motor off khi ESP boot |
| MDD10A PWM2 → GND | 10K | Pull-down | BẮT BUỘC |
| MDD10A DIR1 → GND | 10K | Pull-down | Tùy chọn — DIR xác định lúc boot |
| MDD10A DIR2 → GND | 10K | Pull-down | Tùy chọn |
| BNO085 CS → 3.3V | 10K | Pull-up | BẮT BUỘC — CS idle HIGH khi ESP boot |
| BNO085 RST → 3.3V | 10K | Pull-up | BẮT BUỘC — IMU không bị reset khi ESP boot |

### 4.6 Power tree

```
Pin 3S (+12V) → Switch → Fuse 10A → Junction (cút chia dây, parallel)
  ├── OUT1 → MDD10A V_motor (+12V) → H-bridge → Motor red/white
  ├── OUT2 → Buck XY-3606 IN → OUT 5.2V → ESP32 5V pin (hàn trực tiếp pad)
  └── OUT3 → Buck DFRobot IN → OUT 5.1V (vặn biến trở) → Pi GPIO pin 2
Pin 3S (−) → Junction GND → Star GND point
```

### 4.7 Signal wiring

**ESP → MDD10A** (Bundle B4, 5 dây, inter-tier trên→dưới):
- IO4 → PWM1, IO5 → DIR1, IO6 → PWM2, IO7 → DIR2, GND → GND logic

**ESP ↔ BNO085 SPI** (same-tier tầng trên, JST 8P):
- 3.3V → VCC, GND → GND
- IO14 (SCK) → SCK, IO21 (MOSI) → DI, IO47 (MISO) → DO
- IO38 (CS) → CS, IO18 (INT) → H_INTN, IO8 (RST) → RST
- PS1 tie 3.3V, PS0/WAKE tie 3.3V, BOOTN tie 3.3V

**ESP ↔ Pi 4** (same-tier tầng trên, header 1×3, CHƯA NỐI):
- IO17 (TX) → Pi GPIO 15 (RX), IO16 (RX) → Pi GPIO 14 (TX), GND → GND
- Baud: 921600, COBS framing + CRC16

### 4.8 Inter-tier cable bundles

| Bundle | Nội dung | Chiều | Dây |
|---|---|---|---|
| B1 | Power UP (buck ESP out + buck Pi out) | Dưới→Trên | 4 dây AWG 20 |
| B2 | Encoder Motor L (blue+black+yellow+green) | Dưới→Trên | 4 dây AWG 24, xoắn |
| B3 | Encoder Motor R | Dưới→Trên | 4 dây AWG 24, xoắn |
| B4 | Signal ESP→MDD10A (PWM1,DIR1,PWM2,DIR2,GND) | Trên→Dưới | 5 dây AWG 24 |

---

## 5. Firmware Architecture (ĐÃ TRIỂN KHAI — build thành công)

### 5.1 Tổng quan

- **Project name**: `probot`
- **Framework**: ESP-IDF v5.x (KHÔNG dùng Arduino core)
- **Platform**: Windows, build qua "ESP-IDF 5.x CMD"
- **Kiến trúc**: Layered Architecture 5 lớp, component-based, dependency rule enforced bằng CMake REQUIRES

### 5.2 Layered Architecture — 5 lớp

```
┌──────────────────────────────────────────────────────────────────┐
│ L5  robot_app (APPLICATION / ORCHESTRATION)                      │
│     Tasks FreeRTOS, Robot FSM (phase flags), Watchdog            │
│     → biết "khi nào" gọi cái gì, KHÔNG biết "làm thế nào"       │
├──────────────────────────────────────────────────────────────────┤
│ L4  control (thuật toán THUẦN, KHÔNG phụ thuộc phần cứng)        │
│     controller_if (Strategy vtable) → PID | Fuzzy | MPC         │
│     → input: omega_ref, omega_meas, yaw, dt → output: duty      │
│     → HOST-TESTABLE: compile trên PC bằng Unity/Ceedling        │
├──────────────────────────────────────────────────────────────────┤
│ L3  services (MIDDLEWARE)                                        │
│     robot_state (seqlock), odometry, protocol (COBS+CRC), safety │
├──────────────────────────────────────────────────────────────────┤
│ L2  robot_hal (HARDWARE ABSTRACTION)                             │
│     hal_motor | hal_encoder | hal_imu | hal_uart_link            │
│     → wrap ESP-IDF drivers, expose đơn vị vật lý (rad/s, duty)  │
├──────────────────────────────────────────────────────────────────┤
│ L1  bsp (BOARD SUPPORT) + ESP-IDF Drivers                        │
│     bsp_pins.h = single source of truth cho pin map              │
│     bsp_params.h = hằng số vật lý (wheel diameter, gearbox...)   │
└──────────────────────────────────────────────────────────────────┘

Dependency Rule: mũi tên CHỈ đi xuống.
control (L4) KHÔNG REQUIRES robot_hal → #include "hal_*.h" trong control = BUILD FAIL.
CMake REQUIRES enforces layering at compile time.
```

**Tại sao thiết kế này**: đổi PID → MPC/Fuzzy = đổi 2 dòng binding trong `app_tasks.c` (`controller_pid_make` → `controller_mpc_make`). Toàn bộ task layer và HAL không đổi. Control layer unit-test trên PC host — không cần cắm ESP32.

### 5.3 Directory Structure (đã build thành công)

```
probot/
├── CMakeLists.txt
├── sdkconfig.defaults
├── partitions.csv
├── README.md
├── .gitignore
├── components/
│   ├── bsp/                            ← L1: Board Support (pin map + params)
│   │   ├── include/bsp_pins.h            (single source of truth cho GPIO)
│   │   ├── include/bsp_params.h          (hằng số vật lý + loop rates)
│   │   └── CMakeLists.txt
│   │
│   ├── robot_hal/                      ← L2: Hardware Abstraction (đổi tên từ hal)
│   │   ├── include/hal_motor.h           (MCPWM + DIR, duty ∈[-1,1])
│   │   ├── include/hal_encoder.h         (PCNT x4 decode → counts/rad)
│   │   ├── include/hal_imu.h             (BNO085 SPI wrapper, stub nếu chưa clone)
│   │   ├── include/hal_uart_link.h       (UART2 raw byte I/O)
│   │   ├── src/hal_motor.c
│   │   ├── src/hal_encoder.c
│   │   ├── src/hal_imu.c
│   │   ├── src/hal_uart_link.c
│   │   └── CMakeLists.txt               (REQUIRES bsp; PRIV_REQUIRES driver...)
│   │
│   ├── control/                        ← L4: Thuật toán THUẦN (host-testable)
│   │   ├── include/controller_if.h       (Strategy interface — vtable)
│   │   ├── include/controller_pid.h      (factory: pid → controller_if)
│   │   ├── include/pid.h                 (positional + FF + anti-windup)
│   │   ├── include/feedforward.h
│   │   ├── src/pid.c                     (IMPLEMENTED — đã test trên host)
│   │   ├── src/controller_pid.c          (adapter: bind pid vào controller_if)
│   │   └── CMakeLists.txt               (KHÔNG REQUIRES robot_hal — enforced)
│   │
│   ├── services/                       ← L3: Middleware
│   │   ├── include/robot_state.h         (seqlock — lock-free shared telemetry)
│   │   ├── include/odometry.h            (diff-drive forward/inverse kinematics)
│   │   ├── include/protocol.h            (COBS + CRC16 framing, cmd/telem structs)
│   │   ├── include/safety.h              (fault model, watchdog policy)
│   │   ├── src/odometry.c                (IMPLEMENTED)
│   │   ├── src/protocol.c                (IMPLEMENTED: crc16_ccitt, cobs_encode/decode)
│   │   ├── src/safety.c
│   │   └── CMakeLists.txt               (REQUIRES bsp; PRIV_REQUIRES esp_timer)
│   │
│   ├── robot_app/                      ← L5: Orchestration (đổi tên từ app)
│   │   ├── include/app_tasks.h           (app_tasks_start entry point)
│   │   ├── include/robot_fsm.h           (phase flags + state machine)
│   │   ├── src/app_internal.h            (shared globals: g_state, g_cmd_mailbox...)
│   │   ├── src/app_tasks.c               (init HAL, bind PID, spawn tasks on cores)
│   │   ├── src/robot_fsm.c
│   │   ├── src/task_speed_ctrl.c         (Core 1, prio 15, 200 Hz)
│   │   ├── src/task_imu.c               (Core 0, prio 10, INT-driven)
│   │   ├── src/task_comm.c              (Core 0, prio 8, 100 Hz)
│   │   ├── src/task_telemetry.c         (Core 0, prio 5, 50 Hz)
│   │   └── CMakeLists.txt               (REQUIRES robot_hal control services bsp)
│   │
│   └── esp32_BNO08x/                  ← third-party (chưa clone, build stub yaw=0)
│
├── main/
│   ├── CMakeLists.txt                   (REQUIRES robot_app)
│   └── main.c                           (CHỈ app_main → app_tasks_start)
│
└── test/
    └── test_control/test_pid.c          (host unit test stub)
```

### 5.4 FreeRTOS Task Layout

| Task | Freq | Core | Priority | Chức năng | IPC |
|---|---|---|---|---|---|
| speed_ctrl_task | 200 Hz | Core 1 (APP_CPU) | 15 | PID speed control 2 bánh, watchdog check | Đọc PCNT (atomic), đọc IMU (volatile float), peek mailbox, publish seqlock |
| imu_spi_task | INT-driven (~200 Hz) | Core 0 (PRO_CPU) | 10 | Đọc BNO085 qua SPI (Game RV + Gyro) | Wake bằng task notification từ ISR |
| comm_task | 100 Hz | Core 0 | 8 | UART ↔ Pi 4 (Phase 6+) | xQueueOverwrite cmd → mailbox; rs_read telemetry |
| telemetry_task | 50 Hz | Core 0 | 5 | Debug log qua USB monitor | rs_read seqlock snapshot |

**Core assignment rationale**: Core 1 chuyên cho hard-RT control loop (cách ly khỏi WiFi/BT stack chạy trên Core 0). Core 0 chạy I/O-bound tasks chịu được jitter.

### 5.5 IPC Design (Inter-Task Communication)

| Ranh giới dữ liệu | Primitive | Lý do |
|---|---|---|
| IMU INT → wake imu_spi_task | **Task Notification** (vTaskNotifyGiveFromISR) | Nhanh nhất (~45% nhanh hơn binary semaphore), không cấp phát object |
| yaw, yaw_rate, omega_meas (1 writer N reader, cross-core) | **Seqlock** (robot_state.h, stdatomic.h) | Writer wait-free — control loop 200 Hz không bao giờ block. Snapshot multi-field nhất quán |
| Command từ Pi (omega_ref_L/R, mode) | **Queue len=1 + xQueueOverwrite** (mailbox pattern) | Chỉ giữ lệnh mới nhất, không tích lũy stale |
| Watchdog (lost Pi > 200ms) | **_Atomic uint64_t g_last_cmd_us** | Control loop tự kiểm tra mỗi chu kỳ |
| Trạng thái hệ thống (IMU_READY, PI_LINK_UP, FAULT...) | **Event Group** | Nhiều task chờ/kiểm nhiều cờ cùng lúc |
| Encoder count | **PCNT hardware register** (atomic 32-bit read) | Không cần task encoder, không cần mutex — phần cứng tích lũy |

**Nguyên tắc chống race condition**: mỗi peripheral thuộc về đúng MỘT task (ownership). Không dùng Mutex cho sensor data cross-core (gây priority inversion + block control loop). Binary semaphore KHÔNG có priority inheritance → cấm dùng bảo vệ shared data.

### 5.6 Key Design Patterns đã áp dụng

| Pattern | Áp dụng tại | Mục đích |
|---|---|---|
| **Strategy** (vtable function pointers) | controller_if.h | Đổi PID↔MPC↔Fuzzy = đổi binding, task không đổi |
| **Opaque Pointer / Handle** | Mọi HAL + control dùng void *ctx | Encapsulation trong C, caller không truy cập internal |
| **Singleton kiểu C** (file-static + init guard) | Mọi HAL module (static bool s_init) | Tài nguyên duy nhất, idiomatic embedded C |
| **Publisher/Subscriber** (qua robot_state seqlock) | Sensor → comm/telemetry | Decouple producer–consumer, không coupling trực tiếp |
| **Factory** | controller_pid_make() | Trả controller_if_t đã bind, static allocation (no heap) |
| **State Machine** | robot_fsm.c | BOOT→IDLE→RUN→FAULT→ESTOP, thay cho rừng if cờ |

### 5.7 PID Implementation (ĐÃ HOÀN THÀNH + TEST)

- **Positional form** + feedforward + **derivative on measurement** (tránh derivative kick) + **back-calculation anti-windup**
- Output: **duty chuẩn hóa [-1.0, +1.0]** — control layer không biết MCPWM tick
- Đã compile + chạy trên host (gcc -Wall -Wextra, step response verified)
- **Placeholder gains** (cần tune Phase 5): kp=0.05, ki=0.20, kd=0.0, kff=0.02, kaw=1.0

### 5.8 IMU Driver

- Dùng component **`myles-parfeniuk/esp32_BNO08x`** (C++, esp-idf v5.x, sh2 HAL chính thức Hillcrest Labs, SPI only, multi-tasked INT-driven)
- KHÔNG dùng Adafruit Arduino lib (không ổn định với ESP32-S3)
- **Chưa clone** — hal_imu.c hiện chạy stub (yaw=0). Kích hoạt bằng:
  1. `cd components && git clone https://github.com/myles-parfeniuk/esp32_BNO08x.git`
  2. Uncomment `esp32_BNO08x` trong `robot_hal/CMakeLists.txt` PRIV_REQUIRES
  3. Define `HAL_IMU_USE_BNO08X` (qua Kconfig hoặc CMake `target_compile_definitions`)
  4. `idf.py fullclean && idf.py build`
- **Lưu ý**: thư viện có menuconfig riêng (esp_BNO08x → GPIO/SPI config). Giữ nhất quán với bsp_pins.h bằng cách truyền config struct trong hal_imu.c (single source of truth = BSP).
- **IMU reports**: SH2_GAME_ROTATION_VECTOR (quaternion no-mag → yaw), SH2_GYROSCOPE_CALIBRATED (yaw_rate native), SH2_LINEAR_ACCELERATION (tùy chọn)

### 5.9 Config key values (bsp_params.h)

```c
GEARBOX_RATIO         30.0f
ENC_COUNTS_PER_REV    1320        // 11 × 4 × 30
WHEEL_DIAMETER_M      0.065f
WHEEL_BASE_M          0.180f      // CẦN ĐO THỰC TẾ
PWM_FREQ_HZ           20000
CTRL_LOOP_HZ          200
MOTOR_DEADZONE_DUTY   0.12f       // normalized, cần đo lại Phase 4
CMD_TIMEOUT_US        200000      // mất Pi command > 200ms → stop
```

---

## 6. Communication protocol ESP ↔ Pi 4 (Phase 6+)

- UART GPIO (không USB CDC), baud 921600
- Framing: [0x7E][COBS(payload)][CRC16-LE][0x7E]
- CmdPacket (Pi→ESP, 100Hz): {omega_ref_L, omega_ref_R, mode, seq}
- TelemPacket (ESP→Pi, 100Hz): {omega_meas_L, omega_meas_R, yaw, yaw_rate, pwm_L, pwm_R, vbat, fault_flags, seq}
- COBS + CRC16-CCITT đã implement trong services/protocol.c

---

## 7. Perfboard 8×12cm layout

```
Top edge:    [5V IN terminal] [100µF] [100nF] [LED+R1k]
Left side:   [JST 4P Encoder L] [JST 4P Encoder R]
Center:      [ESP32-S3 female header socket 2×20]
Right side:  [JST 8P BNO085 SPI] [Header 1×3 Pi UART]
Bottom edge: [Header 1×5 MDD10A signal]
GND rail:    Dây đồng cứng chạy dọc chu vi, star point tại 5V IN terminal
3.3V rail:   Song song với GND, lấy từ pin 3.3V của ESP32
```

---

## 8. Chiến lược triển khai theo giai đoạn

| Giai đoạn | Mục tiêu | Trạng thái | Ghi chú |
|---|---|---|---|
| Phase 1 | Phần cứng + wiring | ✅ HOÀN TẤT | Wiring verified, perfboard done |
| Phase 2 | Firmware scaffold + build | ✅ HOÀN TẤT | Layered arch, PID impl, build OK |
| Phase 3 | Encoder bring-up | ⬜ TIẾP THEO | Điền TODO trong hal_encoder.c (PCNT), verify đếm |
| Phase 4 | Motor open-loop | ⬜ | Điền TODO trong hal_motor.c (MCPWM), đo deadzone |
| Phase 5 | Speed closed-loop | ⬜ | Tune PID gains, test step response |
| Phase 6 | IMU bring-up | ⬜ | Clone BNO08x, enable, calibrate, verify yaw |
| Phase 7 | Slip-aware supervisory | ⬜ | Tạo test tăng tốc, nền trơn, split-friction; logging |
| Phase 8 | Upper computer (Pi 4) | ⬜ | Dashboard, ROS 2, UART link |
| Phase 9 | Cơ khí nâng cấp | ⬜ | Chassis custom SolidWorks |
| Phase 10 | Phát triển nâng cao | ⬜ | Path tracking, MPC, vision, multi-robot |

---

## 9. Phương pháp đánh giá và bài test cần có

Tối thiểu phải log: tốc độ hai bánh, yaw-rate, heading, tín hiệu điều khiển, điện áp pin.
Bài test cơ bản:
- Tăng tốc thẳng trên nền bám đều
- Tăng tốc / thắng trên nền trơn
- Split-friction: hai bánh điều kiện bám khác nhau → ép yaw

---

## 10. Nguyên tắc không thay đổi

- Không chọn linh kiện theo kiểu combo bán sẵn; mọi quyết định dựa trên tải, tốc độ, nguồn, mở rộng.
- Không dùng Pi 4 làm bộ điều khiển thấp tầng.
- Không để phần mềm cứu cơ khí/nguồn quá tệ.
- Không phát biểu quá mức về ước lượng slip tuyệt đối khi chưa có ground truth.
- Không nhảy vào fuzzy/control nâng cao khi baseline encoder + IMU + speed control chưa ổn.
- **Dependency Rule**: control layer KHÔNG phụ thuộc phần cứng. Vi phạm = build fail.
- **Ownership**: mỗi peripheral thuộc đúng 1 task. Không dùng mutex cho sensor data cross-core.

---

## 11. Ngữ cảnh rút gọn để khởi tạo chat mới

> Tôi đang phát triển robot vi sai 2WD, project firmware tên `probot` (ESP-IDF v5.x, ESP32-S3).
> Kiến trúc phân tầng 5 lớp đã build thành công: bsp (L1, pin map + params) → robot_hal (L2, MCPWM/PCNT/SPI/UART abstraction) → services (L3, seqlock shared state, odometry, COBS+CRC protocol, safety) → control (L4, Strategy-pattern controller_if cho PID/MPC/Fuzzy, KHÔNG phụ thuộc phần cứng, host-testable) → robot_app (L5, FreeRTOS tasks pinned to cores, robot FSM).
> PID (positional + FF + derivative-on-measurement + back-calc anti-windup) đã implement + test trên host. IPC: seqlock cho telemetry cross-core, xQueueOverwrite mailbox cho command, task notification cho IMU INT. Control loop 200 Hz trên Core 1, I/O trên Core 0.
> Phần cứng: JGB37-520 encoder, MDD10A sign-magnitude driver, BNO085 SPI, LiPo 3S, buck 5V cho Pi 4.
> HAL hiện ở dạng structured stub (TODO markers cho PCNT/MCPWM/SPI). IMU driver (myles-parfeniuk/esp32_BNO08x) chưa clone — stub yaw=0.
> Bước tiếp: Phase 3 — encoder bring-up (điền hal_encoder.c dùng PCNT v5 driver).
> Tôi cần mọi đề xuất bám theo kiến trúc phân lớp đã chốt, đúng kỹ thuật, tính mở rộng, ưu tiên data-driven evaluation.

---

*Tài liệu này là bản tổng hợp kỹ thuật tính đến 03/06/2026. Khi triển khai, đối chiếu thông số module với datasheet tại thời điểm thực tế.*
