# Session Summary — Probot Phase 3 & 4 (encoder + motor bring-up)

> Tổng kết phiên làm việc nối tiếp `project_context_20260603_v2.md`. Dùng để dán vào
> project_context hoặc khởi tạo chat mới. Stack: ESP-IDF v5.5.0, ESP32-S3, FreeRTOS.

---

## Trạng thái giai đoạn

| Phase | Mục tiêu | Trạng thái |
|---|---|---|
| 3 | Encoder bring-up (PCNT) | ✅ HOÀN TẤT + verified phần cứng |
| 4 | Motor open-loop (MCPWM) + đo deadzone | ✅ Code xong, build OK; dấu/chiều đã fix trong code, **chờ re-run xác nhận** |
| 5 | Speed closed-loop (tune PID) | ⬜ TIẾP THEO |

---

## Phase 3 — Encoder (PCNT v5) — DONE

**File:** `components/robot_hal/src/hal_encoder.c` (điền đầy stub).

- **x4 hardware decode**: mỗi motor 1 PCNT unit, 2 channel (chan_A `edge=A,level=B`; chan_B `edge=B,level=A`), edge/level action chuẩn quadrature.
- **Tràn 16-bit → 32-bit tự động**: `flags.accum_count = 1` + watch point tại `±32000`. Driver ESP-IDF tự cộng dồn `accum_value` trong watch-point ISR; `pcnt_unit_get_count()` đọc `hw + accum` **dưới spinlock** → an toàn cross-core (Core 1 đọc ↔ ISR), KHÔNG cần task/mutex.
- **Glitch filter 1µs** (cạnh Hall >100µs kể cả max RPM).
- **Sign per-side data-driven** qua `BSP_ENC_*_SIGN`.

**Kết quả test phần cứng:** quay tay 1 vòng → ~**1316** count (≈1320 CPR, sai số cơ khí OK); đúng chiều; đứng yên không trôi.

---

## Phase 4 — Motor open-loop (MCPWM) — code DONE

**File:** `components/robot_hal/src/hal_motor.c` (điền đầy stub).

- **MCPWM v5**: 1 timer 20kHz (clk PLL_F160M, resolution 10MHz, period 500 ticks) dùng chung 2 operator → 2 comparator/generator. PWM trên IO4/IO6; DIR là GPIO push-pull IO5/IO7.
- `set_duty`: clamp [-1,1] → dấu ra DIR (có cờ invert) → |duty| qua deadzone → compare ticks. Update compare on-TEZ (glitch-free).
- **Deadzone đổi 0.12 → 0.0** (mapping tuyến tính thật; placeholder cũ làm command nhảy bậc).

**Bring-up rig:** `components/robot_app/src/task_motor_bringup.c` (guard `APP_MOTOR_BRINGUP` trong `app_tasks.c`, mặc định 0). Sweep hysteresis nhánh B mỗi bánh/chiều:
- **UP** (tăng từ nghỉ) → *breakaway* (stiction).
- **DN** (giảm khi đang quay) → *running deadband*.
- Tự in dòng `>>` tổng kết + bảng `dcount`. Có `diag_hand_spin(10)` đầu task để tách lỗi software vs điện.

---

## Sự cố đã xử lý trong phiên

1. **Encoder đóng băng (omega=0) khi motor chạy**, dù quay tay (Phase 3) thì đếm tốt.
   - Nguyên nhân: **đấu lộn dây đọc encoder + nhiễu điện** khi motor chạy thật. Người dùng **chỉnh lại dây** → hết đóng băng, count tuyến tính đẹp trở lại.
   - (Ghi nhớ cho tương lai nếu tái phát dưới tải: motor chổi than JGB37-520 → nhiễu; chống bằng tụ 0.1µF ngang cực motor, pull-up ngoài 4.7k cho A/B, star-ground, tách bó dây.)

2. **R+ làm xe LÙI dù dây PWM/DIR đúng sơ đồ** → KHÔNG phải bug. Là do **motor phải lắp gương (mirror-mount)**: cùng `dir=1` nhưng đẩy xe ngược. Code đúng.

---

## Calibration đã chốt (trong `bsp_params.h`)

```c
#define BSP_ENC_L_SIGN          (-1)   /* trái: encoder ngược dấu so chuyển động */
#define BSP_ENC_R_SIGN          (+1)
#define BSP_MOTOR_L_DIR_INVERT   0
#define BSP_MOTOR_R_DIR_INVERT   1     /* phải mirror-mount → đảo DIR bằng SW */
#define BSP_MOTOR_DEADZONE_DUTY  0.0f  /* giữ 0; PID lo phần thấp */
```

Bất đối xứng L/R là bản chất của mirror-mount + đấu dây giống nhau. Mục tiêu mỗi bánh:
**+duty → tiến → +omega** (nếu sai dấu, PID Phase 5 thành positive feedback → runaway).

**⚠ CHỜ XÁC NHẬN:** re-run sweep phải thấy `+duty → +omega` cả 4 nhánh (L±, R±) và xe đúng chiều. Chưa verify phần cứng sau fix.

## Đặc tính đo được (độ lớn hợp lệ; seed Phase 5)

- **Running deadband ≈ 0.04** cả hai bánh (breakaway 0.04–0.08, hysteresis ~0.04).
- **Tuyến tính:** `ω ≈ 34·duty − 0.63` (vùng 0.06→0.30), tại 0.30 ≈ 9.6 rad/s.
- **Feedforward seed:** `kff ≈ 1/34 ≈ 0.029` duty/(rad/s) (placeholder hiện 0.02).
- **Đối xứng L/R** chênh <3% → tốt cho đi thẳng.

---

## Files đã thay đổi trong phiên

- `components/bsp/include/bsp_params.h` — thêm `BSP_ENC_*_SIGN`, `BSP_MOTOR_*_DIR_INVERT`; deadzone → 0.
- `components/robot_hal/src/hal_encoder.c` — PCNT v5 đầy đủ.
- `components/robot_hal/src/hal_motor.c` — MCPWM đầy đủ + DIR invert.
- `components/robot_app/src/task_motor_bringup.c` — rig sweep (MỚI).
- `components/robot_app/src/app_tasks.c` — flag `APP_MOTOR_BRINGUP` + nhánh spawn; **lưu ý: task_speed_ctrl/imu/comm đang bị comment từ Phase 3**.
- `components/robot_app/src/app_internal.h`, `components/robot_app/CMakeLists.txt` — đăng ký task bring-up.

## Build

`. "$env:IDF_PATH\export.ps1" | Out-Null; idf.py build` — lỗi `esp-rom-elfs` của export.ps1 là vô hại, build vẫn `Project build complete`.

---

## Vào Phase 5 (closed-loop) cần làm

1. Re-run sweep, xác nhận dấu (`+duty → +omega`, đúng chiều) cả 4 nhánh.
2. `APP_MOTOR_BRINGUP = 0`; **bỏ comment** `task_speed_ctrl`/`task_imu`/`task_comm` trong `app_tasks.c`; gỡ `diag_hand_spin`.
3. Cập nhật `PID_CFG.kff ≈ 0.029`; bắt đầu tune kp/ki theo step response (200 Hz loop, Core 1).
4. Đo `BSP_WHEEL_BASE_M` thực tế (đang để 0.180 placeholder) cho odometry.

---

## Ngữ cảnh rút gọn để khởi tạo chat mới

> Probot — robot vi sai 2WD, firmware `probot` (ESP-IDF v5.5, ESP32-S3), kiến trúc 5 lớp
> (bsp/robot_hal/services/control/robot_app). **Phase 3 (encoder PCNT x4 + accum overflow)
> và Phase 4 (motor MCPWM 20kHz open-loop) đã xong + build OK.** Encoder verified 1316/rev.
> Đã calibrate dấu: `ENC_L_SIGN=-1, ENC_R_SIGN=+1, MOTOR_R_DIR_INVERT=1` (motor phải mirror-mount),
> deadzone=0. Đặc tính: running deadband ~0.04, ω≈34·duty, kff≈0.029. Đang chờ re-run xác nhận
> dấu `+duty→+omega` cả 4 nhánh. Bước tiếp: **Phase 5 — speed closed-loop**, set kff≈0.029, tune
> PID step-response, bật lại các control task (đang comment), đo wheel base thực.