# probot_pi

> Raspberry Pi–side **fuzzy supervisor** for the `probot` differential-drive platform.
> Two-block Mamdani controller (yaw-stability + traction) streaming modulated wheel-speed setpoints to an ESP32 over **UART @ 921600 8N1**, with COBS-framed CRC16 payloads.

![Python](https://img.shields.io/badge/python-3.9%2B-blue)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%204-c51a4a)
![Link](https://img.shields.io/badge/UART-921600%208N1-green)
![Loop](https://img.shields.io/badge/control%20loop-100%20Hz-orange)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

---

## Table of Contents

- [Architecture](#architecture)
- [Repository Layout](#repository-layout)
- [Installation](#installation)
- [Running](#running)
- [Real-Time Fuzzy Evaluation (LUT)](#real-time-fuzzy-evaluation-lut)
- [Dashboard](#dashboard)
- [Wiring](#wiring)
- [Protocol](#protocol)
- [Design Notes & Spec Reconciliation](#design-notes--spec-reconciliation)
- [Troubleshooting](#troubleshooting)

---

## Architecture

**Strict separation of concerns** — the Pi reasons about *what* the wheels should do; the ESP32 enforces *how fast* they actually spin.

```
┌────────────────────────────┐       UART2 @ 921600        ┌───────────────────────────┐
│        Raspberry Pi 4       │   COBS([payload‖CRC16_LE])  │           ESP32           │
│   (this repo, Python)       │  ◄────────────────────────► │       (firmware)          │
│                             │                              │                           │
│  ┌──────────────────────┐   │   telem: ω_meas, ψ, r, …    │  ┌─────────────────────┐  │
│  │  Fuzzy Supervisor    │   │                              │  │  Per-wheel PID      │  │
│  │   ├ yaw block (5×5)  │   │   cmd:   ω_ref_L', ω_ref_R'  │  │  Encoder / IMU HAL  │  │
│  │   └ traction (3×3)   │   │                              │  │  Motor driver       │  │
│  └──────────────────────┘   │                              │  └─────────────────────┘  │
└────────────────────────────┘                              └───────────────────────────┘
```

Each 10 ms tick, the Pi computes:

```
ω_ref_L' = λ · ωE_L − Δω_yaw
ω_ref_R' = λ · ωE_R + Δω_yaw
```

where **`Δω_yaw`** comes from the yaw-stability block (inputs: `e_ψ`, `r_err`) and **`λ ∈ [0, 1]`** comes from the traction block (inputs: `σ_err`, `|r_err|`). The ESP32 just tracks these references.

> **Architecture rule:** the ESP32 does not run fuzzy; the Pi does not run low-level control.

---

## Repository Layout

```
probot_pi/                       ← project root (install.sh, requirements.txt, .venv)
└── probot_pi/                   ← Python package (run with `python -m probot_pi`)
    ├── bsp/params.py            # mirror of bsp_params.h + protocol + fuzzy tuning
    ├── hal/
    │   ├── serial_link.py       # pyserial + pure-Python COBS/CRC16 (ESP-compatible)
    │   └── robot_state.py       # thread-safe latest telemetry + link health
    ├── services/
    │   ├── kinematics.py        # diff-drive forward/inverse (mirrors odometry.c)
    │   └── expected.py          # r_ref + slip-error (σ_err) estimator
    ├── control/
    │   ├── fuzzy_yaw.py         # Block 1: e_ψ, r_err → Δω_yaw   (5×5 = 25 rules)
    │   ├── fuzzy_traction.py    # Block 2: σ_err, |r_err| → λ    (3×3 = 9 rules)
    │   ├── fuzzy_lut.py         # pre-builds bilinear-interpolation LUTs
    │   └── supervisor.py        # combines blocks, injects pre-PID setpoints
    └── app/
        ├── main_loop.py         # 100 Hz: read telem → fuzzy → send cmd
        ├── sim.py               # offline diff-drive plant (no hardware)
        ├── dashboard.py         # SSE + REST web UI (Phase 10)
        └── logger.py            # CSV logger for offline analysis
```

The package is nested one level (`probot_pi/probot_pi/`) so both `python -m probot_pi` and clean absolute imports resolve correctly.

---

## Installation

### Requirements

- Raspberry Pi 4 (64-bit Raspberry Pi OS recommended)
- Python **3.9+**
- ESP32-side firmware running the matching protocol (see *probot_firmware* repo)

### One-shot install

```bash
git clone https://github.com/<your-user>/probot_pi.git
cd probot_pi
chmod +x install.sh
./install.sh                 # apt deps + .venv + requirements + dialout group
./install.sh --setup-uart    # ALSO frees /dev/serial0 (disables on-board BT) — needs reboot
```

`install.sh` is **idempotent**, backs up any boot file it edits, and verifies every dependency actually imports (`scikit-fuzzy`, `scipy`, …) before declaring success. The `scipy` aarch64 wheel is large — first install can take several minutes.

### Why `--setup-uart`

To reach 921600 baud reliably, the Pi must use the **PL011** UART (`ttyAMA0` → `/dev/serial0`), **not** the core-clocked mini-UART (whose effective baud drifts with CPU governor changes). The flag:

- Adds `enable_uart=1` and `dtoverlay=disable-bt` to `/boot/firmware/config.txt`
- Strips the serial login console from `cmdline.txt`
- Disables the `serial-getty@ttyAMA0` and `hciuart` services

> **Side effect:** on-board Bluetooth is disabled. A reboot is required.

---

## Running

Activate the venv, then choose a mode:

```bash
source .venv/bin/activate
```

### Offline simulation (no hardware)

```bash
python -m probot_pi --sim --v 0.3 --w 0.0 --log run.csv --duration 8
```

Runs the full supervisor against an in-process diff-drive plant with a scripted right-wheel slip event — useful for validating the fuzzy blocks before touching real hardware.

### Live, against the ESP32

```bash
python -m probot_pi --port /dev/serial0 --baud 921600 --v 0.3 --w 0.0
python -m probot_pi --no-fuzzy ...      # PID-only baseline (λ=1, Δω=0 pass-through)
```

### Common flags

| Flag | Effect |
|---|---|
| `--sim` | Use the in-process plant instead of the UART link |
| `--port <dev>` | Serial device (default `/dev/serial0`) |
| `--baud <n>` | Link baud rate (default 921600) |
| `--v`, `--w` | Commanded linear/angular velocity (m/s, rad/s) |
| `--no-fuzzy` | Disable supervisor (pass-through PID baseline) |
| `--no-lut` | Force exact `skfuzzy.compute()` each tick (slow, for comparison) |
| `--log <file.csv>` | CSV log for offline analysis |
| `--dashboard` | Launch the web UI on port 8000 |
| `--duration <s>` | Auto-stop after N seconds |

---

## Real-Time Fuzzy Evaluation (LUT)

**Problem.** `skfuzzy.control.ControlSystemSimulation.compute()` takes ~25 ms per call on a Pi 4. Running both blocks every tick caps the loop at ~20 Hz — unacceptable for a 100 Hz control loop.

**Solution.** Sample each rule-base surface onto a 2-D grid **once** at startup (using `skfuzzy`, ~1 minute), cache it to `control/_lut_cache/*.npz`, and use **bilinear interpolation** at runtime (~µs per evaluation). The fuzzy *design* — membership functions, rule table, defuzzification — is unchanged; only the evaluation strategy differs.

```bash
python -m probot_pi.control.fuzzy_lut    # pre-build LUTs (else built lazily on first run)
python -m probot_pi --no-lut ...         # disable LUT, use exact compute (slow)
```

The cache key includes all tuning ranges from `bsp/params.py`, so editing the tuning **auto-invalidates and rebuilds** the LUT.

> Watch the `over=` counter in the status line — it must stay ≈ 0 (no period overruns).

---

## Dashboard

A self-contained web dashboard for **control + live tuning + demo scenarios** — designed to drive a live report or demo from. Transport is **SSE + REST** (no socket.io, no CDN), so it works on the robot's own WiFi with **zero internet dependency**.

```bash
python -m probot_pi --dashboard --port /dev/serial0          # browse http://<pi-ip>:8000
python -m probot_pi --dashboard --sim                        # try with no hardware
python -m probot_pi --dashboard --port /dev/serial0 --log demo.csv
```

### Features

- **Control panel**: START / STOP / **E-STOP**, slew-limited `v` and `w` sliders, live FUZZY ON/OFF toggle.
- **Demo scenarios** (one-click): `straight`, `step_turn`, `spin`, `figure8`, `slip_test` — scripted v/w profiles so each run is repeatable and you can A/B compare FUZZY ON vs OFF under the *same* commanded motion. A progress bar tracks the phase.
- **Live tuning** (no LUT rebuild): `k_yaw`, `k_trac`, `slip_scale`, `slip_expect`. These are gains **outside** the LUT (rule tables and MF shapes are design-time); changes take effect instantly at 100 Hz.
- **Live charts**: wheel speed (meas vs ref, L/R), heading (`e_ψ`, `r_err`), fuzzy outputs (`λ`, `Δω_yaw`, `σ_err`), link health (rate, overruns, faults, rx/bad/drop).

### Suggested report demo

Run `straight` (or `slip_test`) once with FUZZY OFF, once with ON, `--log` both, then show heading and `λ` traces side by side.

> The built-in Werkzeug server is fine for 1–2 viewers on a local network. Do **not** expose to the public internet.

---

## Wiring

| ESP32 (UART2) | Pi 4 (40-pin header)       | Notes                |
|---|---|---|
| TX = IO17 | RXD `GPIO15` (pin 10) | Cross-connect TX↔RX |
| RX = IO16 | TXD `GPIO14` (pin 8)  |                      |
| GND       | GND (e.g. pin 6)      | Common ground is mandatory |

Both sides are **3.3 V logic** — no level shifter required.

---

## Protocol

- **Physical**: UART2, **921600 baud**, 8N1, no flow control.
- **Framing**: `[0x00] [ COBS( payload ‖ CRC16_LE ) ] [0x00]`
- **CRC**: CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`), little-endian appended to payload **before** COBS encoding.
- **Telemetry → Pi**: wheel `ω_L`, `ω_R`, heading `ψ`, yaw rate `r`, IMU accel, link sequence, `mode` byte, fault flags.
- **Command → ESP32**: `ω_ref_L'`, `ω_ref_R'`, `mode`.
- **`mode` byte** (firmware FSM): `0 = idle`, `1 = run`, `2 = estop`.

> Fuzzy ON/OFF is a **Pi-side** flag (`--no-fuzzy` or dashboard toggle), **not** the `mode` byte. When off, the supervisor passes through with `λ=1, Δω=0`.

---

## Design Notes & Spec Reconciliation

- **Plant constants** are mirrored from the **firmware** `bsp_params.h` — the single source of truth:
  - `WHEEL_BASE = 0.290 m`
  - `COUNTS_PER_REV = 1320`
  - `DEADZONE = 0.0`
  - (The planning doc's `0.180 / 0.12 / 0x7E` values are stale and superseded.)
- **Slip estimation** has no ground-truth vehicle-speed sensor on this platform, so `σ_err` is a **proxy** derived from the wheel-vs-IMU yaw-rate residual (see `services/expected.py`). Rule tables and `slip_scale` are an initial design — empirical tuning in Phase 9.
- **Real-time budget**: 100 Hz loop → 10 ms period. Measured breakdown on Pi 4 with LUT enabled:
  - Telemetry parse: < 0.2 ms
  - Fuzzy (both blocks via LUT): < 0.1 ms
  - Command encode + send: < 0.3 ms
  - Slack: ≈ 9 ms — plenty of margin.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `Permission denied: /dev/serial0` | User not in `dialout` group | `sudo usermod -aG dialout $USER`, log out/in |
| Garbled bytes at 921600 | mini-UART instead of PL011 | Re-run `./install.sh --setup-uart` and reboot |
| `over=` counter climbing | LUT not loaded, or CPU throttling | Pre-build LUT, check `vcgencmd get_throttled` |
| `bad=` (CRC failures) rising | Loose wiring or shared GND missing | Re-seat jumpers, verify common ground |
| `scipy` install fails | Old pip on aarch64 | `pip install --upgrade pip` then re-run install |
| Dashboard unreachable | Firewall, or wrong IP | `ip addr` on Pi; open port 8000; same WiFi |

---
## Acknowledgments

- `scikit-fuzzy` for the Mamdani toolkit.
- ESP32 firmware sibling repo: *probot_firmware*.
