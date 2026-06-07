Probot Pi: Fuzzy Supervisor 🤖Raspberry Pi-side fuzzy supervisor for Probot (Phase 8). The Pi runs a 2-block Mamdani supervisor (yaw-stability + traction) utilizing scikit-fuzzy, and streams modulated wheel-speed setpoints to the ESP32 over UART2 @ 921600 8N1 via the frame format [0x00][COBS(payload‖crc16_le)][0x00].
The ESP32 is strictly responsible for the Hardware Abstraction Layer (HAL) and the low-level PID speed loop.
⚠️ Architecture Rule: > The ESP32 does not run fuzzy logic; the Raspberry Pi does not handle low-level control.Each cycle, the Pi computes the modulated setpoints and streams them down. The ESP32 simply tracks them with its per-wheel PID controller.$$\omega'_{ref\_L} = \lambda \cdot \omega_{E\_L} - \Delta\omega_{yaw}$$$$\omega'_{ref\_R} = \lambda \cdot \omega_{E\_R} + \Delta\omega_{yaw}$$📂 Project LayoutThe Python package is nested one level deep (probot_pi/probot_pi/) to ensure both python -m probot_pi and clean absolute imports work seamlessly.Plaintextprobot_pi/                 ← Project root (install.sh, requirements.txt, .venv)
└── probot_pi/             ← Python package
    ├── bsp/params.py      # Mirror of bsp_params.h + protocol + fuzzy tuning
    ├── hal/
    │   ├── serial_link.py # pyserial + pure-Python COBS/CRC16 (matches ESP32)
    │   └── robot_state.py # Thread-safe latest telemetry + link health
    ├── services/
    │   ├── kinematics.py  # Diff-drive forward/inverse (mirrors odometry.c)
    │   └── expected.py    # r_ref + slip-error (σ_err) estimator
    ├── control/
    │   ├── fuzzy_yaw.py       # Block 1: e_ψ, r_err → Δω_yaw (25 rules)
    │   ├── fuzzy_traction.py  # Block 2: σ_err, |r_err| → λ (9 rules)
    │   └── supervisor.py      # Combines blocks, pre-PID setpoint injection
    └── app/
        ├── main_loop.py   # 100 Hz: read telem → fuzzy → send cmd
        ├── sim.py         # Offline diff-drive plant (no hardware needed)
        └── logger.py      # CSV log for offline analysis
⚙️ Installation (Raspberry Pi)Copy the probot_pi/ folder to your Raspberry Pi, then execute the following commands:Bashcd probot_pi
chmod +x install.sh

# 1. Standard Installation
./install.sh               # Installs apt deps, .venv, requirements, and configures dialout group

# 2. Hardware UART Setup (Required for 921600 baud rate)
./install.sh --setup-uart  # Frees /dev/serial0 (disables on-board BT) — Reboot required
Note: install.sh is completely idempotent. It backs up any modified boot files and strictly verifies all dependencies (like scikit-fuzzy, scipy) before declaring success. The first installation of scipy on aarch64 may take a few minutes.Why --setup-uart?To reliably hit the 921600 baud rate, the Pi must use the PL011 UART (ttyAMA0 → /dev/serial0), bypassing the core-clocked mini-UART. This flag configures enable_uart=1 + dtoverlay=disable-bt, strips the serial login console from cmdline.txt, and disables the getty/hciuart services. This permanently disables on-board Bluetooth and requires a reboot to take effect.🚀 UsageActivate the virtual environment before running any scripts:Bashsource .venv/bin/activate
1. Simulation ModeRun the full supervisor against an in-process, synthetic plant (with scripted right-wheel slip) to validate logic before touching hardware:Bashpython -m probot_pi --sim --v 0.3 --w 0.0 --log run.csv --duration 8
2. Live Hardware ExecutionRun against the ESP32 via UART:Bash# Full Fuzzy Supervisor
python -m probot_pi --port /dev/serial0 --baud 921600 --v 0.3 --w 0.0

# PID-only baseline (Fuzzy bypassed)
python -m probot_pi --no-fuzzy --port /dev/serial0 --baud 921600 --v 0.3 --w 0.0
3. Real-Time Fuzzy Execution via LUTCalculating skfuzzy.compute() on the fly takes ~25 ms, capping the system at ~20 Hz. To maintain the required 100 Hz loop, the supervisor pre-computes the fuzzy surfaces into Look-Up Tables (LUTs) cached at control/_lut_cache/*.npz, utilizing rapid bilinear interpolation (~µs) at runtime.Bash# Pre-build LUTs manually (otherwise built automatically on first run)
python -m probot_pi.control.fuzzy_lut

# Force exact skfuzzy calculation each tick (SLOW - for comparison only)
python -m probot_pi --no-lut
Note: The cache key includes tuning ranges. Any edits to bsp/params.py will automatically trigger a LUT rebuild. Monitor the over= counter in the terminal; it should remain at 0 to ensure no period overruns.📊 Web Dashboard (Phase 10)A self-contained web dashboard is available for control, live tuning, and running demo scenarios. It uses SSE and REST (no Socket.io, no external CDNs), meaning it works perfectly offline on the robot's local Wi-Fi.Bash# Hardware mode
python -m probot_pi --dashboard --port /dev/serial0

# Simulation mode
python -m probot_pi --dashboard --sim

# Hardware mode with CSV logging
python -m probot_pi --dashboard --port /dev/serial0 --log demo.csv
Once running, open http://<pi-ip>:8000 in your browser.Dashboard Features:Control: START, STOP, E-STOP, slew-limited $v$ and $\omega$ sliders, and a live FUZZY ON/OFF toggle.Demos: Pre-scripted scenarios (straight, step_turn, spin, figure8, slip_test) with repeatable $v/\omega$ profiles to easily compare PID vs. Fuzzy under exact conditions.Live Tuning: Adjust k_yaw, k_trac, slip_scale, and slip_expect on the fly. These external gains bypass the LUT and take effect instantly at 100 Hz.Telemetry Charts: Real-time visualization of wheel speeds, headings ($e_\psi$, $r_{err}$), fuzzy outputs ($\lambda$, $\Delta\omega_{yaw}$, $\sigma_{err}$), and communication link health.Demo tip: Run slip_test once with FUZZY OFF and once with FUZZY ON while logging (--log). Compare the heading and $\lambda$ traces side-by-side for your report.🔧 Hardware WiringCross the TX/RX lines and ensure a shared common ground. Both the Pi and ESP32 use 3.3V logic, so no level shifter is required.ESP32 (UART2)Raspberry Pi (GPIO Header)TX (IO17)RXD GPIO15 (Pin 10)RX (IO16)TXD GPIO14 (Pin 8)GNDGND📌 Developer Notes & Spec ReconciliationPlant Constants: Mirrored directly from the firmware's bsp_params.h which serves as the ground truth (WHEEL_BASE=0.290, COUNTS_PER_REV=1320, DEADZONE=0.0). Ignore older planning docs citing 0.180 / 0.12.FSM Mode: The mode byte strictly follows firmware FSM (0=idle, 1=run, 2=estop). Fuzzy ON/OFF is exclusively a Pi-side software flag. When disabled, the supervisor acts as a passthrough ($\lambda=1, \Delta\omega_{yaw}=0$).Slip Proxy ($\sigma$): Since there is no ground-truth vehicle-speed sensor, slip is estimated via the wheel-vs-IMU yaw-rate residual (see services/expected.py). Current rules are a baseline for Phase 9 tuning.
