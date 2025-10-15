# Spider Robot on Zephyr

This repository contains the C/Zephyr port of the quadruped "SpiderBot" gait controller that was originally written in Rust. The firmware targets the ESP32 and drives twelve hobby servos through a PCA9685 PWM expander while coordinating inverse kinematics, gait sequencing, and Wi‑Fi command handling with Zephyr RTOS primitives.

## Architecture Overview
- **Hardware targets**: ESP32 board configured as `BOARD=esp32` and a PCA9685 PWM controller declared on `i2c0` through `app.overlay`.
- **Servo control**: `servos.c` wraps the Zephyr PWM API, maps legs/joints to PCA9685 channels, and converts desired angles into pulse widths.
- **Shared robot state**: `robot_state.c` materialises the global kinematic constants, mutex-protected position arrays, and helper routines that replaced the per-thread ownership model used in the Rust implementation.
- **Motion control threads**:
  - `motors_thread`: high-priority loop that interpolates `g_state.site_now` towards `site_expect`, runs inverse kinematics, and writes the resulting servo angles.
  - `gait_thread`: consumes commands, populates `site_expect` with precomputed trajectories, and blocks until motion completion using `wait_all_reach`.
  - `tcp_server_thread`: connects to Wi‑Fi using credentials supplied via Kconfig, accepts TCP clients on port 5000, parses simple text commands, and forwards them through a Zephyr message queue.
- **Command set**: Short mnemonics map to gait routines (`sit`, `stand`, `sf`, `sb`, `tl`, `tr`, `shake`, `wave`). Each command may be suffixed by an optional repeat count (`wave 3`).

Compared to the Rust version, this port relies on a single globally shared `robot_state_t` protected by `k_mutex`/`k_sem` so that the C threads can cooperate safely while staying close to the Zephyr scheduling model.

## Repository Layout
| Path | Purpose |
| --- | --- |
| `src/` | Application entry point, servo driver, gait logic, and RTOS threads. |
| `include/` | Public headers shared across modules (`robot_state`, servo API, gait commands). |
| `app.overlay` | Device tree overlay enabling the PCA9685 on the I²C bus. |
| `prj.conf` / `Kconfig*` | Zephyr configuration, Wi‑Fi credentials hooks, and logging/network options. |
| `tests/kinematics/` | Ztest suite that validates the inverse kinematics solver against reference values. |
| `DEVICE_TREE.md` | Notes collected while experimenting with Zephyr device tree bindings. |
| `telnet` | One-line helper showing how to send a sample command to the TCP server. |

## Getting Started

### Prerequisites
1. Install the Zephyr SDK, Python requirements, and `west` per the [Zephyr getting started guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).
2. Export the Zephyr environment before building: `source ~/zephyrproject/zephyr/zephyr-env.sh`.
3. Clone this repository inside your Zephyr workspace or update `CMakeLists.txt` if you use a different board.

### Configure Wi‑Fi credentials
Create a `config_secrets.conf` file next to `prj.conf` and add your network information:

```ini
CONFIG_MY_WIFI_SSID="YourAccessPoint"
CONFIG_MY_WIFI_PSK="SuperSecret"
```

These symbols are referenced by `prj.conf` via `#include "config_secrets.conf"` and consumed by the Wi‑Fi connection routine. Never commit real credentials to version control.

### Build and flash
```bash
west build -b esp32 .
west flash
```

The build sets `CONFIG_MAIN_STACK_SIZE=4096`, enables PWM/I²C, logging, floating-point printf support, and the Wi‑Fi/networking stack required by the TCP server. Adjust the board argument if you are using a different ESP32 variant.

### Run-time workflow
1. Power the ESP32 and ensure the PCA9685 is wired to `i2c0` at address `0x7f` as in `app.overlay`.
2. After boot, the firmware connects to the configured Wi‑Fi network and starts listening on TCP port 5000.
3. Send commands from a terminal, e.g.:
   ```bash
   printf "stand 1\n" | nc <robot-ip> 5000
   printf "sf 3\n" | nc <robot-ip> 5000
   ```
   The `telnet` helper shows a single-line example using the `stand` command.
4. The gait thread translates the command into coordinated leg trajectories while the motor thread performs the inverse kinematics and servo updates.

## Testing

A ztest suite validates the inverse kinematics calculations against known-good values from the original SpiderBot firmware. Run it with the native POSIX board for quick iteration:

```bash
west build -b native_posix tests/kinematics
west test -d build
```

The test initialises the shared robot state and asserts that the computed joint angles stay within 0.001° of the reference solution.

## Calibration and Extensions
- Use `center_all_servos()` while the horns are detached to align each joint to 90° before mounting.
- Update `robot_state.c` if you change link lengths or gait parameters; the runtime initialiser recomputes turn radii and other derived constants from those measurements.
- Consult `DEVICE_TREE.md` for lessons learned while working with Zephyr device tree bindings and PWM peripherals.

## Troubleshooting Tips
- Double-check that the PWM device appears as `pca9685` in the final device tree when `init_servos()` runs; the routine logs an error if the device is not ready.
- If Wi‑Fi fails to connect, enable additional networking logs in `prj.conf` (`CONFIG_NET_LOG`, `CONFIG_WIFI_LOG_LEVEL_ERR`) to surface driver errors.
- The gait scheduler assumes a deterministic update period (`UPDATE_PERIOD = 20 ms`). Monitor Zephyr logs for missed mutex locks or timing drifts when tuning gait speeds.

With this README you can reproduce the Zephyr-based SpiderBot firmware, extend its gait repertoire, and keep its behaviour aligned with the original Rust prototype.
