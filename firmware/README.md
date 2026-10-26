# firmware/ — build, test, flash, run

ESP32 application for the XGO-Rider2 main board. The balance controller is the trained 18-32-32-4 policy in `src/policy/trained_weights.h` (model `rider_balance_v6`); everything else (IMU fusion, leg position control, servo bus, BLE/USB protocol, guards) is ordinary C++ around it. [中文](README.zh-CN.md) · [Architecture](docs/ARCHITECTURE.md) · [Safety](../SAFETY.md)

```
src/                 firmware (C++17, Arduino-ESP32 on PlatformIO)
  policy/            float32 actor: policy.cpp/.h, dead_zone_comp.h, trained_weights.h (the weights)
  fall_guard.h  lift_guard.h  runtime_guard.*  led_status.h     protections
  actuator_bus.*  board_imu.*  board_support.*  ble_transport.*  command_protocol.*  hardware and protocol
  control/           IMU fusion, leg kinematics and the smoothing/PID helpers
  main.cpp           tasks, console, status JSON
tests/               host tests: 16 C++ programs + 9 Python test modules (no hardware, no serial port)
tools/               flasher, USB console, trace capture and reports, exporter
docs/                ARCHITECTURE.md, RL_POLICY.zh-CN.md (deep dive, Chinese), release-notes/ (hardware test notes, Chinese)
platformio.ini  partitions.csv  partitions.bin  build.sh  requirements.txt
```

## Build

Needs Python 3 and a network connection the first time (PlatformIO downloads the pinned toolchain: `espressif32@6.9.0`, Arduino-ESP32 2.0.17, Xtensa GCC 8.4).

```sh
sh build.sh                 # creates .venv, installs requirements.txt, builds env rider2-rl-only, writes dist/
```

Result: `dist/firmware.bin` (≈ 1.24 MB; the app slot is 1.5 MB) and `dist/manifest.json` (SHA-256, image checks, linked-model metadata, build versions). `dist/firmware.elf` and `.map` are for debugging and contain build-directory strings; publish only `firmware.bin` and `manifest.json`.

The build is **reproducible bit for bit**: the same sources built in two different directories give the same `firmware.bin` (the compiler is told to rewrite the project and package paths). Build from a real path, not through a symlink. Reference: sources of this repository state → `sha256 4a85fc892c9eeea0bfe1d131610a366b4a297489158675fc9f56f713f532863b`, 1,241,600 bytes.

The exporter refuses to write a manifest unless the image checksum and the appended SHA-256 are valid, the image fits the stock 0x180000-byte slot, the linked model matches the weights header, and none of the old wheel-PID symbols is linked (`rider2-rl-only` has no fallback controller).

## Test

```sh
python3 tools/run_host_tests.py     # needs a C++17 compiler (clang++ or c++), numpy and scipy
```

25 groups: policy inference and its limits, guards (free fall, lift, runtime/tilt), the servo-bus codec and transport, the BLE/USB command parser and a seeded fuzz test of it, flash-settings writer (stubbed flash), the LED fault mapping, the weights exporter, the flasher (every esptool call is faked), the trace tools and reports. C++ is compiled with `-Wall -Wextra -Werror`; the policy, the leg controller and the protocol fuzz test run under AddressSanitizer + UBSan. Nothing opens a serial port.

## Flash

Read [../SAFETY.md](../SAFETY.md) and **back up the stock flash first**. The flasher needs `esptool.py` (PlatformIO installs one: `~/.platformio/packages/tool-esptoolpy/esptool.py`).

```sh
# 1. offline check of dist/ (no port is opened)
python3 tools/flash_app.py --esptool ~/.platformio/packages/tool-esptoolpy/esptool.py --plan
# 2. connect the robot's USB download port: inspect the device, still no write
python3 tools/flash_app.py --esptool ~/.platformio/packages/tool-esptoolpy/esptool.py --port /dev/cu.usbserial-XXXX
# 3. write the app slot that the bootloader currently selects, then verify the digest (≈ 75 s at 115200 baud)
python3 tools/flash_app.py --esptool ~/.platformio/packages/tool-esptoolpy/esptool.py --port /dev/cu.usbserial-XXXX --flash
```

It refuses to continue when the partition table differs from the stock layout, when the OTA metadata is not CRC-valid or not in a settled state, or when the manifest/image checks fail. If the board stays in download mode after an error, power-cycle it. Open the port at 115200 baud afterwards.

## Run

After the flash, power the robot, stand it up and hold it still: it starts balancing by itself (conditions in [../SAFETY.md](../SAFETY.md)). The stock phone app drives it over BLE (service `0xFFF0`). Phone commands are bounded for the learned policy: forward ≤ 165 mm/s, turn ≤ 36 °/s.

### USB console (115200 baud, one command per line, attaching does not reset the board)

```sh
python3 tools/rider_cli.py --port /dev/cu.usbserial-XXXX status     # JSON: state, stop reason, sensors, servos, policy, guards, timing
python3 tools/rider_cli.py --port /dev/cu.usbserial-XXXX stop       # wheels to zero, legs released, latched
python3 tools/rider_cli.py --port /dev/cu.usbserial-XXXX arm        # clear the latch; the robot restarts when upright and still
```

Raw console lines start with `!` (`!status`, `!stop`, `!arm`, `!neutral`, `!move F T`, `!height B`, `!cal begin`, `!cal save`, `!name SUFFIX`, `!trace dump`, `!trace usb on`, `!liftguard on|off|status`, `!fallguard on|off|status|clear|ms N`, `!dzcomp …`). `!stop` always wins over everything.

Stop reasons in `status` / `last_stop_reason`: `tilt`, `free_fall`, `policy_fault`, `imu_missing_stale_or_nonfinite`, `actuator_feedback_missing_or_stale`, `control_gap_over_30ms`, `invalid_control_output`, `invalid_flash_calibration`, `low_battery`, `calibrating`, `manual_stop`, `startup`. LED while stopped: tilt solid red, free fall fast red flash, other faults slow red flash, requested stop amber.

### Recording and reports

| Tool | Does |
| --- | --- |
| `capture_trace_usb.py` | 55 Hz trace over USB (pitch, rates, wheel commands/speeds, loads, accelerometer, flags) with spoken test protocols (`--protocol drop`, `release`, `lift`, …) |
| `incident_snapshot.py` | keeps `status` and the last ≈ 18 s of the trace (held in RAM) right after a fall or a stop |
| `drop_report.py`, `replay_fall_guard.py` | analyse a free-fall test / replay recordings through the firmware's own detector |
| `standing_report.py`, `dzcomp_ab_test.py`, `start_cycle_test.py` | does it stand still, dead-zone preload A/B, reboot-and-start cycles |
| `calibrate_wheel_speed.py` | bench check of the wheel-speed scale the simulator assumes (0.28 of true speed) |
| `capture_trace.py`, `hardware_ble_test.py` | BLE trace capture and read-only BLE round trips (need `bleak`) |

## Use your own weights

```sh
cp ../training/artifacts/<your-run>/policy_weights.h          src/policy/trained_weights.h
cp ../training/artifacts/<your-run>/policy_weights.manifest.json src/policy/trained_weights.manifest.json
python3 tools/run_host_tests.py && sh build.sh
```

The header must carry `contract 6` and the `wheels_legs` action space; the build and the exporter check both, and `static_assert` stops a mismatch. A new observation contract changes `policy.cpp` and `training/rider_sim/env2.py` together — see [../training/README.md](../training/README.md).

## Docs

* [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — tasks, timing, the policy contract, the protections.
* [docs/RL_POLICY.zh-CN.md](docs/RL_POLICY.zh-CN.md) — policy deployment details, guards and their evidence (Chinese).
* [docs/release-notes/](docs/release-notes/) — notes of the hardware test sessions (Chinese).
