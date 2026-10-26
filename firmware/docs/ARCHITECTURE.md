# Firmware architecture

How the ESP32 application is organised, what runs when, what the policy sees and produces, and which protections surround it. Numbers marked *measured* come from the maintainer's USB traces on a running robot (`!status`: `imu_timing`, `control_*`, `policy.*`).

## Hardware seen by the firmware

| Part | Interface | Used for |
| --- | --- | --- |
| IMU (accelerometer ±2 g = 16384 counts/g, gyro ±2000 °/s) | I²C, read every ≥ 2 ms | attitude, rates, free-fall detection |
| Wheel hub servos, IDs 11 and 21 | half-duplex servo bus | wheel command out; speed, position, load back |
| Leg servos, IDs 12 and 22 (vertical slides, 45–81 mm extension, ≈ 63 mm standing) | same bus | leg position out, position back |
| 4 RGB LEDs, calibration switch, battery ADC | board | state display, calibration, low-battery stop |
| USB serial (115200) | text console + the binary protocol | commands, `status` JSON, trace export |
| BLE service `0xFFF0` (write `0xFFF2`, notify `0xFFF1`), extra UART | the stock binary protocol | phone app, telemetry |

## Tasks and timing

```
Arduino loop()   (main task, priority 1)      USB console, protocol parsers, BLE bytes -> queue, status JSON, trace export
rider_control    (core 1, priority 3, 16 KB stack)
   every loop pass:  servo-bus poll, command events (≤ 4 per pass), action engine
   every ≥ 2 ms:     sensor():  read IMU -> free-fall detectors -> IMU fusion -> odometry
   every ≥ 4 ms:     control(): guards -> start/stop state machine -> leg targets -> POLICY -> filters -> servo bus writes
   every 10 ms:      publish a snapshot (critical-section copy) for status/telemetry
BLE stack        callbacks only enqueue received bytes; no parsing, no flash, no bus access
```

*Measured while balancing:* the IMU is read every 2.0–6.0 ms (mean ≈ 3.3 ms; read + fusion ≤ 1.8 ms; no read failures), the control tick averages ≈ 5.9 ms, the policy runs on **every** control tick and takes ≈ 0.4–0.8 ms against a 2 ms deadline. A gap of more than 30 ms between control ticks is a fault (`control_gap_over_30ms`).

Memory of the reference build: RAM 112,468 of 327,680 bytes (34 %), flash 1,235,497 of 1,572,864 bytes (79 %). The policy needs 7,184 bytes of constant weights and a few hundred bytes of state.

## The policy contract (contract 6)

The network is a plain `Linear(18,32) → tanh → Linear(32,32) → tanh → Linear(32,4)` with fixed input normalisation (mean/std, clipped to ±5). Inputs, in order:

| # | Input | Unit | Note |
| --- | --- | --- | --- |
| 0, 1 | pitch, roll | rad | fused attitude |
| 2 | pitch rate | rad/s | gyro, filtered |
| 3, 4 | roll rate, yaw rate | rad/s | |
| 5, 6 | wheel speeds (left, right) | m/s of the servo's speed register | first-order low-pass, τ = 0.12 s |
| 7, 8 | leg encoder offsets | | **masked** (first-layer columns are zero) |
| 9, 10 | forward command, turn command | m/s, rad/s | from the phone, bounded |
| 11 | height command | units / 36 | |
| 12 | position error | m | odometry − reference |
| 13, 14 | previous wheel commands | | **masked** |
| 15, 16 | inference gap, control tick | s | **masked** |
| 17 | position-error integral | m·s, ±1 | kept by the firmware (a memoryless network cannot remove a centre-of-mass offset otherwise) |

Outputs: wheel commands for the two hub servos (scaled to ±1023 after subtracting a small yaw-integral correction, rate-limited to 20,000 units/s) and two leg-extension offsets (**fixed at zero in this release**; the legs are position-controlled and level the body). The masked inputs are still supplied by the firmware, but the exported first layer ignores them, so the network is mathematically independent of timing jitter, the previous command and the leg encoders.

Every input and every intermediate value is checked for finiteness; an invalid state, a non-finite number, an inference slower than 2 ms or a gap above 30 ms makes the controller return "stop" — there is **no fallback controller** in the RL-only build (the build checks that the old wheel-PID code is not linked).

A dead-zone preload (`policy/dead_zone_comp.h`) adds a small learned offset to the common wheel command so the robot does not "step" back and forth while standing, because the wheels do not move for small commands (a dead zone of roughly 20–45 command units). It learns only while the robot stands quietly, starts from a stored value (12 units) and can be inspected or switched off with `!dzcomp`.

## Start, stop and re-arm

* **Start**: ≥ 3 s since boot, IMU calibrated, flash calibration valid, battery valid, all four servos fresh, the calibration switch released, |pitch| < 8°, |roll| < 5°.
* **Stop** (`ActuatorCommandQueue::stop`): wheel commands to zero **and the leg servos released** (no torque). Every protective stop does this.
* **Re-arm**: after a driving-time protective stop (tilt, fall, policy, IMU, servo dropout, timing, invalid output) the robot re-selects the policy by itself once it is upright (|pitch| < 8°, |roll| < 5°), the wheels are still (< 30 mm/s) and every servo answers, for 250 control ticks (≈ 1–1.5 s). Manual stops, low battery and calibration stay latched until `!arm` / the phone's stand-up.

## Protections (all run in the control task)

| Guard | Rule | Source |
| --- | --- | --- |
| Tilt | \|pitch\| > 45° or \|roll\| > 60° → stop | `runtime_guard.cpp` |
| Lift | mean or half-difference of the filtered wheel speeds > 300 (register mm/s) for 6 ticks → hold wheel command at zero (policy history cleared) until upright, calm and still for 60 ticks (100 after a quick re-trip) | `lift_guard.h` |
| Free fall | raw total acceleration < 0.5 g for `freefall_ms` (60 ms, adjustable 20–200 ms, RAM only), or < 0.061 g for 6 ms ("deep" detector — the IMU validity gate would otherwise call such samples dead data) → stop, servos released before the impact; the fall ends after > 0.8 g for 40 ms | `fall_guard.h` |
| Fusion restart rule | a read that only shows ≈ 0 g does not restart the IMU fusion unless it lasted > 30 ms | `imuFusionRestart` |
| Actuators | any servo older than 100 ms (wheels) / 250 ms (legs) → stop | `runtime_guard.cpp` |
| Timing | > 30 ms between control ticks → stop | |
| Calibration | invalid leg zero points → stop | |
| Low battery | stop and latch | `board_support` |

Guard priority is fixed in `outputGuard()`: manual, calibration, low battery, invalid calibration, IMU, actuators, timing, **fall**, tilt, startup. The free-fall guard and the tilt stop complement each other: in a clean fall the acceleration is ≈ 0 and the attitude estimate is frozen (IMU samples under 0.061 g are not fused), so the free-fall guard acts; in a spinning fall the centripetal acceleration at the IMU (≈ 4 cm from the centre of mass in the model) lifts the reading above 0.5 g and the tilt stop acts. Simulation of table-edge falls and its limits are in `training/docs/REBUILD.zh-CN.md` §12.

## Flash layout

```
0x1000  bootloader        0x8000 partition table   0x9000 NVS (0x5000)     0xe000 OTA data (0x2000)
0x10000 app0 (0x180000)   0x190000 app1 (0x180000)
0x310000..0x32ffff        NOT a partition: leg zero points (0x310000), BLE name (0x320000), pitch trim (0x321000), as in the stock firmware
0x330000 SPIFFS (0xd0000)
```

The settings writer only touches those three sectors after checking that no partition overlaps them, rewrites the whole sector and reads it back. The flasher only writes the app slot selected by the OTA data and verifies it.

## Console, protocol and trace

* The binary protocol is `55 00 len type reg payload checksum 00 aa` (type 0 = write, 2 = read) on USB, the extra UART and BLE; unknown registers and malformed lengths are rejected, the parser has fixed buffers and a 100 ms inter-byte timeout (`tests/test_protocol_fuzz.cpp` feeds it random streams, mutated frames and every register/length combination under ASan + UBSan).
* The text console (`!…`) exists only on USB. `!stop` is handled out of band and wins over queued commands.
* A 1000-sample ring (≈ 55 Hz, ≈ 18 s) of pitch, rates, wheel commands/speeds, servo loads/positions, accelerometer and flag bits is kept in RAM while the policy drives; `!trace dump` / `!trace usb on` export it, `incident_snapshot.py` saves it with `status` after an incident.

## Tests

`tools/run_host_tests.py` builds and runs 16 C++ test programs and 9 Python test modules on the host (see [../README.md](../README.md)). The flasher tests replace every esptool call by a fake; the policy tests compare the generated C++ with a PyTorch reference to 1e-5 when PyTorch is installed; the guard tests replay synthetic acceleration patterns modelled on the recordings through the firmware's own detector code.
