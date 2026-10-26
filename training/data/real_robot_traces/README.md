# Real-robot traces (the data behind the wheel plant)

Numeric telemetry recorded from one XGO-Rider2 on 2026-10-21 while an *earlier* policy balanced it. That policy rang at 5–14 Hz, and the ringing is exactly what makes the identification possible: it is a narrow-band signal with coherence ≈ 1 between wheel command and body response (see [../../docs/PIPELINE.md](../../docs/PIPELINE.md), step 1).

* Source: the 55 Hz BLE trace of `firmware/tools/capture_trace.py` (one JSON object per sample, no identifiers).
* Used by: `tools/sysid_frf.py` → `models/measured_frf.json` (99 balanced chunks of ≈ 4 s, 8 frequency bins, 4.9–13.9 Hz).
* Re-run it: `.venv/bin/python tools/sysid_frf.py --logs data/real_robot_traces/*.jsonl --output /tmp/frf.json` and compare with the shipped file.

Fields of each line (integers, as the firmware sends them): `seq` · `pitch` (deg × 100) · `pitch_rate` (rad/s × 1000, firmware sign) · `roll` (deg × 100) · `yaw_rate` (deg/s × 100) · `cmd11`, `cmd21` (wheel commands, ±1023) · `sp11`, `sp21` (wheel speed register, "mm/s" register units) · `fsp11`, `fsp21` (the same, filtered) · `legA`, `legB` (leg servo positions, ticks) · `a0`, `a1` (policy outputs).

| File | What was running |
| --- | --- |
| `trace-run1.jsonl`, `trace-run3-heightfix.jsonl`, `ble-trace.jsonl` | plain standing / driving, phone control |
| `leveling-ab.jsonl`, `slew-ab.jsonl`, `smooth-ab.jsonl`, `yaw-slew-ab.jsonl` | A/B tests of the previous firmware's settings (roll levelling, command slew, smoothing, yaw slew); the ringing is present in all of them |

These traces describe the previous policy's closed loop, not v6. Standing recordings of v6 are not published.
