# The training pipeline, step by step

How a policy gets from a real robot trace to the ESP32, why each step exists, and the evidence behind it. Commands are in [../README.md](../README.md); the full engineering log (Chinese) is [REBUILD.zh-CN.md](REBUILD.zh-CN.md). [中文版](PIPELINE.zh-CN.md)

![pipeline](images/pipeline.png)

## 0. Why the pipeline looks like this: the first attempt failed on the robot

The first pipeline simulated the wheel drive as an assumed *speed servo*, initialised the network from a delay-free LQR, ran PPO, and accepted a policy if it did not fall. In simulation 80 of 80 runs passed. On the robot it **rang at 7–14 Hz** all the time. It was not a tuning problem; it was a process problem:

| Finding | Evidence |
| --- | --- |
| The real wheel behaves like a torque source plus ≈ 13 ms of delay and lag, not like a speed servo | measured command → pitch-rate phase −92° (4.8 Hz) … −153° (13.5 Hz); the old simulator had −47° … −103° (30–45° too little lag) |
| The simulated wheel-speed signal was ≈ 3× too large | measured wheel-speed / pitch-rate ≈ 0.83; a free-rolling wheel pair would give 2.6; the old simulator 3.0–4.3 |
| The old policy had almost no stability margin on the *real* response | its local gains on the measured response: sensitivity peak **22** (v5), **12.9** (v8); loop gain above 8 Hz 1.49 (> 1) |
| It fed its previous command back (a high-frequency amplifier), updated only every 12 ms (≈ 6 ms extra hold), and its yaw gain (130–144) sat next to the stability boundary of that loop (stable up to ≈ 120, self-excited at 250) | policy inputs 13/14, `kPeriodUs`, yaw gain analysis |
| Acceptance did not look at jitter | only falls and tracking errors were counted |
| The follow-up A/B tests on the robot (rate limit, smoothing, gain scaling) were symptom treatment | adding lag to a loop that is already marginal at high frequency makes it worse |

The lesson that shaped everything below: **a policy that survives in simulation says nothing about the robot unless the simulated plant is measured on the robot, and acceptance must include jitter and stability margins.**

## 1. Identify the plant from the robot's own ringing (`tools/sysid_frf.py`)

While the robot balances, it rings. The ringing is a narrow-band signal with coherence ≈ 1 between the wheel command and the body's response, so closed-loop data identify the plant at the ringing frequency without any special excitation. For each ≈ 4 s balanced chunk the tool fits `H(f*) = S_uy / S_uu` at the dominant command frequency (Welch cross-spectra) for

* the firmware pitch rate per unit of common wheel command, and
* the mean wheel speed (as read over the servo bus) per unit of common wheel command,

then takes medians in frequency bins → `models/measured_frf.json` (99 chunks, 8 bins, 4.9–13.9 Hz, from `data/real_robot_traces/`).

![frf](images/frf.png)

Result: the gain falls from 0.0148 to 0.0057 rad/s per command unit and the phase goes from −92° to −153° — an integrator (−90°) plus ≈ 13 ms of command latency and torque lag. The wheel speed seen on the bus is only ≈ 0.83× the pitch rate, which a free-rolling wheel pair cannot do; the explanation is that **the speed register reads ≈ 0.28× of the true rim speed**. A bench check with the robot pushed 0.9 m by hand later confirmed 0.275 (encoder distance 0.878 m vs 0.242 m integrated from the speed register).

## 2. A simulator the real robot cannot embarrass (`rider_sim/plant.py`, `env2.py`)

* **Model** (`models/rider_measured.xml`): free-floating chassis, 578 g measured total (battery and compute module included), tyre Ø 61.6 mm, track 86.6 mm, legs as two equivalent prismatic joints with the firmware's 45–81 mm kinematics. No balance spring or hidden stabiliser: a unit test checks that zero wheel action falls.
* **Wheel drive**: a torque source with command dead time (nominal 2 ms), first-order torque lag (nominal 7.2 ms), an extra 2.5 ms on the IMU path, and a speed sensor that reads 0.28× of the true speed.
* **Randomised each episode**: wheel damping and armature, dead time (≈ 1–7 ms), lag (−40 … +60 %), IMU delay, Coulomb friction, extra centre-of-mass offset (−4 … +14 mm), speed-sensor scale (0.28 ± 30 %), part masses and inertias inside the measured assembly mass, ground friction 0.55–1.15 and tilt, IMU and wheel-feedback noise, the control / feedback time slots (control tick 4, 6 or 8 ms), pushes (0.1–0.8 N for 80 ms every 1–3 s) and initial pitch/roll/heading.
* **The key trick**: after everything else is drawn, the wheel **torque constant and the sensor scale are solved so that this particular sampled robot still reproduces the measured frequency response**, then scattered by the run-to-run scatter seen in the data (−25 … +35 %). No sample can be easier to control than the real robot. `tools/sim_frf_check.py` verifies the calibrated simulator against the data: magnitude ratio 0.92–1.09, worst phase difference 9.2° (the old simulator: 45° and a 3× wheel signal).
* **The firmware C++ runs in the loop.** `native/*.cpp` compile `../firmware/src` — IMU fusion, leg position controller, servo-bus frame decoder (integer speed quantisation, wheel direction, encoder wrap) and the policy controller (observation, inference, rate limits, integrals) — into libraries that MuJoCo drives through ctypes. Nothing is rewritten in Python. Physical sensor values go through the mounting transform, quantisation and noise before the firmware's own byte decoder sees them. Simulated microseconds advance independently of the host's scheduling, so a busy laptop never looks like an MCU timeout.

## 3. A teacher: the robust linear core (`tools/design_linear.py`)

Static gains `u = kp·pitch + kd·pitch_rate + kv·wheel_speed + kx·position + ki·∫position` on the common wheel command, found by minimising a noise-driven LQ cost on the **delayed, identified** nominal plant **subject to stability on stress plants**: 2× torque constant, +10 ms latency, 2× lag, wheel sensor 0.25 × … 1 ×, 0.6× torque constant, with a minimum damping ratio. Random starts followed by Nelder–Mead. It initialises the network *and* serves as the reference for judging what the learned part adds. It is not the old delay-free LQR; that one is how the first attempt ended up with a loop gain above one at the ringing frequency.

## 4. Behaviour cloning → critic warm-up → PPO (`train2.py`)

* **Network**: `MlpPolicy`, actor 18 → 32 → 32 → 4 with tanh (the part that is exported), critic 64 → 64 (training only). Wheels are outputs 0–1; the leg outputs 2–3 are pinned to zero in this release.
* **Behaviour cloning**: ≈ 240 k teacher samples collected on randomised robots with exploration noise 0.01–0.08, 30 epochs (final MSE 3.4e-6). The cloned network *is* the teacher, evaluated first so PPO is only kept if it beats it.
* **Critic warm-up**: the actor is frozen for 300 k steps while the value function learns from on-policy data.
* **PPO**: 5 parallel environments (one standing-only, four with random drive commands and pushes), 512 steps × 5 per rollout, batch 2048, 5 epochs, γ 0.997, λ 0.95, clip 0.1, learning rate 4e-5, target KL 0.015, no entropy bonus, initial wheel exploration σ 0.03.
* **Corner sampling**: in half of the episodes every random plant parameter is put at an end of its range. These thin-margin cases are where a controller with little margin fails, so training must see them often.
* **Reward**: `1 + exp(-((pitch − equilibrium)/0.2)²) + exp(-(v_err/0.12)²) + 2·exp(-(yaw_err/0.08)²) + 1.2·exp(-(pos_err/0.08)²) − 0.25·min((pos_err/0.15)², 100)` minus penalties on the command (`0.05·|a|²`), its rate (`6·|Δa|²`) and second difference (`3·|Δ²a|²`), roll, and body angular rate; −50 on falling. The rate/jerk/body-rate penalties are what keeps the output from shaking.

## 5. Choose by acceptance score, then average (`evaluate2.py`, `tools/average_checkpoints.py`)

Every 500 k steps the checkpoint is run on six scenarios × 8 seeds (standing + pushes, driving, +12 ms delay, weak motors, corners, hard pushes) and scored

```
score = 100·survival − 0.8·median(hf_rms) − 0.3·median(pitch-rate σ) − 60·median(velocity error) − 10·median(final position error)
```

(`hf_rms` = rms of the common wheel command above 3 Hz, the "shake" you can feel.)

![training curve](images/training_curve.png)

The score peaks around 3.5 M steps and **collapses afterwards** (4 M: 66, 4.5 M: 43, 5 M: 29 in the original run), so the best single checkpoint is not what ships. The exported weights are the **average of the 1.5, 2.0, 2.5, 3.0 and 3.5 M-step checkpoints**, which was steadier than any single one in the acceptance suite.

## 6. Export and verify (`export_actor2.py`, `verify_export2.py`)

* Weights become a float32 C++ header (output-major, torch layout) with the fixed input normalisation, the action scale (1023), the yaw-integral profile (gain/limit 0.08) and metadata (name, model id, contract 6, sha256).
* The six inputs the network must ignore (leg encoders, previous commands, timing) are **zeroed in the first layer**, so the exported network is mathematically independent of them even though the firmware still supplies them; the two leg output rows are zero.
* Verification against the C++ that will run on the ESP32: random inputs, maximum action error 2.6e-7; 15,000 closed-loop samples, observation error ≤ 7.5e-6; the position integral, yaw integral and per-tick rate limiter agree tick by tick (`artifacts/balance-v6/native_verification.json`).

## 7. Accept or reject: survival, jitter, and the margin on the real response

![survival](images/survival.png)

Native C++ controller, 32 seeds × 20 s on the calibrated, randomised plant: standing 32/32, standing + pushes 32/32, random drive commands + pushes 32/32, 5× sensor noise 32/32, motor gain × 1.8 / +12 ms delay / 18 ms lag / weak motors 32/32 each, 2.5× pushes and large initial tilt 31/32; the weakest areas are "corners" (all random parameters extreme) 23/32 and a wheel-speed scale of 0.4, outside the training range, 25/32.

![margin](images/loop_margin.png)

The more telling check uses the **measured** frequency response (`tools/loop_margin.py`): the network's local gains are closed around the real data, no simulator involved. Sensitivity peak **1.83** for v6 against **22.1** for the previous policy; loop gain above 8 Hz **0.40** against 1.49. On the model side: nominal damping ratio 0.55, gain margin ×3.6 up / ×0.51 down, delay margin 58 ms.

**What RL contributes, honestly.** The cloned linear core is already stable. On the same 24 seeds it survives the corner scenario 11/24 times and the hard-push scenario 19/24; after PPO and weight averaging 19/24 and 22/24, with a smaller pitch-rate in random-command runs (12 vs 15 °/s) and slightly slower speed tracking (0.058 vs 0.048 m/s). PPO from scratch (no teacher) also learns to balance in this simulator, but converges to a **high-gain** solution (pitch 4200, wheel speed 10250, position 3540): sensitivity peak 2.5 on the real response, delay margin only 16 ms, command jitter 2–3× that of v6 — so it was not chosen. That is exactly how the first process went wrong: looking at simulated survival alone selects high-gain policies.

## 8. On the robot

`firmware/` runs the same policy controller; see [../../firmware/docs/ARCHITECTURE.md](../../firmware/docs/ARCHITECTURE.md). Maintainer's measurements, October 2026, three XGO-Rider2 units: the 7–14 Hz ringing is gone (wheel-command energy above 3 Hz 1–2 % of the total, rms 5–8 versus 150–260 before; pitch-rate σ 16–24 °/s versus 70–150 °/s); hands-off standing for 20 s keeps the wheel speed ≤ 7; phone control and pushes behave normally. The hardware found three things the simulator did not have, and each became a firmware feature: wheels in the air run away within 0.2 s (lift guard), a servo that stops answering when the first leg command arrives (automatic re-arm once every servo answers again), and a wheel dead zone of roughly 20–45 command units that makes a standing robot step back and forth (dead-zone preload, which learns only while standing quietly). Falls from a table led to the free-fall guard.

## 9. Limits

* The simulator has no static friction, gear backlash or soft ground; wheel impedance and bus delay are covered only by randomisation. The legs are not trained (outputs fixed at zero).
* Survival in the "corner" scenario is 72 %: where all parameters are extreme at once, the policy is the weakest.
* Simulation scores are not hardware validation. The hardware results above come from three units and are not a certification.
