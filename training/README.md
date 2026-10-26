# training/ — reproduce and extend the training

MuJoCo simulator with the **firmware C++ in the loop**, system identification from real traces, a delay-aware linear teacher, PPO with corner sampling, checkpoint averaging, export to the ESP32 header, and the acceptance suite. CPU only. [中文](README.zh-CN.md) · [Pipeline explained](docs/PIPELINE.md) ([中文](docs/PIPELINE.zh-CN.md)) · [Safety](../SAFETY.md)

```
train2.py  evaluate2.py  export_actor2.py  verify_export2.py  viewer2.py     the pipeline (contract 6)
run.sh  setup.sh  requirements.txt                                          entry points / pinned dependencies
rider_sim/     env2.py (simulator), plant.py (identified wheel plant), firmware.py + native_policy.py (C++ in the loop), calibration.py (masses), ...
native/        C++ bridges that compile ../firmware/src into the simulator
models/        rider_measured.xml (MuJoCo model), measured_frf.json (the real robot's frequency response), robust_linear_gains_i.json (teacher), mass_audit/calibrated_distribution.json
models/cad/    the robot's CAD display meshes and the model drawn with them (CC BY-NC 4.0: non-commercial use only, optional)
tools/         sysid_frf.py, sim_frf_check.py, design_linear.py, loop_margin.py, average_checkpoints.py, probes (steps, leg wave, shoves, quantisation, falls and get-up, ...)
data/real_robot_traces/   numeric traces from the real robot that the identification used
artifacts/balance-v6/     the shipped model: policy_weights.h, actor.json, policy.zip, configs, evaluation reports
artifacts/balance-legs-v5/actor.json    the previous policy that rang on the robot (before/after comparison only)
tests/         24 unit tests (2 of them skip themselves without `models/cad/`)
docs/          PIPELINE.md / PIPELINE.zh-CN.md (start here), REBUILD.zh-CN.md (the engineering log, Chinese), MODEL.zh-CN.md (older model notes, Chinese)
```

`training/` and `firmware/` must stay siblings: the simulator compiles `../firmware/src` (IMU filter, leg controller, servo-bus decoder, policy controller) into a shared library at first use (`build/*.dylib`, needs `clang++`), so the simulated robot runs the same code as the real one.

## Setup

**Development and training currently run on macOS. Linux has also been verified to run; WSL2 is recommended for Windows.** Training uses CPU only on every platform and does not require CUDA.

| Platform | Environment and validation scope |
| --- | --- |
| macOS | Apple silicon, Python 3.11; the full published recipe has run here. The native C++ bridge needs `clang++`, available through Xcode Command Line Tools. |
| Linux | Verified in Debian 12 / ARM64 / Python 3.11 Docker: dependency installation, all 24 training tests, shipped-policy verification and evaluation, 32,768 PPO steps, resume to 40,960 total steps, export and new-policy verification, virtual-display viewing, and offscreen rendering. The full 4 million-step recipe has not been rerun on Linux. Requires `clang++`; graphics also require the appropriate OpenGL / GLFW / EGL or OSMesa system libraries. |
| Windows | Use Python 3.11 and `clang++` inside WSL2 Linux, then run the same commands below. WSL2 and native Windows have not yet been tested. Interactive viewing requires a working graphical display, such as WSLg. |

On Windows, clone the entire repository into WSL2's Linux filesystem, for example `~/xgorider2_rl`, and create the virtual environment and compile the native bridges there. Recreate `.venv` and the `build/` native cache when changing systems; do not reuse environments from macOS or Windows. Keep `training/` and `firmware/` as siblings. Training and numerical evaluation can run without a graphical window.

Prepare Python 3.11 (check `python3 --version`), virtual-environment support, and the system dependencies above, then run from `training/`:

```sh
sh setup.sh          # python3 -m venv .venv && pip install -r requirements.txt   (mujoco 3.16, torch 2.9 CPU, stable-baselines3 2.9, ...)
```

The pins are exact; other CPUs, systems, or dependency versions may train slightly differently. Everything below uses `.venv/bin/python`. You can also copy the [coding-agent prompt on the repository homepage](../README.md#copy-to-your-coding-agent-reproduce-and-train) to automate setup, verification, training, and export.

## Look at the shipped policy

```sh
sh run.sh view                    # interactive: W/↑ forward, S/↓ reverse, A/← left, D/→ right, X straight, Space stop, R reset, T random commands, mouse = camera
sh run.sh view --pushes --domain 1.0    # random shoves on a randomised robot
sh run.sh verify                  # exported weights, float32 C++ inference, scheduling, observation and integrals == the reference (max command error ≈ 1e-5 units)
sh run.sh evaluate --seeds 8      # acceptance suite on the native C++ policy (all scenarios; --scenarios stand drive corner ... to select)
sh run.sh margin                  # stability margins of artifacts/balance-v6/actor.json on the REAL measured frequency response
```

The window draws the robot from its CAD meshes (`models/cad/`): `rider_sim/env2.py` loads `models/cad/rider_measured_cad.xml` when it renders and `models/rider_measured.xml` otherwise (also when `models/cad/` is deleted). Both have the same bodies, masses, inertias, joints and (primitive) collision geometry, so training and evaluation never see the meshes; a unit test checks the equivalence. **`models/cad/` is licensed for non-commercial use only** (CC BY-NC 4.0, see `models/cad/LICENSE`); it is optional, and only the fall / get-up probes and two tests need it.

To reproduce the published table (`artifacts/balance-v6/RESULTS.md`) exactly, use its seed set and episode length:

```sh
.venv/bin/python evaluate2.py --policy native artifacts/balance-v6/policy_weights.h --seeds 32 --seed-start 61000 --seconds 20 \
    --scenarios stand stand_push drive stress_gain stress_delay stress_lag stress_sensor stress_weak hard_push noise5 corner
```

(the default seed set, 51000…, gives a few episodes of difference in the weak scenarios: corner 24/32, hard push 30/32, wheel-speed scale 0.4 30/32 — the weak areas are sensitive to the seeds.)

`evaluate2.py` prints, per scenario: survival, `hf_rms` (rms of the common wheel command above 3 Hz — the "shaking" you feel), command spread and total variation, pitch / pitch-rate standard deviation, velocity and yaw tracking error, final position error.

## Train it yourself

The recipe that produced the shipped model. Timing on an Apple M4 MacBook Air (10 cores, CPU only, evaluations included): the 4 M-step reproduction run took 8 min 24 s; the original run logged 925 s for 5 M steps.

```sh
.venv/bin/python train2.py --output runs/v6 --envs 5 --steps 4000000 \
    --eval-every 500000 --eval-seeds 8 --eval-workers 5 --corner-fraction 0.5 --seed 7171
```

Stages (`train2.py` prints each one):

1. collect ≈ 240 k samples from the linear teacher (`robust_linear_gains_i.json`) on randomised robots with exploration noise, **behaviour-clone** them into the 18-32-32-4 actor;
2. **critic warm-up**: actor frozen for 300 k steps while the value function learns;
3. **PPO** on 5 parallel randomised environments (one standing-only, four with random drive commands and pushes); half of all episodes are *corner* episodes in which every random parameter sits at an end of its range; reward = survival + upright + velocity / yaw / position tracking − command-rate, command-jerk and body-rate penalties;
4. every 500 k steps the checkpoint is scored by the **acceptance score** (survival + jitter + tracking on six scenarios); the behaviour-cloned start is scored first, so PPO is only worth keeping if it beats it.

Checkpoint selection, export, verification (the shipped model was made exactly like this):

```sh
.venv/bin/python tools/average_checkpoints.py runs/v6/step-{1500000,2000000,2500000,3000000,3500000}.zip --output runs/v6/avg.zip
.venv/bin/python export_actor2.py runs/v6/avg.zip --output artifacts/mine --name rider_balance_v6 --trained-steps 3500000
.venv/bin/python verify_export2.py artifacts/mine
.venv/bin/python evaluate2.py --policy native artifacts/mine/policy_weights.h --seeds 32 --seconds 20
.venv/bin/python tools/loop_margin.py artifacts/mine/actor.json --old artifacts/balance-legs-v5/actor.json
```

Averaging the 1.5 M–3.5 M checkpoints is deliberate: the score peaks around 3.5 M steps and **collapses later** (4 M: 66, 4.5 M: 43, 5 M: 29 in the original run), so the best single checkpoint is not the one to ship. `evaluate2.py` / `loop_margin.py` decide whether the average is accepted.

**Reproducibility:** running the command above on the maintainer's machine (Apple M4, PyTorch 2.9.1, NumPy 2.4.6) and the averaging/export steps reproduces the shipped `artifacts/balance-v6/policy_weights.h` byte for byte (SHA-256 `3929ac158858e75f86a6792ea803f2b00ddb5dc6ae4d47d9a9ca9a11dcb22576`); the evaluation scores of every checkpoint match the original run to ≤ 0.1. Other CPUs or library versions may drift in the last bits and therefore in the weights — compare the evaluation numbers, not the hash.

Put the result into the firmware: [../firmware/README.md](../firmware/README.md#use-your-own-weights).

## Continue training

```sh
.venv/bin/python train2.py --output runs/more --resume artifacts/balance-v6/policy.zip --steps 1000000 --reset-sigma 0.02 --resume-warmup
```

`--resume-warmup` re-warms the critic (use it when you changed the reward). Checkpoint `.zip` files are PyTorch/pickle: **only load files you trust.**

## Re-identify the plant from your own robot

The simulator is only as good as its wheel plant. `tools/sysid_frf.py` turns balance traces (the 55 Hz trace of `firmware/tools/capture_trace*.py`) into `models/measured_frf.json`; `tools/sim_frf_check.py` checks that the simulator reproduces it (magnitude within 9 %, phase within 9°); `tools/design_linear.py` redesigns the linear teacher on it:

```sh
.venv/bin/python tools/sysid_frf.py --logs data/real_robot_traces/*.jsonl      # or your own traces
.venv/bin/python tools/sim_frf_check.py
.venv/bin/python tools/design_linear.py --integral --speed-tau 0.12 --drift-weight 0.1 --zeta-min 0.25 --samples 12000 --output models/robust_linear_gains_i.json
```

Details and the evidence for why this matters: [docs/PIPELINE.md](docs/PIPELINE.md).

## What you can change

| Want to… | Look at |
| --- | --- |
| change the reward | `RiderEnv2.reward_cfg` and `step()` in `rider_sim/env2.py`; weights are `train2.py --rate-penalty --jerk-penalty --body-rate-penalty --still-v --still-x` |
| change what is randomised | `rider_sim/plant.py` (wheel plant), `rider_sim/calibration.py` (masses), `RiderEnv2.reset()` (noise, delays, friction, ground tilt, pushes) |
| train the legs | the two leg outputs are masked to zero in `train2.py` (`mask = [1, 1, 0, 0]`) and exported as zero rows in `export_actor2.py`; the firmware already applies them (`action_space = wheels_legs`) |
| change the observation | contract 6 is fixed in three places that must agree: `rider_sim/env2.py::_observation`, `firmware/src/policy/policy.cpp`, `export_actor2.py` (+ bump `kContractVersion`; the build refuses mismatched weights) |
| study falls and getting up | `tools/getup_probe.py`, `tools/fall_angle_probe.py` (the CAD meshes of body and legs in `models/cad/` as collision shells, via `tools/shell_model.py`); results in `docs/REBUILD.zh-CN.md` sections 10 and 12 |
| try ideas quickly | `tools/step_probe*.py` (small steps), `tools/fun_probes.py` (leg wave, shoves), `tools/standing_dead_zone.py`, `tools/quant_demo.py` (what int8/int4 weights would do) |

## Tests

```sh
.venv/bin/python -m unittest discover -s tests -p 'test_*.py'      # 24 tests (without models/cad/ two are skipped)
```

## Limits

The simulator has no static friction, gear backlash or soft ground; wheel impedance and bus delay are covered only by randomisation. The legs are not trained. The weakest evaluation scenario ("corner": all random parameters extreme) survives 23 / 32. Simulation scores are not hardware validation — see [../SAFETY.md](../SAFETY.md).
