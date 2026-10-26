# Notices / 声明

## Release and trademarks

This repository is an official open-source release by the manufacturer of the XGO-Rider2 robot, licensed under the Apache License, Version 2.0 (see `LICENSE`), **except** `training/models/cad/`, the robot's CAD display files, which are licensed for non-commercial use only (CC BY-NC 4.0, see `training/models/cad/LICENSE`). "XGO" and "XGO-Rider2" are the manufacturer's product names; neither license grants any right to use them.

本仓库是 XGO-Rider2 厂家官方发布的开源项目，采用 Apache License 2.0（见 `LICENSE`），**例外**：`training/models/cad/`（机器人的 CAD 显示文件）仅限非商业使用（CC BY-NC 4.0，见 `training/models/cad/LICENSE`）。“XGO”“XGO-Rider2”是厂家的产品名称，两种许可证都不授予使用这些名称的权利。

## Provenance of the firmware

The ESP32 application in `firmware/` is a re-implementation of the stock application (version string `R-1.1.6`) that was written after analysing the stock firmware image, so that the stock phone app, the stock BLE/UART binary protocol and the stock flash locations (leg zero points, BLE name, pitch trim) keep working. Many source comments say which stock routine a piece of code follows (about 60 comments in 19 files, plus `battery_tables.h`, whose table values were read from the stock image, and `control/rider_motion.h`, a numeric reconstruction of stock routines).

* **No stock firmware image and no disassembly or instruction dumps are part of this repository.** (The robot's CAD display meshes are included: see "Data that is included".)
* The balance controller itself (the learned policy, the free-fall / lift / tilt guards, the dead-zone preload, the training pipeline and the simulator) is original work.
* The parts that follow the stock firmware are published by the manufacturer, who holds the rights to that firmware.

`firmware/` 里的 ESP32 应用是对原厂应用（版本串 `R-1.1.6`）的重新实现，写在分析原厂固件镜像之后，目的是让原厂手机 App、原厂 BLE/UART 二进制协议和原厂 flash 位置（腿零点、BLE 名称、俯仰修正）继续可用。许多源码注释写明了代码对应原厂的哪段例程（19 个文件约 60 处，另外 `battery_tables.h` 的表值读自原厂镜像，`control/rider_motion.h` 是对原厂例程的数值重建）。

* **本仓库不包含原厂固件镜像、反汇编或指令转储。**（机器人的 CAD 显示网格包含在内，见下面的 “Data that is included”。）
* 平衡控制本身（学习到的策略、自由落体/拿起/倾倒保护、死区预加载、训练流程和仿真器）是原创工作。
* 跟随原厂固件的那部分，由拥有该固件权利的厂家一并发布。

## Data that is included

* `training/models/mass_audit/calibrated_distribution.json`: mass-distribution priors per part (mass, centre, specific inertia) that were estimated from the robot's CAD and the measured assembly masses. Numbers only; no geometry.
* `training/models/cad/` (`meshes/*.stl`, `rider_cad.xml`, `rider_measured_cad.xml`, `cad_inspection.json`): display meshes (binary STL, 8.9 MB) converted from the robot's CAD assembly (v2026.09.18), the model files that use them, and the component list of that assembly. The manufacturer publishes them **for non-commercial use only** (CC BY-NC 4.0, see `training/models/cad/LICENSE`), just to make simulation and training convenient; the STEP file itself (138 MB) is not in this repository. The meshes only draw the robot (and serve as collision shells in two fall probes): training and evaluation use the simple collision geometry of `rider_measured.xml`, and everything except the two fall probes works without this folder.
* `training/data/real_robot_traces/`: numeric telemetry (pitch, rates, wheel commands and speeds, leg positions) recorded from one robot while it balanced with an earlier policy, used to identify the wheel plant. No identifiers.
* `training/artifacts/balance-v6/`: the trained policy v6 (weights, training configuration, evaluation reports) and `balance-legs-v5/actor.json`, the earlier policy that rang on the robot (kept for the before/after stability-margin comparison).
* Checkpoint files (`*.zip`, `*.pt`) are PyTorch/pickle formats. **Only load checkpoints you trust.**

## Promotional videos / 项目视频

`media/` contains the Chinese and English project videos, covers, and subtitles. Video visuals use the CAD meshes described above and retain their CC BY-NC 4.0 terms. The embedded music (Kevin MacLeod, "Digital Lemonade", CC BY 4.0) and sound effects (Kenney Interface Sounds, CC0) retain their own licenses. The code's Apache-2.0 license does not replace these media licenses. Full source links, required attribution, and a description of the simulated footage are in [media/README.md](media/README.md).

`media/` 包含中英文项目视频、封面和字幕。CAD 画面保留上述 CC BY-NC 4.0 条款；背景音乐（Kevin MacLeod 的 Digital Lemonade，CC BY 4.0）与音效（Kenney Interface Sounds，CC0）保留各自的许可。代码的 Apache-2.0 许可不替代这些媒体许可。素材来源、转载所需的署名和仿真画面说明见 [media/README.md](media/README.md)。

## Third-party software

Not redistributed here; installed by the build/training scripts. Each keeps its own license:

| Component | Used for | License |
| --- | --- | --- |
| PlatformIO Core, platform `espressif32` | build | Apache-2.0 |
| Arduino-ESP32 (framework-arduinoespressif32 2.0.17), ESP-IDF 4.4 | firmware runtime, BLE | LGPL-2.1 (Arduino core), Apache-2.0 (ESP-IDF, BLE library) |
| Xtensa ESP32 GCC toolchain | compiler | GPL with runtime-library exception |
| esptool | flashing | GPL-2.0 |
| MuJoCo | simulator | Apache-2.0 |
| Stable-Baselines3, Gymnasium | PPO, environment API | MIT |
| PyTorch | networks | BSD-3-Clause |
| NumPy, SciPy, Matplotlib, imageio | numerics, plots, image output | BSD-style |
| pyserial, bleak | USB / BLE tools | BSD-3-Clause, MIT |

A firmware image that you build contains LGPL code (Arduino-ESP32). This repository ships the complete source and build recipe, which is what the LGPL asks for when you redistribute the binary together with it.
