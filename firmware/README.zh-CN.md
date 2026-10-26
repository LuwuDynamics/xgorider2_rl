# firmware/ — 构建、测试、烧录、运行

XGO-Rider2 主控板（ESP32）上的应用。平衡控制器是 `src/policy/trained_weights.h` 里训练好的 18-32-32-4 策略（模型 `rider_balance_v6`）；其余部分（IMU 融合、腿位置控制、舵机总线、BLE/USB 协议、保护）都是围绕它的普通 C++。[English](README.md) · [架构](docs/ARCHITECTURE.md) · [安全须知](../SAFETY.md)

```
src/                 固件（C++17，PlatformIO 上的 Arduino-ESP32）
  policy/            float32 actor：policy.cpp/.h、dead_zone_comp.h、trained_weights.h（权重）
  fall_guard.h  lift_guard.h  runtime_guard.*  led_status.h     保护
  actuator_bus.*  board_imu.*  board_support.*  ble_transport.*  command_protocol.*  硬件与协议
  control/           IMU 融合、腿运动学、平滑/PID 辅助函数
  main.cpp           任务、控制台、状态 JSON
tests/               主机测试：16 个 C++ 程序 + 9 个 Python 测试模块（不需要硬件，不打开串口）
tools/               烧录器、USB 控制台、trace 采集与报告、导出器
docs/                ARCHITECTURE.md、RL_POLICY.zh-CN.md（深入说明）、release-notes/（硬件测试记录）
platformio.ini  partitions.csv  partitions.bin  build.sh  requirements.txt
```

## 构建

需要 Python 3；第一次要联网（PlatformIO 会下载固定版本的工具链：`espressif32@6.9.0`、Arduino-ESP32 2.0.17、Xtensa GCC 8.4）。

```sh
sh build.sh                 # 创建 .venv，安装 requirements.txt，构建 rider2-rl-only，写入 dist/
```

结果：`dist/firmware.bin`（约 1.24 MB，应用槽 1.5 MB）和 `dist/manifest.json`（SHA-256、镜像检查、链接进去的模型元数据、构建版本）。`dist/firmware.elf` 和 `.map` 只用于调试，里面有构建目录字符串；发布时只放 `firmware.bin` 和 `manifest.json`。

构建**逐位可复现**：同样的源码在两个不同目录构建出同一个 `firmware.bin`（编译器被要求改写工程和包的路径）。请在真实路径下构建，不要通过符号链接。参考值：本仓库状态的源码 → `sha256 4a85fc892c9eeea0bfe1d131610a366b4a297489158675fc9f56f713f532863b`，1,241,600 字节。

导出器在以下条件不满足时拒绝写 manifest：镜像校验和与附加的 SHA-256 有效、镜像能放进原厂 0x180000 字节的槽、链接的模型与权重头文件一致、旧的轮子 PID 符号一个都没有链接进去（`rider2-rl-only` 没有后备控制器）。

## 测试

```sh
python3 tools/run_host_tests.py     # 需要 C++17 编译器（clang++ 或 c++）、numpy、scipy
```

25 组：策略推理及其限制、保护（自由落体、拿起、运行时/倾倒）、舵机总线编解码与传输、BLE/USB 命令解析及其带固定种子的模糊测试、flash 设置写入（flash 是桩）、LED 故障码映射、权重导出器、烧录器（所有 esptool 调用都是假的）、trace 工具和报告。C++ 用 `-Wall -Wextra -Werror` 编译；策略、腿控制器和协议模糊测试在 AddressSanitizer + UBSan 下运行。全程不打开串口。

## 烧录

先读 [../SAFETY.md](../SAFETY.md)，并且**先备份原厂 flash**。烧录器需要 `esptool.py`（PlatformIO 会装一份：`~/.platformio/packages/tool-esptoolpy/esptool.py`）。

```sh
# 1. 离线检查 dist/（不打开串口）
python3 tools/flash_app.py --esptool ~/.platformio/packages/tool-esptoolpy/esptool.py --plan
# 2. 接上机器人的 USB 下载口：检查设备，仍然不写
python3 tools/flash_app.py --esptool ~/.platformio/packages/tool-esptoolpy/esptool.py --port /dev/cu.usbserial-XXXX
# 3. 写入引导程序当前选中的应用槽，再校验摘要（115200 波特率下约 75 秒）
python3 tools/flash_app.py --esptool ~/.platformio/packages/tool-esptoolpy/esptool.py --port /dev/cu.usbserial-XXXX --flash
```

分区表和原厂不同、OTA 元数据校验不过或不在稳定状态、manifest/镜像检查不过时，它都会拒绝继续。出错后板子如果留在下载模式，请断电重启。之后用 115200 波特率打开串口。

## 运行

烧录完上电，把机器人扶起来托稳不动：它会自己开始平衡（条件见 [../SAFETY.md](../SAFETY.md)）。原厂手机 App 通过 BLE（服务 `0xFFF0`）遥控它。手机指令按策略验收范围限幅：前进 ≤ 165 mm/s，转向 ≤ 36 °/s。

### USB 控制台（115200 波特率，一行一条命令，接入不会复位板子）

```sh
python3 tools/rider_cli.py --port /dev/cu.usbserial-XXXX status     # JSON：状态、停机原因、传感器、舵机、策略、保护、耗时
python3 tools/rider_cli.py --port /dev/cu.usbserial-XXXX stop       # 轮归零、腿卸力，锁定
python3 tools/rider_cli.py --port /dev/cu.usbserial-XXXX arm        # 解除锁定；机器人扶正静止后重新起立
```

原始控制台命令以 `!` 开头（`!status`、`!stop`、`!arm`、`!neutral`、`!move F T`、`!height B`、`!cal begin`、`!cal save`、`!name SUFFIX`、`!trace dump`、`!trace usb on`、`!liftguard on|off|status`、`!fallguard on|off|status|clear|ms N`、`!dzcomp …`）。`!stop` 优先于一切。

`status` / `last_stop_reason` 里的停机原因：`tilt`、`free_fall`、`policy_fault`、`imu_missing_stale_or_nonfinite`、`actuator_feedback_missing_or_stale`、`control_gap_over_30ms`、`invalid_control_output`、`invalid_flash_calibration`、`low_battery`、`calibrating`、`manual_stop`、`startup`。停机时的 LED：倾倒红色常亮，自由落体红色快闪，其他故障红色慢闪，主动停机琥珀色。

### 记录和报告

| 工具 | 作用 |
| --- | --- |
| `capture_trace_usb.py` | USB 上 55 Hz trace（俯仰、角速度、轮指令/轮速、负载、加速度计、标志位），带语音提示的测试流程（`--protocol drop`、`release`、`lift` …） |
| `incident_snapshot.py` | 摔倒或停机之后马上保存 `status` 和 RAM 里最近约 18 秒的 trace |
| `drop_report.py`、`replay_fall_guard.py` | 分析自由落体测试 / 用固件自己的检测器回放录像 |
| `standing_report.py`、`dzcomp_ab_test.py`、`start_cycle_test.py` | 站得稳不稳、死区预加载 A/B、复位后起立循环 |
| `calibrate_wheel_speed.py` | 台架核对仿真假设的轮速比例（真实速度的 0.28） |
| `capture_trace.py`、`hardware_ble_test.py` | BLE trace 采集和只读 BLE 往返测试（需要 `bleak`） |

## 换成自己的权重

```sh
cp ../training/artifacts/<你的训练>/policy_weights.h          src/policy/trained_weights.h
cp ../training/artifacts/<你的训练>/policy_weights.manifest.json src/policy/trained_weights.manifest.json
python3 tools/run_host_tests.py && sh build.sh
```

头文件必须是 `contract 6` 和 `wheels_legs` 动作空间；构建和导出器都会检查，`static_assert` 会挡住不匹配。要改观测约定，需要同时改 `policy.cpp` 和 `training/rider_sim/env2.py`，见 [../training/README.zh-CN.md](../training/README.zh-CN.md)。

## 文档

* [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — 任务、耗时、策略接口、保护（English）。
* [docs/RL_POLICY.zh-CN.md](docs/RL_POLICY.zh-CN.md) — 策略部署细节、保护及其证据。
* [docs/release-notes/](docs/release-notes/) — 硬件测试记录。
