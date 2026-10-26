# Rider2 RL-only 固件发布说明（2026-10-22）

> 注：这是作者的工程笔记，按原工作区的目录写成。公开仓库里：`simulation/` 就是 `training/`，`rider/rider2-firmware/` 就是 `firmware/`，`sh build.sh` 的产物在 `dist/`（正文里的 `dist-rl-only*/` 是历史构建目录），`logs/` 里的测试记录和 `evidence/` 没有公开。


轮子只由训练好的网络 `rider_balance_v6` 控制（没有 PID 回退）；腿的高度和横滚调平仍由原来的腿位置控制器负责。

## 镜像

* `dist-rl-only/firmware.bin`（副本 `dist-rl-only-release-20261022/`），1,234,304 字节，
  sha256 `a5e01f76f3e471e457fb473bda10cf844e5edf4b732301209846387246d7fb97`；
  `!status` 里的 `build` 字段是 `rider2-rl-only | rider_balance_v6 | lift guard v4 | servo-dropout auto re-arm | 2026-10-22`。
* 模型 `rider_balance_v6`，model_id 656001308，观测约定 contract 6（来龙去脉见 `../../../training/docs/REBUILD.zh-CN.md`）。
* 刷入（只写应用分区，不动 bootloader、分区表、NVS、标定区）：
  `tools/flash_app.py --port /dev/cu.usbserial-XXXX --artifact-dir dist-rl-only --flash --esptool ~/.platformio/packages/tool-esptoolpy/esptool.py`
  （刷入前托住机器人；刷完它重启，处于“等待起立”状态。）

## 行为

* **起立**：开机约 3 秒后，把机器人托到直立（|俯仰|<8°、|横滚|<5°）并保持静止，策略自动启动，起来后再放手。躺在地上不会自己起。
* **拿起保护**（默认开）：两轮平均速度（或半差）超过 300（寄存器单位）持续约 30 ms，就把轮指令压成 0，腿保持上电；
  机器人重新直立、没被手晃动、轮子静止约 0.3 s 后自动放开（放开后 1 s 内又触发，下次等约 0.5 s）。
  实测把空转从“满速几秒”变成“冲到约 400 后 0.3 s 内停下”。保护触发后到放开之前松手，机器人会倒。
* **舵机掉线自动恢复**：某个舵机停止应答（`actuator_feedback_missing_or_stale`）会停机，所有舵机重新应答且机器人直立静止约 1 s 后自动重新起立，
  不需要手机“起立”。舵机一直不应答就一直不起，也不会有任何动作。
* 手动停机（`!stop`）、低电、标定中仍然锁定，需要手机“起立”（action 255）或 `!arm`。
* 倒地（俯仰>45°）、控制超时等停机，扶正静止约 1 s 后自动重新起立。
* App 里的 6 个动作是原厂脚本，只改高度、横滚、前进和转向的指令，轮子仍由网络控制；速度上限 165 mm/s、转向 36°/s。

## USB 控制台（115200，用 `tools/rider_cli.py` 或其它串口工具）

`!status`（JSON；看 `reason`、`last_stop_reason`、`feedback[].age`、`lift_guard`、`build`）、`!stop`、`!arm`、`!policy off|shadow|active|status`、
`!liftguard on|off|status`、`!speedcal 0.2..4`（仅静止时，复位后回到 1.0；训练假设轮速读数是真实速度的 0.28 倍，实测 0.275，不用改）、
`!trace dump`（导出最近约 18 秒 55 Hz 的数据）、`!trace usb on|off`（连续导出，同时写出停机/启动事件行 `# ev`）。
`tools/rider_cli.py` 和下面几个工具打开串口时不会复位板子；别的串口工具可能会。

## 出问题时

* 起不来：`!status` 看 `reason`。`startup` 多半是没托直立或没静止；`actuator_feedback_missing_or_stale` 看 `feedback` 里哪个舵机 `age` 是空的；
  `invalid_flash_calibration` 是腿零位没标定。
* 出怪事（摔了、轮子突然松、没反应）后**马上**跑：
  `tools/incident_snapshot.py --port /dev/cu.usbserial-XXXX --note "发生了什么"`，它把状态和最近 18 秒的 trace 存到 (private test log, not published)<时间>/`（只读，不复位）。
* 录一段数据：`tools/capture_trace_usb.py`（不占蓝牙）；复位-起立统计：`tools/start_cycle_test.py`；分析：`../../training/tools/trace_report.py`。

## 已知限制

* 没有实机长时间验证；仿真里没有齿轮间隙和地面软硬（轮子死区和静摩擦现在有了，站立踱步和修复见 `2026-10-22-to-24-dead-zone-fall-guard.zh-CN.md`）；腿的两路网络输出固定为 0；偏航增益没有辨识。
* 拿起保护的阈值只在一台机器上、约 10 分钟的数据里定过（推的力度最大时两轮平均速度 222，限值 300）；站在地上轮子突然松掉就是误触发，用 `!liftguard off` 关掉并跑 `incident_snapshot.py`。
* 旧机器的左腿 12 号舵机会在腿第一次收到指令时哑掉，之后只有断舵机电才恢复；已判断为舵机齿轮损坏，换了一台机器。

## 回退

* 这台机器（MAC xx:xx:xx:xx:xx:xx，ESP32-D0WD-V3）刷入前的整片 4 MB flash 备份：(private test log, not published)，
  以及切出来的应用分区 `app0-before.bin`、标定区 `calibration-gap-before.bin`。只恢复应用：
  `esptool.py --port /dev/cu.usbserial-XXXX --baud 460800 write_flash 0x10000 (private test log, not published)；
  整片恢复（会连标定一起覆盖，只在必要时用）：`write_flash 0x0 (private test log, not published)。
* 之前的镜像都留着：`dist-rl-only-v6-flashed-20261022/`（第一版 v6）、`...-v6-usbtrace-liftguard-20261022/`、`...-v6-liftguard-v3-…/`、`...-v4-…/`、`...-v5-…/`，
  上一代 RL 是 `dist-rl-only-v8-contract5/`，原厂是 `dist/`。
