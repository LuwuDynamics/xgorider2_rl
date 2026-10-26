# Safety / 安全须知

**English first, 中文在后。**

This firmware drives real motors. A balancing robot that is wrong, stale or hacked will fall, spin its wheels or crush a finger. Read this page before the first flash and again before the first run.

## Before you flash

1. **Back up the stock firmware** (the 4 MB flash contains the stock app, the leg calibration and the BLE name — keep all of it):
   ```sh
   python ~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32 --port <PORT> --baud 460800 read_flash 0 0x400000 stock-backup.bin
   ```
   (If the read fails with a CRC/length error, retry with `--baud 230400` or `115200`.) Keep that file private: it contains your robot's device keys. To go back: `esptool.py --chip esp32 --port <PORT> write_flash 0x0 stock-backup.bin`.
2. Use only `firmware/tools/flash_app.py`. It reads the partition table and the OTA metadata first, **refuses** any device that is not a stock XGO-Rider2 layout, and writes only the app slot the bootloader currently selects. It never touches the bootloader, partition table, NVS, OTA data, SPIFFS or the calibration sector range 0x310000–0x32ffff.
3. Flashing replaces the stock application. You are responsible for your robot; there is no warranty.

## First run

* Support the body so the wheels are free or the robot cannot fall far; keep the **power switch within reach**.
* The firmware starts balancing **by itself** as soon as all of these hold: at least 3 s since boot, IMU calibrated, leg calibration valid, all four servos answering, battery value valid, and the robot is held upright and still (|pitch| < 8°, |roll| < 5°). Holding the calibration switch blocks it.
* Wheels in the air are not a balance test. A wheel that spins freely runs away within 0.2 s (the lift guard then holds the wheel command at zero).
* After any protective stop the robot restarts by itself when it is stood up, upright and still for about one second. A stop you ask for (`!stop`, calibration, low battery) stays latched until you `!arm`.
* Do not use it near people, pets, stairs, water or edges you care about. The policy was qualified for low speeds (≤ 165 mm/s forward, ≤ 36 °/s turn by the firmware's limits).

## Protections that exist, and what they do *not* cover

| Protection | What it does | Limits |
| --- | --- | --- |
| Tilt stop | \|pitch\| > 45° or \|roll\| > 60° → wheels to zero, leg servos released | stops after the tip-over has started |
| Lift guard | wheel speed runs away in the air → wheel command held at zero until upright and calm | a heuristic on wheel speed |
| Free-fall guard | total acceleration < 0.5 g for 60 ms (adjustable 20–200 ms with `!fallguard ms N`), or < 0.061 g for 6 ms → stop, servos released *before* the impact | blind when the body spins fast in the air (the tilt stop is the backup); drops below roughly 15–20 cm can land before the servos are released; releasing the servos does not guarantee gear teeth survive |
| Fault stops | IMU / servo feedback lost, control overrun, invalid numbers, low battery → wheels zero, legs released | |
| LED fault codes | tilt: solid red · free fall: fast red flash · other fault: slow red flash · requested stop: amber | |

## Known security limitation: the BLE interface is open

The firmware keeps the stock BLE service (`0xFFF0`) **without pairing or authentication**, exactly as the stock firmware does. Anyone within radio range can connect and

* drive the robot, stop it, or tell it to stand up;
* change runtime tuning values (wheel gain, smoothing, slew, wheel-speed scale) — even while it balances;
* enter leg calibration (the robot goes limp) and **save new leg zero points to flash**, or change the BLE name (flash write).

Treat the robot like a toy that is only safe in a trusted place. If you deploy it anywhere else, the minimum hardening is: accept the tuning commands (`0xb3–0xb8`) and flash-writing commands (`0x04`, `0x13`) only from USB, require a physical button for calibration saves, and add BLE pairing. These changes are not part of this release.

## About the model

* The policy is trained in simulation with a plant identified from specific robots. Another unit, another payload (the compute module weighs differently), a worn gearbox or a different floor can behave differently. The weakest simulated region ("corners", all random parameters extreme) survives 72 % of the time.
* The legs are not learned in this release. Their two outputs are fixed at zero.
* Hardware validation was done by the maintainer on three robots. It is not a certification.

---

# 安全须知（中文）

这个固件直接驱动真实电机。平衡机器人一旦出错、数据过期或被人恶意控制，会摔倒、空转或夹手。第一次烧录前、第一次运行前各读一遍。

## 烧录之前

1. **备份原厂固件**（4 MB flash 里有原厂应用、腿校准和 BLE 名称，全部保留）：
   ```sh
   python ~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32 --port <串口> --baud 460800 read_flash 0 0x400000 stock-backup.bin
   ```
   （如果读取报 CRC/长度错误，换成 `--baud 230400` 或 `115200` 重试。）这个文件请自己保管，不要公开：里面有你机器人的设备密钥。要恢复原厂：`esptool.py --chip esp32 --port <串口> write_flash 0x0 stock-backup.bin`。
2. 只用 `firmware/tools/flash_app.py`。它先读分区表和 OTA 元数据，**不是**原厂 XGO-Rider2 的布局就拒绝，只写引导程序当前选中的那个应用槽；不碰引导程序、分区表、NVS、OTA 数据、SPIFFS，也不碰校准区 0x310000–0x32ffff。
3. 烧录会替换原厂应用。机器人由你自己负责，不提供任何担保。

## 第一次运行

* 托住机身，让轮子悬空或者机器人摔不远；**电源开关要随手够得着**。
* 以下条件同时满足，固件就会**自己**开始平衡：开机至少 3 秒、IMU 校准完成、腿校准有效、四个舵机都有应答、电池数据有效，并且机器人被托直立、不动（|俯仰| < 8°，|横滚| < 5°）。按住校准开关会阻止启动。
* 轮子悬空不是平衡测试。空转的轮子 0.2 秒内就会飞速运转（拿起保护会把轮指令按住为 0）。
* 任何保护性停机之后，把它扶正、直立静止约 1 秒，会自动重新起立。你主动要求的停机（`!stop`、校准、低电）保持锁定，直到 `!arm`。
* 不要在有人、宠物、楼梯、水、你在乎的桌边使用。策略按低速工况验收（固件限制：前进 ≤ 165 mm/s，转向 ≤ 36 °/s）。

## 已有的保护和它们没覆盖的地方

| 保护 | 做什么 | 局限 |
| --- | --- | --- |
| 倾倒停机 | \|俯仰\| > 45° 或 \|横滚\| > 60° → 轮归零、腿舵机卸力 | 倒下开始之后才停 |
| 拿起保护 | 轮子在空中飞转 → 轮指令按住为 0，直到扶正并平静 | 基于轮速的启发式 |
| 自由落体卸力 | 总加速度 < 0.5 g 连续 60 ms（`!fallguard ms N` 可调 20–200 ms），或 < 0.061 g 连续 6 ms → 停机，**落地之前**卸掉舵机力 | 机体在空中快速旋转时它看不见（倾倒停机兜底）；大约 15–20 cm 以下的跌落可能在卸力完成前就落地；卸力不保证齿轮不断 |
| 故障停机 | IMU/舵机反馈丢失、控制超时、数值无效、低电 → 轮归零、腿卸力 | |
| LED 故障码 | 倾倒：红色常亮 · 自由落体：红色快闪 · 其他故障：红色慢闪 · 主动停机：琥珀色 | |

## 已知的安全局限：BLE 接口是开放的

固件保留了原厂 BLE 服务（`0xFFF0`），**没有配对、没有认证**，和原厂固件一样。射频范围内的任何人都可以连上并且：

* 遥控机器人、让它停机、让它起立；
* 修改运行时调参值（轮增益、平滑、斜率、轮速比例），平衡过程中也可以；
* 进入腿校准（机器人会瘫软）并把**新的腿零点写进 flash**，或者改 BLE 名称（写 flash）。

请把它当成只能在可信环境里使用的玩具。要在别处部署，至少要做：调参命令（`0xb3–0xb8`）和写 flash 的命令（`0x04`、`0x13`）只接受来自 USB 的，校准保存要求按物理按钮，再加上 BLE 配对。这些改动不在本次发布里。

## 关于模型

* 策略是在用具体几台机器辨识出的仿真里训练的。换一台机器、换负载（计算模块重量不同）、齿轮磨损或换个地面，表现可能不同。最弱的仿真区域（“角点”，所有随机参数取极端）存活率 72 %。
* 本版没有训练腿，腿的两路输出固定为 0。
* 硬件验证是作者在三台机器上做的，不是认证。
