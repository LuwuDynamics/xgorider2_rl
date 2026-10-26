# ESP32 原生强化学习策略推理

> 注：这是作者的工程笔记，按原工作区的目录写成。公开仓库里：`simulation/` 就是 `training/`，`rider/rider2-firmware/` 就是 `firmware/`，`sh build.sh` 的产物在 `dist/`（正文里的 `dist-rl-only*/` 是历史构建目录），`logs/` 里的测试记录和 `evidence/` 没有公开。


> ## Contract 6（2026-10-21 重建，当前代码）
> 旧版（contract 5）在实机 7–14Hz 持续共振，原因和新流程见 `training/docs/REBUILD.zh-CN.md`。contract 6 相对 contract 5 的变化：
>
> * **每个控制 tick 推理**：`kPeriodUs` 10000→3000（控制循环本身 ≥4ms，所以实际每 tick 一次；原来每 12ms 一次，平均多 6ms 保持延迟）。
> * **输入 5/6 = 经低通的轮速**：一阶低通，时间常数 `kSpeedFilterTauS`=0.12s，复位后用第一个样本初始化。轮速通道在 5–14Hz 的增益/相位最不可靠，用结构把它挪出快回路。
> * **输入 17 = 位置误差积分**（m·s，`kPosIntegralLimit`=±1），在该 tick 更新前采样，arm/停机/模式切换清零。原来的偏航积分状态（原输入 17）仍在内部修正轮命令，但不再作为输入。
> * 输入 7/8（腿编码器）、13/14（上一拍命令）、15/16（推理/控制间隔）仍由固件提供，但**导出的权重对它们的第一层列为 0**，网络与之无关（`export_actor2.py`）。
> * 腿的两个输出（2/3）本轮导出为 0；腿由原 `LegPositionController` 负责。
> * 新增运行时参数：`!speedcal X`（串口）/ BLE 寄存器 0xb8（值=64×X），缩放策略看到的轮速，范围 0.2–4，仅静止时可改。默认 1。
>   训练仿真假设轮速读数≈真实速度的 0.28 倍；若手推核对（`tools/calibrate_wheel_speed.py`）显示不是，用它修正而不必重训。
>   2026-10-22 实测：读数 = 编码器速度的 0.275 倍（r=3.63，记录在 (private test log, not published)），与假设吻合，保持默认 1。
>   `speed_cal` 只存在 RAM 里，复位后回到 1.0；这块板子每次打开串口都会复位，如果将来需要非 1 的值，应改固件默认值而不是靠 `!speedcal`。
> * `trained_weights.h` 的 contract 必须为 6，旧 contract 5 权重（v4/v5/v8）不能直接加载。
> * 旧诊断寄存器 0xb4–0xb7（轮增益/平滑/斜率）保留，默认值为中性（1.0 / 0 / 20000 / 20000）。
>
> 下文其余部分描述 contract 5 的细节，凡与上面冲突处以本节为准；观测表里 5、6、17 号输入按本节理解。

本模块把训练后的 actor 放进 Rider2 的经典 ESP32，在本地控制任务里计算双轮命令。
训练在 Mac/工作站进行；下位机只做前向推理，不包含 ONNX Runtime、训练器或 critic。

## 当前默认版本：RL-only 双轮 + 双舵机残差（contract 5）

`sh build.sh`（或 `sh build.sh rl-only`）构建 `rider2-rl-only`，输出 `dist-rl-only/`。
模型名称、ID 和 SHA-256 见 `src/policy/trained_weights.h` 的 `policy_export` 元数据。上电选中 active，
等待原有传感器校准、反馈有效和扶正条件后启动。网络输出四维：两轮直接命令，以及两个腿部伸缩偏移；
腿部基准高度、几何和侧倾调平仍由 `LegPositionController` 生成，RL 只叠加限幅残差。

该构建不编译 `motion_controller.cpp`。导出器检查 ELF 中没有旧的轮子平衡、速度、
位置保持、航向保持、摩擦补偿函数和对应 PID 构造函数。`LegPositionController` 只生成腿目标和滤波参考，
`LearnedWheelDrive` 不接受传统轮命令，任何 off、shadow、不可用输出或推理故障都只能给出零轮命令并停机。
RL 的两路腿部残差在 `offsetLegExtension()` 中换算成舵机目标，范围受限，不会越过 45–81mm 机械伸缩区间。
RL-only 构建强制启用地平线/侧倾调平回路，与训练时的仿真一致。
停机/倒地后的恢复：手动停机、低电、校准仍锁定，需要手机“起立”（action 255）或 `!arm` 重新选择 active 并等待扶正。
倒地、策略故障、控制超时、传感器故障，以及舵机不应答（2026-10-22 起；12 号腿舵机曾在腿第一次收到指令时哑掉几秒）这类驾驶中发生的自动停机，
在机器人被扶正并静止约 1 秒、所有舵机重新有应答之后会自动重新选择 active；轮速需低于 30mm/s，且仍要等扶正条件满足才启动。
某个舵机一直不应答就一直不会重启，也不会有任何动作。
任何情况下都不会切回 PID。旧 `!trim`（依赖轮子 PID 的俯仰配平）不适用于该版本，腿零位校准继续支持。

蓝牙遥控与动作生成器保留，速度/转向限制为 165mm/s 和 36°/s，高度目标0–29、侧倾命令±4°。
旧动作仍可驱动腿部运动，但幅度受当前策略参考范围限制；能执行动作不表示轮子用了传统控制。
模型自带的限幅转向积分项保留，属于该模型导出的控制配置。
训练环境新增了舵机一阶跟踪滞后、回差、轮速反馈噪声和更大的命令/反馈延迟随机化，
并根据实测质量模型重训；因此权重和观测契约与旧 `rider_balance_measured_v4` 不兼容。

`!status` 新增 `controller=rl_only`、`legacy_wheel_pid=false`、`wheel_source=rl/stopped`、
`rl_output_ticks` 和 `last_stop_reason`。新增 BLE 读寄存器 `0xb2`，固定16字节：

| 字节 | 内容 |
| --- | --- |
| 0 | 控制构建：0=历史混合版，1=RL-only |
| 1 | 当前轮输出来源：0=停机，1=传统控制，2=RL |
| 2 | 策略模式：0=off，1=shadow，2=active |
| 3 | 最近停机原因（`runtime_guard.h` 的 StopReason） |
| 4–7 | 实际推理调用次数，uint32 LE |
| 8–11 | 推理失败次数，uint32 LE |
| 12–15 | 已提交 RL 轮输出的控制 tick 数，uint32 LE |

0xb0 不变；0xb1 的 `contract` 字节为 5，`action_space` 字节为 2（`wheels_legs`）。原有45字节遥测不变。可用 `tools/hardware_ble_test.py --rl-only --read-only`
读取构建和实际输出来源；手机需先释放 BLE 连接。停机时 source=0，不应仅凭 active 推断正在平衡。
本次主机验证与刷写/设备读取证据分别记录在 `evidence/rl_host_validation.json` 和
(private test log, not published)；实机能否自由平衡需根据地面运行遥测与实际观察判断。

## 保留的历史混合构建

`sh build.sh diagnostic` 附带 `diagnostic_untrained` 诊断权重，只能用于 `shadow`。
Mac 的 MuJoCo 环境、训练和模型评估位于 `../../training/`。
训练模型经验证、导出到 `src/policy/trained_weights.h` 后，使用 `sh build.sh trained`
构建独立的 `dist-trained/`；默认诊断模型和原有 `dist/` 保留。
`trained=true` 是导出者的明确声明，不代表已通过仿真或整机验收。

## 网络与模式（模式回退描述仅适用于历史混合构建）

网络固定为 `18 -> Linear(32) -> Tanh -> Linear(32) -> Tanh -> Linear(4) -> Clip(-1,1)`。
共有 1796 个权重/偏置，占 7184 字节；均值和逆标准差另占 144 字节。
前两维按 `action_scale` 映射成轮命令；后两维是归一化腿部伸缩残差，按 `leg_action_scale=0.005m`
映射到每条腿 ±5mm，并以 10 单位/秒（约 200ms 满量程）限速。观测仍为 18 维（contract 5）。
参数是 `const float`，放在固件只读段；推理没有堆分配，网络局部数组共 84 个 float（336 字节），另有调用栈和控制器状态。
最终 RAM/Flash 使用量以构建结果为准，这些数字不含原有固件、RTOS 和 BLE。

- `off`：不执行网络；RL-only 构建中双轮保持零输出，绝不回退旧轮控。
- `shadow`：执行网络并记录结果，返回的轮命令与原控制器完全相同；也能在手动停机状态下测推理耗时。
- `active`：应用模型声明的动作类型（`wheels_legs`：两轮+两腿残差），需要已安装的模型有效且明确标记为已训练。

模型的 `action_space` 在编译时固定，不能用同一组权重临时切换动作含义：

- `residual`：输出在原双轮命令上相加，`action_scale` 最大 128。
- `wheels`：直接替换双轮命令，`action_scale` 最大 1023。PID 仍计算腿位置和运动参考，其轮输出不叠加。
- `wheels_legs`：前两维同 `wheels`；后两维叠加到腿部伸缩目标，`leg_action_scale` 最大 0.02m。

模型可显式声明 `yaw_integral_gain` 和 `yaw_integral_limit`，默认0/.05。
正式模型用学习网络负责平衡，并加限幅的转向积分校正消除恒定转向偏差；每个控制 tick
按 `(目标角速度-实测角速度)*gain*dt` 积分，将该项从两个原生轮动作中同时减去。
校正保持共同轮驱动不变，经过同样的限速/限幅。停机、arm、切换模式时清零，限幅防止积累过大。
配置属于模型 SHA-256，增益/限幅是额外2个 float，状态为1个 float；gain大于0只允许直接轮动作。
`action` 表示网络输出；观测13/14为上一控制 tick 的实际命令，包含转向校正和限速/取整，另报告 `yaw_integral`。

两种动作均经过每秒 20000 个驱动单位的变化率限制、四舍五入和最终 `[-1023,1023]` 限幅。
首次动作从零开始，使用 4ms 的初始变化率步长，最大变化 80；随后使用实际控制间隔。
腿部保留现有几何/调平控制。初版网络不控制腿部，也不解除原有倒地、低电、反馈过期等保护。

直接轮命令模式的 active 接管限定在训练过的低速参考范围：平滑前进≤165mm/s、
平滑转向≤36°/s、高度内部量0–29、侧倾指令≤4°，边界给编码量化留少量余量。
超限产生 `reference_out_of_range`，退出策略并锁定停机。此模型未开放高速、极端高度或动作库接管。
shadow 仍可观测较大参考；选择 off 后原控制路径的范围不变。

## 观测约定 v4

v2 将索引 12 改为前进位置误差；v3 将轮速输入改为执行器最近一次返回的轮速，
取消策略输入上原有 0.75/0.25 的轮速滤波，以减少滞后。原 PID 继续使用原有滤波轮速。
v4 将历史动作改为实际命令，并增加控制间隔与转向积分状态。
v1/v2/v3 权重不能作为 v4 模型直接加载。

以下是原始观测的固定顺序。推理先做 float32 的 `(raw - mean) * inv_std`，
再裁剪到 `[-observation_clip, observation_clip]`，默认上限 5。
训练时必须使用同样的顺序、滤波、符号、动作限制和采样/保持方式。

| 索引 | 原始观测 | 固件来源/转换 |
| --- | --- | --- |
| 0 | pitch，rad | `imu.pitch_deg * pi/180` |
| 1 | roll，rad | `imu.roll_deg * pi/180` |
| 2 | 原固件 pitch 角速度信号，rad/s | `imu.pitch_rate_rad_s`，已有滤波 |
| 3 | 原固件 roll 陀螺仪信号，rad/s | `imu.gyro_roll_filtered * pi/180` |
| 4 | yaw 陀螺仪信号，rad/s | `(imu.gyro_deg_s[2] - imu.gyro_bias[2]) * pi/180` |
| 5 | ID11 最近一次返回的轮速，m/s | `wheel_speed_mm_s * (61.6/55) / 1000` |
| 6 | ID21 轮速，m/s | 同上，包含现有 ID21 方向处理 |
| 7 | ID12 腿编码器偏移 | `(position_ticks - zero12) / 512` |
| 8 | ID22 腿编码器偏移 | `(position_ticks - zero22) / 512` |
| 9 | 平滑后的前进参考，m/s | `motion.command.filtered_forward / 1000` |
| 10 | 平滑后的转向参考，rad/s | `motion.command.filtered_turn * pi/180` |
| 11 | 平滑后的高度参考 | `motion.command.filtered_height / 36`，是固件内部量，不是绝对高度 |
| 12 | 前进位置误差，m | 双轮平均编码器里程乘 `61.6/55`，减启用时的起点，再减平滑前进参考的时间积分 |
| 13 | 上一控制 tick 的 ID11 命令 | 转向校正、变化率限制、取整后除以1023 |
| 14 | 上一控制 tick 的 ID21 命令 | 同上 |
| 15 | 两次推理之间的实际间隔，s | 首次固定 0.01；随后使用微秒时钟差值 |
| 16 | 两次控制 tick 的间隔，s | 首次0.004；随后使用微秒时钟差值 |
| 17 | 上一 tick 的转向积分状态 | 在当前 tick 更新积分之前采样 |

停机影子运行时索引 9..11 使用零参考；索引 12 仍反映相对里程变化。
位置参考在每个控制 tick 按实际间隔积分，首个 tick 捕获起点；首次推理的位置误差为零。
每次 arm、停机转移、校准、模式切换清空动作和位置参考历史。
模型权重和所有状态由控制任务独占；串口/BLE 仅通过已有命令队列改变模式。

特别注意当前恢复固件的惯例：pitch/roll 融合公式对陀螺仪项做减法，
所以表中的角速度信号不能直接等同于 `d(pitch)/dt` 或 `d(roll)/dt`。
仿真应复现当前安装方向和滤波，而不是按通用 IMU 坐标系猜测符号。
策略轮速和里程按所给 STEP 的 61.6mm 胎径修正恢复固件中的 55mm 换算；滚动半径仍需要实机标定。
两个输出依次对应执行器 ID11、ID21；其物理左右、转向符号和闭环响应仍待整机验证。
输出单位是驱动协议整数，不能直接当作 N.m、电流或 m/s。仿真需辨识轮驱响应。

## 调度与异常处理

网络目标间隔 10000us。它跟随现有控制循环运行，满足间隔后才计算一次；中间保持动作。
例如控制循环实际约 6ms，则策略可能约每 12ms 更新，并非已保证 100Hz。
训练环境应包含真实采样间隔、滤波和驱动延迟。

推理的归一化/矩阵计算/激活累计超过 2000us，或两次推理间隔超过 30000us，均判失败。
每个控制 tick 都检查原始观测有限性，即使当次仅保持动作也会检查。
状态过期、数值异常、模型异常同样使策略退出。
`active` 失败请求锁定停机，不会把损坏的网络输出发给电机；`shadow` 失败仅关闭策略。
正常的倒地/低电等保护仍先于动作输出执行，活动策略在停机转移时退出，不能自动恢复接管。
计时使用无符号差值，覆盖 `micros()` 回绕。

`!status` 中的 `policy` 对象包含模式、错误、模型 ID、训练声明、动作类型、调用次数、
最大/最近推理耗时、最近采样间隔，以及观测和动作。
这些耗时由 ESP32 自己测量。历史模型实测证据见 (private test log, not published)；
更换固件或训练权重后需重新测量，诊断权重的耗时不能代替实际策略的耗时验收。
更改观测含义、网络结构、滤波或动作限制后，应升级约定并重新训练/验证。

## 导出训练权重

`tools/export_policy.py` 接受显式 JSON，或带元数据的 PyTorch actor checkpoint。
导出器检查形状、激活、有限值、float32 可表示性、归一化参数和动作范围。
它生成 `src/policy/policy_weights.h` 和对应 `.manifest.json`，记录参数摘要及模型 SHA-256。

训练端 actor 必须是以下结构，导出的是学习后的参数，不是重新初始化的网络：

```python
actor = torch.nn.Sequential(
    torch.nn.Linear(18, 32), torch.nn.Tanh(),
    torch.nn.Linear(32, 32), torch.nn.Tanh(),
    torch.nn.Linear(32, 2),
)
# 完成训练并恢复最佳验证 checkpoint 后：
torch.save({
    "contract": 4, "architecture": [18, 32, 32, 2],
    "name": "rider_balance_v4", "trained": True,
    "hidden_activation": "tanh", "output_activation": "clip",
    "action_space": "wheels", "action_scale": 1023.0,
    "yaw_integral_gain": 0.04, "yaw_integral_limit": 0.05,
    "observation_clip": 5.0,
    "observation_mean": observation_mean.tolist(),
    "observation_std": observation_std.tolist(),
    "state_dict": actor.state_dict(),
}, "actor.pt")
```

```sh
python3 tools/export_policy.py actor.pt --torch
sh build.sh
```

actor 输入使用上述归一化和裁剪；训练/推理动作使用确定性的 actor 均值，再裁剪到 `[-1,1]`。
不导出 critic、采样噪声、`log_std` 或优化器。当前导出器不直接读取 SB3 的 PPO zip；
使用 SB3 时需按此约定提取 actor 并转换键名，或导出下述 JSON。

JSON 的元数据与上例相同，将 `state_dict` 换成三个 `layers`：
每层包含 `weight` 二维数组 `[out][in]` 和 `bias` 一维数组。
例如依次为 `[32,18]`、`[32,32]`、`[2,32]`。
JSON 导出命令为 `python3 tools/export_policy.py actor.json`，不依赖 PyTorch。

## 诊断 trace：蓝牙之外的 USB 导出（2026-10-22）

固件一直在内存里记一个 1000 条的环形缓冲（约 55 Hz、18.2 秒，只在策略驱动时写入）。原来只能通过蓝牙寄存器 0xb3 取走，
抓取脚本会占住机器人唯一的蓝牙连接，手机 App 就连不上。现在多了三个 USB 控制台命令，不占蓝牙：

* `!trace dump`：一次性把缓冲里现有的样本按时间顺序打印出来，最后一行 `# end N`；
* `!trace usb on` / `!trace usb off`：从“现在”开始连续打印新样本。

每行一个样本（`src/trace.h` 的 `kCsvHeader` 是列名，主机测试 `tests/test_trace.cpp` 锁定了格式）：
`T,seq,ms,pitch,pitch_rate,roll,yaw_rate,cmd11,cmd21,sp11,sp21,fsp11,fsp21,leg_a,leg_b,a0,a1,load11,load21,pos11,pos21,acc_x,acc_y,acc_z,flags,age11,age21,age12,age22,dt_us,load12,load22,pos12,pos22,vbat_adc`。
前 14 个数据列与蓝牙 trace 完全相同（单位见 `tools/capture_trace.py`），新增：`ms` 固件时钟，`load11/21` 轮舵机“当前负载”寄存器原值
（这类舵机 bit10 是方向位），`pos11/21` 轮位置 0..1023，`acc_*` 机体系加速度计原始值（cm/s²），`age*` 四个舵机（11、21、12、22）距上次应答的毫秒数，`dt_us` 这一拍的控制周期，`load12/load22/pos12/pos22` 两个腿舵机的当前负载和位置，`vbat_adc` 电池 ADC 原始值（未滤波，约 2.5 mV/格，用来看舵机使能瞬间有没有电压塌陷），`flags`：bit0 = 拿起保护正在把轮指令压成 0，bit1 = 拿起检测器已触发（只观察时也会置位），bit2 = 本拍轮速超过拿起限值。
USB 流开着的时候，驾驶状态、停机原因或拿起保护发生变化还会多打一行 `# ev ms=… drv=… reason=… last_stop=… lift=… acting=… trips=… bus_to=… bus_st=… bus_len=… bus_chk=…`，用来解释 trace 里的空档（`tools/capture_trace_usb.py` 把它们存到 `OUT.events.jsonl`）。
每个循环只打印几行，所以输出期间 `!stop` 照常及时生效；`!status` 的 feedback 里也多了 `load` 和 `raw_speed`，顶层多了 `accel`。

主机端：`tools/capture_trace_usb.py --port /dev/cu.usbserial-XXXX OUT.jsonl --seconds 90 [--reset] [--cue] [--protocol lift]`，
输出和蓝牙抓取同样的键（`tools/trace_report.py` 通用），再加上新增的列。`tools/rider_cli.py` 和这个抓取脚本打开串口时先放 RTS、再放 DTR，附着不会让板子复位（正在平衡的机器人不受影响；`--reset` 才会重启）；
用别的串口工具（会同时拉起 DTR/RTS）打开时仍可能复位，策略会重新选中 active。

### 出了怪事以后留证据：`tools/incident_snapshot.py`

固件里只留最近约 18 s 的 trace 和最后一次停机原因，复位后就没了，再多开 18 s 也会被覆盖，所以出事后马上跑：
`python tools/incident_snapshot.py --port /dev/cu.usbserial-XXXX --note "发生了什么"`。
它不复位板子，只发 `!status` 和 `!trace dump` 两条只读命令，存到 (private test log, not published)<时间>/`（`status.json`、`trace.jsonl`、`note.txt`），
并在屏幕上给出摘要：停机原因和上次停机原因、拿起保护触发次数、四个舵机多久没应答、总线错误数、trace 里轮速峰值和保护压轮的时间。
trace 里的 `age11/age21/age12/age22` 能看出是哪个舵机先“哑”的（2026-10-22 三次“起不来”都是 12 号，且发生在腿第一次收到指令的那一拍）。

## 拿起保护（lift guard，2026-10-22）

现象：把站着的机器人拿起来，轮子会猛转。实测（(private test log, not published)）：轮子悬空时没有地面反力，策略对轮速的反馈（和倾斜同号，
这是倒立摆状态反馈的正常结构）变成开环不稳定的正反馈；手一碰、一倾斜，指令越过轮子死区（约 ±45 以内不动），
轮速在约 0.2 s 内从 0 飙到 540（寄存器单位，约 2 m/s 真实速度），指令饱和在 ±1023，机身几乎不动（俯仰 ±4°、没有水平加速度）。
站在地上的 v6 机器人：两轮平均速度在用力推时最大 222，手机遥控时只有 56。

做法（`src/lift_guard.h`，纯头文件，`tests/test_lift_guard.cpp` 覆盖）：两个滤波后轮速的平均值（或半差）超过 300（寄存器单位）并持续 6 个控制拍
（控制周期约 5 ms，即约 30 ms）就认为轮子悬空。这时把轮指令压成 0（腿仍然上电），并且每拍重置策略历史，免得积分项在这期间累积。
放开的条件是机器人看起来和站在地板上一样：直立（|俯仰|<8°、|横滚|<5°）、没有被手晃动（俯仰、横滚角速度都<8°/s）、轮速<30，持续约 0.3 s。
放开后约 1 s 内又触发，说明还在空中，那一次的放开要等约 0.5 s（原来是 1 s；2026-10-22 实测有一次在这 1 s 快到时手松开，机器人倒了）。它是输出端的安全保护，不是第二个控制器，也不是 PID 回退。

不用单轮速度而用平均值/半差，是因为侧向用力推时一个轮子会冲到 269 而另一个只有 70（平均 170），单轮规则只剩 10% 余量；
空转时两个轮子基本同速（538/545、−540/−530），平均值照样冲过 300。不对称的空转（一个轮 380、另一个 169）会晚几十毫秒触发。

开关：`!liftguard on|off|status`（运行时变量，复位后回到默认：开）。检测器一直在跑，结果写进 trace 的 `flags` 和 `!status` 的 `lift_guard`；
开关只决定要不要真的压轮。第一版（单轮速度、静止 1 s 起放开、重复触发翻倍）在 2026-10-22 的实测里把空转从“满速几秒”变成
“约 0.2 s 内冲到约 400 然后 0.3 s 内停下”，6 次触发没有一次误触发，但机器人被放回地面后要等 1–2 s 才恢复控制，那期间松手会倒；
这一版（v3）把放开时间缩短并改成“看起来像站在地板上”的判据；第三轮实测（`guard3-run1`，120 s）4 次触发都在峰值约 400 处切断、0.3 s 内停轮；两次“拿起马上放下”触发后在 1.4 s、1.9 s 恢复并站稳；另一次放开后 0.2 s 内又触发（还在空中）、随后在“又触发后的等待”里被松手倒下，因此再把这次等待缩到 0.5 s（v4）。

## 站立时前后踱步：轮子死区与“死区预加载”（2026-10-22，仿真验证，第二台机器人上做过一次有线对比）

现象：机器人站在平面上，有时自己前后小步移动。实机旧数据（(private test log, not published)）里能看到站稳后的“雕像”状态：
轮速恒为 0、俯仰几乎不动，但策略的共模指令停在约 ±46——正好贴着轮子死区的边缘（轮子在 |指令|≈40–50 才开始转）。

机理（仿真复现，`training/docs/REBUILD.zh-CN.md` §6）：倒立摆本身不稳定，指令落在死区里时轮子不出力，倾角会长到指令冲出死区为止；
冲出去时轮子被踢一下，过冲，反方向又是同样的过程。结果是一个确定性的自激慢周期：缓慢滑行约 6 s、一次快速踢回，±65 mm、周期约 7.5 s。
任何大于约 0.002 N·m 的轮子摩擦都会出现，幅度和摩擦成正比；传感器噪声、陀螺零偏、初始条件都不是原因（零噪声、从完全平衡的状态启动也一样）。
v6 对增益下降的下限裕度只有 ×0.51，死区让小幅度下的有效增益低于它，所以会自激。把位置/速度增益调高、加位置死区、静止保持逻辑、去掉位置积分都试过，
要么没用要么更糟（`training/docs/REBUILD.zh-CN.md` §6）。

做法（`src/policy/dead_zone_comp.h`，`Controller` 里调用，`tests/test_dead_zone_comp.cpp` 和 `tests/test_policy.cpp` 覆盖）：
在共模轮指令上加一个常量 `preload`，方向跟着网络当前推的方向（带迟滞 ±8 的符号，不是 sign()，所以噪声不会让它来回翻）。
关键是 `preload` 不是写死的数：预加载大于真实死区时轮子会 10–20 Hz 嗡嗡响（仿真里无摩擦时高频指令能量是原来的 30 倍），所以它在原地学：
* 只在“站着没人遥控、俯仰角速度和轮速都很小、而且网络正在推（|指令|>迟滞）”时才按 3 单位/秒增长——这正是死区挡住它的时候；站得很稳的机器人永远学不出预加载；
* 增长到 `max`（默认 55）为止；
* 符号翻转快于 8 次/秒（漏积分，τ=0.5 s）就是预加载过大在嗡嗡响：立刻乘 0.8、之后不再增长，仍在嗡嗡就继续乘 0.8；**只数网络指令摆幅小（0.5 s 峰值 <30）时的翻转**：被推或被手碰时机器人会带着指令大幅来回摆、同样快速翻符号，那不是嗡嗡响（第二台机器人上第一版没有这条，A/B 刚做完拔线碰机器人，两次误触发把学到的 15.5 砍到 10 并锁死；仿真里被推也会这样）；
* 停机、拿起保护、重新起立只清继电器状态，学到的值保留到断电；
* 加在共模上（两个轮子的差不变，偏航不受影响）；它是执行器一侧的偏置，不是第二个控制器，网络之外它唯一看的是“是否站稳”这个学习开关。

仿真结果（原生 C++ 控制器，`training/docs/REBUILD.zh-CN.md` §6）：轮子摩擦折合 25–70 单位、两个轮子不等（30/55、20/60、0/45）、
纯输入死区、静摩擦大于动摩擦几种模型下，站立 60 s 以后的位移峰峰值从 75–215 mm 降到 4–6 mm，速度均方根从 25–70 降到 4–5 mm/s，往返次数从每分钟约 16 次降到约 0；
无摩擦时完全不变（学不出预加载）。验收套件（16 个种子 × 60 s，8 个场景）存活数一样（角点 12/16 不变），3 Hz 以上指令能量最多 +2.6，速度跟踪略好。
代价是学到预加载以后轮指令里多了一个 ±K 的方波（K≈25–45，约 2–3 Hz 翻转，净力矩在死区内），指令的高频指标从 1 升到 10–30，但轮子几乎不动。
上机以后要看的就是这个：有没有听得见的哒哒声或嗡嗡声。

第一次实机对比（第二台机器人，有 USB 线，站在平面上没人碰，(private test log, not published)）：关闭时位移峰峰值 74 mm、15 s 内 9 个来回、速度 rms 29 mm/s；
打开并学完（约 15 s，学到约 16 单位）以后 8 mm、0.5 个来回、5 mm/s，俯仰标准差 0.80°→0.19°，3 Hz 以上指令能量 2.9→5.5，没有触发嗡嗡响保护。听感还没有人确认。

控制台：`!dzcomp`（状态）、`!dzcomp off|on`、`!dzcomp reset`（忘掉学到的值）、`!dzcomp MAX [HYST [RAMP]]`（MAX 0..80、HYST 2..40、RAMP 0..20）。
`!status` 里有 `dzcomp{max,hysteresis,ramp,preload,sign,flip_rate,trips,locked}`；trace 的 `flags` 第 3 位=预加载已学到且开着，第 4/5 位=继电器在正/负侧，
`cmd11 - a0` 就是实际加进去的预加载。`tools/standing_report.py` 把 trace 按 15 s 窗口算位移、步数、速度、高频能量和预加载；
死区预加载 v2（build 字段 `dead-zone preload v2`）= 上面这条摆幅门限，其余不变。
v3（`dead-zone preload v3`）：**不用每次开机重新学**——预加载有一个开机起始值（默认 12，这台机器人量出来的值），`!dzcomp reset` 回到它；往上涨只在“确实被卡住”时才发生：1 s 平均 |指令| ≥ 6（默认）。v2 是站着只要指令越过迟滞（8）就涨，一直涨到嗡嗡响再退，所以每次开机都要经过一次嗡嗡响；站得很稳的机器人 1 s 平均 |指令| 只有约 4–5（实机 v2 的雕像状态），不涨；还在踱步的（预加载太小）平均 |指令| 远大于 6，才涨。控制台 `!dzcomp MAX [HYST [RAMP [START [GROW]]]]`，`!status` 的 `dzcomp` 里有 `initial`、`grow_mean`。`tools/dzcomp_ab_test.py` 自动录“关闭/打开”两段并给出结论（`tools/capture_trace_usb.py --send "!dzcomp off"` 可以单独发设置命令）。

## 自由落体卸力（free-fall guard，2026-10-22，回放验证；2026-10-23 在第三台机器人上手托着下放测过，真实落下还没测）

起因（用户反馈）：机器人有时从桌子上摔下去，落地时舵机是有力的，齿轮直接断齿。现有的保护停机本来就会把轮子归零、把腿舵机卸力（`ActuatorCommandQueue::stop`），
但倾角停机要等落地后俯仰超过 45° 才触发，冲击时舵机还在使劲。

做法（`src/fall_guard.h`，纯头文件，`tests/test_fall_guard.cpp` 覆盖）：加速度计量的是比力，站着约 1 g、被拿起/被推 0.7–1.8 g、自由落体约 0 g。
**原始样本的总加速度低于 0.5 g 连续 60 ms（默认，可用 `!fallguard ms N` 改）就判定为下落**，立即按保护停机处理（轮归零、腿舵机卸力，停机原因 `free_fall`），然后沿用已有的自动恢复：
落地后（总加速度回到 0.8 g 以上 40 ms）直立、静止约 1.25 s 自动重新起立；下落期间 `outputGuard` 一直不通过，所以不会在半空重启。手拿起时加速度还在 1 g 附近，不触发。
80 cm 的桌子自由落体 0.4 s，检测后还剩约 0.34 s，20 cm（0.2 s）约 0.14 s，5 cm（0.1 s）约 0.04 s。

必须从**原始样本**喂：加速度计量程是 ±2 g（1 g = 16384 计数），现有 IMU 有效性判断（原始模长 >1000 计数，即 >0.06 g）只抓传感器死掉或正好 0 g，
而且它不通过时融合后的 `s.imu.acceleration` 不更新，所以 `sensor()` 里在这道门之前算总加速度。

阈值依据：2026-10-22 全部录像共 560 s（站立、推、拿起放下、遥控），最深的低谷 0.23 g 和 0.29 g，都只有一个 55 Hz 样本（≤约 36 ms），出现在接触瞬间（推、轮子落地），
没有任何地方连续两个样本低于 0.6 g；用固件自己的检测器回放（`tools/replay_fall_guard.py --all`）一共触发 0 次，最长的低于 0.5 g 的一段 20 ms。

2026-10-24 补充：真实松手（第三台机器人）显示，真正的自由落体读数会低于 0.061 g，被 IMU 有效性判断当成“数据无效”，立刻以 `imu_missing_stale_or_nonfinite` 停机（舵机卸了，但灯不变）；v6 把守卫改了以后，策略自己的新鲜度检查又抢先以 `policy_fault` 停机，v7 把这一道也对齐。
现在有第二个“深度”检测器（总加速度 <0.061 g 连续 20 ms，`deepConfig()`）把这种情形报成 `Fall`，IMU 在落体期间算有效；0.5 g/60 ms 的检测器管读数没那么低的落体（旋转、手拿着往下放）。
停机时的灯见 `release-notes/2026-10-22-to-24-dead-zone-fall-guard.zh-CN.md` v6 的表（倾倒红常亮、落体红快闪、其他故障红慢闪、命令停机琥珀色）。

开关：`!fallguard on|off|status`（运行时变量，复位后回到默认：开）；`!fallguard ms` 查询当前保持时间，`!fallguard ms N`（N=20..200，RAM 里，复位后回到 60）改保持时间，
`!status` 的 `fall_guard.freefall_ms` 显示当前值，`min_g`/`longest_low_ms`/`deep_trips` 是开机以来的统计（`!fallguard clear` 清零）；检测器一直在跑，写进 `!status` 的 `fall_guard{enabled,falling,low,trips,...}` 和 trace 的 `flags`
（第 6 位=这个样本低于限值，第 7 位=已判定下落）；停机事件行 `# ev` 里能看到 `reason=free_fall`。

保持时间为什么是 60 ms：最初是 80 ms。2026-10-23 在第三台机器人上手托着快速下放三次（(private test log, not published)，`tools/drop_report.py`），
加速度总量最低到 0.13 g，但连续低于 0.5 g 的时间只有约 60–75 ms，80 ms 一次都没触发。以前非下落的最长低谷是一个样本（≤约 36 ms），60 ms 还留约 24 ms 余量；
改成 40 ms 手摔会触发（测试用），但只剩几毫秒的余量，所以做成可调：测试时 `!fallguard ms 40`，平时用默认。落地前剩余时间 = 下落时间 − 保持时间，
对会伤舵机的高度（≥20 cm）差别不大，主要是 5–10 cm 的小摔。

落下时的姿态/旋转会不会让它漏掉（2026-10-24，仿真，`training/tools/fall_angle_probe.py`，`training/docs/REBUILD.zh-CN.md` §12；真机只测过近似直立的松手）：**姿态本身不影响**——判断用三轴合成的大小，自由落体时任何朝向都接近 0 g，
仿真里从手里以俯仰 −90°…90°、横滚 0°…180° 松手，396 次全部触发，离落地余量 ≥140 ms。**旋转会影响**：IMU 离整机重心约 40 mm（训练模型的估计，真机位置不知道），带着滚转掉下去时它读到 ω²r，
ω 约 11 rad/s（630°/s）就超过 0.5 g，主检测器（0.5 g/60 ms）和深检测器（0.061 g/6 ms）都看不到。仿真里斜着（约 40°）或倒着从桌沿冲出去时一个轮先掉、身体边掉边滚，约 10% 的桌沿跌落（912 次里 92 次）自由落体判断一次都没触发，
**这些全靠倾倒停机**（融合出的俯仰 >45° 或横滚 >60°，转这么快时落体后约 60–90 ms 触发）卸力；反过来，干净的直落时 |a|≈0，IMU 数据被门挡掉、融合角度不更新，倾倒停机看不到，靠自由落体判断。两层互补。
卸力生效（触发 +12 ms）到第一次碰地的余量，304 次跌落/每个桌高（4 个 IMU 位置 × 6 个种子 × 角度/速度/推）：75 cm 全部 ≥196 ms；30 cm 最差 40 ms（236 次 ≥100 ms）；15 cm（下落只有约 110 ms，检测加释放要 70–100 ms）有 17 次落地时仍有力、约一半余量 <50 ms。
试过用陀螺仪扣掉 ω×(ω×r)：自由落体判断不再失明，但与倾倒停机合起来的最差余量几乎不变，r 估错时（IMU 靠近重心）还会变差，所以没有改固件。

限制：没有真实掉落的数据（阈值来自“不会误触发”的录像、手托着的下放和物理）；卸力不保证不断齿——冲击时齿轮仍会被倒着带动，只是舵机不再主动对抗；
误触发的代价是它在你手里突然松力，然后自己站起来。

## 构建、验证与控制

在本目录运行：

```sh
python3 -B tools/run_host_tests.py
sh build.sh
```

构建使用本目录 `.venv` 的 PlatformIO 6.1.18，并复用 `~/.platformio` 工具链缓存。
默认应用/ELF/MAP、镜像校验和、源码摘要、模型信息输出到 `dist-rl-only/`。
显式 `sh build.sh diagnostic` 输出历史混合诊断版 `dist-rl/`。
`sh build.sh trained` 使用 `rider2-trained` 配置，输出 `dist-trained/`，策略启动仍为 off。
导出器从实际 ELF 中核对模型 ID、约定版本、训练标记和动作类型，避免仅凭头文件声明。
`dist/` 和原有实机证据保留为上一份交付；新的主机报告为 `evidence/rl_host_validation.json`。
测试包括生成 C++ 对 Python/PyTorch 的数值比较、异常/超时/保持/回绕、原有控制与协议回归。
没有 PyTorch 时会明确跳过 PyTorch 部分，仍检查纯 Python/C++ 数值。

新增控制命令：

```text
!stop
!policy shadow
!policy status
```

所有模式切换都要求显式手动停机。`shadow` 可在停机时运行，检查 `calls/max_us/error`。
安装并验证训练权重后，可先 `!stop`、`!policy active`，再单独 `!arm`。
选择 active 本身不会 arm。原固件在满足条件时可自动启动 PID 平衡；策略 off 不等于整机停机。
PC 工具对应 `python3 tools/rider_cli.py --port PORT policy shadow/status/active/off`，
也可使用已包含 pyserial 的 `.venv/bin/python`，将 `PORT` 换成实际端口。

二进制扩展仍使用原有帧封装：写 `0xb0` 的一个字节，0=off、1=shadow、2=active。
读 `0xb0` 返回当前模式；读 `0xb1` 返回 8 字节：模型 ID(uint32 LE)、动作类型(0 residual/1 wheels)、
trained、valid、约定版本。原有 45 字节遥测保持兼容。

新产物可用 `tools/flash_app.py --artifact-dir dist-rl --plan --esptool PATH` 离线核验。
本地训练和仿真结果见 `../../training/README.md` 与模型报告。
设备检查应分别记录固件身份、ESP32 推理耗时、实际输出来源与实机闭环结果。
构建/刷写成功和策略 active 均不能单独证明地面平衡已通过。
