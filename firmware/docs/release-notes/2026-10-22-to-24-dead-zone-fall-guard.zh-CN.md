# Rider2 RL-only 固件（死区预加载版）发布说明（2026-10-22，已刷入第二台机器人）

> 注：这是作者的工程笔记，按原工作区的目录写成。公开仓库里：`simulation/` 就是 `training/`，`rider/rider2-firmware/` 就是 `firmware/`，`sh build.sh` 的产物在 `dist/`（正文里的 `dist-rl-only*/` 是历史构建目录），`logs/` 里的测试记录和 `evidence/` 没有公开。


这一版比 `2026-10-22-v6-first-hardware-tests.zh-CN.md` 里的 a5e01f76… 只多一件事：站在平面上前后踱步的修复（“死区预加载”），
其余（拿起保护、舵机掉线自动恢复、USB trace 等）一模一样。2026-10-22 已刷入第二台机器人（写入后读回校验一致，`!status` 的 build 字段和 `dzcomp` 默认值都对），
并做了第一次实机对比（下面）。

## 第一次实机对比（2026-10-22，USB 线接着，机器人站在平面上没人碰；数据在 (private test log, not published)）

| | 补偿关（75 s） | 补偿开，学完以后（t≥40 s） |
| --- | --- | --- |
| 轮子位移峰峰值（15 s 窗口的中位数） | 74 mm | 8 mm（最稳的 30–60 s：1.7 mm） |
| 15 s 内往返步数（>5 mm） | 9 | 0.5（最稳的窗口 0） |
| 地面速度 rms | 29 mm/s | 5 mm/s |
| 俯仰标准差 | 0.80° | 0.19° |
| 共模轮指令 3 Hz 以上 rms | 2.9 | 5.5 |

* 预加载约 15 s 内学到约 16 单位（旧机器人的死区约 45，仿真里预计学到 25–45）并保持不变（±15.5，翻转约 2 次/秒，没有触发嗡嗡响保护）：
  这台新机器人的轮子比旧的顺得多，真实死区大概只有 15–25 单位。“关”这段的踱步（±25–35 mm，约每 3 s 一个来回）也和仿真里摩擦 15–25 单位的结果吻合。
* “开”这段的最后 15 s 里有三次外力碰撞（偏航角速度突然变大：约 78 s、93–95 s、101–103 s，位移 5–40 mm），应该是碰到线或机器人；它每次都自己回到静止。
* 局限：线有拉力（两段都有）、只有一次、105 s、一台机器人、一种地面；**听感**（有没有哒哒声）数据里看不到，要人耳判断。

## v8（已刷入第三台机器人，2026-10-24）：落体时 IMU 融合不再被单个低 g 读数重启，中途的“startup”停机也有灯

v7 刷入后做的第三次真摔（(private test log, not published)）：采集里两次松手都按设计停机，原因 **`free_fall`**，分别在第一个低于 0.5 g 的样本之后 **26 ms 和 34 ms**；倾倒那次是 `tilt`（俯仰 44° 时，|a| 约 0.9 g）。
统计（`!fallguard clear` 之后）：`trips` 3、`deep_trips` 3、最低 0.019 g、最长一次失重 212 ms（采集结束后你又摔了一次，到 268 s 时 `trips` 4、最长 222 ms）。
**采集结束后的那一次摔，停机原因却记成了 `startup`（模式 252，灯不亮）**：落体时第二个低 g 读数（<0.061 g）让 `sensor()` 在下一次读取时把 IMU 融合整个重启（`s.imu={}`），"已校准"标志变假，
`outputGuard` 里 `Startup` 判断在落体检测器还没数满 6 ms 之前就返回了，于是先以 `Startup` 停机。两次采集里的摔恰好是落体检测器先数满，这一次 IMU 读取间隔略大一点就输了。

改了什么：

* **融合重启的条件**（`rider2::fall::imuFusionRestart`）：上一次读取“只是读到约 0 g”（芯片有回应）时，不再因此重启融合，除非已经超过 30 ms 没有有效读取（真正的落体）。单个低 g 读数不会清掉姿态估计，也就不会引出 `Startup` 停机。
* **中途的 startup 当作 IMU 停机**（`stopReasonWhileDriving`）：机器人正在驱动时，`Startup` 只可能是 IMU 融合重启了（"已校准"没了），改记成 `Imu`，灯红色慢闪，原因看得到；刚上电等待起立时的 `Startup` 不变。
* 测试：`test_fall_guard` 74 项（含融合重启的真值表），`test_runtime_guard` 49 项。
* 镜像：`dist-rl-only-fallguard-v5-20261024/firmware.bin`，1,242,080 字节，sha256 `b9ed509ac0a7832a6afdb7f7445802ea607b8d7f11f420fbcf793a62231631c6`，build 字段 `... free-fall guard 60 ms + low-g = fall (v3) + LED fault codes ... 2026-10-24`。
* 刷入（(private test log, not published)，先核对 MAC，只写 app0，摘要校验一致；刷之前机器人已停机、轮指令 0、腿卸力，所以没要求托着）后的 `!status`：build 字符串带 `(v3)`，腿零点 590/416，四个舵机反馈 2–20 ms，策略 active，电池 82%，IMU 读取间隔 2458/2999/3492 µs。
  刚开机时 `imu_calibrated` 是 false：IMU 校准（陀螺零偏）只在 |横滚|<4° 时累计（`rider_leg_imu.cpp`），机器人趴着的姿态横滚 7.7°，所以托直立、静止约 1 秒才会校准完，不是新版本的问题。
* 你的观察：前面几次摔（v7）灯是红色快闪，和 `free_fall` 对得上。
* **v8 上机验证（第四次真摔，(private test log, not published)）**：两次松手都以 `free_fall` 停机，在第一个低于 0.5 g 的样本之后 **22 ms 和 26 ms**（停机时最近的样本低到 0.12 g 和 0.08 g）；
  两次倾倒停机（`tilt`），一次向前（俯仰 +44.5°）、一次向后（−44.9°，向后倒的动态过程也越过了 45°）。统计（`!fallguard clear` 之后）`trips` 2、`deep_trips` 2、最低 0.032 g、最长一次失重 198 ms（约 19 cm 高），
  触发次数和 `free_fall` 停机次数对得上，没有被漏掉或记成别的原因的摔；整段里驱动中没有出现 `startup`、`policy_fault`、`imu_missing_stale_or_nonfinite` 停机。驱动时 `imu_timing`：间隔 2001/3353/6002 µs，读取加融合最长 1827 µs，读失败 0。

## v7（已刷入第三台机器人，2026-10-24）：落体停机的原因对齐成 `free_fall`

v6 刷入后做的第二次真摔（(private test log, not published)，两次松手加一次倾倒）：两次松手都在第一个低于 0.5 g 的样本之后 **38 ms 和 27 ms** 停机卸力了，
但停机原因是 **`policy_fault`**，不是 `free_fall`（灯会是红色慢闪，不是快闪）。原因：`runPolicy` 里策略自己的“IMU 数据新不新鲜”检查（`state.fresh`，IMU 无效就失败，
`Controller::fail(Error::State)` → `stop(StopReason::Policy)`）在落体判断之前触发。v6 只改了守卫（`outputGuard`），没改这一道。开机清零后的统计：`trips` 2、`deep_trips` 1、`min_g` 0.027、`longest_low_ms` 223（最长一次失重 223 ms，约 24 cm 高）。
真摔那次的 IMU 读取（驱动时）：533948 次读取，间隔最小/平均/最大 2454/3095/6004 µs，一次读取加融合最长 1909 µs，读失败 0；控制周期 4.0/5.9/7.0 ms。IMU 不慢，最坏 6 ms。

改了什么：

* 落体检测器看到低 g 读数（<0.061 g）时，策略的新鲜度检查也算 IMU 有效（`fall_guard_enabled` 时），这几毫秒用最后一次姿态；之后由落体检测器停机，原因 `free_fall`，灯红色快闪，落地站稳后自动恢复。
* 深度检测器的保持时间 20 ms 改成 6 ms（连续两次读取，IMU 读取间隔约 3 ms）：以前 IMU 一读到低于 0.061 g 就立刻停（没有任何去抖），现在要两次，单个坏读数不会停机，延迟只多约 3 ms。
* 镜像：`dist-rl-only-fallguard-v4-20261024/firmware.bin`，1,241,952 字节，sha256 `477082aedd9285f3e08edfdcb2f6777ca326441c4713d51d747b3213d9e588f9`，build 字段 `... free-fall guard 60 ms + low-g = fall (v2) + LED fault codes ... 2026-10-24`。
* 刷入（(private test log, not published)，先核对 MAC，只写 app0，摘要校验一致）后的第一次 `!status`（`status-after-flash-v4.json`）：build 字符串带 `(v2)`，IMU 有效已校准，腿零点 590/416，四个舵机反馈 2–20 ms，策略 active，`fall_guard` 保持时间 60 ms，电池 64%；`imu_timing`（等待时）间隔 2462/2999/3504 µs，读取加融合最长 1370 µs。

## v6（已刷入第三台机器人，2026-10-24）：真的落体不再是“IMU 故障”，卸力有明确的灯，状态里能看到 IMU 读取和落体的统计

起因：你摔了几次，有的灯红有的不红，还觉得有一次没卸力。第三台机器人上做的真实松手（(private test log, not published)）说明：
**真实自由落体时加速度计读数低于 0.061 g（1000 计数）**，正好是固件里 IMU 有效性判断（`norm>1000000`）认为“数据无效”的地方，于是 IMU 被判无效，**立刻（这次第一个低于 0.061 g 的读数，比跌破 0.5 g 晚约 30 ms）以 `imu_missing_stale_or_nonfinite` 停机**：舵机其实卸力了（`drv=0`，和倾倒停机走同一个 `bus.stop(true)`），但这个停机原因落在模式 252，**灯不更新**（原固件的灯只处理模式 0、250、1），
（更正：`Imu` 其实在自动恢复的名单里，前几次录像里它停机后也自己重启了。）证据：松手那次，加速度总量 1.0 → 0.86 → 0.69 → 0.43 → 0.18 g（约 50 ms），停机事件比第一个低于 0.5 g 的样本晚 32 ms，原因是 IMU；之后落体检测器才数满（`fall_guard.trips` 1），那时机器人早已停了。
倾倒（自己歪倒，|a| 一直在 0.5–0.8 g，转动让总加速度不低）要等俯仰超过 45° 才停机、灯才红（红灯是落地后的倾倒停机）；向后倒停在约 −36°、向前倒腿收着停在约 32°（都是仿真的静止姿态，真机还没测；腿伸着向前倒真机读到 46.8°，超过 45°，会停），到不了 45° 的**根本不停机、不卸力、不红灯**。
**“IMU 读取慢导致判断滞后”不成立**：IMU 400 kHz I2C，控制循环中位数 5.0 ms，加速度每 15 ms 一个样本都在变（没有重复样本），卸力在失重开始后 32 ms 就发生了；问题是原因编码和灯。

改了什么：

* **低 g 的 IMU 不再当故障**：第二个“深度”检测器（`deepConfig()`，总加速度 <0.061 g 连续 20 ms）把这种落体报成 `Fall`（模式 0，灯红，原因 `free_fall`，落地后自动恢复）；落体期间 IMU 算有效，不会抢先报 `Imu`。真正的 IMU 故障（读失败、数据空白、重复样本、>30 ms 没有有效样本）仍然是 `Imu` 停机。`!fallguard off` 时回到以前的行为。
* **卸力的灯**（`src/board_support.cpp` 的 `renderFaultLeds`，`src/led_status.h` 的 `ledFaultFor`；只在停机且没在平衡时画，平衡中和低电量闪烁不变）：

  | 灯 | 含义 |
  | --- | --- |
  | 红色常亮 | 倾倒停机（`tilt`），舵机已卸力 |
  | 红色快闪（5 Hz） | 落体停机（`free_fall`），舵机已卸力 |
  | 红色慢闪（1 Hz） | 其他保护停机：IMU/舵机反馈丢失、策略或控制出错、校准无效等，舵机已卸力 |
  | 琥珀色常亮 | 被命令停的（手机/USB/开关的停止），舵机已卸力 |
  | 平时的颜色 | 正在平衡（舵机有力），或刚上电还没出过故障 |

  灯跟着“上次停机原因”走，一直保持到机器人重新开始平衡为止（所以重新起立前的等待阶段仍是红色，舵机仍是松的）。原来的 `renderLeds` 没动（和原固件逐指令对照的测试不受影响）。
* **统计**：`!status` 的 `fall_guard` 多了 `deep_trips`、`min_g`（开机以来最低的加速度总量）、`longest_low_ms`（低于 0.5 g 的最长连续时间），新增 `imu_timing`（读取间隔最小/平均/最大 µs、一次读取+融合最长用时、读失败/空白/重复样本数）；`!fallguard clear` 清零这些统计和触发计数；`tools/incident_snapshot.py` 的摘要也会打印。
* 镜像：`dist-rl-only-fallguard-v3-20261024/firmware.bin`，1,241,920 字节，sha256 `2ab45fbeaa7ca31e697b0e89e2a0201ba907ed56bf06fc9388c9bec5a52f7e28`，build 字段 `... | free-fall guard 60 ms + low-g = fall + LED fault codes | ... | 2026-10-24`。

刷入（(private test log, not published)，先核对 MAC xx:xx:xx:xx:xx:xx，只写 app0，写入后校验摘要一致）后的第一次 `!status`（`status-after-flash-v3.json`）：build 字符串正确，IMU 有效且已校准，腿零点 590/416 有效，四个舵机反馈 2–11 ms，策略 active，`fall_guard` 保持时间 60 ms，
`imu_timing`（机器人在等待起立、没有驱动时读的）：16399 次读取，间隔最小/平均/最大 2454/2999/3484 µs（约 333 Hz），一次读取加融合最长 1360 µs，读失败/空白/重复样本都是 0。驱动时控制循环更忙，间隔会更大，要在驱动一轮后再读一次。

没处理的：**倾倒停在 45° 以内的摔法（向后 −36°、向前腿收着 32°）仍然不停机**。为什么没直接把倾倒限降到 30°：764 s 的驾驶记录里，手拿着倾斜时 |俯仰|>30° 出现 9 次、最长 2.2 s，>40° 也有 1.2 s，降低限值会让手里的机器人也被卸力。
要分开手拿着和躺在地上，得再加一个条件（比如腿负载或车轮落地），还没做。

## v5（已构建，**未刷入**，2026-10-23）：落体保持时间默认 60 ms，可用 `!fallguard ms N` 调

第三台机器人的手托下放（见 v4 一节最后）显示连续低于 0.5 g 只有约 60–75 ms，80 ms 一次都不触发。保持时间默认改成 60 ms（以前非下落的最长低谷 ≤约 36 ms，仍留约 24 ms 余量），并加控制台 `!fallguard ms`（查询）和 `!fallguard ms N`（N=20..200，只在 RAM 里，复位后回到 60）；`!status` 的 `fall_guard.freefall_ms` 显示当前值，`!fallguard status` 也带上保持时间。测试手摔时用 `!fallguard ms 40`，平时不用动。

* 镜像：`dist-rl-only-fallguard-v2-20261023/firmware.bin`，1,240,560 字节，sha256 `8558b42e138eb8674aa8a7900412af4e9dd76281379c0f86cdb015764662b2be`，build 字段 `... | free-fall guard 60 ms (!fallguard ms N) | servo-dropout auto re-arm | 2026-10-23`。
* 解析和范围检查在 `src/fall_guard.h`（`validHoldMs`、`parseHoldCommand`），`tests/test_fall_guard.cpp` 55 项覆盖（范围边界、NaN/inf、多余文字、查询不改值、40 ms 和 60 ms 各自触发/不触发的下落长度）。`tools/replay_fall_guard.py` 的默认保持时间也改成 60 ms。
* 手摔测试：`tools/capture_trace_usb.py OUT.jsonl --port ... --arm --cue --protocol drop --send "!fallguard ms 40"`（`--send` 现在也接受 `!fallguard ...`），录完用 `python3 tools/drop_report.py OUT.jsonl` 出报告；真松手落体测试同样的录法。

## v4（已刷入第三台机器人，2026-10-23）：在 v3 上加自由落体卸力

从桌上摔下来时舵机有力、冲击断齿：加速度计原始总加速度 <0.5 g 连续 80 ms 就判定下落，立即停机（轮归零、腿舵机卸力，原因 `free_fall`），落地后直立静止自动恢复，手拿起不触发。
细节、阈值依据和限制：`RL_POLICY.md` “自由落体卸力”。全部 560 s 录像用固件自己的检测器回放 0 次触发（最深低谷 0.23 g，只有一个样本）。

* 镜像：`dist-rl-only-fallguard-20261022/firmware.bin`，1,239,856 字节，sha256 `ffc9cacf837c9b022a95bd2019722da78630b56fcda7d79523987a6b5fb9aec0`，build 字段 `... | dead-zone preload v3 | free-fall guard | ...`。
* 控制台 `!fallguard on|off|status`；`!status` 的 `fall_guard`；trace `flags` 第 6/7 位。
* **第三台机器人（2026-10-23，MAC xx:xx:xx:xx:xx:xx，ESP32-D0WD v1.1，4 MB）**：板子上原来是厂家固件（app0，ESP-IDF v4.4.2，2022-09-15 构建，1,106,128 字节），
  刷之前整片 4 MB 备份在 (private test log, not published)（`full-flash-before.bin` sha256 `38c2d545…2fff`，`app0-before.bin`，`calibration-gap-before.bin`）。
  只写 app0（`tools/flash_app.py --flash`，写入 76 s，校验通过）；刷后复读：引导、分区表、OTA 数据都没变，腿校准区 0x310000–0x32ffff 逐字节相同（零点 590/416），
  NVS 里原有 31 个键都在，只多了首次启动写入的射频校准（`cal_data`、`cal_version`）。首次启动 `!status`（(private test log, not published)）：
  固件和 build 字符串正确，IMU 有效且已校准，策略 active、0 次失败；**四个舵机一个字节都没回**（`bus_rx_bytes` 0，`bus_timeouts` 随时间增长），电池 6.77 V / 10%，
  `last_stop_reason=actuator_feedback_missing_or_stale`（舵机一回应、直立静止就会自动恢复）——多半是机器人电源开关没开或电池太空，开电源/充电后再读一次。
  开电源后四个舵机都有反馈，`bus_timeouts` 不再增长。
* **第三台机器人的第一次落体试验（2026-10-23，手托着快速往下放，不松手；(private test log, not published)，`tools/drop_report.py` 出报告）**：
  85 s、5581 个样本、没有丢样本。三次快速下放里加速度总量最低到 0.13 g、0.24 g、0.65 g，但连续低于 0.5 g 的时间只有约 60–75 ms（触发要连续 80 ms），
  所以**一次都没触发**（trace 的 bit7 从没置位，没有 `free_fall` 停机事件，`!status` 的 `fall_guard.trips`=0）；慢慢往下放、托稳、拿起（|a| 最低 0.59–0.79 g）也没有误触发。
  用 `tools/replay_fall_guard.py` 回放同一份记录会报 2 次触发，但那是偏保守的估计（每个 15 ms 的样本按 18 ms 算），实机检测器看到的是每个控制 tick，没有触发才是准的。
  要看到真触发需要真正松手落下（≥10 cm，下落 ≥0.14 s），落在软垫上；手托着的下放持续不够久。
* **真机验证（需要你配合）**：拔线，在软的地方（折起的毛巾/床上）上方 15–20 cm 处让站着的机器人自己落下去（别拿手抓着跌），然后插线读 `!status`：`fall_guard.trips` 应为 1，`last_stop_reason` 是 `free_fall`，
  它落地后站直不动应自动重新起立。拿起放下、轻推多次，`trips` 不应增加。

## v3（已构建，**未刷入**）：开机就带起始值，不用每次重新学

起因：v1/v2 每次开机预加载从 0 开始，往上涨到嗡嗡响再退回来（这台机器人上 24→19→15→12，约 1 秒的嗡嗡响，然后锁定），所以每次开机都要“重新学”。这台机器人量出来的值是 12–15.5，直接写成开机起始值（`initial` 12）；往上涨改成只在确实被卡住时才发生（1 s 平均 |指令| ≥ 6，`grow_mean`）；嗡嗡响保护照旧（摆幅门限、×0.8、锁定）当兜底。

仿真（`simulation/tools/standing_dead_zone.py` 同一批摩擦 12–45 单位，120 s、60 s 之后）：起始 12 + 门限 6 时，摩擦 12–15（这台机器人的范围）预加载停在 12.5–13.5、触发 0 次（v2 是 1 次嗡嗡响链）、往返次数 0.5–4/分（v2 是 12–25/分）；摩擦 20–45 预加载涨到 18–39，位移 5–7 mm、速度 4–5 mm/s，和 v2 一样好，没有开机嗡嗡响。验收套件存活数不变，速度跟踪更好（0.055→0.049、0.057→0.045），3 Hz 以上指令能量 +1.3–3.4（无摩擦的仿真机器人开机时 12 的预加载会嗡嗡响一下，实机的死区不会为 0）。
门限太高（≥12）会让摩擦 20–35 的机器人停在偏小的预加载上继续小步（往返 20–33/分），所以取 6。

* 镜像：`dist-rl-only-dzcomp-v3-20261022/firmware.bin`，1,238,512 字节，sha256 `979026586e45cf3b2e81a4b5a2c9aaace1c4151b955fc89f3d5a97de8c176ee1`，build 字段 `... | dead-zone preload v3 | ...`。
* 刷入后怎么验证：不用等它学。起立后马上 `!status`：`dzcomp.preload` 应该一开始就是 12、`trips` 0；站一会儿、拿起放下、推几下以后还是 12 左右、`trips` 仍是 0。

## v2（已刷入第二台机器人）：嗡嗡响保护不再被推/手碰误触发

实机发现（上面那次对比之后读 `!status`）：学到的预加载从 15.5 掉到 9.99、`trips` 3、`locked`。第一次是学习时真的嗡嗡响（K≈19.4 → 15.5），
后两次是拔线、碰机器人时机器人被动了一下，指令来回大幅摆动，保护把它当成嗡嗡响，各砍 20% 并锁死。v2 只在网络指令摆幅（0.5 s 峰值）<30 时才累计符号翻转；
推和手碰的摆幅大，不数；真正的嗡嗡响摆幅小，照样抓。仿真里同样的推不再触发，站立表和验收套件结果不变。其它一模一样。

* 镜像：`dist-rl-only-dzcomp-v2-20261022/firmware.bin`，1,237,584 字节，sha256 `43af7a3204d0684507b876a88fbed6e7b5dfbea255e7dce5996aec3faf19c2ef`，
  `!status` 的 build 字段 `... | dead-zone preload v2 | ...`。机器人上现在是下面第一版（`dist-rl-only-dzcomp-20261022`，sha256 4e9c5777…）。
* **实机验证（2026-10-22）**：刷入后无线站立、拿起放下 3–4 次、推几下，再插线读状态：站得很稳（最近 15 s 位移峰峰值 0.7 mm、速度 rms 1.7 mm/s、俯仰 1.75–2.25°），预加载 12.28、嗡嗡响保护触发 3 次（学习时连续乘 0.8 的结果，24→19→15→12）、已锁定。接着再做一轮（拿起放下 4 次、轻推 4 次，线接着，最后一下机器人倒了，停机原因 tilt）：预加载仍是 12.28、触发仍是 3 次，没有被碰撞砍小——第一版在同样的碰撞里被砍了 36%。数据 (private test log, not published)、`v2-handling-snapshot`。
* 带 USB 线时每次放下机器人会被线带着走一点（用户的感觉，线的拉力和轮子摩擦力矩同量级）；对比实验尽量无线，状态用 `!status` 读。
* 刷入后怎么验证：起立后站稳一会儿让它学（`!status` 的 `dzcomp.preload` 约 15，`trips` 最多 1），然后拿起放下几次、推几下，再读 `!status`：`preload` 应该还是约 15，`trips` 不再增加。

## 镜像（第一版，已刷入）

* `dist-rl-only-dzcomp-20261022/firmware.bin`（同 `dist-rl-only/`），1,237,616 字节，
  sha256 `4e9c5777a8c376c57000a988fbb74d308d0e9de797ebfd0c6bf93ec1ca1d8360`；
  `!status` 的 `build` 字段：`rider2-rl-only | rider_balance_v6 | lift guard v4 | dead-zone preload | servo-dropout auto re-arm | 2026-10-22`。
* 模型还是 `rider_balance_v6`（权重没动）。静态内存 112,292 字节（上一版 112,188）。
* 刷入（只写应用分区，托住机器人，刷完它重启）：
  `tools/flash_app.py --port /dev/cu.usbserial-XXXX --artifact-dir dist-rl-only-dzcomp-20261022 --flash --esptool ~/.platformio/packages/tool-esptoolpy/esptool.py`
* 回退：同一条命令把 `--artifact-dir` 换成 `dist-rl-only-release-20261022`（a5e01f76…）。

## 它解决什么

机器人站着有时自己前后小步移动。原因是轮子在 |指令|≈45 以内不动（死区/摩擦），而倒立摆不稳定：指令落在死区里时轮子不出力，
倾角一直长到指令冲出死区，再被踢一下、过冲、反方向重复。仿真里是一个确定性的慢周期（缓慢滑行约 6 s、快速踢回，±65 mm，周期约 7.5 s），
旧机器人的实机数据里有它的前兆（站稳时指令停在约 ±46、轮速为 0）。详细证据、试过没用的办法和数字：`../../../training/docs/REBUILD.zh-CN.md` §6；
机制和参数：`RL_POLICY.md` “站立时前后踱步”一节。

## 它做什么

在共模轮指令上加一个常量预加载，方向跟着网络当前推的方向（迟滞 ±8），大小在原地学：
只在“站着没人遥控、网络在推、轮子慢”时每秒长 3 个单位，上限 55；一旦符号来回翻得太快（>8 次/秒，说明预加载大过真实死区、轮子在嗡嗡响）
就乘 0.8 并停止增长。所以它会停在比真实死区略小的地方；站得很稳的机器人（指令没越过 ±8）永远学不出预加载。
学到的值保留到断电；停机、拿起保护、重新起立不清它。它是执行器一侧的偏置，不是第二个控制器，也不是 PID。

仿真（原生 C++ 控制器）：摩擦折合 25–70 单位的各种模型下，站立位移峰峰值从 75–215 mm 降到 4–6 mm，速度从 25–70 降到 4–5 mm/s，往返次数从约 16/分降到约 0；
无摩擦时完全不变；8 个验收场景存活数不变，4 分钟带推带遥控的长测 48 次没有一次倒。

## 在机器人上怎么看有没有用

1. 刷入，托直立让它起来，放在平面上，**不要碰它**（手机遥控保持在中位）。开机后最多 15–40 s 它在学，这段时间仍可能踱步。
2. 一条命令自动对比（USB 接着，不复位板子；用系统 `python3`，需要 numpy 和 scipy，固件目录的 `.venv/bin/python` 没有它们）：
   `python3 tools/dzcomp_ab_test.py --port /dev/cu.usbserial-XXXX --cue`
   先录约 75 s “补偿关”，再录约 105 s “补偿开”（前 40 s 算学习期），给出位移峰峰值、步数、速度、3 Hz 以上指令能量，并给结论。
   数据存在 (private test log, not published)<时间>/`；单独分析任何 trace：`python3 tools/standing_report.py TRACE.jsonl`。USB 线本身会拉机器人（拉力和轮子摩擦力矩同量级），想看不接线的结果，拔线站几分钟，再插回读 `!status` 和取 trace。
3. 凭耳朵和眼睛：要留意**有没有轮子的哒哒声或嗡嗡声**（学到预加载后指令里有一个 ±K 的方波，约 2–3 Hz 翻转，净力矩在死区内，仿真里轮子几乎不动）。

## 控制台

`!dzcomp`（状态）、`!dzcomp off`（关，学到的值保留）、`!dzcomp on`（默认值）、`!dzcomp reset`（忘掉学到的值）、
`!dzcomp MAX [HYST [RAMP]]`（MAX 0..80，HYST 2..40，RAMP 0..20 单位/秒）。设置只在运行时有效，复位后回到默认（开，55/8/3）。
`!status` 的 `dzcomp` 块：`max/hysteresis/ramp`、学到的 `preload`、继电器 `sign`、翻转率 `flip_rate`、嗡嗡响触发次数 `trips`、`locked`。
trace 的 `flags`：第 3 位=已学到且开着，第 4/5 位=继电器在正/负侧；`cmd11 - a0` 就是实际加进去的预加载。
`tools/capture_trace_usb.py --send "!dzcomp off"` 可以在录之前发设置命令（只接受 `!dzcomp`/`!liftguard`）。

## 已知限制 / 没验证的

* **只做过一次有线对比。** 实机的死区可能有一部分是静摩擦（站稳时指令停在 46 而轮子不动，说明静摩擦大于 46），仿真里静摩擦大于动摩擦的情形也试了，效果相近，但不等于实机。
* 嗡嗡响的判据（翻转 >8 次/秒）是在仿真里调的；实机噪声更大时可能误触发，每次误触发只让预加载少 20%。
* 学到以后轮指令的高频指标会从约 1 升到 10–30（继电器方波）；如果上机能听到哒哒声，`!dzcomp off`，把 `dzcomp_ab_test.py` 的数据发给我。
* 一直被遥控的机器人学不到预加载（学习只在安静站立时进行）；开机后前 15–40 s 还没学完。
* 其余限制同 `2026-10-22-v6-first-hardware-tests.zh-CN.md`。
