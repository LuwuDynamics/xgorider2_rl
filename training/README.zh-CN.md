# training/ — 复现并继续训练

MuJoCo 仿真（**固件 C++ 在仿真环里运行**）、从实机 trace 做系统辨识、带延迟的线性教师、带角点采样的 PPO、检查点权重平均、导出成 ESP32 头文件、验收套件。只用 CPU。[English](README.md) · [流程详解](docs/PIPELINE.zh-CN.md)（[English](docs/PIPELINE.md)）· [安全须知](../SAFETY.md)

```
train2.py  evaluate2.py  export_actor2.py  verify_export2.py  viewer2.py     流程本体（contract 6）
run.sh  setup.sh  requirements.txt                                          入口 / 固定版本的依赖
rider_sim/     env2.py（仿真器）、plant.py（辨识出的轮驱）、firmware.py + native_policy.py（C++ 在环）、calibration.py（质量）…
native/        把 ../firmware/src 编进仿真器的 C++ 桥
models/        rider_measured.xml（MuJoCo 模型）、measured_frf.json（实机频响）、robust_linear_gains_i.json（教师）、mass_audit/calibrated_distribution.json
models/cad/    机器人的 CAD 显示网格和按它绘制的模型（CC BY-NC 4.0：仅限非商业使用，可选）
tools/         sysid_frf.py、sim_frf_check.py、design_linear.py、loop_margin.py、average_checkpoints.py，以及各种试验（台阶、腿波、推搡、量化、摔倒与起身…）
data/real_robot_traces/   系统辨识用的实机数值 trace
artifacts/balance-v6/     发布的模型：policy_weights.h、actor.json、policy.zip、配置、评估报告
artifacts/balance-legs-v5/actor.json    之前那版在实机上共振的策略（只用于前后对比）
tests/         24 个单元测试（没有 `models/cad/` 时其中 2 个自动跳过）
docs/          PIPELINE.zh-CN.md / PIPELINE.md（先看这个）、REBUILD.zh-CN.md（工程日志）、MODEL.zh-CN.md（较早的模型说明）
```

`training/` 和 `firmware/` 必须是同级目录：仿真器第一次使用时会把 `../firmware/src`（IMU 滤波、腿控制器、舵机总线解码、策略控制器）编译成动态库（`build/*.dylib`，需要 `clang++`），所以仿真里的机器人跑的就是真机上的那份代码。

## 环境

**当前在 macOS 上开发和训练，Linux 已验证可以运行；Windows 建议使用 WSL2。** 所有平台训练都只用 CPU，无需 CUDA。

| 平台 | 环境与验证范围 |
| --- | --- |
| macOS | Apple 芯片、Python 3.11；完整发布配方已运行。原生 C++ 桥需要 `clang++`（可由 Xcode Command Line Tools 提供）。 |
| Linux | Debian 12 / ARM64 / Python 3.11 的 Docker 环境已验证：依赖安装、24 个训练测试、发布策略验证与评估、32,768 步 PPO、续训至累计 40,960 步、导出与新策略验证、虚拟显示及离屏渲染。完整 400 万步配方尚未在 Linux 复跑。需要 `clang++`；图形显示或渲染还需对应的 OpenGL / GLFW / EGL 或 OSMesa 系统库。 |
| Windows | 建议在 WSL2 的 Linux 环境中安装 Python 3.11 和 `clang++`，执行下方同一套命令；WSL2 与原生 Windows 尚未实测。交互窗口需要可用的图形显示环境（如 WSLg）。 |

Windows 用户应将整个仓库克隆到 WSL2 的 Linux 文件系统（例如 `~/xgorider2_rl`），并在其中创建虚拟环境和编译原生桥。跨系统时请重新创建 `.venv` 与 `build/` 编译缓存，不要复用 macOS 或 Windows 的环境。保留 `training/` 与 `firmware/` 的同级关系，训练和数值评估可以在无窗口环境中完成。

先准备好 Python 3.11（确认 `python3 --version`）、虚拟环境支持及上述系统依赖，再从 `training/` 执行：

```sh
sh setup.sh          # python3 -m venv .venv && pip install -r requirements.txt   （mujoco 3.16、torch 2.9 CPU、stable-baselines3 2.9 …）
```

版本是精确固定的，换 CPU、系统或依赖版本，训练结果可能略有不同。下面都用 `.venv/bin/python`。也可复制[仓库首页的智能体提示词](../README.zh-CN.md#复制给智能体复现与训练)，让智能体完成配置、验证、训练与导出。

## 看看发布的策略

```sh
sh run.sh view                    # 交互窗口：W/↑ 前进，S/↓ 后退，A/← 左转，D/→ 右转，X 直行，空格停车，R 重置，T 随机指令，鼠标 = 视角
sh run.sh view --pushes --domain 1.0    # 随机化的机器人 + 随机推搡
sh run.sh verify                  # 导出的权重、float32 C++ 推理、调度、观测和积分 == 参考实现（最大命令误差约 1e-5 单位）
sh run.sh evaluate --seeds 8      # 原生 C++ 策略的验收套件（所有场景；--scenarios stand drive corner … 可选）
sh run.sh margin                  # artifacts/balance-v6/actor.json 在**实测**频响上的稳定裕度
```

窗口里的机器人按 CAD 网格绘制（`models/cad/`）：`rider_sim/env2.py` 在渲染时加载 `models/cad/rider_measured_cad.xml`，其它时候（包括删掉 `models/cad/` 之后）用 `models/rider_measured.xml`。两者的刚体、质量、惯量、关节和（简单的）碰撞几何完全相同，训练和评估看不到网格；有单元测试检查这一点。**`models/cad/` 仅限非商业使用**（CC BY-NC 4.0，见 `models/cad/LICENSE`），它是可选的，只有摔倒/起身探针和两个测试会用到。

要精确复现发布的表（`artifacts/balance-v6/RESULTS.md`），用它的种子集和回合长度：

```sh
.venv/bin/python evaluate2.py --policy native artifacts/balance-v6/policy_weights.h --seeds 32 --seed-start 61000 --seconds 20 \
    --scenarios stand stand_push drive stress_gain stress_delay stress_lag stress_sensor stress_weak hard_push noise5 corner
```

（默认种子集 51000… 在较弱的场景里会差几个回合：角点 24/32、大推力 30/32、轮速比例 0.4 为 30/32——弱的地方对种子敏感。）

`evaluate2.py` 每个场景输出：存活数、`hf_rms`（3 Hz 以上共同轮命令的均方根——手感上的“抖”）、命令标准差与总变差、俯仰/俯仰角速度标准差、速度与偏航跟踪误差、末位置误差。

## 自己训练

训练发布的那个模型用的配方。耗时（Apple M4 MacBook Air，10 核，纯 CPU，含评估）：400 万步的复现跑了 8 分 24 秒；原来那次 500 万步的日志是 925 秒。

```sh
.venv/bin/python train2.py --output runs/v6 --envs 5 --steps 4000000 \
    --eval-every 500000 --eval-seeds 8 --eval-workers 5 --corner-fraction 0.5 --seed 7171
```

阶段（`train2.py` 会逐个打印）：

1. 在随机化机器人上用线性教师（`robust_linear_gains_i.json`）加探索噪声采集约 24 万个样本，**行为克隆**进 18-32-32-4 的 actor；
2. **critic 预热**：actor 冻结 30 万步，只让价值函数学习；
3. 在 5 个并行的随机化环境里跑 **PPO**（一个只站立，四个带随机遥控指令和推搡）；一半的回合是*角点*回合，所有随机参数都取区间端点；奖励 = 存活 + 直立 + 速度/偏航/位置跟踪 − 命令变化率、命令二阶差分、机体角速度惩罚；
4. 每 50 万步用**验收分**给检查点打分（六个场景上的存活 + 抖动 + 跟踪）；行为克隆的起点先打分，PPO 只有赢过它才值得保留。

选检查点、导出、验证（发布的模型就是这样做出来的）：

```sh
.venv/bin/python tools/average_checkpoints.py runs/v6/step-{1500000,2000000,2500000,3000000,3500000}.zip --output runs/v6/avg.zip
.venv/bin/python export_actor2.py runs/v6/avg.zip --output artifacts/mine --name rider_balance_v6 --trained-steps 3500000
.venv/bin/python verify_export2.py artifacts/mine
.venv/bin/python evaluate2.py --policy native artifacts/mine/policy_weights.h --seeds 32 --seconds 20
.venv/bin/python tools/loop_margin.py artifacts/mine/actor.json --old artifacts/balance-legs-v5/actor.json
```

故意平均 150 万–350 万步的检查点：分数在 350 万步附近见顶，**之后会塌**（原始训练里 400 万步 66、450 万步 43、500 万步 29），所以最好的单个检查点不是该发布的那个。平均之后的权重是否接受，由 `evaluate2.py` / `loop_margin.py` 决定。

**可复现性：** 在作者的机器上（Apple M4，PyTorch 2.9.1，NumPy 2.4.6）运行上面的命令以及平均/导出步骤，得到的 `artifacts/balance-v6/policy_weights.h` 与发布的那份逐字节相同（SHA-256 `3929ac158858e75f86a6792ea803f2b00ddb5dc6ae4d47d9a9ca9a11dcb22576`）；每个检查点的评估分数与原始训练相差 ≤ 0.1。换 CPU 或库版本，最后几位可能漂移，权重也就不同——请比较评估数字，不要比哈希。

把结果放进固件：[../firmware/README.zh-CN.md](../firmware/README.zh-CN.md)。

## 继续训练

```sh
.venv/bin/python train2.py --output runs/more --resume artifacts/balance-v6/policy.zip --steps 1000000 --reset-sigma 0.02 --resume-warmup
```

`--resume-warmup` 会重新预热 critic（改了奖励时用）。检查点 `.zip` 是 PyTorch/pickle 格式：**只加载你信任的文件。**

## 用你自己的机器人重新辨识轮驱

仿真好不好，全看轮驱模型。`tools/sysid_frf.py` 把平衡时的 trace（`firmware/tools/capture_trace*.py` 的 55 Hz trace）变成 `models/measured_frf.json`；`tools/sim_frf_check.py` 检查仿真能否复现它（幅值 9 % 以内、相位 9° 以内）；`tools/design_linear.py` 在它上面重新设计线性教师：

```sh
.venv/bin/python tools/sysid_frf.py --logs data/real_robot_traces/*.jsonl      # 或者你自己的 trace
.venv/bin/python tools/sim_frf_check.py
.venv/bin/python tools/design_linear.py --integral --speed-tau 0.12 --drift-weight 0.1 --zeta-min 0.25 --samples 12000 --output models/robust_linear_gains_i.json
```

细节以及为什么这很重要的证据：[docs/PIPELINE.zh-CN.md](docs/PIPELINE.zh-CN.md)。

## 能改什么

| 想做的事 | 去哪里 |
| --- | --- |
| 改奖励 | `rider_sim/env2.py` 的 `RiderEnv2.reward_cfg` 和 `step()`；权重是 `train2.py --rate-penalty --jerk-penalty --body-rate-penalty --still-v --still-x` |
| 改随机化 | `rider_sim/plant.py`（轮驱）、`rider_sim/calibration.py`（质量）、`RiderEnv2.reset()`（噪声、延迟、摩擦、地面倾斜、推搡） |
| 训练腿 | `train2.py` 里腿的两路输出被屏蔽为 0（`mask = [1, 1, 0, 0]`），`export_actor2.py` 导出时这两行为 0；固件已经会用它们（`action_space = wheels_legs`） |
| 改观测 | contract 6 固定在三处，必须一致：`rider_sim/env2.py::_observation`、`firmware/src/policy/policy.cpp`、`export_actor2.py`（并把 `kContractVersion` 加一；权重不匹配时构建会拒绝） |
| 研究摔倒和起身 | `tools/getup_probe.py`、`tools/fall_angle_probe.py`（通过 `tools/shell_model.py` 把 `models/cad/` 里机身和腿的 CAD 网格当碰撞外壳）；结果见 `docs/REBUILD.zh-CN.md` 第 10、12 节 |
| 快速试想法 | `tools/step_probe*.py`（小台阶）、`tools/fun_probes.py`（腿波、推搡）、`tools/standing_dead_zone.py`、`tools/quant_demo.py`（int8/int4 权重会怎样） |

## 测试

```sh
.venv/bin/python -m unittest discover -s tests -p 'test_*.py'      # 24 个测试（没有 models/cad/ 时 2 个跳过）
```

## 局限

仿真里没有静摩擦、齿轮间隙和软地面；轮子阻抗和总线延迟只靠随机化覆盖。腿没有训练。最弱的评估场景（“角点”：所有随机参数取极端）存活 23 / 32。仿真分数不是硬件验证——见 [../SAFETY.md](../SAFETY.md)。
