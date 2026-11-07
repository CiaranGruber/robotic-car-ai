# 实车 System ID 测试计划（周二 29 Sep）

> **团队仓库说明（control/low-level-sysid）**：本文是在 GitLab `ai4r_policy` 仓库
> `feature/sysid-test-sequencer` 分支上做 sysID 的记录。文中的 sequencer、`sysid_*` / `speed_*` 参数和
> 安全保护都在那个分支的 `scripts/policy_node.py` 里，**本仓库没有移植 sequencer**。辨识出来的
> 速度控制器和转向映射已移植到 `policy/control/`，参数在 `config/ai4r_policy.yaml` 的 `control:` 下，
> 说明见 [policy/control/README.md](../../policy/control/README.md)。

面向 low-level control / system ID 小组（Chris、Yifei）。技术术语保留英文。
背景知识见 [code-walkthrough.md](code-walkthrough.md)，下文用 “WT §x” 引用其中章节。
标注“（推断）”的数字来自 reference-only 默认值或推算，**不是这台车的实测标定**。

测试 policy 在本地分支 `feature/sysid-test-sequencer`（未 push）。它只改了
`scripts/policy_node.py`（INSERT markers 之间的 sequencer + 6 个 `sysid_*` 参数）和
`config/ai4r_policy.yaml`。**这个分支不能合并回 `dev`**：YAML 的 active 设置改成了
timer mode，两个 release contract test（默认 config 必须是 `cone_detection`）会按设计失败。

---

## 0. Sequencer 怎么用

- 配置：`policy_update_mode: timer`，50 Hz，`required_sensors: [wheel_speed]`
  （WT §4.2 的 “Wheel-speed-controller tuning”）。IMU gyro 是 optional，只用于 `debug2`。
- 每个 segment i：保持 `(sysid_drive_sequence[i], sysid_steer_sequence[i])`
  `sysid_hold_s` 秒，然后发 0 `sysid_rest_s` 秒（设 `0.0` 就是直接阶跃到下一档）。
  最后一个 segment 之后一直发 0，直到你请求 state 2。
- 纯时间查表：用 `policy_elapsed_s` 定位 segment，没有 `sleep`、没有循环。
  `dt` 只用于累计里程，且有 `dt > 0.0` 保护（首步 `dt == 0`）。
- 安全：
  - drive 只允许 `[0, 0.3]`（代码里硬上限 `SYSID_DRIVE_LIMIT = 0.3`），不做倒车
    （wheel speed unsigned，无法保护倒车）。
  - wheel speed 超过 `sysid_max_speed_m_per_s`，或累计里程超过 `sysid_max_distance_m`，
    **latch 为 0 直到下次进入 state 3**。
  - 参数不合法（长度不一致、越界、写了 `0` 而不是 `0.0` 等）node 启动时直接报错退出。
    **每次重启 policy 后先看 log 有没有 `Policy update source: timer`**，再 Enable 车辆。
  - 这些只是软件兜底。真正的急停手段仍然是：Foxglove 请求 state 2、vehicle Disable、RC 接管
    （WT §6.8）。每次测试都要有一个人手在急停上。
- 输出：
  - `debug1` = 当前 segment 序号（hold 中）；`-1` = rest；`-2` = 序列结束或 guard 已触发。
  - `debug2` = `yaw_rate / wheel_speed`（1/m，左转为正），只在 gyro fresh 且速度 > 0.3 m/s 时发布。
  - 每次 phase 切换打一行 `sysid t=... phase=... drive=... steer=... dist=...` 到 `/rosout`，
    guard 触发时带 `STOP: 原因`。
- 换测试：把 YAML 里 6 行 active `sysid_*` 换成对应 preset（YAML 注释里有 A1/A2/B/C/D/E），
  然后 `dream runtime restart ai4r_policy`。**改 YAML 不需要 rebuild**（WT §4.2）；
  改 `policy_node.py` 需要 rebuild 一次（WT §5）。

### 上车准备（`<CAR_IP>`：今天用 8 号车 <CAR_IP>）（课程 System Project Docs 3–5）

1. 连 UniWireless + GlobalProtect，`ssh ai4r@<CAR_IP>`（PowerShell 即可）。
2. 启动 unit（不需要 `oakd_cone_detector`；IMU 只用于 `debug2`，C/D 需要）：
   ```bash
   dream runtime start foxglove_bridge
   dream runtime start traxxas_vehicle_interface
   dream runtime start bno08x_imu_interface
   dream runtime start ai4r_policy
   dream runtime status
   ```
3. 本机（本分支）：`scp -r scripts config ai4r@<CAR_IP>:~/ai4r_student_workspace/src/ai4r_policy/`
   车上：`dream build ros student` → `dream runtime restart ai4r_policy` →
   `dream runtime logs ai4r_policy --tail 100` 确认 `Policy update source: timer`。
   之后只改 YAML 时：scp YAML → restart 即可，不用 build。
4. Foxglove：`ws://<CAR_IP>:1234`，课程 layout。Main tab 第一列是 Vehicle / Policy 按钮，
   debug1/debug2 有现成面板；Wheel speed tab 有 speed 和 encoder period。

### 每次 run 的操作顺序

1. 改 YAML → scp → `dream runtime restart ai4r_policy` → 看 log 确认无报错。
   **policy restart 后车辆仍保持 Enabled**；挪车、摆车前先 Vehicle Request Disable。
2. 车放到起点，车头对准空旷方向，确认 wheel speed 有数据（静止为 0）、Policy state = Publishing zeros。
3. 开始录包（见 §0.1），口头报 run 编号并记录。
4. Vehicle Request Enable → 等 Vehicle state = Enabled（RC 在 manual 时 Enable 无效）。
5. Policy publishing actions（state 3）。
6. `debug1` 变 `-2` 且车停稳 → Policy publishing ZERO actions（state 2）→ Vehicle Request Disable → 停止录包。
   **先停 policy 再 Disable**，不要用 state 1（Not publishing）当停车按钮。
7. 重跑同一序列：Enable 后再按 Policy publishing actions（从 segment 0 重新开始，里程和 guard 重置）。

命令行等价（Foxglove 不可用时）：
```bash
ros2 topic pub --once /car/request std_msgs/msg/UInt8 '{data: 1}'   # Enable（0 = Disable）
ros2 topic echo --once /car/traxxas_state                           # 确认 Enabled
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'  # 运行（2 = 停）
```

### 0.1 Foxglove / rosbag 要看和要录的 topic

namespace 以车上实际为准（下面按 `/car` 写）：

| Topic | 用途 |
| --- | --- |
| `/car/drive_and_steer_set_point_normalized` | 命令（slew 之前，WT §6.3） |
| `/car/wheel_speed_m_per_sec` | 速度（unsigned，无 header） |
| `/car/imu/data` | yaw rate（`angular_velocity.z`，注意是 IMU sensor frame 的原始消息） |
| `/car/debug1`, `/car/debug2` | segment 序号、实时曲率 |
| `/car/imu_heading_angle` | 相对 heading（度），trim 测试用 |
| `/car/policy_fsm_state_string` | 是否被掉回 state 2 以及原因 |
| `/rosout` | `sysid ...` phase 日志 |

- Foxglove 布局：一个 Plot 面板画 `drive`、`wheel_speed`、`debug1`；另一个画 `debug2` 和
  `imu/data.angular_velocity.z`；一个 Log 面板；一个 Raw Messages 看 state string。
- 录包：优先在车上 `ros2 bag record -s mcap -o sysid_<run> <上面的 topics>`（按 DREAM runtime
  允许的方式执行）；如果不方便，至少用 Foxglove 导出 Plot 的 CSV。**数据以 bag 为准。**
- Foxglove 看到的是 debug1/debug2 的 policy-step 值（50 Hz）；wheel speed 本身约 50 Hz 遥测，
  但数值只在 encoder period 更新时才变（见测试 4）。

---

## 1. 时间预算（总计约 2 h 15 min）

| # | 内容 | 时间 |
| --- | --- | --- |
| 0 | 上车前：拉分支、build、numpy 检查、Foxglove 布局、量 wheelbase | 20 min |
| 5 | numpy import 检查（包含在 0 里） | 5 min |
| 4a | 推车测 wheel speed 低速极限（不上电机） | 10 min |
| 1b | Preset B：deadband 细台阶（同时是 4b 低速测量） | 20 min |
| 1a | Preset A1 / A2：drive → steady-state speed | 25 min |
| 3 | Preset E：speed step response | 15 min |
| 2 | Preset D + C：trim、steering → curvature | 30 min |
| — | 缓冲（电池、重跑、场地） | 15 min |

顺序理由：先不动车的检查，再从最慢的 B 开始，拿到 deadband 和速度量级后再放开 A2/E/C 的速度。
电池电压会影响 effort → speed，**记录每组 run 的电池情况**，重要的 run 满电和半电各做一次更好。

---

## 2. 测试 5：numpy 能否 import（上车第一件事）

- 目的：决定后续 policy 能不能用 `numpy`（WT §5）。
- 步骤（在运行 policy 的同一个环境里，source 过 ROS/student workspace 之后）：
  ```bash
  python3 -c "import sys, numpy; print(sys.executable, numpy.__version__)"
  python3 -c "import scipy; print(scipy.__version__)"   # 顺便看，可选
  head -1 ~/ai4r_student_workspace/install/ai4r_policy/lib/ai4r_policy/policy_node.py
  ```
  最后一行确认 shebang 是 `#!/usr/bin/env python3`，即和上面测的是同一个解释器
  （install 路径按实际 workspace 调整）。
- 记录：版本号或报错原文。**本分支没有 import numpy**，结果只作为后续开发依据。
- 如果能 import，后续 numpy 也只能放在文件顶部 import，不要在 policy step 里 import。
- **结果（29 Sep，8 号车）**：policy 进程用系统 `/usr/bin/python3`（PATH 无 venv），
  `numpy 1.26.4`，来自 `/usr/lib/python3/dist-packages`，可以 import。登录 shell 里的
  `python3` 是另一个 DREAM venv（也是 1.26.4），测试时要用系统 Python 才算数。

---

## 3. 测试 1：drive_action → steady-state speed，以及 deadband

### 1b. Deadband 细台阶（Preset B，YAML 当前 active）

- 测什么：车从静止开始动的最小 effort（breakaway，上升台阶），以及减小 effort 时停下的
  effort（下降台阶）。两者之差是静摩擦造成的 hysteresis。
- 序列：`0.04 → 0.12`（每档 +0.01，每档 3 s，rest 0）再 `0.11 → 0.04`，共 51 s。
  guard：1.5 m/s，15 m。
- 场地：直线 ≥ 15 m。如果太短，把序列拆成上升、下降两次 run。
- 同时人工观察：车轮第一次转动时 `debug1` 的值，以及车完全停下的档位（口头报，记录）。
- 如果 0.12 都不动：停止，把 B 的上限加到 0.16 再跑（仍低于代码上限 0.3）。
- 如果 0.06 就明显在跑：说明 deadband 很小，把 A1 的档位整体往下调。
- 做 2 次（满电 / 后段各一次更好）。

### 1a. Steady-state speed（Preset A1，然后 A2）

- 测什么：`drive_action` 各档的稳态速度 v_ss(u)。
- A1：`0.08, 0.10, 0.12, 0.14`，hold 4 s，rest 3 s，guard 2.0 m/s / 20 m。
- **看完 A1 的速度再决定 A2**：如果 0.14 已经超过 ~1.5 m/s，A2 就不要跑到 0.20，
  改成更小的档位并同步降低 guard。A2 默认 `0.16, 0.18, 0.20`，guard 3.0 m/s / 25 m。
- 每次 rest 车会减速（ESC 在 neutral 可能 coast 或 drag brake，顺便观察记录是哪一种）。
- 每组 2 次，往返方向各一次可以抵消坡度。

### 数据处理

1. 用 `debug1` 切 segment（值为 k 的时间段）。
2. 每个 segment 丢掉前 50%（过渡 + wheel speed 滤波滞后），对后 50% 的 wheel speed 取均值和
   标准差 → 一个点 (u_k, v_ss,k)。
3. 画 v_ss vs u。对 v_ss > 0 的点拟合 `v_ss = K (u − u0)`：K 是静态增益（m/s per unit effort），
   u0 是 deadband（x 截距）。如果明显非线性，分段线性或二次拟合，保留查表。
4. Preset B：上升台阶里第一个 v_ss > 0 的档位是 breakaway effort u_up，下降台阶里最后一个
   v_ss > 0 的档位后一档是 stop effort u_down。控制器 feedforward 建议取
   `u_ff = u0 + v_ref / K`，并对 u_up 做补偿（推断）。
5. 注意 B 中 1 cm/s 级别的速度会受测试 4 的测量极限影响，u_up 以“人工看到车轮转动”为准，
   wheel speed 只做参考。

---

## 4. 测试 2：steering_action → curvature，以及 trim

先量 wheelbase L（前后轴距，卷尺，记录到 mm）。

### D. Trim（先做）

- 测什么：让车直行所需的 `steering_action` 偏置 δ0。
- 序列：drive 用 1a 中约 1 m/s 的档位（preset 默认 0.12，按实测替换），steer
  `-0.04, 0.0, 0.04`，每段 4 s，rest 3 s，guard 1.8 m/s / 20 m。
- 同时看 `debug2`（曲率）和 `imu_heading_angle`（每段的 heading 变化）。
- 第一次 run 后，用 `debug2` 在 steer 上线性插值出曲率为 0 的 δ0，把序列中心改成
  `δ0 − 0.02, δ0, δ0 + 0.02` 再跑一次确认。
- 结论写成“policy 端 steering offset = δ0”。Foxglove 的 Set / Increment steering trim 的单位
  可能和 `steering_action` 不同（推断：policy 输出会缩放到标定区间，WT §3.3），不要直接填 δ0；
  用 Increment 小步调，每次用 steer 全 0 的序列复查 `debug2` ≈ 0。车是共用的，记下原 trim，测完调回。

### C. Curvature

- 测什么：`steering_action` → 稳态曲率 κ（1/m）的映射，以及左右是否对称、正方向是左还是右。
- 序列：drive 同 D，steer `0.25, -0.25, 0.5, -0.5, 0.75`，每段 8 s（足够绕大半圈），
  rest 2 s，guard 1.8 m/s / 60 m。
- 场地：至少 8 m × 8 m 空地（推断：小舵角转弯半径可能 3–5 m）。第一次 run 只跑前两段看半径，
  场地不够就减小 hold 或只做大舵角。
- 可选交叉验证：在某一段放一个锥桶标记起点，用卷尺量圆的直径。
- **不要跑 steer ±1.0**：输出会被缩放到标定区间（WT §3.3），满舵留到确认半径以后。

### 数据处理

1. 按 `debug1` 切段，取每段后 50%。
2. κ = r / v，r = `imu/data.angular_velocity.z`（原始 IMU frame；如果 IMU 安装不是 z 朝上，
   用 `debug2`，它已经转到 `base_link`），v = wheel speed；或直接平均 `debug2`。
3. 拟合 `κ = a (δ − δ0)`，得到 a 和 δ0；比较左右两侧斜率判断是否对称。
4. 转成前轮转角：`δ_wheel = atan(L κ)`（kinematic bicycle model）。
5. 从 steering slew 10 /s（WT §3.1）和每段开头的 κ 上升时间，估计 steering 响应时间常数。
6. 注意 wheel speed unsigned、只在前进时用；v 很小时 κ 噪声大（debug2 已限制 v > 0.3 m/s）。

---

## 5. 测试 3：speed step response

- 测什么：effort 阶跃下速度的时间常数 τ、纯延迟 td，以及升速/降速是否对称。
- 序列（Preset E）：`0.10 → 0.16 → 0.10 → 0.16`，每段 3 s，rest 0（直接阶跃），
  最后一段结束后阶跃到 0（coast-down）。guard 2.5 m/s / 25 m。
  档位按 1a 结果调整：两档都在 deadband 以上、速度在 0.5–1.5 m/s 之间最合适。
- 跑 3 次用于平均。

### 数据处理

1. 找到每个阶跃时刻（`debug1` 变化时刻 = 命令变化时刻；drive slew 50 /s 几乎无影响，WT §3.1）。
2. 拟合 FOPDT：`v(t) = v0 + Δv (1 − exp(−(t − t0 − td)/τ))`，得 τ、td、Δv。
3. 用 Δv / Δu 和 1a 的 K 对比，检查是否一致。
4. **td 里包含测量滞后**：wheel speed 是对 `encoder_time_filter_window = 6` 个 encoder period
   的平均（WT §3.5），低速时这部分滞后很大。用测试 4 的结果把测量滞后和车本身动态分开，
   或者用 IMU 的纵向 specific force（减去重力分量）做加速度的交叉检查。
5. 最后的 coast-down（→ 0）段给出无驱动的减速特性，也能看到停车时 wheel speed 多久归零。

---

## 6. 测试 4：wheel speed 测量的低速极限

理论估计（推断，基于 reference-only 默认值 `encoder_cycles_per_revolution: 3.0`、
`encoder_to_wheel_ratio: 2.72`、`wheel_diameter_m: 0.105`，且假设 cycle 计的是 motor 侧）：
车轮每转约 8.16 个 encoder cycle，每个 cycle 约 0.040 m。

| 速度 | encoder period | 6 个 period 的滤波窗口 |
| --- | --- | --- |
| 1.0 m/s | 0.04 s | 0.24 s |
| 0.3 m/s | 0.13 s | 0.8 s |
| 0.1 m/s | 0.40 s | 2.4 s |
| 0.05 m/s | 0.81 s | 4.9 s |
| < 0.013 m/s | > 3 s | 被 `encoder_timeout_seconds = 3.0` 判为 0 |

### 4a. 推车（车辆 Disabled，policy 在 state 2）

- 抬起驱动轮或在地上**慢慢**推车/转轮，尽量匀速，速度从极慢逐步加快；再从较快突然停住。
- 录 `wheel_speed_m_per_sec`。看：多慢时读数一直是 0；读数“台阶式”更新的间隔；停住后多久归零
  （预期接近 3 s）。
- 可以用卷尺 + 秒表粗略标定一次：推 2 m 用了多少秒，对比读数均值。

### 4b. 电机驱动（和 Preset B 同一批数据）

- 在 B 的每个低档位上，统计 wheel speed **数值发生变化**的平均间隔（去掉重复值），画
  “更新间隔 vs 速度”，和上表对比，得到实际的最低可用速度 v_min。
- 结论用于速度控制器：低于 v_min 时不要相信速度反馈（例如切换成 feedforward 或限制积分）。

---

## 7. 数据处理脚本骨架（离线，笔记本上）

bag 为 mcap 时可用 `rosbags` 或 Foxglove 导出 CSV。CSV 的处理思路：

```python
import pandas as pd
cmd = pd.read_csv("drive_and_steer.csv")   # columns: t, drive, steer
v = pd.read_csv("wheel_speed.csv")          # columns: t, data
seg = pd.read_csv("debug1.csv")             # columns: t, data
df = pd.merge_asof(v.sort_values("t"), seg.sort_values("t").rename(columns={"data": "seg"}), on="t")
df = pd.merge_asof(df, cmd.sort_values("t"), on="t")
rows = []
for k, g in df[df.seg >= 0].groupby("seg"):
    tail = g[g.t >= g.t.min() + 0.5 * (g.t.max() - g.t.min())]
    rows.append((k, tail.drive.iloc[0], tail.steer.iloc[0], tail.data.mean(), tail.data.std()))
print(pd.DataFrame(rows, columns=["seg", "drive", "steer", "v_mean", "v_std"]))
```

之后用 `numpy.polyfit` 拟合 K、u0，用 `scipy.optimize.curve_fit` 拟合 FOPDT。

---

## 8. 现场记录表（每个 run 一行）

| run | preset | 修改过的档位 | 电池 | 方向/地面 | 人工观察（何时起转、何时停） | guard 是否触发 | bag 文件名 |
| --- | --- | --- | --- | --- | --- | --- | --- |

测试完成后，把真实观察到的 command timing、停车行为补到 `docs/acceptance.md` 的 robot
checklist（目前均为 not run，WT §6.8），未观察到的保持 not run。

---

## 9. 实测记录（29 Sep，8 号车）

- 连接：UniWireless + GlobalProtect，`ssh ai4r@<CAR_IP>`，Foxglove `ws://<CAR_IP>:1234`。
  车上 runtime 比 v0.1.0 新：`sensor_timeout_s.lidar` 必须改名为 `sensor_timeout_s.lidar_scan`。
- `vehicle_control_source`（`std_msgs/Int8`）在 Disabled 和运行时都显示 2，含义未确认。
- Traxxas `rc_drive_min/center/max_us` = 1000/1500/2000（组件默认值）。
- Preset B（0.04–0.12）完全在 deadband 内；0.40–0.58 只有电机嗡嗡声，轮子不转。
  demonstrator 确认只是 effort 太低。代码上限因此从 0.3 提到 0.7。
- 架空 0.60→0.70 台阶（两次，都被 15 m 里程保护在下降段停止），按每档结束时的累计里程估算平均速度：

| drive | run 1 | run 2 |
| --- | --- | --- |
| 0.62 | 不转 | ≈0.6 m/s（起转） |
| 0.64 | 不转 | ≈0.8 |
| 0.66 | ≈0.8（起转） | ≈1.0 |
| 0.68 | ≈1.2 | ≈1.3 |
| 0.70 | ≈1.4 | ≈1.4 |

  架空起转点 0.62–0.66（静摩擦，每次不同）；起转后约 10 m/s 每单位 drive，线性截距约 0.55。
  这是架空结果，地面负载下起转点会更高，后续 A/E/C/D 档位要按地面结果重新定。
- 地面直线（sequencer 0.64–0.70，再 0.70–0.80，每档 3 s，rest 3 s）：到 0.68 不动；
  **applied ≈ 0.771（命令 0.78）时才勉强起步**。地面起转点比架空（0.62–0.66）高约 0.12。
- Foxglove 导出的 applied 与命令关系约为 `applied ≈ 1.055 × command − 0.050`
  （0.60→0.583，0.70→0.686，0.80→0.794），建模用 applied 作为输入。
- 代码上限按 Chris 的要求从 0.7 再提到 0.85。
- 地面 A2（0.78–0.84，hold 3 s，rest 3 s）的 Foxglove CSV（wheel speed + encoder period，22 s）：
  - 3 s 内速度仍在上升（0.25→0.34、0.25→0.53、0.42→0.63 m/s），**hold 至少 6 s**。
  - **停车后 wheel speed 冻结在最后一个值（≈0.3 m/s）约 3 s**，encoder period 随后跳到 1–3 s 再归零。
    log 里 rest 段里程增长是测量假象。速度控制器要考虑低速/刚停车时最长约 3 s 的读数滞后。
  - Foxglove 导出只包含当前窗口的数据；完整数据以 rosbag 为准。
  - 与 log 对齐后，各 hold 结束时的地面速度（未到稳态）：0.78→≈0.34，0.80→≈0.53，0.82→≈0.63 m/s；
    0.84 段只有 ≈0.43 且读数冻结（疑似车停住/卡住，待确认）。前三档约每 +0.02 drive 加 0.15 m/s，
    粗略截距 ≈0.73。
- **rosbag `sysid_A_run3`**（地面，命令 0.80/0.82/0.84，applied 0.789/0.811/0.830，hold 6 s，rest 4 s）：

| 命令 | applied | 各秒峰值速度 (m/s) | 末 2 s 均值 |
| --- | --- | --- | --- |
| 0.80 | 0.789 | 0.16 0.38 0.45 0.48 0.51 0.57 | 0.52（仍在上升） |
| 0.82 | 0.811 | 0.21 0.48 0.63 0.69 → 13.5 s 撞击后停住 | 无效 |
| 0.84 | 0.830 | 0.15 0.45 0.60 0.68 0.75 0.78 | 0.74（接近稳态） |

  - 0.82 段在第 3.5 s：IMU 水平加速度尖峰 3–5 m/s²、yaw rate −0.7 rad/s，之后 IMU 完全静止且
    specific force x 从 ≈−0.25 变为 ≈−0.9（车身倾斜约 4°），applied 仍为 0.811：判断为车撞到/卡在障碍物上。
  - 上升时间常数粗估 τ ≈ 2 s（含测量滞后）；6 s 仍未完全稳态，稳态测试建议 hold ≥ 8 s。
  - drive 归零后首次读到 0 需 3.2–4.0 s（滑行 + encoder timeout）。
  - applied 在命令变化时经过中间值（如 0.246），与 50 /s drive slew 一致。
  - 粗略地面稳态：≈5 m/s 每单位 applied，速度为 0 的截距 ≈0.68–0.70（未完全稳态，仅供参考）。

### 单档 8 s 地面测试（rosbag run4–run6，无碰撞、无保护触发）

| run | 命令 | applied | 每秒峰值速度 (m/s) | 稳态 v（末 3 s） | 开始动 | 到 63% | drive→0 后读到 0 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 4 | 0.78 | 0.771 | 0.10 0.22 0.33 0.39 0.46 0.53 0.53 0.52 | 0.513 | 0.89 s | 2.97 s | 3.43 s |
| 5 | 0.80 | 0.794 | 0.10 0.34 0.47 0.56 0.59 0.60 0.61 0.61 | 0.592 | 0.62 s | 2.22 s | — |
| 6 | 0.82 | 0.811 | 0.18 0.46 0.60 0.69 0.72 0.73 0.74 0.78 | 0.737 | 0.72 s | 2.06 s | 3.91 s |

- 线性拟合（命令单位）：`v_ss ≈ 5.6 × (u − 0.69)` m/s；用 applied：`v_ss ≈ 5.5 × (applied − 0.68)`。
  0.80→0.82 的增量（0.15）大于 0.78→0.80（0.08），有一定非线性，只适用于约 0.5–0.75 m/s。
- 一阶加纯延迟粗估：纯延迟 td ≈ 0.6–0.9 s（起步静摩擦 + 测量滞后），τ ≈ 1.4–2.1 s。
- 停车后读数归零约 3.4–3.9 s。
- 直线段末 3 s 平均 yaw rate 0.03–0.045 rad/s（原始 IMU z，符号未确认），即曲率约 0.05–0.06 1/m，
  半径约 17–20 m：steer 0 时有轻微偏转，可作为 trim 测试的起点。
- 速度控制器建议（推断）：feedforward `u_ff = 0.69 + v_ref / 5.6`，再加慢速 PI；低于约 0.5 m/s 不可靠。

### 一次一档 0.80–1.00（rosbag `sysid_C_all`，hold 5 s，11 档，无保护触发）

`python tools/sysid_analyze.py sysid_C_all/sysid_C_all_0.mcap` 输出：

| drive | applied | v_ss（末 3 s） | t63 | 曲率 (1/m) |
| --- | --- | --- | --- | --- |
| 0.80 | 0.794 | 0.385 | 1.80 s | −0.063 |
| 0.82 | 0.811 | 0.547 | 1.81 s | −0.089 |
| 0.84 | 0.834 | 0.514 | 1.86 s | −0.092 |
| 0.86 | 0.851 | 0.683 | 1.66 s | −0.087 |
| 0.88 | 0.874 | 0.718 | 1.59 s | −0.082 |
| 0.90 | 0.897 | 0.813 | 1.67 s | −0.060 |
| 0.92 | 0.914 | 0.842 | 1.61 s | −0.067 |
| 0.94 | 0.937 | 0.946 | 1.60 s | −0.066 |
| 0.96 | 0.960 | 1.090 | 1.53 s | −0.074 |
| 0.98 | 0.977 | 1.078 | 1.50 s | −0.067 |
| 1.00 | 1.000 | 1.230 | 1.50 s | −0.080 |

- 拟合：**`v_ss ≈ 3.97 × (drive − 0.697)` m/s**，0.4–1.2 m/s 范围内基本线性（0.80→预测 0.41，1.00→1.20）。
  runs 4–6 的 5.6 斜率来自很窄的范围且 8 s hold，这组覆盖更宽，优先用这组。
- 低档（0.80–0.84）5 s 未完全稳态，v_ss 偏低（run5 的 8 s 结果 0.80→0.59）。
- 从命令到 63% 约 1.5–1.8 s（含起步延迟与测量滞后），高档更快。
- steer 0 时曲率一直约 −0.06 到 −0.09 1/m（向右，半径约 11–16 m），与速度无关：是固定的 steering 偏置，需做 trim。
- "moved" 列在后几档为 0，是掉头时手推车产生的读数，不代表起步时间。
- 推断的初始控制器：feedforward `drive = 0.70 + v_ref / 4.0`；P 增益按 SIMC 粗估 Kp ≈ 0.3–0.4 drive/(m/s)。

### 速度控制器首测（换电池后，rosbag `sysid_D_kp02`，FF + P，Kp = 0.2）

| 段 | 目标 | 末段速度 | drive |
| --- | --- | --- | --- |
| 0–4 s | 0.6 m/s | ≈0.27 | 0.97→0.92 |
| 4–8 s | 0.9 m/s | ≈0.58 | 1.00（饱和） |
| 8–12 s | 0.6 m/s | ≈0.40 | 0.87→0.90 |

- 换电池并重启后，同样 drive 的速度只有之前的约 1/2–1/3（0.92→0.27 vs 0.84；1.0→0.58 vs 1.23）。
  怀疑 ESC profile（Training 模式）、电池或地面变化，待确认。
- P 方向正确（速度不足时 drive 上升直至饱和），但模型变化时仅 P 的稳态误差很大：需要小积分项或每次开机重新标定 feedforward。

### PI 速度控制（同一块低电量电池，rosbag `sysid_D_pi015`，Kp = 0.2，Ki = 0.15）

| 段 | 目标 | P only（sysid_D_kp02） | PI（本次） | PI 的 drive |
| --- | --- | --- | --- | --- |
| 0–4 s | 0.6 | 0.27 | 0.53（仍在上升） | ≈1.00（饱和） |
| 4–8 s | 0.9 | 0.58 | 0.71 | 1.00（饱和，物理上限） |
| 8–12 s | 0.6 | 0.40 | 0.64→0.59（收敛到目标附近） | 0.96–0.97 |

- PI 明显减小了稳态误差：第三段在 drive ≈0.96 时稳定在 0.6 附近。
- 前两段 drive 一直顶在 1.0：这块电池下速度受执行器上限限制，不是控制器的问题。
- 两次之间电池状态不完全相同（drive 1.0 时 0.58 vs 0.71），对比有误差。
- 更正：13:08 的断线是主板充电宝没电，**电机电池全程未更换**。同一电机电池下 drive = 1.0 的速度：
  12:47 → 1.23 m/s，13:25 → 0.58 m/s，13:36 → 0.71 m/s。速度下降主要是电机电池在测试过程中放电（推断，待测电压确认）。
  结论：电机电池电量对 drive→速度影响很大，固定 feedforward 不足，需要 PI。

### PI，满电电机电池（rosbag `sysid_D_pi015_full`）

- 0.6 目标：峰值 0.79，4 s 末 0.79（超调约 30%，feedforward 对满电偏大）；0.9 目标：1.01–1.03。
- 7.67 s 撞击（IMU 水平冲击 8 m/s²、yaw rate 1.7 rad/s），之后车卡住，wheel speed 冻结在 0.35，
  PI 把 drive 推到 1.0 持续约 3.5 s。
- 改进（已实现并用全部录包离线验证）：
  1. **stall guard**：开始移动后 wheel speed 数值 0.6 s 不变且 drive > 0 即锁定为 0。
     所有正常录包中移动时单值最长只保持 0.29 s，不会误触发；两次碰撞（run3、本次）都能触发。
  2. **积分带** `speed_i_band`：|误差| 小于该值才积分，减小加速阶段的超调。
  3. 仿真比较后取 `speed_ff_offset: 0.66`、`speed_i_band: 0.5`：满电峰值 0.88→0.73，低电量仍能跟到 0.6 附近。

### PI v2（满电，rosbag `sysid_D_pi015_v2`，ff_offset 0.66，i_band 0.5，含 stall guard）

| 段 | 目标 | 结果 |
| --- | --- | --- |
| 0–4 s | 0.6 | 平滑上升到 0.61–0.62，**无超调** |
| 4–8 s | 0.9 | 1.01–1.04（超调约 15%，drive 正在下调） |
| 8–12 s | 0.6 | 1.02→0.66 正常下降；9.69 s 撞击（4.4 m/s²），之后 0.17–0.24 m/s 缓慢顶推，drive 升到 0.96 |

- 撞击后车轮仍缓慢转动，读数在变，stall guard 未触发（按设计只抓“完全停住”）。
- 新增 **impact guard**：水平 specific force 在启动 0.3 s 后超过 3.5 m/s² 即锁定为 0。
  所有录包中正常启动峰值 ≤ 2.8 m/s²（都在启动后 0.15 s 内），三次碰撞峰值 4.4 / 6.7 / 8.6 m/s²。
- 本次约 6.5 m 处撞到障碍物：`sysid_max_distance_m` 应设为实际可用直道长度减 1.5 m。

### PI v3（满电，路线清空，rosbag `sysid_D_pi015_v3`）：第一次完整无碰撞跑完

| 段 | 目标 | 过程 | 段末速度 | 段末误差 |
| --- | --- | --- | --- | --- |
| 0–4 s | 0.6 | 平滑上升，无超调，约 1.3 s 到 63% | 0.59–0.62 | ≈0 |
| 4–8 s | 0.9 | 峰值 1.07（超调 19%），drive 0.98→0.91 缓慢下调 | 1.05 | +0.15 |
| 8–12 s | 0.6 | 1.0→0.55 平滑下降，轻微下冲 | 0.55–0.57 | −0.04 |

- stall / impact guard 均未误触发；总里程 7.9 m。
- 0.9 m/s 段的超调来自 feedforward 对满电偏大（满电 u0≈0.62，FF 用 0.66），4 s 内积分来不及修正。
- 下一步可选：hold 延长到 6 s 看稳态；或 Ki 提到约 0.25；或满电时 ff_offset 用 0.63。
