# 每台车的标定流程（速度 + trim + 转向）

> **团队仓库说明（control/low-level-sysid）**：本文是在 GitLab `ai4r_policy` 仓库
> `feature/sysid-test-sequencer` 分支上做 sysID 的记录。文中的 sequencer、`sysid_*` / `speed_*` 参数和
> 安全保护都在那个分支的 `scripts/policy_node.py` 里，**本仓库没有移植 sequencer**。辨识出来的
> 速度控制器和转向映射已移植到 `policy/control/`，参数在 `config/ai4r_policy.yaml` 的 `control:` 下，
> 说明见 [policy/control/README.md](../../policy/control/README.md)。

适用于本地分支 `feature/sysid-test-sequencer`。所有测试都用同一份代码，**只改
`config/ai4r_policy.yaml` 里的几行**。技术术语保留英文。背景和 8 号车的实测结果见
[sysid-test-plan.md](sysid-test-plan.md)。

## 代码在哪

| 文件 | 内容 |
| --- | --- |
| `scripts/policy_node.py` | 测试 sequencer（INSERT POLICY CODE markers 之间）+ 速度控制器（feedforward + PI）+ 安全保护 |
| `config/ai4r_policy.yaml` | 所有测试参数（`sysid_*`、`speed_*`），注释里有各个 preset |
| `tools/sysid_analyze.py` | 在电脑上分析录包，输出每档速度、曲率和拟合结果，不需要 ROS |

安全保护（代码里固定，每次运行都生效）：
- drive 只能向前，最大 1.0；
- 速度超过 `sysid_max_speed_m_per_s`、或里程超过 `sysid_max_distance_m` → 停；
- **stall**：车动起来后，wheel speed 读数 0.6 s 不变 → 停（车被卡住）；
- **impact**：起步 0.3 s 后，IMU 水平加速度 > 3.5 m/s² → 停（撞到东西）。

停下后 debug1 = −2，log 里有 `STOP: 原因`。

---

## 0. 准备（每次上车，约 15 分钟）

**需要**：满电的电机电池、充满的主板充电宝、遥控器、卷尺、至少 8 m 的直道（清空障碍物）、
6 × 6 m 的空地（转向测试用）。量一下直道真正可用的长度 D。

三个 PowerShell 窗口：
- **A**：本机，在 GitLab `ai4r_policy` 仓库（`feature/sysid-test-sequencer` 分支）的文件夹里
- **B**：`ssh ai4r@<CAR_IP>`，用来控制
- **C**：本机 PowerShell（同样在 ai4r_policy 文件夹），只用来运行录包脚本

**B：备份车上原来的代码**（车是共用的）
```bash
rm -rf ~/ai4r_policy_backup && cp -r ~/ai4r_student_workspace/src/ai4r_policy ~/ai4r_policy_backup
```

**A：拷代码**（确认 `git branch` 是 `feature/sysid-test-sequencer`）
```powershell
scp -r scripts config ai4r@<CAR_IP>:ai4r_student_workspace/src/ai4r_policy/
```

**B：build + 启动**
```bash
dream build ros student
dream runtime start foxglove_bridge
dream runtime start traxxas_vehicle_interface
dream runtime start bno08x_imu_interface
dream runtime stop ai4r_policy
dream runtime start ai4r_policy
sleep 5
dream runtime status
```
四个 unit 都要是 `running`。若报 `Rename sensor_timeout_s.lidar ...`，YAML 已经用了新名字 `lidar_scan`，确认拷的是这个分支。

**Foxglove**：`ws://<CAR_IP>:1234`，课程 layout。确认 Disarmed、Zero actions、RC 状态 OK。ESC 已开（LED 亮）。

### 每次改完 YAML 的固定动作
```powershell
# A
scp config/ai4r_policy.yaml ai4r@<CAR_IP>:ai4r_student_workspace/src/ai4r_policy/config/
```
```bash
# B
dream runtime stop ai4r_policy
dream runtime start ai4r_policy
sleep 5
ros2 param get /car/ai4r_policy sysid_drive_sequence
```
只改 YAML **不用 build**。只有改了 `policy_node.py` 才要先 `dream build ros student`。

### 每一轮的固定动作（录包一条命令搞定）
1. **窗口 C 改为本机 PowerShell**（在 ai4r_policy 文件夹里），每个测试运行一次：
   ```powershell
   powershell -ExecutionPolicy Bypass -File tools\record_run.ps1 -Car <CAR_IP> -Name cal_speed_8
   ```
   它会登录车上开始录包，看到 `=== Recording ... press Enter to stop ===` 就可以开始跑。
2. Foxglove：**Traxxas Request Enable** → 等 Enabled → **Policy publishing actions**
3. 车跑完自动停（debug1 = −2）→ **Policy publishing ZERO actions** → **Traxxas Request Disable**
4. 一次一档模式：车掉头摆好，回到第 2 步跑下一档（录包一直在录）
5. 整个测试结束：在窗口 C **按回车**。脚本会正确关闭录包、拷回到 `bags\` 文件夹、并自动运行分析脚本打印结果。

录包名自动加时间戳，不会重名。设置 SSH key 后全程不用输密码：
```powershell
ssh-keygen -t ed25519            # 一路回车
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh ai4r@<CAR_IP> "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
```
（每台车做一次；课程的 Optional SSH Key Setup 页面也有说明。）

---

## 1. 开环速度标定（约 15 分钟）

**目的**：得到这台车、这块电池下的 `speed_ff_offset` 和 `speed_ff_gain`。

YAML（替换对应行，`sysid_max_distance_m` 用 D − 1.5）：
```yaml
    speed_control: false
    sysid_one_segment_per_run: true
    sysid_drive_sequence: [0.80, 0.82, 0.84, 0.86, 0.88, 0.90, 0.92, 0.94, 0.96, 0.98, 1.00]
    sysid_steer_sequence: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    sysid_hold_s: 5.0
    sysid_rest_s: 4.0
    sysid_max_speed_m_per_s: 1.5
    sysid_max_distance_m: 6.5
```
录包名 `cal_speed_<车号>`，一次一档跑 11 次，每次掉头。
如果 0.80 已经很快（> 0.8 m/s），整体往下移（例如 0.70–0.90）；如果 0.80 不动，就往上移。

**结果**：脚本最后一行
```
Fit over N moving segments: v_ss = K * (drive - u0) m/s
```
把 `u0` 填进 `speed_ff_offset`，`K` 填进 `speed_ff_gain`。8 号车当时是 u0 ≈ 0.70，K ≈ 3.97（电池状态不同会变）。

## 2. 速度控制验证（约 10 分钟）

**目的**：确认 PI 在这台车上能跟住目标速度。

```yaml
    speed_control: true
    speed_ff_offset: <第 1 步的 u0>
    speed_ff_gain: <第 1 步的 K>
    speed_kp: 0.2
    speed_ki: 0.15
    speed_i_max: 0.2
    speed_i_band: 0.5
    speed_feedback_min_m_per_s: 0.4
    sysid_one_segment_per_run: false
    sysid_drive_sequence: [0.6, 0.9, 0.6]
    sysid_steer_sequence: [0.0, 0.0, 0.0]
    sysid_hold_s: 4.0
    sysid_rest_s: 0.0
```
录包名 `cal_pi_<车号>`，跑一次（约 8 m）。

**判断**：每段末速度和目标差 < 0.1 m/s、没有来回振荡就算合格。
- 超调太大 → `speed_kp` 或 `speed_ki` 减小；
- 跟得太慢、误差大 → `speed_ki` 加到 0.25。

## 3. Trim（约 15 分钟）

**目的**：让 steer = 0 时车真正走直线。

```yaml
    speed_control: true
    sysid_one_segment_per_run: false
    sysid_drive_sequence: [0.6]
    sysid_steer_sequence: [0.0]
    sysid_hold_s: 5.0
```
1. 跑一次，看 Foxglove 里的 **debug2**（曲率，1/m，**正数 = 往左**）。
2. 在 **Increment steering trim** 里填 `0.02`（往右偏时，先试正值；看 debug2 往 0 靠近还是更远，再决定方向），按一次，再跑。
3. 直到 |debug2| < 0.01，记下 `/car/steering_trim_current_normalized`。
4. **结束时把 trim 调回原来的值**（先记下原值）。

## 4. 转向映射（约 30 分钟）

**目的**：steer → 曲率的关系、左右对称性。先用卷尺量**轴距 L**。

```yaml
    speed_control: true
    sysid_one_segment_per_run: true
    sysid_drive_sequence: [0.6, 0.6, 0.6, 0.6, 0.6, 0.6]
    sysid_steer_sequence: [0.25, -0.25, 0.5, -0.5, 0.75, -0.75]
    sysid_hold_s: 6.0
```
在 6 × 6 m 空地上，一次一档跑 6 次，录包名 `cal_steer_<车号>`。第一档先看圈多大，场地不够就把 hold 改成 4.0。

**结果**：脚本每档的 `kappa` 列就是曲率（1/m）。
- 画 steer–κ 图，拟合 `κ = a × (steer − δ0)`，比较左右两边的斜率；
- 前轮转角 `δ = atan(L × κ)`；
- 可选：用锥桶标出圆，量直径 d，和 `2/κ` 对比。

---

## 5. 结束（约 5 分钟）

**B：把车恢复原样**
```bash
dream runtime stop ai4r_policy
rm -rf ~/ai4r_student_workspace/src/ai4r_policy
cp -r ~/ai4r_policy_backup ~/ai4r_student_workspace/src/ai4r_policy
dream build ros student
rm -rf ~/cal_* ~/sysid_*
```
Foxglove 确认 Disarmed，trim 已还原；关 ESC、拔电池、给充电宝充电。

## 6. 记录表（每台车一行，放在这个文件末尾）

| 日期 | 车号 | 电池状态 | ff_offset (u0) | ff_gain (K) | PI 验证结果 | trim | 转向斜率 a（左 / 右） | 轴距 L |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 29 Sep | 8 | 用了一下午，逐渐掉电 | ≈0.70 | ≈3.97 | 0.6 ✓，0.9 超调 15% | 未调（steer 0 时 κ ≈ −0.07） | 未测 | 未测 |
| 5 Oct | 20 | 电机电池全程同一块；中途只换了主板充电宝 | 0.48（拟合 0.475，PI 积分修正） | 9.18 | 0.5 m/s：kp 0.15、ki 0.04、i_band 0.2，超调 15%，稳态误差 ≈0.01 | −0.18（原值 0.0；steer 0 时 κ ≈ −0.05，直线约在 steer −0.06） | 左 −0.63（−0.75 后饱和，R_min ≈2.1 m）/ 右 −0.96（R_min ≈1.1 m） | 未测 |

20 号车备注（5 Oct）：
- **正 steer = 往右转**（debug2 正 = 往左，符号正确）。拟合（trim −0.18，0.5 m/s）：
  右 κ = −0.959·steer + 0.038；左（−0.75 ≤ steer < 0）κ = −0.630·steer + 0.008。
- 开环一阶加延时模型（cmd 为输入）：v_ss = 9.18·(cmd − 0.475)，τ ≈ 2.7 s，延时 ≈ 0.2 s；
  速度到 4 s 还没稳，测稳态要 hold ≥ 8 s。applied 比 cmd 小约 0.02（vehicle interface 映射）。
- 起步点 cmd ≈ 0.52–0.53（静摩擦，时动时不动）。目标 < 0.4 m/s 时 PI 不工作，车可能起不了步。
- 0.75 开环约 3 s 就超过 1.5 m/s；0.85 约 1.8 s。
- 主板重启后 trim 会回到 0.0，要重新设。数据：`c20_deadband`、`c20_speedmap`、`c20_pi05(_v2)`、`c20_steer`、`bags/c20_steer4`。
