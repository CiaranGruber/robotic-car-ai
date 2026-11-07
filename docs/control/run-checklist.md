# 单档地面测试：每一轮的完整步骤（8 号车）

> **团队仓库说明（control/low-level-sysid）**：本文是在 GitLab `ai4r_policy` 仓库
> `feature/sysid-test-sequencer` 分支上做 sysID 的记录。文中的 sequencer、`sysid_*` / `speed_*` 参数和
> 安全保护都在那个分支的 `scripts/policy_node.py` 里，**本仓库没有移植 sequencer**。辨识出来的
> 速度控制器和转向映射已移植到 `policy/control/`，参数在 `config/ai4r_policy.yaml` 的 `control:` 下，
> 说明见 [policy/control/README.md](../../policy/control/README.md)。

每一轮只改两处：**第 0 步的 drive 值** 和 **录包名字 runN**。

| 轮次 | drive | 录包名 |
| --- | --- | --- |
| 1 | 0.78 | sysid_A_run4 |
| 2 | 0.82 | sysid_A_run5 |
| 3 | 0.85 | sysid_A_run6 |

需要三个 PowerShell 窗口：
- **窗口 A**：本机，在 ai4r_policy 文件夹里
- **窗口 B**：`ssh ai4r@<CAR_IP>`，用来控制 policy
- **窗口 C**：`ssh ai4r@<CAR_IP>`，只用来录包

---

## 第 0 步：本机改 YAML（VS Code）

打开 `config/ai4r_policy.yaml`，只改第 117 行的数字，其余 5 行保持这样：

```yaml
    sysid_drive_sequence: [0.78]
    sysid_steer_sequence: [0.0]
    sysid_hold_s: 8.0
    sysid_rest_s: 4.0
    sysid_max_speed_m_per_s: 1.5
    sysid_max_distance_m: 15.0
```

Ctrl+S 保存。

## 第 1 步：复制到车上（窗口 A）

```powershell
scp config/ai4r_policy.yaml ai4r@<CAR_IP>:ai4r_student_workspace/src/ai4r_policy/config/
```

看到 `ai4r_policy.yaml 100%` 即成功。

## 第 2 步：重启 policy 并确认（窗口 B，整块粘贴）

```bash
dream runtime stop ai4r_policy
dream runtime start ai4r_policy
sleep 5
dream runtime status | grep ai4r_policy
ros2 param get /car/ai4r_policy sysid_drive_sequence
```

必须看到 `running` 和 `[0.78]`（本轮的值）。否则停下，把输出发给 Claude。

## 第 3 步：摆车

- 车放在起点，车头对准直道，**前方 10 m 内无障碍**。
- Yifei 拿着遥控器站在车旁。
- Foxglove：Vehicle 显示 Disabled/Disarmed，Policy 显示 Zero actions。

## 第 4 步：开始录包（窗口 C）

```bash
ros2 bag record -s mcap -o ~/sysid_A_run4 /car/drive_and_steer_set_point_normalized /car/drive_and_steer_applied_normalized /car/wheel_speed_m_per_sec /car/encoder_period_seconds /car/debug1 /car/debug2 /car/imu/data /rosout
```

等看到 `All requested topics are subscribed`。之后不要碰这个窗口，也不要按空格。

## 第 5 步：跑（Foxglove）

1. **Traxxas Request Enable**，等显示 **Enabled**
2. **Policy publishing actions**
3. 车跑 8 秒，自动停下（debug1 变为 -2）
4. **Policy publishing ZERO actions**
5. **Traxxas Request Disable**

有任何异常，立刻按 ZERO actions 或用遥控器接管。

## 第 6 步：停止录包（窗口 C）

按 **Ctrl+C**，等回到 `$` 提示符，然后检查：

```bash
ros2 bag info ~/sysid_A_run4
```

能看到 Duration 和各 topic 的 Count 即可。

## 第 7 步：拷回电脑（窗口 A）

```powershell
scp -r ai4r@<CAR_IP>:sysid_A_run4 .
```

把 `sysid_A_run4` 文件夹里的 `.mcap` 和 `metadata.yaml` 发给 Claude。

## 第 8 步：下一轮

回到第 0 步：把 drive 改成下一档，录包名改成下一个 runN，车拉回起点。
