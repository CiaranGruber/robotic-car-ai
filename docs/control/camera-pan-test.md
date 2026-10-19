# 相机左右朝向测试（独立脚本）

`scripts/camera_pan_test.py` 是手动测试工具，不是驾驶策略。只发布
`/<namespace>/pan_set_point_normalized`（std_msgs/Float32）；不发布油门、转向、Enable、
策略状态或 TF。无需修改 policy_node.py、YAML 或 CMake；直接运行源文件，不需要 build。
默认 dry-run，不连接 ROS；只有 `--live` 才操作云台。

## 本地分支与文件

计划放到用户指定的 `control/low-level-sysid` 分支；自动切换因 Git 写权限被拒绝，
仓库仍在 dev，尚未复制文件、提交或推送。请先在本机终端切换分支，再复制交付文件。
该分支早于 dev；只需上传这个独立脚本，不要顺带覆盖车上整套驾驶代码。
普通上传脚本会排除 scripts 下的辅助脚本，所以本测试使用下面的单文件 scp。

在本机 PowerShell 中执行（当前工作区已检查为干净；若现在有修改，先保存到提交或 stash）：

```powershell
cd "C:\Users\微星\Documents\Repository\ELEN20006\robotic-car-ai"
git switch --track -c control/low-level-sysid origin/control/low-level-sysid
```

如果提示本地分支已经存在，改用 `git switch control/low-level-sysid`。
只有切换成功后才复制文件：

```powershell
$panFiles = "C:\Users\微星\Documents\Codex\2026-09-28\xian\outputs"
Copy-Item "$panFiles\camera_pan_test.py" .\scripts\camera_pan_test.py
Copy-Item "$panFiles\camera-pan-test.md" .\docs\control\camera-pan-test.md
Copy-Item "$panFiles\test_camera_pan_test.py" .\tests\policy\test_camera_pan_test.py
git status --short --branch
```

## 1. 在 Windows 上预演

在项目根目录的 PowerShell 中执行：

```powershell
python scripts/camera_pan_test.py
```

依次输入 `0`、`0.05`、`0`、`-0.05`、`0`、`q`。这次只记录命令，不操作硬件。
如本机 Python 不在 PATH，可跳过预演；不要为了预演安装 ROS。

## 2. 只上传这一个脚本

先确认当天车辆 IP、共享车辆的使用安排。以下在本机 PowerShell 中运行：

```powershell
$carIp = "当天车辆的实际IP"
scp .\scripts\camera_pan_test.py "ai4r@${carIp}:camera_pan_test.py"
ssh "ai4r@$carIp"
```

## 3. 车上准备

在 Foxglove 确认 Vehicle 为 Disabled/Disarmed，车辆静止。Pan 即使车辆 Disabled 也能动，
留出镜头、支架和线缆的运动空间。停止策略及其他可能发云台命令的工具：

```bash
dream runtime stop ai4r_policy
dream runtime status
```

车辆接口需要运行才能接收 pan 指令；如未运行，在完成车辆状态检查后：

```bash
dream runtime start traxxas_vehicle_interface
```

在已加载课程 ROS 2 环境的 SSH 终端运行：

```bash
python3 ~/camera_pan_test.py --live --namespace /car
```

启动不发送任何位置。若缺少 rclpy/std_msgs，使用课程 ROS 环境，而非普通无 ROS 的 Python。
若报告无订阅者，检查 namespace、ROS 环境和接口状态；若有其他发布者，先停止它。
发布者检测受 ROS discovery 延迟影响，不是硬件联锁；脚本也不能确认车是否真的 Disabled。

## 4. 手动小步测试

- 输入 `0` 并回车，明确请求回中。初始位置未知，首次回中不保证只移动很小角度。
- 等相机停稳，观察是否朝车身正前方。不要假设 0 一定是准确的机械朝前位置。
- 输入 `0.05`，观察正值向哪侧移动。可以输入 `note positive moves right` 保存观察。
- 输入 `0`，然后 `-0.05`，检查另一侧，再输入 `0`。
- 每个数值是绝对目标，不是增量，也不是角度。正负方向和每单位对应的角度必须实测。
- 默认范围为 [-0.15, 0.15]，相邻命令最大差 0.05。这是软件限制，不是实测机械安全范围。
- `q`、Ctrl+C 或终端断开不会自动回中；结束前手动分步输入到 0，观察后退出。
- 不要用该脚本扫描云台后直接启动车道跟踪。离开标定零位后，固定相机 TF 可能失效。

## 5. 检查回中后的朝向

以车身中心线为参考放置标记，先观察镜头/图像是否朝正前方。只有已回到已知零位、
检测器和 TF 配置正确时，才用正前方约 2 m 锥桶的检测 y 值检查残余偏差。
不要用偏转过程中基于固定 TF 的锥桶位置直接拟合真实云台角度。

CSV 自动保存在运行目录（文件名 pan-test-日期时间.csv），可记录车号、物理方向和对齐情况。
它记录的是发送意图/发送状态和人工备注，没有位置反馈；sent_unverified 不证明舵机到位。
该脚本不自动拟合角度、不修改安装参数、不改变 MPC 的相机偏移配置。

结束后恢复操作前的服务状态；本脚本不会自动启动驾驶策略。

## 验证范围

本地可执行 `python -B tests/policy/test_camera_pan_test.py`：检查输入范围、首次回中、步长、
CSV 不覆盖、启动/退出不发送命令以及发布者/订阅者检查。模拟测试不等于 ROS 通信或实车验证。
ROS Jazzy 的仓库完整 gate 及实物运动/角度验证必须另行进行。
