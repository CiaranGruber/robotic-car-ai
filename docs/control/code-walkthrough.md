# ai4r_policy 代码导读（中文）

本文基于 v0.1.0 源码（`scripts/policy_node.py`、`config/*.yaml`、`README.md`、
`CMakeLists.txt`、`package.xml`）整理，面向组内同学快速上手。技术术语保留英文。
凡是仓库里没有直接写明、属于推断的内容，都标注了“（推断）”。以源码和 YAML
注释为准；本文与源码冲突时请以源码为准。

---

## 0. 整体结构一览

`policy_node.py` 是一个单文件、单线程（`SingleThreadedExecutor`）的 ROS 2 node，
名字固定为 `ai4r_policy`。它做三件事：

1. **收 observations**：订阅 cone detections、lidar、wheel speed、IMU，检查合法性和
   freshness 后缓存到 `self.observations`。
2. **算 actions**：在选定的 trigger 到来时调用 `calculate_policy_actions()`，也就是
   我们唯一需要写代码的地方（`INSERT POLICY CODE` markers 之间）。
3. **发 actions + 管状态机**：校验输出、clip 到 `[-1, 1]`，发布到
   `drive_and_steer_set_point_normalized` 和（可选）`pan_set_point_normalized`。

状态机（FSM）只有三个状态：

| State | 含义 |
| --- | --- |
| 1 | Not publishing any actions：完全不发命令（不是 Disarm，也不是立即停车） |
| 2 | Publishing zero actions：持续发 drive=0、steer=0（**启动默认状态**） |
| 3 | Publishing policy actions：运行我们的 policy |

- 通过 `policy_fsm_transition_request`（`std_msgs/UInt16`）请求切换，例如
  `ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'`。
- 任何 required sensor 丢失/过期、policy 抛异常、输出 NaN/inf，都会自动掉回 state 2，
  **不会自动恢复**，必须再次显式请求 state 3。
- **Policy 永远不会 Enable 车辆**。车辆 Enable/Disable/Disarm 是 Traxxas vehicle
  interface 的事。正确顺序：启动 policy（state 2 持续发 0）→ 请求车辆 Enable 并等到
  Enabled → 请求 policy state 3。state 2 的持续零命令正好满足 vehicle 的
  pre-enable fresh-zero handshake。

**policy 代码里绝对不要**：`sleep`、等待输入、无界循环、`rclpy.spin`。整个 node 只有
一个线程，policy 卡住会连带 supervision timer 一起卡住，只能靠车端独立的 command
watchdog 兜底。

---

## 1. Observations（输入）

`calculate_policy_actions(observations, sensor_age_s, sensor_stamp_ns, dt, policy_elapsed_s, is_first_policy_step)`
在函数开头已经把原始 observations 拆成了易用变量：

### 1.1 Cone detections

| 变量 | 含义 |
| --- | --- |
| `cone_data_available` | 有 fresh 的 cone 消息为 `True` |
| `num_cones` | 本帧锥桶数量，**可能为 0**，必须处理 |
| `x_coords`, `y_coords`, `z_coords` | 平行列表，单位 m，frame 为 `base_link` |
| `cone_colour` | `ConeDetection.COLOR_YELLOW`(1) / `COLOR_BLUE`(2) |
| `cone_confidence` | `[0, 1]` |

- `base_link`：+x 向前，+y 向左，+z 向上；原点在车辆约定 CG 正下方的名义地面上。
- 坐标是**车体坐标**，不是 world 坐标。
- `z` 是检测点离地高度，不是锥桶总高。
- 相机 pose 是固定的 zero-pan pose（默认光心在 CG 后 0.10 m、离地 0.30 m、下倾 20°，
  可在 `config/camera_mount.yaml` 覆盖）。**不补偿地形和车辆 pitch**。
  **物理转动 pan 后 cone 坐标不可信**，直到 DREAM 提供实测 dynamic TF。
- 空帧（`num_cones == 0` 且 `cone_data_available == True`）是合法数据；cone 为
  optional 且过期时，`cone_data_available == False`。

### 1.2 Wheel speed

- `wheel_speed_in_meters_per_second`：`float` 或 `None`。
- **Unsigned**，不区分前进/倒车。
- `None` 表示遥测缺失/过期，**不等于测到 0**。
- 由 Traxxas node 根据稀疏的 encoder period 估计，低速时测量更慢、更延迟（详见第 5 节）。
- 消息没有 header，只能用接收时间判断 freshness。

### 1.3 Lidar

- `lidar` 是 dict（或 `None`），含 `ranges`、`intensities`、`frame_id`、`angle_min`、
  `angle_max`、`angle_increment`、`time_increment`、`scan_time`、`range_min`、`range_max`。
- `lidar_ranges[i]` 的角度是 `angle_min + i * angle_increment`，**在 lidar 自身 frame
  里，没有转到 `base_link`**。
- `inf` 可能表示没有回波，`NaN` 不是零距离障碍物。只保留 finite 且在
  `[range_min, range_max]` 内的点（源码里有示例）。

### 1.4 IMU

三个字段**独立**校验和计算 freshness（一个新的 gyro 消息不代表新的 heading）：

| 变量 | 含义 |
| --- | --- |
| `orientation_xyzw` | body frame 四元数，(x, y, z, w)，绝对朝向参考 magnetic ENU |
| `roll_angle_in_radians`, `pitch_angle_in_radians` | 由上面算出 |
| `heading_angle_in_radians` | **相对** heading，只在进入 state 3 时归零，wrap 到 `[-pi, pi]`，会漂移 |
| `angular_velocity_rad_per_sec` | body frame 角速度 (x, y, z)，rad/s |
| `specific_force_m_per_sec_squared` | body frame specific force，m/s²，**包含重力** |

- 旋转到 body frame 用的是 DREAM 提供的 mounting TF。
- Partial IMU 消息是正常的；`covariance[0] == -1` 表示该字段缺失，其数值 0 不是测量值。
- 相对 heading 也会发布到 `imu_heading_angle`（度）。

### 1.5 时间相关

| 变量 | 含义 |
| --- | --- |
| `sensor_age_s[name]` | 上次被接受样本的年龄（秒），没有则 `None` |
| `sensor_stamp_ns[name]` | 该样本的 ROS stamp（ns），wheel speed 为 `None` |
| `dt` | 两次 policy step 之间**实际测得**的 monotonic 秒数，**首步为 0.0**，除以 `dt` 前必须判断 |
| `policy_elapsed_s` | 进入 state 3 后经过的秒数 |
| `is_first_policy_step` | 每次进入 state 3 的第一步为 `True`，用来重置积分器等状态 |

timer mode 下同一个传感器样本可能被多个 step 重复使用：**一个 policy step 不一定是一次新测量**。

---

## 2. Actions（输出）

函数返回 `(drive_action, steering_action, camera_pan_action, debug1, debug2)`：

| 输出 | 含义 |
| --- | --- |
| `drive_action` | 归一化 `[-1, 1]`，**motor effort (ESC)**，不是 m/s 的速度 setpoint |
| `steering_action` | 归一化 `[-1, 1]`，映射到 vehicle interface 标定的 steering interval，0 为中位 |
| `camera_pan_action` | 归一化 pan servo target，不是弧度；`None` = 不发命令、保持当前位置；0 = 回中。即使车辆未 Enable 也可能转动 |
| `debug1`, `debug2` | 可选 `float`，发布到 `debug1`/`debug2`，方便在 Foxglove / rosbag 里看 |

- 发布前先整体校验：drive/steer 必须是 finite number；pan/debug 可以是 `None`。
- 超出 `[-1, 1]` 会被 **clip** 并打 warning。
- NaN/inf/异常 → 回到 state 2 发零，**不会把非法值变成电机命令**。
- 计算返回后会再检查一次 sensor health，过期则不发布本次结果。
- 停止时 pan 保持原位（pan hold），不会回中。

---

## 3. Vehicle interface 的限制（`config/traxxas_vehicle_interface.yaml`）

这个文件描述的是 Traxxas vehicle interface node，**不是** policy 的参数。所有参数都是
startup-only；policy 的 launch 不会把它加载到 vehicle node，需要 DREAM admit 后
`dream runtime restart traxxas_vehicle_interface` 才生效。注释掉的 “REFERENCE ONLY”
项只是组件默认值，**不代表我们这台车的实际标定**。

### 3.1 Slew rates（active 参数，可以申请调整）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `drive_slew_rate_per_s` | 50.0 /s | drive 命令每秒最大变化量（归一化单位） |
| `steering_slew_rate_per_s` | 10.0 /s | steering 命令每秒最大变化量 |

换算一下（推断，按线性 rate limit 计算）：

- drive 从 0 到 1 最少约 0.02 s，从 -1 到 1 约 0.04 s，基本不构成限制。
- **steering 从 0 到满舵约 0.1 s，左满到右满约 0.2 s**。对 50 Hz 的 policy 来说，
  每个 step（0.02 s）steering 最多变 0.2。这是一个明显的 actuator rate limit，
  做 steering 模型和控制器设计时要考虑进去。
- slew rate 限的是**命令**的变化，不是物理加速度，也不是速度 setpoint。

### 3.2 Timeout

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `policy_command_timeout_s`（reference only） | 0.5 s | 超过这个时间没收到新 action，vehicle 会 disable 控制；也限制了 Enable 后等待 fresh zero 的时间 |
| `telemetry_timeout_s`（reference only） | 0.1 s | MCU 遥测的 deadline（≥ 两个 0.02 s 名义周期） |

- vehicle 内部控制周期 0.01 s，MCU 遥测名义周期 0.02 s（来自 YAML 注释）。
- 我们的 action 间隔必须留足余量，远小于 0.5 s。cone_detection mode 下 action 频率等于
  相机频率（默认 10 Hz → 0.1 s），timer mode 默认 50 Hz。
- **不要通过加大 timeout 去掩盖 policy 算得太慢的问题**。

policy 侧也有自己的 timeout（`config/ai4r_policy.yaml`）：

- `sensor_timeout_s.*`：每个传感器默认 0.5 s，age ≥ 该值即过期。stamped 数据要求
  接收时间和 ROS stamp **两者都** fresh。
- `cone_empty_timeout_s`：默认 1.0 s，required cones 连续空帧超过这个时间就停车。
  启动时必须先有至少一个非空 batch。
- `supervision_rate_hz`：默认 20 Hz，watchdog 和 state 2 的零命令频率。
- `timestamp_tolerance_s`：默认 0.05 s，允许 stamp 稍微“来自未来”。

### 3.3 Trim 与 steering interval（reference only）

- `steering_min_normalized` / `steering_max_normalized`：默认 `-0.5` / `0.5`。
  我们输出的 `[-1, 1]` steering 会**缩放**到这个区间，所以 `steering_action = 1.0`
  并不是舵机的机械极限。两个端点要成对设置。
- `steering_trim_normalized`：默认 0.0，初始中位修正，必须在上述区间内。
- 运行时 trim 通过专门的 steering-trim topics 调整，不是 live 修改参数。
- 每台车的物理限制和 trim 由 DREAM 提供；手动 RC 不受这些 ROS 端点限制。

### 3.4 Deadband

**仓库里没有任何 deadband 参数或说明**（policy、vehicle YAML、README 都没有）。
与之相关的只有：

- `rc_drive_min_us` / `rc_drive_center_us` / `rc_drive_max_us`（默认 1000/1500/2000 µs）
  以及 steering 对应值：RC receiver 标定，是物理测量值，不是 gain。标定错了，
  neutral/forward/reverse 的含义都会变。
- `rc_stop_threshold_us`（默认 1400 µs）：倒车 stop gesture 的阈值，不是 deadband。

因此（推断）：ESC 在 neutral 附近的 drive deadband（小 effort 不起转）、steering
的机械间隙等，需要我们自己通过实验辨识，并在 policy 里补偿（例如 deadband
compensation / feedforward offset）。这是 system ID 小组的重点工作之一。

### 3.5 Encoder / wheel speed 相关（active 参数）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `encoder_time_filter_window` | 6 | 对 encoder **period** 样本做平均的窗口长度，越大越平滑但越滞后 |
| `encoder_timeout_seconds` | 3.0 s | 多久没有新 encoder period 就判定车轮速度为 0 |

reference-only 的几何参数：`encoder_cycles_per_revolution: 3.0`、
`encoder_to_wheel_ratio: 2.72`、`wheel_diameter_m: 0.105`。这些决定了测得的 period
如何换算成 m/s，抄错会直接让速度失真。

---

## 4. Policy update mode 与 `required_sensors`（`config/ai4r_policy.yaml`）

### 4.1 `policy_update_mode`

同一时间只有**一个** trigger，没有自动 fallback：

| Mode | 什么时候执行一次 policy |
| --- | --- |
| `cone_detection`（默认） | 每收到一个被接受的 fresh cone batch（频率 ≈ `camera_frame_frequency_hz`，默认 10 Hz） |
| `lidar` | 每收到一个被接受的 fresh lidar scan |
| `timer` | 按 `policy_update_rate_hz`（默认 50 Hz）用缓存的 observations 执行 |

其他 callback 只缓存数据，不触发 policy。timer mode 的 timer 和 supervision timer
都用 steady clock。

### 4.2 `required_sensors`

可选值：`cone_detections`、`lidar`、`wheel_speed`、`imu_orientation`、
`imu_angular_velocity`、`imu_specific_force`。

- required 的传感器缺失或过期 → 自动回 state 2，并拒绝进入 state 3。
- optional 的传感器缺失或过期 → 在 policy 里是 `None`。
- 约束：`cone_detection` mode 必须 require `cone_detections`；`lidar` mode 必须
  require `lidar`；`timer` mode 可以设为 `[]`（刻意做 open-loop 实验）。
- 名字不能重复，不能拼错，否则 node 启动时直接报错。

YAML 里给了可以直接复制的配置（替换掉 active 的设置，不要重复 key）：

```yaml
# Lidar-only
policy_update_mode: lidar
required_sensors: [lidar]

# Wheel-speed controller tuning（最适合我们小组）
policy_update_mode: timer
policy_update_rate_hz: 50.0
required_sensors: [wheel_speed]

# Cone-following + heading + speed feedback
policy_update_mode: cone_detection
required_sensors: [cone_detections, imu_orientation, wheel_speed]
```

改 `config/*.yaml` **不需要 rebuild**：DREAM 每次 start/restart 时读取
`~/ai4r_student_workspace/src/ai4r_policy/config/` 下的源 YAML。改完只重启受影响的 unit，
例如 `dream runtime restart ai4r_policy`。

### 4.3 自定义参数（例如 controller gain）

三步，缺一不可（源码 181–188 行有注释）：

1. 在 `__init__` 里 `self.declare_parameter('speed_kp', 0.2, ParameterDescriptor(read_only=True))`。
2. 在 `ai4r_policy.yaml` 的 `ros__parameters` 下加 `speed_kp: 0.2`。
3. `self.speed_kp = self.get_parameter('speed_kp').value`。

注意 `0.2` 是 double，`0` 是 integer，ROS 里是不同类型；gain 请写成 `0.0` 这种形式。
只写 YAML 不 declare 是不生效的。

---

## 5. Student build 之后能不能 import 额外模块？

结论：**标准库可以随便 import；第三方库要看车上是否已经装好；自己新建的 `.py`
helper 模块默认不会被安装，import 会失败。**

依据和解释：

- `CMakeLists.txt` 只做了 `install(PROGRAMS scripts/policy_node.py DESTINATION lib/${PROJECT_NAME})`，
  即**只安装这一个文件**。在 `scripts/` 下新建 `my_helpers.py` 不会被 install，
  运行时 `import my_helpers` 会 `ModuleNotFoundError`（推断，基于 CMake 规则）。
  而且仓库约定所有 policy 代码都放在 `policy_node.py` 里（`AGENTS.md`、`CONTRIBUTING.md`），
  不要拆文件。
- **Python 标准库**（`math`、`collections`、`time`、`statistics` 等）永远可用。
  `math` 已经在文件顶部 import 了。注意：不要在 policy 里调用 `time.sleep`。
- **第三方库（如 `numpy`、`scipy`）**：`package.xml` 没有声明它们；README 明确说 policy
  “does not ... install student dependencies”。所以只有当车上的 Python 环境已经装了才能用。
  ROS 2 Jazzy 环境通常带有 `numpy`（推断，未在车上验证，本次按要求没有 ssh 上车）。
  上车前应先确认，例如 `python3 -c "import numpy; print(numpy.__version__)"`。
- 如果 import 失败，`policy_node.py` 会在启动时直接崩溃，**连 state 2 的零命令都发不出来**；
  此时只能靠 vehicle 端 0.5 s 的 command timeout 让车 disable。所以新增 import 一定要先在
  车上验证，而且放在文件顶部（模块级），**不要在 policy step 里 import**（首次 import
  可能耗时、阻塞实时循环）。
- 修改 `policy_node.py` 后需要在 student workspace 里重新 build（`colcon build`）再
  重启 policy，因为运行的是 install 目录里的拷贝（推断：CMake `install(PROGRAMS)` 是复制；
  若用 `--symlink-install` 则不一定需要，具体以 DREAM runtime guide 的 build 流程为准）。
  YAML 则不需要 rebuild（见 4.2）。

---

## 6. 对 low-level control / system ID 小组的要点

1. **drive 是 ESC effort，不是速度**。要做速度控制，就必须自己闭环：
   `wheel_speed` → controller → `drive_action`。系统辨识的核心对象就是
   “effort → wheel speed” 的静态增益、deadband、时间常数和延迟。
2. **推荐实验配置**：`timer` mode、50 Hz、`required_sensors: [wheel_speed]`
   （YAML 里现成的 “Wheel-speed-controller tuning” 例子），不依赖相机和 lidar。
   做 open-loop 阶跃/斜坡/PRBS 时，可以用 `policy_elapsed_s` 生成输入序列，
   用 `is_first_policy_step` 重置状态。
3. **记录数据**：用 `debug1`/`debug2` 发出 commanded effort、测得速度等，配合
   `ros2 bag record` 录 `drive_and_steer_set_point_normalized`、`wheel_speed_m_per_sec`、
   `imu/data`、`debug1`、`debug2`。注意我们记录的是**slew 之前**的命令；vehicle 内部
   slew 之后真正送到 ESC 的值不在 policy 的 topic 里。
4. **Wheel speed 测量的局限**（辨识结果会受影响）：
   - unsigned：做倒车实验时无法区分方向，需要结合 command 符号判断。
   - 基于稀疏 encoder period，低速时更新慢；`encoder_time_filter_window = 6` 会引入额外滞后；
     `encoder_timeout_seconds = 3.0` 意味着从低速停下来要最多 3 s 才会报 0。
     辨识低速段和 deadband 时要特别小心，这段滞后是测量造成的，不是车本身的动态。
   - `dt` 是实测值，辨识时请用 `dt` 和 `sensor_stamp_ns` / `sensor_age_s`，不要假设 1/50 s；
     timer mode 下同一个 wheel speed 样本可能被重复使用，拟合时要去重。
5. **Actuator 限制要进模型**：
   - drive slew 50 /s、steering slew 10 /s（满舵约 0.1 s）；
   - steering 输出被缩放到 `[-0.5, 0.5]` 区间（默认值，每台车以 DREAM 标定为准），
     再加 trim；辨识 “steering_action → 转弯半径 / yaw rate” 时要以实际标定为准；
   - deadband 在仓库里没有，需要我们自己测。
6. **Steering 辨识**：`imu_angular_velocity` 的 z 分量就是 yaw rate（body frame，rad/s），
   配合 `wheel_speed` 可以估计 “steering → curvature” 的映射（bicycle model）。
   `specific_force` 含重力，不能直接当纵向加速度积分；需要先减去重力分量（可用 orientation）。
   IMU 采样率由固件决定，名义约 50 Hz。
7. **调参流程**：gain 做成 ROS parameter（第 4.3 节），改 YAML 后只需
   `dream runtime restart ai4r_policy`，无需 rebuild。vehicle 的 slew rate、encoder 参数
   属于 `traxxas_vehicle_interface.yaml`，需要 DREAM admit 并重启 vehicle interface，
   不要随意改 reference-only 的标定项。
8. **安全**：policy 发 0 不等于车一定停下，也不能从手动 RC 手里夺回控制；
   state 1 不是急停。实验前确认 vehicle Enable 流程和 RC 接管。仓库 `docs/acceptance.md`
   的 robot checklist 目前全部是 **not run**（包括实际 command timing、物理停车行为），
   这些正是我们上车实验时可以顺便验证并记录的内容。

---

## 7. 快速索引

| 想找的东西 | 位置 |
| --- | --- |
| 我们写代码的地方 | `scripts/policy_node.py` 中 `calculate_policy_actions()` 的 `INSERT POLICY CODE` markers 之间 |
| Observations/Actions 官方说明 | 同函数内 `EXPLANATION OF THE OBSERVATIONS` / `EXPLANATION OF THE ACTIONS` 注释块 |
| 自定义参数 | `PolicyNode.__init__` 中 “TO ADD A STUDENT PARAMETER” 注释 |
| Freshness / health 逻辑 | `_store()`、`_fresh()`、`health_problem()` |
| 状态机 | `fsm_transition_request_callback()`、`_change_state()`、`supervision_callback()` |
| Policy 设置 | `config/ai4r_policy.yaml` |
| Vehicle 限制与标定 | `config/traxxas_vehicle_interface.yaml` |
| 相机 / IMU 设置 | `config/oakd_cone_detector.yaml`、`config/bno08x_imu_interface.yaml`、`config/camera_mount.yaml` |
