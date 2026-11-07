# ArUco ID 2 静态识别测试

脚本 `scripts/aruco_detection_test.py` 只订阅检测结果，不读取摄像头设备，不发布控制指令，
不修改配置。依赖车上的 rclpy 和 dream_interfaces/FiducialDetections，不需要 OpenCV，
不需要 build。检测算法仍由外部 DREAM aruco_detector 执行。

## 上传

scripts 中的辅助程序被常规 car_upload_filter.txt 排除；只手动传这一个文件，不上传整套旧分支。
本机项目根目录 PowerShell：

```powershell
$carIp = "当天车辆IP"
scp .\scripts\aruco_detection_test.py "ai4r@${carIp}:aruco_detection_test.py"
ssh "ai4r@$carIp"
```

## 车上运行

先确认车辆 Disabled/Disarmed。停止策略，摄像头保持已知的朝前零位。
在车上的课程 ROS 环境执行：

```bash
dream runtime stop ai4r_policy
dream runtime start oakd_cone_detector
dream runtime start aruco_detector
dream runtime status
python3 ~/aruco_detection_test.py --id 2 --duration 60
```

本机 YAML 不代表车上的生效参数。确认实际节点名称（ros2 node list），然后检查：

```bash
ros2 param get /car/oakd_cone_detector publish_rgb_image
ros2 param get /car/aruco_detector dictionary_name
ros2 param get /car/aruco_detector allowed_marker_ids
ros2 param get /car/aruco_detector default_marker_size_m
ros2 param get /car/aruco_detector min_consecutive_frames
```

预期 RGB 开启、字典与纸质标记相符、ID 2 被允许。照片上的 ID 2 文字不是字典证明。
尺寸应为黑色外框真实边长（米），不是纸张边长。错误尺寸会影响位置估计。
不自动启动或重启服务、调整参数或 Enable。start 已运行的服务不会加载新的配置。

## 60 秒建议测试

1. 前 10 秒不放标记：应有检测消息，但显示 TARGET_NOT_VISIBLE。
2. 10–30 秒，将平整标记正对镜头，放在画面中央约 0.5–1 m：应看到 ID 2。
3. 30–40 秒移走标记：收到新的空批次后应恢复 TARGET_NOT_VISIBLE。
4. 40–60 秒重新放入：应再次识别。不要让车行驶。

之后可单独测试不同距离、角度和光照。每轮自动产生不同名字的 JSONL 日志。
`--duration 0` 持续运行到 Ctrl+C；`--topic /other/fiducial_detections` 指定其他命名空间。
`--dictionary` 只改变监视器的预期字典，不配置检测器。

## 输出解释

| 状态 | 意义 |
|---|---|
| WAITING_FOR_MESSAGES | 尚未收到消息，检查 topic、服务和 ROS 环境 |
| TARGET_NOT_VISIBLE | 数据在更新，但这批没有 ID 2；也可能看到其他 ID |
| TARGET_SEEN_POSE_OK | ID 2 和可用的位姿数值；不是物理精度证明 |
| TARGET_SEEN_NO_POSE | ID 2 已识别，但不能用其位姿 |
| MULTIPLE_TARGETS | 同一批有多个 ID 2，脚本不自动选择一个 |
| STREAM_STALE | 数据超过默认 1 秒，不沿用旧的“看到目标”结论 |
| DICTIONARY_MISMATCH | 实际字典与 --dictionary 不符 |
| BAD_TIMESTAMP / REPEATED_OR_OLDER_TIMESTAMP | 时间戳异常、重复或乱序 |

xyz 属于消息 header.frame_id，一般为相机光学坐标：x 右、y 下、z 前。
程序不做 TF 转换，不应当成车体 x 前、y 左、z 上。reprojection error 不是检测置信度。
只有校准、尺寸和位姿等都正确时才讨论距离精度。

结束汇总“收到批次数、符合新鲜度/字典的批次数、含目标批次数”。这不是原始视频的识别率，
因为外部连续帧过滤和传输丢帧已影响消息。退出码 2 表示没有任何新鲜且字典匹配的批次，
退出码 0 仅表示接收过这样的数据，不表示一定见到 ID 2 或通过实车验收。

日志保存在运行目录，使用独占创建避免覆盖。每行记录原始批次的 ID、角点、尺寸、位姿和时间；
移除标记后的空批次也会记录。用 SCP 将显示的文件名拷回电脑即可。

## 验证记录

离线测试：`python -B tests/policy/test_aruco_detection_test.py`。
尚未执行 ROS Jazzy 完整 gate、真实消息通信或实车测试；不据此声称识别精度和物理位置正确。
