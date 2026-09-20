"""Real ROS types, controlled time for policy contracts, and an isolated graph.

Run through tools/verify_fast.sh on Ubuntu/ROS Jazzy. No sensor driver, serial
device, vehicle process, or physical actuator is started by these tests.
"""
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, UInt16
from dream_interfaces.msg import ConeDetection, ConeDetections, DriveAndSteer
import yaml


SHARE = Path(get_package_share_directory("ai4r_policy"))
SCRIPT = Path(get_package_prefix("ai4r_policy")) / "lib/ai4r_policy/policy_node.py"
spec = importlib.util.spec_from_file_location("ai4r_policy_test_subject", SCRIPT)
policy = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = policy
spec.loader.exec_module(policy)


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class ControlledClock:
    def __init__(self):
        self.monotonic = 10.0
        self.ros_ns = 100_000_000_000

    def advance(self, seconds):
        self.monotonic += seconds
        self.ros_ns += round(seconds * 1e9)

    def now(self):
        return Time(nanoseconds=self.ros_ns)


@pytest.fixture
def make_node(monkeypatch):
    context = Context()
    rclpy.init(context=context)
    nodes = []

    def create(mode="timer", required=(), **extra):
        settings = {"policy_update_mode": mode, "required_sensors": list(required), **extra}
        overrides = [Parameter(name, value=value) if value != [] else
                     Parameter(name, Parameter.Type.STRING_ARRAY, [])
                     for name, value in settings.items()]
        node = policy.PolicyNode(context=context, namespace="contract_test",
                                 parameter_overrides=overrides)
        nodes.append(node)
        node.supervision_timer.cancel()
        if node.policy_timer:
            node.policy_timer.cancel()
        clock = ControlledClock()
        node._monotonic = lambda: clock.monotonic
        monkeypatch.setattr(node, "get_clock", lambda: clock)
        node._last_status_at = -math.inf
        for name in ("action", "pan", "state", "state_text", "heading", "debug1", "debug2"):
            setattr(node, name + "_publisher", Recorder())
        node.test_clock = clock
        return node

    yield create
    for node in reversed(nodes):
        node.destroy_node()
    context.shutdown()


def stamp(node, msg, offset=0.0, frame="base_link"):
    msg.header.stamp = Time(nanoseconds=node.test_clock.ros_ns + round(offset * 1e9)).to_msg()
    msg.header.frame_id = frame
    return msg


def cones(node, empty=False, **kwargs):
    msg = stamp(node, ConeDetections(), **kwargs)
    if not empty:
        cone = ConeDetection()
        cone.position.x, cone.position.y = 1.0, 0.5
        cone.color = ConeDetection.COLOR_YELLOW
        cone.classification_confidence = 0.8
        msg.detections = [cone]
    return msg


def scan(node, **kwargs):
    msg = stamp(node, LaserScan(), frame="laser", **kwargs)
    msg.angle_min, msg.angle_max, msg.angle_increment = 0.0, 0.1, 0.1
    msg.range_min, msg.range_max = 0.05, 12.0
    msg.ranges = [1.0, float("inf")]
    return msg


def imu(node, yaw=0.0, orientation=True, **kwargs):
    msg = stamp(node, Imu(), **kwargs)
    msg.orientation.z = math.sin(yaw / 2)
    msg.orientation.w = math.cos(yaw / 2)
    msg.orientation_covariance[0] = 0.0 if orientation else -1.0
    msg.angular_velocity_covariance[0] = -1.0
    msg.linear_acceleration_covariance[0] = -1.0
    return msg


def request(node, state):
    node.fsm_transition_request_callback(UInt16(data=state))


def driving_policy(*args):
    return 0.4, -0.2, None, None, None


def test_startup_zero_and_not_publishing_meanings(make_node):
    node = make_node()
    assert node.fsm_state == 2
    node.supervision_callback()
    assert [(m.drive, m.steer) for m in node.action_publisher.messages] == [(0.0, 0.0)]
    assert node.action_publisher.messages[0].units == DriveAndSteer.UNITS_NORMALIZED
    assert node.pan_publisher.messages == []
    request(node, 1)
    count = len(node.action_publisher.messages)
    node.supervision_callback()
    node.run_policy_step()
    node.stop_before_shutdown()
    assert len(node.action_publisher.messages) == count
    request(node, 99)
    assert node.fsm_state == 1
    request(node, 2)
    assert len(node.action_publisher.messages) == count + 1


@pytest.mark.parametrize("mode,required", [("cone_detection", ["cone_detections"]),
                                          ("lidar", ["lidar"]), ("timer", [])])
def test_only_selected_trigger_executes_policy(make_node, mode, required):
    node = make_node(mode, required)
    node.cone_detection_callback(cones(node))
    node.lidar_callback(scan(node))
    request(node, 3)
    calls = []
    node.calculate_policy_actions = lambda *args: (calls.append(args) or driving_policy())
    node.test_clock.advance(0.05)
    node.cone_detection_callback(cones(node))
    node.lidar_callback(scan(node))
    node.wheel_speed_callback(Float32(data=0.5))
    node.imu_callback(imu(node))
    node.supervision_callback()
    assert len(calls) == (0 if mode == "timer" else 1)
    assert (node.policy_timer is not None) == (mode == "timer")
    if mode == "timer":
        node.run_policy_step()
        assert len(calls) == 1
    assert calls[0][-1] is True
    assert calls[0][-3] == 0.0
    # Repeated sensor stamps do not trigger another calculation.
    node.cone_detection_callback(cones(node))
    node.lidar_callback(scan(node))
    assert len(calls) == 1


@pytest.mark.parametrize("required", [["lidar"], ["wheel_speed"]])
def test_exercises_need_no_cone_publisher_and_recovery_is_explicit(make_node, required):
    mode = "lidar" if required == ["lidar"] else "timer"
    node = make_node(mode, required)
    feed = (lambda: node.lidar_callback(scan(node))) if mode == "lidar" else (
        lambda: node.wheel_speed_callback(Float32(data=0.0)))
    request(node, 3)
    assert node.fsm_state == 2
    feed()
    node.calculate_policy_actions = driving_policy
    request(node, 3)
    assert node.fsm_state == 3
    node.run_policy_step()
    assert node.action_publisher.messages[-1].drive > 0.0
    node.test_clock.advance(0.5)
    node.supervision_callback()
    assert node.fsm_state == 2
    assert node.action_publisher.messages[-1].drive == 0.0
    feed()
    node.supervision_callback()
    assert node.fsm_state == 2
    request(node, 3)
    assert node.fsm_state == 3


def test_empty_cones_and_missing_stream_have_independent_deadlines(make_node):
    node = make_node("cone_detection", ["cone_detections"])
    node.cone_detection_callback(cones(node, empty=True))
    request(node, 3)
    assert node.fsm_state == 2  # Never had a nonempty frame.
    node.test_clock.advance(0.1)
    node.cone_detection_callback(cones(node))
    request(node, 3)
    for _ in range(4):
        node.test_clock.advance(0.2)
        node.cone_detection_callback(cones(node, empty=True))
        assert node.fsm_state == 3
    node.test_clock.advance(0.201)
    node.cone_detection_callback(cones(node, empty=True))
    assert node.fsm_state == 2
    assert "nonempty" in node.state_reason
    node.test_clock.advance(0.01)
    node.cone_detection_callback(cones(node))
    assert node.fsm_state == 2
    request(node, 3)
    node.test_clock.advance(0.5)
    node.supervision_callback()
    assert node.fsm_state == 2
    assert "missing or stale" in node.state_reason


@pytest.mark.parametrize("offset", [-0.5, 0.051, -100.0])
def test_rejected_stamps_do_not_refresh_data(make_node, offset):
    node = make_node("lidar", ["lidar"])
    node.lidar_callback(scan(node, offset=offset))
    assert node.observations["lidar"] is None
    node.lidar_callback(scan(node))
    received = node.observations["lidar"].received_at
    node.test_clock.advance(0.1)
    node.lidar_callback(scan(node, offset=-0.1))
    assert node.observations["lidar"].received_at == received


def test_optional_stale_values_are_unavailable_and_snapshots_are_copies(make_node):
    node = make_node()
    node.cone_detection_callback(cones(node))
    node.lidar_callback(scan(node))
    request(node, 3)
    captured = []

    def student(values, ages, stamps, *args):
        captured.append((values, ages, stamps))
        if values["lidar"] is not None:
            values["lidar"]["ranges"][0] = 99.0
        return driving_policy()

    node.calculate_policy_actions = student
    node.run_policy_step()
    assert node.observations["lidar"].value["ranges"][0] == 1.0
    node.test_clock.advance(0.5)
    node.run_policy_step()
    assert captured[-1][0]["lidar"] is None
    assert captured[-1][0]["cone_detections"] is None
    assert captured[-1][1]["lidar"] == pytest.approx(0.5)
    assert node.fsm_state == 3


def test_imu_mounting_rotation_and_partial_field_freshness(make_node):
    node = make_node(required=["imu_orientation"])
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = "base_link", "imu_sensor"
    transform.transform.rotation.z = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    node.tf_buffer.set_transform_static(transform, "test")
    msg = imu(node, yaw=math.pi / 2, frame="imu_sensor")
    msg.angular_velocity_covariance[0] = 0.0
    msg.angular_velocity.x = 1.0
    msg.linear_acceleration_covariance[0] = 0.0
    msg.linear_acceleration.z = 9.81
    node.imu_callback(msg)
    assert policy.roll_pitch_yaw(node.observations["imu_orientation"].value)[2] == pytest.approx(0.0)
    assert node.observations["imu_angular_velocity"].value == pytest.approx((0.0, 1.0, 0.0))
    assert node.observations["imu_specific_force"].value == pytest.approx((0.0, 0.0, 9.81))
    request(node, 3)
    node.test_clock.advance(0.5)
    partial = imu(node, orientation=False, frame="imu_sensor")
    partial.angular_velocity_covariance[0] = 0.0
    partial.angular_velocity.x = 2.0
    node.imu_callback(partial)
    now, ros_now = node._times()
    assert node._fresh("imu_angular_velocity", now, ros_now)
    assert not node._fresh("imu_orientation", now, ros_now)
    node.supervision_callback()
    assert node.fsm_state == 2


def test_missing_imu_transform_or_bad_quaternion_cannot_satisfy_required_heading(make_node):
    node = make_node(required=["imu_orientation"])
    node.imu_callback(imu(node, frame="missing_mount"))
    request(node, 3)
    assert node.fsm_state == 2
    msg = imu(node)
    msg.orientation.w = 0.0
    node.imu_callback(msg)
    assert node.observations["imu_orientation"] is None


def test_older_imu_field_does_not_replace_newer_but_new_peer_is_accepted(make_node):
    node = make_node()
    node.imu_callback(imu(node, yaw=0.4))
    orientation = node.observations["imu_orientation"]
    msg = imu(node, yaw=1.0, offset=-0.01)
    msg.angular_velocity_covariance[0] = 0.0
    msg.angular_velocity.x = 0.2
    node.imu_callback(msg)
    assert node.observations["imu_orientation"] is orientation
    assert node.observations["imu_angular_velocity"].value[0] == pytest.approx(0.2)


def test_heading_and_time_reset_only_on_actual_state_entry(make_node):
    node = make_node(required=["imu_orientation"])
    node.imu_callback(imu(node, yaw=0.4))
    request(node, 3)
    start = node.policy_started_at
    node.run_policy_step()
    node.test_clock.advance(0.1)
    node.imu_callback(imu(node, yaw=0.9))
    request(node, 3)
    assert node.heading_reference == pytest.approx(0.4)
    assert node.policy_started_at == start
    assert node.previous_policy_step_at is not None
    assert node.heading_publisher.messages[-1].data == pytest.approx(math.degrees(0.5))
    request(node, 2)
    request(node, 3)
    assert node.heading_reference == pytest.approx(0.9)
    assert node.previous_policy_step_at is None
    assert node.policy_started_at > start


def test_optional_delayed_heading_does_not_silently_tare(make_node):
    node = make_node()
    request(node, 3)
    node.imu_callback(imu(node, yaw=0.5))
    assert node.heading_reference is None
    assert not node.heading_publisher.messages


@pytest.mark.parametrize("required", [["lidar"], ["wheel_speed"]])
def test_backward_clock_invalidates_stamped_data_only(make_node, required):
    node = make_node(required=required)
    node.lidar_callback(scan(node))
    node.wheel_speed_callback(Float32(data=0.0))
    request(node, 3)
    node.test_clock.ros_ns -= 1_000_000_000
    node.supervision_callback()
    assert node.observations["lidar"] is None
    assert node.fsm_state == (2 if "lidar" in required else 3)


@pytest.mark.parametrize("result", [(math.nan, 0, None, None, None),
                                    (0.4, 0, math.inf, None, None),
                                    ("0.4", 0, None, None, None),
                                    (True, 0, None, None, None)])
def test_invalid_result_stops_before_any_pan_or_nonzero_publication(make_node, result):
    node = make_node()
    request(node, 3)
    node.calculate_policy_actions = lambda *args: result
    node.run_policy_step()
    assert node.fsm_state == 2
    assert all(msg.drive == 0.0 and msg.steer == 0.0 for msg in node.action_publisher.messages)
    assert not node.pan_publisher.messages


def test_clipping_pan_hold_exception_and_overrun(make_node):
    node = make_node(required=["wheel_speed"])
    node.wheel_speed_callback(Float32(data=0.0))
    request(node, 3)
    node.calculate_policy_actions = lambda *args: (3.0, -4.0, 2.0, None, None)
    node.run_policy_step()
    msg = node.action_publisher.messages[-1]
    assert (msg.drive, msg.steer) == (1.0, -1.0)
    assert node.pan_publisher.messages[-1].data == 1.0

    def slow_student(*args):
        node.test_clock.advance(0.5)
        return driving_policy()

    node.calculate_policy_actions = slow_student
    node.run_policy_step()
    assert node.fsm_state == 2
    assert node.action_publisher.messages[-1].drive == 0.0
    node.supervision_callback()
    assert len(node.pan_publisher.messages) == 1
    node.wheel_speed_callback(Float32(data=0.0))
    request(node, 3)

    def broken_student(*args):
        raise ZeroDivisionError("student computation")

    node.calculate_policy_actions = broken_student
    node.run_policy_step()
    assert node.fsm_state == 2
    assert node.action_publisher.messages[-1].drive == 0.0


def test_shipped_student_calculation_is_zero_and_handles_missing_observations(make_node):
    node = make_node()
    request(node, 3)
    node.run_policy_step()
    assert node.action_publisher.messages[-1].drive == 0.0
    assert node.action_publisher.messages[-1].steer == 0.0
    assert not node.pan_publisher.messages


def test_invalid_sensor_content_and_runtime_parameter_changes(make_node):
    node = make_node()
    bad_cone = cones(node)
    bad_cone.detections[0].position.x = math.nan
    node.cone_detection_callback(bad_cone)
    node.cone_detection_callback(cones(node, frame="optical"))
    bad_scan = scan(node)
    bad_scan.angle_increment = 0.0
    node.lidar_callback(bad_scan)
    node.wheel_speed_callback(Float32(data=-0.1))
    assert all(value is None for value in node.observations.values())
    assert not node.set_parameters_atomically([Parameter("use_sim_time", value=True)]).successful
    assert not node.set_parameters_atomically([Parameter("policy_update_mode", value="lidar")]).successful


@pytest.mark.parametrize("mode,required,extra", [
    ("automatic", [], {}), ("lidar", [], {}), ("cone_detection", [], {}),
    ("timer", ["unknown"], {}), ("timer", ["lidar", "lidar"], {}),
    ("timer", [], {"supervision_rate_hz": 0.0}),
    ("timer", [], {"timestamp_tolerance_s": -1.0}),
    ("timer", [], {"policy_frame_id": "/base_link"}),
    ("timer", [], {"use_sim_time": True}),
])
def test_invalid_startup_settings_fail(make_node, mode, required, extra):
    with pytest.raises(ValueError):
        make_node(mode, required, **extra)


def test_installed_configs_and_namespaced_loading(tmp_path):
    for name in ("ai4r_policy", "traxxas_vehicle_interface", "oakd_cone_detector", "bno08x_imu_interface"):
        document = yaml.safe_load((SHARE / "config" / f"{name}.yaml").read_text())
        assert list(document) == [f"/**/{name}"]
        assert isinstance(document[f"/**/{name}"]["ros__parameters"], dict)
    context = Context()
    rclpy.init(context=context)
    node = policy.PolicyNode(context=context, namespace="namespaced",
        cli_args=["--ros-args", "--params-file", str(SHARE / "config/ai4r_policy.yaml")])
    try:
        assert node.get_fully_qualified_name() == "/namespaced/ai4r_policy"
        assert node.policy_update_mode == "cone_detection"
        assert node.required_sensors == ["cone_detections"]
    finally:
        node.destroy_node()
        context.shutdown()

    # [] has no element from which the ROS YAML parser can infer an array type.
    # Confirm the deliberate open-loop configuration also works through YAML.
    overlay = tmp_path / "open_loop.yaml"
    overlay.write_text("/**/ai4r_policy:\n  ros__parameters:\n"
                       "    policy_update_mode: timer\n    required_sensors: []\n")
    context = Context()
    rclpy.init(context=context)
    node = policy.PolicyNode(context=context, namespace="namespaced",
        cli_args=["--ros-args", "--params-file", str(overlay)])
    try:
        assert node.policy_update_mode == "timer"
        assert node.required_sensors == []
    finally:
        node.destroy_node()
        context.shutdown()


def test_installed_launch_starts_executable_and_shuts_down(tmp_path):
    assert os.access(SCRIPT, os.X_OK), "Installed policy script is not executable"
    environment = dict(os.environ, ROS_LOG_DIR=str(tmp_path / "ros_logs"))
    process = subprocess.Popen(
        ["ros2", "launch", "ai4r_policy", "ai4r_policy.launch.py", "namespace:=launch_check"],
        cwd=tmp_path, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, start_new_session=True)
    try:
        try:
            output, _ = process.communicate(timeout=2.0)
            pytest.fail("Policy launch exited before shutdown was requested:\n" + output)
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=5.0)
        assert process.returncode == 0, output
        assert "Policy update source: cone_detection" in output
        assert "[ERROR]" not in output and "Traceback" not in output
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=3.0)


@pytest.mark.parametrize("mode", ["lidar", "timer"])
def test_real_ros_graph_zero_handshake_sensor_qos_and_watchdog(mode):
    """Exercise DDS/timers with synthetic peers, not a simulated physical car."""
    context = Context()
    rclpy.init(context=context)
    required = "lidar" if mode == "lidar" else "wheel_speed"
    node = policy.PolicyNode(context=context, namespace="graph_test",
        parameter_overrides=[Parameter("policy_update_mode", value=mode),
                             Parameter("required_sensors", value=[required])])
    peer = Node("synthetic_peer", context=context, namespace="graph_test")
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(peer)
    actions, pans = [], []
    peer.create_subscription(DriveAndSteer, "drive_and_steer_set_point_normalized", actions.append, 10)
    peer.create_subscription(Float32, "pan_set_point_normalized", pans.append, 10)
    request_pub = peer.create_publisher(UInt16, "policy_fsm_transition_request", 10)
    # A reliable scan publisher must match the policy's best-effort subscription.
    sensor_pub = peer.create_publisher(LaserScan if mode == "lidar" else Float32,
                                      "scan" if mode == "lidar" else "wheel_speed_m_per_sec",
                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    node.calculate_policy_actions = driving_policy

    def spin_until(condition, timeout=3.0, feed=False, start=False):
        deadline = time.monotonic() + timeout
        next_send = 0.0
        while time.monotonic() < deadline:
            if feed and time.monotonic() >= next_send:
                if mode == "lidar":
                    msg = LaserScan()
                    msg.header.stamp = peer.get_clock().now().to_msg()
                    msg.header.frame_id = "laser"
                    msg.angle_max, msg.angle_increment = 0.1, 0.1
                    msg.range_min, msg.range_max, msg.ranges = 0.05, 12.0, [1.0, math.inf]
                else:
                    msg = Float32(data=0.0)
                sensor_pub.publish(msg)
                if start:
                    request_pub.publish(UInt16(data=3))
                next_send = time.monotonic() + 0.05
            executor.spin_once(timeout_sec=0.01)
            if condition():
                return
        pytest.fail("ROS graph did not reach expected state within its bounded deadline")

    try:
        spin_until(lambda: len(actions) >= 2)
        assert all(m.drive == 0.0 and m.units == DriveAndSteer.UNITS_NORMALIZED for m in actions)
        # Fresh zeros keep arriving after an operator's separate Enable boundary.
        # This observes the policy side of that handshake, not MCU acceptance.
        count_at_enable = len(actions)
        spin_until(lambda: len(actions) > count_at_enable)
        spin_until(lambda: any(m.drive > 0.0 for m in actions), feed=True, start=True)
        spin_until(lambda: node.fsm_state == 2 and actions[-1].drive == 0.0)
        zero_count = len(actions)
        spin_until(lambda: len(actions) > zero_count + 2, feed=True)
        assert node.fsm_state == 2
        assert not pans
        assert not any(name.endswith("/request") for name, _ in node.get_topic_names_and_types())
    finally:
        executor.remove_node(peer)
        executor.remove_node(node)
        node.destroy_node()
        peer.destroy_node()
        executor.shutdown()
        context.shutdown()
