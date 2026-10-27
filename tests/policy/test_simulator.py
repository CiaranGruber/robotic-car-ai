"""Hardware-free checks of the car simulator (tests/policy/simulator), without ROS or a browser.

Run from the repository root with: python3 -m pytest tests/policy/test_simulator.py
"""
from __future__ import annotations

import json
import math
import struct
import urllib.request

import numpy as np
import pytest

from tests.policy.simulator.live import DEFAULT_TRANSFORMS, rotate
from tests.policy.simulator.ros2_cdr import MessageDecoder
from tests.policy.simulator.sensors import SensorSettings, Sensors, _marker_quaternion
from tests.policy.simulator.server import SimulationRunner, make_handler, to_json
from tests.policy.simulator.simulation import POLICY_STATE, ZERO_STATE, Simulation, run_settings
from tests.policy.simulator.track import TRACKS_DIR, load_track

BUNDLED = sorted(path.stem for path in TRACKS_DIR.glob("*.yaml"))


def run(sim: Simulation) -> Simulation:
    while not sim.finished:
        sim.step()
    return sim


@pytest.mark.parametrize("name", BUNDLED)
def test_every_bundled_track_loads(name):
    track = load_track(name)
    assert track.cones and track.roads and track.duration_s > 0
    assert all(0 <= m.id <= 49 and m.size_m > 0 for m in track.markers)


def test_the_lab_straight_drives_to_the_end_without_touching_a_cone():
    sim = run(Simulation(load_track("lab_straight")))
    summary = sim.summary()
    last_pair = max(c.x for c in sim.track.cones)
    assert sim.ever_started and summary.cones_touched == 0
    # The Stage 1 stop zone: 0 to 0.5 m past the last cone pair, with a little rounding
    assert 0.0 <= sim.car.state.x - last_pair <= 0.6
    assert abs(sim.car.state.y) < 0.2


def test_the_node_refuses_to_start_without_a_required_sensor():
    track = load_track("lab_straight")
    sim = Simulation(track, run_settings(track, {"required_sensors": ["cone_detections", "rc_drive_and_steer"]}))
    for _ in range(300):
        sim.step()
    assert sim.node_state == ZERO_STATE and not sim.ever_started
    assert sim.node_reason == "Policy start refused: Required sensor rc_drive_and_steer is missing or stale"
    assert sim.car.state.distance_m == 0.0


def test_cone_batches_reach_the_policy_with_the_measured_latency():
    sim = Simulation(load_track("lab_straight"))
    ages = []
    while sim.time_s < 4.0:
        if sim.step()["policy"]:
            ages.append(float(sim.observations.cones.sensor_age))
    assert sim.node_state == POLICY_STATE
    # One policy step per cone batch (policy_update_mode: cone_detection), each about 0.2 s old
    assert len(ages) == pytest.approx(30, abs=2)
    assert all(0.19 <= age <= 0.21 for age in ages)


def test_a_right_turn_sign_at_the_cross_takes_the_right_exit():
    sim = run(Simulation(load_track("cross", marker_id=4)))
    assert any(e.text == "marker command TURN_RIGHT" for e in sim.events)
    assert sim.car.state.y < -3.0 and abs(math.degrees(sim.car.state.heading) + 90.0) < 15.0
    assert not sim.cones_touched


def test_a_sign_for_the_closed_road_of_a_t_stops_the_car_before_the_junction():
    sim = run(Simulation(load_track("t_junction", marker_id=5)))
    assert sim.memory.data.intersection.stop_reason == "requested exit is closed by a supported cone row"
    assert sim.car.state.x < -1.2 and not sim.cones_touched


def test_markers_are_decoded_only_after_the_detectors_consecutive_frames():
    track = load_track("cross")
    sensors = Sensors(track, SensorSettings(noise=False, min_consecutive_frames=3), np.random.default_rng(0))
    pose = (track.markers[0].x - 1.0, 0.0, 0.0)
    published = [bool(sensors.camera_markers(pose)) for _ in range(4)]
    assert published == [False, False, True, True]
    detection = sensors.camera_markers(pose)[0]
    assert detection.id == track.markers[0].id
    assert detection.pose.position.x == pytest.approx(1.0) and detection.pose.position.y == pytest.approx(-0.7)


def test_a_marker_pose_points_its_z_axis_out_of_its_face():
    q = _marker_quaternion(math.radians(150.0))
    assert math.hypot(q.x, q.y, q.z, q.w) == pytest.approx(1.0)
    z = rotate((q.x, q.y, q.z, q.w), (0.0, 0.0, 1.0))
    assert z == pytest.approx((math.cos(math.radians(150.0)), math.sin(math.radians(150.0)), 0.0), abs=1e-9)
    assert rotate((q.x, q.y, q.z, q.w), (0.0, 1.0, 0.0)) == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)


def test_the_default_camera_transform_looks_forwards_and_20_degrees_down():
    _, rotation = DEFAULT_TRANSFORMS["camera"]
    forward = rotate(rotation, (0.0, 0.0, 1.0))  # The optical frame looks along its z axis
    assert forward == pytest.approx((math.cos(math.radians(20)), 0.0, -math.sin(math.radians(20))), abs=1e-3)


DEFINITION = """std_msgs/Header header
ConeDetection[] detections
float32 acquisition_to_publish_latency_s
================================================================================
MSG: test_msgs/ConeDetection
uint8 COLOR_YELLOW=0
uint8 COLOR_BLUE=1
uint8 color
geometry_msgs/Point position
float32 classification_confidence
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
"""


def _cdr(cones: list[tuple[int, float, float, float, float]]) -> bytes:
    """A hand-written little-endian CDR message for DEFINITION, with its alignment padding."""
    out = bytearray(b"\x00\x01\x00\x00")
    pad = lambda n: out.extend(b"\0" * ((-(len(out) - 4)) % n))
    out += struct.pack("<iI", 7, 500)
    frame = b"base_link\0"
    out += struct.pack("<I", len(frame)) + frame
    pad(4)
    out += struct.pack("<I", len(cones))
    for colour, x, y, z, confidence in cones:
        out += struct.pack("<B", colour)
        pad(8)
        out += struct.pack("<ddd", x, y, z)
        out += struct.pack("<f", confidence)
    pad(4)
    out += struct.pack("<f", 0.08)
    return bytes(out)


def test_ros_messages_are_decoded_from_their_definitions():
    decoder = MessageDecoder("test_msgs/msg/ConeDetections", DEFINITION)
    message = decoder.decode(_cdr([(1, 1.5, 0.4, 0.1, 0.9), (0, 2.0, -0.4, 0.1, 0.8)]))
    assert message["header"] == {"stamp": {"sec": 7, "nanosec": 500}, "frame_id": "base_link"}
    assert [c["color"] for c in message["detections"]] == [1, 0]
    assert message["detections"][1]["position"] == {"x": 2.0, "y": -0.4, "z": pytest.approx(0.1)}
    assert message["acquisition_to_publish_latency_s"] == pytest.approx(0.08)
    assert decoder.constants["test_msgs/ConeDetection"] == {"COLOR_YELLOW": "0", "COLOR_BLUE": "1"}


def test_viewer_frames_are_valid_json_without_nan():
    assert json.loads(to_json({"a": math.nan, "b": [1.0, math.inf], "c": (2, 3)})) == {"a": None, "b": [1.0, None],
                                                                                      "c": [2, 3]}


def test_the_viewer_server_serves_the_page_the_scene_and_live_frames():
    from http.server import ThreadingHTTPServer
    import threading
    runner = SimulationRunner("aruco_lane", speed=4.0)
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(runner))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    runner.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        assert b"AI4R Car Simulator" in urllib.request.urlopen(base + "/").read()
        scene = json.loads(urllib.request.urlopen(base + "/scene").read())
        assert scene["mode"] == "sim" and len(scene["track"]["cones"]) == len(runner.sim.track.cones)
        assert [m["command"] for m in scene["track"]["markers"]] == ["KEEP_LEFT", "KEEP_CENTRE"]
        assert len(scene["track"]["markers"][0]["bits"]) == 6
        with urllib.request.urlopen(base + "/stream", timeout=5) as stream:
            line = next(line for line in stream if line.startswith(b"data: "))
        frame = json.loads(line[len(b"data: "):])
        assert frame["mode"] == "sim" and "car" in frame and "layers" in frame
        request = urllib.request.Request(base + "/control", json.dumps({"op": "pause"}).encode(), method="POST")
        urllib.request.urlopen(request).read()
    finally:
        runner.stop_requested = True
        server.shutdown()


def test_hand_driving_turns_the_way_the_key_says():
    runner = SimulationRunner("lab_straight")
    runner.manual_keys = (1, 1)
    drive, steering = runner._manual_command()
    model = runner.sim.car.model
    assert drive > model.breakaway_drive and model.curvature_per_m(steering) > 0.0  # forwards and to the left
    runner.manual_keys = (0, -1)
    assert model.curvature_per_m(runner._manual_command()[1]) < 0.0
