"""Hardware-free checks of the local map of remembered cones, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_local_map.py
"""
import dataclasses
import json
import math
import random

import pytest

from policy.input_output import (AngularVelocity, CarObservations, CarOrientation, ConeBatch, ConeColour, ConeDetection,
                                 ObservedCarState, PolicyState, Position, SensorAge, WheelSpeed)
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector
from policy.lane_detection.lanes import LaneStatus
from policy.local_map.local_map import LocalMap, LocalMapConfig, dead_reckon
from policy.memory import CarMemory, CarPose
from policy.policy_runner import MovementPolicy
from policy.action_policy.mpc import MpcConfig
from policy.cone_filter.cone_filter import ConeFilterConfig
from policy.control.control import ControlConfig

BLUE, YELLOW = ConeColour.BLUE, ConeColour.YELLOW
CONFIG = LocalMapConfig()
DT_S = 0.1
SPEED_M_PER_S = 0.5


def lane(length_m=8.0, width_m=1.0, spacing_m=0.3):
    """(colour, x, y) of the cones of a straight lane along the map's +x axis from x = 0, blue on the left."""
    return [(colour, i * spacing_m, side * width_m / 2.0)
            for i in range(round(length_m / spacing_m) + 1) for colour, side in ((BLUE, 1.0), (YELLOW, -1.0))]


def body(pose, x, y):
    """A map point in base_link at the pose (x, y, heading)."""
    car_x, car_y, heading = pose
    dx, dy = x - car_x, y - car_y
    return math.cos(heading) * dx + math.sin(heading) * dy, -math.sin(heading) * dx + math.cos(heading) * dy


def camera(cones, pose, hidden=(), noise_m=0.0, rng=None):
    """The cones a camera at the pose sees, in base_link: 0.4-2.5 m ahead, within 43 degrees of straight ahead, and
    not hidden (indices into cones)."""
    seen = []
    for index, (colour, x, y) in enumerate(cones):
        bx, by = body(pose, x, y)
        if index not in hidden and 0.4 <= bx <= 2.5 and abs(math.atan2(by, bx)) <= math.radians(43.0):
            seen.append(ConeDetection(Position(bx + (rng.gauss(0.0, noise_m) if noise_m else 0.0),
                                               by + (rng.gauss(0.0, noise_m) if noise_m else 0.0), 0.05), colour, 0.9))
    return seen


def observe(cones, elapsed_s, first=False, wheel_speed=SPEED_M_PER_S, orientation=None, ang_velocity=None):
    speed = None if wheel_speed is None else WheelSpeed(wheel_speed, SensorAge(0.0, None))
    return CarObservations(None if cones is None else ConeBatch(cones, SensorAge(0.1, None), 0.0), None, None, None,
                           ObservedCarState(orientation, ang_velocity, None, speed),
                           PolicyState(0.0 if first else DT_S, elapsed_s, first))


def drive(local_map, memory, cones, steps, pose=(0.0, 0.0, 0.0), curvature_per_m=0.0, requested_curvature_per_m=0.0,
          hidden=(), noise_m=0.0, start=0):
    """Drives the car at SPEED_M_PER_S along an arc of curvature_per_m, telling the local map that it asked for
    requested_curvature_per_m. :return: The car's true pose and the last step's recalled cones."""
    rng = random.Random(0)
    recalled = []
    for step in range(start, start + steps):
        if step > 0:
            distance_m = SPEED_M_PER_S * DT_S
            x, y, heading = pose
            middle = heading + curvature_per_m * distance_m / 2.0
            pose = (x + distance_m * math.cos(middle), y + distance_m * math.sin(middle),
                    heading + curvature_per_m * distance_m)
        recalled = local_map.step(observe(camera(cones, pose, hidden, noise_m, rng), step * DT_S, first=step == 0),
                                  memory, SPEED_M_PER_S, requested_curvature_per_m)
    return pose, recalled


def nearest_error(cone, cones, pose):
    """Distance in metres from a recalled cone to the nearest true cone of its colour, in base_link."""
    return min(math.dist((cone.pos.x, cone.pos.y), body(pose, x, y)) for colour, x, y in cones if colour == cone.colour)


# ---------------------------------------------------------------------------------------------------------------
# Dead reckoning
# ---------------------------------------------------------------------------------------------------------------
def test_dead_reckoning_follows_the_requested_curvature_without_an_imu():
    pose = CarPose(0.0, 0.0, 0.0)
    for step in range(20):
        pose = dead_reckon(pose, observe([], step * DT_S), None, SPEED_M_PER_S, 0.5)
    # 1 m around a 2 m radius to the left
    assert (pose.x_m, pose.y_m, pose.heading_rad) == pytest.approx(
        (2.0 * math.sin(0.5), 2.0 * (1.0 - math.cos(0.5)), 0.5), abs=1e-9)


def test_dead_reckoning_prefers_the_imu_heading_then_its_yaw_rate():
    start = CarPose(0.0, 0.0, 0.0)
    orientation = CarOrientation(0.0, 0.0, 0.25, SensorAge(0.0, None))
    gyro = AngularVelocity(0.0, 0.0, -1.0, SensorAge(0.0, None))
    # The change of the IMU's relative heading since the previous step, even though the car asked to turn right
    turned = dead_reckon(start, observe([], 0.1, orientation=orientation, ang_velocity=gyro), 0.2, 0.0, -2.0)
    assert turned.heading_rad == pytest.approx(0.05)
    # Without a previous heading, the yaw rate over the step
    assert dead_reckon(start, observe([], 0.1, orientation=orientation, ang_velocity=gyro), None, 0.0,
                       2.0).heading_rad == pytest.approx(-0.1)
    # Without a wheel speed, the speed asked for
    moved = dead_reckon(start, observe([], 0.1, wheel_speed=None), None, 0.3, 0.0)
    assert (moved.x_m, moved.y_m) == pytest.approx((0.03, 0.0))


# ---------------------------------------------------------------------------------------------------------------
# Remembering and recalling cones
# ---------------------------------------------------------------------------------------------------------------
def test_cones_beside_the_car_are_recalled_once_out_of_view():
    cones, memory = lane(), CarMemory()
    pose, recalled = drive(LocalMap(CONFIG), memory, cones, 21)
    assert pose[0] == pytest.approx(1.0)
    seen = camera(cones, pose)
    # Each recalled cone is a real cone the camera does not see now, and none is farther behind than kept
    assert recalled
    for cone in recalled:
        assert nearest_error(cone, cones, pose) < 1e-6
        assert all(math.dist((cone.pos.x, cone.pos.y), (other.pos.x, other.pos.y)) > 0.1 for other in seen)
        assert cone.pos.x >= -CONFIG.cone_keep_behind_m - 1e-9
    # Both rows are recalled beside the car, where the camera cannot see
    beside = {cone.colour for cone in recalled if abs(cone.pos.x) <= 0.3}
    assert beside == {BLUE, YELLOW}
    # The memory holds them in the map frame, and forgets those behind
    assert all(cone.x_m >= pose[0] - CONFIG.cone_keep_behind_m - 1e-9 for cone in memory.data.cones)


def test_a_cone_seen_once_is_neither_recalled_nor_kept():
    cones, memory = lane(), CarMemory()
    local_map = LocalMap(CONFIG)
    pose, _ = drive(local_map, memory, cones, 5)
    stray = ConeDetection(Position(1.0, 0.0, 0.05), BLUE, 0.4)
    local_map.step(observe(camera(cones, pose) + [stray], 0.5), memory, 0.0, 0.0)
    assert any(cone.sightings == 1 for cone in memory.data.cones)
    for step in range(6, 13):
        recalled = local_map.step(observe(camera(cones, pose), step * DT_S), memory, 0.0, 0.0)
        assert all(abs(cone.pos.y) > 0.3 for cone in recalled)
    # Not confirmed within UNCONFIRMED_KEEP_S, so forgotten
    assert all(abs(cone.y_m) > 0.3 for cone in memory.data.cones)


def test_cones_hidden_behind_an_obstacle_are_recalled_and_steady_the_lane():
    cones, memory = lane(), CarMemory()
    local_map = LocalMap(CONFIG)
    pose, _ = drive(local_map, memory, cones, 6)
    # A box hides every right cone more than 1.0 m ahead
    hidden = {index for index, (colour, x, _) in enumerate(cones) if colour == YELLOW and x > pose[0] + 1.0}
    pose, recalled = drive(local_map, memory, cones, 4, pose, hidden=hidden, start=6)
    seen = camera(cones, pose, hidden)
    hidden_recalled = [cone for cone in recalled if cone.colour == YELLOW and cone.pos.x > 1.0]
    assert hidden_recalled and all(nearest_error(cone, cones, pose) < 1e-6 for cone in hidden_recalled)
    # With them the lane is measured with both edges, at its true width
    detector = LaneDetector(LaneDetectionConfig(max_curvature_per_m=2.0))
    lane_seen = detector.detect(observe(seen, 1.0), recalled)
    assert lane_seen.status == LaneStatus.BOTH_EDGES
    assert lane_seen.lane_width_m == pytest.approx(1.0, abs=0.01)
    assert lane_seen.lateral_error_m == pytest.approx(0.0, abs=0.01)


@pytest.mark.parametrize("noise_m, within_m", [(0.01, 0.03), (0.03, 0.06)])
def test_the_pose_is_corrected_against_the_remembered_cones(noise_m, within_m):
    # The car drives straight, but dead reckoning thinks it turns left at 0.2 1/m (11 degrees per metre)
    cones, memory = lane(), CarMemory()
    pose, recalled = drive(LocalMap(CONFIG), memory, cones, 41, requested_curvature_per_m=0.2, noise_m=noise_m)
    assert pose[0] == pytest.approx(2.0)
    # Dead reckoning alone would have turned 0.4 rad. The map turns a little with the pose, which does not matter:
    # the recalled cones stay where the cones are around the car, within about the detector's noise
    assert abs(memory.data.pose.heading_rad) < 0.15
    beside = [cone for cone in recalled if abs(cone.pos.x) <= 0.4]
    assert beside and max(nearest_error(cone, cones, pose) for cone in recalled) < within_m


def test_without_cones_matching_the_pose_drifts_as_dead_reckoning_does():
    memory = CarMemory()
    drive(LocalMap(CONFIG), memory, [], 41, requested_curvature_per_m=0.2)
    assert memory.data.pose.heading_rad == pytest.approx(0.4)


def test_the_map_starts_again_when_the_batch_no_longer_matches_it():
    cones, memory = lane(), CarMemory()
    local_map = LocalMap(CONFIG)
    pose, _ = drive(local_map, memory, cones, 10)
    # The car slides 0.4 m to the left without dead reckoning noticing
    slid = (pose[0], pose[1] + 0.4, pose[2])
    recalled = local_map.step(observe(camera(cones, slid), 1.0), memory, 0.0, 0.0)
    assert recalled == []
    assert all(cone.sightings == 1 for cone in memory.data.cones)
    assert len(memory.data.cones) == len(camera(cones, slid))


def test_a_new_run_starts_a_new_map():
    cones, memory = lane(), CarMemory()
    local_map = LocalMap(CONFIG)
    drive(local_map, memory, cones, 10)
    local_map.step(observe([], 0.0, first=True), memory, 0.0, 0.0)
    assert memory.data.pose == CarPose(0.0, 0.0, 0.0) and memory.data.cones == ()


def test_switched_off_the_map_is_kept_but_nothing_is_recalled():
    memory = CarMemory()
    local_map = LocalMap(dataclasses.replace(CONFIG, use_remembered_cones=False))
    _, recalled = drive(local_map, memory, lane(), 21)
    assert recalled == [] and memory.data.cones and local_map.map_cones


def test_without_a_batch_the_pose_still_moves_and_nothing_is_recalled():
    cones, memory = lane(), CarMemory()
    local_map = LocalMap(CONFIG)
    drive(local_map, memory, cones, 10)
    before = memory.data.pose
    assert local_map.step(observe(None, 1.0), memory, SPEED_M_PER_S, 0.0) == []
    assert memory.data.pose.x_m == pytest.approx(before.x_m + SPEED_M_PER_S * DT_S)
    assert len(memory.data.cones) > 0


@pytest.mark.parametrize("change", [{"use_remembered_cones": 1}, {"cone_match_distance_m": 0.0},
                                    {"cone_min_sightings": 0}, {"cone_min_sightings": 2.0},
                                    {"cone_keep_behind_m": -0.1}, {"cone_keep_s": math.inf}])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        LocalMapConfig(**change)


# ---------------------------------------------------------------------------------------------------------------
# Lane detection with recalled cones
# ---------------------------------------------------------------------------------------------------------------
def test_recalled_cones_never_make_a_lane_the_batch_does_not_give():
    recalled = [ConeDetection(Position(x, side * 0.5, 0.0), colour, 0.9)
                for x in (-0.3, 0.0, 0.3) for colour, side in ((BLUE, 1.0), (YELLOW, -1.0))]
    detector = LaneDetector(LaneDetectionConfig())
    # One cone ahead gives no lane alone, so the end of the rows is still seen as before
    lane_seen = detector.detect(observe([ConeDetection(Position(1.0, 0.5, 0.05), BLUE, 0.9)], 0.0), recalled)
    assert lane_seen.status == LaneStatus.NO_LANE
    assert detector.detect(observe(None, 0.0), recalled) is None


def test_recalled_cones_complete_an_edge_the_batch_shows_too_little_of():
    seen = [ConeDetection(Position(x, 0.5, 0.05), BLUE, 0.9) for x in (0.8, 1.1, 1.4, 1.7)]
    recalled = [ConeDetection(Position(x, -0.5, 0.0), YELLOW, 0.9) for x in (-0.3, 0.0, 0.3, 0.6)]
    detector = LaneDetector(LaneDetectionConfig(lane_width_m=1.4))
    alone = detector.detect(observe(seen, 0.0))
    assert alone.status == LaneStatus.LEFT_EDGE_ONLY and alone.lane_width_m == pytest.approx(1.4)
    both = LaneDetector(LaneDetectionConfig(lane_width_m=1.4)).detect(observe(seen, 0.0), recalled)
    assert both.status == LaneStatus.BOTH_EDGES and both.lane_width_m == pytest.approx(1.0)
    assert both.right_edge.nearest_m == pytest.approx(-0.3)


# ---------------------------------------------------------------------------------------------------------------
# In the movement policy
# ---------------------------------------------------------------------------------------------------------------
def test_the_movement_policy_remembers_cones_in_the_car_memory():
    cones, memory = lane(), CarMemory()
    policy = MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig())
    pose = (0.0, 0.0, 0.0)
    for step in range(15):
        pose = (step * SPEED_M_PER_S * DT_S, 0.0, 0.0)
        policy.step(observe(camera(cones, pose), step * DT_S, first=step == 0), memory)
        memory.step()
    assert memory.data.pose.x_m == pytest.approx(pose[0], abs=0.05)
    assert policy.local_map.recalled and policy.lanes.status == LaneStatus.BOTH_EDGES
    assert {cone.colour for cone in policy.local_map.recalled if cone.pos.x < 0.4} == {BLUE, YELLOW}
    # The memory log holds the map, and loads again
    data = json.loads(json.dumps(memory.data.serialise()))
    assert CarMemory.deserialise(data).data == memory.data
