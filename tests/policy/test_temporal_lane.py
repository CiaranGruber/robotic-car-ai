"""Motion/frame and rejection regressions for the opt-in temporal lane geometry, without ROS."""
from dataclasses import replace
import math

import pytest

from policy.input_output import (AngularVelocity, CarObservations, ConeBatch, ConeColour, ConeDetection,
                                 ObservedCarState, PolicyState, Position, RcInput, SensorAge, WheelSpeed)
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector
from policy.lane_detection.temporal import detect_temporal_lane
from policy.memory import CarMemory
from policy.memory import MotionPose
from policy.lane_detection.temporal import _pose_at
from tests.policy.test_lane_detection import lane_cones


def observation(t, cones, speed=0.5, yaw_rate=0.0, age=0.0, stamp=None, dt=0.1):
    return CarObservations(
        None if cones is None else ConeBatch(cones, SensorAge(age, round((10 + t - age) * 1e9)
                                                          if stamp is None else stamp), 0.0),
        None, None, None,
        ObservedCarState(None, AngularVelocity(0.0, 0.0, yaw_rate, SensorAge(0.0, None)), None,
                         WheelSpeed(speed, SensorAge(0.0, None))),
        PolicyState(0.0 if t == 0 else dt, t, t == 0))


def detector(**changes):
    return LaneDetector(replace(LaneDetectionConfig(), memory_enabled=True, **changes))


def test_remembered_cone_moves_in_translation_and_rotation_and_fresh_batch_is_unchanged():
    memory, fit = CarMemory(), detector()
    initial = [ConeDetection(Position(1.0, 0.5, 0.05), ConeColour.BLUE, 0.9)]
    detect_temporal_lane(observation(0, initial, yaw_rate=1.0), memory, fit)
    current = observation(0.1, [], yaw_rate=1.0)
    lane, augmented = detect_temporal_lane(current, memory, fit)
    dx, dy = 0.5 * math.sin(0.1), 0.5 * (1 - math.cos(0.1))
    expected_x = math.cos(0.1) * (1 - dx) + math.sin(0.1) * (0.5 - dy)
    expected_y = -math.sin(0.1) * (1 - dx) + math.cos(0.1) * (0.5 - dy)
    assert len(augmented) == 1 and current.cones == []
    assert augmented[0].pos.x == pytest.approx(expected_x)
    assert augmented[0].pos.y == pytest.approx(expected_y)
    assert not lane.has_lane  # Memory alone cannot make a blind lane.


def test_latency_uses_acquisition_pose_and_preserves_age_without_double_compensation():
    memory, fit = CarMemory(), detector()
    cone = ConeDetection(Position(1, 0.5, 0.05), ConeColour.BLUE, 0.9)
    detect_temporal_lane(observation(0, [cone], age=0.1), memory, fit)
    assert not memory.data.lane_tracking.tracks  # No extrapolation before motion history.
    detect_temporal_lane(observation(0.1, [cone], age=0.1), memory, fit)
    obs = observation(0.2, [], age=0.1)
    _, cones = detect_temporal_lane(obs, memory, fit)
    assert cones[0].pos.x == pytest.approx(0.95)  # Only acquisition-to-acquisition travel, not 0.1 m.
    assert cones.sensor_age is obs.cones.sensor_age


def test_duplicate_measurement_does_not_refresh_or_reinsert_expired_cones():
    memory, fit = CarMemory(), detector(memory_max_age_s=0.25)
    seen = lane_cones()
    initial = observation(0, seen)
    detect_temporal_lane(initial, memory, fit)
    for t in (0.1, 0.2, 0.3):
        reused = replace(observation(t, seen, age=t), cones=ConeBatch(seen, SensorAge(t, initial.cones.sensor_age.stamp_ns), 0))
        detect_temporal_lane(reused, memory, fit)
    assert not memory.data.lane_tracking.tracks


@pytest.mark.parametrize("failure", ["missing_speed", "stale_gyro", "reverse_rc", "gap", "nan", "missing_cones", "backwards_stamp"])
def test_unknown_motion_or_stream_loss_clears_memory(failure):
    memory, fit = CarMemory(), detector()
    detect_temporal_lane(observation(0, lane_cones()), memory, fit)
    obs = observation(0.1, [])
    if failure == "missing_speed":
        obs = replace(obs, car=replace(obs.car, wheel_speed=None))
    elif failure == "stale_gyro":
        obs = replace(obs, car=replace(obs.car, ang_velocity=replace(obs.car.ang_velocity, sensor_age=SensorAge(1, None))))
    elif failure == "reverse_rc":
        obs = replace(obs, rc=RcInput(-0.1, 0, SensorAge(0, None)))
    elif failure == "gap":
        obs = observation(0.5, [], dt=0.5)
    elif failure == "nan":
        obs = observation(0.1, [], yaw_rate=math.nan)
    elif failure == "missing_cones":
        obs = observation(0.1, None)
    else:
        obs = observation(0.1, [], stamp=1)
    detect_temporal_lane(obs, memory, fit)
    assert not memory.data.lane_tracking.tracks
    assert memory.data.lane_tracking.reference is None


def test_track_association_expiry_distance_and_count_are_bounded():
    memory, fit = CarMemory(), detector(memory_max_distance_m=0.06, memory_max_tracks=4)
    cones = lane_cones()
    detect_temporal_lane(observation(0, cones), memory, fit)
    assert len(memory.data.lane_tracking.tracks) == 4
    moved = [replace(c, pos=replace(c.pos, x=c.pos.x - 0.05)) for c in cones]
    detect_temporal_lane(observation(0.1, moved), memory, fit)
    assert len(memory.data.lane_tracking.tracks) == 4
    detect_temporal_lane(observation(0.2, []), memory, fit)
    assert len(memory.data.lane_tracking.tracks) == 4
    detect_temporal_lane(observation(0.3, []), memory, fit)
    assert not memory.data.lane_tracking.tracks


def test_jump_rejected_without_poisoning_width_and_consistent_lane_recovers():
    memory, fit = CarMemory(), detector(continuity_enabled=True, continuity_lateral_slack_m=0.03)
    lane, _ = detect_temporal_lane(observation(0, lane_cones()), memory, fit)
    assert lane.has_lane
    reference, width = memory.data.lane_tracking.reference, fit.lane_width_m
    jumped, _ = detect_temporal_lane(observation(0.1, lane_cones(offset_m=0.45, width_m=1.4)), memory, fit)
    assert not jumped.has_lane and memory.data.lane_tracking.rejected
    assert memory.data.lane_tracking.reference == reference and fit.lane_width_m == width
    recovered, _ = detect_temporal_lane(observation(0.2, lane_cones()), memory, fit)
    assert recovered.has_lane


def test_expired_reference_does_not_admit_a_different_lane_and_new_run_resets():
    memory, fit = CarMemory(), LaneDetector(LaneDetectionConfig(continuity_enabled=True))
    detect_temporal_lane(observation(0, lane_cones()), memory, fit)
    for i in range(1, 9):
        lane, _ = detect_temporal_lane(observation(i / 10, lane_cones(offset_m=0.6)), memory, fit)
        assert not lane.has_lane
    restarted = replace(observation(0, lane_cones(offset_m=0.6)), policy=PolicyState(0, 0, True))
    lane, _ = detect_temporal_lane(restarted, memory, fit)
    assert lane.has_lane


def test_motion_compensated_guard_accepts_real_vehicle_turn_and_serialises_state():
    memory, fit = CarMemory(), detector(continuity_enabled=True, max_curvature_per_m=2.0)
    fixed = lane_cones(radius_m=2.0)
    for i in range(5):
        t = i / 10
        angle = 0.25 * t
        x, y = 2 * math.sin(angle), 2 * (1 - math.cos(angle))
        moved = [replace(c, pos=Position(math.cos(angle) * (c.pos.x - x) + math.sin(angle) * (c.pos.y - y),
                                         -math.sin(angle) * (c.pos.x - x) + math.cos(angle) * (c.pos.y - y), c.pos.z))
                 for c in fixed]
        lane, _ = detect_temporal_lane(observation(t, moved, yaw_rate=0.25), memory, fit)
        assert lane.has_lane
    restored = CarMemory.deserialise(memory.data.serialise())
    assert restored.data == memory.data
    assert CarMemory.deserialise({"policy_step": 2}).data.lane_tracking.tracks == ()


def test_fresh_outlier_cannot_renew_a_lane_fitted_only_to_remembered_cones():
    memory, fit = CarMemory(), detector(continuity_enabled=True)
    detect_temporal_lane(observation(0, lane_cones()), memory, fit)
    reference = memory.data.lane_tracking.reference
    outlier = ConeDetection(Position(2, 1.5, 0.05), ConeColour.BLUE, 0.9)
    lane, _ = detect_temporal_lane(observation(0.1, [outlier]), memory, fit)
    assert not lane.has_lane
    assert memory.data.lane_tracking.reference == reference


def test_partial_occlusion_keeps_inner_edge_but_does_not_extend_fresh_horizon():
    memory, fit = CarMemory(), detector(continuity_enabled=True)
    detect_temporal_lane(observation(0, lane_cones()), memory, fit)
    one = [ConeDetection(Position(0.95, -0.5, 0.05), ConeColour.YELLOW, 0.9)]
    obs = observation(0.1, one)
    assert not LaneDetector(LaneDetectionConfig()).detect(obs).has_lane
    lane, _ = detect_temporal_lane(obs, memory, fit)
    assert lane.has_lane and lane.left_edge is not None and lane.right_edge is not None
    assert lane.visible_until_m == pytest.approx(0.95)
    assert obs.cones == one


def test_new_run_with_missing_motion_resets_remembered_width_and_tracks():
    memory, fit = CarMemory(), detector(continuity_enabled=True)
    detect_temporal_lane(observation(0, lane_cones(width_m=1.5)), memory, fit)
    assert fit.lane_width_m > fit.config.lane_width_m
    obs = observation(0, lane_cones())
    obs = replace(obs, car=replace(obs.car, ang_velocity=None))
    lane, _ = detect_temporal_lane(obs, memory, fit)
    assert not lane.has_lane
    assert fit.lane_width_m == fit.config.lane_width_m
    assert not memory.data.lane_tracking.tracks


def test_acquisition_roundoff_at_history_start_uses_first_pose_not_latest():
    poses = (MotionPose(0.2, 0, 0, 0, 0, 1, 0), MotionPose(0.3, 0.1, 0, 0, 0.1, 1, 0))
    result = _pose_at(poses, 0.3 - 0.1)
    assert result.time_s == pytest.approx(0.2)
    assert result.x == pytest.approx(0)
    assert _pose_at(poses, 0.19) is None


@pytest.mark.parametrize("interval,accepted", [(0.1, False), (0.2, True)])
def test_lateral_threshold_uses_elapsed_seconds(interval, accepted):
    memory = CarMemory()
    fit = LaneDetector(LaneDetectionConfig(continuity_enabled=True))
    detect_temporal_lane(observation(0, lane_cones()), memory, fit)
    lane, _ = detect_temporal_lane(observation(interval, lane_cones(offset_m=0.18), dt=interval), memory, fit)
    assert lane.has_lane == accepted


@pytest.mark.parametrize("field,value", [("memory_max_tracks", 257), ("motion_max_gap_s", 1.0),
                                        ("continuity_max_lateral_rate_m_per_s", math.nan), ("memory_enabled", 1)])
def test_invalid_temporal_settings_rejected(field, value):
    with pytest.raises(ValueError):
        LaneDetectionConfig(**{field: value})
