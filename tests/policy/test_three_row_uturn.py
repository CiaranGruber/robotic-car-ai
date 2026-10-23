"""Offline three-row geometry, memory admission, learned inference and stop regressions."""
from dataclasses import replace
import json
import math

import numpy as np
import pytest

from policy.action_policy.three_row_uturn import (FEATURE_COUNT, FEATURE_VERSION, ThreeRowConfig,
                                                 ThreeRowNetwork, ThreeRowPolicy, turn_path)
from policy.input_output import (ConeColour, ConeDetection, LidarCartesian, LidarCartesianPoint,
                                 Position, SensorAge)
from policy.memory import CarMemory, ConeTrack, MotionPose, ThreeRowTurnMemory
from tests.policy.stage1_scenarios import body_corners
from tests.policy.test_temporal_lane import observation
from tests.policy.three_row_simulation import ThreeRowCase, ThreeRowTrack, simulate
from tests.policy.train_three_row_uturn import check_frozen_inputs, source_hashes


def weights(path, bias=0., **changes):
    arrays = dict(mean=np.zeros(FEATURE_COUNT), scale=np.ones(FEATURE_COUNT),
                  w1=np.zeros((FEATURE_COUNT, 32)), b1=np.zeros(32), w2=np.zeros((32, 32)),
                  b2=np.zeros(32), w3=np.zeros((32, 1)), b3=np.array([bias]),
                  contract=np.array([FEATURE_VERSION, 1., .92, .6]))
    np.savez(path, **{**arrays, **changes})


def rows():
    return [ConeDetection(Position(x, y, .05), colour, .9)
            for y, colour in ((-.5, ConeColour.BLUE), (.5, ConeColour.YELLOW), (1.5, ConeColour.BLUE))
            for x in (.8, 1.3, 1.8)]


def scan(points, age=0.):
    return LidarCartesian([LidarCartesianPoint(Position(x, y, .1), i)
                           for i, (x, y) in enumerate(points)], SensorAge(age, None), "base_link")


def test_layout_has_three_straight_rows_and_exact_outer_extension():
    track = ThreeRowTrack(placement_jitter_m=0.)
    cones = track.row_cones()
    assert {row for row, _, _, _ in cones} == {0, 1, 2}
    assert len({y for _, _, _, y in cones}) == 3
    ends = [max(x for row, _, x, _ in cones if row == i) for i in range(3)]
    assert ends == [4., 3., 4.]


def test_full_body_divider_crossing_is_caught_even_with_all_corners_inside_floor():
    track = ThreeRowTrack()
    corners = body_corners(1., .5, 0.)
    assert all(track.outside_distance(x, y) <= 0 for x, y in corners)
    assert track.footprint_outside_distance(corners) > 0
    assert track.footprint_outside_distance(body_corners(1., 0., 0.)) <= 0


def test_forward_connector_has_bounded_curvature_and_reversed_return_heading():
    points = np.array(turn_path(4.75, 0., 1., 1.18, 0., .2))
    assert points[-1] == pytest.approx((.2, 1.))
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    angles = np.unwrap(np.arctan2(segments[:, 1], segments[:, 0]))
    curvature = np.diff(angles)/((lengths[:-1]+lengths[1:])/2)
    assert max(abs(curvature)) <= 1/1.18+.001
    assert abs(angles[-1]) == pytest.approx(math.pi)
    assert points[:, 0].max() > 6.  # A declared wide manoeuvre, not a fictitious 0.5m-radius semicircle.


@pytest.mark.parametrize("change", [{"lookahead_m": .7}, {"route_max_age_s": 41.},
                                    {"turn_speed_m_per_s": .5}, {"turning_radius_m": .5}])
def test_inconsistent_or_unbounded_configuration_is_rejected(change):
    with pytest.raises(ValueError):
        ThreeRowConfig(**change)


@pytest.mark.parametrize("bias,expected", [(100., 1.), (-100., -.92), (0., 0.)])
def test_network_enforces_asymmetric_steering_limits(tmp_path, bias, expected):
    path = tmp_path/"weights.npz"
    weights(path, bias)
    assert ThreeRowNetwork(path).predict(np.zeros(FEATURE_COUNT)) == pytest.approx(expected)


@pytest.mark.parametrize("change", [{"scale": np.zeros(FEATURE_COUNT)}, {"w3": np.zeros((32, 2))},
                                    {"b1": np.full(32, np.nan)}, {"contract": np.array([999, 1, .92, .6])}])
def test_bad_model_artifacts_are_rejected(tmp_path, change):
    path = tmp_path/"weights.npz"
    weights(path, **change)
    with pytest.raises(ValueError):
        ThreeRowNetwork(path)


@pytest.mark.parametrize("values", [np.zeros(3), np.full(FEATURE_COUNT, np.nan),
                                    np.full(FEATURE_COUNT, 11.)])
def test_bad_or_out_of_distribution_features_cannot_produce_actions(tmp_path, values):
    path = tmp_path/"weights.npz"
    weights(path)
    assert ThreeRowNetwork(path).predict(values) is None


def test_separate_evaluation_rejects_changed_source_config_or_checkpoint(tmp_path):
    from dataclasses import asdict
    import hashlib
    path = tmp_path/"weights.npz"
    weights(path)
    config = ThreeRowConfig()
    report = {"model_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "config": asdict(config), "source_sha256": source_hashes()}
    check_frozen_inputs(report, config, path)
    for change in ({"model_sha256": "changed"}, {"config": {}}, {"source_sha256": {}}):
        with pytest.raises(ValueError):
            check_frozen_inputs({**report, **change}, config, path)


def test_legacy_memory_and_populated_strict_json_roundtrip():
    assert CarMemory.deserialise({"policy_step": 7}).data.three_row_turn == ThreeRowTurnMemory()
    policy = ThreeRowPolicy()
    policy._store(ThreeRowTurnMemory(poses=(MotionPose(1., 2., 3., .2, .3, .4, .1),),
                  tracks=(ConeTrack(2., .5, .05, "YELLOW", .9, 1., .3),),
                  row_ends=(math.nan, -math.inf, 3.), route=((1., 2.), (3., 4.))))
    data = json.loads(json.dumps(policy.memory.data.serialise(), allow_nan=False))
    restored = CarMemory.deserialise(data).data.three_row_turn
    assert restored.poses == policy.state.poses and restored.tracks == policy.state.tracks
    assert restored.route == policy.state.route and math.isnan(restored.row_ends[0])
    assert ThreeRowTurnMemory.deserialise({"stopped": True}).stopped


def test_camera_warmup_keeps_motion_history_for_first_delayed_detection():
    policy = ThreeRowPolicy()
    for i in range(2):
        command = policy.run_policy(observation(i/10, None, speed=0.))[0]
        assert command.speed_m_per_s == 0. and not policy.state.stopped
    policy.run_policy(replace(observation(.2, rows(), speed=0., age=.2), lidar_cartesian=scan([])))
    assert policy.state.tracks and not policy.state.stopped


def test_repeated_frame_stops_and_cannot_refresh_observation_time():
    policy = ThreeRowPolicy()
    policy.run_policy(replace(observation(0., rows()), lidar_cartesian=scan([])))
    previous = policy.state.last_acquisition_s
    command = policy.run_policy(observation(.1, rows(), stamp=int(10e9)))[0]
    assert policy.state.stopped and "repeated" in policy.state.stop_reason
    assert command.brake and command.speed_m_per_s == 0.
    assert policy.state.last_acquisition_s == previous


def test_path_loss_after_motion_latches_even_when_already_stationary_and_fresh_cones_cannot_restart():
    policy = ThreeRowPolicy()
    first = replace(observation(0., rows(), speed=0.), lidar_cartesian=scan([]))
    assert policy.run_policy(first)[0].speed_m_per_s > 0
    assert policy.state.has_driven
    # Exhaust the remembered approach support with the car already stationary. Recovery must not
    # silently restart merely because the measured speed has fallen below a latching threshold.
    policy._store(replace(policy.state, tracks=(), row_offsets=(), route=()))
    command = policy.run_policy(replace(observation(.1, [], speed=0.), lidar_cartesian=scan([])))[0]
    assert policy.state.stopped and command.brake and command.speed_m_per_s == 0.
    recovered = replace(observation(.2, rows(), speed=0.), lidar_cartesian=scan([]))
    assert policy.run_policy(recovered)[0].speed_m_per_s == 0.
    assert policy.state.stopped
    assert policy.run_policy(first)[0].speed_m_per_s > 0.  # Explicit new-run reset.


def test_lidar_age_transports_points_to_current_body_pose_and_rejects_nonfinite_points():
    policy = ThreeRowPolicy()
    policy._store(ThreeRowTurnMemory(poses=(MotionPose(0., 0., 0., 0., 0., .4, 0.),
                                            MotionPose(.1, .04, 0., 0., .04, .4, 0.))))
    obs = observation(.1, [], speed=.4)
    assert policy._safe(0., replace(obs, lidar_cartesian=scan([(.99, 0.)])))
    assert not policy._safe(0., replace(obs, lidar_cartesian=scan([(.99, 0.)], age=.1)))
    assert not policy._safe(0., replace(obs, lidar_cartesian=scan([(math.nan, 0.)])))


@pytest.mark.parametrize("failure", ["lidar_nan", "lidar_stale", "lidar_blocked", "gyro_stale", "wheel_missing"])
def test_invalid_sensors_latch_zero_brake_without_exception(failure):
    policy = ThreeRowPolicy()
    obs = replace(observation(0., rows()), lidar_cartesian=scan([]))
    if failure == "lidar_nan":
        obs = replace(obs, lidar_cartesian=scan([(math.nan, 0.)]))
    elif failure == "lidar_stale":
        obs = replace(obs, lidar_cartesian=scan([], age=1.))
    elif failure == "lidar_blocked":
        obs = replace(obs, lidar_cartesian=scan([(.3, 0.)]))
    elif failure == "gyro_stale":
        obs = replace(obs, car=replace(obs.car, ang_velocity=replace(obs.car.ang_velocity,
                                           sensor_age=SensorAge(1., None))))
    else:
        obs = replace(obs, car=replace(obs.car, wheel_speed=None))
    command = policy.run_policy(obs)[0]
    assert policy.state.stopped and command.brake and command.speed_m_per_s == 0.


def return_state(seen=1.):
    return ThreeRowTurnMemory(poses=(MotionPose(2., 3., 1., math.pi, 1., .4, 0.),),
                row_heading=0., row_offsets=(-.5, .5, 1.5), route=((3., 1.), (0., 1.)),
                route_seen_s=seen, route_seen_distance_m=0.)


def return_cones():
    return [ConeDetection(Position(x, y, .05), colour, .9)
            for y, colour in ((.5, ConeColour.YELLOW), (-.5, ConeColour.BLUE)) for x in (1., 1.5)]


def test_only_fresh_separated_support_from_both_return_boundaries_renews_anchor():
    policy = ThreeRowPolicy()
    policy._store(return_state())
    pose = policy.state.poses[-1]
    policy._fresh_return(pose, pose, return_cones()[:2])
    assert policy.state.route_seen_s == 1.
    policy._fresh_return(pose, pose, return_cones())
    assert policy.state.route_seen_s == 2.
    assert len(policy.state.route) > 2


def test_expired_route_stops_before_fresh_return_can_renew_it():
    policy = ThreeRowPolicy()
    policy._store(return_state(seen=-40.))
    obs = replace(observation(2.1, return_cones(), speed=.4), lidar_cartesian=scan([]))
    command = policy.run_policy(obs)[0]
    assert policy.state.stop_reason == "observed route expired"
    assert policy.state.route_seen_s == -40. and command.brake and policy.state.stopped


def sparse_rows(gap=1., blue_slope=0.):
    return [ConeDetection(Position(x, y, .05), colour, .9)
            for colour, pairs in ((ConeColour.YELLOW, ((.8, .5), (1.8, .5))),
                  (ConeColour.BLUE, tuple((x, .5-gap+blue_slope*(x-.8)) for x in (.8, 1.3, 1.8))))
            for x, y in pairs]


def test_two_point_divider_startup_needs_persistent_parallel_boundary_and_stationary_fresh_frames():
    policy = ThreeRowPolicy()
    for i in range(10):
        obs = replace(observation(i/10, sparse_rows(), speed=0.), lidar_cartesian=scan([]))
        assert policy.run_policy(obs)[0].speed_m_per_s == 0.
    obs = replace(observation(1., sparse_rows(), speed=0.), lidar_cartesian=scan([]))
    assert policy.run_policy(obs)[0].speed_m_per_s > 0.
    assert policy.state.startup_samples >= 8


@pytest.mark.parametrize("gap,blue_slope", [(.5, 0.), (1., .3)])
def test_sparse_startup_rejects_implausible_gap_and_nonparallel_boundary(gap, blue_slope):
    policy = ThreeRowPolicy()
    for i in range(20):
        obs = replace(observation(i/10, sparse_rows(gap, blue_slope), speed=0.), lidar_cartesian=scan([]))
        assert policy.run_policy(obs)[0].speed_m_per_s == 0.
    assert not policy.state.has_driven


def test_sparse_startup_cannot_count_elapsed_time_across_missing_geometric_evidence():
    policy = ThreeRowPolicy()
    for i in range(12):
        cones = sparse_rows() if i < 5 or i == 11 else []
        obs = replace(observation(i/10, cones, speed=0.), lidar_cartesian=scan([]))
        assert policy.run_policy(obs)[0].speed_m_per_s == 0.
    assert policy.state.startup_samples == 1


def test_approach_can_replace_truncated_turn_only_with_new_evidence_from_all_three_rows():
    policy = ThreeRowPolicy()
    now = MotionPose(10., 2.8, 0., 0., 3., .4, 0.)
    policy._store(ThreeRowTurnMemory(row_heading=0., row_offsets=(-.5, .5, 1.5),
                  row_ends=(4., 3., 4.), row_end_seen_s=(9., 9., 9.), observed_start=0.))
    policy._plan(now)
    assert policy.state.route_seen_s == 10.
    policy._store(replace(policy.state, row_ends=(4.5, 3.5, 4.5), row_end_seen_s=(10.2, 9., 9.)))
    now = replace(now, time_s=11., x=3.4)
    policy._plan(now)
    assert policy.state.route_seen_s == 10.
    policy._store(replace(policy.state, row_end_seen_s=(10.2, 10.2, 10.2)))
    policy._plan(now)
    assert policy.state.route_seen_s == 11. and policy.state.turn_start_u == 5.25
    policy._store(replace(policy.state, row_ends=(5., 4., 5.), row_end_seen_s=(11.2,)*3))
    policy._plan(replace(now, time_s=12., x=4., yaw=.3))
    assert policy.state.route_seen_s == 11.  # Never move a committed turn with later single-side data.


@pytest.mark.parametrize("profile", ["nominal", "outer_hidden", "short_all_hidden"])
def test_teacher_navigates_three_row_course_with_bounded_temporary_cone_loss(profile):
    result = simulate(ThreeRowCase(profile=profile), ThreeRowPolicy())
    assert result["metrics"]["safe_completion"], result["metrics"]


def test_prolonged_camera_loss_expires_anchored_route_and_brakes():
    result = simulate(ThreeRowCase(profile="long_all_hidden"), ThreeRowPolicy())
    metrics = result["metrics"]
    assert metrics["stop_reason"] == "observed route expired" and metrics["latched_stop"]
    assert not metrics["completed"] and metrics["cones_touched"] == 0 and metrics["max_outside_m"] == 0
    assert result["trajectory"][-1, 4] < .015
