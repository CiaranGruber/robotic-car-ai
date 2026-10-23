"""Closed-boundary geometry, sensed-cap admission and conditional forward-turn regressions."""
from dataclasses import asdict, replace
import math

import numpy as np
import pytest

from policy.action_policy.connected_uturn import (ConnectedTurnConfig, ConnectedTurnNetwork,
                                                 ConnectedTurnPolicy, inside_turn_path, minimum_strip_width)
from policy.memory import ConeTrack, MotionPose, ThreeRowTurnMemory
from tests.policy.connected_uturn_simulation import ConnectedUTurnTrack, ThreeRowCase, simulate
from tests.policy.stage1_scenarios import body_corners
from tests.policy.test_three_row_uturn import weights, return_cones, return_state


def hypothetical():
    return ConnectedTurnConfig(max_left_curvature_per_m=2.2, turning_radius_m=.5)


def test_blue_straights_extend_one_metre_then_join_through_cap_without_an_apron():
    track = ConnectedUTurnTrack(placement_jitter_m=0.)
    points = track.row_cones()
    assert not any("apron" in name for name in asdict(track))
    assert track.outer_end_m == 4. and track.furthest_blue_x_m == 5.
    for row in (0, 2):
        assert max(x for i, _, x, _ in points if i == row) == 4.
    assert max(x for i, _, x, _ in points if i == 1) == 3.
    cap = [(x, y) for i, _, x, y in points if i == 3]
    assert len(cap) >= 5 and all(math.hypot(x-4., y-.5) == pytest.approx(1.) for x, y in cap)
    assert all(colour.name == "BLUE" for i, colour, _, _ in points if i == 3)
    assert track.outside_distance(5.1, .5) > 0.
    assert track.outside_distance(4.5, 1.5) > 0.  # Inside bounding box, outside curved cap.


@pytest.mark.parametrize("scale", [.8, 1., 1.2])
def test_cap_shape_changes_physical_cones_and_continuous_scoring_together(scale):
    track = ConnectedUTurnTrack(cap_depth_scale=scale, placement_jitter_m=0.)
    assert track.furthest_blue_x_m == pytest.approx(4.+scale)
    cap = [(x, y) for row, _, x, y in track.row_cones() if row == 3]
    for x, y in cap:
        assert ((x-4.)/scale)**2+(y-.5)**2 == pytest.approx(1.)
        assert track.outside_distance(x, y) == pytest.approx(0., abs=1e-12)
        assert track.outside_distance(x+.01, y) > 0.
        assert track.outside_distance(x-.01, y) < 0.
    assert np.max(track.boundary()[:, 0]) == pytest.approx(track.furthest_blue_x_m, abs=.001)


def test_invalid_cap_shape_is_rejected():
    for scale in (math.nan, math.inf, 0., .49, 2.01):
        with pytest.raises(ValueError, match="Cap depth"):
            ConnectedUTurnTrack(cap_depth_scale=scale)


def test_scorer_rejects_full_body_escape_between_discrete_blue_cones_and_crossed_divider():
    track = ConnectedUTurnTrack()
    assert track.footprint_outside_distance(body_corners(4.9, .5, 0.)) > 0.
    corners = body_corners(1., .5, 0.)
    assert all(track.outside_distance(x, y) <= 0. for x, y in corners)
    assert track.footprint_outside_distance(corners) > 0.
    assert track.footprint_outside_distance(body_corners(1., 0., 0.)) <= 0.


def test_strip_bound_matches_independent_rectangular_semicircle_envelope():
    radius = 1.
    corners = np.concatenate([body_corners(radius*math.sin(a), radius*(1-math.cos(a)), a)
                              for a in np.linspace(0., math.pi, 1000)])
    required = minimum_strip_width(1., front=.297, rear=.198)
    assert np.ptp(corners[:, 1]) == pytest.approx(required, abs=1e-5)
    assert minimum_strip_width(1.) > 2.3
    assert minimum_strip_width(2.2) < 2.


def test_closed_model_rejects_open_apron_checkpoint_even_with_equal_limits(tmp_path):
    path = tmp_path/"open.npz"
    weights(path)
    with pytest.raises(ValueError, match="contract"):
        ConnectedTurnNetwork(path, ConnectedTurnConfig())


def cap_state(count=3, seen=.8):
    blue = tuple(ConeTrack(4.6+math.cos(a), .5+math.sin(a), .05, "BLUE", .9, seen, 0.)
                 for a in np.linspace(-.6, .6, count))
    return ThreeRowTurnMemory(row_heading=0., row_offsets=(-.5, .5, 1.5),
                              route=((0., 0.), (1., 0.)), tracks=blue, observed_start=0.)


def test_cap_uses_observed_distinct_positions_and_original_acquisition_age():
    policy = ConnectedTurnPolicy(hypothetical())
    policy._store(cap_state())
    now = MotionPose(1., 0., 0., 0., 0., 0., 0.)
    policy._rows(now, replace(now, time_s=.8), [])
    assert policy.state.cap_u == pytest.approx(4.6)
    assert policy.state.cap_v == pytest.approx(.5)
    assert policy.state.cap_radius_m == pytest.approx(1.)
    assert policy.state.cap_seen_s == .8
    policy._rows(replace(now, time_s=3.), now, [])
    assert policy.state.cap_seen_s == .8  # Old tracks cannot make a new observation timestamp.


def test_two_cap_points_do_not_admit_a_turn_or_invent_an_endpoint():
    policy = ConnectedTurnPolicy(hypothetical())
    policy._store(replace(cap_state(2), route=()))
    now = MotionPose(1., 0., 0., 0., 0., 0., 0.)
    policy._rows(now, now, [])
    policy._plan(now)
    assert policy.state.cap_u is None and not policy.state.route


def test_planned_half_circle_stays_inside_closed_boundary_with_full_footprint():
    track = ConnectedUTurnTrack()
    path = inside_turn_path(2., 0., 4., .5, .5, 0., .12, 1.)
    xy = np.array(path)
    headings = np.arctan2(np.diff(xy[:, 1]), np.diff(xy[:, 0]))
    assert all(track.footprint_outside_distance(body_corners(x, y, h)) <= 0.
               for (x, y), h in zip(xy[:-1], headings))
    assert max(xy[:, 0]) < track.furthest_blue_x_m


def test_short_fresh_return_cannot_discard_or_refresh_supported_terminal_tail():
    policy = ConnectedTurnPolicy(hypothetical())
    policy._store(return_state())
    previous = policy.state
    now = previous.poses[-1]
    policy._fresh_return(now, now, return_cones())
    assert policy.state.route == previous.route
    assert policy.state.route_seen_s == previous.route_seen_s


def test_prolonged_all_cone_loss_stops_before_route_expiry():
    policy = ConnectedTurnPolicy(hypothetical())
    policy._store(replace(return_state(), last_nonempty_s=1.))
    policy._plan(MotionPose(5., 2., 1., math.pi, 1., .3, 0.))
    assert policy.state.stopped and "empty cone stream" in policy.state.stop_reason
    assert policy.state.route_seen_s == 1.


def test_braking_budget_covers_current_speed_through_slowdown_and_next_admitted_speed():
    from tests.policy.test_temporal_lane import observation
    policy = ConnectedTurnPolicy(hypothetical())
    assert policy._guard_speed(2., observation(0., [], speed=.29)) == .29
    assert policy._guard_speed(2., observation(0., [], speed=.1)) == .15
    assert policy._guard_speed(0., observation(0., [], speed=.1)) == .3
    policy._store(replace(policy.state, speed_commands=((.8, .3), (.95, .15))))
    assert policy._guard_speed(2., observation(1., [], speed=.15)) == .3
    assert policy._guard_speed(2., observation(1.4, [], speed=.15)) == .15


def test_cap_route_time_and_distance_anchors_use_same_acquisition_pose():
    policy = ConnectedTurnPolicy(hypothetical())
    earlier = MotionPose(.8, 2., 0., 0., 1., .3, 0.)
    now = MotionPose(1., 2.06, 0., 0., 1.06, .3, 0.)
    policy._store(ThreeRowTurnMemory(poses=(earlier, now), row_heading=0., row_offsets=(-.5, .5, 1.5),
                  observed_start=0., cap_u=4., cap_v=.5, cap_radius_m=1., cap_seen_s=.8))
    policy._plan(now)
    assert policy.state.route and policy.state.route_seen_s == .8 and policy.state.route_seen_distance_m == 1.


def test_missing_cap_acquisition_pose_latches_stop_without_exception():
    policy = ConnectedTurnPolicy(hypothetical())
    now = MotionPose(1., 2., 0., 0., 1., .3, 0.)
    policy._store(ThreeRowTurnMemory(poses=(now,), row_heading=0., row_offsets=(-.5, .5, 1.5),
                  observed_start=0., cap_u=4., cap_v=.5, cap_radius_m=1., cap_seen_s=.8))
    policy._plan(now)
    assert policy.state.stopped and not policy.state.route and "pose unavailable" in policy.state.stop_reason


def test_previous_steering_assumption_is_rejected_in_closed_course_without_driving():
    result = simulate(ThreeRowCase(ConnectedUTurnTrack()), ConnectedTurnPolicy())
    metrics = result["metrics"]
    assert metrics["latched_stop"] and metrics["distance_m"] == 0.
    assert "too narrow" in metrics["stop_reason"]
    assert not metrics["completed"] and metrics["cones_touched"] == 0 and metrics["max_outside_m"] == 0.


@pytest.mark.parametrize("profile", ["nominal", "random_drops", "divider_hidden", "outer_hidden", "short_all_hidden"])
def test_hypothetical_tighter_steering_navigates_connected_boundary_with_temporary_cone_loss(profile, monkeypatch):
    def checkpoint_forbidden(*args, **kwargs):
        raise AssertionError("Online geometric control must not load trained weights")
    monkeypatch.setattr(np, "load", checkpoint_forbidden)
    case = ThreeRowCase(ConnectedUTurnTrack(), profile=profile, max_left_curvature_per_m=2.2)
    policy = ConnectedTurnPolicy(hypothetical())
    assert policy.network is None
    result = simulate(case, policy)
    assert result["metrics"]["safe_completion"], result["metrics"]


def test_hypothetical_model_stops_for_prolonged_camera_loss_without_leaving_enclosure():
    case = ThreeRowCase(ConnectedUTurnTrack(), profile="long_all_hidden", max_left_curvature_per_m=2.2)
    result = simulate(case, ConnectedTurnPolicy(hypothetical()))
    metrics = result["metrics"]
    assert metrics["latched_stop"] and "empty cone stream" in metrics["stop_reason"]
    assert metrics["cones_touched"] == 0 and metrics["max_outside_m"] == 0.
    assert result["trajectory"][-1, 4] < .015


def test_closed_checkpoint_contract_covers_radius_speed_and_memory_limits(tmp_path):
    path = tmp_path/"closed.npz"
    config = hypothetical()
    weights(path, contract=ConnectedTurnNetwork.contract(config))
    ConnectedTurnNetwork(path, config)
    for change in ({"turning_radius_m": .55}, {"turn_inset_m": .2}, {"turn_speed_m_per_s": .1},
                   {"route_max_age_s": 30.}, {"camera_empty_hold_s": 2.}):
        with pytest.raises(ValueError, match="contract"):
            ConnectedTurnNetwork(path, replace(config, **change))


def test_longer_observed_divider_refines_noisy_startup_heading_with_bounded_changes():
    policy = ConnectedTurnPolicy(hypothetical())
    tracks = tuple(ConeTrack(x, y, .05, colour, .9, .8, 0.)
                   for colour, y in (("YELLOW", .5), ("BLUE", -.5), ("BLUE", 1.5))
                   for x in (.5, 1., 1.5, 2.))
    policy._store(ThreeRowTurnMemory(tracks=tracks, row_heading=.12, row_offsets=(-.5, .5, 1.5)))
    pose = MotionPose(1., 0., 0., 0., 0., 0., 0.)
    for i in range(30):
        previous = policy.state.row_heading
        policy._rows(replace(pose, time_s=1.+i*.1), pose, [])
        assert abs(policy.state.row_heading-previous) <= .005+1e-10
    assert abs(policy.state.row_heading) < .001


def test_refinement_reprojects_cached_endpoint_sources_without_advancing_original_start():
    policy = ConnectedTurnPolicy(hypothetical())
    points = ((3., -.5), (2.5, .5), (3., 1.5))
    theta = .1
    start_xy = (-.2, 1.5)  # This original point has already aged out of the active cone tracks.
    project = lambda p, angle: math.cos(angle)*p[0]+math.sin(angle)*p[1]
    tracks = tuple(ConeTrack(x, y, .05, colour, .9, .8, 0.)
                   for colour, y in (("YELLOW", .5), ("BLUE", -.5), ("BLUE", 1.5))
                   for x in (.5, 1., 1.5, 2.))
    ends = tuple(project(p, theta) for p in points)
    policy._store(ThreeRowTurnMemory(tracks=tracks, row_heading=theta, row_offsets=(-.5, .5, 1.5),
                  row_ends=ends, row_end_xy=points, row_end_seen_s=(.2,)*3,
                  observed_start=project(start_xy, theta), observed_start_xy=start_xy, observed_start_seen_s=.1,
                  cap_u=4., cap_v=.5, cap_radius_m=1., cap_seen_s=.8))
    pose = MotionPose(1., 0., 0., 0., 0., 0., 0.)
    policy._rows(pose, pose, [])
    new = policy.state
    assert new.row_heading == pytest.approx(.095)
    assert new.row_ends == pytest.approx(tuple(project(p, new.row_heading) for p in points))
    assert new.observed_start == pytest.approx(project(start_xy, new.row_heading))
    assert new.observed_start_xy == start_xy and new.observed_start_seen_s == .1
    assert new.row_end_seen_s == (.2,)*3 and new.cap_u is None
    from policy.memory import CarMemory
    assert CarMemory.deserialise(policy.memory.data.serialise()).data == policy.memory.data


def test_fast_policy_calls_keep_pending_high_speed_until_actuator_delay_has_drained(monkeypatch):
    from policy.action_policy.three_row_uturn import ThreeRowPolicy
    from policy.control.actions import DriveCommand
    from tests.policy.test_temporal_lane import observation
    def actions(self, obs, lanes=None):
        return [DriveCommand(.3 if obs.policy.policy_elapsed_s == 0. else .15, 2.)]
    monkeypatch.setattr(ThreeRowPolicy, "run_policy", actions)
    policy = ConnectedTurnPolicy(hypothetical())
    for i in range(501):
        now = i*.001
        obs = observation(now, [], speed=.15, dt=.001)
        policy.run_policy(obs)
        assert len(policy.state.speed_commands) <= 9
        if now <= .2:
            assert policy._guard_speed(2., obs) == .3
    assert policy._guard_speed(2., obs) == .15
    from policy.memory import CarMemory
    assert CarMemory.deserialise(policy.memory.data.serialise()).data == policy.memory.data


def test_legacy_finite_endpoints_without_finite_sources_keep_original_frame():
    policy = ConnectedTurnPolicy(hypothetical())
    tracks = tuple(ConeTrack(x, .5, .05, "YELLOW", .9, .8, 0.) for x in (.5, 1., 1.5, 2.))
    policy._store(ThreeRowTurnMemory(tracks=tracks, row_heading=.1, row_ends=(3., 2.5, 3.),
                  row_end_xy=((math.nan, math.nan),)*3, row_end_seen_s=(.2,)*3))
    now = MotionPose(1., 0., 0., 0., 0., 0., 0.)
    policy._rows(now, now, [])
    assert policy.state.row_heading == .1
