"""Bounded two-edge lane reuse under partial camera occlusion, without ROS."""
from dataclasses import replace
import math

import pytest

from policy.input_output import ConeColour, Position
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector
from policy.lane_detection.lanes import LaneStatus
from policy.lane_detection.temporal import detect_temporal_lane
from policy.memory import CarMemory, LaneTrackingMemory
from tests.policy.test_lane_detection import lane_cones
from tests.policy.test_temporal_lane import observation


def setup(**settings):
    fit = LaneDetector(LaneDetectionConfig(single_edge_hold_enabled=True, **settings))
    return CarMemory(), fit


def one_side(cones, colour=ConeColour.BLUE):
    return [c for c in cones if c.colour == colour]


@pytest.mark.parametrize('colour', [ConeColour.BLUE, ConeColour.YELLOW])
def test_one_edge_transports_saved_lane_and_preserves_colour_role(colour):
    memory, fit = setup(max_curvature_per_m=2.0)
    fixed = lane_cones(radius_m=2.0)
    original, _ = detect_temporal_lane(observation(0, fixed, yaw_rate=.25), memory, fit)
    anchor = memory.data.lane_tracking.both_edge_reference
    for i in range(1, 6):
        t = i / 10
        angle = .25 * t
        x, y = 2 * math.sin(angle), 2 * (1 - math.cos(angle))
        moved = [replace(c, pos=Position(math.cos(angle)*(c.pos.x-x)+math.sin(angle)*(c.pos.y-y),
                                         -math.sin(angle)*(c.pos.x-x)+math.cos(angle)*(c.pos.y-y), c.pos.z))
                 for c in one_side(fixed, colour)]
        lane, _ = detect_temporal_lane(observation(t, moved, yaw_rate=.25), memory, fit)
        assert lane.has_lane and memory.data.lane_tracking.holding_single_edge
        assert lane.status == (LaneStatus.LEFT_EDGE_ONLY if colour == ConeColour.BLUE else LaneStatus.RIGHT_EDGE_ONLY)
        assert lane.path_curvature_per_m == original.path_curvature_per_m
        assert lane.lane_width_m == original.lane_width_m
        assert lane.centre_line.project(0, 0) == pytest.approx(0, abs=1e-9)
        # Transport independently: a point of the original lane remains on the held lane.
        px, py = original.centre_line.point_at(1)
        bx, by = math.cos(angle)*(px-x)+math.sin(angle)*(py-y), -math.sin(angle)*(px-x)+math.cos(angle)*(py-y)
        assert lane.centre_line.signed_distance(bx, by) == pytest.approx(0, abs=1e-9)
        assert memory.data.lane_tracking.both_edge_reference == anchor
    assert CarMemory.deserialise(memory.data.serialise()).data == memory.data


def test_single_edge_without_anchor_and_both_missing_never_invent_lane():
    memory, fit = setup(memory_enabled=True)
    lane, _ = detect_temporal_lane(observation(0, one_side(lane_cones())), memory, fit)
    assert not lane.has_lane
    detect_temporal_lane(observation(.1, lane_cones()), memory, fit)
    lane, augmented = detect_temporal_lane(observation(.2, []), memory, fit)
    assert len(augmented) > 0 and not lane.has_lane
    assert not memory.data.lane_tracking.holding_single_edge


@pytest.mark.parametrize('settings', [dict(single_edge_hold_max_age_s=.25), dict(single_edge_hold_max_distance_m=.12)])
def test_one_edge_never_renews_anchor_and_expiry_stops_prediction(settings):
    memory, fit = setup(**settings)
    detect_temporal_lane(observation(0, lane_cones()), memory, fit)
    anchor = memory.data.lane_tracking.both_edge_reference
    for i in range(1, 4):
        t = i / 10
        cones = [replace(c, pos=replace(c.pos, x=c.pos.x-.5*t)) for c in one_side(lane_cones())]
        lane, _ = detect_temporal_lane(observation(t, cones), memory, fit)
        assert lane.has_lane == (i < 3)
        assert memory.data.lane_tracking.both_edge_reference == anchor


@pytest.mark.parametrize('failure', ['motion', 'contradiction', 'new_run'])
def test_invalid_support_or_motion_does_not_follow_saved_lane(failure):
    memory, fit = setup()
    detect_temporal_lane(observation(0, lane_cones()), memory, fit)
    obs = observation(.1, one_side(lane_cones(offset_m=.5 if failure == 'contradiction' else 0)))
    if failure == 'motion':
        obs = replace(obs, car=replace(obs.car, ang_velocity=None))
    elif failure == 'new_run':
        obs = observation(0, one_side(lane_cones()))
    lane, _ = detect_temporal_lane(obs, memory, fit)
    assert not lane.has_lane and not memory.data.lane_tracking.holding_single_edge


def test_old_serialised_temporal_state_has_default_anchor():
    old = LaneTrackingMemory().serialise()
    del old['both_edge_reference'], old['holding_single_edge']
    restored = LaneTrackingMemory.deserialise(old)
    assert restored.both_edge_reference is None and not restored.holding_single_edge


def test_two_edge_colour_flip_rejected_and_same_roles_reacquire():
    memory, fit = setup()
    detect_temporal_lane(observation(0, lane_cones()), memory, fit)
    anchor = memory.data.lane_tracking.both_edge_reference
    lane, _ = detect_temporal_lane(observation(.1, lane_cones(colours=(ConeColour.YELLOW, ConeColour.BLUE))), memory, fit)
    assert not lane.has_lane and memory.data.lane_tracking.rejected
    assert memory.data.lane_tracking.both_edge_reference == anchor
    lane, _ = detect_temporal_lane(observation(.2, lane_cones()), memory, fit)
    assert lane.status == LaneStatus.BOTH_EDGES
    assert memory.data.lane_tracking.both_edge_reference.pose.time_s == pytest.approx(.2)


@pytest.mark.parametrize("speed", [0.0, 0.5])
def test_controller_refuses_held_lane_shorter_than_horizon_and_stopping_travel(speed):
    from policy.action_policy.mpc import MpcConfig
    from policy.action_policy.policy import MpcPolicy
    memory, fit = setup()
    cones = lane_cones()
    lane, _ = detect_temporal_lane(observation(0, cones), memory, fit)
    controller = MpcPolicy(MpcConfig())
    controller.run_policy(observation(0, cones, speed=speed), lane, lane_support_m=.3)
    assert controller.stopped_for_lane_loss
    assert controller.solve_count == 0
    assert controller.plan_curvatures_per_m is None


def test_controller_accepts_covered_horizon_and_caps_last_seen_distance():
    from policy.action_policy.mpc import MpcConfig
    from policy.action_policy.policy import MpcPolicy
    memory, fit = setup()
    cones = lane_cones()
    lane, _ = detect_temporal_lane(observation(0, cones), memory, fit)
    controller = MpcPolicy(MpcConfig())
    controller.run_policy(observation(0, cones), lane, lane_support_m=1.2)
    assert not controller.stopped_for_lane_loss and controller.solve_count == 1
    assert controller.farthest_cone_ahead_m == pytest.approx(1.2)


@pytest.mark.parametrize('colours', [(ConeColour.BLUE,), (ConeColour.BLUE, ConeColour.YELLOW)])
def test_failed_fresh_arc_fit_can_still_follow_supported_saved_boundaries(monkeypatch, colours):
    memory, fit = setup()
    initial, _ = detect_temporal_lane(observation(0, lane_cones()), memory, fit)
    anchor = memory.data.lane_tracking.both_edge_reference
    # Deliberate fitter failure isolates the fallback contract from the unchanged edge fitter.
    monkeypatch.setattr(fit, 'detect', lambda obs: replace(initial, status=LaneStatus.NO_LANE,
                                                        left_edge=None, right_edge=None, centre_line=None))
    cones = [replace(c, pos=replace(c.pos, x=c.pos.x-.05)) for c in lane_cones() if c.colour in colours]
    lane, _ = detect_temporal_lane(observation(.1,cones),memory,fit)
    assert lane.has_lane and memory.data.lane_tracking.holding_single_edge
    assert lane.path_curvature_per_m == initial.path_curvature_per_m
    assert (lane.right_edge is not None) == (ConeColour.YELLOW in colours)
    assert memory.data.lane_tracking.both_edge_reference == anchor
    lane, _ = detect_temporal_lane(observation(.2,[]),memory,fit)
    assert not lane.has_lane and not memory.data.lane_tracking.holding_single_edge
