"""ArUco chooses an admitted junction path while normal measured lanes stay unchanged."""
from dataclasses import replace
import math

import numpy as np
import pytest

from policy.action_policy.intersection_navigator import make_route, route_projection
from policy.input_output import (AngularVelocity, ConeBatch, ConeColour, LidarCartesian, ObservedCarState,
                                 PolicyState, SensorAge, WheelSpeed)
from policy.intersection_state import IntersectionMemory, IntersectionPose
from policy.lane_detection.selection import select_intersection_lane
from policy.markers.commands import MarkerCommand
from policy.memory import CarMemory, MpcStop
from policy.visualisation import StepDrawer
from tests.policy.intersection_simulation import (TURN_LOSS_DURATIONS_S, make_policy, settings, turn_occlusion)
from tests.policy.intersection_turning import track_for
from tests.policy.test_intersection_navigation import active_state, navigator, observation
from tests.policy.test_lane_detection import lane_cones
from tests.policy.test_markers import marker, straight_lane


def selected(command):
    pose = IntersectionPose(1., -2.5, 0., 0., 3., .4, 0.)
    state = IntersectionMemory(phase="turn", poses=(pose,), width_m=2.4, frame_heading=0.,
                                centre_u=0., command=command, route_seen_s=.9)
    state = replace(state, route=make_route(state, pose, 2.5, command))
    _, progress, arc = route_projection(state.route, pose)
    return state, pose, progress, arc, select_intersection_lane(state, pose, progress, arc, .4, .1, 12)


@pytest.mark.parametrize("command, sign", [(MarkerCommand.TURN_LEFT, 1), (MarkerCommand.TURN_RIGHT, -1),
                                           (MarkerCommand.STRAIGHT, 0)])
def test_same_admitted_cross_selects_the_requested_path_and_curvature(command, sign):
    state, pose, progress, arc, lane = selected(command)
    assert lane.command is command and lane.evidence_age_s == pytest.approx(.1)
    if sign:
        assert sign*lane.points[-1][1] > 4.
        assert min(sign*k for k in lane.reference_curvatures_per_m) > .39
    else:
        assert all(abs(y) < 1e-9 for _, y in lane.points)
        assert lane.reference_curvatures_per_m == (0.,)*12
    expected = np.interp(progress+.4*.1*np.arange(12), arc, np.array(state.route)[:, 3])
    assert lane.reference_curvatures_per_m == pytest.approx(expected)


def test_selected_reference_is_transformed_to_current_estimated_body_pose():
    state, pose, progress, arc, _ = selected(MarkerCommand.TURN_LEFT)
    rotated = replace(pose, heading=math.pi/2)
    lane = select_intersection_lane(state, rotated, progress, arc, .4, .1, 12)
    world_end = np.array(state.route[-1][:2])
    x, y = lane.points[-1]
    assert (rotated.x-y, rotated.y+x) == pytest.approx(world_end)


def test_unadmitted_command_cannot_form_an_active_lane_reference():
    state, pose, progress, arc, _ = selected(MarkerCommand.TURN_LEFT)
    with pytest.raises(ValueError, match="admitted"):
        select_intersection_lane(replace(state, phase="approach"), pose, progress, arc, .4, .1, 12)


@pytest.mark.parametrize("curvature", (0., .5, -.5))
def test_direction_markers_leave_normal_straight_and_curved_lanes_unchanged(curvature):
    params, _ = settings()
    outcomes = []
    for marker_id in (None, 1, 5, 7):
        policy, memory = make_policy(params, navigation=True), CarMemory()
        actions = []
        for step in range(5):
            obs = straight_lane(markers=[] if marker_id is None else [marker(marker_id)], first=step == 0)
            cones = lane_cones(radius_m=math.inf if not curvature else 1/abs(curvature),
                               turn=1. if curvature >= 0 else -1., width_m=1.4, length_m=2.)
            obs = replace(obs, cones=ConeBatch(cones, SensorAge(0., step*100_000_000), 0.),
                          lidar_cartesian=LidarCartesian([], SensorAge(0., None), "base_link"),
                          car=ObservedCarState(None, AngularVelocity(0., 0., .4*curvature, SensorAge(0., None)),
                                               None, WheelSpeed(.4, SensorAge(0., None))),
                          policy=PolicyState(.1 if step else 0., step*.1, step == 0))
            action = policy.step(obs, memory)
            memory.step()
            assert policy.selected_lane is None
            assert memory.data.intersection is not None and not memory.data.intersection.route
            assert policy.lanes.has_lane
            assert policy.lanes.path_curvature_per_m == pytest.approx(curvature, abs=.03)
            actions.append((action.drive_action, action.steering_action))
        assert (memory.data.intersection_command is None) == (marker_id is None)
        outcomes.append(actions)
    assert np.array(outcomes[0]) == pytest.approx(np.asarray(outcomes[1]))
    assert np.array(outcomes[0]) == pytest.approx(np.asarray(outcomes[2]))
    assert np.array(outcomes[0]) == pytest.approx(np.asarray(outcomes[3]))


def test_selection_is_cleared_when_the_route_stops_or_completes():
    nav, mpc = navigator()
    memory = CarMemory()
    memory.update_intersection(active_state())
    nav.step(observation(), None, memory, mpc)
    assert nav.selected_lane is not None
    memory.stop_mpc(MpcStop.INTERSECTION)
    nav.step(observation(time=1.2), None, memory, mpc)
    assert nav.selected_lane is None
    memory = CarMemory()
    memory.update_intersection(replace(active_state(), phase="complete", completed_distance_m=5.))
    nav.step(observation(), None, memory, mpc)
    assert nav.selected_lane is None


def test_completed_handoff_learns_lane_width_once_and_accepts_reversed_exit_colours():
    params, _ = settings()
    policy, memory = make_policy(params, navigation=True), CarMemory()
    memory.update_intersection(replace(active_state(), phase="complete", completed_distance_m=5.))
    obs = observation()
    # Southbound on the fixture: yellow is left and blue is right.
    cones = lane_cones(width_m=1.4, colours=(ConeColour.YELLOW, ConeColour.BLUE))
    obs = replace(obs, cones=ConeBatch(cones, SensorAge(0., 123), 0.))
    policy.step(obs, memory)
    assert policy.selected_lane is None and policy.lanes.has_lane
    assert policy.lane_detector.config.left_colour == "AUTO"
    assert policy.lane_detector.lane_width_m == pytest.approx(1.+.2*(1.4-1.))


def test_intersection_reference_and_fresh_mpc_prediction_are_drawn_without_ordinary_solver_tick():
    nav, mpc = navigator()
    memory = CarMemory()
    memory.update_intersection(active_state())
    obs = observation()
    nav.step(obs, None, memory, mpc)
    params, _ = settings()
    policy = make_policy(params, navigation=True)
    policy.selected_lane = nav.selected_lane
    drawer = StepDrawer()
    action = policy.car_controller.step(obs, [])
    for _ in range(2):
        drawing = drawer.draw(policy, obs, action)
        assert drawing["lane"].lines[0].points == list(nav.selected_lane.points)
        assert drawing["plan/mpc"].lines


@pytest.mark.parametrize("profile", tuple(TURN_LOSS_DURATIONS_S))
@pytest.mark.parametrize("sign", (-1., 1.))
def test_camera_occlusion_has_finite_duration_and_recovers_after_a_real_turn(profile, sign):
    cones = straight_lane().cones
    track = track_for()
    output, started, active = turn_occlusion(cones, track, (-2., 0., 0.), profile, 1., None)
    assert output is cones and started is None and not active
    _, started, active = turn_occlusion(cones, track, (-2., 0., sign*math.radians(25)), profile, 2., started)
    assert started == 2. and active
    output, _, active = turn_occlusion(cones, track, (0., sign*3., sign*math.pi/2), profile,
                                      2.+TURN_LOSS_DURATIONS_S[profile], started)
    assert not active and output is cones
