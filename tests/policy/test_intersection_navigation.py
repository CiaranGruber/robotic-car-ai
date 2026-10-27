"""Intersection choice, acquisition frames, retained-route bounds and closed-loop lane alignment."""
from dataclasses import replace
import math

import numpy as np
import pytest

from policy.action_policy.avoidance import AvoidanceConfig
from policy.action_policy.intersection_navigator import (IntersectionNavigator, IntersectionNavigatorConfig, cross_fits,
                                                         make_route)
from policy.action_policy.lane_path import LanePath, PathSegment
from policy.action_policy.mpc import MpcConfig, MpcController, PathTrackingState
from policy.action_policy.policy import MpcPolicy
from policy.input_output import (AngularVelocity, CarObservations, ConeBatch, LidarCartesian, LidarCartesianPoint,
                                 ObservedCarState, PolicyState, Position, SensorAge, WheelSpeed)
from policy.intersection_state import IntersectionCone, IntersectionMemory, IntersectionPose
from policy.markers.commands import MarkerCommand
from policy.markers.markers import MarkerConfig, MarkerReader
from policy.memory import CarMemory, Memory, MpcStop
from policy.obstacle_detection.obstacle_detection import ObstacleDetectionConfig
from policy.obstacle_detection.obstacles import LaneObstacle
from tests.policy.intersection_simulation import CrossIntersection, make_policy, rollout, settings
from tests.policy.test_markers import marker, straight_lane


def navigator(**mpc_changes):
    config = MpcConfig(**mpc_changes)
    return (IntersectionNavigator(IntersectionNavigatorConfig(enabled=True), config, ObstacleDetectionConfig(), AvoidanceConfig()),
            MpcPolicy(config))


def active_state():
    pose = IntersectionPose(1., 3.5, 3.2, math.pi/2, 6., .4, 0.)
    state = IntersectionMemory(phase="turn", poses=(replace(pose, time_s=.8, y=3.12, distance_m=5.92), pose),
                  frame_heading=0., centre_v=0., width_m=2.4, frame_seen_s=.8, command=MarkerCommand.TURN_LEFT,
                  centre_u=3.5, route_seen_s=.8, route_distance_m=3., last_nonempty_s=.9)
    route = make_route(state, replace(pose, x=0., y=0., heading=0.), 2.5, MarkerCommand.TURN_LEFT)
    tracks = tuple(IntersectionCone(x, y, colour, .9) for x, colour in ((2.3, "BLUE"), (4.7, "YELLOW")) for y in (4., 4.5))
    return replace(state, route=route, cones=tracks)


def observation(time=1.1, *, wheel_age=0., gyro_age=0., lidar_age=0., points=()):
    return CarObservations(None, None, None, LidarCartesian(list(points), SensorAge(lidar_age, None), "base_link"),
             ObservedCarState(None, AngularVelocity(0., 0., 0., SensorAge(gyro_age, None)), None,
                              WheelSpeed(.4, SensorAge(wheel_age, None))), PolicyState(.1, time, False))


def test_disabled_navigator_preserves_existing_low_speed_and_curvature_configs():
    params, _ = settings()
    params["mpc"].update(target_speed_m_per_s=.3, max_curvature_per_m=.2)
    params["avoidance"]["enabled"] = False
    make_policy(params, navigation=False)
    with pytest.raises(ValueError):
        make_policy(params, navigation=True)


def test_current_steering_limit_requires_wider_cross_without_increasing_hardware_limit():
    for direction in (MarkerCommand.TURN_LEFT, MarkerCommand.TURN_RIGHT):
        assert not cross_fits(0., 1., 2.5, direction, .3, .3, .25)
        assert cross_fits(0., 2.4, 2.5, direction, .3, .3, .25)
    assert 1/2.5 < MpcConfig().max_curvature_per_m


@pytest.mark.parametrize("sensor", ["wheel_age", "gyro_age", "lidar_age"])
@pytest.mark.parametrize("age", [math.nan, -.1, .3])
def test_invalid_motion_or_lidar_age_latches_stop(sensor, age):
    nav, mpc = navigator()
    memory = CarMemory()
    memory.update_intersection(active_state())
    actions = nav.step(observation(**{sensor: age}), None, memory, mpc)
    assert actions[0].speed_m_per_s == 0. and actions[0].brake
    assert memory.data.mpc_stop is MpcStop.INTERSECTION
    assert nav.step(observation(time=1.2), None, memory, mpc)[0].speed_m_per_s == 0.


def test_lidar_is_transported_from_acquisition_pose_instead_of_current_pose():
    nav, _ = navigator()
    state = replace(active_state(), poses=(IntersectionPose(.8, 0., 0., 0., 0., .4, 0.),
                                          IntersectionPose(1., .2, 0., math.pi/2, .2, .4, 0.)))
    obs = observation(lidar_age=.2, points=(LidarCartesianPoint(Position(1., 0., .1), 0),))
    pose, world = nav._lidar_world(obs, state)
    assert pose.time_s == pytest.approx(.8)
    assert world[0] == pytest.approx((1., 0.))


def test_refitting_cached_rows_cannot_renew_the_frame_distance_anchor():
    nav, _ = navigator()
    poses = tuple(IntersectionPose(t, .4*t, 0., 0., .4*t, .4, 0.) for t in (.8, 1., 1.1))
    tracks = tuple(IntersectionCone(x, y, colour, .9) for y, colour in ((1.2, "BLUE"), (-1.2, "YELLOW"))
                   for x in (1., 2., 3.))
    state = IntersectionMemory(poses=poses, cones=tracks, frame_heading=0., width_m=2.4,
                               frame_seen_s=.9, frame_distance_m=.36)
    obs = replace(observation(), cones=ConeBatch([], SensorAge(.2, None), .2))
    updated = nav._observe(obs, None, state)
    assert updated.frame_seen_s == .9
    assert updated.frame_distance_m == pytest.approx(.36)


def test_raw_lidar_return_blocks_swept_turn_even_without_a_three_point_cluster():
    nav, _ = navigator()
    state = active_state()
    obs = observation(points=(LidarCartesianPoint(Position(.55, 0., .1), 0),))
    assert not nav._clear(obs, state, .2)


def test_active_route_obeys_duration_stop_and_original_expiry():
    for changes, state in (({"run_duration_s": 1.}, active_state()), ({}, replace(active_state(), route_seen_s=-24.))):
        nav, mpc = navigator(**changes)
        memory = CarMemory()
        memory.update_intersection(state)
        commands = nav.step(observation(), None, memory, mpc)
        assert commands[0].speed_m_per_s == 0.
        assert memory.data.intersection.phase == "stopped"
        assert memory.data.intersection.route_seen_s == state.route_seen_s


def test_reusing_one_outgoing_camera_acquisition_cannot_complete_handoff():
    nav, mpc = navigator()
    memory = CarMemory()
    memory.update_intersection(active_state())
    for time in (1.1, 1.2, 1.3, 1.4):
        nav.step(observation(time=time), None, memory, mpc)
        assert memory.data.intersection.phase != "complete"


def test_committed_choice_ignores_new_markers_and_serialises_with_legacy_defaults():
    memory = CarMemory()
    memory.update_intersection(active_state())
    reader = MarkerReader(MarkerConfig(turn_left_id=3, turn_right_id=4))
    for _ in range(3):
        assert reader.step(straight_lane(markers=[marker(4)]), memory) is None
    assert memory.data.intersection.command is MarkerCommand.TURN_LEFT
    assert memory.data.intersection_command is None
    assert CarMemory.deserialise(memory.data.serialise()).data == memory.data
    old = {k: v for k, v in memory.data.serialise().items()
           if k not in ("intersection", "intersection_command_seen_s")}
    assert Memory.deserialise(old).intersection is None
    memory.start_marker_run()
    memory.start_mpc_run()
    assert memory.data.intersection is None and memory.data.mpc_stop is None


def test_intersection_takeover_discards_obstacles_and_path_in_the_previous_lane_frame():
    params, _ = settings()
    policy = make_policy(params, navigation=True)
    memory = CarMemory()
    memory.update_intersection(active_state())
    memory.remember_lane_obstacles([LaneObstacle(-.2, 0., -.3, -.1, 12)])
    old_path = LanePath(0., (PathSegment.hold(0., 1., .2),))
    policy.avoidance.path = old_path
    policy.avoidance.driven_since_path_m = .2
    policy.action_policy.avoidance_path = old_path

    policy.step(observation(), memory)

    assert policy.selected_lane is not None
    assert memory.data.mpc_stop is None
    assert memory.data.pose is not None
    assert memory.data.lane_obstacles == ()
    assert policy.avoidance.remembered == []
    assert policy.avoidance.path is None and policy.avoidance.driven_since_path_m == 0.
    assert policy.action_policy.avoidance_path is None


def test_mpc_previews_the_route_curvature_before_the_turn_starts():
    controller = MpcController(MpcConfig())
    straight = controller.solve(PathTrackingState(0., 0., 0.), 0., 0., .4)
    preview = controller.solve(PathTrackingState(0., 0., 0.), 0., 0., .4,
                               reference_curvatures=np.r_[np.zeros(3), np.full(9, .4)])
    assert preview.converged and straight.converged
    assert preview.curvatures_per_m[0] > straight.curvatures_per_m[0]
    assert np.max(preview.curvatures_per_m) <= .48
    with pytest.raises(ValueError, match="horizon"):
        controller.solve(PathTrackingState(0., 0., 0.), 0., 0., .4, reference_curvatures=np.array([math.nan]))


@pytest.mark.parametrize("direction", ["left", "right", "straight"])
def test_signal_selects_exit_aligns_with_lane_and_consumes_command(direction):
    track = CrossIntersection(width_m=2.4, arm_length_m=8., cone_spacing_m=.2,
                              marker_x_m=-3.8, marker_y_m=-.7, exit_gate_m=3.6)
    result = rollout(direction, track=track, navigation=True)
    metrics = result["metrics"]
    assert metrics["navigation_pass"], (metrics, result["phases"])
    assert metrics["command_consumed"] and metrics["handoff_completed"]
    assert not metrics["wrong_exit"] and metrics["cone_contacts"] == 0
