"""Grouped marker directions, unsupported-marker stops and explicit recovery."""
from array import array
from dataclasses import replace
import math

import pytest

from policy.input_output import FiducialBatch, SensorAge
from policy.markers.commands import MarkerCommand
from policy.markers.markers import MarkerConfig, MarkerReader
from policy.memory import CarMemory, CarPose, IntersectionTurn, MpcStop, TurnPhase
from policy.visualisation import _intersection_lines
from scripts.mcap_export import DEFAULT_SETTINGS_PATH, build_policy, read_settings
from tests.policy.test_markers import marker, read, straight_lane
from tests.policy.test_intersection_navigation import active_state, observation
from tests.policy.test_obstacle_detection import arc_points, lidar


def configured():
    settings = read_settings(DEFAULT_SETTINGS_PATH)
    return settings, MarkerConfig(**{key.removeprefix("markers."): value
                                    for key, value in settings.items() if key.startswith("markers.")})


def test_preparation_rejects_an_obstacle_horizon_beyond_its_bounded_preview():
    settings, _ = configured()
    settings["obstacle_detection.corridor_length_m"] = 7.
    with pytest.raises(ValueError, match="6.4"):
        build_policy(settings)


@pytest.mark.parametrize("marker_id, expected", [
    *[(i, MarkerCommand.TURN_LEFT) for i in range(1, 5)],
    *[(i, MarkerCommand.TURN_RIGHT) for i in (5, 6)],
    *[(i, MarkerCommand.STRAIGHT) for i in (7, 8)],
    *[(i, MarkerCommand.STOP) for i in (0, 9, 49)],
])
def test_deployed_configuration_dispatches_each_requested_id(marker_id, expected):
    _, config = configured()
    reader, memory = MarkerReader(config), CarMemory()
    assert read(reader, memory, [marker(marker_id)], [marker(marker_id)]) == [None, expected]
    assert memory.data.intersection_command == (None if expected is MarkerCommand.STOP else expected)


@pytest.mark.parametrize("changes", [
    {"turn_left_ids": [1, 1]}, {"turn_left_ids": [1], "turn_right_ids": [1]},
    {"turn_left_ids": [True]}, {"turn_left_ids": [1.5]}, {"turn_left_ids": [-2]},
    {"turn_left_ids": []}, {"turn_left_ids": "1,2"}, {"turn_left_ids": 1},
    {"confirm_steps": 1.5}, {"confirm_steps": True},
])
def test_invalid_groups_cannot_start(changes):
    with pytest.raises(ValueError):
        MarkerConfig(**changes)


def test_ros_integer_arrays_are_normalised_and_unknown_marker_does_not_replay_old_direction():
    config = MarkerConfig(turn_left_ids=array("q", [1, 2, 3, 4]))
    assert config.turn_left_ids == (1, 2, 3, 4)
    reader, memory = MarkerReader(config), CarMemory()
    read(reader, memory, [marker(2)], [marker(2)], [], None)
    assert memory.data.intersection_command is MarkerCommand.TURN_LEFT
    assert read(reader, memory, [marker(9)], [marker(9)]) == [None, MarkerCommand.STOP]


@pytest.mark.parametrize("action_policy", ["mpc", "open_space"])
def test_unsupported_stop_persists_after_marker_loss_and_valid_marker_until_explicit_restart(action_policy):
    settings, _ = configured()
    settings["action_policy"] = action_policy
    policy, memory = build_policy(settings), CarMemory()
    initial = policy.step(straight_lane(first=True), memory)
    if action_policy == "mpc":
        assert initial.drive_action > 0.
    for _ in range(2):
        actions = policy.step(straight_lane(markers=[marker(9)]), memory)
    assert actions.drive_action <= 0.
    assert actions.steering_action == policy.car_controller.config.steering_centre_action
    assert memory.data.mpc_stop is MpcStop.MARKER
    assert actions.camera_pan_action is None
    assert policy.lanes is None and policy.cones is None and policy.action_policy.lane_state is None
    for detections in ([], None, [marker(1)], [marker(1)]):
        assert policy.step(straight_lane(markers=detections), memory).drive_action <= 0.
        assert memory.data.mpc_stop is MpcStop.MARKER
        assert memory.data.intersection_command is None
    assert CarMemory.deserialise(memory.data.serialise()).data == memory.data
    restarted = policy.step(straight_lane(first=True), memory)
    if action_policy == "mpc":
        assert restarted.drive_action > 0.
    assert memory.data.mpc_stop is None


def test_single_step_confirmation_can_stop_on_the_first_policy_step():
    settings, _ = configured()
    settings["markers.confirm_steps"] = 1
    policy, memory = build_policy(settings), CarMemory()
    actions = policy.step(straight_lane(markers=[marker(0)], first=True), memory)
    assert actions.drive_action <= 0. and memory.data.mpc_stop is MpcStop.MARKER


def test_unsupported_marker_stops_an_active_turn_and_clears_the_selected_route():
    settings, _ = configured()
    settings["intersection.enabled"] = False
    settings["intersection_navigator.enabled"] = True
    policy, memory = build_policy(settings), CarMemory()
    memory.update_intersection(active_state())
    fiducials = FiducialBatch([marker(9)], SensorAge(0., None), "DICT_4X4_50", "base_link")
    policy.step(replace(observation(), fiducials=fiducials), memory)
    assert policy.selected_lane is not None
    actions = policy.step(replace(observation(time=1.2), fiducials=fiducials), memory)
    assert actions.drive_action <= 0. and memory.data.mpc_stop is MpcStop.MARKER
    assert memory.data.intersection.phase == "stopped" and not memory.data.intersection.route
    assert memory.data.mpc_plan is None
    assert policy.selected_lane is None and policy.intersections.selected_lane is None


def test_missing_lane_and_no_previous_plan_cannot_produce_forward_drive():
    settings, _ = configured()
    policy, memory = build_policy(settings), CarMemory()
    obs = replace(straight_lane(first=True), cones=None)
    assert policy.step(obs, memory).drive_action <= 0.


def test_unknown_marker_clears_the_new_marker_based_manoeuvre_too():
    settings, _ = configured()
    settings["markers.confirm_steps"] = 1
    policy, memory = build_policy(settings), CarMemory()
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_LEFT, TurnPhase.ARC, 1., 0., 0., 1.))
    policy.intersection_turn = memory.data.intersection_turn
    policy.manoeuvre = (.3, .5)
    actions = policy.step(straight_lane(markers=[marker(49)]), memory)
    assert actions.drive_action <= 0. and memory.data.mpc_stop is MpcStop.MARKER
    assert memory.data.intersection_turn is None
    assert policy.intersection_turn is None and policy.manoeuvre is None


@pytest.mark.parametrize("marker_id, direction", [(1, 1.), (5, -1.)])
def test_confirmed_turn_immediately_prepares_outside_at_turn_speed_and_keeps_target_without_cones(marker_id, direction):
    settings, _ = configured()
    settings["markers.confirm_steps"] = 1
    policy, memory = build_policy(settings), CarMemory()
    actions = policy.step(straight_lane(markers=[marker(marker_id, x=1.8, y=.3)], first=True), memory)
    assert policy.lanes.has_lane
    assert memory.data.intersection_turn.phase is TurnPhase.APPROACH
    assert policy.manoeuvre[0] == settings["intersection.turn_speed_m_per_s"]
    assert direction * policy.manoeuvre[1] < 0.  # Prepare to the side opposite the turn.
    assert actions.drive_action > 0.
    missing = replace(straight_lane(), cones=None, fiducials=None, lidar_cartesian=lidar([]))
    assert policy.step(missing, memory).drive_action > 0.
    assert direction * policy.manoeuvre[1] < 0.  # Do not freeze the incomplete current offset.
    assert policy.obstacles.path_curvature_per_m == policy.manoeuvre[1]
    assert policy.manoeuvre_path_m is not None
    blocked = replace(missing, lidar_cartesian=lidar(arc_points(.55, policy.manoeuvre[1])))
    for _ in range(settings["mpc.obstacle_confirm_steps"]):
        actions = policy.step(blocked, memory)
    assert actions.drive_action <= 0. and memory.data.mpc_stop is MpcStop.OBSTACLE


@pytest.mark.parametrize("command, side, radius", [
    (MarkerCommand.TURN_LEFT, 1., 1. / .48), (MarkerCommand.TURN_RIGHT, -1., 1.2),
])
def test_actual_turn_command_drives_obstacle_corridor_and_real_obstacles_still_latch(command, side, radius):
    settings, _ = configured()
    policy, memory = build_policy(settings), CarMemory()
    memory.start_map(None)
    # Offset away from the arc, requiring a pursuit correction above the nominal curvature.
    memory.move_car(CarPose(0., -side * .3, 0.), None)
    memory.plan_turn(IntersectionTurn(command, TurnPhase.ARC, 0., 0., 0., 1.2, "CENTRE", 0., side * radius))
    obs = replace(straight_lane(), cones=None, lidar_cartesian=lidar([]))
    assert policy.step(obs, memory).drive_action > 0.
    curvature = policy.manoeuvre[1]
    assert side * curvature > 0.
    assert policy.obstacles.path_curvature_per_m == curvature
    drawn_arc = _intersection_lines(policy)[1].points
    pose, turn = policy.pose, memory.data.intersection_turn
    # Every displayed point belongs to the effective circle, including when only the remaining arc is drawn.
    for x, y in drawn_arc:
        world_x = pose.x_m + math.cos(pose.heading_rad) * x - math.sin(pose.heading_rad) * y
        world_y = pose.y_m + math.sin(pose.heading_rad) * x + math.cos(pose.heading_rad) * y
        assert math.hypot(world_x - turn.arc_centre_x_m, world_y - turn.arc_centre_y_m) == pytest.approx(radius)
    # The guard still sees a physical object along the corrected path on either turn.
    blocked = replace(obs, lidar_cartesian=lidar(arc_points(.55, curvature)))
    for _ in range(policy.action_policy.config.obstacle_confirm_steps):
        actions = policy.step(blocked, memory)
    assert memory.data.mpc_stop is MpcStop.OBSTACLE
    assert actions.drive_action <= 0.
    assert policy.step(obs, memory).drive_action <= 0.
    assert memory.data.mpc_stop is MpcStop.OBSTACLE


def test_an_infeasible_left_arc_latches_no_path_stop_until_an_explicit_new_run():
    settings, _ = configured()
    policy, memory = build_policy(settings), CarMemory()
    memory.start_map(None)
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_LEFT, TurnPhase.APPROACH, 0., 0., 0., .8))
    obs = replace(straight_lane(), cones=None, lidar_cartesian=lidar([]))
    assert policy.step(obs, memory).drive_action <= 0.
    assert memory.data.mpc_stop is MpcStop.LANE_LOSS
    assert policy.step(straight_lane(), memory).drive_action <= 0.
    assert memory.data.mpc_stop is MpcStop.LANE_LOSS
    assert policy.step(straight_lane(first=True), memory).drive_action > 0.
    assert memory.data.mpc_stop is None


@pytest.mark.parametrize("confirm_steps", [1, 2])
@pytest.mark.parametrize("marker_id, command, side", [
    *[(i, MarkerCommand.TURN_LEFT, "RIGHT") for i in range(1, 5)],
    *[(i, MarkerCommand.TURN_RIGHT, "LEFT") for i in (5, 6)],
    *[(i, MarkerCommand.STRAIGHT, "CENTRE") for i in (7, 8)],
])
def test_grouped_ids_prepare_the_pulled_turner_and_keep_position_after_marker_loss(
        marker_id, command, side, confirm_steps):
    settings, _ = configured()
    settings["markers.confirm_steps"] = confirm_steps
    policy, memory = build_policy(settings), CarMemory()
    for step in range(confirm_steps):
        policy.step(straight_lane(markers=[marker(marker_id)], first=step == 0), memory)
    assert memory.data.intersection_turn.command is command
    assert policy.action_policy.lane_side == side
    clearance = (policy.action_policy.config.edge_clearance_m if side == "CENTRE"
                 else policy.turner.config.aside_clearance_m)
    assert policy.action_policy.edge_clearance_m == clearance
    for detections in ([], None):
        policy.step(straight_lane(markers=detections), memory)
        assert memory.data.intersection_turn.command is command
        assert policy.action_policy.lane_side == side
        assert policy.action_policy.edge_clearance_m == clearance


def test_straight_signal_and_completed_turn_restore_the_normal_lane_position():
    settings, _ = configured()
    policy, memory = build_policy(settings), CarMemory()
    policy.step(straight_lane(first=True), memory)
    for _ in range(2):
        policy.step(straight_lane(markers=[marker(6)]), memory)
    assert policy.action_policy.lane_side == "LEFT"
    for _ in range(2):
        policy.step(straight_lane(markers=[marker(8)]), memory)
    assert memory.data.intersection_turn.command is MarkerCommand.STRAIGHT
    assert policy.action_policy.lane_side == "CENTRE"
    assert policy.action_policy.edge_clearance_m == policy.action_policy.config.edge_clearance_m
    for _ in range(2):
        policy.step(straight_lane(markers=[marker(1)]), memory)
    assert policy.action_policy.lane_side == "RIGHT"
    policy.turner._finish(memory.data.intersection_turn, memory)
    assert memory.data.intersection_turn is None
    assert policy.action_policy.lane_side == "CENTRE"
    assert policy.action_policy.edge_clearance_m == policy.action_policy.config.edge_clearance_m
