"""Runtime corridor integration: real control conversion, sensor guards, lidar turns and explicit-run resets."""
from dataclasses import replace
import math
import numpy as np

import pytest

from policy.action_policy.corridor import CorridorControlConfig, rounded_path, swept_lidar
from policy.action_policy.mpc import MpcConfig
from policy.cone_filter.cone_filter import ConeFilterConfig
from policy.control.control import ControlConfig, steering_for_curvature
from policy.input_output import (AngularVelocity, CarObservations, ConeBatch, ConeColour, ConeDetection,
                                 LidarCartesian, LidarCartesianPoint, ObservedCarState, PolicyState,
                                 Position, SensorAge, WheelSpeed)
from policy.memory import CarMemory
from policy.policy_runner import MovementPolicy
from policy.visualisation import StepDrawer, summary


def observations(t=0., *, points=(), speed=.4, cone_age=0., lidar_age=0., cones=True):
    batch = [ConeDetection(Position(x, y, .05), colour, 1.) for x in (.5, 1., 1.5, 2., 2.5)
             for y, colour in ((.5, ConeColour.BLUE), (-.5, ConeColour.YELLOW))]
    return CarObservations(
        ConeBatch(batch if cones else [], SensorAge(cone_age, round((10+t-cone_age)*1e9)), 0.),
        None, None, LidarCartesian([LidarCartesianPoint(Position(x, y, .12), i)
                                   for i, (x, y) in enumerate(points)], SensorAge(lidar_age, None), "base_link"),
        ObservedCarState(None, AngularVelocity(0., 0., 0., SensorAge(0., None)), None,
                         WheelSpeed(speed, SensorAge(0., None))), PolicyState(0. if t == 0 else .1, t, t == 0))


def movement(**changes):
    config = CorridorControlConfig(lookahead_m=1.2, nominal_width_m=1., route_memory_enabled=True,
                                   route_max_age_s=6.5, route_max_distance_m=2.6,
                                   max_left_curvature_per_m=1., **changes)
    return MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig(brake_action=.3),
                          action_policy="corridor", corridor_config=config)


def test_selected_mode_uses_existing_control_and_fixed_camera_and_draws_connected_route():
    policy, memory = movement(), CarMemory()
    obs = observations()
    actions = policy.step(obs, memory)
    assert actions.drive_action > 0 and abs(actions.steering_action) <= 1
    assert actions.camera_pan_action == 0
    assert policy.corridor_policy.last_command.speed_m_per_s == .4
    assert memory.data.corridor_experiment.runtime_started
    assert memory.data.corridor_experiment.runtime_route
    layers = StepDrawer().draw(policy, obs, actions)
    assert layers["lane"].lines and layers["plan/mpc"].lines
    assert "connected corridor" in summary(policy, obs, actions)
    assert CarMemory.deserialise(memory.data.serialise()).data == memory.data


def test_close_lidar_return_brakes_and_remains_latched_until_new_explicit_run():
    policy, memory = movement(), CarMemory()
    assert policy.step(observations(), memory).drive_action > 0
    blocked = observations(.1, points=((.65, -.01), (.65, 0), (.65, .01)))
    assert policy.step(blocked, memory).drive_action == -.3
    assert memory.data.corridor_experiment.stopped
    assert "lidar obstacle" in memory.data.corridor_experiment.stop_reason
    assert policy.step(observations(.2), memory).drive_action <= 0
    assert policy.step(observations(), memory).drive_action > 0


@pytest.mark.parametrize("field", ["cones", "speed", "gyro", "lidar"])
def test_sensor_loss_stops_without_automatic_recovery(field):
    policy, memory = movement(), CarMemory()
    policy.step(observations(), memory)
    obs = observations(.1)
    if field == "cones": obs = replace(obs, cones=None)
    if field == "speed": obs = replace(obs, car=replace(obs.car, wheel_speed=None))
    if field == "gyro": obs = replace(obs, car=replace(obs.car, ang_velocity=None))
    if field == "lidar": obs = replace(obs, lidar_cartesian=None)
    assert policy.step(obs, memory).drive_action <= 0
    assert policy.step(observations(.2), memory).drive_action <= 0


def test_empty_cones_and_overspeed_stop_the_run():
    for obs in (observations(.1, cones=False), observations(.1, speed=.8)):
        policy, memory = movement(), CarMemory()
        policy.step(observations(), memory)
        assert policy.step(obs, memory).drive_action <= 0
        assert memory.data.corridor_experiment.stopped


def test_startup_waits_for_unequal_image_and_lidar_ages_without_latching_before_driving():
    policy, memory = movement(), CarMemory()
    actions = []
    for i in range(6):
        actions.append(policy.step(observations(i/10, cone_age=.05, lidar_age=.35), memory))
    assert all(a.drive_action == 0 for a in actions[:4])
    assert actions[-1].drive_action > 0
    assert not memory.data.corridor_experiment.stopped


def test_lidar_sweep_turns_with_full_footprint_and_does_not_exempt_cones():
    # A cluster near the commanded left arc is well outside the straight corridor.
    angle = .65
    point = (math.sin(angle), 1-math.cos(angle))
    points = [(point[0], point[1]+dy) for dy in (-.01, 0, .01)]
    straight = swept_lidar(points, 0., .3, .3, .25, .075, 1.5, 3)[0]
    curved = swept_lidar(points, 1., .3, .3, .25, .075, 1.5, 3)[0]
    assert curved < straight
    assert math.isfinite(curved)
    assert math.isinf(swept_lidar([(1., .8)], 0., .3, .3, .25, .075, 1.5, 3)[0])


def test_old_corridor_logs_supply_new_runtime_defaults():
    from policy.memory import CorridorExperimentMemory
    assert not CorridorExperimentMemory.deserialise({"started": True}).runtime_started


def test_fresh_support_does_not_renew_the_retained_rounded_route_anchor():
    policy, memory = movement(), CarMemory()
    policy.step(observations(), memory)
    route = memory.data.corridor_experiment.runtime_route
    anchor = memory.data.corridor_experiment.runtime_route_seen_s
    distance = memory.data.corridor_experiment.runtime_route_seen_distance_m
    policy.step(observations(.1), memory)
    state = memory.data.corridor_experiment
    assert state.runtime_route == route and state.runtime_route_seen_s == anchor
    assert state.runtime_route_seen_distance_m == distance


def test_warmup_motion_reset_discards_rounded_plan_in_the_old_odometry_origin():
    policy, memory = movement(), CarMemory()
    assert policy.step(observations(lidar_age=.35), memory).drive_action == 0.
    assert memory.data.corridor_experiment.runtime_route
    obs = observations(.1, lidar_age=.35)
    obs = replace(obs, car=replace(obs.car, ang_velocity=replace(obs.car.ang_velocity, sensor_age=SensorAge(.2, None))))
    assert policy.step(obs, memory).drive_action == 0.
    assert not memory.data.corridor_experiment.runtime_route
    assert memory.data.corridor_experiment.runtime_route_seen_s is None


def test_unsafe_configuration_is_rejected():
    with pytest.raises(ValueError):
        movement(target_speed_m_per_s=.8)
    with pytest.raises(ValueError):
        movement(clearance_cost_weight=math.nan)
    with pytest.raises(ValueError):
        movement(preferred_cone_clearance_m=1.)
    with pytest.raises(ValueError):
        MovementPolicy(MpcConfig(obstacle_stop_enabled=False), ConeFilterConfig(), ControlConfig(),
                       action_policy="corridor")


def test_explicit_endpoint_scaling_uses_full_range_only_in_corridor_mode():
    retained = movement().car_controller.config
    endpoint = movement(full_range_steering=True).car_controller.config
    assert retained.steering_left_action_per_curvature == -1.587
    assert steering_for_curvature(1., endpoint) == pytest.approx(-1.)
    assert steering_for_curvature(-.92, endpoint) == pytest.approx(1.)
    assert steering_for_curvature(0., endpoint) == retained.steering_centre_action


def test_expired_gyro_after_driving_latches_stop_even_before_node_timeout():
    policy, memory = movement(), CarMemory()
    policy.step(observations(), memory)
    obs = observations(.1)
    obs = replace(obs, car=replace(obs.car, ang_velocity=replace(obs.car.ang_velocity, sensor_age=SensorAge(.2, None))))
    assert policy.step(obs, memory).drive_action <= 0
    assert memory.data.corridor_experiment.stopped


def test_lidar_guard_includes_the_centered_steering_braking_escape():
    points = [( .6, y) for y in (-.155, -.15, -.145)]
    follow = swept_lidar(points, 1., .3, .3, .25, .075, 1.5, 3,
                         initial_curvature=1., settle_distance_m=.06)[0]
    assert math.isinf(follow)
    policy, memory = movement(full_range_steering=True), CarMemory()
    obs = observations(points=points)
    obs = replace(obs, car=replace(obs.car, ang_velocity=replace(obs.car.ang_velocity, yaw=.4)))
    assert policy.step(obs, memory).drive_action <= 0
    assert math.isinf(policy.corridor_policy._scan_for_command(1., obs)[0])
    assert policy.corridor_policy._braking_scan[0] < .24


def test_braking_escape_reserves_reaction_travel_before_the_next_guard():
    policy, memory = movement(full_range_steering=True), CarMemory()
    obs = observations(points=[(.61, y) for y in (-.155, -.15, -.145)])
    obs = replace(obs, car=replace(obs.car, ang_velocity=replace(obs.car.ang_velocity, yaw=.4)))
    policy.step(obs, memory)
    assert math.isinf(policy.corridor_policy._scan_for_command(1., obs)[0])
    assert policy.corridor_policy._scan_for_command(0., obs)[0] > .24
    assert not policy.corridor_policy._command_guard(np.array([1.]), obs)[0]
    assert policy.corridor_policy._braking_scan[0] <= .34


def test_observed_corner_rounding_uses_planning_radius_and_preserves_forward_endpoint():
    path = rounded_path(((0., 0.), (3., 0.), (3., 3.)), 1., .92)
    assert path[0] == (0., 0.) and path[-1] == (3., 3.)
    arc = [p for p in path if 2. < p[0] < 3. and 0. < p[1] < 1.]
    assert len(arc) > 20
    assert all(math.hypot(x-2., y-1.) == pytest.approx(1.) for x, y in arc)


def test_short_fresh_terminal_boundary_bridges_backwards_only_with_bounded_support():
    from policy.action_policy.corridor_experiment import boundary_centre_path
    config = CorridorControlConfig(nominal_width_m=1.)
    def fit(xs):
        cones = [ConeDetection(Position(x, -.5, .05), ConeColour.YELLOW, 1.) for x in xs]
        return boundary_centre_path(cones, ConeColour.YELLOW, "BLUE", 1., config, terminal_straight=True)
    path = fit((.8, 1.3))
    assert path[0] == pytest.approx((0., 0.)) and path[-1] == pytest.approx((1.3, 0.))
    assert not fit((.8, .9))  # Insufficient independent span after clustering.
    assert not fit((1.2, 1.7))  # Unobserved backward bridge exceeds the bounded cone gap.
    assert not fit((.4, 1.5))  # Disconnected detections cannot support the terminal straight.


def test_adjacent_rounded_corners_cannot_reverse_on_a_shared_segment():
    path = rounded_path(((0., 0.), (1., 0.), (1., 1.3), (2.6, 2.5)), 1., 1.)
    assert all(b[1] >= a[1]-1e-8 for a, b in zip(path, path[1:]))


def test_extended_first_tangent_cannot_create_a_reverse_initial_connector():
    path = rounded_path(((.1, 0.), (.5, 0.), (.5, 3.)), 1., .92)
    assert all(b[0] >= a[0]-1e-8 for a, b in zip(path, path[1:]))
    assert path[0][1] > 0.  # Trim the rounded arc itself, not an artificial straight back to its tangent.


def test_stop_delay_uncertainty_is_zero_at_the_common_initial_pose():
    result = swept_lidar([(.08, .223)]*3, 1., .3, .3, .25, .075, .2525, 3,
                         initial_curvature=.8, settle_distance_m=.06, brake_after_m=0.,
                         delay_uncertainty_m=.0125)
    assert result[0] > 0.


def test_forward_sampling_does_not_artificially_widen_a_straight_side_band():
    result = swept_lidar([(.6, .21)]*3, 0., .3, .3, .25, .075, 1.5, 3)
    assert math.isinf(result[0])
    assert math.isfinite(swept_lidar([(.6, .195)]*3, 0., .3, .3, .25, .075, 1.5, 3)[0])


@pytest.mark.parametrize("delay", [.012, .037, .063, .088])
def test_sampled_stop_envelope_covers_intermediate_delay_footprint_corners(delay):
    # Independently integrate a much finer spatial-response trajectory, then place returns on its body corners.
    ds, x, y, heading, k = .0001, 0., 0., 0., .7
    for s in np.arange(0., delay+.24, ds):
        k += ((1. if s < delay else 0.)-k)*ds/.06
        heading += k*ds
        x, y = x+math.cos(heading)*ds, y+math.sin(heading)*ds
    for forward, left in ((.3, .125), (-.3, .125), (.3, -.125)):
        point = (x+forward*math.cos(heading)-left*math.sin(heading),
                 y+forward*math.sin(heading)+left*math.cos(heading))
        assert any(math.isfinite(swept_lidar(
            [point]*3, 1., .3, .3, .25, 0., d+.24+.0125, 3,
            initial_curvature=.7, settle_distance_m=.06, brake_after_m=d,
            delay_uncertainty_m=.0125)[0]) for d in np.linspace(0., .1, 5))
