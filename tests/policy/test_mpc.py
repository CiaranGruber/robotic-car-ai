"""Hardware-free checks of the MPC movement policy, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_mpc.py
"""
import dataclasses
import math

import pytest

from policy.action_policy.mpc import MpcConfig
from policy.action_policy.policy import MpcPolicy
from policy.action_policy.reference import reference_from_cones
from policy.control.actions import DriveCommand
from policy.input_output import (
    CarObservations, ConeBatch, ConeColour, ConeDetection, ObservedCarState, PolicyState, Position, SensorAge)

LANE_WIDTH_M = 1.0


def lane_cones(offset_m, heading_rad, colours=(ConeColour.YELLOW, ConeColour.BLUE)):
    """Cones of a straight lane, seen from a car offset_m left of the lane centre and heading_rad left of the
    lane direction. Yellow is on the left and blue on the right, 0.5 m apart, visible up to 4 m ahead."""
    cones = []
    for colour, side in ((ConeColour.YELLOW, 1.0), (ConeColour.BLUE, -1.0)):
        if colour not in colours:
            continue
        for i in range(-4, 20):
            along, across = i * 0.5, side * LANE_WIDTH_M / 2.0 - offset_m
            x = math.cos(heading_rad) * along + math.sin(heading_rad) * across
            y = -math.sin(heading_rad) * along + math.cos(heading_rad) * across
            if 0.0 < x <= 4.0:
                cones.append(ConeDetection(Position(x, y, 0.1), colour, 0.9))
    return cones


def observe(cones, first_step=False, cone_age_s=0.0):
    return CarObservations(
        cones=(None if cones is None else ConeBatch(cones, SensorAge(cone_age_s, None), acquisition_to_publish_latency_s=0.0)),
        fiducials=None,
        lidar_scan=None,
        lidar_cartesian=None,
        car=ObservedCarState(None, None, None, None),
        policy=PolicyState(dt=0.0 if first_step else 0.1, policy_elapsed_s=0.0, is_first_policy_step=first_step),
    )


def command(policy, cones, first_step=True, cone_age_s=0.0):
    instructions = policy.run_policy(observe(cones, first_step, cone_age_s), None)
    assert len(instructions) == 1 and isinstance(instructions[0], DriveCommand)
    return instructions[0]


def test_centred_car_drives_straight_at_the_target_speed():
    result = command(MpcPolicy(MpcConfig()), lane_cones(0.0, 0.0))
    assert result.speed_m_per_s == MpcConfig().target_speed_m_per_s
    assert result.curvature_per_m == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("offset_m, heading_rad", [(0.3, 0.0), (-0.3, 0.0), (0.0, 0.2), (0.0, -0.2)])
def test_turns_back_towards_the_lane_centre(offset_m, heading_rad):
    result = command(MpcPolicy(MpcConfig()), lane_cones(offset_m, heading_rad))
    # Left of the centre or pointing left needs a right turn (negative curvature), and the reverse
    assert math.copysign(1.0, result.curvature_per_m) == -math.copysign(1.0, offset_m + heading_rad)


def test_curvature_stays_within_the_limit():
    config = MpcConfig(max_curvature_per_m=0.5)
    result = command(MpcPolicy(config), lane_cones(0.4, 0.5))
    assert result.curvature_per_m == pytest.approx(-config.max_curvature_per_m)


@pytest.mark.parametrize("colours", [(ConeColour.YELLOW,), (ConeColour.BLUE,)])
def test_one_visible_boundary_still_gives_the_lane_centre(colours):
    state = reference_from_cones(lane_cones(0.2, 0.1, colours), LANE_WIDTH_M)
    both = reference_from_cones(lane_cones(0.2, 0.1), LANE_WIDTH_M)
    assert state.lateral_error_m == pytest.approx(both.lateral_error_m)
    assert state.heading_error_rad == pytest.approx(both.heading_error_rad)


@pytest.mark.parametrize("cones", [
    None,  # no fresh detection batch
    [],  # a valid empty frame
    lane_cones(0.0, 0.0)[:1],  # one cone cannot be fitted
    lane_cones(0.8, 0.0),  # outside the lane: both boundaries are on the right
], ids=["missing", "empty", "one-cone", "outside-lane"])
def test_stops_without_a_usable_lane(cones):
    assert command(MpcPolicy(MpcConfig()), cones) == DriveCommand(0.0, 0.0)


def test_first_policy_step_forgets_the_previous_run():
    policy = MpcPolicy(MpcConfig())
    for _ in range(5):
        command(policy, lane_cones(0.4, 0.0), first_step=False)
    assert policy.previous_curvature_per_m < 0.0
    # A new run starting centred must not continue the previous run's turn
    assert command(policy, lane_cones(0.0, 0.0), first_step=True).curvature_per_m == pytest.approx(0.0, abs=1e-6)


def test_closed_loop_settles_on_the_lane_centre_despite_detection_delay():
    """Drives a kinematic car at the commanded speed and curvature, with each detection one step old."""
    policy = MpcPolicy(MpcConfig())
    step_s, substeps = 0.1, 10
    offset, heading = 0.3, 0.0
    delayed_pose = (offset, heading)
    offsets = []
    for step in range(80):
        result = command(policy, lane_cones(*delayed_pose), first_step=step == 0, cone_age_s=step_s)
        delayed_pose = (offset, heading)
        for _ in range(substeps):
            distance = result.speed_m_per_s * step_s / substeps
            heading += distance * result.curvature_per_m
            offset += distance * math.sin(heading)
        offsets.append(offset)
    assert abs(offset) < 0.02 and abs(heading) < 0.02
    assert min(offsets) > -0.1  # crosses the centre by less than 10 cm
    assert policy.fallback_count == 0


@pytest.mark.parametrize("change", [
    {"horizon_steps": 0}, {"horizon_steps": 2.0}, {"step_s": 0.0}, {"curvature_weight": 0.0},
    {"lateral_error_weight": -1.0}, {"target_speed_m_per_s": math.nan}, {"max_curvature_per_m": math.inf},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        dataclasses.replace(MpcConfig(), **change)
