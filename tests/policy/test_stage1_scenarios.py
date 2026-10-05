"""Stage 1 test scenarios (Card 2): checks of the scenario harness, and the MPC on every tuning case.

Run from the repository root with: python3 -m pytest tests/policy/test_stage1_scenarios.py
The unseen cases only run with --run-unseen, for the final comparison (milestone M1).
"""
import dataclasses
import functools
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import stage1_scenarios as s1  # noqa: E402
from stage1_tuning_cases import TUNING_CASES  # noqa: E402
from policy.action_policy.policy import MpcPolicy  # noqa: E402
from policy.control.actions import DriveCommand  # noqa: E402
from policy.input_output import ConeColour  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent / "unseen"))
from stage1_unseen_cases import UNSEEN_CASES  # noqa: E402


# ------------------------------------------------------------------------------------------------
# The case tables
# ------------------------------------------------------------------------------------------------
def test_case_sets_are_complete_and_separate():
    assert len(TUNING_CASES) >= 4 and len(UNSEEN_CASES) >= 2
    assert all(case.case_set == "tuning" for case in TUNING_CASES)
    assert all(case.case_set == "unseen" for case in UNSEEN_CASES)
    names = [case.name for case in TUNING_CASES + UNSEEN_CASES]
    assert len(names) == len(set(names))


def test_tuning_cases_cover_roads_of_different_sizes():
    assert len({case.lane for case in TUNING_CASES}) >= 3


@pytest.mark.parametrize("change", [
    {"noise_m": -0.01}, {"noise_m": math.nan}, {"delay_s": 0.15}, {"delay_s": -0.1},
    {"detector_range_m": 0.0}, {"car_mass_kg": -1.0}, {"floor": "mud"},
])
def test_invalid_disturbances_are_rejected(change):
    with pytest.raises(ValueError):
        dataclasses.replace(s1.Disturbances(), **change)


@pytest.mark.parametrize("change", [
    {"length_m": 0.0}, {"width_m": math.inf}, {"cone_spacing_m": -0.5}, {"length_m": 6.2},
])
def test_invalid_lanes_are_rejected(change):
    with pytest.raises(ValueError):
        dataclasses.replace(s1.Lane(), **change)


def test_cone_indices_must_be_on_the_case_lane():
    with pytest.raises(ValueError):
        s1.Stage1Case("bad", "tuning", "", disturbances=s1.Disturbances(
            missed_cones=((ConeColour.BLUE, s1.STAGE1_LANE.cones_per_row),)))


def test_lane_sizes():
    assert s1.STAGE1_LANE.cones_per_row == 13
    assert s1.Lane(6.0, 1.2, 0.75).cones_per_row == 9
    assert s1.STAGE1_LANE.floor_area_m == pytest.approx((8.5, 1.7))


# ------------------------------------------------------------------------------------------------
# The harness: lane, disturbances, stand-in control and metrics
# ------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("lane", list(dict.fromkeys(case.lane for case in TUNING_CASES + UNSEEN_CASES)),
                         ids=lambda lane: lane.name)
def test_simulated_lane_matches_the_drawing(lane):
    pytest.importorskip("dreamgym")
    env = s1.make_env(s1.Stage1Case("lane", "tuning", "", lane=lane))
    env.reset(seed=0)
    xy, colour_ids = env.unwrapped.road.get_cone_arrays_from_spec()
    env.close()
    simulated = sorted((int(c), round(float(x), 6), round(float(y), 6)) for (x, y), c in zip(xy, colour_ids))
    drawn = sorted((c.value, round(x, 6), round(y, 6)) for c, _, x, y in s1.all_cones(lane))
    assert simulated == drawn
    assert len(drawn) == 2 * lane.cones_per_row


def detector_batch(cones):
    """A dream-gym style observation holding the given (x, y, colour) body-frame detections."""
    return {
        "cone_detections_count": np.array([len(cones)]),
        "cone_detections_forward_positions_in_body_frame_m": np.array([c[0] for c in cones]),
        "cone_detections_left_positions_in_body_frame_m": np.array([c[1] for c in cones]),
        "cone_detections_color_ids": np.array([c[2].value for c in cones]),
    }


def test_missed_and_wrong_colour_cones_are_applied_to_the_right_cones():
    # Car on the start line, centred: cone index i is at body x = 0.5 i
    pose = (0.0, 0.0, 0.0)
    batch = detector_batch([(1.0, 0.5, ConeColour.YELLOW), (1.5, 0.5, ConeColour.YELLOW),
                            (1.0, -0.5, ConeColour.BLUE), (1.52, -0.49, ConeColour.BLUE)])
    case = s1.Stage1Case("d", "tuning", "", disturbances=s1.Disturbances(
        missed_cones=((ConeColour.YELLOW, 2),), wrong_colour_cones=((ConeColour.BLUE, 3),)))
    result = [(d.pos.x, d.pos.y, d.colour) for d in s1.disturbed_detections(batch, pose, case)]
    assert result == pytest.approx([(1.5, 0.5, ConeColour.YELLOW), (1.0, -0.5, ConeColour.BLUE),
                                    (1.52, -0.49, ConeColour.YELLOW)])


def test_extra_objects_are_detected_only_inside_the_camera_view():
    case = s1.Stage1Case("d", "tuning", "", disturbances=s1.Disturbances(
        extra_objects=((2.0, 0.8, ConeColour.YELLOW), (2.0, 5.0, ConeColour.BLUE), (3.5, 0.0, ConeColour.BLUE)),
        detector_range_m=3.0))
    result = [(d.pos.x, d.pos.y, d.colour) for d in s1.disturbed_detections(detector_batch([]), (0.0, 0.0, 0.0), case)]
    # The second is outside the 80 degree view and the third is beyond the 3 m range
    assert result == pytest.approx([(2.0, 0.8, ConeColour.YELLOW)])


def test_floor_and_mass_disturbances_reach_the_car_model():
    pytest.importorskip("dreamgym")
    case = s1.Stage1Case("d", "tuning", "", disturbances=s1.Disturbances(floor="wet", car_mass_kg=3.6))
    env = s1.make_env(case)
    env.reset(seed=0)
    assert env.unwrapped.tire_force_condition == "wet"
    env.close()


class RecordingPolicy:
    """Drives straight at 1 m/s and records the cone observations it receives."""

    def __init__(self):
        self.cones = []

    def run_policy(self, observations, lanes):
        self.cones.append(observations.cones)
        return [DriveCommand(1.0, 0.0)]


def test_detection_delay_holds_back_batches_and_reports_their_age():
    pytest.importorskip("dreamgym")
    policy = RecordingPolicy()
    case = s1.Stage1Case("delay", "tuning", "", disturbances=s1.Disturbances(delay_s=0.2))
    s1.run_case(case, policy)
    assert policy.cones[0] is None and policy.cones[1] is None
    assert policy.cones[2] is not None and policy.cones[2].sensor_age == pytest.approx(0.2)


@pytest.mark.parametrize("curvature, sign", [(0.0, 0.0), (1.0, 1.0), (-1.0, -1.0)])
def test_stand_in_control_steers_left_for_positive_curvature(curvature, sign):
    drive, steering = s1.command_to_action(DriveCommand(1.0, curvature), 1.0)
    assert np.sign(steering) == sign
    assert drive > 0.0


def test_stand_in_control_holds_still_after_a_stop():
    assert s1.command_to_action(DriveCommand(0.0, 0.0), 0.0).tolist() == [0.0, 0.0]
    assert s1.command_to_action(DriveCommand(0.0, 0.0), 1.0)[0] < 0.0  # brakes while moving


def straight_run(y_m, stop_x_m, speed=1.0):
    """A run along the lane at a constant offset, stopping at stop_x_m, with one command per 0.1 s."""
    x = np.arange(0.0, stop_x_m + 1e-9, 0.05)
    speeds = np.full_like(x, speed)
    speeds[-1] = 0.0
    case = s1.Stage1Case("synthetic", "tuning", "")
    return s1.Stage1Result(case, x / speed, x, np.full_like(x, y_m), np.zeros_like(x), speeds,
                           commands=[(t, DriveCommand(speed, 0.0)) for t in np.arange(0.0, x[-1], 0.1)],
                           calc_times_s=[0.001] * 10)


def test_metrics_pass_a_centred_run_that_stops_in_the_zone():
    run = straight_run(0.0, s1.STAGE1_LANE.length_m + 0.3)
    metrics = s1.compute_metrics(run, fallback_count=0)
    assert metrics["cones_touched"] == 0 and metrics["body_outside_lane_m"] == 0.0
    assert metrics["stop_past_last_cone_m"] == pytest.approx(0.3)
    assert s1.check_thresholds(metrics, s1.Thresholds()) == []


def test_metrics_count_touched_cones_and_leaving_the_lane():
    # Centre 0.45 m left: the body reaches 0.575 m, through the yellow row at 0.5 m
    metrics = s1.compute_metrics(straight_run(0.45, s1.STAGE1_LANE.length_m + 0.3), fallback_count=0)
    assert metrics["cones_touched"] == s1.STAGE1_LANE.cones_per_row
    assert metrics["body_outside_lane_m"] == pytest.approx(0.45 + s1.CAR_WIDTH_M / 2 - s1.STAGE1_LANE.width_m / 2)
    failures = s1.check_thresholds(metrics, s1.Thresholds())
    assert any("cones_touched" in f for f in failures) and any("body_outside_lane_m" in f for f in failures)


@pytest.mark.parametrize("stop_x_m", [s1.STAGE1_LANE.length_m - 0.3, s1.STAGE1_LANE.length_m + 0.8])
def test_stopping_outside_the_zone_fails(stop_x_m):
    metrics = s1.compute_metrics(straight_run(0.0, stop_x_m), fallback_count=0)
    assert [f for f in s1.check_thresholds(metrics, s1.Thresholds()) if "stop_past_last_cone_m" in f]


# ------------------------------------------------------------------------------------------------
# The MPC on the cases
# ------------------------------------------------------------------------------------------------
@functools.cache
def mpc_result(case):
    """Runs the shipped MPC settings, set to the case's speed and lane width, once per case."""
    pytest.importorskip("dreamgym")
    return s1.run_case(case, MpcPolicy(s1.mpc_config_for(case)))


def driving_failures(result):
    """The failures other than the stop position, which is tested separately."""
    return [f for f in result.failures if "stop_past_last_cone_m" not in f]


DYNAMIC_MODEL_XFAIL = pytest.mark.xfail(
    reason="Unexplained: on dream-gym's dynamic tyre model the car holds a steady heading error (3 degrees dry, "
           "6 degrees wet) without turning, so it ends 0.12 m off centre. Check the tyre model against the 1:10 car "
           "(Task A2) before tuning on T11.", strict=False)


@pytest.mark.parametrize("case", [pytest.param(case, marks=DYNAMIC_MODEL_XFAIL) if case.disturbances.floor
                                  else case for case in TUNING_CASES], ids=lambda case: case.name)
def test_mpc_drives_the_lane_on_tuning_cases(case):
    result = mpc_result(case)
    assert driving_failures(result) == [], s1.format_table([result])


@pytest.mark.xfail(reason="Known gap: the MPC stops when the last cones leave the camera view, 0.4 to 0.9 m before "
                          "the last cone pair. Task A7 decides how to keep going to the stop zone.", strict=False)
@pytest.mark.parametrize("case", TUNING_CASES, ids=lambda case: case.name)
def test_mpc_stops_in_the_stop_zone_on_tuning_cases(case):
    result = mpc_result(case)
    assert result.passed, s1.format_table([result])


@pytest.mark.unseen
@pytest.mark.parametrize("case", UNSEEN_CASES, ids=lambda case: case.name)
def test_mpc_passes_unseen_cases(case):
    result = mpc_result(case)
    assert result.passed, s1.format_table([result])
