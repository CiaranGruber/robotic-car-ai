"""Checks the lab car measurements, the camera yaw correction, the synthetic tracks and the MPC tuning rule.

See docs/mpc-lab-calibration.md. Run from the repository root with:
    python -m pytest tests/policy/test_mpc_lab_calibration.py
"""
from __future__ import annotations

import dataclasses
import json
import math
from functools import lru_cache
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mpc_tuning  # noqa: E402
import real_car_measurements as rc  # noqa: E402
import stage1_scenarios as s1  # noqa: E402
import synthetic_tracks as st  # noqa: E402
from policy.action_policy.mpc import MpcConfig  # noqa: E402
from policy.action_policy.policy import MpcPolicy  # noqa: E402
from policy.action_policy.reference import reference_from_cones, rotate_cones  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig  # noqa: E402
from policy.control.actions import DriveCommand  # noqa: E402
from policy.input_output import ConeColour, ConeDetection, Position  # noqa: E402
from scenarios.scenario_wrapper import Scenario, ScenarioType  # noqa: E402

LAB_CASE = s1.Stage1Case("L1", "lab", "Lab lane, centred", 0.0, 0.0, st.SPEED_M_PER_S, lane=rc.LAB_LANE,
                         car=rc.LAB_CAR)


@lru_cache(maxsize=None)
def measured() -> dict[str, float]:
    return rc.measure()


# ---------------------------------------------------------------------------------------------------------------
# The lab car agrees with the recordings
# ---------------------------------------------------------------------------------------------------------------
def test_lab_lane_matches_the_recorded_tracks():
    m = measured()
    assert m["blue_left_fraction"] > 0.95
    assert rc.LAB_LANE.left_colour == ConeColour.BLUE
    assert rc.LAB_LANE.width_m == pytest.approx(m["width_m"], abs=0.07)
    assert rc.LAB_LANE.cone_spacing_m == pytest.approx(m["cone_spacing_m"], abs=0.03)


def test_lab_camera_matches_the_recorded_detections():
    m, car = measured(), rc.LAB_CAR
    assert car.detection_latency_s == pytest.approx(m["detection_latency_s"], abs=0.05)
    assert car.detector_fov_deg / 2 == pytest.approx(m["bearing_p99_deg"], abs=3.0)
    assert car.detector_min_range_m == pytest.approx(m["x_min"], abs=0.05)
    assert car.detector_range_m == pytest.approx(m["x_p95"], abs=0.3)
    assert 0.5 * m["row_residual_m"] <= car.detector_noise_m <= m["row_residual_m"]
    assert car.camera_yaw_deg == pytest.approx(m["camera_yaw_deg"], abs=1.5)


def test_lab_speed_response_matches_the_recordings():
    m = measured()
    assert m["speed_time_constant_min_s"] <= rc.LAB_CAR.speed_time_constant_s <= m["speed_time_constant_max_s"]


def test_simulated_lab_detections_look_like_the_recorded_ones():
    """Driving the simulated lab car straight down the lab lane, as in the recordings, gives detections with the same
    range and the same apparent turn of the rows from the camera yaw."""
    class Straight:
        def run_policy(self, observations, lanes):
            return [DriveCommand(st.SPEED_M_PER_S, 0.0)]

    # No steering trim, so only the camera makes the lane look turned
    case = dataclasses.replace(LAB_CASE, car=dataclasses.replace(rc.LAB_CAR, straight_curvature_per_m=0.0))
    result = s1.run_case(case, Straight())
    xs = [cone.pos.x for observations, _ in result.steps if observations.cones for cone in observations.cones]
    assert min(xs) >= rc.LAB_CAR.detector_min_range_m
    assert max(xs) <= rc.LAB_CAR.detector_range_m + 0.1
    headings = [-math.atan(fit[2]) for observations, _ in result.steps
                if observations.cones and (fit := rc.fit_rows(observations)) is not None]
    assert math.degrees(sorted(headings)[len(headings) // 2]) == pytest.approx(rc.LAB_CAR.camera_yaw_deg, abs=1.5)


# ---------------------------------------------------------------------------------------------------------------
# The camera yaw correction
# ---------------------------------------------------------------------------------------------------------------
def straight_lane_seen_by_yawed_camera(yaw_deg: float) -> list[ConeDetection]:
    """Cones of a straight 0.8 m lane ahead of a centred car, seen by a camera pointing yaw_deg left."""
    cones = [ConeDetection(Position(0.5 + 0.2 * i, side * 0.4, 0.0), colour, 0.9)
             for i in range(8) for side, colour in ((1, ConeColour.BLUE), (-1, ConeColour.YELLOW))]
    return rotate_cones(cones, -math.radians(yaw_deg))


def test_rotate_cones_turns_left_for_positive_angles():
    cone = ConeDetection(Position(1.0, 0.0, 0.1), ConeColour.BLUE, 0.8)
    rotated = rotate_cones([cone], math.radians(90.0))[0]
    assert (rotated.pos.x, rotated.pos.y, rotated.pos.z) == pytest.approx((0.0, 1.0, 0.1))
    assert (rotated.colour, rotated.confidence) == (cone.colour, cone.confidence)
    assert rotate_cones([cone], 0.0) == [cone]


def test_yaw_correction_removes_the_false_heading_error():
    cones = straight_lane_seen_by_yawed_camera(7.0)
    uncorrected = reference_from_cones(cones, 0.8)
    corrected = reference_from_cones(rotate_cones(cones, math.radians(7.0)), 0.8)
    assert math.degrees(uncorrected.heading_error_rad) == pytest.approx(7.0, abs=0.01)
    assert corrected.heading_error_rad == pytest.approx(0.0, abs=1e-9)
    assert corrected.lateral_error_m == pytest.approx(0.0, abs=1e-9)


def test_mpc_steers_right_without_the_yaw_correction_and_straight_with_it():
    cones = s1.ConeBatch(straight_lane_seen_by_yawed_camera(7.0), s1.SensorAge(0.0, None), 0.0)
    observations = s1.CarObservations(cones, None, None, None, s1.ObservedCarState(None, None, None, None),
                                      s1.PolicyState(0.0, 0.0, True))
    uncorrected = MpcPolicy(MpcConfig(lane_width_m=0.8, camera_yaw_offset_deg=0.0)).run_policy(observations, None)[0]
    corrected = MpcPolicy(MpcConfig(lane_width_m=0.8, camera_yaw_offset_deg=7.0)).run_policy(observations, None)[0]
    assert uncorrected.curvature_per_m < -0.05
    assert corrected.curvature_per_m == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("value", [math.nan, math.inf])
def test_camera_yaw_offset_must_be_finite(value):
    with pytest.raises(ValueError):
        MpcConfig(camera_yaw_offset_deg=value)


PAN_HELD_CASE = dataclasses.replace(LAB_CASE, car=dataclasses.replace(rc.LAB_CAR, camera_yaw_deg=0.0))
"""The lab lane with the camera pan held at zero, as the policy now does, assuming that removes the yaw."""


def lab_result(settings: dict, case=LAB_CASE):
    return s1.run_case(case, s1.ConeFilteredPolicy(MpcPolicy(MpcConfig(**settings)), ConeFilterConfig()))


def test_camera_yaw_correction_reduces_tracking_error():
    """Correcting the recorded 7 degree yaw more than halves the endpoint tracking error."""
    uncorrected = lab_result({**mpc_tuning.MEASURED_SETTINGS, "lane_end_distance_m": 0.0})
    corrected = lab_result({**mpc_tuning.MEASURED_SETTINGS, "lane_end_distance_m": 0.0,
                            "camera_yaw_offset_deg": rc.LAB_CAR.camera_yaw_deg})
    assert corrected.metrics["lateral_error_at_lane_end_m"] < 0.5 * uncorrected.metrics["lateral_error_at_lane_end_m"]
    assert corrected.metrics["lateral_error_at_lane_end_m"] < 0.05
    assert mpc_tuning.lane_keeping_passed(corrected)


def test_shipped_mpc_settings_are_the_measured_ones():
    config = MpcConfig()
    for name, value in mpc_tuning.MEASURED_SETTINGS.items():
        assert getattr(config, name) == pytest.approx(value), name
    assert mpc_tuning.lane_keeping_passed(lab_result({}, PAN_HELD_CASE))


# ---------------------------------------------------------------------------------------------------------------
# The lab car in the simulation
# ---------------------------------------------------------------------------------------------------------------
def test_lab_car_clips_curvature_to_what_car_20_can_drive():
    control = s1.StandInControl(rc.LAB_CAR, 0.5)
    left = control.action(DriveCommand(0.5, 3.0), 0.5)[1]
    right = control.action(DriveCommand(0.5, -3.0), 0.5)[1]
    assert left == pytest.approx(math.atan(s1.WHEELBASE_M * rc.LAB_CAR.max_left_curvature_per_m)
                                 / s1.MAX_STEERING_RAD, abs=1e-6)
    assert right == pytest.approx(-math.atan(s1.WHEELBASE_M * rc.LAB_CAR.max_right_curvature_per_m)
                                  / s1.MAX_STEERING_RAD, abs=1e-6)


def test_lab_car_speed_follows_the_measured_first_order_response():
    """From rest, the car does not respond for the dead time, then reaches 63% of the requested speed one time
    constant later."""
    class Fixed:
        def run_policy(self, observations, lanes):
            return [DriveCommand(0.5, 0.0)]

    car = dataclasses.replace(rc.LAB_CAR, straight_curvature_per_m=0.0)
    result = s1.run_case(dataclasses.replace(LAB_CASE, car=car), Fixed())
    dead = result.time_s <= car.speed_dead_time_s
    assert max(result.speed_m_per_s[dead]) < 0.01
    at = int(round((car.speed_dead_time_s + car.speed_time_constant_s) / s1.ENV_STEP_S))
    assert result.speed_m_per_s[at] == pytest.approx(0.63 * 0.5, abs=0.05)


def test_car_profile_rejects_invalid_values():
    for kwargs in ({"detector_min_range_m": 5.0}, {"detection_latency_s": 0.15}, {"speed_time_constant_s": 0.0},
                   {"max_left_curvature_per_m": 0.0}, {"camera_yaw_deg": math.nan}, {"speed_dead_time_s": -0.1}):
        with pytest.raises(ValueError):
            s1.CarProfile(**kwargs)


def test_moved_cones_move_their_detections_and_the_touch_check():
    moved = s1.Disturbances(moved_cones=((ConeColour.BLUE, 5, 0.0, -0.1),))
    case = dataclasses.replace(LAB_CASE, disturbances=moved)
    cones = {(colour, index): (x, y) for colour, index, x, y in s1.case_cones(case)}
    assert cones[(ConeColour.BLUE, 5)] == pytest.approx((1.0, 0.3))
    assert cones[(ConeColour.BLUE, 4)] == pytest.approx((0.8, 0.4))
    with pytest.raises(ValueError):
        dataclasses.replace(LAB_CASE, disturbances=s1.Disturbances(moved_cones=((ConeColour.BLUE, 99, 0.0, 0.0),)))


# ---------------------------------------------------------------------------------------------------------------
# Synthetic tracks
# ---------------------------------------------------------------------------------------------------------------
def test_synthetic_tracks_are_repeatable_and_within_their_ranges():
    ranges = st.LAB_RANGES
    assert st.synthetic_case(7) == st.synthetic_case(7)
    assert st.synthetic_case(7) != st.synthetic_case(8)
    for case in st.synthetic_cases(st.TRAINING_SEEDS[:40]):
        assert ranges.lane_width_m[0] <= case.lane.width_m <= ranges.lane_width_m[1]
        assert case.lane.cone_spacing_m in ranges.cone_spacing_m
        assert abs(case.start_left_m) <= ranges.start_left_m[1]
        assert abs(case.start_heading_deg) <= ranges.start_heading_deg[1]
        assert ranges.camera_yaw_deg[0] <= case.car.camera_yaw_deg <= ranges.camera_yaw_deg[1]
        assert case.car.calibrated_camera_yaw_deg == 0.0
        assert case.case_set == "synthetic" and not case.car.rolling_start


def test_synthetic_tracks_cover_every_recorded_track_type():
    cases = st.synthetic_cases(st.TRAINING_SEEDS)
    assert any(not (c.disturbances.moved_cones or c.disturbances.missed_cones or c.disturbances.wrong_colour_cones
                    or c.disturbances.extra_objects) for c in cases)  # clean, like straight_even
    assert any(c.disturbances.moved_cones for c in cases)  # staggered, like messy_cone
    assert any(c.disturbances.extra_objects and c.disturbances.wrong_colour_cones for c in cases)  # outlier_cone
    assert any(c.lane.left_colour == ConeColour.YELLOW for c in cases)


def test_training_and_validation_tracks_are_separate():
    assert not set(st.TRAINING_SEEDS) & set(st.VALIDATION_SEEDS)


def test_synthetic_scenarios_load_like_the_real_ones(tmp_path):
    path = st.export((3,), tmp_path)[0]
    document = json.loads(path.read_text())
    schema = json.loads((Path(__file__).resolve().parents[2] / "scenarios" / "data" / "schemas"
                         / "car_scenario_data_schema.json").read_text())
    assert set(document) <= set(schema["properties"]) and set(schema["required"]) <= set(document)
    for step in document["test_data"]:
        assert set(step) == set(schema["$defs"]["scenario_step"]["required"])
    steps = list(Scenario(path).steps())
    assert len(steps) == len(document["test_data"]) > 10
    assert all(observations.cones is not None for observations, _ in steps[2:])


def test_saved_synthetic_scenarios_load():
    names = sorted(st.SYNTHETIC_SCENARIOS_DIR.glob("*_scenario.json"))
    assert names, "export some tracks with synthetic_tracks.py --export"
    for path in names:
        scenario = Scenario.from_name(ScenarioType.SYNTHETIC, path.name.removesuffix("_scenario.json"))
        assert scenario.name.startswith("Synthetic track")
        assert next(scenario.steps())[0].policy.is_first_policy_step


# ---------------------------------------------------------------------------------------------------------------
# The tuning rule
# ---------------------------------------------------------------------------------------------------------------
def summary(rate, lateral, smooth, name):
    return {"lane_keeping_passed": rate, "max_abs_lateral_error_m": lateral, "rms_curvature_rate_per_m_s": smooth,
            "settings": {"name": name}}


def test_choose_prefers_pass_rate_then_balances_accuracy_and_smoothness():
    summaries = [summary(0.90, 0.01, 0.01, "accurate but fails more"),
                 summary(1.00, 0.05, 0.40, "accurate"),
                 summary(0.99, 0.06, 0.20, "balanced"),
                 summary(1.00, 0.20, 0.10, "smooth")]
    assert mpc_tuning.choose(summaries, track_count=100)["settings"]["name"] == "balanced"


def test_pareto_front_drops_beaten_designs():
    summaries = [summary(1, 0.05, 0.4, "a"), summary(1, 0.06, 0.2, "b"), summary(1, 0.07, 0.5, "beaten")]
    assert [s["settings"]["name"] for s in mpc_tuning.pareto_front(summaries)] == ["a", "b"]


def test_option_horizons_stay_within_the_measured_detection_range():
    longest = max(mpc_tuning.OPTION_GRID["horizon_steps"]) * MpcConfig().step_s * st.SPEED_M_PER_S
    assert longest <= st.LAB_RANGES.detector_range_m[0]
