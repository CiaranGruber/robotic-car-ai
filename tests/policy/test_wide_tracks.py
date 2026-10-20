"""Checks the wide synthetic tracks (wide_tracks.py) and the MPC keeping to the left or right of the lane on them."""
import json
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import wide_tracks as wt  # noqa: E402
from scenarios.scenario_wrapper import Scenario, ScenarioType  # noqa: E402

SCHEMA = Path(__file__).resolve().parents[2] / "scenarios" / "data" / "schemas" / "car_scenario_data_schema.json"


def test_wide_tracks_are_repeatable_and_within_their_ranges():
    assert wt.wide_track(7) == wt.wide_track(7)
    assert wt.wide_track(7) != wt.wide_track(8)
    ranges = wt.WIDE_RANGES
    for track in wt.wide_tracks(wt.TRAINING_SEEDS):
        lane, car = track.case.lane, track.case.car
        assert ranges.lane_width_m[0] <= lane.width_m <= ranges.lane_width_m[1]
        assert abs(car.camera_yaw_deg) <= 3.0 and not car.rolling_start
        for piece in lane.pieces:
            if piece[0] == "arc" and piece[1] > 0:
                assert 1 / piece[1] >= ranges.left_radius_m[0]
        if track.switch_to is not None:
            assert track.switch_to != track.side and math.isfinite(track.switch_after_m)


def test_wide_tracks_cover_both_sides_switches_and_curves():
    tracks = wt.wide_tracks(wt.TRAINING_SEEDS)
    assert {track.side for track in tracks if track.switch_to is None} == {"LEFT", "RIGHT"}
    assert {(track.side, track.switch_to) for track in tracks if track.switch_to} == {("LEFT", "RIGHT"),
                                                                                      ("RIGHT", "LEFT")}
    names = {track.case.lane.name for track in tracks}
    assert any(name.startswith("L ") for name in names) and any(name.startswith("R ") for name in names)
    assert "straight" in names and any("+" in name for name in names)
    assert any(track.case.lane.swapped for track in tracks) and any(track.case.lane.outliers for track in tracks)


def test_training_and_validation_tracks_are_separate():
    assert not set(wt.TRAINING_SEEDS) & set(wt.VALIDATION_SEEDS)


@pytest.mark.parametrize("seed", wt.TRAINING_SEEDS[:10])
def test_mpc_keeps_to_the_asked_side_of_a_clean_wide_track(seed):
    """Without swapped-colour or outlier cones, the car keeps the lane and drives within 0.07 m of the line its lane
    side asks for, switching sides where the track asks."""
    result, scores = wt.run_track(wt.clean_track(wt.wide_track(seed)))
    assert result.lane_kept
    assert scores["side_error_median_m"] < 0.07 and scores["scored_m"] > 1.0


@pytest.mark.xfail(strict=True, reason="lane detection reports a lane about 0.4 m wide, curving at about 1 1/m, "
                                       "from one swapped-colour cone, and the car near the left edge touches a cone")
def test_one_swapped_cone_does_not_send_the_car_into_the_cones():
    """W26: keep left in a 1.35 m lane with one cone seen in the other colour. Centred, the car stops safely."""
    result, _ = wt.run_track(wt.wide_track(26))
    assert result.metrics["cones_touched"] == 0


def test_wide_scenarios_load_like_the_real_ones(tmp_path):
    path = wt.export((4,), tmp_path)[0]
    document = json.loads(path.read_text())
    schema = json.loads(SCHEMA.read_text())
    assert set(document) <= set(schema["properties"]) and set(schema["required"]) <= set(document)
    for step in document["test_data"]:
        assert set(step) == set(schema["$defs"]["scenario_step"]["required"])
    steps = list(Scenario(path).steps())
    assert len(steps) == len(document["test_data"]) > 10
    assert steps[0][0].policy.is_first_policy_step
    assert sum(observations.cones is not None for observations, _ in steps) > len(steps) // 2


def test_saved_wide_scenarios_load():
    names = sorted(wt.SYNTHETIC_SCENARIOS_DIR.glob("wide_w*_scenario.json"))
    assert names, "export some tracks with wide_tracks.py --export"
    for path in names:
        scenario = Scenario.from_name(ScenarioType.SYNTHETIC, path.name.removesuffix("_scenario.json"))
        assert scenario.name.startswith("Synthetic track W") and "mpc.lane_side" in scenario.description
        assert next(scenario.steps())[0].policy.is_first_policy_step
