"""Hardware-free checks of the drive report, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_run_report.py
"""
import dataclasses
import itertools
import json

import pytest

from policy.data_logging.data_logging import log_input, log_output, start_data_log, validate_parameters
from policy.input_output import (CarObservations, LidarCartesian, LidarCartesianPoint, ObservedCarState, PolicyState,
                                 Position, SensorAge)
from policy.memory import CarMemory
from scenarios.scenario_wrapper import Scenario, ScenarioType
from scripts import mcap_export, run_report


def _record_data_log(folder, step_count):
    """Drives the policy through the start of a recorded obstacle run and logs it, as policy_node does.

    :return: The data log's observations file.
    """
    settings = mcap_export.read_settings(mcap_export.DEFAULT_SETTINGS_PATH)
    policy = mcap_export.build_policy(settings)
    memory = CarMemory()
    params = start_data_log(validate_parameters(True, str(folder), "info", False), settings)
    scenario = Scenario.from_name(ScenarioType.REAL, "short_block_in_way")
    for observations, _ in itertools.islice(scenario.steps(), step_count):
        log_input(observations, params)
        log_output(policy.step(observations, memory), params)
        memory.step()
    return params.observations_log_path


def test_a_data_log_replayed_with_its_settings_is_reported_as_it_drove(tmp_path):
    observations_path = _record_data_log(tmp_path, 40)
    steps, config_path = mcap_export.recorded_steps(None, observations_path, ScenarioType.REAL)
    result = run_report.report("run", steps, mcap_export.build_policy(mcap_export.read_settings(config_path)), 1.0)

    assert result["steps"] == 40
    assert result["replay_mismatches"] == 0
    assert sum(result["lane_status"].values()) == pytest.approx(1.0)
    assert result["obstacles_seen"]
    assert result["lane_while_driving"]["width_error_median_m"] >= 0.0
    # Every number can be written as JSON for --json
    assert json.loads(json.dumps(result)) == result


def test_folders_give_every_data_log_and_scenario_in_them(tmp_path):
    observations_path = _record_data_log(tmp_path, 2)
    scenario_path = tmp_path / "copy_scenario.json"
    scenario_path.write_text(Scenario.from_name(ScenarioType.REAL, "short_block_in_way").scenario_path.read_text())
    assert run_report.inputs([tmp_path], ["straight_even"]) == [
        ("straight_even", None), (None, observations_path), (None, scenario_path)]
    with pytest.raises(mcap_export.McapExportError):
        run_report.inputs([observations_path.with_name("notes.txt")], [])
    empty = tmp_path / "empty"
    empty.mkdir()
    with pytest.raises(mcap_export.McapExportError):
        run_report.inputs([empty], [])


def test_json_output_has_one_entry_per_drive(capsys):
    assert run_report.main(["-n", "straight_even", "-n", "outlier_cone", "--json"]) == 0
    results = json.loads(capsys.readouterr().out)
    assert [result["name"] for result in results] == ["straight_even", "outlier_cone"]
    assert all(result["replay_mismatches"] is None or result["replay_mismatches"] >= 0 for result in results)


def test_side_clearance_is_the_gap_beside_the_body_without_the_cars_own_returns():
    policy = mcap_export.build_policy(mcap_export.read_settings(mcap_export.DEFAULT_SETTINGS_PATH))
    half_width_m = policy.avoidance.config.car_width_m / 2.0
    points = [(0.10, 0.02),  # Inside the body: the car itself
              (1.00, 0.00),  # Ahead of the car's front: not beside it
              (0.00, half_width_m + 0.20), (-0.10, -(half_width_m + 0.12))]
    lidar = LidarCartesian([LidarCartesianPoint(Position(x, y, 0.12), index) for index, (x, y) in enumerate(points)],
                           SensorAge(0.05, 7), "base_link")
    observations = CarObservations(None, None, None, lidar, ObservedCarState(None, None, None, None),
                                   PolicyState(0.1, 1.0, False))
    step = mcap_export.RecordedStep(observations, None)
    assert run_report._side_clearance_m(step, policy) == pytest.approx(0.12)
    no_lidar = mcap_export.RecordedStep(dataclasses.replace(observations, lidar_cartesian=None), None)
    assert run_report._side_clearance_m(no_lidar, policy) is None
