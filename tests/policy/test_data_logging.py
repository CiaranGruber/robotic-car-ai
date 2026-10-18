"""Hardware-free checks of the drive data-log files, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_data_logging.py
"""
from datetime import datetime
import json

from policy.data_logging import data_logging
from policy.data_logging.data_logging import start_data_log, validate_parameters


class FixedTime(datetime):
    """A clock stopped at one second, so every run starts in the same second."""

    @classmethod
    def now(cls, tz=None):
        return cls(2026, 10, 12, 14, 3, 27)


def test_runs_started_in_the_same_second_get_separate_matching_pairs(tmp_path, monkeypatch):
    monkeypatch.setattr(data_logging, "datetime", FixedTime)
    params = validate_parameters(True, str(tmp_path), "info", False)
    first = start_data_log(params)
    (tmp_path / first.observations_log_filename).write_text('[\n"first run"\n]\n')
    second = start_data_log(params)
    assert (first.observations_log_filename, first.actions_log_path) == (
        "2026-10-12_14-03-27_observations.json", "2026-10-12_14-03-27_actions.json")
    assert (second.observations_log_filename, second.actions_log_path) == (
        "2026-10-12_14-03-27_2_observations.json", "2026-10-12_14-03-27_2_actions.json")
    # The first run's data is kept
    assert json.loads((tmp_path / first.observations_log_filename).read_text()) == ["first run"]


def test_run_settings_are_saved_beside_the_pair(tmp_path, monkeypatch):
    monkeypatch.setattr(data_logging, "datetime", FixedTime)
    settings = {"control.max_drive_action": 0.6, "required_sensors": ["cone_detections", "wheel_speed"]}
    start_data_log(validate_parameters(True, str(tmp_path), "info", False), settings)
    assert json.loads((tmp_path / "2026-10-12_14-03-27_config.json").read_text()) == settings
    # A later run in the same second does not replace them
    later = start_data_log(validate_parameters(True, str(tmp_path), "info", False), {"other": 1})
    assert later.observations_log_filename == "2026-10-12_14-03-27_2_observations.json"
    assert json.loads((tmp_path / "2026-10-12_14-03-27_config.json").read_text()) == settings


def test_nothing_is_written_without_data_logging(tmp_path):
    start_data_log(validate_parameters(False, str(tmp_path), "info", False), {"a": 1})
    assert list(tmp_path.iterdir()) == []
