"""Tests for the scenario converter.

Run from the repository root with:
python3 -m pytest tests/policy/test_scenario_converter.py
"""

from __future__ import annotations

import json
import logging
from pathlib import Path

import pytest

from scripts.data_converter import (
    ScenarioConverterError,
    ScenarioDescription,
    convert_scenario,
    main,
    parse_descriptions,
    piped_text,
    sync_scenarios,
)

ROOT = Path(__file__).resolve().parents[2]
SCHEMAS = ROOT / "scenarios" / "data" / "schemas"

_OBSERVATION = {"cones": None, "policy": {"dt": 0.0}}
_ACTION = {"drive_action": 0.55, "steering_action": 0.0}
_OBSERVATION_2 = {"cones": None, "policy": {"dt": 0.1}}
_ACTION_2 = {"drive_action": 0.4, "steering_action": 0.1}


class _Tty:
    """Standard input that is a terminal, so the converter does not read it."""

    def isatty(self) -> bool:
        return True

    def read(self) -> str:
        raise AssertionError("stdin should not be read")


class _IdleStream:
    """Standard input that is not a terminal and must not be read."""

    def isatty(self) -> bool:
        return False

    def fileno(self) -> int:
        raise OSError("not a real stream")

    def read(self) -> str:
        raise AssertionError("stdin should not be read")


class _Pipe:
    """Piped standard input holding a descriptions document."""

    def __init__(self, text: str):
        """
        :param text: JSON text presented as a pipe.
        """
        self._text = text

    def isatty(self) -> bool:
        return False

    def read(self) -> str:
        return self._text


def _write_pair(
    folder: Path,
    stem: str,
    observations: list,
    actions: list | None,
):
    """
    :param folder: Folder that should contain the logged pair.
    :param stem: Input stem, such as messy_cone_test.
    :param observations: Observation records to write.
    :param actions: Action records to write, or None to omit the actions file.
    """
    folder.mkdir(parents=True, exist_ok=True)
    observations_path = folder / f"{stem}_observations.json"
    observations_path.write_text(json.dumps(observations), encoding="utf-8")
    if actions is not None:
        actions_path = folder / f"{stem}_actions.json"
        actions_path.write_text(json.dumps(actions), encoding="utf-8")


def _read_scenario(path: Path) -> dict:
    """
    :param path: Scenario file to read.
    :return: The parsed scenario document.
    """
    return json.loads(path.read_text(encoding="utf-8"))


def test_input_name_fills_name_and_output():
    description = ScenarioDescription(input_name="messy_cone_test")
    assert description.input_name == "messy_cone_test"
    assert description.output_name == "messy_cone_scenario"
    assert description.name == "Messy cone"
    assert description.description == ""

    straight = ScenarioDescription(input_name="straight_even_test")
    assert straight.output_name == "straight_even_scenario"
    assert straight.name == "Straight even"


def test_output_name_fills_name_and_input():
    description = ScenarioDescription(output_name="messy_cone_scenario")
    assert description.output_name == "messy_cone_scenario"
    assert description.input_name == "messy_cone_test"
    assert description.name == "Messy cone"


def test_name_fills_input_and_output():
    description = ScenarioDescription(name="Messy cone")
    assert description.input_name == "messy_cone_test"
    assert description.output_name == "messy_cone_scenario"
    assert description.name == "Messy cone"

    spaced = ScenarioDescription(name="Messy   cone")
    assert spaced.input_name == "messy_cone_test"
    assert spaced.output_name == "messy_cone_scenario"


def test_explicit_fields_are_kept():
    description = ScenarioDescription(
        input_name="messy_cone_test",
        output_name="custom_scenario",
        name="Custom label",
        description="Kept",
    )
    assert description.input_name == "messy_cone_test"
    assert description.output_name == "custom_scenario"
    assert description.name == "Custom label"
    assert description.description == "Kept"


def test_name_prefers_input_when_output_is_also_set():
    description = ScenarioDescription(
        input_name="messy_cone_test",
        output_name="custom_scenario",
    )
    assert description.name == "Messy cone"
    assert description.output_name == "custom_scenario"


def test_empty_identity_is_rejected():
    with pytest.raises(ScenarioConverterError, match="needs an input_name"):
        ScenarioDescription()
    with pytest.raises(ScenarioConverterError, match="needs an input_name"):
        ScenarioDescription(description="only a description")
    with pytest.raises(ScenarioConverterError, match="must not be empty"):
        ScenarioDescription(input_name="  ", name="Messy cone")


def test_unknown_and_non_string_fields_are_rejected():
    with pytest.raises(ScenarioConverterError, match="Unknown description fields"):
        ScenarioDescription(input_name="messy_cone_test", extra="no")
    with pytest.raises(ScenarioConverterError, match="input_name must be a string"):
        ScenarioDescription(input_name=1)
    with pytest.raises(ScenarioConverterError, match="must be a file stem"):
        ScenarioDescription(input_name="folder/messy_cone_test")


def test_descriptions_document_shape():
    entries = parse_descriptions(
        '[{"input_name": "messy_cone_test", "description": "A run"}]'
    )
    assert entries == [{"input_name": "messy_cone_test", "description": "A run"}]
    with pytest.raises(ScenarioConverterError, match="JSON array"):
        parse_descriptions('{"input_name": "messy_cone_test"}')
    with pytest.raises(ScenarioConverterError, match="Unknown description fields"):
        parse_descriptions('[{"input_name": "messy_cone_test", "title": "x"}]')
    with pytest.raises(ScenarioConverterError, match="not valid JSON"):
        parse_descriptions("{")


def test_descriptions_file_and_stdin_reject_duplicates(tmp_path):
    source = tmp_path / "in"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    descriptions_file = tmp_path / "descriptions.json"
    descriptions_file.write_text(
        '[{"input_name": "messy_cone_test", "description": "From file"}]',
        encoding="utf-8",
    )
    with pytest.raises(ScenarioConverterError, match="Duplicate input_name"):
        sync_scenarios(
            source,
            tmp_path / "out",
            descriptions_file=descriptions_file,
            descriptions_text='[{"name": "Messy cone"}]',
        )


def test_duplicate_output_names_are_rejected(tmp_path):
    source = tmp_path / "in"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    _write_pair(source, "straight_even_test", [_OBSERVATION], [_ACTION])
    descriptions_file = tmp_path / "descriptions.json"
    descriptions_file.write_text(
        '[{"input_name": "messy_cone_test", "output_name": "straight_even_scenario"}]',
        encoding="utf-8",
    )
    with pytest.raises(ScenarioConverterError, match="Duplicate output_name"):
        sync_scenarios(
            source,
            tmp_path / "out",
            descriptions_file=descriptions_file,
        )


def test_sync_writes_every_pair_and_leaves_scanned_description_empty(tmp_path):
    source = tmp_path / "in"
    destination = tmp_path / "out"
    observations = [_OBSERVATION, _OBSERVATION_2]
    actions = [_ACTION, _ACTION_2]
    _write_pair(source, "messy_cone_test", observations, actions)
    _write_pair(source, "straight_even_test", [_OBSERVATION], [_ACTION])
    descriptions_file = tmp_path / "descriptions.json"
    descriptions_file.write_text(
        '[{"input_name": "messy_cone_test", "description": "A messy cone run"}]',
        encoding="utf-8",
    )
    destination.mkdir()
    (destination / "unrelated.txt").write_text("keep", encoding="utf-8")

    written = sync_scenarios(
        source,
        destination,
        descriptions_file=descriptions_file,
    )

    assert written == [
        destination / "messy_cone_scenario.json",
        destination / "straight_even_scenario.json",
    ]
    messy = _read_scenario(written[0])
    assert messy["name"] == "Messy cone"
    assert messy["description"] == "A messy cone run"
    assert messy["test_data"] == [
        {"observations": _OBSERVATION, "actions": _ACTION},
        {"observations": _OBSERVATION_2, "actions": _ACTION_2},
    ]
    straight = _read_scenario(written[1])
    assert straight["name"] == "Straight even"
    assert straight["description"] == ""
    assert (destination / "unrelated.txt").read_text(encoding="utf-8") == "keep"


def test_overwrite_keeps_old_name_and_description_when_new_ones_are_defaults(
    tmp_path,
):
    source = tmp_path / "in"
    destination = tmp_path / "out"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    sync_scenarios(
        source,
        destination,
        descriptions_text=(
            '[{"input_name": "messy_cone_test", "name": "Custom messy", '
            '"description": "Hand-written notes"}]'
        ),
    )
    _write_pair(
        source,
        "messy_cone_test",
        [_OBSERVATION, _OBSERVATION_2],
        [_ACTION, _ACTION_2],
    )

    written = sync_scenarios(source, destination)

    assert written == [destination / "messy_cone_scenario.json"]
    scenario = _read_scenario(written[0])
    assert scenario["name"] == "Custom messy"
    assert scenario["description"] == "Hand-written notes"
    assert scenario["test_data"] == [
        {"observations": _OBSERVATION, "actions": _ACTION},
        {"observations": _OBSERVATION_2, "actions": _ACTION_2},
    ]


def test_overwrite_replaces_name_and_description_when_explicitly_set(tmp_path):
    source = tmp_path / "in"
    destination = tmp_path / "out"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    sync_scenarios(
        source,
        destination,
        descriptions_text=(
            '[{"input_name": "messy_cone_test", "name": "Custom messy", '
            '"description": "Hand-written notes"}]'
        ),
    )

    written = sync_scenarios(
        source,
        destination,
        descriptions_text=(
            '[{"input_name": "messy_cone_test", "name": "Updated messy", '
            '"description": "New notes"}]'
        ),
    )

    scenario = _read_scenario(written[0])
    assert scenario["name"] == "Updated messy"
    assert scenario["description"] == "New notes"


def test_sync_ignore_list_skips_input_name_and_display_name(tmp_path):
    source = tmp_path / "in"
    destination = tmp_path / "out"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    _write_pair(source, "straight_even_test", [_OBSERVATION], None)
    _write_pair(source, "outlier_cone_test", [_OBSERVATION], [_ACTION])

    written = sync_scenarios(
        source,
        destination,
        ignore_list="messy_cone_test, Straight even",
    )

    assert written == [destination / "outlier_cone_scenario.json"]
    assert not (destination / "straight_even_scenario.json").exists()


def test_dry_run_writes_nothing(tmp_path, caplog):
    source = tmp_path / "in"
    destination = tmp_path / "out"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    caplog.set_level(logging.INFO, logger="scripts.data_converter")

    written = sync_scenarios(source, destination, dry_run=True)

    assert written == [destination / "messy_cone_scenario.json"]
    assert not destination.exists()
    assert "Would write" in caplog.text
    assert "Messy cone" in caplog.text


def test_dry_run_still_rejects_a_length_mismatch(tmp_path):
    source = tmp_path / "in"
    _write_pair(source, "messy_cone_test", [_OBSERVATION, _OBSERVATION_2], [_ACTION])
    with pytest.raises(ScenarioConverterError, match="2 observations and 1 actions"):
        sync_scenarios(source, tmp_path / "out", dry_run=True)
    assert not (tmp_path / "out").exists()


def test_missing_actions_file_is_rejected(tmp_path):
    source = tmp_path / "in"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], None)
    with pytest.raises(ScenarioConverterError, match="Actions file does not exist"):
        sync_scenarios(source, tmp_path / "out")
    assert not (tmp_path / "out").exists()


def test_convert_from_input_matches_sync_names(tmp_path):
    source = tmp_path / "in"
    destination = tmp_path / "out"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    observations_path = source / "messy_cone_test_observations.json"

    converted = convert_scenario(
        input_value=str(observations_path),
        description="From convert",
        output_folder=destination,
    )
    synced = sync_scenarios(source, tmp_path / "synced")

    assert converted == destination / "messy_cone_scenario.json"
    assert _read_scenario(converted)["name"] == _read_scenario(synced[0])["name"]
    assert _read_scenario(converted)["description"] == "From convert"
    assert _read_scenario(converted)["test_data"] == _read_scenario(synced[0])["test_data"]


def test_convert_from_name_only(tmp_path, monkeypatch):
    source = tmp_path / "in"
    destination = tmp_path / "out"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    monkeypatch.setattr("scripts.data_converter.DEFAULT_INPUT_FOLDER", source)
    monkeypatch.setattr("scripts.data_converter.DEFAULT_OUTPUT_FOLDER", destination)

    code = main(["convert", "-n", "Messy cone"], stdin=_Tty())

    assert code == 0
    scenario = _read_scenario(destination / "messy_cone_scenario.json")
    assert scenario["name"] == "Messy cone"
    assert scenario["test_data"] == [
        {"observations": _OBSERVATION, "actions": _ACTION},
    ]


def test_convert_cli_overrides_piped_description(tmp_path):
    source = tmp_path / "in"
    destination = tmp_path / "out"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    observations_path = source / "messy_cone_test_observations.json"
    piped = _Pipe('{"name": "Other name", "description": "Piped"}')

    code = main(
        [
            "convert",
            "-i",
            str(observations_path),
            "-o",
            str(destination / "messy_cone_scenario.json"),
            "-n",
            "Messy cone",
            "-d",
            "From the flag",
        ],
        stdin=piped,
    )

    assert code == 0
    scenario = _read_scenario(destination / "messy_cone_scenario.json")
    assert scenario["name"] == "Messy cone"
    assert scenario["description"] == "From the flag"


def test_convert_accepts_a_one_item_descriptions_array(tmp_path):
    source = tmp_path / "in"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    destination = convert_scenario(
        name="Messy cone",
        descriptions_text='[{"description": "From an array"}]',
        input_folder=source,
        output_folder=tmp_path / "out",
    )
    assert _read_scenario(destination)["description"] == "From an array"
    with pytest.raises(ScenarioConverterError, match="one description object"):
        convert_scenario(
            name="Messy cone",
            descriptions_text="[{}, {}]",
            input_folder=source,
            output_folder=tmp_path / "out",
        )


def test_main_sync_dry_run_and_missing_convert_flags(tmp_path, caplog):
    source = tmp_path / "in"
    destination = tmp_path / "out"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    caplog.set_level(logging.INFO, logger="scripts.data_converter")

    code = main(
        ["sync", "-i", str(source), "-o", str(destination), "-d", "-x", "messy_cone_test"],
        stdin=_Tty(),
    )
    assert code == 0
    assert not destination.exists()
    assert "Skipping ignored input messy_cone_test" in caplog.text

    failed = main(["convert"], stdin=_Tty())
    assert failed == 1
    assert "convert requires --input or --name" in caplog.text


def test_idle_stream_is_not_read():
    assert piped_text(_Tty()) is None
    assert piped_text(_IdleStream()) is None


def test_sync_reads_piped_descriptions(tmp_path):
    source = tmp_path / "in"
    destination = tmp_path / "out"
    _write_pair(source, "messy_cone_test", [_OBSERVATION], [_ACTION])
    code = main(
        ["sync", "-i", str(source), "-o", str(destination)],
        stdin=_Pipe('[{"input_name": "messy_cone_test", "description": "Piped"}]'),
    )
    assert code == 0
    assert _read_scenario(destination / "messy_cone_scenario.json")["description"] == "Piped"
