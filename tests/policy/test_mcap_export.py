"""Hardware-free checks of the Foxglove MCAP export, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_mcap_export.py
"""
import itertools
import json

from mcap.reader import make_reader

from policy.data_logging.data_logging import log_input, log_output, start_data_log, validate_parameters
from policy.memory import CarMemory
from scenarios.scenario_wrapper import Scenario, ScenarioType
from scripts import mcap_export

TOPICS = {"/cones/detected", "/cones/used", "/lidar/points", "/lane", "/lane/observed", "/obstacles", "/plan/avoidance", "/plan/mpc",
          "/fiducials", "/car", "/policy/state"}
"""Every topic the export writes."""


def test_replay_uses_the_recorded_intersection_settings():
    settings = mcap_export.read_settings(mcap_export.DEFAULT_SETTINGS_PATH)
    settings.update({"intersection.enabled": True, "intersection.turning_radius_m": 3.})
    policy = mcap_export.build_policy(settings)
    assert policy.intersections.config.enabled
    assert policy.intersections.config.turning_radius_m == 3.


def _record_data_log(tmp_path, step_count):
    """Drives the policy through the start of a recorded obstacle run and logs it, as policy_node does.

    :return: The data log's observations file.
    """
    settings = mcap_export.read_settings(mcap_export.DEFAULT_SETTINGS_PATH)
    policy = mcap_export.build_policy(settings)
    memory = CarMemory()
    params = start_data_log(validate_parameters(True, str(tmp_path), "info", False), settings)
    scenario = Scenario.from_name(ScenarioType.REAL, "short_block_in_way")
    for observations, _ in itertools.islice(scenario.steps(), step_count):
        log_input(observations, params)
        log_output(policy.step(observations, memory), params)
        memory.step()
    return params.observations_log_path


def test_a_data_log_replayed_with_its_settings_chooses_its_actions_and_draws_every_step(tmp_path):
    step_count = 20
    observations_path = _record_data_log(tmp_path, step_count)
    output = tmp_path / "run.mcap"

    steps, config_path = mcap_export.recorded_steps(None, observations_path, ScenarioType.REAL)
    mismatches = mcap_export.export(steps, mcap_export.build_policy(mcap_export.read_settings(config_path)), output)

    assert mismatches == 0
    with output.open("rb") as file:
        reader = make_reader(file)
        summary = reader.get_summary()
        counts = {channel.topic: summary.statistics.channel_message_counts[channel_id]
                  for channel_id, channel in summary.channels.items()}
        states = [json.loads(message.data) for _, _, message in reader.iter_messages(topics=["/policy/state"])]
    assert counts == {topic: step_count for topic in TOPICS}
    assert all(state["actions_match"] for state in states)
    # The run reaches the block in the lane within these steps
    assert any(state["lane_obstacle_count"] for state in states)


def test_a_scenario_without_settings_is_replayed_with_the_repository_settings(tmp_path):
    output = tmp_path / "straight.mcap"

    assert mcap_export.main(["-n", "straight_even", "-o", str(output)]) == 0

    with output.open("rb") as file:
        summary = make_reader(file).get_summary()
    assert {channel.topic for channel in summary.channels.values()} == TOPICS
    assert summary.statistics.message_count == 52 * len(TOPICS)


def test_the_foxglove_layout_only_shows_exported_topics_and_state_values(tmp_path):
    layout = json.loads((mcap_export.REPO_ROOT / "docs" / "foxglove" / "mcap_replay_layout.json").read_text())
    panels = layout["configById"].values()
    output = tmp_path / "straight.mcap"
    mcap_export.main(["-n", "straight_even", "-o", str(output)])
    with output.open("rb") as file:
        state = json.loads(next(make_reader(file).iter_messages(topics=["/policy/state"]))[2].data)

    assert set().union(*(panel.get("topics", {}) for panel in panels)) <= TOPICS
    for path in [path["value"] for panel in panels for path in panel.get("paths", [])]:
        topic, field = path.split(".", 1)
        assert topic == "/policy/state"
        value = state
        for key in field.split("."):
            value = value[key]
