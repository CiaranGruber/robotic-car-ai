"""Replay a recorded drive through the policy and write everything it saw and decided to an MCAP file for Foxglove.

Example::

    python -m scripts.mcap_export -n fail_two_obstacles_curve
    python -m scripts.mcap_export -t synthetic -n synthetic_s0 -o s0.mcap
    python -m scripts.mcap_export -p logs/2026-10-19_10-02-11_observations.json

The input is a scenario (-n with -t, or -p to a *_scenario.json file) or a drive data log (-p to its
*_observations.json file; the *_actions.json and *_config.json beside it are read when they exist). Each recorded
step is run through a new MovementPolicy, so the drawn lane, obstacles and plans are the policy's own results. The
settings come from --settings, else the data log's *_config.json, else config/ai4r_policy.yaml.

Open the file in Foxglove (drag it into the window, or File > Open local file). Everything is in base_link, the
car's body frame (+x forwards, +y left), at the time the policy step ran. Import docs/foxglove/mcap_replay_layout.json
once as a layout (the layout menu > Import from file) for a top-down 2D map around the car beside plots of the actions,
lane and speed. Without it, set a 3D panel's display frame to base_link and its camera to 2D. Topics:

- /cones/detected, /cones/used, /lane, /obstacles, /plan/avoidance, /plan/mpc, /fiducials and /car: the layers of
  policy/visualisation.py, which policy_node also publishes live as policy_visualisation/markers.
- /lidar/points: the Cartesian lidar returns.
- /policy/state (JSON): numbers for the Plot panel, such as replayed.drive_action, logged.drive_action, lane.*,
  obstacle_distance_m and mpc.*. actions_match is false when the replay chose other actions than the recording.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import math
import struct
import sys
from pathlib import Path
from typing import Any

import foxglove
import yaml
from foxglove.channels import PointCloudChannel, SceneUpdateChannel
from foxglove.messages import (Color, CubePrimitive, LinePrimitive, LinePrimitiveLineType, PackedElementField,
                               PackedElementFieldNumericType, Point3, PointCloud, Pose, Quaternion, SceneEntity,
                               SceneEntityDeletion, SceneEntityDeletionType, SceneUpdate, SpherePrimitive,
                               TextPrimitive, Timestamp, Vector3)

from policy import visualisation
from policy.action_policy.avoidance import AvoidanceConfig
from policy.action_policy.mpc import MpcConfig
from policy.action_policy.open_space import OpenSpaceConfig
from policy.cone_filter.cone_filter import ConeFilterConfig
from policy.control.control import ControlConfig
from policy.data_logging.data_logging import ACTIONS_LOG_SUFFIX, CONFIG_LOG_SUFFIX, OBSERVATIONS_LOG_SUFFIX
from policy.input_output import CarActions, CarObservations
from policy.lane_detection.lane_detection import LaneDetectionConfig
from policy.local_map.local_map import LocalMapConfig
from policy.markers.markers import MarkerConfig
from policy.action_policy.intersection import IntersectionConfig
from policy.memory import CarMemory
from policy.obstacle_detection.obstacle_detection import ObstacleDetectionConfig
from policy.policy_runner import MovementPolicy
from scenarios import SCENARIO_SUFFIX
from scenarios.scenario_wrapper import Scenario, ScenarioType

_LOGGER = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[1]
"""Repository root. The script lives in scripts/."""

DEFAULT_SETTINGS_PATH = REPO_ROOT / "config" / "ai4r_policy.yaml"
"""Settings used when neither --settings nor a data log's *_config.json gives them."""

FRAME_ID = "base_link"
"""Frame of every drawing: the car's body frame."""

CONFIG_TYPES = {
    "mpc": MpcConfig, "open_space": OpenSpaceConfig, "cone_filter": ConeFilterConfig, "control": ControlConfig,
    "lane_detection": LaneDetectionConfig, "obstacle_detection": ObstacleDetectionConfig,
    "avoidance": AvoidanceConfig, "local_map": LocalMapConfig, "markers": MarkerConfig,
    "intersection": IntersectionConfig,
}
"""The MovementPolicy settings, by their prefix in config/ai4r_policy.yaml, as policy_node reads them."""

ACTION_TOLERANCE = 1e-6
"""Largest difference between a replayed and a recorded action that still counts as the same action."""


class McapExportError(Exception):
    """The export could not complete."""


@dataclasses.dataclass(frozen=True)
class RecordedStep:
    """One recorded policy step."""
    observations: CarObservations
    """The observations the policy was given."""
    actions: CarActions | None
    """The actions the car published, or None when the recording has none for this step."""


def read_settings(path: Path) -> dict[str, Any]:
    """Reads policy settings by their full parameter name, such as mpc.target_speed_m_per_s.

    :param path: A data log's *_config.json, which is already by full name, or a ROS parameter YAML file such as
        config/ai4r_policy.yaml, whose first node's ros__parameters are used.
    :return: The settings by full parameter name.
    """
    text = path.read_text(encoding="utf-8")
    if path.suffix == ".json":
        return json.loads(text)
    document = yaml.safe_load(text)
    for node in document.values():
        if isinstance(node, dict) and "ros__parameters" in node:
            return _flatten(node["ros__parameters"])
    raise McapExportError(f"{path} has no ros__parameters")


def _flatten(values: dict[str, Any], prefix: str = "") -> dict[str, Any]:
    """
    :return: The nested settings by full dotted name.
    """
    flat = {}
    for name, value in values.items():
        if isinstance(value, dict):
            flat.update(_flatten(value, f"{prefix}{name}."))
        else:
            flat[f"{prefix}{name}"] = value
    return flat


def build_policy(settings: dict[str, Any]) -> MovementPolicy:
    """Creates the MovementPolicy policy_node would run with these settings.

    A setting that is missing, for example from a log made before it existed, keeps its default.

    :param settings: Settings by full parameter name (see read_settings).
    :return: A new movement policy.
    """
    configs, missing = {}, []
    for prefix, config_type in CONFIG_TYPES.items():
        values = {}
        for field in dataclasses.fields(config_type):
            name = f"{prefix}.{field.name}"
            if name in settings:
                values[field.name] = settings[name]
            else:
                missing.append(name)
        configs[prefix] = config_type(**values)
    if missing:
        _LOGGER.info("Using the defaults for the settings not given: %s", ", ".join(missing))
    return MovementPolicy(
        configs["mpc"], configs["cone_filter"], configs["control"], configs["open_space"],
        settings.get("action_policy", "mpc"), lane_detection_config=configs["lane_detection"],
        obstacle_detection_config=configs["obstacle_detection"], avoidance_config=configs["avoidance"],
        local_map_config=configs["local_map"], marker_config=configs["markers"],
        intersection_config=configs["intersection"])


def read_data_log(observations_path: Path) -> list[RecordedStep]:
    """Reads a drive data log.

    :param observations_path: The log's *_observations.json file. The *_actions.json beside it is read when it exists.
    :return: The recorded steps, as many as there are observations.
    """
    observations = json.loads(observations_path.read_text(encoding="utf-8"))
    actions_path = _sibling(observations_path, ACTIONS_LOG_SUFFIX)
    actions = json.loads(actions_path.read_text(encoding="utf-8")) if actions_path.is_file() else []
    if len(actions) != len(observations):
        _LOGGER.warning("%d observations but %d actions: the replayed actions are only compared for the first %d",
                        len(observations), len(actions), min(len(actions), len(observations)))
    return [RecordedStep(CarObservations.deserialise(entry),
                         CarActions.deserialise(actions[index]) if index < len(actions) else None)
            for index, entry in enumerate(observations)]


def _sibling(observations_path: Path, suffix: str) -> Path:
    """
    :return: The file of the same data log with the given suffix, such as _actions.json.
    """
    return observations_path.with_name(observations_path.name.removesuffix(OBSERVATIONS_LOG_SUFFIX) + suffix)


def _start_time_ns(steps: list[RecordedStep]) -> int:
    """
    :return: The ROS time of the first step in nanoseconds, from a sensor stamp and its age, or 0 when no sample in
        the recording has a stamp.
    """
    for step in steps:
        observations = step.observations
        for sample in (observations.cones, observations.lidar_cartesian, observations.lidar_scan,
                       observations.fiducials, observations.car.orientation):
            age = getattr(sample, "sensor_age", None)
            if age is not None and age.stamp_ns is not None:
                return age.stamp_ns + round((float(age) - observations.policy.policy_elapsed_s) * 1e9)
    return 0


class McapExporter:
    """Writes one replayed drive to an MCAP file, one message per topic per policy step."""

    def __init__(self, writer_context: foxglove.Context):
        """
        :param writer_context: The context of the open MCAP writer; the channels log only to it.
        """
        self._scenes = {layer: SceneUpdateChannel(f"/{layer}", context=writer_context)
                        for layer in visualisation.LAYERS}
        self._lidar = PointCloudChannel("/lidar/points", context=writer_context)
        self._state = foxglove.Channel("/policy/state", message_encoding="json", context=writer_context,
                                       schema=foxglove.Schema(name="ai4r_policy.PolicyStepState", encoding="jsonschema",
                                                              data=json.dumps({"type": "object"}).encode()))

    def write_step(self, layers: dict[str, visualisation.Layer], policy: MovementPolicy, step: RecordedStep,
                   replayed: CarActions, time_ns: int):
        """Writes the drawings and numbers of one replayed step.

        :param layers: The step's drawing.
        :param policy: The movement policy just after it ran the step.
        :param step: The recorded step.
        :param replayed: The actions the policy chose in the replay.
        :param time_ns: Time of the step in nanoseconds.
        """
        stamp = Timestamp(sec=time_ns // 1_000_000_000, nsec=time_ns % 1_000_000_000)
        for name, layer in layers.items():
            self._scenes[name].log(SceneUpdate(
                deletions=[SceneEntityDeletion(timestamp=stamp, type=SceneEntityDeletionType.All)],
                entities=[_entity(name, layer, stamp)]), log_time=time_ns)
        lidar = step.observations.lidar_cartesian
        points = [] if lidar is None else [point.pos for point in lidar.points]
        self._lidar.log(PointCloud(
            timestamp=stamp, frame_id=FRAME_ID if lidar is None else lidar.frame_id, pose=_pose((0.0, 0.0, 0.0)),
            point_stride=12, fields=[PackedElementField(name=axis, offset=4 * index,
                                                        type=PackedElementFieldNumericType.Float32)
                                     for index, axis in enumerate("xyz")],
            data=b"".join(struct.pack("<fff", point.x, point.y, point.z) for point in points)), log_time=time_ns)
        self._state.log(_state(policy, step, replayed), log_time=time_ns)


def export(steps: list[RecordedStep], policy: MovementPolicy, output: Path) -> int:
    """Runs the recorded steps through the policy, as policy_node does, and writes the MCAP file.

    :param steps: The recorded steps, in order.
    :param policy: A new movement policy with the drive's settings.
    :param output: The MCAP file to write; an existing file is replaced.
    :return: How many steps the replay chose other actions in than the recording.
    """
    start_ns = _start_time_ns(steps)
    memory = CarMemory()
    drawer = visualisation.StepDrawer()
    mismatches = 0
    writer_context = foxglove.Context()
    with foxglove.open_mcap(output, allow_overwrite=True, context=writer_context):
        exporter = McapExporter(writer_context)
        for step in steps:
            replayed = policy.step(step.observations, memory)
            memory.step()
            if step.actions is not None and not _same_actions(replayed, step.actions):
                mismatches += 1
            exporter.write_step(drawer.draw(policy, step.observations, replayed), policy, step, replayed,
                                start_ns + round(step.observations.policy.policy_elapsed_s * 1e9))
    return mismatches


def _same_actions(replayed: CarActions, recorded: CarActions) -> bool:
    """
    :return: True when the replay chose the recorded drive and steering actions.
    """
    return (abs(replayed.drive_action - recorded.drive_action) <= ACTION_TOLERANCE
            and abs(replayed.steering_action - recorded.steering_action) <= ACTION_TOLERANCE)


def _colour(colour: visualisation.Colour) -> Color:
    """
    :return: The colour as a Foxglove colour.
    """
    return Color(r=colour[0], g=colour[1], b=colour[2], a=colour[3])


def _pose(position: tuple[float, float, float],
          orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)) -> Pose:
    """
    :return: A pose at the position with the (x, y, z, w) rotation.
    """
    return Pose(position=Vector3(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]))


def _entity(name: str, layer: visualisation.Layer, stamp: Timestamp) -> SceneEntity:
    """
    :return: The layer as one Foxglove scene entity in base_link.
    """
    return SceneEntity(
        timestamp=stamp, frame_id=FRAME_ID, id=name,
        lines=[LinePrimitive(type=LinePrimitiveLineType.LineLoop if line.closed else LinePrimitiveLineType.LineStrip,
                             pose=_pose((0.0, 0.0, 0.0)), thickness=visualisation.LINE_WIDTH_M,
                             color=_colour(line.colour), points=[Point3(x=x, y=y, z=0.0) for x, y in line.points])
               for line in layer.lines],
        spheres=[SpherePrimitive(pose=_pose((sphere.x, sphere.y, 0.0)), color=_colour(sphere.colour),
                                 size=Vector3(x=sphere.diameter_m, y=sphere.diameter_m, z=sphere.diameter_m))
                 for sphere in layer.spheres],
        cubes=[CubePrimitive(pose=_pose(box.position, box.orientation), color=_colour(box.colour),
                             size=Vector3(x=box.size[0], y=box.size[1], z=box.size[2]))
               for box in layer.boxes],
        texts=[TextPrimitive(pose=_pose(text.position), billboard=True, font_size=visualisation.TEXT_HEIGHT_M,
                             scale_invariant=False, color=_colour(visualisation.COLOURS["text"]), text=text.text)
               for text in layer.texts])


def _state(policy: MovementPolicy, step: RecordedStep, replayed: CarActions) -> dict[str, Any]:
    """
    :return: The step's numbers for the Plot panel. Values that are unknown or infinite are None.
    """
    lanes, mpc, observations = policy.lanes, policy.action_policy, step.observations
    speed = observations.car.wheel_speed
    state = {
        "policy_elapsed_s": observations.policy.policy_elapsed_s,
        "dt": observations.policy.dt,
        "wheel_speed_m_per_s": None if speed is None else float(speed),
        "replayed": {"drive_action": replayed.drive_action, "steering_action": replayed.steering_action},
        "logged": None if step.actions is None else {"drive_action": step.actions.drive_action,
                                                     "steering_action": step.actions.steering_action},
        "actions_match": None if step.actions is None else _same_actions(replayed, step.actions),
        "cones": {"detected": len(observations.cones or []), "used": len(policy.cones or [])},
        "lane": None if lanes is None else {
            "status": lanes.status.name, "lateral_error_m": lanes.lateral_error_m,
            "heading_error_deg": None if lanes.heading_error_rad is None else math.degrees(lanes.heading_error_rad),
            "lane_width_m": lanes.lane_width_m, "path_curvature_per_m": lanes.path_curvature_per_m,
            "visible_until_m": lanes.visible_until_m},
        "obstacle_distance_m": policy.obstacle_distance_m,
        "lane_obstacle_count": None if policy.lane_obstacles is None else len(policy.lane_obstacles),
        "local_map": {"remembered_cones": len(policy.local_map.map_cones),
                      "recalled_cones": len(policy.local_map.recalled),
                      "remembered_obstacles": len(policy.avoidance.remembered),
                      "pose": None if policy.local_map.pose is None else policy.local_map.pose.serialise()},
        "mpc": {"solve_count": mpc.solve_count, "solve_time_s": mpc.last_solve_time_s,
                "stopped_for_obstacle": mpc.stopped_for_obstacle, "stopped_for_lane_loss": mpc.stopped_for_lane_loss,
                "stopped_for_solver_failure": mpc.stopped_for_solver_failure},
    }
    return _finite(state)


def _finite(value: Any) -> Any:
    """
    :return: The value with every infinite or NaN number replaced by None, which JSON can hold.
    """
    if isinstance(value, dict):
        return {name: _finite(item) for name, item in value.items()}
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def recorded_steps(name: str | None, path: Path | None, scenario_type: ScenarioType) -> tuple[list[RecordedStep],
                                                                                               Path | None]:
    """Reads a scenario or a drive data log.

    :param name: Scenario stem without _scenario.json, or None.
    :param path: Path to a *_scenario.json or *_observations.json file, or None.
    :param scenario_type: Scenario folder used when resolving by name.
    :return: The recorded steps, and the data log's *_config.json when it has one, else None.
    """
    scenario_path = Scenario.from_name(scenario_type, name).scenario_path if name is not None else path
    if not scenario_path.is_file():
        raise McapExportError(f"File does not exist: {scenario_path}")
    if scenario_path.name.endswith(OBSERVATIONS_LOG_SUFFIX):
        config_path = _sibling(scenario_path, CONFIG_LOG_SUFFIX)
        return read_data_log(scenario_path), config_path if config_path.is_file() else None
    scenario = Scenario(scenario_path)
    return [RecordedStep(observations, actions) for observations, actions in scenario.steps()], None


def _default_output(name: str | None, path: Path | None) -> Path:
    """
    :return: <stem>.mcap in the current folder, named after the scenario or data log.
    """
    stem = name if name is not None else path.name.removesuffix(SCENARIO_SUFFIX).removesuffix(OBSERVATIONS_LOG_SUFFIX)
    return Path(f"{stem}.mcap")


def build_parser() -> argparse.ArgumentParser:
    """
    :return: The argument parser.
    """
    parser = argparse.ArgumentParser(prog="mcap_export", description=__doc__.split("\n")[0])
    parser.add_argument("-t", "--type", choices=[member.name.lower() for member in ScenarioType],
                        default=ScenarioType.REAL.name.lower(), help="Scenario type for -n. Default: real.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("-n", "--name", help="Scenario stem under the selected type folder, such as straight_even.")
    source.add_argument("-p", "--path", type=Path,
                        help="Path to a *_scenario.json file or a data log's *_observations.json file.")
    parser.add_argument("-s", "--settings", type=Path,
                        help="Policy settings: a ROS parameter YAML file or a data log's *_config.json. Default: the "
                             "data log's *_config.json, else config/ai4r_policy.yaml.")
    parser.add_argument("-o", "--output", type=Path, help="MCAP file to write. Default: <stem>.mcap here.")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Exports one scenario or data log.

    :param argv: Arguments, excluding the program name. Defaults to sys.argv.
    :return: Zero on success, or one when the export fails.
    """
    if not logging.getLogger().handlers:
        logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = build_parser().parse_args(argv)
    try:
        steps, log_config = recorded_steps(args.name, args.path, ScenarioType[args.type.upper()])
        settings_path = args.settings or log_config or DEFAULT_SETTINGS_PATH
        output = args.output or _default_output(args.name, args.path)
        mismatches = export(steps, build_policy(read_settings(settings_path)), output)
    except McapExportError as exc:
        _LOGGER.error("%s", exc)
        return 1
    _LOGGER.info("Wrote %d steps to %s with the settings in %s", len(steps), output, settings_path)
    if mismatches:
        _LOGGER.warning("The replay chose other actions than the recording in %d of %d steps; the drawings show the "
                        "replay (see /policy/state actions_match)", mismatches, len(steps))
    return 0


if __name__ == "__main__":
    sys.exit(main())
