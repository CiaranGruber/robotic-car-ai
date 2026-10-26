"""Replay recorded drives through the policy and report how well each one kept off the cones, dealt with obstacles and
drove on, so a change can be measured on the same drives before it goes on the car.

Example::

    python -m scripts.run_report logs/2026-10-22_16-58-39_observations.json --lane-width 0.8
    python -m scripts.run_report logs/                       # every data log and scenario in the folder
    python -m scripts.run_report -t synthetic -n synthetic_s0 --json

Each input is a drive data log (*_observations.json), a scenario (*_scenario.json, or -n with -t) or a folder of them.
It is replayed as scripts/mcap_export.py replays it, with the same settings: --settings, else the data log's
*_config.json, else config/ai4r_policy.yaml. The numbers are the replay's own, so a changed policy or setting is
measured on the recorded observations; replay_mismatches counts the steps where the replay chose other actions than
the recording (0 when it repeats the drive).

For each drive it reports:
- steps, duration_s, step_interval_ms (the recorded time between policy steps) and replay_calculation_ms (the
  policy's calculation time on this computer, not the car's).
- driving_steps and speeds. The lane and clearance numbers only count driving steps: the car moved (wheel speed above
  MOVING_M_PER_S) and the MPC had not stopped it for the rest of the run. The wheel speed lags by seconds near
  standstill, so without the second condition a stopped car would still count.
- Keeping off the cones: lane_status (share of steps per lane detection status), the lane estimate while driving
  (lateral and heading error, the share of headings over HEADING_LIMIT_DEG, heading jumps between steps over
  HEADING_JUMP_DEG, the estimated width and, with --lane-width, its error) and side_clearance_m: the gap from the
  car's sides to the nearest lidar return beside its body. Returns inside the body are the car itself and are left out;
  returns just outside it can still be the car's own wheels while avoidance.car_width_m is narrower than the car.
- Obstacles: whether lane obstacles were seen, and whether the car passed them or the MPC stopped for one (with the
  obstacle distance at the stop).
- Driving on: the lateral and heading error of the driving steps after the last lane obstacle had gone from view.
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Any

from policy.data_logging.data_logging import OBSERVATIONS_LOG_SUFFIX
from policy.memory import CarMemory
from policy.policy_runner import MovementPolicy
from scenarios import SCENARIO_SUFFIX
from scenarios.scenario_wrapper import ScenarioType
from scripts.mcap_export import (DEFAULT_SETTINGS_PATH, McapExportError, RecordedStep, build_policy, read_settings,
                                 recorded_steps, step_state)

_LOGGER = logging.getLogger(__name__)

MOVING_M_PER_S = 0.05
"""Wheel speed in m/s above which the car counts as moving."""

HEADING_LIMIT_DEG = 30.0
"""Heading error in degrees beyond which a lane estimate is implausible for a car driving along the lane."""

HEADING_JUMP_DEG = 15.0
"""Change of the heading error in degrees between consecutive driving steps that the car itself cannot turn in one
step, so it is a jump of the lane estimate."""

NEAR_CONE_M = 0.05
"""Side clearance in metres below which a step counts as nearly touching."""

STOP_REASONS = {"stopped_for_obstacle": "obstacle", "stopped_for_lane_loss": "lane ended or lost",
                "stopped_for_solver_failure": "solver failure"}
"""The MPC's stops for the rest of the run (step_state's mpc fields), and how the report names them."""


def _spread_ms(values: list[float]) -> dict[str, float] | None:
    """
    :param values: Durations in seconds.
    :return: Their median, 95th percentile (nearest rank) and maximum in milliseconds, or None without values.
    """
    if not values:
        return None
    ordered = sorted(values)
    p95 = ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]
    return {"median": 1000 * statistics.median(ordered), "p95": 1000 * p95, "max": 1000 * ordered[-1]}


def _side_clearance_m(step: RecordedStep, policy: MovementPolicy) -> float | None:
    """
    :param step: The recorded step.
    :param policy: The movement policy, for the car's body (car_front_m, car_rear_m and car_width_m).
    :return: The gap in metres from the car's sides to the nearest lidar return beside its body, or None without
        fresh lidar points or without a return beside the body.
    """
    lidar = step.observations.lidar_cartesian
    if lidar is None:
        return None
    front_m = policy.obstacle_detection_config.car_front_m
    rear_m, half_width_m = policy.avoidance.config.car_rear_m, policy.avoidance.config.car_width_m / 2.0
    gaps = [abs(point.pos.y) - half_width_m for point in lidar.points
            if -rear_m <= point.pos.x <= front_m and abs(point.pos.y) > half_width_m]
    return min(gaps, default=None)


def _lane_numbers(states: list[dict[str, Any]], lane_width_m: float | None) -> dict[str, Any] | None:
    """
    :param states: step_state of the driving steps, in order.
    :param lane_width_m: The real lane width in metres, or None when unknown.
    :return: The lane estimate's errors over the steps with a lane, or None when none had one.
    """
    lanes = [state["lane"] for state in states
             if state["lane"] is not None and state["lane"]["lateral_error_m"] is not None]
    if not lanes:
        return None
    lateral = [lane["lateral_error_m"] for lane in lanes]
    heading = [lane["heading_error_deg"] for lane in lanes if lane["heading_error_deg"] is not None]
    widths = [lane["lane_width_m"] for lane in lanes if lane["lane_width_m"] is not None]
    jumps = [abs(b - a) > HEADING_JUMP_DEG for a, b in zip(heading, heading[1:])]
    numbers = {
        "steps": len(lanes),
        "abs_lateral_median_m": statistics.median(abs(value) for value in lateral),
        "lateral_rms_m": math.sqrt(statistics.fmean(value * value for value in lateral)),
        "abs_lateral_max_m": max(abs(value) for value in lateral),
        "abs_heading_median_deg": statistics.median(abs(value) for value in heading) if heading else None,
        "heading_over_limit_share": (sum(abs(value) > HEADING_LIMIT_DEG for value in heading) / len(heading)
                                     if heading else None),
        "heading_jump_share": sum(jumps) / len(jumps) if jumps else None,
        "width_median_m": statistics.median(widths) if widths else None,
    }
    if lane_width_m is not None and widths:
        numbers["width_error_median_m"] = statistics.median(abs(width - lane_width_m) for width in widths)
    return numbers


def report(name: str, steps: list[RecordedStep], policy: MovementPolicy,
           lane_width_m: float | None = None) -> dict[str, Any]:
    """Replays one drive and measures it.

    :param name: The drive's name for the report.
    :param steps: The recorded steps, in order.
    :param policy: A new movement policy with the drive's settings.
    :param lane_width_m: The real lane width in metres, or None when unknown.
    :return: The drive's numbers (see the file's description), JSON-friendly.
    """
    memory = CarMemory()
    states, calculations, clearances = [], [], []
    for step in steps:
        started = time.perf_counter()
        replayed = policy.step(step.observations, memory)
        calculations.append(time.perf_counter() - started)
        memory.step()
        state = step_state(policy, step, replayed)
        # The wheel speed lags by seconds near standstill, so a car the MPC has stopped can still read as moving
        driving = ((state["wheel_speed_m_per_s"] or 0.0) > MOVING_M_PER_S
                   and not any(state["mpc"][key] for key in STOP_REASONS))
        if driving:
            clearance = _side_clearance_m(step, policy)
            if clearance is not None:
                clearances.append(clearance)
        states.append({**state, "driving": driving})
    driving_states = [state for state in states if state["driving"]]
    speeds = [state["wheel_speed_m_per_s"] for state in driving_states]
    statuses = [state["lane"]["status"] if state["lane"] is not None else "NONE" for state in states]
    stop = next(((STOP_REASONS[key], state) for state in states for key in STOP_REASONS if state["mpc"][key]), None)
    seen = [index for index, state in enumerate(states) if state["lane_obstacle_count"]]
    passed_from = seen[-1] + 1 if seen and (stop is None or stop[0] != "obstacle") else None
    return {
        "name": name,
        "steps": len(states),
        "duration_s": states[-1]["policy_elapsed_s"] if states else 0.0,
        "replay_mismatches": (sum(state["actions_match"] is False for state in states)
                              if any(state["actions_match"] is not None for state in states) else None),
        "step_interval_ms": _spread_ms([state["dt"] for state in states[1:] if state["dt"]]),
        "replay_calculation_ms": _spread_ms(calculations),
        "driving_steps": len(driving_states),
        "max_speed_m_per_s": max(speeds, default=0.0),
        "mean_driving_speed_m_per_s": statistics.fmean(speeds) if speeds else 0.0,
        "lane_status": {status: statuses.count(status) / len(statuses) for status in sorted(set(statuses))},
        "lane_while_driving": _lane_numbers(driving_states, lane_width_m),
        "side_clearance_m": None if not clearances else {
            "min": min(clearances), "median": statistics.median(clearances),
            "near_share": sum(clearance < NEAR_CONE_M for clearance in clearances) / len(clearances)},
        "obstacles_seen": bool(seen),
        "obstacles_passed": passed_from is not None,
        "latched_stop": None if stop is None else {
            "reason": stop[0], "at_s": stop[1]["policy_elapsed_s"],
            "obstacle_distance_m": stop[1]["obstacle_distance_m"] if stop[0] == "obstacle" else None},
        "after_obstacles": (None if passed_from is None else
                            _lane_numbers([state for state in states[passed_from:] if state["driving"]], lane_width_m)),
    }


def inputs(paths: list[Path], names: list[str]) -> list[tuple[str | None, Path | None]]:
    """
    :param paths: Data logs, scenarios or folders of them.
    :param names: Scenario stems, resolved by recorded_steps under the -t folder.
    :return: (name, path) pairs for recorded_steps, one per drive, in order.
    :raises McapExportError: When a path is neither a data log, a scenario nor a folder holding one.
    """
    found: list[tuple[str | None, Path | None]] = [(name, None) for name in names]
    for path in paths:
        if path.is_dir():
            drives = sorted([*path.glob(f"*{OBSERVATIONS_LOG_SUFFIX}"), *path.glob(f"*{SCENARIO_SUFFIX}")])
            if not drives:
                raise McapExportError(f"No *{OBSERVATIONS_LOG_SUFFIX} or *{SCENARIO_SUFFIX} files in {path}")
            found += [(None, drive) for drive in drives]
        elif path.name.endswith((OBSERVATIONS_LOG_SUFFIX, SCENARIO_SUFFIX)):
            found.append((None, path))
        else:
            raise McapExportError(f"Not a data log (*{OBSERVATIONS_LOG_SUFFIX}) or scenario (*{SCENARIO_SUFFIX}): {path}")
    return found


def _format(value: Any, unit: str = "", scale: float = 1.0, digits: int = 2) -> str:
    """
    :return: The value scaled and rounded with its unit, or "-" when it is None.
    """
    return "-" if value is None else f"{value * scale:.{digits}f}{unit}"


def text(result: dict[str, Any]) -> str:
    """
    :param result: One drive's numbers from report.
    :return: The numbers as a few readable lines.
    """
    def spread(numbers: dict[str, float] | None) -> str:
        return "-" if numbers is None else (f"median {numbers['median']:.1f} ms, p95 {numbers['p95']:.1f} ms, "
                                            f"max {numbers['max']:.1f} ms")

    def lane(numbers: dict[str, Any] | None) -> str:
        if numbers is None:
            return "no lane"
        width_error = numbers.get("width_error_median_m")
        return (f"|lateral| median {_format(numbers['abs_lateral_median_m'], ' m')}, RMS "
                f"{_format(numbers['lateral_rms_m'], ' m')}, max {_format(numbers['abs_lateral_max_m'], ' m')}; "
                f"|heading| median {_format(numbers['abs_heading_median_deg'], ' deg', digits=1)}, "
                f"over {HEADING_LIMIT_DEG:.0f} deg {_format(numbers['heading_over_limit_share'], '%', 100, 0)}, "
                f"jumps {_format(numbers['heading_jump_share'], '%', 100, 0)}; width median "
                f"{_format(numbers['width_median_m'], ' m')}"
                + ("" if width_error is None else f" (error {_format(width_error, ' m')})"))

    clearance = result["side_clearance_m"]
    stop = result["latched_stop"]
    lines = [
        f"== {result['name']}: {result['steps']} steps, {result['duration_s']:.1f} s, driving {result['driving_steps']} "
        f"steps at {result['mean_driving_speed_m_per_s']:.2f} m/s (max {result['max_speed_m_per_s']:.2f}); replay "
        f"mismatches {'-' if result['replay_mismatches'] is None else result['replay_mismatches']}",
        f"   timing: step interval {spread(result['step_interval_ms'])}; "
        f"replay calculation {spread(result['replay_calculation_ms'])}",
        "   lane status: " + ", ".join(f"{status} {share:.0%}" for status, share in result["lane_status"].items()),
        f"   cones: {lane(result['lane_while_driving'])}",
        "   side clearance: " + ("no lidar returns beside the car" if clearance is None else
                                 f"min {_format(clearance['min'], ' m')}, median {_format(clearance['median'], ' m')}, "
                                 f"under {NEAR_CONE_M * 100:.0f} cm {_format(clearance['near_share'], '%', 100, 0)}"),
        "   obstacles: " + ("none seen" if not result["obstacles_seen"] else
                            "passed" if result["obstacles_passed"] else "not passed"),
        "   stopped: " + ("no" if stop is None else
                          f"{stop['reason']} at {stop['at_s']:.1f} s"
                          + ("" if stop["obstacle_distance_m"] is None
                             else f", {stop['obstacle_distance_m']:.2f} m before it")),
    ]
    if result["after_obstacles"] is not None:
        lines.append(f"   after the obstacles: {lane(result['after_obstacles'])}")
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    """
    :return: The argument parser.
    """
    parser = argparse.ArgumentParser(prog="run_report", description=__doc__.split("\n")[0])
    parser.add_argument("paths", nargs="*", type=Path,
                        help="Data logs (*_observations.json), scenarios (*_scenario.json) or folders of them.")
    parser.add_argument("-n", "--name", action="append", default=[],
                        help="Scenario stem under the -t folder, such as straight_even. May be repeated.")
    parser.add_argument("-t", "--type", choices=[member.name.lower() for member in ScenarioType],
                        default=ScenarioType.REAL.name.lower(), help="Scenario type for -n. Default: real.")
    parser.add_argument("-s", "--settings", type=Path,
                        help="Policy settings for every drive: a ROS parameter YAML file or a data log's *_config.json. "
                             "Default: each data log's *_config.json, else config/ai4r_policy.yaml.")
    parser.add_argument("--lane-width", type=float, help="The real lane width in metres, to report the width error.")
    parser.add_argument("--json", action="store_true", help="Print the numbers as JSON instead of text.")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Reports each drive.

    :param argv: Arguments, excluding the program name. Defaults to sys.argv.
    :return: Zero on success, or one when a drive cannot be read.
    """
    if not logging.getLogger().handlers:
        logging.basicConfig(level=logging.WARNING, format="%(levelname)s: %(message)s")
    args = build_parser().parse_args(argv)
    if not args.paths and not args.name:
        build_parser().error("give a data log, scenario or folder, or -n")
    results = []
    try:
        for name, path in inputs(args.paths, args.name):
            steps, log_config = recorded_steps(name, path, ScenarioType[args.type.upper()])
            policy = build_policy(read_settings(args.settings or log_config or DEFAULT_SETTINGS_PATH))
            label = name if name is not None else path.name.removesuffix(OBSERVATIONS_LOG_SUFFIX).removesuffix(
                SCENARIO_SUFFIX)
            # The policy's own step messages are not part of the report
            logging.disable(logging.INFO)
            try:
                results.append(report(label, steps, policy, args.lane_width))
            finally:
                logging.disable(logging.NOTSET)
    except McapExportError as exc:
        _LOGGER.error("%s", exc)
        return 1
    print(json.dumps(results, indent=2) if args.json else "\n".join(text(result) for result in results))
    return 0


if __name__ == "__main__":
    sys.exit(main())
