"""Explains what the policy's cone filter keeps and removes in one scenario frame, for the scenario renderer.

The frames hold the cones as received, before the filter, so running policy/cone_filter on them with the settings in
config/ai4r_policy.yaml shows what MovementPolicy.step kept on the car (action_policy mpc).

Note: This module was primarily written by AI with manual review and adjustments.
"""

from __future__ import annotations

import dataclasses
import re
from pathlib import Path

from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones, remove_implausible_cones
from policy.input_output import ConeBatch, ConeDetection

POLICY_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "ai4r_policy.yaml"
"""The policy node's ROS parameter file, which holds the cone_filter settings."""


def load_cone_filter_config(path: Path = POLICY_CONFIG_PATH) -> ConeFilterConfig:
    """Reads the cone_filter settings from the policy node's parameter file.

    The cone_filter block only holds flat "name: value" scalars, so it is read line by line and PyYAML is not
    needed. Settings missing from the file keep their defaults, as in policy_node. An unknown or invalid setting
    raises the same error as when policy_node starts.

    :param path: The ROS parameter file.
    :return: The cone filter settings.
    """
    defaults = ConeFilterConfig()
    types = {field.name: type(getattr(defaults, field.name)) for field in dataclasses.fields(ConeFilterConfig)}
    values = {}
    block_indent = None
    for line in path.read_text(encoding="utf-8").splitlines():
        text = line.split("#", 1)[0].rstrip()
        if not text:
            continue
        indent = len(text) - len(text.lstrip())
        if block_indent is None:
            if text.strip() == "cone_filter:":
                block_indent = indent
            continue
        if indent <= block_indent:
            break
        name, _, value = text.strip().partition(":")
        value = value.strip()
        if types.get(name) is bool:
            if value.lower() not in ("true", "false"):
                raise ValueError(f"cone_filter.{name} must be true or false, not {value}")
            values[name] = value.lower() == "true"
        elif types.get(name) is int and re.fullmatch(r"[+-]?\d+", value):
            values[name] = int(value)
        else:
            values[name] = float(value)
    if block_indent is None:
        raise ValueError(f"No cone_filter settings in {path}")
    return ConeFilterConfig(**values)


def explain_cone_filter(cones: ConeBatch | None, config: ConeFilterConfig) -> list[str | None] | None:
    """Runs the cone filter as MovementPolicy.step does and finds why each removed cone was removed.

    :param cones: The cones of one frame as received, or None when the frame has no cone batch.
    :param config: The cone filter settings.
    :return: For each cone in received order, None when it is kept, otherwise why it was removed; None when cones is
        None.
    """
    if cones is None:
        return None
    # The filter returns the received cone objects, so identity tells which ones it kept
    kept = {id(cone) for cone in filter_cones(cones, config)}
    plausible = {id(cone) for cone in remove_implausible_cones(cones, config)}
    return [None if id(cone) in kept else _removal_reason(cone, config, id(cone) in plausible) for cone in cones]


def format_cone_filter_report(cones: ConeBatch | None, reasons: list[str | None] | None,
                              config: ConeFilterConfig) -> str:
    """
    :param cones: The cones of one frame as received, or None.
    :param reasons: The result of explain_cone_filter for those cones.
    :param config: The cone filter settings used.
    :return: A summary line, then one line per cone in received order.
    """
    settings = (f"max_forward_m {config.max_forward_m}, max_lateral_m {config.max_lateral_m}, "
                f"check_straight_rows {str(config.check_straight_rows).lower()}")
    if cones is None or reasons is None:
        return f"cone filter: no cone batch in this frame ({settings})"
    kept_count = sum(reason is None for reason in reasons)
    lines = [f"cone filter: kept {kept_count} of {len(cones)} cones ({settings})"]
    for index, (cone, reason) in enumerate(zip(cones, reasons)):
        status = "kept   " if reason is None else "REMOVED"
        line = (f"  {index:2d} {status} {cone.colour.name:<6} x {cone.pos.x:5.2f}  y {cone.pos.y:+5.2f}  "
                f"z {cone.pos.z:+5.2f}  confidence {cone.confidence:.2f}")
        lines.append(line if reason is None else f"{line}  {reason}")
    return "\n".join(lines)


def _removal_reason(cone: ConeDetection, config: ConeFilterConfig, plausible: bool) -> str:
    """
    :param cone: A cone the filter removed.
    :param config: The cone filter settings.
    :param plausible: Whether the cone passed steps 1 and 2 (space, confidence and duplicates).
    :return: Why the cone was removed.
    """
    if plausible:
        return f"off its colour's straight row (row_residual_m {config.row_residual_m})"
    pos = cone.pos
    reasons = []
    if pos.x <= 0.0:
        reasons.append("behind the car (x <= 0)")
    if pos.x > config.max_forward_m:
        reasons.append(f"x above max_forward_m {config.max_forward_m}")
    if abs(pos.y) > config.max_lateral_m:
        reasons.append(f"|y| above max_lateral_m {config.max_lateral_m}")
    if not config.min_height_m <= pos.z <= config.max_height_m:
        reasons.append(f"z outside [{config.min_height_m}, {config.max_height_m}]")
    if cone.confidence < config.min_confidence:
        reasons.append(f"confidence below min_confidence {config.min_confidence}")
    if not reasons:
        reasons.append(f"duplicate of a more confident {cone.colour.name} cone within merge_distance_m "
                       f"{config.merge_distance_m}")
    return ", ".join(reasons)
