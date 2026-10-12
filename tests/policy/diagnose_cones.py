"""Inspect recorded cone batches without ROS or driving hardware.

Run: python -m tests.policy.diagnose_cones path/to/scenario.json
Uses the default filter and MPC lane width; reports perception inputs separately
from geometric lane fits. This cannot diagnose camera images or depth calibration.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

from policy.action_policy.mpc import MpcConfig
from policy.action_policy.reference import reference_from_cones, rotate_cones
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones, remove_implausible_cones
from scenarios.scenario_wrapper import Scenario


def inspect_batch(cones, config=ConeFilterConfig(), mpc=MpcConfig()):
    """Report what each filtering stage receives and retains for one batch."""
    plausible = remove_implausible_cones(cones, config)
    kept = filter_cones(cones, config)
    state = reference_from_cones(None if kept is None else rotate_cones(kept, math.radians(mpc.camera_yaw_offset_deg)),
                                 mpc.lane_width_m)
    return {
        "available": cones is not None,
        "raw": 0 if cones is None else len(cones),
        "plausible_unique": 0 if plausible is None else len(plausible),
        "kept": 0 if kept is None else len(kept),
        "kept_colours": {} if kept is None else dict(Counter(c.colour.name for c in kept)),
        "beyond_forward_limit": 0 if cones is None else sum(c.pos.x > config.max_forward_m for c in cones),
        "usable_lane": state is not None,
        "age_s": None if cones is None else float(cones.sensor_age),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenarios", type=Path, nargs="+")
    args = parser.parse_args()
    for path in args.scenarios:
        batches = [inspect_batch(obs.cones) for obs, _ in Scenario(path).steps()]
        print(json.dumps({"scenario": str(path), "batches": len(batches),
                          "usable_lane_batches": sum(b["usable_lane"] for b in batches),
                          "missing_batches": sum(not b["available"] for b in batches),
                          "empty_batches": sum(b["available"] and b["raw"] == 0 for b in batches),
                          "nonempty_without_lane": sum(b["raw"] > 0 and not b["usable_lane"] for b in batches),
                          "first_unusable": next((b for b in batches if not b["usable_lane"]), None)}, indent=2))


if __name__ == "__main__":
    main()
