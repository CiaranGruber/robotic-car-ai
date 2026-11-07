"""Tunes the MPC's horizon and weights on the synthetic lab tracks (Task A4), and checks the result on new tracks.

Every option is run on the training tracks (synthetic_tracks.TRAINING_SEEDS) with the measured settings fixed: car
20's left and right curvature limits, the 0.8 m lab lane width and 0.5 m/s. The rule below chooses one option from
the training results only. The chosen option is then compared with the shipped settings on the validation tracks,
which played no part in choosing it. The unseen cases are never used.

These tracks are all straight. The shipped weights were chosen on curved lanes as well (curved_lanes.py), so this
sweep's choice is a reference for straight lanes, not the shipped setting (docs/mpc-lab-calibration.md).

Choosing rule:
1. Keep the options with the highest lane-keeping pass rate on the training tracks (within one track).
2. Of those, choose the one with the smallest sum of its mean largest lateral error and mean curvature rate RMS,
   each divided by the best value among the kept options, which balances accuracy against smooth steering.

The lane-keeping pass rate uses the Card 2 thresholds that the MPC's settings control: no cone touched, the body
inside the lane, at most 0.10 m from the centre at the lane end, smooth steering and no solver fallback, and the car
must drive to within 1 m of the last cone pair. The speed
error depends on Car Control and the car, and the stop position on Task A7, so they are reported but not used.

Run from the repository root (a long run: 180 options on 100 tracks) with:
    python3 tests/policy/mpc_tuning.py --figure <path for the Pareto plot>
"""
from __future__ import annotations

import itertools
from multiprocessing import Pool
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from policy.action_policy.mpc import MpcConfig  # noqa: E402
from policy.action_policy.policy import MpcPolicy  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig  # noqa: E402
from real_car_measurements import (  # noqa: E402
    CAR20_MAX_LEFT_CURVATURE_PER_M, CAR20_MAX_RIGHT_CURVATURE_PER_M, LAB_LANE,
)
from stage1_scenarios import ConeFilteredPolicy, run_case  # noqa: E402
from synthetic_tracks import SPEED_M_PER_S, TRAINING_SEEDS, VALIDATION_SEEDS, synthetic_case  # noqa: E402

MEASURED_SETTINGS = {
    "target_speed_m_per_s": SPEED_M_PER_S,
    "lane_width_m": LAB_LANE.width_m,
    "camera_yaw_offset_deg": 0.0,
    "max_curvature_per_m": round(CAR20_MAX_LEFT_CURVATURE_PER_M, 2),
    "max_right_curvature_per_m": round(CAR20_MAX_RIGHT_CURVATURE_PER_M, 2),
}
"""The settings fixed by measurement: car 20's left and right curvature limits. No camera yaw correction, because
the policy now holds the camera pan at zero (synthetic_tracks.TrackRanges.camera_yaw_deg)."""
UNCORRECTED_SETTINGS = {"target_speed_m_per_s": SPEED_M_PER_S, "lane_width_m": LAB_LANE.width_m,
                        "camera_yaw_offset_deg": 0.0, "max_curvature_per_m": 3.0, "max_right_curvature_per_m": 0.0,
                        "heading_error_weight": 1.0, "curvature_change_weight": 1.0}
"""The settings before these measurements, only set to the lab speed and lane width: the 3.0 1/m curvature limit
both ways from the 45 degree shared default, and the first weights."""
OPTION_GRID = {
    "horizon_steps": (8, 12, 16, 20, 25),
    "lateral_error_weight": (3.0, 10.0, 30.0),
    "heading_error_weight": (0.3, 1.0, 3.0),
    "curvature_change_weight": (0.3, 1.0, 3.0, 10.0),
}
"""The options tried: every combination. Each predicted step is 0.1 s, so 25 steps predict 1.25 m at 0.5 m/s, within
the measured 1.8-3.0 m detection range."""
LANE_KEEPING_METRICS = ("cones_touched", "body_outside_lane_m", "lateral_error_at_lane_end_m",
                        "rms_curvature_rate_per_m_s", "fallback_count")
"""The metrics whose thresholds make up the lane-keeping pass."""


def options() -> list[dict]:
    """
    :return: Every option in OPTION_GRID, as MpcConfig keyword arguments.
    """
    return [dict(zip(OPTION_GRID, values)) for values in itertools.product(*OPTION_GRID.values())]


MAX_STOP_BEFORE_END_M = 1.0
"""A run only passes lane keeping if the car drove to within this distance of the last cone pair, so stopping early
cannot pass."""


def lane_keeping_passed(result) -> bool:
    """
    :return: Whether a run missed none of the lane-keeping thresholds and drove to near the lane end.
    """
    reached_end = float(np.max(result.x_m)) >= result.case.lane.length_m - MAX_STOP_BEFORE_END_M
    return reached_end and not any(failure.startswith(LANE_KEEPING_METRICS) for failure in result.failures)


def _run(job: tuple[dict, int]) -> dict:
    """Runs one option on one synthetic track. Module-level, so worker processes can run it."""
    settings, seed = job
    case = synthetic_case(seed)
    result = run_case(case, ConeFilteredPolicy(MpcPolicy(MpcConfig(**settings)), ConeFilterConfig()))
    return {**result.metrics, "lane_keeping_passed": float(lane_keeping_passed(result)),
            "passed": float(result.passed)}


def evaluate(settings_list: list[dict], seeds: tuple[int, ...], processes: int | None = None) -> list[dict]:
    """Runs each group of MPC settings on each track and summarises the runs.

    :param settings_list: MpcConfig keyword arguments for each design.
    :param seeds: The synthetic tracks.
    :param processes: Worker processes; one per core if None.
    :return: For each design, its mean metrics over the tracks (with the pass rates) and its worst calculation time.
    """
    jobs = [(settings, seed) for settings in settings_list for seed in seeds]
    with Pool(processes) as pool:
        runs = pool.map(_run, jobs, chunksize=8)
    summaries = []
    for i, settings in enumerate(settings_list):
        group = runs[i * len(seeds):(i + 1) * len(seeds)]
        summary = {"settings": settings}
        for name in ("lane_keeping_passed", "passed", "max_abs_lateral_error_m", "lateral_error_at_lane_end_m",
                     "rms_curvature_rate_per_m_s", "mean_speed_error_m_per_s", "cones_touched", "fallback_count"):
            summary[name] = float(np.nanmean([run[name] for run in group]))
        summary["p99_calc_time_s"] = float(max(run["p99_calc_time_s"] for run in group))
        summary["stopped_in_zone"] = float(np.mean([0.0 <= run["stop_past_last_cone_m"] <= 0.5 for run in group]))
        summaries.append(summary)
    return summaries


def choose(summaries: list[dict], track_count: int) -> dict:
    """Chooses one design with the rule in the module docstring.

    :param summaries: The training results of each option, from evaluate.
    :param track_count: Number of training tracks, for 'within one track'.
    :return: The chosen option's summary.
    """
    best_rate = max(s["lane_keeping_passed"] for s in summaries)
    kept = [s for s in summaries if s["lane_keeping_passed"] >= best_rate - 1.0 / track_count - 1e-9]
    best_lateral = min(s["max_abs_lateral_error_m"] for s in kept)
    best_smooth = min(s["rms_curvature_rate_per_m_s"] for s in kept)
    return min(kept, key=lambda s: s["max_abs_lateral_error_m"] / best_lateral
               + s["rms_curvature_rate_per_m_s"] / best_smooth)


def pareto_front(summaries: list[dict]) -> list[dict]:
    """
    :return: The designs no other design beats on both mean largest lateral error and mean curvature rate RMS.
    """
    return [s for s in summaries if not any(
        o["max_abs_lateral_error_m"] <= s["max_abs_lateral_error_m"]
        and o["rms_curvature_rate_per_m_s"] <= s["rms_curvature_rate_per_m_s"] and o is not s
        and (o["max_abs_lateral_error_m"], o["rms_curvature_rate_per_m_s"])
        != (s["max_abs_lateral_error_m"], s["rms_curvature_rate_per_m_s"]) for o in summaries)]


def plot(summaries: list[dict], chosen: dict, baselines: dict[str, dict], path: Path):
    """Saves the training results as a Pareto plot, coloured by lane-keeping pass rate."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 5))
    xs = [s["rms_curvature_rate_per_m_s"] for s in summaries]
    ys = [s["max_abs_lateral_error_m"] for s in summaries]
    points = ax.scatter(xs, ys, c=[100 * s["lane_keeping_passed"] for s in summaries], cmap="viridis", s=18)
    fig.colorbar(points, label="lane-keeping pass rate on training tracks [%]")
    front = sorted(pareto_front(summaries), key=lambda s: s["rms_curvature_rate_per_m_s"])
    ax.plot([s["rms_curvature_rate_per_m_s"] for s in front], [s["max_abs_lateral_error_m"] for s in front],
            "k--", linewidth=0.8, label="Pareto front")
    ax.plot(chosen["rms_curvature_rate_per_m_s"], chosen["max_abs_lateral_error_m"], "r*", markersize=14,
            label="chosen")
    for (name, summary), marker in zip(baselines.items(), ("s", "D")):
        ax.plot(summary["rms_curvature_rate_per_m_s"], summary["max_abs_lateral_error_m"], marker, color="red",
                markerfacecolor="none", markersize=9, label=name)
    ax.set_xlabel("mean curvature rate RMS [1/(m s)] (smaller is smoother)")
    ax.set_ylabel("mean largest lateral error [m]")
    ax.set_title(f"MPC options on {len(TRAINING_SEEDS)} synthetic lab tracks")
    ax.legend(fontsize=8)
    ax.grid(linestyle=":", linewidth=0.4)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def format_row(name: str, summary: dict) -> str:
    """
    :return: One Markdown table row of a design's summary.
    """
    s = summary
    return (f"| {name} | {100 * s['lane_keeping_passed']:.0f}% | {s['max_abs_lateral_error_m']:.3f} | "
            f"{s['lateral_error_at_lane_end_m']:.3f} | {s['rms_curvature_rate_per_m_s']:.2f} | "
            f"{s['cones_touched']:.2f} | {s['mean_speed_error_m_per_s']:.3f} | {100 * s['stopped_in_zone']:.0f}% | "
            f"{1000 * s['p99_calc_time_s']:.1f} |")


TABLE_HEADER = ("| Design | Lane-keeping pass | Largest lat. err. [m] | Lat. err. at end [m] | Curv. rate RMS "
                "[1/(m s)] | Cones touched | Speed err. [m/s] | Stopped in zone | Worst p99 calc [ms] |\n"
                "| --- | --- | --- | --- | --- | --- | --- | --- | --- |")


def card2_check(settings: dict) -> list:
    """Runs settings on the Card 2 tuning cases (shared-default car), each at its own speed and lane width.

    :return: The Stage1Result of each case.
    """
    from stage1_tuning_cases import TUNING_CASES

    results = []
    for case in TUNING_CASES:
        config = MpcConfig(**{**settings, "target_speed_m_per_s": case.speed_m_per_s,
                              "lane_width_m": case.lane.width_m, "camera_yaw_offset_deg": 0.0})
        results.append(run_case(case, ConeFilteredPolicy(MpcPolicy(config), ConeFilterConfig())))
    return results


def main():
    import argparse
    from stage1_scenarios import format_table

    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--figure", type=Path, help="save the Pareto plot here")
    args = parser.parse_args()
    grid = [{**MEASURED_SETTINGS, **option} for option in options()]
    training = evaluate(grid, TRAINING_SEEDS)
    chosen = choose(training, len(TRAINING_SEEDS))
    tuned = {k: v for k, v in chosen["settings"].items() if k in OPTION_GRID}
    print(f"Chosen from {len(grid)} options on {len(TRAINING_SEEDS)} training tracks: {tuned}\n")
    designs = {"previous settings (speed and lane width only)": UNCORRECTED_SETTINGS,
               "shipped settings": MEASURED_SETTINGS,
               "shipped limits, weights tuned on these straight tracks": chosen["settings"]}
    baselines = dict(zip(designs, evaluate(list(designs.values()), TRAINING_SEEDS)))
    validation = dict(zip(designs, evaluate(list(designs.values()), VALIDATION_SEEDS)))
    for title, results in (("Training tracks", baselines), ("Validation tracks", validation)):
        print(f"{title} ({len(TRAINING_SEEDS if title.startswith('Training') else VALIDATION_SEEDS)}):")
        print(TABLE_HEADER)
        print("\n".join(format_row(name, summary) for name, summary in results.items()))
        print()
    front = sorted(pareto_front(training), key=lambda s: s["rms_curvature_rate_per_m_s"])
    print("Training Pareto front:")
    print(TABLE_HEADER)
    print("\n".join(format_row(str({k: v for k, v in s["settings"].items() if k in OPTION_GRID}), s) for s in front))
    print("\nCard 2 tuning cases (shared-default car) with the tuned weights and curvature limit:")
    print(format_table(card2_check({k: v for k, v in chosen["settings"].items()
                                    if k in OPTION_GRID or k.startswith("max_")})))
    if args.figure:
        args.figure.parent.mkdir(parents=True, exist_ok=True)
        plot(training, chosen, {"previous settings": baselines[list(designs)[0]],
                                "shipped settings": baselines[list(designs)[1]]}, args.figure)


if __name__ == "__main__":
    main()
