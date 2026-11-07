"""Stage 1 design runs (Card 3): the cone-only baseline (Task A3) and the MPC options with their Pareto front (A4).

Every design runs through the same harness as the test cases (stage1_scenarios.run_case), on the tuning cases only,
with the shipped cone filter, the same lane reference from the cones and the same lane-loss and lane-end rule (Task
A7), so only the steering differs. The results go into docs/stage1-design.md.

Run from the repository root (each takes a few minutes):
    python3 tests/policy/stage1_design.py baseline
    python3 tests/policy/stage1_design.py mpc --figure docs/figures/stage1_mpc_pareto.png
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import dataclasses
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import stage1_scenarios as s1  # noqa: E402
from stage1_tuning_cases import TUNING_CASES  # noqa: E402
from policy.action_policy.mpc import MpcConfig, MpcSolution, PathTrackingState  # noqa: E402
from policy.action_policy.policy import MpcPolicy  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig  # noqa: E402

EXCLUDED_FROM_FRONT = ("T11",)
"""Cases left out of the Pareto metrics: T11's offset comes from the simulated tyre model (Task A2), not the design."""


class PurePursuitController:
    """Pure pursuit on the lane centre line fitted from the cones: the cone-only baseline of Task A3.

    It steers along the arc through the centre-line point lookahead_m ahead, scaled by gain. It has MpcController's
    solve signature, so MpcPolicy runs it with the same reference, lane-loss and lane-end rule; its 'plan' is that
    curvature at every step. It ignores the detection age, as a simple tracker would.
    """

    def __init__(self, config: MpcConfig, lookahead_m: float, gain: float):
        """
        :param config: The MPC settings, for the horizon and the curvature limit.
        :param lookahead_m: Distance ahead of the car of the point it steers towards, in metres.
        :param gain: Factor on the pure pursuit curvature.
        """
        self.config = config
        self.lookahead_m = lookahead_m
        self.gain = gain

    def solve(self, state: PathTrackingState, previous_curvature_per_m: float, delay_s: float) -> MpcSolution:
        start = time.perf_counter()
        # The centre-line point lookahead_m ahead, in the car's frame (left positive)
        ahead_m = self.lookahead_m
        left_m = -state.lateral_error_m - ahead_m * math.tan(state.heading_error_rad)
        curvature = self.gain * 2.0 * left_m / (ahead_m ** 2 + left_m ** 2) + state.path_curvature_per_m
        limit = self.config.max_curvature_per_m
        curvature = min(max(curvature, -limit), limit)
        return MpcSolution(np.full(self.config.horizon_steps, curvature), True, time.perf_counter() - start)


@dataclass(frozen=True)
class Design:
    """One design to run: the baseline with a lookahead and gain, or the MPC with some settings changed."""
    name: str
    kind: str
    """'pure_pursuit', 'mpc' or 'rl'."""
    settings: tuple[tuple[str, float | str], ...] = ()
    """For 'mpc', MpcConfig fields to change; for 'pure_pursuit', lookahead_m and gain; for 'rl', model (a path)."""
    noises_m: tuple[float | None, ...] = (None,)
    """Detector noise settings to run every case with: a noise for every cone, or None for each case's own. The
    design's results over all of them are summarised together."""

    def policy(self, case: s1.Stage1Case):
        config = s1.mpc_config_for(case)
        settings = dict(self.settings)
        if self.kind == "mpc":
            policy = MpcPolicy(dataclasses.replace(config, **settings))
        elif self.kind == "rl":
            from stage1_rl import RlController
            policy = MpcPolicy(config)
            policy.controller = RlController(config, settings["model"])
        else:
            policy = MpcPolicy(config)
            policy.controller = PurePursuitController(config, settings["lookahead_m"], settings["gain"])
        return s1.ConeFilteredPolicy(policy, ConeFilterConfig())


def with_noise(case: s1.Stage1Case, noise_m: float | None) -> s1.Stage1Case:
    """
    :return: The case with that detector noise on every cone, or unchanged for None.
    """
    if noise_m is None:
        return case
    return dataclasses.replace(case, disturbances=dataclasses.replace(case.disturbances, noise_m=noise_m))


def _run(job: tuple[Design, s1.Stage1Case]) -> s1.Stage1Result:
    design, case = job
    return s1.run_case(case, design.policy(case))


def run_designs(designs: list[Design], cases=TUNING_CASES) -> dict[str, list[s1.Stage1Result]]:
    """Runs every design on every case with each of its noise settings, in parallel processes.

    :return: The results of each design by name.
    """
    jobs = [(design, with_noise(case, noise)) for design in designs for noise in design.noises_m for case in cases]
    with ProcessPoolExecutor() as pool:
        results = list(pool.map(_run, jobs, chunksize=4))
    by_design: dict[str, list[s1.Stage1Result]] = {design.name: [] for design in designs}
    for (design, _), result in zip(jobs, results):
        by_design[design.name].append(result)
    return by_design


def rms_lateral_error_m(result: s1.Stage1Result) -> float:
    """RMS distance from the lane centre from 1 m after the start line to the last cone pair: how well the design
    tracks the centre once the start offset has had 1 m to settle."""
    lane = result.case.lane
    beside = (result.x_m >= 1.0) & (result.x_m <= lane.length_m)
    return float(np.sqrt(np.mean(result.y_m[beside] ** 2))) if beside.any() else math.nan


REQUIREMENT_OF_METRIC = {
    "body_outside_lane_m": "R1a", "lateral_error_at_lane_end_m": "R1b", "cones_touched": "R2",
    "mean_speed_error_m_per_s": "R3", "rms_curvature_rate_per_m_s": "R4", "stop_past_last_cone_m": "R5",
    "did not stop": "R5", "p99_calc_time_s": "R6a", "fallback_count": "R6b", "lane_lost_to_stop_request_s": "R7",
}
"""The Card 1.1 requirement each failure message is about."""


def failed_requirements(result: s1.Stage1Result) -> str:
    """
    :return: The case name and the requirements it fails, such as 'T5 (R1b, R4)'.
    """
    ids = sorted({req for failure in result.failures for key, req in REQUIREMENT_OF_METRIC.items()
                  if failure.startswith(key)})
    return f"{result.case.name} ({', '.join(ids)})"


def summarise(name: str, results: list[s1.Stage1Result]) -> dict:
    """The design's metrics over the cases. Tracking and smoothness leave out EXCLUDED_FROM_FRONT."""
    scored = [r for r in results if r.case.name not in EXCLUDED_FROM_FRONT]
    return {
        "name": name,
        "passed": sum(r.passed for r in results),
        "cases": len(results),
        "failed": ", ".join(failed_requirements(r) for r in results if not r.passed),
        "tracking_m": float(np.mean([rms_lateral_error_m(r) for r in scored])),
        "worst_lateral_end_m": max(r.metrics["lateral_error_at_lane_end_m"] for r in scored),
        "outside_lane_m": max(r.metrics["body_outside_lane_m"] for r in results),
        "touched": sum(r.metrics["cones_touched"] for r in results),
        "smoothness": float(np.mean([r.metrics["rms_curvature_rate_per_m_s"] for r in scored])),
        "worst_smoothness": max(r.metrics["rms_curvature_rate_per_m_s"] for r in scored),
        "fallbacks": sum(r.metrics["fallback_count"] for r in results),
        "p99_calc_ms": 1000.0 * max(r.metrics["p99_calc_time_s"] for r in results),
    }


def mark_pareto_front(rows: list[dict], axes=("tracking_m", "smoothness")) -> list[dict]:
    """Marks each row on_front if no other row is at least as good on every axis and better on one (smaller is
    better). Only designs that pass every run, so meet every threshold with no fallbacks, can be on the front
    (Card 1.3)."""
    def eligible(row):
        return row["passed"] == row["cases"] and row["fallbacks"] == 0

    for row in rows:
        row["on_front"] = eligible(row) and not any(
            eligible(other) and other is not row
            and all(other[a] <= row[a] for a in axes) and any(other[a] < row[a] for a in axes)
            for other in rows)
    return rows


def format_summary(rows: list[dict]) -> str:
    lines = ["| Design | Passed | Failed | Tracking RMS [m] | Worst lat. err. at end [m] | Outside lane [m] | Touched "
             "| Curv. rate RMS mean / worst [1/(m s)] | Fallbacks | p99 calc [ms] | Pareto front |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in rows:
        lines.append(f"| {r['name']} | {r['passed']}/{r['cases']} | {r['failed'] or '-'} | {r['tracking_m']:.4f} | "
                     f"{r['worst_lateral_end_m']:.3f} | {r['outside_lane_m']:.3f} | {r['touched']:.0f} | "
                     f"{r['smoothness']:.2f} / {r['worst_smoothness']:.2f} | {r['fallbacks']:.0f} | "
                     f"{r['p99_calc_ms']:.1f} | {'yes' if r.get('on_front') else ''} |")
    return "\n".join(lines)


def plot_front(rows: list[dict], path: Path, title: str, label_all: bool = False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 5.5))
    passing = [r for r in rows if r["passed"] == r["cases"]]
    failing = [r for r in rows if r["passed"] < r["cases"]]
    ax.plot([r["smoothness"] for r in failing], [1000 * r["tracking_m"] for r in failing], "x", color="0.75",
            markersize=4, label="Fails a threshold on some run")
    ax.plot([r["smoothness"] for r in passing], [1000 * r["tracking_m"] for r in passing], "o", color="0.45",
            markersize=4, label="Passes every run")
    front = sorted((r for r in rows if r.get("on_front")), key=lambda r: r["smoothness"])
    ax.plot([r["smoothness"] for r in front], [1000 * r["tracking_m"] for r in front], "o-", color="C3",
            markersize=6, label="Pareto front of the passing designs")
    if label_all:
        front = front + [r for r in rows if not r.get("on_front")]
    # Label each distinct front point with a letter; designs that differ only in an inactive setting share one
    points: dict[tuple[float, float], list[str]] = {}
    for r in front:
        points.setdefault((round(r["smoothness"], 2), round(1000 * r["tracking_m"], 1)), []).append(r["name"])
    key = []
    for letter, ((x, y), names) in zip("ABCDEFGHIJKLMNOP", points.items()):
        ax.annotate(letter, (x, y), textcoords="offset points", xytext=(-12, -3), fontsize=9, color="C3",
                    fontweight="bold")
        key.append(f"{letter}: " + " / ".join(names))
    ax.text(0.98, 0.02, "\n".join(key), transform=ax.transAxes, ha="right", va="bottom", fontsize=7,
            family="monospace", bbox=dict(facecolor="white", edgecolor="0.8"))
    if label_all:
        ax.set_xlim(0.0, 1.1 * max(r["smoothness"] for r in rows))
        ax.set_ylim(0.0, 1100 * max(r["tracking_m"] for r in rows))
    else:
        ax.set_xlim(0.0, 1.1)
        ax.set_ylim(5.0, 20.0)
    ax.legend(loc="upper left" if label_all else "upper right", fontsize=8)
    ax.set_xlabel("Mean RMS curvature rate [1/(m s)] (R4, smaller is smoother)")
    ax.set_ylabel("Mean RMS lateral error after 1 m [mm] (smaller is closer to the centre)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def baseline_designs() -> list[Design]:
    """Task A3: pure pursuit over a grid of lookahead distances and gains, without and with detector noise."""
    designs = []
    for noise in (0.0, 0.03):
        for lookahead in (0.5, 0.75, 1.0, 1.5):
            for gain in (0.5, 1.0, 1.5):
                designs.append(Design(f"PP L={lookahead} k={gain} noise={noise}", "pure_pursuit",
                                      (("lookahead_m", lookahead), ("gain", gain)), (noise,)))
    return designs


def mpc_designs() -> list[Design]:
    """Task A4: a grid of MPC options over the horizon, the cost weights and the curvature limit, each run with the
    cases' own detector noise and with 0.03 m on every cone, plus the A3 baseline for reference."""
    noises = (None, 0.03)
    designs = [Design(BASELINE, "pure_pursuit", (("lookahead_m", 1.0), ("gain", 1.0)), noises)]
    for horizon in (12, 20, 30):
        for lateral in (3.0, 10.0, 30.0):
            for heading in (0.3, 1.0, 3.0):
                for change in (0.3, 1.0, 3.0, 10.0):
                    for max_curvature in (3.0, 1.0, 0.5):
                        settings = (("horizon_steps", horizon), ("lateral_error_weight", lateral),
                                    ("heading_error_weight", heading), ("curvature_change_weight", change),
                                    ("max_curvature_per_m", max_curvature))
                        designs.append(Design(mpc_name(dict(settings)), "mpc", settings, noises))
    return designs


BASELINE = "PP L=1.0 k=1.0 (A3 baseline)"
"""The A3 baseline: pure pursuit with a 1.0 m lookahead and gain 1.0."""


def mpc_name(settings: dict) -> str:
    """A short name for an MPC option: horizon, lateral/heading weights, curvature-change weight, curvature limit."""
    return (f"MPC N{settings['horizon_steps']} q{settings['lateral_error_weight']:g}/"
            f"{settings['heading_error_weight']:g} r{settings['curvature_change_weight']:g} "
            f"k{settings['max_curvature_per_m']:g}")


CHOSEN_MPC = "MPC N20 q3/1 r3 k3 (A4 choice)"
"""The Task A4 choice, now the MpcConfig defaults."""


def rl_designs(models: list[Path]) -> list[Design]:
    """Task A5: each trained RL policy, with the A4 MPC and the A3 baseline for reference, on the same runs as A4."""
    noises = (None, 0.03)
    return ([Design(BASELINE, "pure_pursuit", (("lookahead_m", 1.0), ("gain", 1.0)), noises),
             Design(CHOSEN_MPC, "mpc", (), noises)]
            + [Design(f"RL {model.stem}", "rl", (("model", str(model)),), noises) for model in models])


STARTING = mpc_name({"horizon_steps": 12, "lateral_error_weight": 10.0, "heading_error_weight": 1.0,
                     "curvature_change_weight": 1.0, "max_curvature_per_m": 3.0})
"""The A4 starting values (the MPC from the first car run) in the grid."""


def m1(rl_model: Path, figures: Path | None) -> str:
    """Milestone M1: the A4 MPC and the A5 RL policy (and the A3 baseline for reference) on the unseen cases, with
    the cases' own disturbances, judged by the Card 1.3 rule: eligible only if every unseen case passes; the winner
    is on the Pareto front of the worst case of R1b, R3 and R4 over the cases.

    :return: The per-case tables and the decision table, as Markdown.
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent / "unseen"))
    from stage1_unseen_cases import UNSEEN_CASES

    designs = [Design(BASELINE, "pure_pursuit", (("lookahead_m", 1.0), ("gain", 1.0))),
               Design(CHOSEN_MPC, "mpc"),
               Design(f"RL {rl_model.stem} (A5 choice)", "rl", (("model", str(rl_model)),))]
    results = run_designs(designs, UNSEEN_CASES)
    parts = []
    rows = []
    for name, runs in results.items():
        parts.append(f"**{name}**\n\n{s1.format_table(runs)}")
        rows.append({"name": name, "eligible": all(r.passed for r in runs),
                     "failed": ", ".join(failed_requirements(r) for r in runs if not r.passed),
                     "R1b": max(r.metrics["lateral_error_at_lane_end_m"] for r in runs),
                     "R3": max(r.metrics["mean_speed_error_m_per_s"] for r in runs),
                     "R4": max(r.metrics["rms_curvature_rate_per_m_s"] for r in runs),
                     "R2": sum(r.metrics["cones_touched"] for r in runs),
                     "R1a": max(r.metrics["body_outside_lane_m"] for r in runs)})
        if figures:
            for r in runs:
                s1.plot_result(r, figures / f"stage1_M1_{name.split()[0]}_{r.case.name}.png")
    axes = ("R1b", "R3", "R4")
    for row in rows:
        row["on_front"] = row["eligible"] and not any(
            other["eligible"] and other is not row and all(other[a] <= row[a] for a in axes)
            and any(other[a] < row[a] for a in axes) for other in rows)
    table = ["| Design | Every unseen case passes | Failed | Worst R1b [m] | Worst R3 [m/s] | Worst R4 [1/(m s)] "
             "| Cones touched | Worst outside lane [m] | Pareto front |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in rows:
        table.append(f"| {r['name']} | {'yes' if r['eligible'] else 'no'} | {r['failed'] or '-'} | {r['R1b']:.3f} | "
                     f"{r['R3']:.3f} | {r['R4']:.2f} | {r['R2']:.0f} | {r['R1a']:.3f} | "
                     f"{'yes' if r['on_front'] else ''} |")
    return "\n\n".join(parts + ["\n".join(table)])


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("which", choices=("baseline", "mpc", "rl", "m1"))
    parser.add_argument("--figures", type=Path, help="for 'm1', save each unseen run's trajectory in this folder")
    parser.add_argument("--models", type=Path, nargs="*", default=[], help="trained RL models, for 'rl'")
    parser.add_argument("--figure", type=Path, help="save the Pareto front figure here")
    parser.add_argument("--csv", type=Path, help="save every design's summary here")
    args = parser.parse_args()
    if args.which == "m1":
        print(m1(args.models[0], args.figures))
        return
    designs = {"baseline": baseline_designs, "mpc": mpc_designs}.get(args.which, lambda: rl_designs(args.models))()
    rows = mark_pareto_front([summarise(name, runs) for name, runs in run_designs(designs).items()])
    if args.csv:
        import csv
        with args.csv.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    if args.which in ("mpc", "rl"):
        shown = [r for r in rows if r["on_front"] or r["name"] in (BASELINE, STARTING) or args.which == "rl"]
        print(format_summary(sorted(shown, key=lambda r: (-r["passed"], r["tracking_m"]))))
    else:
        print(format_summary(rows))
    if args.figure:
        plot_front(rows, args.figure, "Stage 1 tuning cases, own noise and 0.03 m", label_all=args.which == "rl")


if __name__ == "__main__":
    main()
