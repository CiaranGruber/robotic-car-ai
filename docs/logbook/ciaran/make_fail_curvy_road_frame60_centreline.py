"""Draw fail_curvy_road frame 60 with the centre line from the 26 Oct 2026 policy.

The figure uses a detached git worktree of commit ``d1d8096`` (tip of
``origin/dev`` on 26 Oct 2026) so lane detection, the cone filter and the local
map match that day's code and ``config/ai4r_policy.yaml``. Scenario JSON is
taken from the current repository.

Run from the repository root with the project virtual environment active::

    python docs/logbook/ciaran/make_fail_curvy_road_frame60_centreline.py

Outputs:
    docs/logbook/ciaran/figures/fail_curvy_road_frame60_centreline_oct26.png
"""

from __future__ import annotations

import argparse
import logging
import math
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

_LOGGER = logging.getLogger(__name__)

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
FIGURES = HERE / "figures"
OUT_PATH = FIGURES / "fail_curvy_road_frame60_centreline_oct26.png"
POLICY_WORKTREE = HERE / "_policy_oct26"
OCT26_COMMIT = "d1d809609d98a91e7cab06c9e6bd60d693a92989"
SCENARIO_PATH = REPO / "scenarios" / "data" / "real_scenarios" / "fail_curvy_road_scenario.json"
FRAME_NUMBER = 60

LEFT_COLOUR = "#2F80ED"
RIGHT_COLOUR = "#F2C94C"
CENTRE_COLOUR = "#2a7f62"


def ensure_policy_worktree(worktree: Path, commit: str) -> Path:
    """Create or reuse a detached worktree of the pinned policy commit.

    :param worktree: Directory for the worktree.
    :param commit: Git commit hash to check out.
    :return: Absolute worktree path.
    """
    worktree = worktree.resolve()
    if worktree.is_dir() and (worktree / "policy" / "policy_runner.py").is_file():
        head = subprocess.check_output(
            ["git", "-C", str(worktree), "rev-parse", "HEAD"],
            text=True,
        ).strip()
        if head.startswith(commit[:7]):
            return worktree
        _LOGGER.info("Updating worktree %s to %s", worktree, commit[:7])
        subprocess.run(["git", "-C", str(worktree), "checkout", "--detach", commit], check=True)
        return worktree

    if worktree.exists():
        raise RuntimeError(f"Worktree path exists but is not a usable checkout: {worktree}")

    _LOGGER.info("Creating policy worktree at %s (%s)", worktree, commit[:7])
    subprocess.run(
        ["git", "-C", str(REPO), "worktree", "add", "--detach", str(worktree), commit],
        check=True,
    )
    return worktree


def _take(cls, block: dict):
    """Build a config dataclass from the matching YAML keys only.

    :param cls: Dataclass type.
    :param block: YAML parameter block.
    :return: Config instance.
    """
    return cls(**{key: value for key, value in block.items() if key in cls.__dataclass_fields__})


def _along(start_m: float, end_m: float, step_m: float = 0.05) -> list[float]:
    """Sample distances along an arc, inclusive of the end.

    :param start_m: First distance in metres.
    :param end_m: Last distance in metres.
    :param step_m: Step size in metres.
    :return: Sampled distances.
    """
    if end_m < start_m:
        start_m, end_m = end_m, start_m
    values = []
    sample = start_m
    while sample < end_m - 1e-9:
        values.append(sample)
        sample += step_m
    values.append(end_m)
    return values


def _to_plot(x_forward: float, y_left: float) -> tuple[float, float]:
    """Map body-frame metres to the scenario renderer's plot axes.

    :param x_forward: Body-frame forward coordinate in metres.
    :param y_left: Body-frame left coordinate in metres.
    :return: Plot (horizontal, vertical) coordinates.
    """
    return -y_left, x_forward


def _plot_arc(axis, arc, start_m: float, end_m: float, *, colour: str, linewidth: float, label: str | None,
              linestyle: str = "-"):
    """Draw one lane arc in plot coordinates.

    :param axis: Matplotlib axes.
    :param arc: ``LaneArc`` in the body frame.
    :param start_m: Arc distance to start drawing.
    :param end_m: Arc distance to stop drawing.
    :param colour: Line colour.
    :param linewidth: Line width.
    :param label: Optional legend label.
    :param linestyle: Matplotlib linestyle.
    """
    points = [arc.point_at(sample) for sample in _along(start_m, end_m)]
    xs = [_to_plot(x, y)[0] for x, y in points]
    ys = [_to_plot(x, y)[1] for x, y in points]
    axis.plot(xs, ys, color=colour, linewidth=linewidth, linestyle=linestyle, label=label, zorder=5)


def build_policy(policy_repo: Path):
    """Construct ``MovementPolicy`` from the pinned checkout's YAML.

    :param policy_repo: Detached worktree of the 26 Oct commit.
    :return: Configured policy instance.
    """
    from policy.action_policy.avoidance import AvoidanceConfig
    from policy.action_policy.mpc import MpcConfig
    from policy.cone_filter.cone_filter import ConeFilterConfig
    from policy.control.control import ControlConfig
    from policy.lane_detection.lane_detection import LaneDetectionConfig
    from policy.local_map.local_map import LocalMapConfig
    from policy.markers.markers import MarkerConfig
    from policy.obstacle_detection.obstacle_detection import ObstacleDetectionConfig
    from policy.policy_runner import MovementPolicy

    params = yaml.safe_load((policy_repo / "config" / "ai4r_policy.yaml").read_text(encoding="utf-8"))[
        "/**/ai4r_policy"
    ]["ros__parameters"]
    return MovementPolicy(
        mpc_config=_take(MpcConfig, params["mpc"]),
        cone_filter_config=_take(ConeFilterConfig, params["cone_filter"]),
        control_config=_take(ControlConfig, params["control"]),
        lane_detection_config=_take(LaneDetectionConfig, params["lane_detection"]),
        local_map_config=_take(LocalMapConfig, params["local_map"]),
        obstacle_detection_config=_take(ObstacleDetectionConfig, params["obstacle_detection"]),
        avoidance_config=_take(AvoidanceConfig, params["avoidance"]),
        marker_config=_take(MarkerConfig, params["markers"]),
    )


def lane_at_frame(policy_repo: Path, scenario_path: Path, frame_number: int):
    """Replay the scenario under the pinned policy up to one frame.

    :param policy_repo: Detached worktree of the 26 Oct commit.
    :param scenario_path: Scenario JSON in the current repository.
    :param frame_number: 1-based frame index to capture.
    :return: Observations, lane detection, remembered width, recalled cones and
        removed-cone indices.
    """
    from policy.memory import CarMemory
    from scenarios.cone_filter_report import explain_cone_filter, load_cone_filter_config
    from scenarios.scenario_wrapper import Scenario

    policy = build_policy(policy_repo)
    memory = CarMemory()
    scenario = Scenario(scenario_path)
    cone_filter_config = load_cone_filter_config(policy_repo / "config" / "ai4r_policy.yaml")

    observations = None
    for index, (step_observations, _actions) in enumerate(scenario.steps(), start=1):
        policy.step(step_observations, memory)
        if index == frame_number:
            observations = step_observations
            break
    if observations is None:
        raise ValueError(f"Scenario has fewer than {frame_number} frames: {scenario_path}")

    reasons = explain_cone_filter(observations.cones, cone_filter_config) or []
    return (
        observations,
        policy.lanes,
        float(policy.lane_detector.lane_width_m),
        list(policy.local_map.recalled),
        {i for i, reason in enumerate(reasons) if reason is not None},
    )


def draw_figure(
    observations,
    lanes,
    *,
    remembered_width_m: float,
    recalled_cones: list,
    removed_cones: set[int],
    out_path: Path,
    commit: str,
    frame_number: int,
):
    """Write the altered scenario-renderer view with lane overlays.

    :param observations: Raw observations for the frame.
    :param lanes: ``LaneDetection`` from the pinned policy, or ``None``.
    :param remembered_width_m: ``LaneDetector`` remembered width at the frame.
    :param recalled_cones: Local-map cones recalled into the detect call.
    :param removed_cones: Cone indices removed by the pinned cone filter.
    :param out_path: Output PNG path.
    :param commit: Short commit label for the title.
    :param frame_number: 1-based frame number.
    """
    from policy.input_output import ConeColour
    from scenarios.renderer import _draw_observations, _frame_limits, _frame_points

    plt.rcParams.update({
        "font.size": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "savefig.dpi": 150,
    })

    status = "NO_LANE" if lanes is None else lanes.status.name
    title = (
        f"fail_curvy_road frame {frame_number}  |  policy {commit[:7]} (26 Oct 2026)\n"
        f"{status}  |  remembered width {remembered_width_m:.2f} m  |  "
        f"recalled cones {len(recalled_cones)}"
    )

    figure = plt.figure(figsize=(8.0, 8.4))
    axis = figure.add_axes((0.10, 0.08, 0.80, 0.82))
    _draw_observations(axis, observations, title=title, removed_cones=removed_cones)

    # Recalled cones are not in the logged batch; draw them faded so the left edge is visible.
    for cone in recalled_cones:
        colour = LEFT_COLOUR if cone.colour == ConeColour.BLUE else RIGHT_COLOUR
        px, py = _to_plot(cone.pos.x, cone.pos.y)
        axis.text(
            px,
            py,
            "▲",
            color=colour,
            fontsize=12,
            ha="center",
            va="center",
            alpha=0.35,
            zorder=2,
        )
    if recalled_cones:
        axis.plot([], [], linestyle="none", marker="^", color="#6B7280", alpha=0.35,
                  label="recalled cones (local map)")

    if lanes is not None and lanes.left_edge is not None:
        edge = lanes.left_edge
        _plot_arc(
            axis,
            edge.arc,
            edge.nearest_m,
            edge.farthest_m,
            colour=LEFT_COLOUR,
            linewidth=2.4,
            label=f"left edge ({edge.colour.name}, {edge.cone_count} cones)",
        )
    if lanes is not None and lanes.right_edge is not None:
        edge = lanes.right_edge
        _plot_arc(
            axis,
            edge.arc,
            edge.nearest_m,
            edge.farthest_m,
            colour=RIGHT_COLOUR,
            linewidth=2.4,
            label=f"right edge ({edge.colour.name}, {edge.cone_count} cones)",
        )
    if lanes is not None and lanes.has_lane:
        visible = float(lanes.visible_until_m)
        _plot_arc(
            axis,
            lanes.centre_line,
            0.0,
            visible,
            colour=CENTRE_COLOUR,
            linewidth=2.8,
            label="centre line",
        )
        # Extend a faint guide past the visible cones so the heading is obvious.
        _plot_arc(
            axis,
            lanes.centre_line,
            visible,
            max(visible + 1.0, 2.5),
            colour=CENTRE_COLOUR,
            linewidth=1.4,
            label=None,
            linestyle="--",
        )
        start = _to_plot(lanes.centre_line.x, lanes.centre_line.y)
        axis.scatter([start[0]], [start[1]], s=36, c=CENTRE_COLOUR, zorder=6)
        axis.annotate(
            (
                f"lat {lanes.lateral_error_m:+.2f} m\n"
                f"heading {math.degrees(lanes.heading_error_rad):+.1f}°"
            ),
            xy=start,
            xytext=(12, -28),
            textcoords="offset points",
            fontsize=9,
            color=CENTRE_COLOUR,
        )
    else:
        axis.text(
            0.02,
            0.02,
            "no centre line (NO_LANE)",
            transform=axis.transAxes,
            fontsize=10,
            color="#c8553d",
            va="bottom",
        )

    xlim, ylim = _frame_limits(*_frame_points(observations))
    if lanes is not None and lanes.has_lane:
        for sample in _along(0.0, max(float(lanes.visible_until_m) + 1.0, 2.5)):
            px, py = _to_plot(*lanes.centre_line.point_at(sample))
            xlim = (min(xlim[0], px - 0.2), max(xlim[1], px + 0.2))
            ylim = (min(ylim[0], py - 0.2), max(ylim[1], py + 0.2))
    axis.set_xlim(xlim)
    axis.set_ylim(ylim)
    axis.legend(loc="upper right", frameon=False, fontsize=8)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out_path, bbox_inches="tight")
    plt.close(figure)
    _LOGGER.info("Wrote %s", out_path)


def build_parser() -> argparse.ArgumentParser:
    """
    :return: Argument parser for the centre-line figure script.
    """
    parser = argparse.ArgumentParser(
        description="Draw fail_curvy_road frame 60 centre line using the 26 Oct 2026 policy.")
    parser.add_argument(
        "--policy-repo",
        type=Path,
        default=POLICY_WORKTREE,
        help=f"Checkout of {OCT26_COMMIT[:7]} (default: {POLICY_WORKTREE})",
    )
    parser.add_argument(
        "--scenario",
        type=Path,
        default=SCENARIO_PATH,
        help=f"Scenario JSON (default: {SCENARIO_PATH})",
    )
    parser.add_argument("--frame", type=int, default=FRAME_NUMBER, help=f"1-based frame (default: {FRAME_NUMBER})")
    parser.add_argument("--out", type=Path, default=OUT_PATH, help=f"Output PNG (default: {OUT_PATH})")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Build the pinned policy worktree if needed and write the figure.

    :param argv: Optional argument list excluding the program name.
    :return: Process exit code.
    """
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = build_parser().parse_args(argv)

    if not args.scenario.is_file():
        _LOGGER.error("Scenario file does not exist: %s", args.scenario)
        return 1

    policy_repo = ensure_policy_worktree(args.policy_repo, OCT26_COMMIT)

    # Prefer the pinned policy package over the current tree.
    sys.path.insert(0, str(REPO))
    sys.path.insert(0, str(policy_repo))

    observations, lanes, remembered_width_m, recalled_cones, removed_cones = lane_at_frame(
        policy_repo, args.scenario, args.frame)
    draw_figure(
        observations,
        lanes,
        remembered_width_m=remembered_width_m,
        recalled_cones=recalled_cones,
        removed_cones=removed_cones,
        out_path=args.out,
        commit=OCT26_COMMIT,
        frame_number=args.frame,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
