"""Render fail_curvy_road with the 26 Oct 2026 centre line as a real-time GIF.

Replays the scenario under the pinned policy worktree (``d1d8096``), overlays
lane edges and the centre line on each scenario-renderer frame, then builds a
palette GIF with ffmpeg.

Run from the repository root with the project virtual environment active::

    python docs/logbook/ciaran/make_fail_curvy_road_centreline_gif.py

Outputs:
    docs/logbook/ciaran/figures/_fail_curvy_road_centreline_frames/frame_XXXX.png
    docs/logbook/ciaran/figures/fail_curvy_road_centreline_oct26.gif
"""

from __future__ import annotations

import argparse
import logging
import math
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from make_fail_curvy_road_frame60_centreline import (  # noqa: E402
    CENTRE_COLOUR,
    LEFT_COLOUR,
    OCT26_COMMIT,
    POLICY_WORKTREE,
    RIGHT_COLOUR,
    SCENARIO_PATH,
    _along,
    _plot_arc,
    _to_plot,
    build_policy,
    ensure_policy_worktree,
)

_LOGGER = logging.getLogger(__name__)

FIGURES = HERE / "figures"
FRAMES_DIR = FIGURES / "_fail_curvy_road_centreline_frames"
GIF_PATH = FIGURES / "fail_curvy_road_centreline_oct26.gif"
SPEED_FACTOR = 1.0
HOLD_S = 1.0
DPI = 120


def _expand_limits(
    current: tuple[float, float] | None,
    frame: tuple[float, float],
) -> tuple[float, float]:
    """Grow axis limits to fit a frame; never shrink them.

    :param current: Limits already shown, or None on the first frame.
    :param frame: Limits needed for the current frame.
    :return: Limits that cover both current and frame.
    """
    if current is None:
        return frame
    return min(current[0], frame[0]), max(current[1], frame[1])


def _frame_plot_limits(observations, lanes) -> tuple[tuple[float, float], tuple[float, float]]:
    """Axis limits for one frame, including any centre-line samples.

    :param observations: Observations for the frame.
    :param lanes: Lane detection result, or None.
    :return: (xlim, ylim) in plot coordinates.
    """
    from scenarios.renderer import _frame_limits, _frame_points

    xlim, ylim = _frame_limits(*_frame_points(observations))
    if lanes is not None and lanes.has_lane:
        end_m = max(float(lanes.visible_until_m) + 1.0, 2.5)
        for sample in _along(0.0, end_m):
            px, py = _to_plot(*lanes.centre_line.point_at(sample))
            xlim = (min(xlim[0], px - 0.2), max(xlim[1], px + 0.2))
            ylim = (min(ylim[0], py - 0.2), max(ylim[1], py + 0.2))
        for side in (lanes.left_edge, lanes.right_edge):
            if side is None:
                continue
            for sample in _along(side.nearest_m, side.farthest_m):
                px, py = _to_plot(*side.arc.point_at(sample))
                xlim = (min(xlim[0], px - 0.2), max(xlim[1], px + 0.2))
                ylim = (min(ylim[0], py - 0.2), max(ylim[1], py + 0.2))
    return xlim, ylim


def _draw_lane_overlays(axis, lanes, recalled_cones):
    """Draw recalled cones, edges and the centre line onto an axes.

    :param axis: Matplotlib axes already holding the observation drawing.
    :param lanes: Lane detection result, or None.
    :param recalled_cones: Local-map cones recalled into the detect call.
    """
    from policy.input_output import ConeColour

    for cone in recalled_cones:
        colour = LEFT_COLOUR if cone.colour == ConeColour.BLUE else RIGHT_COLOUR
        px, py = _to_plot(cone.pos.x, cone.pos.y)
        axis.text(
            px,
            py,
            "▲",
            color=colour,
            fontsize=11,
            ha="center",
            va="center",
            alpha=0.35,
            zorder=2,
        )

    if lanes is not None and lanes.left_edge is not None:
        edge = lanes.left_edge
        _plot_arc(
            axis, edge.arc, edge.nearest_m, edge.farthest_m,
            colour=LEFT_COLOUR, linewidth=2.2, label=None,
        )
    if lanes is not None and lanes.right_edge is not None:
        edge = lanes.right_edge
        _plot_arc(
            axis, edge.arc, edge.nearest_m, edge.farthest_m,
            colour=RIGHT_COLOUR, linewidth=2.2, label=None,
        )
    if lanes is not None and lanes.has_lane:
        visible = float(lanes.visible_until_m)
        _plot_arc(
            axis, lanes.centre_line, 0.0, visible,
            colour=CENTRE_COLOUR, linewidth=2.6, label=None,
        )
        _plot_arc(
            axis, lanes.centre_line, visible, max(visible + 1.0, 2.5),
            colour=CENTRE_COLOUR, linewidth=1.3, label=None, linestyle="--",
        )
        start = _to_plot(lanes.centre_line.x, lanes.centre_line.y)
        axis.scatter([start[0]], [start[1]], s=28, c=CENTRE_COLOUR, zorder=6)
        axis.text(
            0.02,
            0.02,
            f"lat {lanes.lateral_error_m:+.2f} m  heading {math.degrees(lanes.heading_error_rad):+.1f}°",
            transform=axis.transAxes,
            fontsize=8,
            color=CENTRE_COLOUR,
            va="bottom",
        )
    else:
        axis.text(
            0.02,
            0.02,
            "no centre line (NO_LANE)",
            transform=axis.transAxes,
            fontsize=9,
            color="#c8553d",
            va="bottom",
        )


def render_frames(
    policy_repo: Path,
    scenario_path: Path,
    frames_dir: Path,
    *,
    dpi: int = DPI,
) -> tuple[list[Path], float]:
    """Draw every scenario step with the pinned policy's lane overlay.

    :param policy_repo: Detached worktree of the 26 Oct commit.
    :param scenario_path: Scenario JSON in the current repository.
    :param frames_dir: Directory for ``frame_XXXX.png`` files.
    :param dpi: Matplotlib save DPI.
    :return: Written frame paths and the ffmpeg input framerate for real-time speed.
    """
    from policy.memory import CarMemory
    from scenarios.cone_filter_report import explain_cone_filter, load_cone_filter_config
    from scenarios.renderer import _draw_observations
    from scenarios.scenario_wrapper import Scenario

    # The Oct 26 policy logs avoidance notes on many frames of this recording.
    logging.getLogger("policy.policy_runner").setLevel(logging.WARNING)
    logging.getLogger("policy.action_policy.avoidance").setLevel(logging.WARNING)

    policy = build_policy(policy_repo)
    memory = CarMemory()
    scenario = Scenario(scenario_path)
    cone_filter_config = load_cone_filter_config(policy_repo / "config" / "ai4r_policy.yaml")
    steps = list(scenario.steps())
    if not steps:
        raise ValueError(f"Scenario has no steps: {scenario_path}")

    records = []
    for observations, _actions in steps:
        policy.step(observations, memory)
        reasons = explain_cone_filter(observations.cones, cone_filter_config) or []
        records.append({
            "observations": observations,
            "lanes": policy.lanes,
            "remembered_width_m": float(policy.lane_detector.lane_width_m),
            "recalled": list(policy.local_map.recalled),
            "removed": {i for i, reason in enumerate(reasons) if reason is not None},
            "kept": sum(reason is None for reason in reasons),
            "total": len(reasons),
            "dt": max(float(observations.policy.dt), 1e-3),
        })

    xlim: tuple[float, float] | None = None
    ylim: tuple[float, float] | None = None
    for record in records:
        frame_xlim, frame_ylim = _frame_plot_limits(record["observations"], record["lanes"])
        xlim = _expand_limits(xlim, frame_xlim)
        ylim = _expand_limits(ylim, frame_ylim)
    assert xlim is not None and ylim is not None

    frames_dir.mkdir(parents=True, exist_ok=True)
    for old in frames_dir.glob("frame_*.png"):
        old.unlink()

    paths: list[Path] = []
    for index, record in enumerate(records):
        status = "NO_LANE" if record["lanes"] is None else record["lanes"].status.name
        title = (
            f"{scenario.name}  |  frame {index + 1}/{len(records)}  |  "
            f"policy {OCT26_COMMIT[:7]} (26 Oct)\n"
            f"{status}  |  remembered width {record['remembered_width_m']:.2f} m  |  "
            f"recalled {len(record['recalled'])}"
        )
        if record["total"]:
            title += f"  |  cones kept {record['kept']}/{record['total']}"

        figure = plt.figure(figsize=(8.0, 8.0))
        axis = figure.add_axes((0.10, 0.08, 0.80, 0.84))
        _draw_observations(
            axis,
            record["observations"],
            title=title,
            removed_cones=record["removed"],
        )
        _draw_lane_overlays(axis, record["lanes"], record["recalled"])
        axis.set_xlim(xlim)
        axis.set_ylim(ylim)
        path = frames_dir / f"frame_{index:04d}.png"
        figure.savefig(path, dpi=dpi)
        plt.close(figure)
        paths.append(path)
        if index % 20 == 0 or index == len(records) - 1:
            _LOGGER.info("Wrote %s (%s)", path.name, status)

    mean_dt = sum(record["dt"] for record in records) / len(records)
    fps = SPEED_FACTOR / mean_dt
    _LOGGER.info("mean dt=%.4fs; playback fps=%.3f", mean_dt, fps)
    return paths, fps


def make_gif_with_ffmpeg(
    frames_dir: Path,
    gif_path: Path,
    fps: float,
    *,
    hold_s: float = HOLD_S,
):
    """Build a palette-based GIF from numbered PNG frames using ffmpeg.

    :param frames_dir: Directory containing ``frame_%04d.png``.
    :param gif_path: Output GIF path.
    :param fps: Playback frames per second.
    :param hold_s: Seconds to freeze on the first and last frames.
    """
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg was not found on PATH")

    gif_path.parent.mkdir(parents=True, exist_ok=True)
    pattern = str(frames_dir / "frame_%04d.png")
    palette = frames_dir / "palette.png"
    video = (
        f"fps={fps:.6f},scale=640:-1:flags=lanczos,"
        f"tpad=start_mode=clone:start_duration={hold_s:.3f}:"
        f"stop_mode=clone:stop_duration={hold_s:.3f}"
    )
    subprocess.run(
        [ffmpeg, "-y", "-framerate", f"{fps:.6f}", "-i", pattern, "-vf", f"{video},palettegen", str(palette)],
        check=True,
    )
    subprocess.run(
        [
            ffmpeg,
            "-y",
            "-framerate",
            f"{fps:.6f}",
            "-i",
            pattern,
            "-i",
            str(palette),
            "-lavfi",
            f"{video}[x];[x][1:v]paletteuse",
            str(gif_path),
        ],
        check=True,
    )
    _LOGGER.info("Wrote %s (%.2fs hold on first and last frames)", gif_path, hold_s)


def build_parser() -> argparse.ArgumentParser:
    """
    :return: Argument parser for the centre-line GIF script.
    """
    parser = argparse.ArgumentParser(
        description="Render fail_curvy_road with the 26 Oct centre line as a real-time GIF.")
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
    parser.add_argument(
        "--frames-dir",
        type=Path,
        default=FRAMES_DIR,
        help=f"Directory for PNG frames (default: {FRAMES_DIR})",
    )
    parser.add_argument(
        "--gif",
        type=Path,
        default=GIF_PATH,
        help=f"Output GIF path (default: {GIF_PATH})",
    )
    parser.add_argument("--dpi", type=int, default=DPI, help=f"PNG DPI (default: {DPI})")
    parser.add_argument(
        "--keep-frames",
        action="store_true",
        help="Keep the PNG frames after building the GIF.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Render the centre-line overlay GIF.

    :param argv: Optional argument list excluding the program name.
    :return: Process exit code.
    """
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = build_parser().parse_args(argv)

    if not args.scenario.is_file():
        _LOGGER.error("Scenario file does not exist: %s", args.scenario)
        return 1

    policy_repo = ensure_policy_worktree(args.policy_repo, OCT26_COMMIT)
    sys.path.insert(0, str(REPO))
    sys.path.insert(0, str(policy_repo))

    _LOGGER.info(
        "Rendering centre-line GIF from %s with policy %s",
        args.scenario,
        OCT26_COMMIT[:7],
    )
    _paths, fps = render_frames(policy_repo, args.scenario, args.frames_dir, dpi=args.dpi)
    make_gif_with_ffmpeg(args.frames_dir, args.gif, fps)

    if not args.keep_frames:
        for path in args.frames_dir.glob("frame_*.png"):
            path.unlink()
        palette = args.frames_dir / "palette.png"
        if palette.is_file():
            palette.unlink()
        _LOGGER.info("Removed temporary PNG frames under %s", args.frames_dir)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
