"""Render fail_curvy_road frames and convert them to a real-time GIF with ffmpeg.

Run from the repository root with the project virtual environment active::

    python docs/logbook/ciaran/make_fail_curvy_road_gif.py

Outputs:
    docs/logbook/ciaran/figures/_fail_curvy_road_frames/frame_XXXX.png
    docs/logbook/ciaran/figures/fail_curvy_road.gif
"""

from __future__ import annotations

import argparse
import logging
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from scenarios.cone_filter_report import explain_cone_filter, load_cone_filter_config  # noqa: E402
from scenarios.renderer import (  # noqa: E402
    _draw_observations,
    _expand_limits,
    _frame_limits,
    _frame_points,
)
from scenarios.scenario_wrapper import Scenario, ScenarioType  # noqa: E402

_LOGGER = logging.getLogger(__name__)

HERE = Path(__file__).resolve().parent
FIGURES = HERE / "figures"
FRAMES_DIR = FIGURES / "_fail_curvy_road_frames"
GIF_PATH = FIGURES / "fail_curvy_road.gif"
SCENARIO_NAME = "fail_curvy_road"
SPEED_FACTOR = 1.0
HOLD_S = 1.0
DPI = 120


def _scenario_limits(frames: list) -> tuple[tuple[float, float], tuple[float, float]]:
    """Compute axis limits that cover every frame.

    :param frames: Observation/action pairs from the scenario.
    :return: Shared (xlim, ylim) for the GIF.
    """
    xlim: tuple[float, float] | None = None
    ylim: tuple[float, float] | None = None
    for observations, _actions in frames:
        frame_xlim, frame_ylim = _frame_limits(*_frame_points(observations))
        xlim = _expand_limits(xlim, frame_xlim)
        ylim = _expand_limits(ylim, frame_ylim)
    assert xlim is not None and ylim is not None
    return xlim, ylim


def render_frames(
    scenario: Scenario,
    frames_dir: Path,
    *,
    dpi: int = DPI,
) -> tuple[list[Path], float]:
    """Draw each scenario step to a PNG and return the real-time playback FPS.

    :param scenario: Scenario to render.
    :param frames_dir: Directory for ``frame_XXXX.png`` files.
    :param dpi: Matplotlib save DPI.
    :return: Written frame paths and the ffmpeg input framerate for real-time speed.
    """
    frames = list(scenario.steps())
    if not frames:
        raise ValueError(f"Scenario has no steps: {scenario.scenario_path}")

    frames_dir.mkdir(parents=True, exist_ok=True)
    for old in frames_dir.glob("frame_*.png"):
        old.unlink()

    cone_filter_config = load_cone_filter_config()
    xlim, ylim = _scenario_limits(frames)
    paths: list[Path] = []

    for index, (observations, _actions) in enumerate(frames):
        fig = plt.figure(figsize=(8.0, 8.0))
        ax = fig.add_axes((0.10, 0.08, 0.80, 0.84))
        reasons = explain_cone_filter(observations.cones, cone_filter_config) or []
        title = f"{scenario.name}  |  frame {index + 1}/{len(frames)}"
        if observations.cones is not None:
            kept_count = sum(reason is None for reason in reasons)
            title += f"  |  cones kept {kept_count}/{len(reasons)}"
        _draw_observations(
            ax,
            observations,
            title=title,
            removed_cones={i for i, reason in enumerate(reasons) if reason is not None},
        )
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        path = frames_dir / f"frame_{index:04d}.png"
        fig.savefig(path, dpi=dpi)
        plt.close(fig)
        paths.append(path)
        if index % 20 == 0 or index == len(frames) - 1:
            _LOGGER.info("Wrote %s", path.name)

    dts = [max(float(observations.policy.dt), 1e-3) for observations, _ in frames]
    mean_dt = sum(dts) / len(dts)
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
    # Clone the first and last frames so the GIF pauses briefly at each end.
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
    :return: Argument parser for the GIF build script.
    """
    parser = argparse.ArgumentParser(
        description="Render fail_curvy_road and convert it to a real-time GIF with ffmpeg.")
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
    """Render the scenario and convert it to a GIF.

    :param argv: Optional argument list excluding the program name.
    :return: Process exit code.
    """
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = build_parser().parse_args(argv)

    scenario = Scenario.from_name(ScenarioType.REAL, SCENARIO_NAME)
    if not scenario.scenario_path.is_file():
        _LOGGER.error("Scenario file does not exist: %s", scenario.scenario_path)
        return 1

    _LOGGER.info("Rendering %s from %s", SCENARIO_NAME, scenario.scenario_path)
    _paths, fps = render_frames(scenario, args.frames_dir, dpi=args.dpi)
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
