"""Interactive rendering of scenario observations in the car body frame.

Note: This module was primarily written by AI with manual review and adjustments.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _configure_tcl_tk_paths():
    """Point Tcl/Tk at the standard Windows Python layout when needed.

    Some Windows CPython installs keep Tcl under tcl/tcl8.6, but _tkinter looks
    in lib/tcl8.6 and then fails with "Can't find a usable init.tcl".
    """
    if sys.platform != "win32":
        return
    base = Path(sys.base_prefix) / "tcl"
    tcl_library = base / "tcl8.6"
    tk_library = base / "tk8.6"
    if "TCL_LIBRARY" not in os.environ and tcl_library.is_dir():
        os.environ["TCL_LIBRARY"] = str(tcl_library)
    if "TK_LIBRARY" not in os.environ and tk_library.is_dir():
        os.environ["TK_LIBRARY"] = str(tk_library)


_configure_tcl_tk_paths()

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.backend_bases import CloseEvent, MouseEvent
from matplotlib.figure import Figure
from matplotlib.widgets import Button, Slider

from policy.input_output import CarActions, CarObservations, ConeColour
from scenarios.scenario_wrapper import Scenario, ScenarioType

_CONE_COLOURS = {
    ConeColour.YELLOW: "#F2C94C",
    ConeColour.BLUE: "#2F80ED",
}
"""Fill colours for yellow and blue cone markers."""

_VELOCITY_ARROW_SCALE = 1.0
"""Plot metres of arrow length per m/s of wheel speed."""

_MIN_FACING_ARROW_M = 0.0
"""Shortest facing arrow when speed is missing or near zero."""


def _frame_limits(
    xs: list[float], ys: list[float],
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Compute padded axis limits for one set of plot points.

    :param xs: X coordinates included in the frame, at least the origin.
    :param ys: Y coordinates included in the frame, at least the origin.
    :return: Padded (x_min, x_max) and (y_min, y_max) for this frame.
    """
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    pad = 0.15 * span
    return (min(xs) - pad, max(xs) + pad), (min(ys) - pad, max(ys) + pad)


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


def _to_plot(x_forward: float, y_left: float) -> tuple[float, float]:
    """Map body-frame metres to bird's-eye plot coordinates.

    Forward (+x) is up the screen and left (+y) is to the left, so lane cones
    run with the car instead of across the page.

    :param x_forward: Body-frame forward coordinate in metres.
    :param y_left: Body-frame left coordinate in metres.
    :return: Plot (horizontal, vertical) coordinates in metres.
    """
    return -y_left, x_forward


def _velocity_arrow_length(observations: CarObservations) -> float:
    """Length of the facing arrow from wheel speed.

    :param observations: Observations for one policy step.
    :return: Body-frame arrow length in metres along +x.
    """
    wheel_speed = observations.car.wheel_speed
    speed_m_per_s = 0.0 if wheel_speed is None else float(wheel_speed)
    return max(speed_m_per_s * _VELOCITY_ARROW_SCALE, _MIN_FACING_ARROW_M)


def _frame_points(observations: CarObservations) -> tuple[list[float], list[float]]:
    """Collect plot points used when sizing axes for one frame.

    :param observations: Observations for one policy step.
    :return: Plot-coordinate x and y samples used for axis limits.
    """
    plot_xs = [0.0]
    plot_ys = [0.0]
    if observations.cones:
        for cone in observations.cones:
            px, py = _to_plot(cone.pos.x, cone.pos.y)
            plot_xs.append(px)
            plot_ys.append(py)
    tip_x, tip_y = _to_plot(_velocity_arrow_length(observations), 0.0)
    plot_xs.append(tip_x)
    plot_ys.append(tip_y)
    return plot_xs, plot_ys


def _draw_observations(
    ax: Axes,
    observations: CarObservations,
    *,
    title: str | None = None,
):
    """Draw one observation frame onto an axes.

    :param ax: Axes to draw on. Cleared before drawing.
    :param observations: Observations for one policy step.
    :param title: Optional axes title. Defaults to a body-frame label.
    """
    ax.cla()
    ax.set_aspect("equal")
    ax.set_xlabel("left ← y (m) → right")
    ax.set_ylabel("x (m, forwards)")
    ax.axhline(0.0, color="0.85", linewidth=0.8)
    ax.axvline(0.0, color="0.85", linewidth=0.8)

    if observations.cones:
        for cone in observations.cones:
            colour = _CONE_COLOURS[cone.colour]
            px, py = _to_plot(cone.pos.x, cone.pos.y)
            ax.text(
                px,
                py,
                "▲",
                color=colour,
                fontsize=14,
                ha="center",
                va="center",
                zorder=2,
            )

    ax.text(0.0, 0.0, "C", fontsize=18, ha="center", va="center", zorder=3)

    tip_x, tip_y = _to_plot(_velocity_arrow_length(observations), 0.0)
    if tip_x != 0.0 or tip_y != 0.0:
        ax.annotate(
            "",
            xy=(tip_x, tip_y),
            xytext=(0.0, 0.0),
            arrowprops={
                "arrowstyle": "->",
                "color": "black",
                "lw": 1.8,
                "mutation_scale": 14,
            },
            zorder=4,
        )

    ax.set_title(title or "Observations (body frame, forward up)")


class ScenarioRenderer:
    """Interactive scenario player with fixed axes, play/pause and reverse."""

    def __init__(self, scenario: Scenario):
        """Load every step and build the interactive figure.

        :param scenario: Scenario whose steps are loaded and played.
        """
        self._scenario = scenario
        self._frames: list[tuple[CarObservations, CarActions]] = list(scenario.steps())
        if not self._frames:
            raise ValueError(f"Scenario has no steps: {scenario.scenario_path}")

        self._index = 0
        self._playing = False
        self._direction = 1
        self._view_initialised = False
        self._xlim, self._ylim = self._limits_for_scenario()

        self._fig: Figure
        self._ax: Axes
        self._slider: Slider
        self._play_button: Button
        self._direction_button: Button
        self._timer = None
        self._build_figure()

    def _limits_for_scenario(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """Compute default axis limits from every frame in the scenario.

        :return: Axis limits covering every frame in the scenario.
        """
        xlim: tuple[float, float] | None = None
        ylim: tuple[float, float] | None = None
        for observations, _actions in self._frames:
            frame_xlim, frame_ylim = _frame_limits(*_frame_points(observations))
            xlim = _expand_limits(xlim, frame_xlim)
            ylim = _expand_limits(ylim, frame_ylim)
        assert xlim is not None and ylim is not None
        return xlim, ylim

    def _build_figure(self):
        """Create the plot, slider and transport controls."""
        self._fig = plt.figure(figsize=(8.0, 8.0))
        self._ax = self._fig.add_axes((0.10, 0.28, 0.80, 0.64))

        slider_ax = self._fig.add_axes((0.15, 0.14, 0.70, 0.04))
        self._slider = Slider(
            slider_ax,
            "Frame",
            0,
            len(self._frames) - 1,
            valinit=0,
            valstep=1,
        )
        self._slider.on_changed(self._on_slider)

        play_ax = self._fig.add_axes((0.15, 0.04, 0.20, 0.06))
        self._play_button = Button(play_ax, "Play")
        self._play_button.on_clicked(self._on_play_clicked)

        direction_ax = self._fig.add_axes((0.40, 0.04, 0.25, 0.06))
        self._direction_button = Button(direction_ax, "Forward")
        self._direction_button.on_clicked(self._on_direction_clicked)

        self._timer = self._fig.canvas.new_timer(interval=1)
        self._timer.add_callback(self._on_timer)

        self._fig.canvas.mpl_connect("close_event", self._on_close)
        self._show_frame(0)

    def _title_for_index(self, index: int) -> str:
        """Build the axes title for one frame.

        :param index: Frame index.
        :return: Title including scenario name and frame position.
        """
        return (
            f"{self._scenario.name}  |  frame {index + 1}/{len(self._frames)}"
        )

    def _show_frame(self, index: int):
        """Draw one frame, preserving any user zoom after the first draw.

        :param index: Frame index to draw.
        """
        # Keep any user zoom/pan; only the first draw uses the scenario defaults.
        if self._view_initialised:
            xlim = self._ax.get_xlim()
            ylim = self._ax.get_ylim()
        else:
            xlim = self._xlim
            ylim = self._ylim
            self._view_initialised = True

        self._index = index
        observations = self._frames[index][0]
        _draw_observations(
            self._ax,
            observations,
            title=self._title_for_index(index),
        )
        self._ax.set_xlim(xlim)
        self._ax.set_ylim(ylim)
        if self._slider.val != index:
            self._slider.set_val(index)
        self._fig.canvas.draw_idle()

    def _set_playing(self, playing: bool):
        """Start or stop timed playback.

        :param playing: Whether playback should run.
        """
        self._playing = playing
        self._play_button.label.set_text("Pause" if playing else "Play")
        if playing:
            if not self._arm_timer_for_next_step():
                return
            self._timer.start()
        else:
            self._timer.stop()
        self._fig.canvas.draw_idle()

    def _step_interval_s(self, from_index: int, to_index: int) -> float:
        """Seconds between two frames from observation dt.

        :param from_index: Frame index before the step.
        :param to_index: Frame index after the step.
        :return: Seconds to wait for that step, from observation dt.
        """
        if to_index > from_index:
            return max(self._frames[to_index][0].policy.dt, 0.0)
        if to_index < from_index:
            return max(self._frames[from_index][0].policy.dt, 0.0)
        return 0.0

    def _arm_timer_for_next_step(self) -> bool:
        """Set the timer interval from the next step's observation dt.

        :return: False when playback has reached the end and was stopped.
        """
        next_index = self._index + self._direction
        if next_index < 0 or next_index >= len(self._frames):
            self._set_playing(False)
            return False
        interval_ms = max(int(self._step_interval_s(self._index, next_index) * 1000), 1)
        self._timer.interval = interval_ms
        return True

    def _advance(self, direction: int):
        """Move one frame in the given direction, or stop at the end.

        :param direction: +1 to move forward, -1 to move backward.
        """
        next_index = self._index + direction
        if next_index < 0 or next_index >= len(self._frames):
            self._set_playing(False)
            return
        self._show_frame(next_index)

    def _on_slider(self, value: float):
        """Handle the frame slider changing.

        :param value: Slider frame value.
        """
        index = int(value)
        if index == self._index:
            return
        # Scrubbing pauses playback so the user keeps control of position.
        if self._playing:
            self._set_playing(False)
        self._show_frame(index)

    def _on_play_clicked(self, _event: MouseEvent):
        """Toggle play and pause.

        :param _event: Matplotlib button click event.
        """
        self._set_playing(not self._playing)

    def _on_direction_clicked(self, _event: MouseEvent):
        """Toggle forward and reverse playback.

        :param _event: Matplotlib button click event.
        """
        self._direction *= -1
        label = "Forward" if self._direction > 0 else "Reverse"
        self._direction_button.label.set_text(label)
        if self._playing:
            self._arm_timer_for_next_step()
        self._fig.canvas.draw_idle()

    def _on_timer(self):
        """Advance one frame when the observation-dt timer fires."""
        if not self._playing:
            return
        self._advance(self._direction)
        if self._playing:
            self._arm_timer_for_next_step()

    def _on_close(self, _event: CloseEvent):
        """Stop the timer when the window closes.

        :param _event: Matplotlib close event.
        """
        self._playing = False
        if self._timer is not None:
            self._timer.stop()

    def show(self):
        """Open the interactive window and block until it is closed."""
        plt.show()


def display_observations(observations: CarObservations) -> None:
    """Show cones and the car on a non-interactive body-frame plot.

    Cones are drawn as coloured ▲ markers. The car is drawn at the origin with a
    velocity arrow along forwards.

    :param observations: Observations for one policy step.
    """
    fig, ax = plt.subplots()
    _draw_observations(ax, observations)
    xlim, ylim = _frame_limits(*_frame_points(observations))
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    fig.tight_layout()
    plt.show()
