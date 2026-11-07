"""Measurements of the real car and camera, and the simulated lab car and lab lane built from them (Tasks A1, A2).

The measurements come from two sources:
- Ciaran's recorded drives in scenarios/data/real_scenarios (the car drove at a fixed drive effort of 0.55 with the
  steering centred, through four lab cone tracks). They are measured here, by the functions below.
- Chris's calibration of car 20 on 5 October (docs/control/car-calibration.md on the control/low-level-sysid
  branch): the curvature each steering action gives, and the speed response's dead time. Those runs are not in this
  repository, so their results are copied into CAR20_* below.

LAB_CAR and LAB_LANE round these numbers into a CarProfile and Lane for the Stage 1 simulation, so the MPC can be
tuned on a simulated car that behaves like ours. test_real_car_measurements.py checks that they still agree with
the recordings, so a new recording that disagrees fails the tests.

Print the measurements beside the shared defaults from the repository root with:
    python3 tests/policy/real_car_measurements.py
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from policy.input_output import CarActions, CarObservations, ConeColour  # noqa: E402
from scenarios.scenario_wrapper import Scenario, ScenarioType  # noqa: E402
from stage1_scenarios import SHARED_DEFAULT_CAR, CarProfile, Lane  # noqa: E402

REAL_SCENARIOS = ("straight_even", "long_track", "messy_cone", "outlier_cone")
"""Ciaran's recorded lab drives."""
CLEAN_SCENARIOS = ("straight_even", "long_track")
"""The recordings with two clean, evenly spaced cone rows, used for the lane size and the steering trim."""

CAR20_MAX_LEFT_CURVATURE_PER_M = 1.0 / 2.1
"""Car 20 turns left on a circle of at least 2.1 m radius: steering saturates beyond -0.75 (Chris, 5 Oct)."""
CAR20_MAX_RIGHT_CURVATURE_PER_M = 0.959 * 0.96
"""Car 20's sharpest right turn: kappa = -0.959 * steer + 0.038 at the largest tested steer, 0.96 (Chris, 5 Oct)."""
CAR20_STRAIGHT_CURVATURE_PER_M = -0.05
"""Car 20 drives a -0.05 1/m curve (to the right) with the steering centred and its default trim (Chris, 5 Oct)."""
CAR20_SPEED_DEAD_TIME_S = 0.2
"""Delay before car 20's speed responds to a drive change (Chris, 5 Oct). Chris measured a 2.7 s time constant at
higher drive efforts; speed_response below measures it for Ciaran's drive effort."""


@dataclass(frozen=True)
class Recording:
    """One policy run of a recorded scenario: from a first policy step to the next one."""
    scenario: str
    steps: list[tuple[CarObservations, CarActions]]


def load_recordings(names: tuple[str, ...] = REAL_SCENARIOS) -> list[Recording]:
    """Loads the recorded scenarios, split into policy runs (a scenario can hold more than one run).

    :param names: Scenario names in scenarios/data/real_scenarios, without _scenario.json.
    :return: The runs, in order.
    """
    recordings = []
    for name in names:
        runs: list[list] = []
        for observations, actions in Scenario.from_name(ScenarioType.REAL, name).steps():
            if observations.policy.is_first_policy_step or not runs:
                runs.append([])
            runs[-1].append((observations, actions))
        recordings += [Recording(name, run) for run in runs]
    return recordings


def _batches(recordings: list[Recording]):
    """Yields each recording's distinct cone batches (a batch can be repeated over policy steps) with its run."""
    for recording in recordings:
        last_stamp = None
        for observations, _ in recording.steps:
            cones = observations.cones
            if cones is None or not cones:
                continue
            stamp = cones.sensor_age.stamp_ns
            if stamp is not None and stamp == last_stamp:
                continue
            last_stamp = stamp
            yield recording, observations


def detection_latency_s(recordings: list[Recording]) -> float:
    """
    :return: Median age of the cone batches when the policy received them, in seconds.
    """
    return float(np.median([float(observations.cones.sensor_age) for _, observations in _batches(recordings)]))


def detection_ranges(recordings: list[Recording]) -> dict[str, float]:
    """
    :return: Percentiles of the detected cones' forwards distance in metres (x_p1, x_p50, x_p95), the nearest one
        (x_min), and the 99th percentile of the absolute bearing in degrees (bearing_p99_deg).
    """
    xs, bearings = [], []
    for _, observations in _batches(recordings):
        for cone in observations.cones:
            xs.append(cone.pos.x)
            bearings.append(abs(math.degrees(math.atan2(cone.pos.y, cone.pos.x))))
    return {"x_min": float(np.min(xs)), "x_p1": float(np.percentile(xs, 1)), "x_p50": float(np.median(xs)),
            "x_p95": float(np.percentile(xs, 95)), "bearing_p99_deg": float(np.percentile(bearings, 99))}


def fit_rows(observations: CarObservations, min_cones: int = 3) -> tuple[float, float, float, np.ndarray] | None:
    """Fits the blue and yellow cones as two parallel straight rows y = intercept + slope * x in base_link.

    :param observations: One step's observations.
    :param min_cones: Fewest cones of each colour for a fit.
    :return: (blue intercept, yellow intercept, shared slope, residuals in metres), or None without enough cones.
    """
    rows = {colour: [(c.pos.x, c.pos.y) for c in observations.cones if c.colour == colour]
            for colour in ConeColour}
    if any(len(points) < min_cones for points in rows.values()):
        return None
    design, ys = [], []
    for column, colour in enumerate((ConeColour.BLUE, ConeColour.YELLOW)):
        for x, y in rows[colour]:
            design.append([1.0 - column, float(column), x])
            ys.append(y)
    design, ys = np.array(design), np.array(ys)
    solution = np.linalg.lstsq(design, ys, rcond=None)[0]
    return float(solution[0]), float(solution[1]), float(solution[2]), ys - design @ solution


def lane_geometry(recordings: list[Recording]) -> dict[str, float]:
    """Measures the lane from the parallel row fits of each cone batch.

    :return: Median lane width in metres (width_m), how often blue is the left row (blue_left_fraction), the
        standard deviation of the cones' distance from their row in metres (row_residual_m, the detector's lateral
        noise plus cone placement), and the median distance between neighbouring detected cones of a row in metres
        (cone_spacing_m).
    """
    widths, residuals, spacings, blue_left = [], [], [], []
    for _, observations in _batches(recordings):
        fit = fit_rows(observations)
        if fit is None:
            continue
        blue, yellow, slope, residual = fit
        widths.append(abs(blue - yellow) / math.hypot(1.0, slope))
        blue_left.append(blue > yellow)
        residuals.extend(residual)
        direction = np.array([1.0, slope]) / math.hypot(1.0, slope)
        for colour in ConeColour:
            along = sorted(np.array([c.pos.x, c.pos.y]) @ direction
                           for c in observations.cones if c.colour == colour)
            if len(along) >= 4:
                spacings.append(float(np.median(np.diff(along))))
    return {"width_m": float(np.median(widths)), "blue_left_fraction": float(np.mean(blue_left)),
            "row_residual_m": float(np.std(residuals)), "cone_spacing_m": float(np.median(spacings))}


def _apparent_heading_deg(recording: Recording) -> float:
    """
    :return: The median heading of the car to the cone rows, as the camera sees it, in degrees.
    """
    headings = [-math.atan(fit[2]) for observations, _ in recording.steps
                if observations.cones and (fit := fit_rows(observations)) is not None]
    return math.degrees(float(np.median(headings)))


def camera_yaw_deg(recording: Recording) -> float:
    """Measures how far left the camera points, from a recording where the car drives along a straight lane.

    The rows' slope gives the car's heading to the lane as the camera sees it. The change in the lane centre's
    sideways distance per metre driven gives the true heading, which does not depend on the camera's direction.
    Their difference is the camera's yaw. It needs a run of a few metres to separate the two.

    :return: The camera's yaw in degrees, positive when it points left of the car's forwards direction.
    """
    distance, distances, apparent, centres = 0.0, [], [], []
    for observations, _ in recording.steps:
        speed = observations.car.wheel_speed
        distance += (0.0 if speed is None else float(speed)) * observations.policy.dt
        if observations.cones is None or not observations.cones:
            continue
        fit = fit_rows(observations)
        if fit is not None:
            blue, yellow, slope, _ = fit
            distances.append(distance - (0.0 if speed is None else float(speed)) * float(observations.cones.sensor_age))
            apparent.append(-math.atan(slope))
            centres.append((blue + yellow) / 2.0 / math.hypot(1.0, slope))
    # The lane centre moves right (its offset falls) as the car heads left
    true_heading = math.atan(-np.polyfit(distances, centres, 1)[0])
    return math.degrees(float(np.median(apparent)) - true_heading)


def speed_response(recording: Recording) -> dict[str, float]:
    """Fits the speed from rest at a fixed drive effort as a first-order response with a dead time:
    speed = steady * (1 - exp(-(t - dead_time) / time_constant)) after dead_time.

    :return: drive (the fixed effort), steady_m_per_s, time_constant_s and dead_time_s (which includes breaking
        away from rest).
    """
    from scipy.optimize import curve_fit

    drives = {actions.drive_action for _, actions in recording.steps}
    if len(drives) != 1:
        raise ValueError("the drive effort was not fixed in this recording")
    times = np.array([o.policy.policy_elapsed_s for o, _ in recording.steps])
    speeds = np.array([0.0 if o.car.wheel_speed is None else float(o.car.wheel_speed) for o, _ in recording.steps])

    def model(t, steady, time_constant, dead_time):
        return np.where(t > dead_time, steady * (1.0 - np.exp(-(t - dead_time) / time_constant)), 0.0)

    (steady, time_constant, dead_time), _ = curve_fit(model, times, speeds, p0=(0.6, 1.5, 0.5),
                                                      bounds=((0.0, 0.05, 0.0), (3.0, 20.0, 3.0)))
    return {"drive": drives.pop(), "steady_m_per_s": float(steady), "time_constant_s": float(time_constant),
            "dead_time_s": float(dead_time)}


# ---------------------------------------------------------------------------------------------------------------
# The simulated lab car and lane, rounded from the measurements above (printed by main)
# ---------------------------------------------------------------------------------------------------------------
LAB_CAR = CarProfile(
    name="lab car (measured)",
    # The detections reach 43 degrees either side (bearing_p99_deg)
    detector_fov_deg=86.0,
    # No cone is detected closer than about 0.4 m ahead (x_min): the camera cannot see the floor below that
    detector_min_range_m=0.4,
    # 95% of detections are within 2.6 m (x_p95), although the long track's farthest cones are seen at 5 m
    detector_range_m=2.5,
    # The cones' distance from their fitted row is 2-4 cm (row_residual_m, which includes cone placement)
    detector_noise_m=0.03,
    # Every batch is about 0.2 s old (detection_latency_s)
    detection_latency_s=0.2,
    max_left_curvature_per_m=round(CAR20_MAX_LEFT_CURVATURE_PER_M, 2),
    max_right_curvature_per_m=round(CAR20_MAX_RIGHT_CURVATURE_PER_M, 2),
    # The long track shows the camera pointing about 7 degrees left (camera_yaw_deg); the shorter recordings agree
    # in direction but are too short to give a size
    camera_yaw_deg=7.0,
    # Chris measured -0.05 on car 20. Ciaran's recordings cannot check it: their heading to the rows includes the
    # camera's yaw, and they are short
    straight_curvature_per_m=CAR20_STRAIGHT_CURVATURE_PER_M,
    # From rest at drive 0.55 the speed reaches 63% of its final value in 1.2-1.8 s (speed_response)
    speed_time_constant_s=1.5,
    speed_dead_time_s=CAR20_SPEED_DEAD_TIME_S,
    rolling_start=False,
)
"""The Stage 1 simulation's car with the measured camera, latency, steering limits, trim and speed response."""

LAB_LANE = Lane(length_m=3.2, width_m=0.8, cone_spacing_m=0.2, left_colour=ConeColour.BLUE)
"""A lane like Ciaran's lab tracks: blue on the left, rows about 0.8 m apart, cones about every 0.2 m, as long as the
long track's first section."""


def measure(recordings: list[Recording] | None = None) -> dict[str, float]:
    """Takes every measurement from the recordings.

    :return: The measurements by name. Units are in the names.
    """
    recordings = load_recordings() if recordings is None else recordings
    clean = [r for r in recordings if r.scenario in CLEAN_SCENARIOS]
    responses = [speed_response(r) for r in recordings]
    long_track = next(r for r in recordings if r.scenario == "long_track")
    return {
        "detection_latency_s": detection_latency_s(recordings),
        **detection_ranges(recordings),
        **lane_geometry(clean),
        "camera_yaw_deg": camera_yaw_deg(long_track),
        "apparent_heading_min_deg": min(_apparent_heading_deg(r) for r in clean),
        "apparent_heading_max_deg": max(_apparent_heading_deg(r) for r in clean),
        "speed_drive": responses[0]["drive"],
        "speed_steady_median_m_per_s": float(np.median([r["steady_m_per_s"] for r in responses])),
        "speed_time_constant_min_s": min(r["time_constant_s"] for r in responses),
        "speed_time_constant_max_s": max(r["time_constant_s"] for r in responses),
        "speed_dead_time_median_s": float(np.median([r["dead_time_s"] for r in responses])),
    }


def format_table(measured: dict[str, float]) -> str:
    """
    :return: A Markdown table of the shared defaults, the measurements and LAB_CAR / LAB_LANE.
    """
    m, shared = measured, SHARED_DEFAULT_CAR
    rows = [
        ("Left row colour", "not agreed (yellow in the Card 2 lanes)",
         f"blue in {100 * m['blue_left_fraction']:.0f}% of batches", LAB_LANE.left_colour.name.lower()),
        ("Lane width [m]", "1.0", f"{m['width_m']:.2f}", f"{LAB_LANE.width_m:g}"),
        ("Cone spacing [m]", "0.5", f"{m['cone_spacing_m']:.2f}", f"{LAB_LANE.cone_spacing_m:g}"),
        ("Camera view [deg]", f"{shared.detector_fov_deg:g}", f"+-{m['bearing_p99_deg']:.0f} (99%)",
         f"{LAB_CAR.detector_fov_deg:g}"),
        ("Nearest detection [m]", "0", f"{m['x_min']:.2f}", f"{LAB_CAR.detector_min_range_m:g}"),
        ("Detection range [m]", f"{shared.detector_range_m:g}", f"{m['x_p95']:.2f} (95%)",
         f"{LAB_CAR.detector_range_m:g}"),
        ("Position noise [m]", f"{shared.detector_noise_m:g}", f"{m['row_residual_m']:.3f} (row residual)",
         f"{LAB_CAR.detector_noise_m:g}"),
        ("Detection latency [s]", "0", f"{m['detection_latency_s']:.2f}", f"{LAB_CAR.detection_latency_s:g}"),
        ("Camera yaw, left [deg]", "0",
         f"{m['camera_yaw_deg']:.1f} (long track); rows look {m['apparent_heading_min_deg']:.0f} to "
         f"{m['apparent_heading_max_deg']:.0f} turned", f"{LAB_CAR.camera_yaw_deg:g}"),
        ("Curvature, steering centred [1/m]", "0", "-0.05 (car 20, Chris)", f"{LAB_CAR.straight_curvature_per_m:g}"),
        ("Largest curvature left / right [1/m]", "3.0 / 3.0 (45 deg)", "car 20 (Chris)",
         f"{LAB_CAR.max_left_curvature_per_m:g} / {LAB_CAR.max_right_curvature_per_m:g}"),
        (f"Speed at drive {m['speed_drive']:g} [m/s]", "-", f"{m['speed_steady_median_m_per_s']:.2f}", "-"),
        ("Speed time constant [s]", "about 0.6 (stand-in)",
         f"{m['speed_time_constant_min_s']:.1f} to {m['speed_time_constant_max_s']:.1f}",
         f"{LAB_CAR.speed_time_constant_s:g}"),
        ("Speed dead time [s]", "0", f"{m['speed_dead_time_median_s']:.1f} from rest; 0.2 moving (Chris)",
         f"{LAB_CAR.speed_dead_time_s:g}"),
    ]
    lines = ["| Quantity | Shared default | Measured | Lab car / lane |", "| --- | --- | --- | --- |"]
    lines += [f"| {' | '.join(row)} |" for row in rows]
    return "\n".join(lines)


if __name__ == "__main__":
    print(format_table(measure()))
