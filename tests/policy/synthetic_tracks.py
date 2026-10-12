"""Random synthetic Stage 1 tracks around the measured lab car and lane, for tuning the MPC (Task A4).

Ciaran's four recorded tracks are too few to tune on, and the Card 2 tuning cases use the shared defaults rather
than our car. Each synthetic track draws its lane, start, car and disturbances from ranges around the measurements
in real_car_measurements.py, like the four recorded tracks: clean rows, staggered (moved) cones, missed and
swapped-colour cones and outlier cones beside the lane. The car's values also vary (domain randomisation), so a
design tuned here does not depend on one exact measurement, such as the camera yaw left after its correction.

A track is fully set by its seed, so a track set is a list of seeds. The training and validation seeds are
separate, and none of these tracks are the unseen cases in unseen/.

Save some tracks as scenario files beside Ciaran's recordings (scenarios/data/synthetic_scenarios), each with the
current MPC's drive, with:
    python3 tests/policy/synthetic_tracks.py --export 8
"""
from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from policy.input_output import ConeColour  # noqa: E402
from real_car_measurements import LAB_CAR  # noqa: E402
from stage1_scenarios import Disturbances, Lane, Stage1Case  # noqa: E402

TRAINING_SEEDS = tuple(range(0, 100))
"""Tracks used to choose the MPC's settings."""
VALIDATION_SEEDS = tuple(range(1000, 1050))
"""Tracks used only to check the chosen settings, never to choose them."""
SPEED_M_PER_S = 0.5
"""Requested speed on every track: the speed Chris's speed controller was checked at on car 20."""
SYNTHETIC_SCENARIOS_DIR = Path(__file__).resolve().parents[2] / "scenarios" / "data" / "synthetic_scenarios"
"""Folder for synthetic tracks saved as scenario files."""


@dataclass(frozen=True)
class TrackRanges:
    """The ranges each synthetic track's values are drawn from, uniformly unless stated. Lengths are in metres."""
    lane_width_m: tuple[float, float] = (0.70, 0.95)
    """Measured 0.73-0.85 m on the recorded tracks."""
    cone_spacing_m: tuple[float, ...] = (0.2, 0.25, 0.3)
    """Choices; measured 0.2 m."""
    lane_length_m: tuple[float, float] = (2.4, 6.0)
    """Rounded to whole cone spacings. The recorded tracks are 1.6-3.4 m; Stage 1's drawn lanes are 6-8 m."""
    blue_left_probability: float = 0.8
    """The lab tracks have blue on the left; the rest have yellow, so a design cannot rely on the side."""
    start_left_m: tuple[float, float] = (-0.15, 0.15)
    start_heading_deg: tuple[float, float] = (-10.0, 10.0)
    camera_yaw_deg: tuple[float, float] = (-3.0, 3.0)
    """True camera yaw, with the MPC correcting none (mpc.camera_yaw_offset_deg 0.0). The recordings showed 6-10
    degrees with the pan never commanded; the policy now holds the pan at zero, so this is the error left after
    that, which nobody has measured yet."""
    detector_noise_m: tuple[float, float] = (0.02, 0.045)
    detector_range_m: tuple[float, float] = (1.8, 3.0)
    detector_min_range_m: tuple[float, float] = (0.35, 0.5)
    detection_latency_s: tuple[float, ...] = (0.2, 0.2, 0.3)
    """Choices; measured 0.2 s, sometimes a batch later."""
    speed_time_constant_s: tuple[float, float] = (1.2, 2.7)
    """1.2-1.8 s from Ciaran's recordings, 2.7 s from Chris's."""
    straight_curvature_per_m: tuple[float, float] = (-0.09, 0.0)
    curvature_limit_scale: tuple[float, float] = (0.85, 1.15)
    """Scales car 20's measured curvature limits, for another car or steering trim."""
    messy_probability: float = 0.3
    """Chance of a staggered track like messy_cone, where every cone is moved."""
    messy_offset_m: float = 0.03
    """Standard deviation of each moved cone's offset along and across the lane."""
    missed_cone_probability: float = 0.05
    """Chance of each cone not being detected."""
    wrong_colour_probability: float = 0.02
    """Chance of each cone being detected with the other colour."""
    outlier_probability: float = 0.4
    """Chance of a track with 1-3 outlier cones beside the lane, like outlier_cone."""
    outlier_gap_m: tuple[float, float] = (0.25, 0.6)
    """How far outside a row an outlier cone is."""


LAB_RANGES = TrackRanges()
"""The ranges around the measured lab car and lane."""


def synthetic_case(seed: int, ranges: TrackRanges = LAB_RANGES) -> Stage1Case:
    """Draws one synthetic track.

    :param seed: Sets every random value of the track, and its detector noise.
    :param ranges: The ranges to draw from.
    :return: The track as a Stage 1 case named S<seed>, in the 'synthetic' case set.
    """
    rng = np.random.default_rng(seed)

    def uniform(bounds):
        return float(rng.uniform(*bounds))

    spacing = float(rng.choice(ranges.cone_spacing_m))
    gaps = max(2, round(uniform(ranges.lane_length_m) / spacing))
    left = ConeColour.BLUE if rng.random() < ranges.blue_left_probability else ConeColour.YELLOW
    lane = Lane(length_m=round(gaps * spacing, 6), width_m=round(uniform(ranges.lane_width_m), 3),
                cone_spacing_m=spacing, left_colour=left)
    scale = uniform(ranges.curvature_limit_scale)
    car = dataclasses.replace(
        LAB_CAR, name="synthetic lab car",
        camera_yaw_deg=round(uniform(ranges.camera_yaw_deg), 2),
        detector_noise_m=round(uniform(ranges.detector_noise_m), 4),
        detector_range_m=round(uniform(ranges.detector_range_m), 2),
        detector_min_range_m=round(uniform(ranges.detector_min_range_m), 2),
        detection_latency_s=float(rng.choice(ranges.detection_latency_s)),
        speed_time_constant_s=round(uniform(ranges.speed_time_constant_s), 2),
        straight_curvature_per_m=round(uniform(ranges.straight_curvature_per_m), 3),
        max_left_curvature_per_m=round(LAB_CAR.max_left_curvature_per_m * scale, 3),
        max_right_curvature_per_m=round(LAB_CAR.max_right_curvature_per_m * scale, 3),
        calibrated_camera_yaw_deg=0.0,
    )
    cones = [(colour, index) for colour in ConeColour for index in range(lane.cones_per_row)]
    messy = rng.random() < ranges.messy_probability
    moved = tuple((colour, index, round(float(rng.normal(0.0, ranges.messy_offset_m)), 3),
                   round(float(rng.normal(0.0, ranges.messy_offset_m)), 3)) for colour, index in cones) if messy else ()
    missed = tuple(cone for cone in cones if rng.random() < ranges.missed_cone_probability)
    wrong = tuple(cone for cone in cones if cone not in missed and rng.random() < ranges.wrong_colour_probability)
    extras = []
    if rng.random() < ranges.outlier_probability:
        for _ in range(int(rng.integers(1, 4))):
            side = 1.0 if rng.random() < 0.5 else -1.0
            extras.append((round(uniform((0.5, lane.length_m)), 2),
                           round(side * (lane.width_m / 2 + uniform(ranges.outlier_gap_m)), 2),
                           ConeColour(int(rng.integers(0, 2)))))
    disturbances = Disturbances(moved_cones=moved, missed_cones=missed, wrong_colour_cones=wrong,
                                extra_objects=tuple(extras))
    features = [f"{lane.width_m:.2f} m lane", f"cones every {spacing:g} m", f"{left.name.lower()} left"]
    features += ["staggered"] * messy + [f"{len(missed)} missed"] * bool(missed)
    features += [f"{len(wrong)} swapped"] * bool(wrong) + [f"{len(extras)} outliers"] * bool(extras)
    return Stage1Case(f"S{seed}", "synthetic", ", ".join(features),
                      start_left_m=round(uniform(ranges.start_left_m), 3),
                      start_heading_deg=round(uniform(ranges.start_heading_deg), 2),
                      speed_m_per_s=SPEED_M_PER_S, disturbances=disturbances, lane=lane, seed=seed, car=car)


def synthetic_cases(seeds: tuple[int, ...], ranges: TrackRanges = LAB_RANGES) -> list[Stage1Case]:
    """
    :return: The synthetic track of each seed.
    """
    return [synthetic_case(seed, ranges) for seed in seeds]


def scenario_document(result, description: str | None = None) -> dict:
    """Converts a run into a scenario document in the format of scenarios/data/schemas/car_scenario_data_schema.json.

    :param result: A Stage1Result with its recorded steps.
    :param description: The scenario's description; the case's description and settings if None.
    :return: The JSON-ready document.
    """
    case = result.case
    if description is None:
        description = (f"Synthetic Stage 1 track (tests/policy/synthetic_tracks.py, seed {case.seed}): "
                       f"{case.description}. {case.lane.length_m:g} m long; start {case.start_left_m:+.3f} m left, "
                       f"{case.start_heading_deg:+.1f} deg. Simulated lab car: camera yaw {case.car.camera_yaw_deg} "
                       f"deg, noise {case.car.detector_noise_m} m, range {case.car.detector_min_range_m}-"
                       f"{case.car.detector_range_m} m, latency {case.car.detection_latency_s} s. Actions are the "
                       f"simulated drive and steering of the MPC at the time of export.")
    return {"name": f"Synthetic track S{case.seed}", "description": description,
            "test_data": [{"observations": observations.serialise(), "actions": actions.serialise()}
                          for observations, actions in result.steps]}


def export(seeds: tuple[int, ...], folder: Path = SYNTHETIC_SCENARIOS_DIR) -> list[Path]:
    """Runs the current MPC on synthetic tracks and saves each run as a scenario file.

    :param seeds: The tracks' seeds.
    :param folder: Folder for synthetic_s<seed>_scenario.json.
    :return: The saved files.
    """
    from stage1_scenarios import run_case, shipped_mpc_policy

    folder.mkdir(parents=True, exist_ok=True)
    paths = []
    for case in synthetic_cases(seeds):
        result = run_case(case, shipped_mpc_policy(case))
        path = folder / f"synthetic_s{case.seed}_scenario.json"
        path.write_text(json.dumps(scenario_document(result), indent=2) + "\n")
        paths.append(path)
    return paths


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--export", type=int, metavar="N", help="save the first N training tracks as scenarios")
    parser.add_argument("--show", type=int, nargs="*", metavar="SEED", help="print these tracks' settings")
    args = parser.parse_args()
    if args.export:
        for path in export(TRAINING_SEEDS[:args.export]):
            print(path)
    for seed in args.show or ():
        case = synthetic_case(seed)
        print(f"S{seed}: {case.description}; start {case.start_left_m:+.3f} m, {case.start_heading_deg:+.1f} deg; "
              f"{case.lane.name}; {case.car}")


if __name__ == "__main__":
    main()
