"""Random wide synthetic tracks for keeping to the left or right of the lane (mpc.lane_side), towards the hook turn.

The lab tracks are about 0.8 m wide, where the MPC has no room to move over (docs/mpc-lane-side.md), so the
synthetic tracks of synthetic_tracks.py cannot check lane_side. These tracks are 1.1-1.6 m wide and include
curves, like the hook turn's roads. Everything else is drawn the way synthetic_tracks.py draws it, around the
measured lab car (real_car_measurements.py): cone spacing and colours, the camera's yaw, noise, range and delay,
the speed lag, car 20's steering limits (scaled) and staggered, swapped-colour and outlier cones.

Each track asks for one lane side: "LEFT" or "RIGHT" throughout, or a switch from one side to the other part way
along, as the hook turn will. The car drives through lane detection with curves on, as config/ai4r_policy.yaml
sets it, on the curved-lane simulation (curved_lanes.py). A track is fully set by its seed. None of these tracks
are the unseen cases in unseen/.

Print how the MPC does on the training tracks with:
    python3 tests/policy/wide_tracks.py
Save some tracks as scenario files beside the other synthetic tracks (scenarios/data/synthetic_scenarios) with:
    python3 tests/policy/wide_tracks.py --export 0 1 2 10 22 33 56
and play one in the team's scenario renderer (the cones as the car saw them):
    python3 -m scripts.scenario_renderer show -t synthetic -n wide_w10
"""
from __future__ import annotations

import dataclasses
import json
import math
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import curved_lanes as cl  # noqa: E402
from policy.action_policy.mpc import MpcConfig  # noqa: E402
from policy.action_policy.policy import MpcPolicy  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones  # noqa: E402
from policy.input_output import ConeColour  # noqa: E402
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector  # noqa: E402
from real_car_measurements import LAB_CAR  # noqa: E402
from synthetic_tracks import LAB_RANGES, SPEED_M_PER_S, SYNTHETIC_SCENARIOS_DIR  # noqa: E402

TRAINING_SEEDS = tuple(range(0, 60))
"""Tracks used to choose the lane side settings."""
VALIDATION_SEEDS = tuple(range(2000, 2030))
"""Tracks used only to check the chosen settings, never to choose them."""
SETTLE_M = 1.0
"""Distance from the first cone pair, and after a switch has crossed, before the car is scored on its side."""


@dataclass(frozen=True)
class WideTrackRanges:
    """The ranges each wide track's values are drawn from, uniformly unless stated. Lengths are in metres."""
    lane_width_m: tuple[float, float] = (1.1, 1.6)
    """Wide enough to move over: 0.15-0.40 m of offset at mpc.edge_clearance_m 0.40."""
    shapes: tuple[str, ...] = ("straight", "left", "right", "right", "s_bend")
    """Choices. Right curves are drawn twice as often: the hook turn is a right turn."""
    straight_m: tuple[float, float] = (1.0, 2.0)
    """Length of the straights before and after the curve."""
    left_radius_m: tuple[float, float] = (2.5, 4.0)
    """Car 20 steers left to at most 0.48 1/m (2.1 m radius), less once scaled, so left curves stay wider."""
    right_radius_m: tuple[float, float] = (1.5, 4.0)
    """Car 20 steers right to 0.92 1/m (1.1 m radius)."""
    sweep_deg: tuple[float, float] = (45.0, 90.0)
    side_probabilities: tuple[float, float, float] = (0.35, 0.35, 0.3)
    """Chance of keeping left, keeping right and switching sides part way along."""
    switch_after_m: tuple[float, float] = (1.5, 3.0)
    """Distance driven from the first cone pair before a switching track changes side."""
    straight_curvature_per_m: tuple[float, float] = (-0.03, 0.03)
    """Steering trim left after Car Control's trim (control.steering_trim). Uncompensated trim of -0.05 1/m and
    more loses runs in wide lanes (docs/mpc-lane-side.md), so calibrate the trim before testing on the car."""
    curvature_limit_scale: tuple[float, float] = (0.9, 1.1)
    """Scales car 20's measured curvature limits."""


WIDE_RANGES = WideTrackRanges()
"""The ranges of the wide tracks."""


@dataclass(frozen=True)
class DisturbedLane(cl.CurvedLane):
    """A curved lane with staggered (moved), swapped-colour and outlier cones, as the recorded tracks have."""
    moved: tuple[tuple[int, float, float], ...] = ()
    """(cone index, metres along, metres across) of each moved cone."""
    swapped: tuple[int, ...] = ()
    """Indices of the cones seen in the other colour."""
    outliers: tuple[tuple[float, float, ConeColour], ...] = ()
    """(distance along the centre line, signed metres from it (+ left), colour) of cones beside the lane."""

    def cones(self) -> list[tuple[ConeColour, float, float, float]]:
        cones = super().cones()
        line = self.centre_line()
        other = {ConeColour.BLUE: ConeColour.YELLOW, ConeColour.YELLOW: ConeColour.BLUE}
        for index, along, across in self.moved:
            colour, s, x, y = cones[index]
            heading = float(np.interp(s, line["s"], line["heading"]))
            cones[index] = (colour, s, x + along * math.cos(heading) - across * math.sin(heading),
                            y + along * math.sin(heading) + across * math.cos(heading))
        for index in self.swapped:
            colour, s, x, y = cones[index]
            cones[index] = (other[colour], s, x, y)
        for s, lateral, colour in self.outliers:
            heading = float(np.interp(s, line["s"], line["heading"]))
            x, y = float(np.interp(s, line["s"], line["x"])), float(np.interp(s, line["s"], line["y"]))
            cones.append((colour, s, x - lateral * math.sin(heading), y + lateral * math.cos(heading)))
        return cones


@dataclass(frozen=True)
class WideTrack:
    """One wide track: the lane, the car and where in the lane to drive."""
    seed: int
    case: cl.CurveCase
    side: str
    """"LEFT" or "RIGHT": the side asked for from the start."""
    switch_to: str | None = None
    """The side asked for after switch_after_m, or None to keep side throughout."""
    switch_after_m: float = math.inf
    description: str = ""


def _shape(rng, name: str, ranges: WideTrackRanges) -> tuple:
    def uniform(bounds):
        return round(float(rng.uniform(*bounds)), 2)

    first, last, sweep = uniform(ranges.straight_m), uniform(ranges.straight_m), uniform(ranges.sweep_deg)
    if name == "straight":
        return (("straight", round(first + last + 1.5, 2)),)
    if name == "s_bend":
        radius = uniform(ranges.left_radius_m)
        return (("straight", first), ("arc", round(1 / radius, 4), sweep / 2),
                ("arc", round(-1 / radius, 4), sweep / 2), ("straight", last))
    radius = uniform(ranges.left_radius_m if name == "left" else ranges.right_radius_m)
    return (("straight", first), ("arc", round((1 if name == "left" else -1) / radius, 4), sweep),
            ("straight", last))


def side_target_m(side: str, width_m: float, edge_clearance_m: float = 0.40) -> float:
    """
    :return: Where the car should drive on a straight for a lane side: edge_clearance_m from that edge, or the
        centre if the lane is too narrow, in metres from the centre line, positive to the left.
    """
    room = max(0.0, width_m / 2.0 - edge_clearance_m)
    return {"LEFT": room, "RIGHT": -room}[side]


def wide_track(seed: int, ranges: WideTrackRanges = WIDE_RANGES) -> WideTrack:
    """Draws one wide track.

    :param seed: Sets every random value of the track, and its detector noise.
    :param ranges: The ranges to draw from.
    :return: The track.
    """
    rng = np.random.default_rng(seed)
    lab = LAB_RANGES

    def uniform(bounds):
        return float(rng.uniform(*bounds))

    width = round(uniform(ranges.lane_width_m), 3)
    spacing = float(rng.choice(lab.cone_spacing_m))
    left = ConeColour.BLUE if rng.random() < lab.blue_left_probability else ConeColour.YELLOW
    shape_name = str(rng.choice(ranges.shapes))
    pieces = _shape(rng, shape_name, ranges)
    side_kind = int(rng.choice(3, p=ranges.side_probabilities))
    side = "LEFT" if (side_kind == 0 or (side_kind == 2 and rng.random() < 0.5)) else "RIGHT"
    switch_to, switch_after = None, math.inf
    if side_kind == 2:
        switch_to = "RIGHT" if side == "LEFT" else "LEFT"
        switch_after = round(uniform(ranges.switch_after_m), 2)
        # Room to cross at 0.2 m per metre (mpc.offset_change_per_m) and settle before the end
        crossing = 2 * abs(side_target_m(side, width)) / 0.2
        length = cl.CurvedLane(pieces=pieces, width_m=width).centre_line()["s"][-1]
        extra = switch_after + crossing + SETTLE_M + 0.5 - length
        if extra > 0:
            pieces += (("straight", round(extra, 2)),)
    scale = uniform(ranges.curvature_limit_scale)
    car = dataclasses.replace(
        LAB_CAR, name="synthetic lab car",
        camera_yaw_deg=round(uniform(lab.camera_yaw_deg), 2),
        detector_noise_m=round(uniform(lab.detector_noise_m), 4),
        detector_range_m=round(uniform(lab.detector_range_m), 2),
        detector_min_range_m=round(uniform(lab.detector_min_range_m), 2),
        detection_latency_s=float(rng.choice(lab.detection_latency_s)),
        speed_time_constant_s=round(uniform(lab.speed_time_constant_s), 2),
        straight_curvature_per_m=round(uniform(ranges.straight_curvature_per_m), 3),
        max_left_curvature_per_m=round(LAB_CAR.max_left_curvature_per_m * scale, 3),
        max_right_curvature_per_m=round(LAB_CAR.max_right_curvature_per_m * scale, 3),
        calibrated_camera_yaw_deg=0.0, rolling_start=False,
    )
    plain = cl.CurvedLane(pieces=pieces, width_m=width, cone_spacing_m=spacing, left_colour=left)
    count = len(plain.cones())
    length = plain.centre_line()["s"][-1]
    messy = rng.random() < lab.messy_probability
    moved = tuple((index, round(float(rng.normal(0.0, lab.messy_offset_m)), 3),
                   round(float(rng.normal(0.0, lab.messy_offset_m)), 3)) for index in range(count)) if messy else ()
    swapped = tuple(index for index in range(count) if rng.random() < lab.wrong_colour_probability)
    outliers = []
    if rng.random() < lab.outlier_probability:
        for _ in range(int(rng.integers(1, 4))):
            sign = 1.0 if rng.random() < 0.5 else -1.0
            outliers.append((round(uniform((0.5, length)), 2),
                             round(sign * (width / 2 + uniform(lab.outlier_gap_m)), 2),
                             ConeColour(int(rng.integers(0, 2)))))
    lane = DisturbedLane(pieces=pieces, width_m=width, cone_spacing_m=spacing, left_colour=left,
                         moved=moved, swapped=swapped, outliers=tuple(outliers))
    case = cl.CurveCase(lane, speed_m_per_s=SPEED_M_PER_S, start_left_m=round(uniform(lab.start_left_m), 3),
                        start_heading_deg=round(uniform(lab.start_heading_deg), 2), car=car, seed=seed)
    asked = f"keep {side.lower()}" if switch_to is None else \
        f"{side.lower()} then {switch_to.lower()} after {switch_after:g} m"
    features = [f"{width:.2f} m lane", lane.name, asked, f"cones every {spacing:g} m", f"{left.name.lower()} left"]
    features += ["staggered"] * messy + [f"{len(swapped)} swapped"] * bool(swapped)
    features += [f"{len(outliers)} outliers"] * bool(outliers)
    return WideTrack(seed, case, side, switch_to, switch_after, ", ".join(features))


def wide_tracks(seeds: tuple[int, ...], ranges: WideTrackRanges = WIDE_RANGES) -> list[WideTrack]:
    """
    :return: The wide track of each seed.
    """
    return [wide_track(seed, ranges) for seed in seeds]


class SideSchedule:
    """Switches the policy to a track's second lane side after a distance driven (wheel speed times dt), as the hook
    turn's manoeuvre will set MpcPolicy.lane_side between steps. The run starts at the policy's mpc.lane_side."""

    def __init__(self, policy, track: WideTrack):
        """
        :param policy: An MpcPolicy whose mpc.lane_side is the track's first side.
        :param track: The track, for its second side and switch distance.
        """
        self.policy, self.track = policy, track
        self.driven_m = 0.0

    def run_policy(self, observations, lanes):
        self.driven_m += float(observations.car.wheel_speed) * observations.policy.dt
        if self.track.switch_to is not None and self.driven_m >= cl.RUN_IN_M + self.track.switch_after_m:
            self.policy.lane_side = self.track.switch_to
        return self.policy.run_policy(observations, lanes)


def lane_detection():
    """Lane detection with curves on, as config/ai4r_policy.yaml sets it, after the cone filter.

    :return: observations -> LaneDetection or None.
    """
    filter_config = ConeFilterConfig(max_forward_m=2.5, check_straight_rows=False)
    detector = LaneDetector(LaneDetectionConfig(left_colour="AUTO", max_curvature_per_m=2.0))
    return lambda observations: detector.detect(
        dataclasses.replace(observations, cones=filter_cones(observations.cones, filter_config)))


def run_track(track: WideTrack, config=None) -> tuple[cl.CurveResult, dict[str, float]]:
    """Drives the MPC along a wide track through lane detection, asking for the track's lane side.

    :param track: The track.
    :param config: The MpcConfig; the defaults if None.
    :return: The run and its side scores: side_error_median_m and side_error_p90_m, the distance from the line the
        side asks for (side_target_m) over the scored samples, and scored_m, the distance scored. A sample is
        scored on the lane after SETTLE_M, before the last 0.5 m, and not while crossing to the other side.
    """
    config = config or MpcConfig()
    policy = MpcPolicy(dataclasses.replace(config, lane_side=track.side))
    result = cl.run_curve(track.case, SideSchedule(policy, track), lane_detection())
    lane = track.case.lane
    line = lane.centre_line()
    centre = cl._CentreLine(line)
    s = np.array([centre.locate(x, y)[0] for x, y in zip(result.x, result.y)])
    first = side_target_m(track.side, lane.width_m, config.edge_clearance_m)
    target = np.full(len(s), first)
    scored = (s > SETTLE_M) & (s < line["s"][-1] - 0.5)
    if track.switch_to is not None:
        second = side_target_m(track.switch_to, lane.width_m, config.edge_clearance_m)
        crossed = track.switch_after_m + abs(second - first) / config.offset_change_per_m + SETTLE_M
        target[s >= track.switch_after_m] = second
        scored &= (s < track.switch_after_m) | (s > crossed)
    error = np.abs(result.lateral_m - target)[scored]
    ds = np.abs(np.diff(s, prepend=s[0]))
    scores = {"side_error_median_m": float(np.median(error)) if error.size else math.nan,
              "side_error_p90_m": float(np.percentile(error, 90)) if error.size else math.nan,
              "scored_m": float(ds[scored].sum())}
    return result, scores


def scenario_document(track: WideTrack, result: cl.CurveResult) -> dict:
    """Converts a run into a scenario document in the format of scenarios/data/schemas/car_scenario_data_schema.json.

    :param track: The track.
    :param result: Its run, with the recorded steps.
    :return: The JSON-ready document.
    """
    car = track.case.car
    description = (f"Wide synthetic track (tests/policy/wide_tracks.py, seed {track.seed}) for mpc.lane_side: "
                   f"{track.description}. Start {track.case.start_left_m:+.3f} m left, "
                   f"{track.case.start_heading_deg:+.1f} deg. Simulated lab car: camera yaw {car.camera_yaw_deg} "
                   f"deg, noise {car.detector_noise_m} m, range {car.detector_min_range_m}-{car.detector_range_m} m, "
                   f"latency {car.detection_latency_s} s, trim {car.straight_curvature_per_m:+g} 1/m. Actions are the "
                   f"simulated drive and steering of the MPC at the time of export.")
    return {"name": f"Synthetic track W{track.seed}", "description": description,
            "test_data": [{"observations": observations.serialise(), "actions": actions.serialise()}
                          for observations, actions in result.steps]}


def export(seeds: tuple[int, ...], folder: Path = SYNTHETIC_SCENARIOS_DIR, clean: bool = True) -> list[Path]:
    """Runs the current MPC on wide tracks and saves each run as a scenario file.

    :param seeds: The tracks' seeds.
    :param folder: Folder for wide_w<seed>_scenario.json.
    :param clean: Save the clean tracks (clean_track), so each scenario shows the lane side rather than lane
        detection losing the lane and the car stopping.
    :return: The saved files.
    """
    folder.mkdir(parents=True, exist_ok=True)
    paths = []
    for track in wide_tracks(seeds):
        track = clean_track(track) if clean else track
        result, _ = run_track(track)
        path = folder / f"wide_w{track.seed}_scenario.json"
        path.write_text(json.dumps(scenario_document(track, result), indent=2) + "\n")
        paths.append(path)
    return paths


def clean_track(track: WideTrack) -> WideTrack:
    """
    :return: The same track and car with no swapped-colour or outlier cones, which lane detection often loses the
        lane to (docs/mpc-lane-side.md). Staggered cones stay.
    """
    lane = dataclasses.replace(track.case.lane, swapped=(), outliers=())
    features = [f for f in track.description.split(", ") if not f.endswith((" swapped", " outliers"))]
    return dataclasses.replace(track, case=dataclasses.replace(track.case, lane=lane),
                               description=", ".join(features + ["no swapped or outlier cones"]))


def outcome(result: cl.CurveResult) -> str:
    """
    :return: "kept"; "stopped" if the car stopped short without touching a cone or leaving the lane, as it does when
        lane detection reports no lane; otherwise "hit" (touched a cone or left the lane).
    """
    if result.lane_kept:
        return "kept"
    if result.metrics["cones_touched"] == 0 and result.metrics["max_outside_m"] <= 0.0:
        return "stopped"
    return "hit"


def summarise(seeds: tuple[int, ...]) -> list[dict]:
    """
    :return: One row per track: its seed and description; on the clean track (clean_track) the outcome and side
        scores; on the track as drawn the outcome with the lane side and, for comparison, centred as on dev.
    """
    rows = []
    for track in wide_tracks(seeds):
        clean, scores = run_track(clean_track(track))
        drawn, _ = run_track(track)
        centred = cl.run_curve(track.case, MpcPolicy(MpcConfig()), lane_detection())
        rows.append({"seed": track.seed, "track": track.description, "clean": outcome(clean), **scores,
                     "drawn": outcome(drawn), "drawn_centred": outcome(centred)})
    return rows


def main():
    import argparse
    from collections import Counter

    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--export", type=int, nargs="+", metavar="SEED", help="save these tracks as scenarios")
    parser.add_argument("--validation", action="store_true", help="run the validation tracks, not the training ones")
    parser.add_argument("--show", type=int, nargs="*", metavar="SEED", help="print these tracks' settings")
    args = parser.parse_args()
    if args.export:
        for path in export(tuple(args.export)):
            print(path)
        return
    if args.show is not None:
        for seed in args.show:
            track = wide_track(seed)
            print(f"W{seed}: {track.description}; start {track.case.start_left_m:+.3f} m, "
                  f"{track.case.start_heading_deg:+.1f} deg; {track.case.car}")
        return
    rows = summarise(VALIDATION_SEEDS if args.validation else TRAINING_SEEDS)
    print("| track | clean: outcome | median side error [m] | 90th pct. [m] | as drawn: lane side | as drawn: centred "
          "| description |")
    print("|---|---|---|---|---|---|---|")
    for row in rows:
        print(f"| W{row['seed']} | {row['clean']} | {row['side_error_median_m']:.3f} | {row['side_error_p90_m']:.3f} | "
              f"{row['drawn']} | {row['drawn_centred']} | {row['track']} |")
    errors = [row["side_error_median_m"] for row in rows]
    for column in ("clean", "drawn", "drawn_centred"):
        counts = Counter(row[column] for row in rows)
        print(f"{column}: " + ", ".join(f"{name} {counts[name]}/{len(rows)}" for name in ("kept", "stopped", "hit")))
    print(f"Clean tracks' median side error {np.nanmedian(errors):.3f} m, worst track {np.nanmax(errors):.3f} m")


if __name__ == "__main__":
    main()
