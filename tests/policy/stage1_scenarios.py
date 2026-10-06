"""Stage 1 test scenarios (Card 2 in docs/movement-policy-tasks.md): straight lanes with even cones.

This module builds Stage 1 lanes of different sizes in dream-gym, runs a movement policy on one test case at a time
and checks the result against the pass thresholds in docs/stage1-test-cases.md. The case tables themselves are in
stage1_tuning_cases.py and unseen/stage1_unseen_cases.py.

The simulator's ground truth is only used to place the car, to create the disturbances and to score the run. The
policy only receives cone detections, the wheel speed and timing, as on the car.

Any policy with run_policy(observations, lanes) -> list[Instruction] can be run, so MPC and RL designs are tested
on the same cases (Card 1.3). Its DriveCommand is converted into drive and steering actions by a simple stand-in
for Car Control (see command_to_action), until Car Control's conversion is available.

Print the metrics table for every tuning case from the repository root with:
    python3 tests/policy/stage1_scenarios.py
Add --unseen for the unseen cases (only for the final comparison, M1), and --figures DIR to save trajectories.
--layout docs/figures redraws the printable floor drawings and the simulated lanes beside them.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

# Imported here rather than inside functions, so every policy object comes from the same import of the policy
# package even if another test re-imports it (test_src_compiles.py does)
import dataclasses  # noqa: E402
from policy.action_policy.mpc import MpcConfig  # noqa: E402
from policy.action_policy.policy import MpcPolicy  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones  # noqa: E402
from policy.control.actions import DriveCommand  # noqa: E402
from policy.input_output import (  # noqa: E402
    CarObservations, ConeBatch, ConeColour, ConeDetection, ObservedCarState, PolicyState, Position, SensorAge,
    WheelSpeed)

# ---------------------------------------------------------------------------------------------------------------
# Lane layout (Card 2.2). The world frame has its origin on the start line at the lane centre, +x along the lane
# and +y to the left. The same numbers are used for the printed drawings in docs/.
# ---------------------------------------------------------------------------------------------------------------
LEFT_COLOUR = ConeColour.YELLOW
"""Row on the left (+y). Not agreed yet (Task 1.4 with Long); the MPC's reference does not assume it."""
RUN_UP_M = 1.0
"""Floor before the start line where the real car gets up to speed from rest."""
STOP_ZONE_M = (0.0, 0.5)
"""The car's centre must stop between these distances in metres past the last cone pair (pass rule)."""
SAFETY_RUN_OUT_M = 1.0
"""Clear floor after the stop zone on the real lane, in case the car overruns."""
SIM_RUN_OUT_M = 4.0
"""Road without cones after the last cone pair in simulation, so the car can overrun the stop zone."""
CONE_RADIUS_M = 0.05
"""Radius of a cone's base in metres, for the cone-touched check. Replace with the lab cones' measured size."""
FLOOR_MARGIN_M = 0.3
"""Clear floor beside each cone row in metres, for the floor area."""


@dataclass(frozen=True)
class Lane:
    """The size of a straight Stage 1 lane: two parallel cone rows, with a cone pair on the start line."""
    length_m: float = 6.0
    """Distance from the first to the last cone pair in metres."""
    width_m: float = 1.0
    """Distance between the two cone rows in metres (shared default)."""
    cone_spacing_m: float = 0.5
    """Distance between neighbouring cones in a row in metres (shared default)."""

    def __post_init__(self):
        for name in ("length_m", "width_m", "cone_spacing_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be positive and finite, not {value}")
        gaps = self.length_m / self.cone_spacing_m
        if abs(gaps - round(gaps)) > 1e-9:
            raise ValueError("length_m must be a whole number of cone spacings, so a cone pair ends the lane")

    @property
    def cones_per_row(self) -> int:
        """Number of cones in each row, including the pairs on the start line and at the end."""
        return round(self.length_m / self.cone_spacing_m) + 1

    @property
    def name(self) -> str:
        """Short name for file names and tables, such as 6.0x1.0m_0.5."""
        return f"{self.length_m:g}x{self.width_m:g}m_{self.cone_spacing_m:g}"

    @property
    def floor_area_m(self) -> tuple[float, float]:
        """Length and width of floor the real lane needs in metres, from the run-up to the safety run-out."""
        return (RUN_UP_M + self.length_m + STOP_ZONE_M[1] + SAFETY_RUN_OUT_M,
                self.width_m + 2 * (CONE_RADIUS_M + FLOOR_MARGIN_M))


STAGE1_LANE = Lane()
"""The main Stage 1 lane from the task card: 6 m long, 1.0 m wide, cones every 0.5 m."""

# ---------------------------------------------------------------------------------------------------------------
# Car and detector (Card A2 shared defaults, from dream-gym's examples/cone_following_with_simple_policy.py)
# ---------------------------------------------------------------------------------------------------------------
WHEELBASE_M = 0.33
"""Front to rear axle distance in metres."""
CAR_MASS_KG = 3.0
"""Car mass in kg."""
MAX_STEERING_RAD = math.radians(45.0)
"""Largest front wheel angle in radians (shared default, likely larger than the real car's)."""
CAR_FRONT_M, CAR_REAR_M, CAR_WIDTH_M = 0.6 * WHEELBASE_M * 1.5, 0.4 * WHEELBASE_M * 1.5, 0.25
"""Body extents from the centre of mass in metres, used for the cone-touched and in-lane checks."""
DETECTOR_FOV_DEG = 80.0
"""Camera view width in degrees."""
DETECTOR_RANGE_M = 4.0
"""Furthest forwards distance a cone is detected at in metres."""
DETECTOR_NOISE_M = 0.01
"""Standard deviation of the detected cone positions in metres, without the noise disturbance."""
ENV_STEP_S = 0.05
"""Simulation step in seconds."""
POLICY_PERIOD_S = 0.1
"""Time between policy steps in seconds, i.e. one cone detection batch per step. Replace with Task 1.5's rate."""
TIME_BUDGET_S = 0.05
"""Calculation time allowed per policy step in seconds (shared default, Task 1.5)."""
MAX_DURATION_S = 20.0
"""A run ends after this long even if the car has not stopped."""
STOPPED_SPEED_M_PER_S = 0.02
"""Speeds below this count as stopped."""
FLOOR_CONDITIONS = ("dry", "wet", "snow", "ice")
"""dream-gym tyre presets usable for the floor disturbance."""


@dataclass(frozen=True)
class Disturbances:
    """Problems added to a case (Card 2.4).

    Cones are named (colour, index), where index 0 is the cone pair on the start line and the lane's
    cones_per_row - 1 is the last pair, which gives the same cone on the floor and in simulation.
    """
    noise_m: float = DETECTOR_NOISE_M
    """Standard deviation of the detected forwards and leftwards cone positions in metres."""
    delay_s: float = 0.0
    """How old each detection batch is when the policy receives it, in seconds. A multiple of POLICY_PERIOD_S."""
    missed_cones: tuple[tuple[ConeColour, int], ...] = ()
    """Cones that are never detected, like a missing or fallen cone, or glare on part of a row."""
    wrong_colour_cones: tuple[tuple[ConeColour, int], ...] = ()
    """Cones detected with the other colour, like a misclassification or a swapped cone."""
    extra_objects: tuple[tuple[float, float, ConeColour], ...] = ()
    """Objects that are not lane cones but are detected as cones, as world (x, y, colour)."""
    detector_range_m: float = DETECTOR_RANGE_M
    """Furthest forwards distance a cone is detected at in metres, shorter for glare or poor lighting."""
    car_mass_kg: float = CAR_MASS_KG
    """Car mass in kg, heavier for a battery or payload."""
    floor: str | None = None
    """Floor grip: None for the nominal kinematic car, or one of FLOOR_CONDITIONS for dream-gym's dynamic car with
    that tyre grip (it uses the dynamic model above 0.2-0.4 m/s)."""

    def __post_init__(self):
        if not (math.isfinite(self.noise_m) and self.noise_m >= 0.0):
            raise ValueError(f"noise_m must be finite and at least 0, not {self.noise_m}")
        steps = self.delay_s / POLICY_PERIOD_S
        if not (math.isfinite(steps) and steps >= 0.0 and abs(steps - round(steps)) < 1e-9):
            raise ValueError(f"delay_s must be a non-negative multiple of {POLICY_PERIOD_S} s, not {self.delay_s}")
        for name in ("detector_range_m", "car_mass_kg"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be positive and finite, not {value}")
        if self.floor is not None and self.floor not in FLOOR_CONDITIONS:
            raise ValueError(f"floor must be None or one of {FLOOR_CONDITIONS}, not {self.floor!r}")


@dataclass(frozen=True)
class Stage1Case:
    """One test case: the lane, where the car starts, how fast it is asked to drive and what goes wrong."""
    name: str
    """Case id, such as T1 or U1."""
    case_set: str
    """'tuning' for design and tuning, or 'unseen' for the final comparison only."""
    description: str
    """Short description for tables."""
    start_left_m: float = 0.0
    """Start position of the car's centre left of the lane centre in metres; negative is to the right."""
    start_heading_deg: float = 0.0
    """Start heading left of the lane direction in degrees; negative points right."""
    speed_m_per_s: float = 1.0
    """Speed the policy is asked to drive at in m/s. The car also starts at this speed (a rolling start)."""
    disturbances: Disturbances = field(default_factory=Disturbances)
    """Problems added to the case."""
    lane: Lane = STAGE1_LANE
    """The lane's size."""
    seed: int = 0
    """Random seed for the detector noise, so a case gives the same result every time."""

    def __post_init__(self):
        for _, index in self.disturbances.missed_cones + self.disturbances.wrong_colour_cones:
            if not 0 <= index < self.lane.cones_per_row:
                raise ValueError(f"{self.name}: cone index {index} is outside 0..{self.lane.cones_per_row - 1}")


@dataclass(frozen=True)
class Thresholds:
    """Pass thresholds on the Stage 1 metrics (Card 2.1). Every one must hold for a case to pass.

    PROVISIONAL until the Card 1.1 requirement and metric table is agreed; keep the two in step.
    """
    max_cones_touched: int = 0
    """R2 no cone touched."""
    max_body_outside_lane_m: float = 0.0
    """R1 stay in lane: how far any body corner may go past a cone row while the car is beside the cones."""
    max_lateral_error_at_lane_end_m: float = 0.10
    """R1 the car has settled near the lane centre by the last cone pair (or where it stopped, if earlier)."""
    max_mean_speed_error_m_per_s: float = 0.10
    """R3 hold speed: mean absolute speed error while cruising beside the cones, until the first stop request."""
    max_rms_curvature_rate_per_m_s: float = 1.0
    """R4 smooth: RMS rate of change of the requested curvature in 1/(m s).

    About a fifth of the simulated steering servo's rate limit (90 deg/s at the shared wheelbase).
    """
    stop_zone_m: tuple[float, float] = STOP_ZONE_M
    """R5 stop at the lane end: allowed stop position past the last cone pair in metres."""
    max_fallback_count: int = 0
    """R6 real time: steps where the solver did not converge."""
    max_p99_calc_time_s: float = TIME_BUDGET_S
    """R6 real time: 99th-percentile calculation time per policy step. Only meaningful on the car computer."""


@dataclass
class Stage1Result:
    """What happened in one run of a case, with its metrics and pass/fail result."""
    case: Stage1Case
    time_s: np.ndarray
    """Time of each simulation step in seconds."""
    x_m: np.ndarray
    """True distance of the car's centre along the lane from the start line in metres."""
    y_m: np.ndarray
    """True distance of the car's centre left of the lane centre in metres."""
    heading_rad: np.ndarray
    """True heading left of the lane direction in radians."""
    speed_m_per_s: np.ndarray
    """True forwards speed in m/s."""
    commands: list[tuple[float, DriveCommand]]
    """Time and command of each policy step."""
    calc_times_s: list[float]
    """Calculation time of each policy step in seconds."""
    metrics: dict[str, float] = field(default_factory=dict)
    failures: list[str] = field(default_factory=list)
    """Metrics that missed their threshold, as readable messages. Empty when the case passed."""

    @property
    def passed(self) -> bool:
        return not self.failures


# ---------------------------------------------------------------------------------------------------------------
# Lane and simulation set-up
# ---------------------------------------------------------------------------------------------------------------
def cone_world_position(colour: ConeColour, index: int, lane: Lane = STAGE1_LANE) -> tuple[float, float]:
    """
    :param colour: The cone's row.
    :param index: The cone's index along the row, 0 on the start line.
    :param lane: The lane.
    :return: The cone's (x, y) world position in metres.
    """
    side = 1.0 if colour == LEFT_COLOUR else -1.0
    return index * lane.cone_spacing_m, side * lane.width_m / 2.0


def all_cones(lane: Lane = STAGE1_LANE) -> list[tuple[ConeColour, int, float, float]]:
    """
    :return: Every cone of the lane as (colour, index, x, y).
    """
    return [(colour, i, *cone_world_position(colour, i, lane))
            for colour in ConeColour for i in range(lane.cones_per_row)]


def road_spec(lane: Lane = STAGE1_LANE) -> dict:
    """The lane as a dream-gym road spec (Card 2.3), with exact cone spacing as on the floor.

    It follows the "Small-Scale Cone Track" in dream-gym's docs/road_examples.md, with a straight lane only: a small
    active-lane minimum width for a 1:10 car and exact (zero deviation) cone spacing. Here the reference line is the
    lane centre, so the world frame matches the drawings.
    """
    spacing = {"distribution": "clipped_normal", "mean_m": lane.cone_spacing_m, "standard_deviation_m": 0.0,
               "lower_bound_m": 0.1, "upper_bound_m": None}
    rows = [{"placement": {"mode": "reference_line",
                           "lateral_offset_from_anchor_m": cone_world_position(c, 0, lane)[1]},
             "color": c.name.lower(), "inter_cone_spacing": spacing} for c in ConeColour]
    lanes = {"reference_line_role": "lane_center", "reference_lane": {"width_m": {"start": lane.width_m}}}
    return {"active_lane_minimum_width_m": 0.01, "elements": [
        {"geometry": {"type": "straight", "length_m": lane.length_m}, "lanes": lanes, "cones": rows},
        {"geometry": {"type": "straight", "length_m": SIM_RUN_OUT_M}, "lanes": lanes, "cones": []},
    ]}


def make_env(case: Stage1Case):
    """Creates the dream-gym environment for a case, with the car on the start line.

    :param case: The case to set up.
    :return: The environment, not reset yet.
    """
    import gymnasium
    import dreamgym  # noqa: F401  (registers the environment)

    def fixed(value):
        return {"lower": value, "upper": value}

    disturbances = case.disturbances
    mass = disturbances.car_mass_kg
    if disturbances.floor is None:
        # A very high transition speed keeps the kinematic model throughout, as in the dream-gym example
        transition = {"lower": 500.0 / 3.6, "upper": 600.0 / 3.6}
    else:
        transition = {"lower": 0.2, "upper": 0.4}
    bicycle_model_config = {
        "axle_geometry": {"front_axle_distance_from_center_of_mass_m": 0.60 * WHEELBASE_M,
                          "rear_axle_distance_from_center_of_mass_m": 0.40 * WHEELBASE_M},
        "mass_properties": {"mass_kg": mass, "yaw_moment_of_inertia_kg_m2": (1.0 / 12.0) * mass * (0.40 ** 2 + 0.25 ** 2)},
        "longitudinal_force_model": {"normalized_motor_command_force_gain_n": 10.0,
                                     "quadratic_aerodynamic_drag_coefficient_kg_per_m": 1.0},
        "steering": {"physical_front_wheel_angle_offset_from_request_rad": 0.0,
                     "requested_front_wheel_angle_magnitude_limit_rad": MAX_STEERING_RAD,
                     "physical_front_wheel_angle_rate_bounds_rad_per_s": {"lower": -math.pi / 2, "upper": math.pi / 2}},
        "kinematic_dynamic_transition": {"body_longitudinal_speed_range_mps": transition},
        "body_geometry": {"front_extent_from_center_of_mass_m": CAR_FRONT_M,
                          "rear_extent_from_center_of_mass_m": CAR_REAR_M, "width_m": CAR_WIDTH_M},
    }
    noise = {"noise_standard_deviation_m": disturbances.noise_m}
    observation_config = {
        "ground_truth_world_position_x_m": {"destination": "info"},
        "ground_truth_world_position_y_m": {"destination": "info"},
        "ground_truth_heading_in_world_frame_rad": {"destination": "info"},
        "ground_truth_body_longitudinal_velocity_mps": {"destination": "info"},
        # Stands in for the wheel speed sensor
        "body_longitudinal_velocity_mps": {"destination": "observation", "noise_standard_deviation_mps": 0.0},
        "cone_detections": {
            "acquisition": {"horizontal_field_of_view_deg": DETECTOR_FOV_DEG,
                            "forward_position_upper_bound_m": disturbances.detector_range_m,
                            "position_error_in_body_frame": {"forward": noise, "left": noise}},
            "positions_in_body_frame_m": {"destination": "observation"},
            "color_ids": {"destination": "observation"},
            "count": {"destination": "observation"},
        },
    }
    initial_state_config = {"bounds": {
        "world_pose": {"x_m": fixed(0.0), "y_m": fixed(case.start_left_m),
                       "heading_rad": fixed(math.radians(case.start_heading_deg))},
        "body_motion": {"longitudinal_velocity_mps": fixed(case.speed_m_per_s), "lateral_velocity_mps": fixed(0.0),
                        "yaw_rate_rad_per_s": fixed(0.0)},
        "front_wheel_steering_angle_rad": fixed(0.0),
    }}
    env = gymnasium.make(
        "dreamgym/autonomous_driving_env", render_mode=None,
        bicycle_model_config=bicycle_model_config, road_spec=road_spec(case.lane),
        integration_config={"method": "rk4", "environment_step_duration_s": ENV_STEP_S, "substep_count": 1},
        # Generous bounds: the run is ended and scored here, not by the environment
        termination_config={"planar_speed_bounds_mps": {"lower": 0.0, "upper": 5.0},
                            "absolute_lateral_error_from_target_line_upper_bound_m": 2.0},
        initial_state_config=initial_state_config, observation_config=observation_config,
    )
    if disturbances.floor is not None:
        env.unwrapped.set_tire_force_condition(disturbances.floor)
    return env


# ---------------------------------------------------------------------------------------------------------------
# Detections, disturbances and the stand-in for Car Control
# ---------------------------------------------------------------------------------------------------------------
def scalar(value) -> float:
    """Converts a dream-gym observation or info value, a number or a one-element array, into a float."""
    return float(np.asarray(value).reshape(-1)[0])


def to_body_frame(x: float, y: float, car_x: float, car_y: float, car_heading: float) -> tuple[float, float]:
    """Converts a world position into the car's body frame (+x forwards, +y left)."""
    dx, dy = x - car_x, y - car_y
    cos_h, sin_h = math.cos(car_heading), math.sin(car_heading)
    return cos_h * dx + sin_h * dy, -sin_h * dx + cos_h * dy


def disturbed_detections(observation: dict, pose: tuple[float, float, float], case: Stage1Case,
                         rng: np.random.Generator | None = None) -> list[ConeDetection]:
    """Converts one dream-gym detection batch into ConeDetections, applying the case's detection disturbances.

    Missed and wrong-colour cones are matched to the lane cone nearest each detection using the car's true pose,
    and extra objects are added where the camera would see them. That is the simulator's ground truth, used only to
    create the disturbance; the policy never sees it.

    :param observation: The dream-gym observation dictionary.
    :param pose: The car's true (x, y, heading) in the world frame.
    :param case: The case, for its lane and disturbances.
    :param rng: Random generator for the extra objects' detector noise; no noise if None.
    :return: The detections the policy receives.
    """
    disturbances = case.disturbances
    count = int(observation["cone_detections_count"][0])
    xs = observation["cone_detections_forward_positions_in_body_frame_m"][:count]
    ys = observation["cone_detections_left_positions_in_body_frame_m"][:count]
    ids = observation["cone_detections_color_ids"][:count]
    special = {key: "missed" for key in disturbances.missed_cones}
    special.update({key: "wrong" for key in disturbances.wrong_colour_cones})
    special_body = [(kind, *to_body_frame(*cone_world_position(*key, case.lane), *pose))
                    for key, kind in special.items()]
    # Anything closer than this to a cone is that cone, even with detector noise
    match_radius = min(0.2, 0.4 * case.lane.cone_spacing_m)
    detections = []
    for x, y, colour_id in zip(xs, ys, ids):
        colour = ConeColour(int(colour_id))
        kind = None
        for key_kind, bx, by in special_body:
            if math.hypot(x - bx, y - by) < match_radius:
                kind = key_kind
        if kind == "missed":
            continue
        if kind == "wrong":
            colour = ConeColour.BLUE if colour == ConeColour.YELLOW else ConeColour.YELLOW
        detections.append(ConeDetection(Position(float(x), float(y), 0.1), colour, 0.9))
    for ox, oy, colour in disturbances.extra_objects:
        bx, by = to_body_frame(ox, oy, *pose)
        if 0.0 < bx <= disturbances.detector_range_m and abs(math.atan2(by, bx)) <= math.radians(DETECTOR_FOV_DEG / 2):
            if rng is not None and disturbances.noise_m > 0.0:
                bx, by = bx + rng.normal(0.0, disturbances.noise_m), by + rng.normal(0.0, disturbances.noise_m)
            detections.append(ConeDetection(Position(bx, by, 0.1), colour, 0.9))
    return detections


def command_to_action(command: DriveCommand, measured_speed_m_per_s: float) -> np.ndarray:
    """A simple stand-in for Car Control, converting a DriveCommand into dream-gym's [drive, steering] action.

    Steering uses the kinematic bicycle model, wheel angle = atan(wheelbase * curvature). Drive cancels the
    simulated drag at the requested speed and corrects the speed error proportionally. Replace this with Car
    Control's conversion once it is agreed (Task 1.4), so the cases test the real chain.

    :param command: The policy's command.
    :param measured_speed_m_per_s: The wheel speed in m/s.
    :return: The normalised [drive, steering] action.
    """
    steering = math.atan(WHEELBASE_M * command.curvature_per_m) / MAX_STEERING_RAD
    target = command.speed_m_per_s
    if target == 0.0 and measured_speed_m_per_s < STOPPED_SPEED_M_PER_S:
        drive = 0.0  # stopped: hold still rather than creep backwards
    else:
        # Drag is 1.0 kg/m * v^2 and the motor gives 10 N per unit of drive
        drive = 0.1 * target ** 2 + 0.5 * (target - measured_speed_m_per_s)
    return np.clip(np.array([drive, steering], dtype=np.float32), -1.0, 1.0)


# ---------------------------------------------------------------------------------------------------------------
# Running a case
# ---------------------------------------------------------------------------------------------------------------
def run_case(case: Stage1Case, policy, thresholds: Thresholds = Thresholds()) -> Stage1Result:
    """Runs a policy on one case and scores it.

    :param case: The case to run.
    :param policy: The policy, with run_policy(observations, lanes) returning a list holding one DriveCommand. It
        is not reset here; it receives is_first_policy_step on the first step, as on the car.
    :param thresholds: The pass thresholds.
    :return: The trajectory, metrics and pass/fail result.
    """
    env = make_env(case)
    observation, info = env.reset(seed=case.seed)
    rng = np.random.default_rng(case.seed)
    end_x = case.lane.length_m + SIM_RUN_OUT_M - 0.5
    steps_per_policy = round(POLICY_PERIOD_S / ENV_STEP_S)
    delay_steps = round(case.disturbances.delay_s / POLICY_PERIOD_S)
    pending: list[list[ConeDetection]] = []
    log = {"t": [], "x": [], "y": [], "heading": [], "speed": []}
    commands, calc_times = [], []
    action = np.zeros(2, dtype=np.float32)
    stopped_since = None
    step = 0

    def record(info):
        log["t"].append(step * ENV_STEP_S)
        log["x"].append(scalar(info["ground_truth_world_position_x_m"]))
        log["y"].append(scalar(info["ground_truth_world_position_y_m"]))
        log["heading"].append(scalar(info["ground_truth_heading_in_world_frame_rad"]))
        log["speed"].append(scalar(info["ground_truth_body_longitudinal_velocity_mps"]))

    record(info)
    while step * ENV_STEP_S < MAX_DURATION_S:
        if step % steps_per_policy == 0:
            pose = (log["x"][-1], log["y"][-1], log["heading"][-1])
            pending.append(disturbed_detections(observation, pose, case, rng))
            cones = pending.pop(0) if len(pending) > delay_steps else None
            speed = scalar(observation["body_longitudinal_velocity_mps"])
            first = step == 0
            observations = CarObservations(
                cones=None if cones is None else ConeBatch(cones, SensorAge(case.disturbances.delay_s, None),
                                                           acquisition_to_publish_latency_s=0.0),
                fiducials=None,
                lidar_scan=None,
                lidar_cartesian=None,
                car=ObservedCarState(None, None, None, WheelSpeed(abs(speed), SensorAge(0.0, None))),
                policy=PolicyState(dt=0.0 if first else POLICY_PERIOD_S, policy_elapsed_s=step * ENV_STEP_S,
                                   is_first_policy_step=first),
            )
            start = time.perf_counter()
            instructions = policy.run_policy(observations, None)
            calc_times.append(time.perf_counter() - start)
            command = next(i for i in instructions if isinstance(i, DriveCommand))
            commands.append((step * ENV_STEP_S, command))
            action = command_to_action(command, abs(speed))
        observation, _, terminated, truncated, info = env.step(action)
        step += 1
        record(info)
        if abs(log["speed"][-1]) < STOPPED_SPEED_M_PER_S:
            stopped_since = step if stopped_since is None else stopped_since
            if (step - stopped_since) * ENV_STEP_S >= 0.5:
                break
        else:
            stopped_since = None
        if terminated or truncated or log["x"][-1] > end_x:
            break
    env.close()
    result = Stage1Result(case, *(np.array(log[k]) for k in ("t", "x", "y", "heading", "speed")),
                          commands=commands, calc_times_s=calc_times)
    result.metrics = compute_metrics(result, getattr(policy, "fallback_count", 0))
    result.failures = check_thresholds(result.metrics, thresholds)
    return result


# ---------------------------------------------------------------------------------------------------------------
# Metrics (Card 1.1) and the pass rule (Card 2.1)
# ---------------------------------------------------------------------------------------------------------------
def body_corners(x: float, y: float, heading: float) -> np.ndarray:
    """
    :return: The world (x, y) of the car body's four corners, as a 4x2 array.
    """
    local = np.array([[CAR_FRONT_M, CAR_WIDTH_M / 2], [CAR_FRONT_M, -CAR_WIDTH_M / 2],
                      [-CAR_REAR_M, -CAR_WIDTH_M / 2], [-CAR_REAR_M, CAR_WIDTH_M / 2]])
    rotation = np.array([[math.cos(heading), -math.sin(heading)], [math.sin(heading), math.cos(heading)]])
    return local @ rotation.T + np.array([x, y])


def distance_to_body(point_x: float, point_y: float, x: float, y: float, heading: float) -> float:
    """
    :return: Distance in metres from a world point to the car body's rectangle; 0.0 inside it.
    """
    bx, by = to_body_frame(point_x, point_y, x, y, heading)
    dx = max(-CAR_REAR_M - bx, 0.0, bx - CAR_FRONT_M)
    dy = max(-CAR_WIDTH_M / 2 - by, 0.0, by - CAR_WIDTH_M / 2)
    return math.hypot(dx, dy)


def compute_metrics(result: Stage1Result, fallback_count: int) -> dict[str, float]:
    """Computes the Stage 1 metrics for one run from the true trajectory.

    :param result: The run, without its metrics.
    :param fallback_count: The policy's solver fallback count for the run, or 0 if it has none.
    :return: The metrics by name. Units are in the names.
    """
    lane = result.case.lane
    x, y, heading, speed = result.x_m, result.y_m, result.heading_rad, result.speed_m_per_s
    beside = (x >= 0.0) & (x <= lane.length_m)
    cones = all_cones(lane)
    touched = set()
    outside = 0.0
    for i in np.flatnonzero(x <= lane.length_m + CAR_REAR_M):
        for colour, index, cx, cy in cones:
            if distance_to_body(cx, cy, x[i], y[i], heading[i]) < CONE_RADIUS_M:
                touched.add((colour, index))
        if beside[i]:
            outside = max(outside, float(np.max(np.abs(body_corners(x[i], y[i], heading[i])[:, 1]))) - lane.width_m / 2)
    # Cruising: from 1 m after the start line to 1 m before the last cone pair, and only until the policy first
    # asks to stop after driving, because stopping is scored by the stop position instead. Stop requests before
    # the first drive request (waiting for a delayed first detection) do not end it.
    times = np.array([t for t, _ in result.commands])
    driving = [t for t, c in result.commands if c.speed_m_per_s > 0.0]
    stop_requests = [t for t, c in result.commands if c.speed_m_per_s == 0.0 and driving and t > driving[0]]
    first_stop_s = stop_requests[0] if stop_requests else math.inf
    cruise = (x >= 1.0) & (x <= lane.length_m - 1.0) & (result.time_s <= first_stop_s)
    target = result.case.speed_m_per_s
    curvatures = np.array([c.curvature_per_m for _, c in result.commands])
    rates = np.diff(curvatures) / np.diff(times) if len(times) > 1 else np.zeros(1)
    stopped = abs(speed[-1]) < STOPPED_SPEED_M_PER_S
    # At the last cone pair, or where the car stopped if that is before it
    end_index = int(np.flatnonzero(x <= lane.length_m)[-1])
    return {
        "cones_touched": float(len(touched)),
        "body_outside_lane_m": max(outside, 0.0),
        "lateral_error_at_lane_end_m": abs(float(y[end_index])),
        "max_abs_lateral_error_m": float(np.max(np.abs(y[beside]))) if beside.any() else math.nan,
        "mean_speed_error_m_per_s": float(np.mean(np.abs(speed[cruise] - target))) if cruise.any() else math.nan,
        "rms_curvature_rate_per_m_s": float(np.sqrt(np.mean(rates ** 2))),
        "stopped": float(stopped),
        "stop_past_last_cone_m": float(x[-1] - lane.length_m) if stopped else math.nan,
        "fallback_count": float(fallback_count),
        "p99_calc_time_s": float(np.percentile(result.calc_times_s, 99)),
        "duration_s": float(result.time_s[-1]),
    }


def check_thresholds(metrics: dict[str, float], thresholds: Thresholds) -> list[str]:
    """
    :return: A message for each threshold the metrics miss. A missing (nan) metric misses its threshold.
    """
    def at_most(name, limit):
        value = metrics[name]
        return [] if value <= limit else [f"{name} = {value:.3g} > {limit:.3g}"]

    stop_low, stop_high = thresholds.stop_zone_m
    stop = metrics["stop_past_last_cone_m"]
    failures = (at_most("cones_touched", thresholds.max_cones_touched)
                + at_most("body_outside_lane_m", thresholds.max_body_outside_lane_m)
                + at_most("lateral_error_at_lane_end_m", thresholds.max_lateral_error_at_lane_end_m)
                + at_most("mean_speed_error_m_per_s", thresholds.max_mean_speed_error_m_per_s)
                + at_most("rms_curvature_rate_per_m_s", thresholds.max_rms_curvature_rate_per_m_s)
                + at_most("fallback_count", thresholds.max_fallback_count)
                + at_most("p99_calc_time_s", thresholds.max_p99_calc_time_s))
    if not metrics["stopped"]:
        failures.append("did not stop")
    elif not stop_low <= stop <= stop_high:
        failures.append(f"stop_past_last_cone_m = {stop:.3g} outside [{stop_low}, {stop_high}]")
    return failures


# ---------------------------------------------------------------------------------------------------------------
# Figures: trajectories for the log-book, printable floor drawings and the simulated lanes
# ---------------------------------------------------------------------------------------------------------------
def _draw_cones(ax, lane: Lane, case: Stage1Case | None = None):
    import matplotlib.pyplot as plt

    missed = case.disturbances.missed_cones if case else ()
    wrong = case.disturbances.wrong_colour_cones if case else ()
    for colour, index, cx, cy in all_cones(lane):
        face = "none" if (colour, index) in missed else ("gold" if colour == ConeColour.YELLOW else "royalblue")
        special = (colour, index) in missed or (colour, index) in wrong
        ax.add_patch(plt.Circle((cx, cy), CONE_RADIUS_M, facecolor=face, edgecolor="red" if special else "k",
                                linewidth=1.5 if special else 0.5))


def plot_result(result: Stage1Result, path: Path):
    """Saves a top-down trajectory figure of one run, with the cones, the stop zone and the body outline."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    case, lane = result.case, result.case.lane
    fig, ax = plt.subplots(figsize=(10, 2.6))
    _draw_cones(ax, lane, case)
    for ox, oy, colour in case.disturbances.extra_objects:
        ax.plot(ox, oy, "s", color="darkorange", markeredgecolor="red", label="extra object")
    ax.axvspan(lane.length_m + STOP_ZONE_M[0], lane.length_m + STOP_ZONE_M[1], color="green", alpha=0.15,
               label="stop zone")
    ax.plot(result.x_m, result.y_m, "k-", label="car centre")
    for i in range(0, len(result.x_m), 10):
        corners = body_corners(result.x_m[i], result.y_m[i], result.heading_rad[i])
        ax.add_patch(plt.Polygon(corners, closed=True, fill=False, edgecolor="grey", linewidth=0.4))
    half = lane.width_m / 2 + 0.4
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, lane.length_m + 1.5)
    ax.set_ylim(-half, half)
    ax.set_xlabel("x along lane [m]")
    ax.set_ylabel("y left [m]")
    ax.set_title(f"{case.name}: {case.description} - {'PASS' if result.passed else 'FAIL'}")
    ax.legend(loc="upper right", fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _draw_floor_layout(ax, lane: Lane):
    """Draws the to-scale floor layout of a lane on an axis (Card 2.2)."""
    import matplotlib.pyplot as plt

    length, width = lane.floor_area_m
    end = length - RUN_UP_M
    half = width / 2
    ax.add_patch(plt.Rectangle((-RUN_UP_M, -half), length, width, fill=False, linestyle=":", color="grey"))
    _draw_cones(ax, lane)
    label_every = max(1, round(1.0 / lane.cone_spacing_m))
    for colour, index, cx, cy in all_cones(lane):
        if index % label_every == 0 or index == lane.cones_per_row - 1:
            ax.annotate(str(index), (cx, cy), xytext=(0, 9 if cy > 0 else -14), textcoords="offset points",
                        ha="center", fontsize=7)
    ax.axvspan(-RUN_UP_M, 0.0, color="grey", alpha=0.12)
    ax.text(-RUN_UP_M / 2, 0.0, "run-up\n(start from rest)", ha="center", va="center", fontsize=8)
    ax.axvline(0.0, color="k", linewidth=1.5)
    ax.text(0.03, -half + 0.05, "START LINE (car centre)", fontsize=8)
    for offset in np.arange(-0.2, 0.2001, 0.05):
        ax.plot([-0.04, 0.04], [offset, offset], "k-", linewidth=0.8)
    ax.text(0.06, 0.2, "offset ticks every 0.05 m (+ left)", fontsize=7, va="center")
    ax.axvspan(lane.length_m + STOP_ZONE_M[0], lane.length_m + STOP_ZONE_M[1], color="green", alpha=0.2)
    ax.text(lane.length_m + sum(STOP_ZONE_M) / 2, 0.0, "STOP\nZONE", ha="center", va="center", fontsize=8)
    ax.text(lane.length_m + STOP_ZONE_M[1] + SAFETY_RUN_OUT_M / 2, 0.0, "clear\nrun-out", ha="center",
            va="center", fontsize=8, color="grey")
    top = lane.width_m / 2 + 0.18
    ax.annotate("", (0.0, top), (lane.length_m, top), arrowprops={"arrowstyle": "<->"})
    ax.text(lane.length_m / 2, top + 0.03, f"{lane.length_m:g} m, cones every {lane.cone_spacing_m:g} m "
            f"(index 0-{lane.cones_per_row - 1})", ha="center", fontsize=8)
    arrow_x = lane.length_m + STOP_ZONE_M[1] + 0.85 * SAFETY_RUN_OUT_M
    ax.annotate("", (arrow_x, -lane.width_m / 2), (arrow_x, lane.width_m / 2), arrowprops={"arrowstyle": "<->"})
    ax.text(arrow_x + 0.05, 0.0, f"{lane.width_m:g} m\nlane", fontsize=8, va="center")
    ax.set_aspect("equal")
    ax.set_xlim(-RUN_UP_M - 0.1, end + 0.1)
    ax.set_ylim(-half - 0.05, half + 0.05)
    ax.set_xticks(np.arange(-RUN_UP_M, end + 0.01, 0.5))
    ax.grid(linestyle=":", linewidth=0.4)
    ax.set_xlabel("x along lane from the start line [m]")
    ax.set_ylabel("y left of lane centre [m]")
    ax.set_title(f"Stage 1 lane {lane.name}: {lane.cones_per_row} {LEFT_COLOUR.name.lower()} cones left, "
                 f"{lane.cones_per_row} blue cones right; floor area {length:g} m x {width:.2g} m")


def plot_layout(folder: Path, lane: Lane = STAGE1_LANE):
    """Saves a lane's printable floor drawing (Card 2.2) and the drawing beside its dream-gym lane (Card 2.3).

    :param folder: Folder for stage1_layout_<lane>.png/.pdf and stage1_drawing_vs_sim_<lane>.png.
    :param lane: The lane to draw.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from dreamgym.envs import Road

    folder.mkdir(parents=True, exist_ok=True)
    floor_length, floor_width = lane.floor_area_m
    height = 1.6 + 2.2 * floor_width
    fig, ax = plt.subplots(figsize=(11.7, height))  # A4 landscape width
    _draw_floor_layout(ax, lane)
    fig.tight_layout()
    for suffix in ("png", "pdf"):
        fig.savefig(folder / f"stage1_layout_{lane.name}.{suffix}", dpi=200, bbox_inches="tight")
    plt.close(fig)

    fig, (top, bottom) = plt.subplots(2, 1, figsize=(11.7, 1.15 * height), sharex=True)
    _draw_floor_layout(top, lane)
    top.set_title(f"Floor drawing, lane {lane.name}")
    Road(road_spec=road_spec(lane)).plot_road_on_matplotlib_axis(bottom, progress_step_m=0.05)
    bottom.set_aspect("equal")
    bottom.set_ylim(top.get_ylim())
    bottom.set_title("The same lane in dream-gym (road_spec in tests/policy/stage1_scenarios.py)")
    bottom.set_xlabel("world x [m]")
    bottom.set_ylabel("world y [m]")
    fig.tight_layout()
    fig.savefig(folder / f"stage1_drawing_vs_sim_{lane.name}.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def format_table(results: list[Stage1Result]) -> str:
    """
    :return: A Markdown table of the main metrics of each run, for the log-book.
    """
    rows = ["| Case | Lane | Touched | Outside lane [m] | Lat. err. at end [m] | Speed err. [m/s] | Curv. rate RMS "
            "[1/(m s)] | Stop past last cone [m] | Fallbacks | p99 calc [ms] | Result |",
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in results:
        m = r.metrics
        rows.append(f"| {r.case.name} | {r.case.lane.name} | {m['cones_touched']:.0f} | "
                    f"{m['body_outside_lane_m']:.3f} | {m['lateral_error_at_lane_end_m']:.3f} | "
                    f"{m['mean_speed_error_m_per_s']:.3f} | {m['rms_curvature_rate_per_m_s']:.2f} | "
                    f"{m['stop_past_last_cone_m']:.2f} | {m['fallback_count']:.0f} | "
                    f"{1000 * m['p99_calc_time_s']:.1f} | "
                    f"{'PASS' if r.passed else 'FAIL: ' + '; '.join(r.failures)} |")
    return "\n".join(rows)


class ConeFilteredPolicy:
    """Runs the cone filter before a policy, as MovementPolicy.step in policy/policy_runner.py does on the car.

    Lane detection is skipped (it does not produce lanes yet) and control is replaced by command_to_action.
    """

    def __init__(self, policy, cone_filter_config):
        """
        :param policy: The movement policy, with run_policy(observations, lanes).
        :param cone_filter_config: The cone filter settings (ConeFilterConfig).
        """
        self.policy = policy
        self.cone_filter_config = cone_filter_config

    @property
    def fallback_count(self) -> int:
        return getattr(self.policy, "fallback_count", 0)

    def run_policy(self, observations: CarObservations, lanes):
        filtered = dataclasses.replace(observations, cones=filter_cones(observations.cones, self.cone_filter_config))
        return self.policy.run_policy(filtered, lanes)


def mpc_config_for(case: Stage1Case):
    """The MPC settings for a case: the shipped defaults, set to the case's speed and lane width, as the team would
    configure them for that lane in config/ai4r_policy.yaml."""
    return MpcConfig(target_speed_m_per_s=case.speed_m_per_s, lane_width_m=case.lane.width_m)


def shipped_mpc_policy(case: Stage1Case) -> ConeFilteredPolicy:
    """The team's current pipeline for a case: the shipped cone filter settings, then the MPC (mpc_config_for)."""
    return ConeFilteredPolicy(MpcPolicy(mpc_config_for(case)), ConeFilterConfig())


def main():
    import argparse
    from stage1_tuning_cases import TUNING_CASES

    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--unseen", action="store_true", help="run the unseen cases (final comparison only)")
    parser.add_argument("--figures", type=Path, help="folder to save a trajectory figure of each case in")
    parser.add_argument("--layout", type=Path, help="save the floor drawings and simulated lanes of the tuning "
                                                    "lanes in this folder, then stop")
    args = parser.parse_args()
    if args.layout:
        for lane in dict.fromkeys(case.lane for case in TUNING_CASES):
            plot_layout(args.layout, lane)
        return
    cases = TUNING_CASES
    if args.unseen:
        sys.path.insert(0, str(Path(__file__).resolve().parent / "unseen"))
        from stage1_unseen_cases import UNSEEN_CASES
        cases = UNSEEN_CASES
    results = []
    for case in cases:
        result = run_case(case, shipped_mpc_policy(case))
        results.append(result)
        if args.figures:
            args.figures.mkdir(parents=True, exist_ok=True)
            plot_result(result, args.figures / f"stage1_{case.name}.png")
    print(format_table(results))


if __name__ == "__main__":
    main()
