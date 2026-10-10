"""
open_space.py

This file holds the RC-assisted open-space policy. The operator holds the remote control (RC) drive forwards to let
the car drive, and the policy steers it through the most open space between the cones.

At each policy step:
1. The car only drives while the RC drive is held forwards, at least rc_drive_threshold. Otherwise, and without
   fresh cone data, it stops with centred steering. The vehicle's own RC stop (drive held backwards) does not depend
   on this policy.
2. The RC steering chooses the preferred heading: centred prefers straight ahead, and full steering prefers
   max_preferred_heading_rad to that side.
3. The cones are the latest detections plus the cones remembered from the last cone_memory_s, moved with the car's
   measured motion. The camera loses cones as the car passes them, so without this memory it would cut back into
   them.
4. Each candidate heading within +-max_heading_rad gives the arc the car would drive towards it (see step 6). The
   arc's corridor is look_ahead_m long, and its clearance is the distance from the arc to the nearest cone in it. A
   heading is open when its clearance is at least clearance_m.
5. The car takes the open heading with the best score: its clearance (up to clearance_cap_m), minus penalties for
   turning away from the preferred heading and from the previous heading. With no cone ahead, that is the preferred
   heading. With no open heading, the car stops.
6. Pure pursuit turns the heading into a curvature: the car drives the arc through the point steering_look_ahead_m
   along it.

Every cone is an obstacle, whatever its colour. The car only follows each arc approximately, because the steering takes
time to move, and the remembered cones drift as the motion estimate does; keep the speed low.
"""
import math
from dataclasses import dataclass
from enum import IntEnum

from policy.control.actions import DriveCommand, Instruction
from policy.input_output import CarObservations, ConeBatch, Position, RcInput

MAX_REMEMBERED_CONES = 100
"""Most cones remembered from earlier detections; the nearest are kept. It bounds each step's calculation time."""


@dataclass(frozen=True)
class OpenSpaceConfig:
    """Settings for the open-space policy, loaded from the open_space parameters in config/ai4r_policy.yaml.

    These defaults are also the parameters' defaults in policy_node. They are unmeasured starting guesses. Invalid
    settings raise a ValueError, which stops policy_node from starting.
    """
    speed_m_per_s: float = 0.5
    """Speed to drive at in m/s while the RC drive is held forwards. Must be positive."""
    rc_drive_threshold: float = 0.25
    """Smallest normalised RC drive that lets the car drive, in (0, 1]."""
    rc_steer_deadband: float = 0.15
    """Normalised RC steering smaller than this in size counts as centred, in [0, 1)."""
    max_preferred_heading_rad: float = 0.5
    """Preferred heading in radians at full RC steering, in [0, max_heading_rad]."""
    max_heading_rad: float = 0.7
    """Largest candidate heading in radians either side of straight ahead, in (0, pi/2)."""
    heading_count: int = 29
    """Number of evenly spaced candidate headings. Integer from 3 to 181; an odd number includes straight ahead."""
    look_ahead_m: float = 1.5
    """Length of each heading's corridor in metres along its arc from the car's origin. Must be positive.

    Cones farther along the corridor are ignored, so keep it within the distance the camera sees cones.
    """
    clearance_m: float = 0.35
    """Smallest distance in metres from a heading's arc to a cone in its corridor for the heading to be open. Must be
    positive.

    Half the car's width, plus the cone's radius, plus a margin.
    """
    clearance_cap_m: float = 1.0
    """Clearance in metres beyond which more clearance is not better. Must be at least clearance_m."""
    preferred_heading_weight_m_per_rad: float = 0.5
    """Score lost per radian between a heading and the preferred heading, in metres of clearance. Must be at least 0.

    Keep it above heading_change_weight_m_per_rad, so the car takes the preferred heading when nothing blocks it.
    """
    heading_change_weight_m_per_rad: float = 0.2
    """Score lost per radian between a heading and the previous step's heading, in metres of clearance. Must be at
    least 0. It stops the choice flickering between two similar gaps."""
    steering_look_ahead_m: float = 1.0
    """Distance in metres along the chosen heading to the pure pursuit point. Must be positive; smaller turns
    harder."""
    cone_memory_s: float = 1.0
    """Seconds a cone is remembered after its last detection, moved with the car's measured motion. Must be finite
    and at least 0; 0.0 uses only the latest detections.

    Long enough for the car to pass a cone that has left the camera's view; longer lets motion errors build up.
    """
    memory_merge_distance_m: float = 0.25
    """A remembered cone closer than this in metres to a new detection is the same cone, and is replaced by it. Must
    be positive."""

    def __post_init__(self):
        count = self.heading_count
        if isinstance(count, bool) or not isinstance(count, int) or not 3 <= count <= 181:
            raise ValueError(f"open_space.heading_count must be an integer from 3 to 181, not {count}")
        for name in ("speed_m_per_s", "look_ahead_m", "clearance_m", "steering_look_ahead_m",
                     "memory_merge_distance_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"open_space.{name} must be positive and finite, not {value}")
        for name in ("preferred_heading_weight_m_per_rad", "heading_change_weight_m_per_rad", "cone_memory_s"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"open_space.{name} must be finite and at least 0, not {value}")
        if not 0.0 < self.rc_drive_threshold <= 1.0:
            raise ValueError(f"open_space.rc_drive_threshold must be in (0, 1], not {self.rc_drive_threshold}")
        if not 0.0 <= self.rc_steer_deadband < 1.0:
            raise ValueError(f"open_space.rc_steer_deadband must be in [0, 1), not {self.rc_steer_deadband}")
        if not 0.0 < self.max_heading_rad < math.pi / 2.0:
            raise ValueError(f"open_space.max_heading_rad must be in (0, pi/2), not {self.max_heading_rad}")
        if not 0.0 <= self.max_preferred_heading_rad <= self.max_heading_rad:
            raise ValueError("open_space.max_preferred_heading_rad must be in [0, max_heading_rad], not "
                             f"{self.max_preferred_heading_rad}")
        if not (math.isfinite(self.clearance_cap_m) and self.clearance_cap_m >= self.clearance_m):
            raise ValueError(f"open_space.clearance_cap_m must be finite and at least clearance_m, not "
                             f"{self.clearance_cap_m}")


class OpenSpaceStatus(IntEnum):
    """Why the open-space policy chose its command. MovementPolicy publishes it on debug2."""
    WAITING_FOR_RC = 0
    """Stopped: the RC drive is not held forwards, or the RC is missing."""
    NO_CONE_DATA = 1
    """Stopped: there is no fresh cone data, so the space ahead is unknown."""
    BLOCKED = 2
    """Stopped: no heading is open."""
    DRIVING = 3
    """Driving along the chosen heading."""


class OpenSpacePolicy:
    """Chooses where to drive from the operator's RC request and the open space between the cones.

    It keeps state between policy steps, so create one when the node starts and reuse it for every step.
    """

    def __init__(self, config: OpenSpaceConfig, positive_steering_turns_left: bool):
        """
        :param config: The open-space settings.
        :param positive_steering_turns_left: Whether a positive steering action turns the car left, from the control
            settings. Positive RC steering is assumed to turn the same way.
        """
        self.config = config
        self.positive_steering_turns_left = positive_steering_turns_left
        spacing = 2.0 * config.max_heading_rad / (config.heading_count - 1)
        self._headings = [-config.max_heading_rad + i * spacing for i in range(config.heading_count)]
        self.reset()

    def reset(self):
        """Forgets the previous run. This is done on the first step after entering the publishing-policy state."""
        self.heading_rad = None
        """Heading in radians (positive left) chosen at this step, even while stopped, so it can be checked by moving
        the car by hand. None without fresh cone data or an open heading."""
        self.status = OpenSpaceStatus.WAITING_FOR_RC
        """Why this step's command was chosen."""
        self.previous_heading_rad = 0.0
        """Latest chosen heading in radians, which the next choice is kept close to."""
        self.previous_command = DriveCommand(0.0, 0.0)
        """Command chosen at the previous step, used to estimate the motion when a measurement is missing."""
        self.remembered_cones: list[tuple[Position, float]] = []
        """Cones in base_link at this step, each with the seconds since its detection. The latest batch's cones come
        first."""
        self.latest_batch_size = 0
        """Number of cones at the start of remembered_cones from the latest batch. They are kept until a new batch
        replaces them, even past cone_memory_s, because the node already stops reusing a stale batch."""
        self.latest_batch_stamp_ns = None
        """ROS stamp of the latest cone batch added to the memory, so a batch reused by timer mode is added once."""

    def run_policy(self, observations: CarObservations) -> list[Instruction]:
        """Chooses the instructions for the car to follow from the observations

        :param observations: The observations taken by the car at the current policy step, with only plausible cones.
        :return: The instructions for the control module to convert into car actions
        """
        if observations.policy.is_first_policy_step:
            self.reset()
        config = self.config
        cones = observations.cones
        self.remember_cones(observations)
        obstacles = [position for position, _ in self.remembered_cones]
        self.heading_rad = None if cones is None else self.choose_heading(
            obstacles, self.preferred_heading_rad(observations.rc))
        if self.heading_rad is not None:
            self.previous_heading_rad = self.heading_rad
        if observations.rc is None or observations.rc.drive < config.rc_drive_threshold:
            self.status = OpenSpaceStatus.WAITING_FOR_RC
        elif cones is None:
            self.status = OpenSpaceStatus.NO_CONE_DATA
        elif self.heading_rad is None:
            self.status = OpenSpaceStatus.BLOCKED
        else:
            self.status = OpenSpaceStatus.DRIVING
            self.previous_command = DriveCommand(config.speed_m_per_s, self.curvature_per_m(self.heading_rad))
            return [self.previous_command]
        self.previous_command = DriveCommand(0.0, 0.0)
        return [self.previous_command]

    def remember_cones(self, observations: CarObservations):
        """Moves the remembered cones with the car's motion since the previous step, forgets the old ones, then adds a
        new cone batch, which replaces the remembered cones it detects again.

        The distance comes from the wheel speed, or the previous command's speed when it is missing. The turn comes
        from the IMU's yaw rate, or the previous command's curvature when it is missing.

        :param observations: The observations taken by the car at the current policy step.
        """
        config = self.config
        dt = observations.policy.dt
        car = observations.car
        distance = (self.previous_command.speed_m_per_s if car.wheel_speed is None else car.wheel_speed) * dt
        turn = (self.previous_command.curvature_per_m * distance if car.ang_velocity is None
                else car.ang_velocity.yaw * dt)
        # Over the step the car moves along a chord at half the turn, then each cone is rotated by -turn
        move_x, move_y = distance * math.cos(turn / 2.0), distance * math.sin(turn / 2.0)
        cos_turn, sin_turn = math.cos(turn), math.sin(turn)
        moved = []
        for index, (position, age) in enumerate(self.remembered_cones):
            if index >= self.latest_batch_size and age + dt >= config.cone_memory_s:
                continue
            x, y = position.x - move_x, position.y - move_y
            moved.append((Position(x * cos_turn + y * sin_turn, y * cos_turn - x * sin_turn, position.z), age + dt))
        cones: ConeBatch | None = observations.cones
        stamp_ns = None if cones is None else cones.sensor_age.stamp_ns
        if cones is not None and (stamp_ns is None or stamp_ns != self.latest_batch_stamp_ns):
            self.latest_batch_stamp_ns = stamp_ns
            # The previous batch's cones are now ordinary remembered cones, which can expire
            moved = [(position, age) for position, age in moved
                     if age < config.cone_memory_s
                     and all(math.hypot(position.x - cone.pos.x, position.y - cone.pos.y)
                             >= config.memory_merge_distance_m for cone in cones)]
            moved.sort(key=lambda item: math.hypot(item[0].x, item[0].y))
            moved = [(cone.pos, 0.0) for cone in cones] + moved[:MAX_REMEMBERED_CONES]
            self.latest_batch_size = len(cones)
        self.remembered_cones = moved

    def preferred_heading_rad(self, rc: RcInput | None) -> float:
        """
        :param rc: The operator's RC sticks, or None when missing.
        :return: The heading in radians (positive left) that the RC steering asks for; 0.0 when centred or missing.
        """
        config = self.config
        if rc is None or abs(rc.steer) < config.rc_steer_deadband:
            return 0.0
        # Grow from zero at the deadband, so the preference has no jump
        fraction = min((abs(rc.steer) - config.rc_steer_deadband) / (1.0 - config.rc_steer_deadband), 1.0)
        turns_left = (rc.steer > 0.0) == self.positive_steering_turns_left
        return math.copysign(fraction * config.max_preferred_heading_rad, 1.0 if turns_left else -1.0)

    def choose_heading(self, cones: list[Position], preferred_heading_rad: float) -> float | None:
        """
        :param cones: The cones' positions in base_link, all treated as obstacles.
        :param preferred_heading_rad: The heading the operator asks for in radians, positive left.
        :return: The open heading with the best score in radians, positive left, or None when no heading is open.
        """
        config = self.config
        best, best_score = None, -math.inf
        for heading in self._headings:
            clearance = self.clearance_m(cones, heading)
            if clearance < config.clearance_m:
                continue
            score = (min(clearance, config.clearance_cap_m)
                     - config.preferred_heading_weight_m_per_rad * abs(heading - preferred_heading_rad)
                     - config.heading_change_weight_m_per_rad * abs(heading - self.previous_heading_rad))
            if score > best_score:
                best, best_score = heading, score
        return best

    def curvature_per_m(self, heading_rad: float) -> float:
        """
        :param heading_rad: A heading in radians, positive left.
        :return: Pure pursuit's curvature in 1/m, positive turning left: the arc from the car through the point
            steering_look_ahead_m along the heading.
        """
        return 2.0 * math.sin(heading_rad) / self.config.steering_look_ahead_m

    def clearance_m(self, cones: list[Position], heading_rad: float) -> float:
        """
        :param cones: The cones' positions in base_link, all treated as obstacles.
        :param heading_rad: The heading in radians, positive left, whose arc is checked.
        :return: Distance in metres from the heading's arc to the nearest cone within look_ahead_m along it;
            infinity when there is none.
        """
        curvature = self.curvature_per_m(heading_rad)
        nearest = math.inf
        for cone in cones:
            if abs(curvature) < 1e-9:
                # Straight ahead
                along, distance = cone.x, abs(cone.y)
            else:
                # The arc is part of a circle round (0, radius). Measure the angle round the centre from the car to
                # the cone, then the distance between the cone and the circle.
                radius = 1.0 / curvature
                along = abs(radius) * math.atan2(cone.x, (radius - cone.y) * math.copysign(1.0, radius))
                distance = abs(math.hypot(cone.x, cone.y - radius) - abs(radius))
            if 0.0 < along <= self.config.look_ahead_m:
                nearest = min(nearest, distance)
        return nearest
