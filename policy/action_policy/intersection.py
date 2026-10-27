"""
intersection.py

Turns left or right at an intersection, or drives straight across it, when an ArUco marker asks (policy/markers:
TURN_LEFT, TURN_RIGHT and STRAIGHT). The marker stands at the intersection like a road sign, beside where the approach
lane's cone rows end; intersection.marker_to_entry_m says how far beyond the marker they end. The intersection is
taken to be where a road of cross_lane_width_m (by default the approach lane's own width) crosses the approach road.

When the marker is confirmed, the manoeuvre places the intersection's entry in the run's map frame from the marker's
position and the car's pose (CarPose, kept by the local map's dead reckoning), and remembers the approach lane's
centre line and direction. Then, in phases (TurnPhase, kept in CarMemory.intersection_turn):

1. APPROACH: lane following (the MPC) drives on, with the lane refining the remembered centre line. Lane detection
   only gets the cones of the approach road, as the cross road's cones past the entry would bend the fitted lane. For a
   turn, the MPC keeps to the outside of the lane (left before a right turn, as a hook turn does), aside_clearance_m
   from its edge: from the middle of a lab-width lane the car cannot steer tightly enough to clear both the inside
   corner and the far side of the road it turns into. The lane side the car had is given back at the end. From
   hold_line_m before the entry, where the approach rows end and lane detection loses the lane, or whenever there is
   no lane, the car holds the remembered approach direction instead, slowing to turn_speed_m_per_s.
2. ARC: at the arc's start (arc_start_m), the car follows a circle of the turn's radius round the intersection's
   inside corner, with pure pursuit on the circle and the car's pose. The start puts the arc's end on the exit lane's
   centre line where the car can steer that tightly; otherwise it keeps the car corner_clearance_m clear of the
   inside corner cone and of the exit's far cone row, or splits the difference when it cannot do both.
3. EXIT: once turned to the exit's direction (or, going straight, once past the entry), the car follows the exit line
   until lane detection gives a lane lined up with it, then lane following takes over and the manoeuvre is finished.
   Lane following may also take over during the arc, once the car has turned handover_heading_deg. Without an exit
   lane within exit_search_m, the car stops for the rest of the run, as for a lost lane.

The obstacle stop keeps working throughout (MpcPolicy.follow_manoeuvre). The pose is dead reckoning, corrected against
the remembered cones while cones are in view, so the arc is only as good as the wheel speed and IMU heading over the
few metres of the manoeuvre.
"""
import logging
import math
from dataclasses import dataclass, replace

from policy.action_policy.policy import MpcPolicy
from policy.input_output import ConeDetection, Position
from policy.lane_detection.lanes import LaneDetection
from policy.markers.commands import MarkerCommand
from policy.memory import CarMemory, CarPose, IntersectionTurn, TurnPhase

_LOGGER = logging.getLogger(__name__)

CONE_RADIUS_M = 0.05
"""Radius of a cone's base in metres, kept clear of the car's body by corner_clearance_m."""
REFINE_GAIN = 0.2
"""Fraction of the difference to the lane's centre line and direction that each usable lane step moves the remembered
approach line by, while the car is more than hold_line_m from the entry."""
HANDOVER_HEADING_RAD = math.radians(20.0)
"""Largest angle between the car and a detected exit lane, in radians, for lane following to take over."""
HANDOVER_EDGE_M = 0.1
"""Lane following takes over only with the car at least this far, in metres, inside the detected exit lane's edges."""
CURVATURE_HEADROOM = 1.2
"""The manoeuvre asks for at most this times its arc's curvature each way (1 / the turn radius), so pure pursuit does
not ask for curves far tighter than the car can drive when it falls behind the arc."""
CORRIDOR_MARGIN_M = 0.25
"""While approaching, cones more than this far outside the approach lane's edges, or this far past the entry, are
left out of lane detection, in metres."""


@dataclass(frozen=True)
class IntersectionConfig:
    """Settings for the intersection manoeuvre, from the intersection parameters in config/ai4r_policy.yaml."""
    enabled: bool = True
    """Whether TURN_LEFT, TURN_RIGHT and STRAIGHT markers start the manoeuvre. False only remembers the command
    (Memory.intersection_command), and the car stops where the approach lane ends, as before."""
    marker_to_entry_m: float = 0.0
    """How far past the marker, along the approach lane, its cone rows end and the intersection starts, in metres.
    Finite. 0.0: the marker stands beside the last cones."""
    left_turn_radius_m: float = 1.1
    """Radius of the arc turned along for a left turn, in metres, positive. At least the car's tightest left turn:
    the default is about 10% more than car 21's reported 1 m (Yuhao's measurement, docs/marker-commands.md). Car 20's
    measured left limit, 0.48 1/m, is a 2.1 m radius, too wide for a lab intersection."""
    right_turn_radius_m: float = 1.2
    """Radius of the arc for a right turn, in metres, positive: about 10% more than the 1.09 m of the measured
    0.92 1/m right limit."""
    cross_lane_width_m: float = 0.0
    """Width of the road turned into, in metres, finite and at least 0. 0.0: the approach lane's measured width."""
    corner_clearance_m: float = 0.03
    """Gap in metres kept between the car's body and a cone's edge at the inside corner and at the exit's far cone row,
    when planning the arc's start. At least 0. In the marker simulation's 0.8 m roads, 0.05 started the arc too late
    for the car's front corner to clear the far row (53/60 turns against 59/60)."""
    move_aside: bool = True
    """Whether the car keeps to the outside of the approach lane before a turn (left before a right turn)."""
    aside_clearance_m: float = 0.25
    """Distance in metres from the outside lane edge to the line followed before a turn (mpc.edge_clearance_m while
    moving aside). Positive. 0.25 leaves 0.075 m between the car's side and the cones' edges."""
    turn_speed_m_per_s: float = 0.3
    """Speed for the end of the approach, the arc and the exit, in m/s, positive."""
    hold_line_m: float = 0.8
    """Distance before the entry, in metres, from which the car follows the remembered approach centre line instead of
    the lane. At least 0."""
    lookahead_m: float = 0.5
    """Pure pursuit lookahead along the approach line, the arc and the exit line, in metres, positive."""
    handover_heading_deg: float = 60.0
    """How far the car must have turned, in degrees, before lane following may take over from the arc. 0-90."""
    exit_search_m: float = 1.5
    """How far along the exit line, past the arc's end or the far side of the intersection, the car looks for the exit
    lane before stopping, in metres, positive."""

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError(f"intersection.enabled must be true or false, not {self.enabled!r}")
        if not isinstance(self.move_aside, bool):
            raise ValueError(f"intersection.move_aside must be true or false, not {self.move_aside!r}")
        for name in ("left_turn_radius_m", "right_turn_radius_m", "turn_speed_m_per_s", "lookahead_m",
                     "exit_search_m", "aside_clearance_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"intersection.{name} must be positive and finite, not {value}")
        for name in ("cross_lane_width_m", "corner_clearance_m", "hold_line_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"intersection.{name} must be finite and at least 0, not {value}")
        if not math.isfinite(self.marker_to_entry_m):
            raise ValueError(f"intersection.marker_to_entry_m must be finite, not {self.marker_to_entry_m}")
        if not 0.0 <= self.handover_heading_deg <= 90.0:
            raise ValueError(f"intersection.handover_heading_deg must be in [0, 90], not {self.handover_heading_deg}")


def arc_start_m(approach_width_m: float, cross_width_m: float, radius_m: float, away_m: float, half_width_m: float,
                front_m: float, margin_m: float) -> tuple[float, bool]:
    """Where to start turning, as a distance along the approach line from the entry (negative before it).

    The arc is a quarter circle of base_link towards the turn side, starting with the car on the approach line's
    direction. Its end is on the exit lane's centre line when that start keeps the car's body margin_m from the
    inside corner cone (where the approach row on the turn side ends) and from the exit's far row; otherwise the
    nearest start that does, or halfway between the two limits when no start does. Turning, the car's inner side sweeps
    radius_m - half_width_m from the arc's centre and its outer front corner sqrt(front_m^2 + (radius_m +
    half_width_m)^2).

    :param approach_width_m: Width of the approach lane in metres.
    :param cross_width_m: Width of the road turned into in metres.
    :param radius_m: Radius of base_link's arc in metres.
    :param away_m: How far the car is from the approach centre line away from the turn side, in metres (negative
        towards it): keeping to the left before a right turn gives the arc more room round the corner.
    :param half_width_m: Half the car's width in metres.
    :param front_m: Distance in metres from base_link to the car's front.
    :param margin_m: Gap in metres to keep between the body and a cone's edge.
    :return: The start in metres, and whether it keeps both clearances.
    """
    ideal = cross_width_m / 2.0 - radius_m
    reach_m = math.hypot(front_m, radius_m + half_width_m)
    latest = cross_width_m - CONE_RADIUS_M - margin_m - reach_m
    inside_m = radius_m - away_m - approach_width_m / 2.0
    room_m = radius_m - half_width_m - CONE_RADIUS_M - margin_m
    earliest = -math.sqrt(room_m * room_m - inside_m * inside_m) if room_m >= abs(inside_m) else 0.0
    if earliest <= latest:
        return min(max(ideal, earliest), latest), True
    return (earliest + latest) / 2.0, False


def _wrapped(angle_rad: float) -> float:
    """
    :return: The angle within [-pi, pi].
    """
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _pursuit_curvature(pose: CarPose, target_x: float, target_y: float) -> float:
    """
    :return: The curvature in 1/m of the circle from the car's pose through the target point, tangent to the car's
        heading (pure pursuit).
    """
    dx, dy = target_x - pose.x_m, target_y - pose.y_m
    distance_sq = dx * dx + dy * dy
    if distance_sq < 1e-9:
        return 0.0
    left_m = -math.sin(pose.heading_rad) * dx + math.cos(pose.heading_rad) * dy
    return 2.0 * left_m / distance_sq


class IntersectionTurner:
    """Carries out the intersection manoeuvre a marker asks for. It keeps its state in CarMemory.intersection_turn."""

    def __init__(self, config: IntersectionConfig, mpc: MpcPolicy, car_width_m: float, car_front_m: float):
        """
        :param config: The intersection settings.
        :param mpc: The MPC, whose lane side and edge clearance the manoeuvre changes while approaching a turn.
        :param car_width_m: The car's width in metres (avoidance.car_width_m).
        :param car_front_m: Distance in metres from base_link to the car's front (obstacle_detection.car_front_m).
        """
        self.config = config
        self.mpc = mpc
        self.half_width_m = car_width_m / 2.0
        self.front_m = car_front_m
        self.clearance_m = self.half_width_m + CONE_RADIUS_M + config.corner_clearance_m
        self.arc_start_m: float | None = None
        """The planned start of the arc at the latest step, in metres along the approach line from the entry, or None
        without a turn being approached."""
        self.feasible = True
        """Whether that start keeps both clearances (arc_start_m)."""
        self.path_curvature_per_m = 0.0
        """Curvature in 1/m of the path the manoeuvre drove at the latest step, which the obstacle check looks along:
        the arc's while turning, otherwise the curvature it asked for (steering back onto the approach or exit line, so
        a car still angled from moving aside does not take the lane's own cones ahead for an obstacle)."""

    def start(self, command: MarkerCommand, marker: Position, lanes: LaneDetection | None, default_width_m: float,
              memory: CarMemory):
        """Places the intersection from a confirmed marker and starts approaching it.

        :param command: TURN_LEFT, TURN_RIGHT or STRAIGHT.
        :param marker: The marker's position in base_link.
        :param lanes: This step's lane, for the approach centre line, direction and width, or None.
        :param default_width_m: The lane width to assume without a lane (mpc.lane_width_m).
        :param memory: The car memory, with the car's pose.
        """
        pose = memory.data.pose
        if not self.config.enabled or pose is None or command.lane_side is not None:
            return
        turn = memory.data.intersection_turn
        if turn is not None and turn.phase is not TurnPhase.APPROACH:
            _LOGGER.info("Marker %s ignored: already turning", command.name)
            return
        heading = pose.heading_rad
        width = default_width_m
        centre_x, centre_y = pose.x_m, pose.y_m
        if lanes is not None and lanes.has_lane:
            heading = _wrapped(heading - lanes.heading_error_rad)
            width = lanes.lane_width_m
            centre_x += math.sin(heading) * lanes.lateral_error_m
            centre_y -= math.cos(heading) * lanes.lateral_error_m
        cos_h, sin_h = math.cos(pose.heading_rad), math.sin(pose.heading_rad)
        marker_x = pose.x_m + cos_h * marker.x - sin_h * marker.y
        marker_y = pose.y_m + sin_h * marker.x + cos_h * marker.y
        along = (math.cos(heading) * (marker_x - centre_x) + math.sin(heading) * (marker_y - centre_y)
                 + self.config.marker_to_entry_m)
        side_before = self.mpc.lane_side if turn is None else turn.lane_side_before
        memory.plan_turn(IntersectionTurn(command, TurnPhase.APPROACH, centre_x + along * math.cos(heading),
                                          centre_y + along * math.sin(heading), heading, width, side_before))
        if self.config.move_aside and command is not MarkerCommand.STRAIGHT:
            # Keep to the outside of the lane before turning
            self.mpc.lane_side = "LEFT" if command is MarkerCommand.TURN_RIGHT else "RIGHT"
            self.mpc.edge_clearance_m = self.config.aside_clearance_m
        else:
            self.mpc.lane_side, self.mpc.edge_clearance_m = side_before, self.mpc.config.edge_clearance_m
        _LOGGER.info("%s: intersection entry %.2f m ahead, lane %.2f m wide", command.name, along, width)

    def lane_cones(self, cones: list[ConeDetection], memory: CarMemory) -> list[ConeDetection]:
        """Leaves out of lane detection the cones that cannot belong to the approach lane while approaching: past the
        entry, or more than CORRIDOR_MARGIN_M outside its edges.

        :param cones: Cones in base_link.
        :param memory: The car memory.
        :return: The cones lane detection may use.
        """
        turn, pose = memory.data.intersection_turn, memory.data.pose
        if turn is None or pose is None or turn.phase is not TurnPhase.APPROACH:
            return cones
        cos_h, sin_h = math.cos(pose.heading_rad), math.sin(pose.heading_rad)
        cos_a, sin_a = math.cos(turn.heading_rad), math.sin(turn.heading_rad)
        half_m = turn.lane_width_m / 2.0 + CORRIDOR_MARGIN_M
        kept = []
        for cone in cones:
            dx = pose.x_m + cos_h * cone.pos.x - sin_h * cone.pos.y - turn.entry_x_m
            dy = pose.y_m + sin_h * cone.pos.x + cos_h * cone.pos.y - turn.entry_y_m
            if cos_a * dx + sin_a * dy <= CORRIDOR_MARGIN_M and abs(-sin_a * dx + cos_a * dy) <= half_m:
                kept.append(cone)
        return kept

    def step(self, lanes: LaneDetection | None, memory: CarMemory) -> tuple[float, float] | None:
        """Moves the manoeuvre on and chooses the command while it drives the car.

        :param lanes: This step's lane (from the cones lane_cones kept while approaching), or None.
        :param memory: The car memory.
        :return: The (speed in m/s, curvature in 1/m) to drive, None for lane following to drive, or (0.0, 0.0) when
            the exit lane was not found and the car must stop. The curvature is at most CURVATURE_HEADROOM times the
            arcs' each way.
        """
        self.path_curvature_per_m = None
        command = self._step(lanes, memory)
        if command is None:
            self.path_curvature_per_m = 0.0
            return None
        highest = CURVATURE_HEADROOM / self.config.left_turn_radius_m
        lowest = -CURVATURE_HEADROOM / self.config.right_turn_radius_m
        command = command[0], min(max(command[1], lowest), highest)
        if self.path_curvature_per_m is None:
            self.path_curvature_per_m = command[1]
        return command

    def _step(self, lanes: LaneDetection | None, memory: CarMemory) -> tuple[float, float] | None:
        """
        :return: step's command before the curvature is limited.
        """
        self.arc_start_m = None
        turn, pose = memory.data.intersection_turn, memory.data.pose
        if turn is None:
            return None
        if pose is None:
            self._finish(turn, memory)
            return None
        config = self.config
        cos_a, sin_a = math.cos(turn.heading_rad), math.sin(turn.heading_rad)
        dx, dy = pose.x_m - turn.entry_x_m, pose.y_m - turn.entry_y_m
        along_m, left_m = cos_a * dx + sin_a * dy, -sin_a * dx + cos_a * dy
        good_lane = (lanes is not None and lanes.has_lane
                     and abs(lanes.lateral_error_m) <= lanes.lane_width_m / 2.0 - HANDOVER_EDGE_M
                     and abs(lanes.heading_error_rad) <= HANDOVER_HEADING_RAD)
        if turn.phase is TurnPhase.APPROACH:
            if good_lane and along_m < -config.hold_line_m:
                turn = self._refine(turn, pose, lanes)
                memory.plan_turn(turn)
                cos_a, sin_a = math.cos(turn.heading_rad), math.sin(turn.heading_rad)
                dx, dy = pose.x_m - turn.entry_x_m, pose.y_m - turn.entry_y_m
                along_m, left_m = cos_a * dx + sin_a * dy, -sin_a * dx + cos_a * dy
            if turn.command is MarkerCommand.STRAIGHT:
                if along_m >= 0.0:
                    turn = self._exit(turn, turn.entry_x_m, turn.entry_y_m, turn.heading_rad)
                    memory.plan_turn(turn)
                    return self._exit_step(turn, pose, lanes, memory, good_lane)
            else:
                side = 1.0 if turn.command is MarkerCommand.TURN_LEFT else -1.0
                radius = self._radius(turn)
                cross = config.cross_lane_width_m or turn.lane_width_m
                self.arc_start_m, self.feasible = arc_start_m(turn.lane_width_m, cross, radius, -side * left_m,
                                                              self.half_width_m, self.front_m,
                                                              config.corner_clearance_m)
                if along_m >= self.arc_start_m:
                    if not self.feasible:
                        _LOGGER.warning("%s: the arc cannot keep clear of both the corner and the far cones",
                                        turn.command.name)
                    # The arc starts beside the car on the approach direction, wherever the car is pointing
                    across_m = left_m + side * radius
                    turn = replace(turn, phase=TurnPhase.ARC,
                                   arc_centre_x_m=turn.entry_x_m + along_m * cos_a - across_m * sin_a,
                                   arc_centre_y_m=turn.entry_y_m + along_m * sin_a + across_m * cos_a)
                    memory.plan_turn(turn)
                    _LOGGER.info("%s: turning, %.2f m from the entry", turn.command.name, along_m)
                    return self._arc_step(turn, pose, lanes, memory, good_lane)
            if along_m < -config.hold_line_m and lanes is not None and lanes.has_lane:
                return None
            # Hold the approach direction at the car's own offset from the centre line (kept inside the lane), so a car
            # keeping to one side before the turn stays there
            half_m = max(0.0, turn.lane_width_m / 2.0 - self.clearance_m)
            offset_m = min(max(left_m, -half_m), half_m)
            target = along_m + config.lookahead_m
            target_x = turn.entry_x_m + target * cos_a - offset_m * sin_a
            target_y = turn.entry_y_m + target * sin_a + offset_m * cos_a
            return config.turn_speed_m_per_s, _pursuit_curvature(pose, target_x, target_y)
        if turn.phase is TurnPhase.ARC:
            return self._arc_step(turn, pose, lanes, memory, good_lane)
        return self._exit_step(turn, pose, lanes, memory, good_lane)

    def _finish(self, turn: IntersectionTurn, memory: CarMemory):
        """Ends the manoeuvre and gives the MPC back its lane side and edge clearance; its line starts afresh in the
        exit lane."""
        memory.finish_turn()
        self.mpc.lane_side, self.mpc.edge_clearance_m = turn.lane_side_before, self.mpc.config.edge_clearance_m
        self.mpc.side_offset_m = None

    def _radius(self, turn: IntersectionTurn) -> float:
        """
        :return: The arc's radius for the turn, in metres.
        """
        if turn.command is MarkerCommand.TURN_LEFT:
            return self.config.left_turn_radius_m
        return self.config.right_turn_radius_m

    @staticmethod
    def _refine(turn: IntersectionTurn, pose: CarPose, lanes: LaneDetection) -> IntersectionTurn:
        """
        :return: The turn with its approach line's direction, sideways place and width moved REFINE_GAIN of the way to
            this step's lane. The entry stays the same distance along the line.
        """
        lane_heading = _wrapped(pose.heading_rad - lanes.heading_error_rad)
        centre_x = pose.x_m + math.sin(lane_heading) * lanes.lateral_error_m
        centre_y = pose.y_m - math.cos(lane_heading) * lanes.lateral_error_m
        cos_a, sin_a = math.cos(turn.heading_rad), math.sin(turn.heading_rad)
        shift_m = REFINE_GAIN * (-sin_a * (centre_x - turn.entry_x_m) + cos_a * (centre_y - turn.entry_y_m))
        return replace(turn, entry_x_m=turn.entry_x_m - sin_a * shift_m, entry_y_m=turn.entry_y_m + cos_a * shift_m,
                       heading_rad=_wrapped(turn.heading_rad + REFINE_GAIN * _wrapped(lane_heading - turn.heading_rad)),
                       lane_width_m=turn.lane_width_m + REFINE_GAIN * (lanes.lane_width_m - turn.lane_width_m))

    @staticmethod
    def _exit(turn: IntersectionTurn, x_m: float, y_m: float, heading_rad: float) -> IntersectionTurn:
        """
        :return: The turn in its exit phase, following the line through (x_m, y_m) in direction heading_rad.
        """
        return replace(turn, phase=TurnPhase.EXIT, exit_x_m=x_m, exit_y_m=y_m, exit_heading_rad=heading_rad)

    def _arc_step(self, turn: IntersectionTurn, pose: CarPose, lanes: LaneDetection | None, memory: CarMemory,
                  good_lane: bool) -> tuple[float, float] | None:
        """
        :return: The command along the arc, or None once lane following takes over.
        """
        config = self.config
        side = 1.0 if turn.command is MarkerCommand.TURN_LEFT else -1.0
        radius = self._radius(turn)
        turned = side * _wrapped(pose.heading_rad - turn.heading_rad)
        if good_lane and turned >= math.radians(config.handover_heading_deg):
            _LOGGER.info("%s: exit lane found after turning %.0f deg", turn.command.name, math.degrees(turned))
            self._finish(turn, memory)
            return None
        exit_heading = _wrapped(turn.heading_rad + side * math.pi / 2.0)
        if turned >= math.pi / 2.0 - math.radians(5.0):
            # The arc's end: follow the exit lane's centre line, cross_lane_width_m / 2 past the entry
            cross = config.cross_lane_width_m or turn.lane_width_m
            cos_a, sin_a = math.cos(turn.heading_rad), math.sin(turn.heading_rad)
            turn = self._exit(turn, turn.entry_x_m + cross / 2.0 * cos_a, turn.entry_y_m + cross / 2.0 * sin_a,
                              exit_heading)
            memory.plan_turn(turn)
            return self._exit_step(turn, pose, lanes, memory, good_lane)
        self.path_curvature_per_m = side / radius
        # Pure pursuit on the circle: the point lookahead_m further round it than the car
        angle = (math.atan2(pose.y_m - turn.arc_centre_y_m, pose.x_m - turn.arc_centre_x_m)
                 + side * config.lookahead_m / radius)
        return config.turn_speed_m_per_s, _pursuit_curvature(pose, turn.arc_centre_x_m + radius * math.cos(angle),
                                                             turn.arc_centre_y_m + radius * math.sin(angle))

    def _exit_step(self, turn: IntersectionTurn, pose: CarPose, lanes: LaneDetection | None, memory: CarMemory,
                   good_lane: bool) -> tuple[float, float] | None:
        """
        :return: The command along the exit line, None once lane following takes over, or (0.0, 0.0) to stop.
        """
        config = self.config
        cos_e, sin_e = math.cos(turn.exit_heading_rad), math.sin(turn.exit_heading_rad)
        along_m = cos_e * (pose.x_m - turn.exit_x_m) + sin_e * (pose.y_m - turn.exit_y_m)
        past_m = along_m
        if turn.command is MarkerCommand.STRAIGHT:
            past_m -= config.cross_lane_width_m or turn.lane_width_m
        if good_lane and (turn.command is not MarkerCommand.STRAIGHT or past_m >= 0.0):
            _LOGGER.info("%s: exit lane found", turn.command.name)
            self._finish(turn, memory)
            return None
        if past_m > config.exit_search_m:
            _LOGGER.warning("%s: no exit lane within %.1f m: stopping", turn.command.name, config.exit_search_m)
            self._finish(turn, memory)
            return 0.0, 0.0
        target = along_m + config.lookahead_m
        return config.turn_speed_m_per_s, _pursuit_curvature(pose, turn.exit_x_m + target * cos_e,
                                                             turn.exit_y_m + target * sin_e)
