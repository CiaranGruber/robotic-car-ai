"""
visualisation.py

This file draws what the movement policy saw and decided at one policy step, as plain shapes on the ground in the car's
body frame (base_link): +x forwards, +y left, metres. It has no ROS or Foxglove types, so policy_node can publish the
shapes live as ROS markers and scripts/mcap_export.py can write them to an MCAP file, and both show the same thing.

Observed cones/edges describe sensor acquisition time. The active selected junction reference and
controller prediction use the estimated current body pose, and are labelled separately from those measurements.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from policy.input_output import CarActions, CarObservations, ConeColour
from policy.lane_detection.lanes import LaneArc, LaneDetection
from policy.markers.commands import MarkerCommand
from policy.memory import TurnPhase
from policy.obstacle_detection.obstacles import LaneObstacle
from policy.policy_runner import MovementPolicy

Colour = tuple[float, float, float, float]
"""Red, green, blue and opacity, each in [0, 1]."""

COLOURS: dict[str, Colour] = {
    "left": (0.18, 0.5, 0.93, 1.0),
    "right": (0.95, 0.79, 0.3, 1.0),
    "centre": (0.2, 0.85, 0.3, 1.0),
    "detected": (0.9, 0.2, 0.2, 1.0),
    "obstacle": (1.0, 0.55, 0.1, 1.0),
    "remembered_obstacle": (1.0, 0.55, 0.1, 0.4),
    "remembered_cone": (0.6, 0.6, 0.6, 0.6),
    "corridor": (0.6, 0.6, 0.6, 0.8),
    "avoidance": (0.9, 0.3, 0.9, 1.0),
    "mpc": (0.2, 0.85, 0.95, 1.0),
    "fiducial": (0.7, 0.4, 1.0, 0.8),
    "intersection": (1.0, 0.45, 0.2, 1.0),
    "car": (0.85, 0.85, 0.85, 0.35),
    "text": (1.0, 1.0, 1.0, 0.6),
}
"""Drawing colours. The lane colours match scripts/lane_overlay_node.py. Foxglove draws text on a black box with the
text's opacity, so the text is see-through to keep what is under it visible."""

CONE_COLOURS = {ConeColour.BLUE: COLOURS["left"], ConeColour.YELLOW: COLOURS["right"]}
"""Colour of a used cone."""

RECALLED_OPACITY = 0.45
"""Opacity of a remembered cone that lane detection used, drawn in its colour, so it stands apart from a seen one."""

LINE_WIDTH_M = 0.03
"""Width of the drawn lines in metres."""

LINE_STEP_M = 0.1
"""Distance in metres along a drawn line between its points."""

TEXT_HEIGHT_M = 0.1
"""Height of one line of drawn text in metres, so text grows and shrinks with the view like the other shapes."""

SUMMARY_BEHIND_M = 1.0
"""Distance in metres from the car's rear to the middle of the step's summary. The summary's lines are under 30
characters, so it stays behind the car and off the lane ahead whichever way the view is turned."""

LAYERS = ("cones/detected", "cones/used", "memory/cones", "lane/observed", "lane", "obstacles", "memory/obstacles",
          "plan/avoidance", "plan/mpc", "plan/intersection", "fiducials", "car")
"""The layers of a drawing, in drawing order:

- cones/detected: every detected cone, small and red, under cones/used.
- cones/used: the cones of the batch the lane detection used, after the cone filter and camera yaw correction, in their
  colour.
- memory/cones: the cones the local map remembers. Those the lane detection also used, because the batch does not show
  them (beside the car, or hidden), are in their colour but faded; the others are small and grey.
- lane/observed: the acquisition-time measured edges/arc while a selected intersection path is active, faded.
- lane: the active current-body selected intersection centre path, otherwise the measured lane edges/centre.
- obstacles: objects inside the lane (orange outlines), or the corridor ahead without a lane (grey) and the nearest
  return in it.
- memory/obstacles: the obstacles the avoidance remembers beside the car (faded orange outlines), where the lidar no
  longer sees them.
- plan/avoidance: the line across the lane planned past the obstacles (magenta). Away from obstacles it is the usual
  line (mpc.lane_side), which covers the centre line when lane_side is "CENTRE" and lane_offset_m is 0.
- plan/mpc: the path the MPC predicted for the car (cyan), from the curvatures it solved, as long as the distance it
  predicted at the measured speed: a point while the car stands. Empty when it has not solved since the last drawing.
- plan/intersection: the intersection manoeuvre a marker asked for (orange): the entry across the approach lane, and
  the arc planned from its start round the corner (or the line across, going straight), then the exit line.
- fiducials: fiducial marker poses with their ids.
- car: the car's outline, and a text summary of the step behind it.
"""


@dataclass(frozen=True)
class Line:
    """A line on the ground through its points."""
    points: list[tuple[float, float]]
    """(x, y) points in metres."""
    colour: Colour
    closed: bool = False
    """True when the line goes back from the last point to the first."""


@dataclass(frozen=True)
class Sphere:
    """A ball resting at a point on the ground."""
    x: float
    y: float
    diameter_m: float
    colour: Colour


@dataclass(frozen=True)
class Box:
    """A box around a centre point."""
    position: tuple[float, float, float]
    """Centre (x, y, z) in metres."""
    size: tuple[float, float, float]
    """Length along x, width along y and height along z, in metres, before the rotation."""
    colour: Colour
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    """Rotation as a quaternion (x, y, z, w)."""


@dataclass(frozen=True)
class Text:
    """A label that faces the viewer."""
    position: tuple[float, float, float]
    """(x, y, z) in metres."""
    text: str


@dataclass
class Layer:
    """The shapes of one layer."""
    lines: list[Line] = field(default_factory=list)
    spheres: list[Sphere] = field(default_factory=list)
    boxes: list[Box] = field(default_factory=list)
    texts: list[Text] = field(default_factory=list)


class StepDrawer:
    """Draws policy steps. It remembers the MPC's solve count, so a plan is only drawn once it is new."""

    def __init__(self):
        self._solve_count = 0

    def draw(self, policy: MovementPolicy, observations: CarObservations, actions: CarActions) -> dict[str, Layer]:
        """Draws the latest policy step.

        :param policy: The movement policy just after it ran the step.
        :param observations: The observations of the step.
        :param actions: The actions the policy chose.
        :return: The step's shapes for each of LAYERS.
        """
        solved = policy.action_policy.solve_count != self._solve_count
        self._solve_count = policy.action_policy.solve_count
        return {
            "cones/detected": Layer(spheres=[Sphere(cone.pos.x, cone.pos.y, 0.05, COLOURS["detected"])
                                             for cone in observations.cones or []]),
            "cones/used": Layer(spheres=[Sphere(cone.pos.x, cone.pos.y, 0.08, CONE_COLOURS[cone.colour])
                                         for cone in policy.cones or []]),
            "memory/cones": _remembered_cone_layer(policy),
            "lane/observed": Layer(lines=[Line(line.points, (*line.colour[:3], .25), line.closed)
                                           for line in _lane_lines(policy.lanes)] if policy.selected_lane is not None else []),
            "lane": Layer(lines=[Line(list(policy.selected_lane.points), COLOURS["centre"])]
                          if policy.selected_lane is not None else _lane_lines(policy.lanes)),
            "obstacles": _obstacle_layer(policy),
            "memory/obstacles": _remembered_obstacle_layer(policy),
            "plan/avoidance": Layer(lines=_avoidance_lines(policy)),
            "plan/mpc": Layer(lines=_mpc_lines(policy) if solved or policy.selected_lane is not None else []),
            "plan/intersection": Layer(lines=_intersection_lines(policy)),
            "fiducials": _fiducial_layer(observations),
            "car": _car_layer(policy, observations, actions),
        }


def summary(policy: MovementPolicy, observations: CarObservations, actions: CarActions) -> str:
    """
    :return: A short text of the step's lane, obstacle, speed and actions, lane side, next intersection command and the
        phase of the intersection manoeuvre under way.
    """
    lanes = policy.lanes
    lines = [f"t {observations.policy.policy_elapsed_s:.2f} s  "
             + ("open space" if lanes is None else lanes.status.name)]
    if lanes is not None and lanes.has_lane:
        lines += [f"lateral {lanes.lateral_error_m:+.2f} m",
                  f"heading {math.degrees(lanes.heading_error_rad):+.1f} deg"]
    if policy.selected_lane is not None:
        selected = policy.selected_lane
        lines += [f"selected {selected.command.name} {selected.phase}",
                  f"route evidence {selected.evidence_age_s:.1f} s"]
    obstacle = policy.obstacle_distance_m
    speed = observations.car.wheel_speed
    intersection = policy.intersection_command
    return "\n".join(lines + [
        f"memory {len(policy.local_map.map_cones)} cones, {len(policy.local_map.recalled)} used",
        "obstacle " + ("unknown" if obstacle is None else "clear" if math.isinf(obstacle) else f"{obstacle:.2f} m"),
        "speed " + ("unknown" if speed is None else f"{float(speed):.2f} m/s"),
        f"drive {actions.drive_action:+.2f}  steer {actions.steering_action:+.2f}",
        f"side {policy.action_policy.lane_side}  next intersection " + ("none" if intersection is None
                                                                        else intersection.name)]
        + ([] if policy.intersection_turn is None else [f"turn {policy.intersection_turn.phase.name.lower()}"]))


def _along(start_m: float, end_m: float) -> list[float]:
    """
    :return: Distances from start_m to end_m, at most LINE_STEP_M apart.
    """
    steps = max(1, math.ceil((end_m - start_m) / LINE_STEP_M))
    return [start_m + (end_m - start_m) * index / steps for index in range(steps + 1)]


def _lane_point(centre_line: LaneArc, s_m: float, left_m: float) -> tuple[float, float]:
    """
    :param centre_line: The lane's centre line.
    :param s_m: Distance in metres along the centre line from beside base_link.
    :param left_m: Distance in metres left of the centre line.
    :return: The point in base_link.
    """
    x, y = centre_line.point_at(s_m)
    heading = centre_line.heading_at(s_m)
    return x - left_m * math.sin(heading), y + left_m * math.cos(heading)


def _lane_lines(lanes: LaneDetection | None) -> list[Line]:
    """
    :return: The lane's edges where cones were found, and its centre line up to where the lane is visible.
    """
    if lanes is None:
        return []
    lines = [Line([edge.arc.point_at(s_m) for s_m in _along(edge.nearest_m, edge.farthest_m)], COLOURS[side])
             for side, edge in (("left", lanes.left_edge), ("right", lanes.right_edge)) if edge is not None]
    if lanes.has_lane:
        lines.append(Line([lanes.centre_line.point_at(s_m) for s_m in _along(0.0, lanes.visible_until_m)],
                          COLOURS["centre"]))
    return lines


def _obstacle_layer(policy: MovementPolicy) -> Layer:
    """
    :return: The objects inside the lane, or without a lane the corridor ahead and the nearest return in it.
    """
    car_front_m = policy.obstacle_detection_config.car_front_m
    if policy.lane_obstacles is not None:
        return Layer(lines=[Line(_lane_outline(policy.lanes.centre_line, car_front_m, obstacle), COLOURS["obstacle"],
                                 closed=True) for obstacle in policy.lane_obstacles])
    obstacles = policy.obstacles
    if obstacles is None:
        return Layer()
    config = policy.obstacle_detection_config
    path = LaneArc(0.0, 0.0, 0.0, obstacles.path_curvature_per_m)
    along = _along(car_front_m, car_front_m + config.corridor_length_m)
    corridor = ([_lane_point(path, s_m, -config.corridor_half_width_m) for s_m in along]
                + [_lane_point(path, s_m, config.corridor_half_width_m) for s_m in reversed(along)])
    point = obstacles.in_path_point
    return Layer(lines=[Line(corridor, COLOURS["corridor"], closed=True)],
                 spheres=[] if point is None else [Sphere(point.x, point.y, 0.1, COLOURS["obstacle"])])


def _lane_outline(centre_line: LaneArc, car_front_m: float, obstacle: LaneObstacle) -> list[tuple[float, float]]:
    """
    :param centre_line: The lane's centre line.
    :param car_front_m: Distance in metres from base_link's origin forwards to the car's front, from which the
        obstacle's distances are measured.
    :param obstacle: An obstacle in the lane.
    :return: The outline of the stretch of lane it covers, going around it.
    """
    near_m, far_m = car_front_m + obstacle.distance_m, car_front_m + obstacle.far_distance_m
    return ([_lane_point(centre_line, s_m, obstacle.right_m) for s_m in _along(near_m, far_m)]
            + [_lane_point(centre_line, s_m, obstacle.left_m) for s_m in reversed(_along(near_m, far_m))])


def _remembered_cone_layer(policy: MovementPolicy) -> Layer:
    """
    :return: The remembered cones: those lane detection used in their colour, faded, and the others small and grey.
    """
    recalled = {id(cone) for cone in policy.local_map.recalled}
    spheres = []
    for cone in policy.local_map.map_cones:
        if id(cone) in recalled:
            spheres.append(Sphere(cone.pos.x, cone.pos.y, 0.08, CONE_COLOURS[cone.colour][:3] + (RECALLED_OPACITY,)))
        else:
            spheres.append(Sphere(cone.pos.x, cone.pos.y, 0.04, COLOURS["remembered_cone"]))
    return Layer(spheres=spheres)


def _remembered_obstacle_layer(policy: MovementPolicy) -> Layer:
    """
    :return: The outlines of the obstacles the avoidance remembers, in this step's lane; empty without a lane.
    """
    lanes = policy.lanes
    if lanes is None or not lanes.has_lane:
        return Layer()
    car_front_m = policy.obstacle_detection_config.car_front_m
    return Layer(lines=[Line(_lane_outline(lanes.centre_line, car_front_m, obstacle), COLOURS["remembered_obstacle"],
                             closed=True) for obstacle in policy.avoidance.remembered])


def _avoidance_lines(policy: MovementPolicy) -> list[Line]:
    """
    :return: The line planned across the lane past the obstacles, up to where the lane is visible.
    """
    path, lanes = policy.lane_path, policy.lanes
    if path is None or lanes is None or not lanes.has_lane:
        return []
    end_m = max([lanes.visible_until_m] + [segment.end_m for segment in path.segments])
    return [Line([_lane_point(lanes.centre_line, s_m, path.offset_at(s_m)) for s_m in _along(0.0, end_m)],
                 COLOURS["avoidance"])]


def _mpc_lines(policy: MovementPolicy) -> list[Line]:
    """
    :return: The car's path over the MPC horizon, from base_link, following the solved curvature of each step.
    """
    mpc = policy.action_policy
    selected = policy.selected_lane
    curvatures = selected.mpc_curvatures_per_m if selected is not None else mpc.plan_curvatures_per_m
    if curvatures is None:
        return []
    step_m = selected.prediction_step_m if selected is not None else mpc.plan_step_distance_m
    arc = LaneArc(0.0, 0.0, 0.0, 0.0)
    points = [(0.0, 0.0)]
    for curvature in curvatures:
        arc = LaneArc(arc.x, arc.y, arc.heading_rad, float(curvature))
        points += [arc.point_at(s_m) for s_m in _along(0.0, step_m)[1:]]
        x, y = arc.point_at(step_m)
        arc = LaneArc(x, y, arc.heading_at(step_m), 0.0)
    return [Line(points, COLOURS["mpc"])]


def _intersection_lines(policy: MovementPolicy) -> list[Line]:
    """
    :return: The intersection manoeuvre's entry, planned arc or line across, and exit line, in base_link at the step.
    """
    turn, pose = policy.intersection_turn, policy.pose
    if turn is None or pose is None:
        return []
    cos_p, sin_p = math.cos(pose.heading_rad), math.sin(pose.heading_rad)

    def local(x_m: float, y_m: float) -> tuple[float, float]:
        dx, dy = x_m - pose.x_m, y_m - pose.y_m
        return cos_p * dx + sin_p * dy, -sin_p * dx + cos_p * dy

    cos_a, sin_a = math.cos(turn.heading_rad), math.sin(turn.heading_rad)

    def approach(along_m: float, left_m: float) -> tuple[float, float]:
        return local(turn.entry_x_m + along_m * cos_a - left_m * sin_a,
                     turn.entry_y_m + along_m * sin_a + left_m * cos_a)

    half = turn.lane_width_m / 2.0
    colour = COLOURS["intersection"]
    lines = [Line([approach(0.0, -half), approach(0.0, half)], colour)]
    config = policy.turner.config
    if turn.phase is TurnPhase.EXIT:
        cos_e, sin_e = math.cos(turn.exit_heading_rad), math.sin(turn.exit_heading_rad)
        end_m = config.exit_search_m + (turn.lane_width_m if turn.command is MarkerCommand.STRAIGHT else 0.0)
        lines.append(Line([local(turn.exit_x_m + s_m * cos_e, turn.exit_y_m + s_m * sin_e)
                           for s_m in _along(0.0, end_m)], colour))
        return lines
    if turn.command is MarkerCommand.STRAIGHT:
        lines.append(Line([approach(s_m, 0.0) for s_m in _along(-config.hold_line_m, turn.lane_width_m)], colour))
        return lines
    side = 1.0 if turn.command is MarkerCommand.TURN_LEFT else -1.0
    radius = config.left_turn_radius_m if side > 0.0 else config.right_turn_radius_m
    if turn.phase is TurnPhase.ARC:
        centre = (turn.arc_centre_x_m, turn.arc_centre_y_m)
    elif policy.turner.arc_start_m is not None:
        # The arc as planned from the car's offset now
        dx, dy = pose.x_m - turn.entry_x_m, pose.y_m - turn.entry_y_m
        across = -sin_a * dx + cos_a * dy + side * radius
        start = policy.turner.arc_start_m
        centre = (turn.entry_x_m + start * cos_a - across * sin_a, turn.entry_y_m + start * sin_a + across * cos_a)
    else:
        return lines
    first = turn.heading_rad - side * math.pi / 2.0
    lines.append(Line([local(centre[0] + radius * math.cos(first + side * angle),
                             centre[1] + radius * math.sin(first + side * angle))
                       for angle in _along(0.0, math.pi / 2.0)], colour))
    return lines


def _fiducial_layer(observations: CarObservations) -> Layer:
    """
    :return: A square at each fiducial marker with a pose, labelled with its id.
    """
    fiducials = observations.fiducials
    layer = Layer()
    for fiducial in [] if fiducials is None else fiducials.detections:
        if fiducial.pose is None:
            continue
        position, orientation = fiducial.pose.position, fiducial.pose.orientation
        centre = (position.x, position.y, position.z)
        size = fiducial.marker_size_m
        layer.boxes.append(Box(centre, (size, size, 0.01), COLOURS["fiducial"],
                               (orientation.x, orientation.y, orientation.z, orientation.w)))
        layer.texts.append(Text(centre, f"id {fiducial.id}"))
    return layer


def _car_layer(policy: MovementPolicy, observations: CarObservations, actions: CarActions) -> Layer:
    """
    :return: The car's footprint from its rear to its front around base_link, and the step's summary behind it.
    """
    front_m = policy.obstacle_detection_config.car_front_m
    rear_m = policy.avoidance.config.car_rear_m
    return Layer(
        boxes=[Box(((front_m - rear_m) / 2.0, 0.0, 0.05), (front_m + rear_m, policy.avoidance.config.car_width_m, 0.1),
                   COLOURS["car"])],
        texts=[Text((-rear_m - SUMMARY_BEHIND_M, 0.0, 0.3), summary(policy, observations, actions))])
