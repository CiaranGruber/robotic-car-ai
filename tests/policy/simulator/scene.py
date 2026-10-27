"""Draws a simulated run in 3D for Foxglove: the floor, roads, cones, ArUco markers, boxes and the car.

Everything is logged through the Foxglove SDK's default context, so it reaches a live server (Foxglove's "Open
connection") and an MCAP recording alike. Topics:
- /tf: world to base_link, the car's true pose.
- /sim/world: floor, roads, cones, ArUco markers with their real DICT_4X4_50 patterns, and boxes, in world.
- /sim/car: the car's 3D model (chassis, wheels steering with the driven curvature, camera, lidar), in base_link.
- /sim/sensors: the camera's field of view and a line to each marker it can decode, in base_link.
- /cones/detected ... /car: the policy's own drawings (policy/visualisation.py), exactly as policy_node publishes
  them as policy_visualisation/markers on the car and scripts/mcap_export.py writes them.
- /lidar/points: the latest lidar scan. /policy/state and /sim/state: numbers for Plot panels. /sim/log: events.
"""
from __future__ import annotations

import json
import math

import foxglove
from foxglove.channels import FrameTransformsChannel, LogChannel, PointCloudChannel, SceneUpdateChannel
from foxglove.messages import (Color, CubePrimitive, CylinderPrimitive, FrameTransform, FrameTransforms,
                               LinePrimitive, LinePrimitiveLineType, Log, LogLevel, PackedElementField,
                               PackedElementFieldNumericType, Point3, PointCloud, Pose, Quaternion, SceneEntity,
                               SceneEntityDeletion, SceneEntityDeletionType, SceneUpdate, TextPrimitive, Timestamp,
                               TriangleListPrimitive, Vector3)
import numpy as np

from policy import visualisation
from policy.input_output import ConeColour
from scripts.mcap_export import _entity as policy_entity
from tests.policy.obstacle_simulation import CAMERA_FORWARD_M, LIDAR_FORWARD_M, LIDAR_HEIGHT_M
from tests.policy.simulator.simulation import POLICY_STATE, Simulation
from tests.policy.simulator.track import Track
from tests.policy.simulator.vehicle import FRONT_AXLE_M, REAR_AXLE_M, WHEEL_TRACK_M

WORLD_PERIOD_S = 2.0
"""How often the unchanging world is sent again, so a Foxglove window opened later still shows it, in seconds."""
CAR_PERIOD_S = 0.05
"""How often the car's pose and model are sent, in seconds (20 Hz)."""
CONE_HEIGHT_M, CONE_BASE_M = 0.15, 0.10
"""Size of a drawn lab cone in metres."""
CAMERA_HEIGHT_M, CAMERA_TILT_DEG = 0.30, 20.0
"""The camera mount's defaults (config/camera_mount.yaml): optical centre height and downward tilt."""

COLOURS = {
    "floor": (0.86, 0.87, 0.85, 1.0), "road": (0.72, 0.74, 0.73, 1.0), "blue": (0.10, 0.36, 0.85, 1.0),
    "yellow": (0.98, 0.78, 0.05, 1.0), "touched": (0.90, 0.15, 0.15, 1.0), "black": (0.02, 0.02, 0.02, 1.0),
    "white": (0.98, 0.98, 0.98, 1.0), "post": (0.45, 0.45, 0.45, 1.0), "box": (0.62, 0.45, 0.28, 1.0),
    "chassis": (0.16, 0.16, 0.18, 1.0), "shell": (0.80, 0.18, 0.14, 1.0), "glass": (0.10, 0.14, 0.20, 0.85),
    "tyre": (0.05, 0.05, 0.05, 1.0), "sensor": (0.08, 0.08, 0.10, 1.0), "view": (0.25, 0.55, 1.0, 0.7),
    "sight": (0.55, 0.20, 0.80, 0.9), "text": (0.10, 0.10, 0.10, 1.0),
}


def _colour(name: str) -> Color:
    r, g, b, a = COLOURS[name]
    return Color(r=r, g=g, b=b, a=a)


def _quaternion(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Quaternion:
    """
    :return: The rotation Rz(yaw) * Ry(pitch) * Rx(roll), in radians.
    """
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(x=sr * cp * cy - cr * sp * sy, y=cr * sp * cy + sr * cp * sy,
                      z=cr * cp * sy - sr * sp * cy, w=cr * cp * cy + sr * sp * sy)


def _pose(x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Pose:
    return Pose(position=Vector3(x=x, y=y, z=z), orientation=_quaternion(roll, pitch, yaw))


def _cube(x, y, z, size, colour, yaw=0.0, pitch=0.0) -> CubePrimitive:
    return CubePrimitive(pose=_pose(x, y, z, pitch=pitch, yaw=yaw), size=Vector3(x=size[0], y=size[1], z=size[2]),
                         color=_colour(colour))


def _cylinder(x, y, z, diameter, height, colour, roll=0.0, yaw=0.0, top_scale=1.0) -> CylinderPrimitive:
    return CylinderPrimitive(pose=_pose(x, y, z, roll=roll, yaw=yaw), size=Vector3(x=diameter, y=diameter, z=height),
                             bottom_scale=1.0, top_scale=top_scale, color=_colour(colour))


def _text(x, y, z, text, size=0.08) -> TextPrimitive:
    return TextPrimitive(pose=_pose(x, y, z), billboard=True, font_size=size, scale_invariant=False,
                         color=_colour("text"), text=text)


def marker_bits(marker_id: int) -> np.ndarray | None:
    """
    :return: The 6x6 cells of a DICT_4X4_50 marker including its black border, rows from the top and columns from
        the left as printed, 1 for white; None without OpenCV.
    """
    try:
        import cv2
    except ImportError:
        return None
    image = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), marker_id, 6)
    return (image > 127).astype(int)


def marker_names(settings: dict) -> dict[int, str]:
    """
    :return: The command each mapped marker ID gives, from the markers.*_id settings.
    """
    names = {}
    for key, value in settings.items():
        if key.startswith("markers.") and key.endswith("_id") and int(value) >= 0:
            names[int(value)] = key.removeprefix("markers.").removesuffix("_id").upper()
    return names


def _road_triangles(outline: np.ndarray) -> list[Point3]:
    """Triangles of a road outline made of its left edge followed by its right edge reversed (track.py)."""
    half = len(outline) // 2
    left, right = outline[:half], outline[half:][::-1]
    points = []
    for i in range(half - 1):
        a, b, c, d = left[i], left[i + 1], right[i + 1], right[i]
        for p in (a, b, c, a, c, d):
            points.append(Point3(x=float(p[0]), y=float(p[1]), z=0.001))
    return points


def world_entities(track: Track, settings: dict, touched: set[int], stamp: Timestamp) -> list[SceneEntity]:
    """
    :param touched: Indices of the cones the car has touched, drawn red.
    :return: The world's entities in the world frame.
    """
    xy = [(c.x, c.y) for c in track.cones] + [(m.x, m.y) for m in track.markers] + [track.start[:2]]
    xy += [tuple(p) for road in track.roads for p in road] + [(o.x, o.y) for o in track.obstacles]
    xy = np.array(xy)
    low, high = xy.min(axis=0) - 1.5, xy.max(axis=0) + 1.5
    floor = SceneEntity(timestamp=stamp, frame_id="world", id="floor", cubes=[
        _cube(*(low + high) / 2, -0.006, (*(high - low), 0.01), "floor")])
    roads = SceneEntity(timestamp=stamp, frame_id="world", id="roads", triangles=[
        TriangleListPrimitive(pose=_pose(0, 0, 0), points=_road_triangles(road), color=_colour("road"))
        for road in track.roads])
    cones = []
    for index, cone in enumerate(track.cones):
        colour = "touched" if index in touched else "blue" if cone.colour is ConeColour.BLUE else "yellow"
        cones.append(_cylinder(cone.x, cone.y, CONE_HEIGHT_M / 2, CONE_BASE_M, CONE_HEIGHT_M, colour, top_scale=0.15))
    bases = [_cube(c.x, c.y, 0.004, (CONE_BASE_M * 1.15, CONE_BASE_M * 1.15, 0.008),
                   "touched" if i in touched else "blue" if c.colour is ConeColour.BLUE else "yellow")
             for i, c in enumerate(track.cones)]
    cone_entity = SceneEntity(timestamp=stamp, frame_id="world", id="cones", cylinders=cones, cubes=bases)
    names = marker_names(settings)
    marker_cubes, marker_cylinders, marker_texts = [], [], []
    for marker in track.markers:
        facing = math.radians(marker.facing_deg)
        n = np.array([math.cos(facing), math.sin(facing)])
        right = np.array([-n[1], n[0]])
        board = marker.size_m * 1.4
        bottom = marker.height_m - board / 2
        if bottom > 0.01:
            marker_cylinders.append(_cylinder(*(marker.x - n[0] * 0.01, marker.y - n[1] * 0.01), bottom / 2, 0.012,
                                              bottom, "post"))
        marker_cubes.append(_cube(marker.x - n[0] * 0.003, marker.y - n[1] * 0.003, marker.height_m,
                                  (0.005, board, board), "white", yaw=facing))
        bits, cell = marker_bits(marker.id), marker.size_m / 6
        for row in range(6):
            for column in range(6):
                if bits is not None and bits[row, column]:
                    continue
                centre = np.array([marker.x, marker.y]) + n * 0.0015 + right * (column - 2.5) * cell
                marker_cubes.append(_cube(*centre, marker.height_m + (2.5 - row) * cell, (0.002, cell, cell), "black",
                                          yaw=facing))
        marker_texts.append(_text(marker.x, marker.y, marker.height_m + board / 2 + 0.08,
                                  f"ID {marker.id}: {names.get(marker.id, 'no command')}", 0.07))
    markers = SceneEntity(timestamp=stamp, frame_id="world", id="aruco", cubes=marker_cubes,
                          cylinders=marker_cylinders, texts=marker_texts)
    boxes = SceneEntity(timestamp=stamp, frame_id="world", id="obstacles", cubes=[
        _cube(o.x, o.y, o.height_m / 2, (o.length_m, o.width_m, o.height_m), "box", yaw=math.radians(o.yaw_deg))
        for o in track.obstacles])
    start = SceneEntity(timestamp=stamp, frame_id="world", id="title", texts=[
        _text(track.start[0] - 0.4 * math.cos(track.start[2]), track.start[1] - 0.4 * math.sin(track.start[2]), 0.05,
              f"start: {track.name}", 0.08)])
    return [floor, roads, cone_entity, markers, boxes, start]


def car_entity(sim: Simulation, stamp: Timestamp) -> SceneEntity:
    """
    :return: The car's 3D model in base_link: chassis, shell, wheels (front ones steered), camera on its mast, lidar.
    """
    car = sim.car
    steer = car.front_wheel_angle_rad
    wheel_d, wheel_w = car.wheel_diameter_m, 0.04
    wheels = [_cylinder(x, side * WHEEL_TRACK_M / 2, wheel_d / 2, wheel_d, wheel_w, "tyre", roll=math.pi / 2,
                        yaw=steer if x > 0 else 0.0)
              for x in (FRONT_AXLE_M, REAR_AXLE_M) for side in (1, -1)]
    length = car.body_front_m + car.body_rear_m
    middle = (car.body_front_m - car.body_rear_m) / 2
    cubes = [
        _cube(middle, 0, 0.045, (length * 0.95, car.body_width_m * 0.7, 0.02), "chassis"),
        _cube(middle - 0.01, 0, 0.095, (length * 0.82, car.body_width_m * 0.82, 0.07), "shell"),
        _cube(middle - 0.04, 0, 0.14, (length * 0.32, car.body_width_m * 0.62, 0.035), "glass"),
        _cube(CAMERA_FORWARD_M, 0, CAMERA_HEIGHT_M, (0.03, 0.09, 0.025), "sensor", pitch=math.radians(CAMERA_TILT_DEG)),
    ]
    cylinders = wheels + [
        _cylinder(CAMERA_FORWARD_M, 0, (0.13 + CAMERA_HEIGHT_M) / 2, 0.015, CAMERA_HEIGHT_M - 0.13, "post"),
        _cylinder(LIDAR_FORWARD_M, 0, LIDAR_HEIGHT_M, 0.07, 0.035, "sensor"),
    ]
    state = sim.car.state
    label = f"{state.speed_m_per_s:.2f} m/s   state {sim.node_state}"
    if sim.cones_touched:
        label += f"   {len(sim.cones_touched)} cones touched"
    return SceneEntity(timestamp=stamp, frame_id="base_link", id="car", frame_locked=True, cubes=cubes,
                       cylinders=cylinders, texts=[_text(middle, 0, 0.45, label, 0.06)])


def sensor_entity(sim: Simulation, stamp: Timestamp) -> SceneEntity:
    """
    :return: The camera's view on the floor and a line to each marker it can decode now, in base_link.
    """
    settings = sim.sensors.settings
    yaw, half = math.radians(settings.camera_yaw_deg), math.radians(settings.camera_fov_deg) / 2
    near, far = settings.camera_min_range_m, settings.camera_range_m
    angles = np.linspace(yaw - half, yaw + half, 25)
    outline = ([(near * math.cos(a), near * math.sin(a)) for a in angles]
               + [(far * math.cos(a), far * math.sin(a)) for a in angles[::-1]])
    view = LinePrimitive(type=LinePrimitiveLineType.LineLoop, pose=_pose(0, 0, 0), thickness=0.01,
                         points=[Point3(x=x, y=y, z=0.01) for x, y in outline], color=_colour("view"))
    lines = [view]
    pose = sim.car.state.pose
    for marker, seen in zip(sim.track.markers, sim.sensors.markers_seen(pose)):
        if seen:
            dx, dy = marker.x - pose[0], marker.y - pose[1]
            c, s = math.cos(pose[2]), math.sin(pose[2])
            lines.append(LinePrimitive(type=LinePrimitiveLineType.LineStrip, pose=_pose(0, 0, 0), thickness=0.008,
                                       points=[Point3(x=CAMERA_FORWARD_M, y=0, z=CAMERA_HEIGHT_M),
                                               Point3(x=c * dx + s * dy, y=-s * dx + c * dy, z=marker.height_m)],
                                       color=_colour("sight")))
    return SceneEntity(timestamp=stamp, frame_id="base_link", id="sensors", frame_locked=True, lines=lines)


class SceneWriter:
    """Logs one run to every open Foxglove sink: a live server, an MCAP file, or both."""

    def __init__(self, sim: Simulation, start_time_ns: int):
        """
        :param start_time_ns: Wall-clock time of simulation time zero, in nanoseconds.
        """
        self.sim, self.start_time_ns = sim, start_time_ns
        self.tf = FrameTransformsChannel("/tf")
        self.world = SceneUpdateChannel("/sim/world")
        self.car = SceneUpdateChannel("/sim/car")
        self.sensors = SceneUpdateChannel("/sim/sensors")
        self.layers = {layer: SceneUpdateChannel(f"/{layer}") for layer in visualisation.LAYERS}
        self.lidar = PointCloudChannel("/lidar/points")
        self.log = LogChannel("/sim/log")
        self.sim_state = foxglove.Channel("/sim/state", message_encoding="json")
        self.policy_state = foxglove.Channel("/policy/state", message_encoding="json")
        self.drawer = visualisation.StepDrawer()
        self.next_world_at, self.next_car_at = 0.0, 0.0
        self.events_logged, self.touched_drawn = 0, 0

    def time_ns(self) -> int:
        return self.start_time_ns + round(self.sim.time_s * 1e9)

    def stamp(self) -> Timestamp:
        ns = self.time_ns()
        return Timestamp(sec=ns // 1_000_000_000, nsec=ns % 1_000_000_000)

    def publish(self, happened: dict[str, bool], force: bool = False):
        """Logs what changed in the latest simulation step.

        :param happened: Simulation.step's result.
        :param force: Send the world and the car now even if they were sent recently, for a reset.
        """
        sim, now, ns, stamp = self.sim, self.sim.time_s, self.time_ns(), self.stamp()
        if force or now >= self.next_world_at or len(sim.cones_touched) != self.touched_drawn:
            self.next_world_at = now + WORLD_PERIOD_S
            self.touched_drawn = len(sim.cones_touched)
            self.world.log(SceneUpdate(entities=world_entities(sim.track, sim.settings, sim.cones_touched, stamp)),
                           log_time=ns)
        if force or now >= self.next_car_at:
            self.next_car_at = now + CAR_PERIOD_S
            x, y, heading = sim.car.state.pose
            self.tf.log(FrameTransforms(transforms=[FrameTransform(
                timestamp=stamp, parent_frame_id="world", child_frame_id="base_link",
                translation=Vector3(x=x, y=y, z=0.0), rotation=_quaternion(yaw=heading))]), log_time=ns)
            self.car.log(SceneUpdate(entities=[car_entity(sim, stamp)]), log_time=ns)
            self.sensors.log(SceneUpdate(entities=[sensor_entity(sim, stamp)]), log_time=ns)
            self.sim_state.log(json.dumps(self._sim_state()), log_time=ns)
        if happened.get("policy") and sim.observations is not None:
            layers = self.drawer.draw(sim.policy, sim.observations, sim.actions)
            for name, layer in layers.items():
                self.layers[name].log(SceneUpdate(
                    deletions=[SceneEntityDeletion(timestamp=stamp, type=SceneEntityDeletionType.All)],
                    entities=[policy_entity(name, layer, stamp)]), log_time=ns)
            self.policy_state.log(json.dumps(self._policy_state()), log_time=ns)
        if happened.get("lidar") and sim.lidar.latest is not None:
            points = sim.lidar.latest[1]
            self.lidar.log(PointCloud(
                timestamp=stamp, frame_id="base_link", pose=_pose(0, 0, 0), point_stride=12,
                fields=[PackedElementField(name=axis, offset=4 * i, type=PackedElementFieldNumericType.Float32)
                        for i, axis in enumerate("xyz")],
                data=np.array([(p.pos.x, p.pos.y, p.pos.z) for p in points], dtype="<f4").tobytes()), log_time=ns)
        for event in sim.events[self.events_logged:]:
            self.log.log(Log(timestamp=stamp, level=LogLevel.Info, message=f"{event.time_s:.2f} s: {event.text}",
                             name="simulator"), log_time=ns)
        self.events_logged = len(sim.events)

    def _sim_state(self) -> dict:
        sim, state = self.sim, self.sim.car.state
        return {"time_s": sim.time_s, "x_m": state.x, "y_m": state.y, "heading_deg": math.degrees(state.heading),
                "speed_m_per_s": state.speed_m_per_s, "curvature_per_m": state.curvature_per_m,
                "applied": {"drive": state.drive_applied, "steering": state.steering_applied},
                "distance_m": state.distance_m, "node_state": sim.node_state, "node_reason": sim.node_reason,
                "driving": sim.node_state == POLICY_STATE, "cones_touched": len(sim.cones_touched),
                "obstacles_touched": len(sim.obstacles_touched), "ended_because": sim.ended_because}

    def _policy_state(self) -> dict:
        sim, policy = self.sim, self.sim.policy
        lanes, actions, mpc = policy.lanes, sim.actions, policy.action_policy
        finite = lambda v: v if v is not None and math.isfinite(v) else None
        intersection = sim.memory.data.intersection
        return {
            "actions": {"drive": actions.drive_action, "steering": actions.steering_action},
            "cones": {"detected": len(sim.observations.cones or []), "used": len(policy.cones or [])},
            "lane": None if lanes is None else {
                "status": lanes.status.name, "lateral_error_m": finite(lanes.lateral_error_m),
                "heading_error_deg": None if lanes.heading_error_rad is None
                else finite(math.degrees(lanes.heading_error_rad)),
                "lane_width_m": finite(lanes.lane_width_m), "path_curvature_per_m": finite(lanes.path_curvature_per_m)},
            "marker_command": None if policy.marker_command is None else policy.marker_command.name,
            "next_intersection": None if policy.intersection_command is None else policy.intersection_command.name,
            "intersection_phase": None if intersection is None else intersection.phase,
            "obstacle_distance_m": finite(policy.obstacle_distance_m),
            "mpc": {"stopped_for_obstacle": mpc.stopped_for_obstacle, "stopped_for_lane_loss": mpc.stopped_for_lane_loss,
                    "solve_time_s": finite(mpc.last_solve_time_s)},
        }
