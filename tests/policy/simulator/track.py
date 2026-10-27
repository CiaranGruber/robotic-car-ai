"""The simulated test area: cone roads, loose cones, ArUco markers and boxes, read from a track file.

A track file (tracks/*.yaml, format in README.md) lays out one test area in world coordinates: metres, +x east,
+y north, angles in degrees anticlockwise from +x. Roads are built with the same generators as the existing
simulations, so the cones sit where those tests put them:
- lane: a cone lane of straight and circular-arc pieces (curved_lanes.CurvedLane);
- cross: an equal-width perpendicular cross, or a T with one arm closed (intersection_simulation.CrossIntersection).
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from pathlib import Path

import numpy as np
import yaml

from policy.input_output import ConeColour
from tests.policy.curved_lanes import CurvedLane
from tests.policy.intersection_simulation import CrossIntersection

ROOT = Path(__file__).resolve().parents[3]
TRACKS_DIR = Path(__file__).resolve().parent / "tracks"
"""Folder of the bundled track files."""


@dataclass(frozen=True)
class Cone:
    """One cone standing on the floor."""
    x: float
    y: float
    colour: ConeColour


@dataclass(frozen=True)
class Marker:
    """One printed ArUco marker standing upright, for example on a sign or a box side."""
    id: int
    """Its DICT_4X4_50 ID (0-49)."""
    x: float
    y: float
    facing_deg: float
    """Direction its printed face points to, in degrees. 180 faces a car driving east towards it."""
    size_m: float
    """Outer edge of its black square in metres, as config/aruco_detector.yaml's marker sizes."""
    height_m: float
    """Height of its centre above the floor in metres."""


@dataclass(frozen=True)
class Obstacle:
    """A box standing on the floor. The lidar sees it, it hides cones from the camera and the car can hit it."""
    x: float
    y: float
    length_m: float
    width_m: float
    height_m: float
    yaw_deg: float = 0.0

    def polygon(self) -> np.ndarray:
        """
        :return: The world (x, y) of its four floor corners, going around it.
        """
        c, s = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        local = np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]]) * (self.length_m / 2, self.width_m / 2)
        return local @ np.array([[c, s], [-s, c]]) + (self.x, self.y)


@dataclass(frozen=True)
class Track:
    """Everything in one simulated test area, and how a run on it starts."""
    name: str
    description: str
    cones: tuple[Cone, ...]
    markers: tuple[Marker, ...] = ()
    obstacles: tuple[Obstacle, ...] = ()
    roads: tuple[np.ndarray, ...] = ()
    """Floor outline of each road as a world (x, y) polygon, only for drawing."""
    start: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """The car's starting world x, y in metres and heading in radians."""
    duration_s: float = 30.0
    """Longest run on this track in seconds."""
    policy_settings: dict = field(default_factory=dict)
    """Settings by full parameter name (such as intersection.enabled) that this track needs on top of
    config/ai4r_policy.yaml, like a car-only override file."""


def _place(points: np.ndarray, origin: tuple[float, float], heading_rad: float) -> np.ndarray:
    """
    :return: The Nx2 points, given in a frame at origin turned by heading_rad, in world coordinates.
    """
    c, s = math.cos(heading_rad), math.sin(heading_rad)
    return np.asarray(points, dtype=float).reshape(-1, 2) @ np.array([[c, s], [-s, c]]) + origin


def _colour(name: str) -> ConeColour:
    try:
        return ConeColour[str(name).upper()]
    except KeyError:
        raise ValueError(f"cone colour must be BLUE or YELLOW, not {name!r}") from None


def _lane(spec: dict) -> tuple[list[Cone], np.ndarray]:
    """A cone lane of pieces, starting at spec['start'] with its first cone pair, pointing along spec['heading_deg']."""
    pieces = tuple((p[0], *map(float, p[1:])) for p in spec["pieces"])
    lane = CurvedLane(pieces=pieces, width_m=float(spec.get("width_m", 0.8)),
                      cone_spacing_m=float(spec.get("cone_spacing_m", 0.2)),
                      left_colour=_colour(spec.get("left_colour", "BLUE")))
    origin, heading = tuple(spec.get("start", (0.0, 0.0))), math.radians(float(spec.get("heading_deg", 0.0)))
    raw = lane.cones()
    xy = _place([(x, y) for _, _, x, y in raw], origin, heading)
    cones = [Cone(float(x), float(y), colour) for (colour, _, _, _), (x, y) in zip(raw, xy)]
    line = lane.centre_line()
    keep = line["s"] >= -0.3  # Draw the road from just behind the first cone pair
    centre = np.stack([line["x"][keep], line["y"][keep]], axis=1)
    normal = np.stack([-np.sin(line["heading"][keep]), np.cos(line["heading"][keep])], axis=1) * lane.width_m / 2
    outline = np.vstack([centre + normal, (centre - normal)[::-1]])
    return cones, _place(outline, origin, heading)


def _cross(spec: dict) -> tuple[list[Cone], list[np.ndarray]]:
    """A perpendicular cross, or a T with closed_exit set, centred at spec['centre'], approached along heading_deg."""
    cross = CrossIntersection(width_m=float(spec.get("width_m", 2.4)), arm_length_m=float(spec.get("arm_length_m", 6.0)),
                              cone_spacing_m=float(spec.get("cone_spacing_m", 0.2)),
                              corner_anchored=bool(spec.get("corner_anchored", False)),
                              closed_exit=spec.get("closed_exit"))
    origin, heading = tuple(spec.get("centre", (0.0, 0.0))), math.radians(float(spec.get("heading_deg", 0.0)))
    raw = cross.cones()
    xy = _place([(x, y) for x, y, _ in raw], origin, heading)
    cones = [Cone(float(x), float(y), colour) for (_, _, colour), (x, y) in zip(raw, xy)]
    roads = [_place([(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)], origin, heading)
             for x0, y0, w, h in cross.road_rectangles()]
    return cones, roads


def default_marker_size_m() -> float:
    """
    :return: The aruco_detector's default marker size from config/aruco_detector.yaml, in metres.
    """
    detector = yaml.safe_load((ROOT / "config/aruco_detector.yaml").read_text())
    return float(detector["/**/aruco_detector"]["ros__parameters"]["default_marker_size_m"])


def track_path(name_or_path: str | Path) -> Path:
    """
    :param name_or_path: A bundled track's name, such as cross, or a path to a track file.
    :return: The track file.
    """
    path = Path(name_or_path)
    if path.suffix in (".yaml", ".yml") and path.is_file():
        return path
    bundled = TRACKS_DIR / f"{name_or_path}.yaml"
    if bundled.is_file():
        return bundled
    names = ", ".join(sorted(p.stem for p in TRACKS_DIR.glob("*.yaml")))
    raise FileNotFoundError(f"No track {name_or_path!r}. Bundled tracks: {names}")


def load_track(name_or_path: str | Path, marker_id: int | None = None) -> Track:
    """Reads a track file.

    :param name_or_path: A bundled track's name or a path to a track file.
    :param marker_id: When given, the ID printed on the track's first marker instead of the file's, so one layout can
        test every command (for example 3, 4 and 5 at a cross).
    :return: The track.
    """
    path = track_path(name_or_path)
    spec = yaml.safe_load(path.read_text()) or {}
    cones, roads = [], []
    for road in spec.get("roads", []):
        (kind, details), = road.items()
        if kind == "lane":
            lane_cones, outline = _lane(details)
            cones += lane_cones
            roads.append(outline)
        elif kind == "cross":
            cross_cones, outlines = _cross(details)
            cones += cross_cones
            roads += outlines
        else:
            raise ValueError(f"{path.name}: unknown road type {kind!r}; use lane or cross")
    cones += [Cone(float(x), float(y), _colour(colour)) for x, y, colour in spec.get("cones", [])]
    size = default_marker_size_m()
    markers = [Marker(int(m["id"]), float(m["x"]), float(m["y"]), float(m.get("facing_deg", 180.0)),
                      float(m.get("size_m", size)), float(m.get("height_m", 0.2))) for m in spec.get("markers", [])]
    if marker_id is not None:
        if not markers:
            raise ValueError(f"{path.name} has no marker to set the ID of")
        markers[0] = replace(markers[0], id=int(marker_id))
    for marker in markers:
        if not 0 <= marker.id <= 49:
            raise ValueError(f"{path.name}: marker ID {marker.id} is not in DICT_4X4_50 (0-49)")
    obstacles = [Obstacle(float(o["x"]), float(o["y"]), float(o.get("length_m", 0.2)), float(o.get("width_m", 0.2)),
                          float(o.get("height_m", 0.3)), float(o.get("yaw_deg", 0.0))) for o in spec.get("obstacles", [])]
    start = spec.get("start", {})
    return Track(name=str(spec.get("name", path.stem)), description=str(spec.get("description", "")),
                 cones=tuple(cones), markers=tuple(markers), obstacles=tuple(obstacles), roads=tuple(roads),
                 start=(float(start.get("x", 0.0)), float(start.get("y", 0.0)),
                        math.radians(float(start.get("heading_deg", 0.0)))),
                 duration_s=float(spec.get("duration_s", 30.0)), policy_settings=dict(spec.get("policy_settings", {})))
