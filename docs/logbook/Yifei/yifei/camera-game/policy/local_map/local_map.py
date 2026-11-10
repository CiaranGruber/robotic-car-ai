"""
local_map.py

This file keeps a short-lived map of the cones around the car, so lane detection can still use the cones a cone batch
does not show: those beside and just behind the car, nearer than the camera sees, and those hidden behind an obstacle or
missed by the detector. The map is kept in CarMemory (Memory.pose and Memory.cones), so the memory log shows it.

At each step:
1. Dead reckoning moves the car's pose in the run's map frame by the distance driven since the previous step (the wheel
   speed, else the speed requested) and the turn: the change of the IMU's relative heading, else the IMU's yaw rate,
   else the curvature requested at the previous step.
2. Each cone of the batch is matched to the nearest remembered cone of its colour within cone_match_distance_m, the
   nearest pairs first, each cone at most once.
3. With at least MIN_ALIGN_CONES matched cones that are confirmed (seen by cone_min_sightings batches), the pose is
   corrected so that the batch lies on the remembered cones: ALIGN_GAIN of the rigid move that fits the matched pairs
   best (least squares), at most MAX_SHIFT_CORRECTION_M and MAX_TURN_CORRECTION_RAD per step. The remembered cones keep
   where the camera saw them relative to the cones it sees now, so dead reckoning only has to be good enough to match
   them, and its drift does not build up between the car and the cones around it while cones are in view. The map as a
   whole may still turn slowly with a biased dead reckoning, which does not matter: only where the cones are relative
   to the car is used.
4. Matched cones move towards their new sighting (a running mean of up to MAX_AVERAGED_SIGHTINGS sightings) and the
   other cones of the batch are added. Cones more than cone_keep_behind_m behind base_link, not seen for cone_keep_s, or
   not confirmed within UNCONFIRMED_KEEP_S are forgotten.
5. The confirmed cones with no cone of their colour in the batch within cone_match_distance_m are returned in the
   batch's frame, for lane detection to add to the batch.

When a batch matches under a third of the confirmed cones in its view, dead reckoning has drifted too far to match them,
so the remembered cones are forgotten and the map starts again from the batch.

Cones are placed with the pose at the step, not at the moment the camera saw them, as the rest of the policy uses the
batch, so remembered cones come back in the frame of a batch of the same age. Cones are assumed not to move.

It keeps its state in CarMemory, and its work is bounded by MAX_REMEMBERED_CONES and the batch size, so it is safe to
run inside the real-time policy step.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from policy.input_output import CarObservations, ConeColour, ConeDetection, Position
from policy.memory import CarMemory, CarPose, RememberedCone

MAX_REMEMBERED_CONES = 64
"""Most cones remembered at once; the longest unseen are forgotten first."""
MAX_AVERAGED_SIGHTINGS = 5
"""A remembered cone moves 1 / min(sightings, this) of the way to each new sighting, so its latest sightings, made
nearer the car, count most once it has been seen this often."""
UNCONFIRMED_KEEP_S = 0.5
"""Seconds a cone seen by fewer than cone_min_sightings batches is kept unseen, so a cone detected by mistake is soon
forgotten."""
MIN_ALIGN_CONES = 3
"""Fewest matched confirmed cones that correct the pose, and that a batch must have for the map to count as lost."""
MIN_ALIGN_SPAN_M = 0.5
"""Smallest distance in metres across the matched cones of a batch that corrects the heading; a narrower group only
corrects the position."""
ALIGN_GAIN = 0.5
"""Fraction of the correction that fits the batch to the remembered cones that is applied at each step. Below 1 it
averages the detector's noise over a few steps instead of moving the pose by all of it."""
MAX_SHIFT_CORRECTION_M = 0.05
"""Largest shift in metres the correction applies to the pose at one step."""
MAX_TURN_CORRECTION_RAD = math.radians(3.0)
"""Largest turn in radians the correction applies to the pose at one step."""
LOST_MATCH_FRACTION = 1.0 / 3.0
"""When a batch matches fewer than this fraction of the confirmed cones in its view (at least MIN_ALIGN_CONES of them),
the remembered cones are forgotten."""


@dataclass(frozen=True)
class LocalMapConfig:
    """Settings for the map of remembered cones, loaded from the local_map parameters in config/ai4r_policy.yaml.

    These defaults are also the parameters' defaults in policy_node. Invalid settings raise a ValueError, which stops
    policy_node from starting.
    """
    use_remembered_cones: bool = True
    """Whether lane detection also uses the remembered cones a batch does not show. Must be true or false. The map is
    kept, and logged in the memory, either way."""
    cone_match_distance_m: float = 0.12
    """Farthest in metres a cone of a batch may be from a remembered cone of its colour to be the same cone. Must be
    positive. Keep it below half the spacing of the cones along an edge, and above the detector's noise plus the error
    of dead reckoning over a step."""
    cone_min_sightings: int = 3
    """Number of batches that must see a cone before it is used by lane detection or corrects the pose, so a cone
    detected once by mistake is not. Must be an integer of at least 1."""
    cone_keep_behind_m: float = 0.5
    """Remembered cones more than this in metres behind base_link are forgotten. Must be finite and at least 0. Cones
    beside the car fix where the car is in the lane; farther behind, a lane whose curve changes no longer fits them."""
    cone_keep_s: float = 4.0
    """Remembered cones not seen for this long in seconds are forgotten. Must be positive and finite."""

    def __post_init__(self):
        if not isinstance(self.use_remembered_cones, bool):
            raise ValueError(f"local_map.use_remembered_cones must be true or false, not {self.use_remembered_cones}")
        for name in ("cone_match_distance_m", "cone_keep_s"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"local_map.{name} must be positive and finite, not {value}")
        if not (math.isfinite(self.cone_keep_behind_m) and self.cone_keep_behind_m >= 0.0):
            raise ValueError(f"local_map.cone_keep_behind_m must be finite and at least 0, "
                             f"not {self.cone_keep_behind_m}")
        count = self.cone_min_sightings
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError(f"local_map.cone_min_sightings must be an integer of at least 1, not {count}")


class LocalMap:
    """Moves the car's pose and remembers the cones around it in CarMemory, and recalls those a batch does not show.

    Create one when the node starts and call step once per policy step, before lane detection.
    """

    def __init__(self, config: LocalMapConfig):
        """
        :param config: The local map settings.
        """
        self.config = config
        self.recalled: list[ConeDetection] = []
        """The remembered cones the latest step returned for lane detection, in its batch's frame."""
        self.map_cones: list[ConeDetection] = []
        """Every cone remembered after the latest step with a batch, in that batch's frame; empty without one."""
        self.pose: CarPose | None = None
        """The car's pose in the run's map after the latest step (Memory.pose), for drawing and plots."""

    def step(self, observations: CarObservations, memory: CarMemory, requested_speed_m_per_s: float,
             requested_curvature_per_m: float) -> list[ConeDetection]:
        """Moves the car's pose, then remembers this step's cones and recalls those it does not show.

        :param observations: The observations at this step, with the cones as lane detection gets them (filtered and
            turned back by the camera yaw offset).
        :param memory: The car memory, the same one for every step. The first step of a run starts a new map.
        :param requested_speed_m_per_s: Speed in m/s requested at the previous step, the speed driven without a wheel
            speed.
        :param requested_curvature_per_m: Curvature in 1/m requested at the previous step, positive turning left, the
            curvature driven without an IMU.
        :return: The confirmed remembered cones with no cone of their colour in this step's batch nearby, in the batch's
            frame, for lane detection. Empty without a batch, or when use_remembered_cones is false.
        """
        car = observations.car
        imu_heading_rad = None if car.orientation is None else car.orientation.heading_angle
        data = memory.data
        if observations.policy.is_first_policy_step or data.pose is None:
            memory.start_map(imu_heading_rad)
        else:
            memory.move_car(dead_reckon(data.pose, observations, data.imu_heading_rad, requested_speed_m_per_s,
                                        requested_curvature_per_m), imu_heading_rad)
        self.recalled, self.map_cones = [], []
        if observations.cones is not None:
            self._remember(observations.cones, observations.policy.policy_elapsed_s, memory)
        self.pose = memory.data.pose
        return self.recalled

    def _remember(self, cones: list[ConeDetection], elapsed_s: float, memory: CarMemory):
        """Matches the batch to the remembered cones, corrects the pose, updates the remembered cones and sets recalled
        and map_cones.

        :param cones: The cones of this step's batch, in base_link.
        :param elapsed_s: policy_elapsed_s of this step.
        :param memory: The car memory.
        """
        config = self.config
        pose = memory.data.pose
        remembered = list(memory.data.cones)
        seen = np.array([(cone.pos.x, cone.pos.y) for cone in cones], dtype=float).reshape(-1, 2)
        colours = [cone.colour for cone in cones]
        pairs = _match(_to_map(pose, seen), colours, remembered, config.cone_match_distance_m)
        confirmed = [(i, j) for i, j in pairs if remembered[j].sightings >= config.cone_min_sightings]
        if self._lost(seen, remembered, len(confirmed), pose):
            remembered, pairs = [], []
        elif len(confirmed) >= MIN_ALIGN_CONES:
            pose = _aligned(pose, seen[[i for i, _ in confirmed]],
                            np.array([(remembered[j].x_m, remembered[j].y_m) for _, j in confirmed]))
            memory.move_car(pose, memory.data.imu_heading_rad)
            pairs = _match(_to_map(pose, seen), colours, remembered, config.cone_match_distance_m)
        placed = _to_map(pose, seen)
        updated = list(remembered)
        matched = set()
        for i, j in pairs:
            cone = remembered[j]
            weight = 1.0 / min(cone.sightings + 1, MAX_AVERAGED_SIGHTINGS)
            updated[j] = RememberedCone(cone.x_m + weight * (placed[i, 0] - cone.x_m),
                                        cone.y_m + weight * (placed[i, 1] - cone.y_m), cone.colour,
                                        cone.sightings + 1, elapsed_s, cones[i].confidence)
            matched.add(i)
        updated += [RememberedCone(float(placed[i, 0]), float(placed[i, 1]), cones[i].colour, 1, elapsed_s,
                                   cones[i].confidence) for i in range(len(cones)) if i not in matched]
        kept = [cone for cone in updated if self._kept(cone, pose, elapsed_s)]
        kept = sorted(kept, key=lambda cone: cone.last_seen_s, reverse=True)[:MAX_REMEMBERED_CONES]
        memory.remember_cones(kept)

        # Back in the batch's frame: every remembered cone for drawing, and the confirmed ones the batch does not show
        body = _to_body(pose, np.array([(cone.x_m, cone.y_m) for cone in kept], dtype=float).reshape(-1, 2))
        self.map_cones = [ConeDetection(Position(float(x), float(y), 0.0), cone.colour, cone.confidence)
                          for (x, y), cone in zip(body, kept)]
        if not config.use_remembered_cones:
            return
        for (x, y), cone, recalled in zip(body, kept, self.map_cones):
            if cone.sightings < config.cone_min_sightings:
                continue
            same = [k for k, colour in enumerate(colours) if colour == cone.colour]
            if not same or np.min(np.hypot(seen[same, 0] - x, seen[same, 1] - y)) > config.cone_match_distance_m:
                self.recalled.append(recalled)

    def _kept(self, cone: RememberedCone, pose: CarPose, elapsed_s: float) -> bool:
        """
        :return: Whether a remembered cone is still kept: no more than cone_keep_behind_m behind base_link, seen within
            cone_keep_s, and confirmed or seen within UNCONFIRMED_KEEP_S.
        """
        config = self.config
        unseen_s = elapsed_s - cone.last_seen_s
        if unseen_s > config.cone_keep_s or (cone.sightings < config.cone_min_sightings
                                             and unseen_s > UNCONFIRMED_KEEP_S):
            return False
        along = (cone.x_m - pose.x_m) * math.cos(pose.heading_rad) + (cone.y_m - pose.y_m) * math.sin(pose.heading_rad)
        return along >= -config.cone_keep_behind_m

    def _lost(self, seen: np.ndarray, remembered: list[RememberedCone], confirmed_matches: int, pose: CarPose) -> bool:
        """
        :param seen: The batch's cones in base_link.
        :param remembered: The remembered cones.
        :param confirmed_matches: How many confirmed remembered cones the batch matched.
        :param pose: The car's pose after dead reckoning.
        :return: Whether the batch matched under LOST_MATCH_FRACTION of the confirmed cones in its view (between its
            nearest and farthest cone, and no farther to the side than its widest), with at least MIN_ALIGN_CONES of
            each, so the remembered cones no longer fit what the camera sees.
        """
        confirmed = [(cone.x_m, cone.y_m) for cone in remembered if cone.sightings >= self.config.cone_min_sightings]
        if len(seen) < MIN_ALIGN_CONES or len(confirmed) < MIN_ALIGN_CONES:
            return False
        body = _to_body(pose, np.array(confirmed))
        widest_rad = np.max(np.abs(np.arctan2(seen[:, 1], seen[:, 0])))
        in_view = ((body[:, 0] >= seen[:, 0].min()) & (body[:, 0] <= seen[:, 0].max())
                   & (np.abs(np.arctan2(body[:, 1], body[:, 0])) <= widest_rad))
        count = int(np.count_nonzero(in_view))
        return count >= MIN_ALIGN_CONES and confirmed_matches < LOST_MATCH_FRACTION * count


def dead_reckon(pose: CarPose, observations: CarObservations, previous_imu_heading_rad: float | None,
                requested_speed_m_per_s: float, requested_curvature_per_m: float) -> CarPose:
    """Moves the pose by the distance and turn driven since the previous step.

    :param pose: The car's pose at the previous step.
    :param observations: The observations at this step, for the time since the previous step, the wheel speed and the
        IMU.
    :param previous_imu_heading_rad: The IMU's relative heading at the previous step, or None without one.
    :param requested_speed_m_per_s: Speed in m/s requested at the previous step, used without a wheel speed.
    :param requested_curvature_per_m: Curvature in 1/m requested at the previous step, used without an IMU.
    :return: The car's pose now. The wheel speed is unsigned, so the car is taken to drive forwards. A distance or turn
        that is not finite leaves the pose where it was.
    """
    dt = observations.policy.dt
    car = observations.car
    speed = requested_speed_m_per_s if car.wheel_speed is None else float(car.wheel_speed)
    distance_m = speed * dt
    if car.orientation is not None and previous_imu_heading_rad is not None:
        turn_rad = _wrapped(car.orientation.heading_angle - previous_imu_heading_rad)
    elif car.ang_velocity is not None:
        turn_rad = car.ang_velocity.yaw * dt
    else:
        turn_rad = requested_curvature_per_m * distance_m
    if not (math.isfinite(distance_m) and math.isfinite(turn_rad)):
        return pose
    # Along an arc: the chord to its end points halfway through the turn
    half_rad = turn_rad / 2.0
    chord_m = distance_m * (1.0 - half_rad * half_rad / 6.0 if abs(half_rad) < 1e-4 else math.sin(half_rad) / half_rad)
    middle_rad = pose.heading_rad + half_rad
    return CarPose(pose.x_m + chord_m * math.cos(middle_rad), pose.y_m + chord_m * math.sin(middle_rad),
                   _wrapped(pose.heading_rad + turn_rad))


def _match(placed: np.ndarray, colours: list[ConeColour], remembered: list[RememberedCone],
           within_m: float) -> list[tuple[int, int]]:
    """
    :param placed: The batch's cones in the map frame, an Nx2 array.
    :param colours: Their colours.
    :param remembered: The remembered cones.
    :param within_m: Farthest distance in metres between a matched pair.
    :return: (batch index, remembered index) of each matched pair: the nearest pairs of the same colour first, each
        cone in at most one pair.
    """
    if len(placed) == 0 or not remembered:
        return []
    stored = np.array([(cone.x_m, cone.y_m) for cone in remembered])
    same = np.array([[colour == cone.colour for cone in remembered] for colour in colours])
    distances = np.where(same, np.hypot(placed[:, None, 0] - stored[None, :, 0],
                                        placed[:, None, 1] - stored[None, :, 1]), np.inf)
    candidates = np.argwhere(distances <= within_m)
    order = np.argsort(distances[candidates[:, 0], candidates[:, 1]], kind="stable")
    pairs, used_seen, used_stored = [], set(), set()
    for i, j in candidates[order].tolist():
        if i not in used_seen and j not in used_stored:
            pairs.append((i, j))
            used_seen.add(i)
            used_stored.add(j)
    return pairs


def _aligned(pose: CarPose, seen: np.ndarray, targets: np.ndarray) -> CarPose:
    """Corrects the pose so that cones seen from it lie nearer the remembered cones they matched.

    :param pose: The car's pose after dead reckoning.
    :param seen: The matched cones of the batch in base_link, an Nx2 array.
    :param targets: The remembered cones they matched, in the map frame, an Nx2 array.
    :return: The pose moved by ALIGN_GAIN of the rigid move that fits the seen cones to the targets best: a turn about
        the seen cones' middle (none when they span under MIN_ALIGN_SPAN_M) and a shift, each limited.
    """
    placed = _to_map(pose, seen)
    middle, target_middle = placed.mean(axis=0), targets.mean(axis=0)
    turn_rad = 0.0
    if math.hypot(*(seen.max(axis=0) - seen.min(axis=0))) >= MIN_ALIGN_SPAN_M:
        a, b = placed - middle, targets - target_middle
        turn_rad = math.atan2(float(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])), float(np.sum(a * b)))
    turn_rad = min(max(ALIGN_GAIN * turn_rad, -MAX_TURN_CORRECTION_RAD), MAX_TURN_CORRECTION_RAD)
    shift = ALIGN_GAIN * (target_middle - middle)
    shift_m = math.hypot(*shift)
    if shift_m > MAX_SHIFT_CORRECTION_M:
        shift *= MAX_SHIFT_CORRECTION_M / shift_m
    cos, sin = math.cos(turn_rad), math.sin(turn_rad)
    dx, dy = pose.x_m - middle[0], pose.y_m - middle[1]
    return CarPose(float(middle[0] + cos * dx - sin * dy + shift[0]), float(middle[1] + sin * dx + cos * dy + shift[1]),
                   _wrapped(pose.heading_rad + turn_rad))


def _to_map(pose: CarPose, points: np.ndarray) -> np.ndarray:
    """
    :param pose: The car's pose.
    :param points: Points in base_link, an Nx2 array.
    :return: The points in the map frame, an Nx2 array.
    """
    cos, sin = math.cos(pose.heading_rad), math.sin(pose.heading_rad)
    return np.column_stack([pose.x_m + cos * points[:, 0] - sin * points[:, 1],
                            pose.y_m + sin * points[:, 0] + cos * points[:, 1]])


def _to_body(pose: CarPose, points: np.ndarray) -> np.ndarray:
    """
    :param pose: The car's pose.
    :param points: Points in the map frame, an Nx2 array.
    :return: The points in base_link, an Nx2 array.
    """
    cos, sin = math.cos(pose.heading_rad), math.sin(pose.heading_rad)
    dx, dy = points[:, 0] - pose.x_m, points[:, 1] - pose.y_m
    return np.column_stack([cos * dx + sin * dy, -sin * dx + cos * dy])


def _wrapped(angle_rad: float) -> float:
    """
    :return: The angle within [-pi, pi].
    """
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))
