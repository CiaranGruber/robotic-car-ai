from collections.abc import Iterable
from dataclasses import dataclass, replace
from enum import Enum
from typing import Any, Self

from policy.input_output import ConeColour
from policy.markers.commands import MarkerCommand, MarkerSighting
from policy.intersection_state import IntersectionMemory
from policy.obstacle_detection.obstacles import LaneObstacle
from policy.serialisation import Serialisable


class MpcStop(Serialisable, Enum):
    """Why the MPC stopped the car for the rest of a run."""
    OBSTACLE = 0
    """obstacle_confirm_steps consecutive steps saw an obstacle in the car's way within the stopping distance."""
    LANE_LOSS = 1
    """The lane was lost for lane_loss_timeout_s, its cone rows ended, or the plan followed without it ran out."""
    SOLVER_FAILURE = 2
    """An MPC solve did not converge."""
    INTERSECTION = 3
    """Intersection admission, motion, route lifetime or swept-clearance validation failed."""

    def serialise(self) -> str:
        """
        :return: The enum member name.
        """
        return self.name

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """
        :param data: Enum member name, or the underlying integer value.
        :return: The matching stop reason.
        """
        if isinstance(data, str):
            return cls[data]
        return cls(data)


@dataclass(frozen=True)
class MpcPlan(Serialisable):
    """The plan of the latest converged MPC solve, followed while the lane is lost."""

    curvatures_per_m: tuple[float, ...]
    """Planned curvature in 1/m for each step of the horizon, positive turning left."""
    step_distance_m: float
    """Distance in metres between the plan's steps: the solve's prediction speed times mpc.step_s. 0.0 for a plan
    solved at zero speed, which cannot be followed while moving."""
    distance_driven_m: float
    """Estimated distance in metres driven since the solve, from the wheel speed over each elapsed step, or the
    previous requested speed without one. This is approximate odometry, not a measured pose."""


@dataclass(frozen=True)
class CarPose(Serialisable):
    """Where the car is in the run's map frame, which is where base_link was at the run's first step: +x forwards then,
    +y left then, metres and radians. The local map (policy/local_map/local_map.py) moves it by dead reckoning and
    corrects it against the remembered cones. It is approximate: the map frame may turn slowly away from the ground
    with a biased dead reckoning, and drifts further without cones; only the remembered cones' places relative to the
    car are relied on."""

    x_m: float
    """Distance in metres along the map's +x axis to base_link's origin."""
    y_m: float
    """Distance in metres along the map's +y axis to base_link's origin."""
    heading_rad: float
    """Direction of the car's +x axis from the map's +x axis in radians, positive to the left, within [-pi, pi]."""


@dataclass(frozen=True)
class RememberedCone(Serialisable):
    """A cone in the run's map frame, averaged over the cone batches that saw it."""

    x_m: float
    """Distance in metres along the map's +x axis."""
    y_m: float
    """Distance in metres along the map's +y axis."""
    colour: ConeColour
    """Colour of the cone."""
    sightings: int
    """Number of cone batches that saw it, at least 1."""
    last_seen_s: float
    """policy_elapsed_s of the latest step whose cone batch saw it."""
    confidence: float
    """Detector confidence of its latest sighting, in [0, 1]."""


@dataclass(frozen=True)
class Memory(Serialisable):
    """Data kept from one policy step to a later one.

    Read the current set with CarMemory.data. Change it only through CarMemory.
    """

    policy_step: int
    """How many policy steps have finished. It starts at 0."""
    mpc_stop: MpcStop | None
    """Why the MPC stopped the car for the rest of this run, or None while it may drive. Only a new run clears it."""
    mpc_plan: MpcPlan | None
    """The plan of the latest converged MPC solve in this run, or None before the first one and after a failed one."""
    pose: CarPose | None = None
    """Where the car is in this run's map frame, or None before the local map's first step of the run."""
    imu_heading_rad: float | None = None
    """The IMU's relative heading (CarOrientation.heading_angle) at the latest step that had one, so the next step can
    turn the pose by the change; None without one."""
    cones: tuple[RememberedCone, ...] = ()
    """The cones remembered in this run, at most local_map.MAX_REMEMBERED_CONES of them."""
    lane_obstacles: tuple[LaneObstacle, ...] = ()
    """Obstacles that came close to the car's front, which LaneAvoidance keeps planning past once the lidar no longer
    sees them ahead. They are in lane coordinates, with distances from where the car's front is now, moved back by the
    distance driven (negative once beside or behind it)."""
    marker_sighting: MarkerSighting | None = None
    """The command marker seen in range in the latest policy step and how many steps in a row saw it, or None when the
    latest step saw none."""
    intersection_command: MarkerCommand | None = None
    """What to do at the next intersection, from the latest TURN_LEFT, TURN_RIGHT or STRAIGHT marker in this run, or
    None before one."""
    intersection_command_seen_s: float | None = None
    intersection: IntersectionMemory | None = None

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """Load memory, including recordings made before MPC, map, marker or intersection state was kept in it."""
        return super().deserialise({"mpc_stop": None, "mpc_plan": None, "pose": None, "imu_heading_rad": None,
                                    "cones": [], "lane_obstacles": [], "marker_sighting": None,
                                    "intersection_command": None, "intersection_command_seen_s": None,
                                    "intersection": None, **data})


class CarMemory:
    """Holds the policy's memory for one run and is the only place that changes it.

    The node creates one at startup, passes it into each policy step, and calls step once that
    step has finished.
    """

    def __init__(self):
        """Starts a run with no policy steps finished."""
        self.__internal = Memory(
            policy_step = 0,
            mpc_stop = None,
            mpc_plan = None,
        )

    @property
    def data(self) -> Memory:
        """
        :return: The memory held now.
        """
        return self.__internal

    def step(self):
        """Records that one policy step has finished by adding one to the policy step count."""
        self.__internal = replace(self.__internal, policy_step=self.__internal.policy_step + 1)

    def start_mpc_run(self):
        """Forgets the MPC's stop and plan of the previous run. The MPC does this on the first step of a run."""
        self.__internal = replace(self.__internal, mpc_stop=None, mpc_plan=None)

    def start_marker_run(self):
        """Forgets the previous run's marker sighting and intersection command. Done on the first step of a run."""
        self.__internal = replace(self.__internal, marker_sighting=None, intersection_command=None,
                                  intersection_command_seen_s=None, intersection=None)

    def see_marker(self, marker_id: int | None):
        """Records which command marker this step saw in range: one more step in a row for the same marker, the first
        step for a different one.

        :param marker_id: The marker's ID, or None when this step saw none.
        """
        sighting = self.__internal.marker_sighting
        if marker_id is None:
            sighting = None
        elif sighting is not None and sighting.marker_id == marker_id:
            sighting = replace(sighting, steps=sighting.steps + 1)
        else:
            sighting = MarkerSighting(marker_id, 1)
        self.__internal = replace(self.__internal, marker_sighting=sighting)

    def command_intersection(self, command: MarkerCommand, seen_s: float | None = None):
        """Records what to do at the next intersection.

        :param command: TURN_LEFT, TURN_RIGHT or STRAIGHT.
        """
        self.__internal = replace(self.__internal, intersection_command=command, intersection_command_seen_s=seen_s)

    def consume_intersection_command(self):
        """Consume a queued command once; the committed choice lives in intersection memory."""
        self.__internal = replace(self.__internal, intersection_command=None, intersection_command_seen_s=None)

    def update_intersection(self, state: IntersectionMemory):
        """Replace the navigator's bounded serialisable state."""
        self.__internal = replace(self.__internal, intersection=state)

    def stop_mpc(self, reason: MpcStop):
        """Records that the MPC stopped the car for the rest of the run.

        :param reason: Why it stopped.
        """
        self.__internal = replace(self.__internal, mpc_stop=reason)

    def plan_mpc(self, curvatures_per_m: Iterable[float], step_distance_m: float):
        """Records the plan of a converged MPC solve, with no distance driven along it yet.

        :param curvatures_per_m: Planned curvature in 1/m for each step of the horizon.
        :param step_distance_m: Distance in metres between the plan's steps.
        """
        self.__internal = replace(self.__internal, mpc_plan=MpcPlan(
            tuple(float(curvature) for curvature in curvatures_per_m), float(step_distance_m), 0.0))

    def drop_mpc_plan(self):
        """Forgets the MPC's plan, after a solve that did not converge."""
        self.__internal = replace(self.__internal, mpc_plan=None)

    def drive_along_mpc_plan(self, distance_m: float):
        """Adds to the distance driven since the MPC's plan. Nothing changes without a plan.

        :param distance_m: Distance in metres driven since the previous step.
        """
        plan = self.__internal.mpc_plan
        if plan is not None:
            self.__internal = replace(self.__internal, mpc_plan=replace(
                plan, distance_driven_m=plan.distance_driven_m + distance_m))

    def start_map(self, imu_heading_rad: float | None):
        """Starts the run's map where the car is now, with no cones remembered. The local map does this on the first
        step of a run.

        :param imu_heading_rad: The IMU's relative heading at this step, or None without one.
        """
        self.__internal = replace(self.__internal, pose=CarPose(0.0, 0.0, 0.0), imu_heading_rad=imu_heading_rad,
                                  cones=())

    def move_car(self, pose: CarPose, imu_heading_rad: float | None):
        """Records where the car is now in the run's map.

        :param pose: The car's pose in the map frame.
        :param imu_heading_rad: The IMU's relative heading at this step, or None without one.
        """
        self.__internal = replace(self.__internal, pose=pose, imu_heading_rad=imu_heading_rad)

    def remember_cones(self, cones: Iterable[RememberedCone]):
        """Replaces the remembered cones.

        :param cones: The cones to remember, in the map frame.
        """
        self.__internal = replace(self.__internal, cones=tuple(cones))

    def remember_lane_obstacles(self, obstacles: Iterable[LaneObstacle]):
        """Replaces the obstacles remembered beside the car.

        :param obstacles: The obstacles, with distances from where the car's front is now.
        """
        self.__internal = replace(self.__internal, lane_obstacles=tuple(obstacles))

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """
        :param data: JSON-friendly data produced by serialising Memory.
        :return: A car memory holding that set.
        """
        memory = cls()
        memory.__internal = Memory.deserialise(data)
        return memory
