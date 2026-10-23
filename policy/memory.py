from collections.abc import Iterable
from dataclasses import dataclass, replace
from enum import Enum
from typing import Any, Self

from policy.markers.commands import MarkerCommand, MarkerSighting
from policy.serialisation import Serialisable


class MpcStop(Serialisable, Enum):
    """Why the MPC stopped the car for the rest of a run."""
    OBSTACLE = 0
    """obstacle_confirm_steps consecutive steps saw an obstacle in the car's way within the stopping distance."""
    LANE_LOSS = 1
    """The lane was lost for lane_loss_timeout_s, its cone rows ended, or the plan followed without it ran out."""
    SOLVER_FAILURE = 2
    """An MPC solve did not converge."""

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
    marker_sighting: MarkerSighting | None = None
    """The command marker seen in range in the latest policy step and how many steps in a row saw it, or None when the
    latest step saw none."""
    intersection_command: MarkerCommand | None = None
    """What to do at the next intersection, from the latest TURN_LEFT, TURN_RIGHT or STRAIGHT marker in this run, or
    None before one."""

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """Load memory, including recordings made before the MPC's stop and plan or the markers were kept in it."""
        return super().deserialise({"mpc_stop": None, "mpc_plan": None, "marker_sighting": None,
                                    "intersection_command": None, **data})


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
        self.__internal = replace(self.__internal, marker_sighting=None, intersection_command=None)

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

    def command_intersection(self, command: MarkerCommand):
        """Records what to do at the next intersection.

        :param command: TURN_LEFT, TURN_RIGHT or STRAIGHT.
        """
        self.__internal = replace(self.__internal, intersection_command=command)

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

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """
        :param data: JSON-friendly data produced by serialising Memory.
        :return: A car memory holding that set.
        """
        memory = cls()
        memory.__internal = Memory.deserialise(data)
        return memory
