from dataclasses import dataclass, replace
from typing import Any, Self

from policy.serialisation import Serialisable


@dataclass(frozen=True)
class Memory(Serialisable):
    """Data kept from one policy step to a later one.

    Read the current set with CarMemory.data. Change it only through CarMemory.
    """

    policy_step: int
    """How many policy steps have finished. It starts at 0."""


class CarMemory:
    """Holds the policy's memory for one run and is the only place that changes it.

    The node creates one at startup, passes it into each policy step, and calls step once that
    step has finished.
    """

    def __init__(self):
        """Starts a run with no policy steps finished."""
        self.__internal = Memory(
            policy_step = 0,
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

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """
        :param data: JSON-friendly data produced by serialising Memory.
        :return: A car memory holding that set.
        """
        memory = cls()
        memory.__internal = Memory.deserialise(data)
        return memory
