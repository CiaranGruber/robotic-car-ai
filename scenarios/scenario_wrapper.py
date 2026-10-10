import json
from enum import Enum
from pathlib import Path
from typing import Generator

from policy.input_output import CarObservations, CarActions
from scenarios import SCENARIO_SUFFIX, SCENARIOS_DIR


class ScenarioType(Enum):
    REAL = "real_scenarios"


class Scenario:
    def __init__(self, scenario_path: Path):
        self.scenario_path = scenario_path
        self._loaded_scenario = None

    @classmethod
    def from_name(cls, scenario_type: ScenarioType, name: str):
        return cls(SCENARIOS_DIR / scenario_type.value / f"{name}{SCENARIO_SUFFIX}")

    def reload(self):
        with open(self.scenario_path, "r") as f:
            self._loaded_scenario = json.load(f)

    @property
    def name(self):
        if not self._loaded_scenario:
            self.reload()
        return self._loaded_scenario["name"]

    @property
    def description(self):
        if not self._loaded_scenario:
            self.reload()
        return self._loaded_scenario["description"]

    def steps(self) -> Generator[tuple[CarObservations, CarActions], None, None]:
        if not self._loaded_scenario:
            self.reload()
        test_data = self._loaded_scenario["test_data"]
        for data in test_data:
            observations = CarObservations.deserialise(data["observations"])
            actions = CarActions.deserialise(data["actions"])
            yield observations, actions
