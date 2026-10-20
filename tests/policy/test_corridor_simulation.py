"""The opt-in braking plant must honour its stated distance budget, independently of U-turn scoring."""
from dataclasses import replace

import pytest

from policy.control.actions import DriveCommand
from tests.policy.curved_lanes import CurveCase, CurvedLane, run_curve


@pytest.mark.parametrize("speed", [.4, .7])
def test_assumed_braking_distance_matches_budget_and_default_still_coasts(speed):
    class Stop:
        def run_policy(self, observations, lanes):
            return [DriveCommand(0., 0., brake=True)]

    case = CurveCase(CurvedLane(pieces=(("straight", 4.),)), speed_m_per_s=speed)
    case = replace(case, car=replace(case.car, rolling_start=True))
    braking = run_curve(case, Stop(), braking_distance_per_speed_s=.6)
    coasting = run_curve(case, Stop())
    distance = braking.x[-1]-braking.x[0]
    assert distance == pytest.approx(speed*.6, abs=.01)
    assert coasting.x[-1]-coasting.x[0] > 2*distance
