"""Turn speed requests and bounded missing-lane reuse without hardware."""
from dataclasses import replace

import numpy as np
import pytest

from policy.action_policy.mpc import MpcConfig, MpcSolution
from policy.action_policy.policy import MpcPolicy
from policy.lane_detection.lane_detection import LaneDetector, LaneDetectionConfig
from tests.policy.test_lane_detection import lane_cones
from tests.policy.test_temporal_lane import observation


@pytest.mark.parametrize('curvature,expected', [(0,.5), (.15,.5), (.30,.45), (.45,.4), (2,.4), (-.45,.4)])
def test_speed_schedule_handles_straights_and_both_turn_directions(curvature, expected):
    policy = MpcPolicy(MpcConfig(turn_slowdown_enabled=True))
    assert policy.speed_for_curvature(curvature) == pytest.approx(expected)


@pytest.mark.parametrize('field,value', [('turn_slowdown_enabled',1), ('turn_speed_m_per_s',0),
                                      ('turn_start_curvature_per_m',-.1), ('turn_full_curvature_per_m',.1),
                                      ('turn_full_curvature_per_m',float('nan'))])
def test_invalid_speed_settings_fail_startup(field,value):
    with pytest.raises(ValueError):
        MpcConfig(**{field:value})


def test_low_cruise_target_is_never_increased_by_turn_cap():
    policy = MpcPolicy(MpcConfig(target_speed_m_per_s=.2, turn_slowdown_enabled=True))
    assert policy.speed_for_curvature(1) == pytest.approx(.2)


def test_planned_turn_slows_even_when_lane_centre_is_straight_and_predicts_measured_speed(monkeypatch):
    policy = MpcPolicy(MpcConfig(turn_slowdown_enabled=True))
    obs = observation(0,lane_cones(),speed=.7)
    lane = LaneDetector(LaneDetectionConfig()).detect(obs)
    seen = {}
    def solve(*args, **kwargs):
        seen.update(kwargs)
        return MpcSolution(np.full(12,.48), True, 0)
    monkeypatch.setattr(policy.controller,'solve',solve)
    command = policy.run_policy(obs,lane)[0]
    assert command.speed_m_per_s == pytest.approx(.4)
    assert seen['speed_m_per_s'] == pytest.approx(.7)
    assert policy.plan_step_distance_m == pytest.approx(.07)


def test_lane_loss_does_not_resume_cruise_speed_in_a_slowed_turn():
    policy = MpcPolicy(MpcConfig(turn_slowdown_enabled=True, lane_end_distance_m=10))
    obs = observation(0,lane_cones(radius_m=2),speed=.5)
    lane = LaneDetector(LaneDetectionConfig(max_curvature_per_m=2)).detect(obs)
    first = policy.run_policy(obs,lane)[0]
    assert first.speed_m_per_s <= .41
    lost = policy.run_policy(observation(.1,[],speed=.4),
                             replace(lane,status=type(lane.status).NO_LANE))[0]
    assert 0 < lost.speed_m_per_s <= first.speed_m_per_s
