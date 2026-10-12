"""Regression checks for continuous following and actionable stop diagnostics."""
import dataclasses

from policy.action_policy.mpc import MpcConfig
from policy.cone_filter.cone_filter import ConeFilterConfig
from policy.control.control import ControlConfig
from policy.input_output import PolicyState, ConeBatch, SensorAge
from policy.policy_runner import MovementPolicy
from tests.policy.test_control import observations
from tests.policy.test_mpc import lane_cones
from tests.policy.diagnose_cones import inspect_batch


def test_continuous_following_remains_active_past_demo_duration():
    policy = MovementPolicy(MpcConfig(run_duration_s=0.0), ConeFilterConfig(), ControlConfig())
    for step in range(121):
        obs = dataclasses.replace(observations(0.5, cones=lane_cones(0.0, 0.0)),
                                  policy=PolicyState(0.0 if step == 0 else 0.1, step * 0.1, step == 0))
        assert policy.step(obs).drive_action > 0.0
    assert "Following cone lane" in policy.status_reason
    assert "cones raw=" in policy.status_reason


def test_optional_duration_reports_why_it_stopped():
    policy = MovementPolicy(MpcConfig(run_duration_s=8.0), ConeFilterConfig(), ControlConfig())
    obs = observations(0.5, first=True, cones=lane_cones(0.0, 0.0))
    assert policy.step(obs).drive_action > 0.0
    obs = dataclasses.replace(obs, policy=PolicyState(0.1, 8.0, False))
    assert policy.step(obs).drive_action == 0.0
    assert "run duration reached" in policy.status_reason


def test_filter_reports_far_cones_without_relaxing_geometry_checks():
    detections = [dataclasses.replace(c, pos=dataclasses.replace(c.pos, x=c.pos.x + 5.0))
                  for c in lane_cones(0.0, 0.0)]
    report = inspect_batch(ConeBatch(detections, SensorAge(0.1, None), 0.0))
    assert report["raw"] > 0 and report["beyond_forward_limit"] == report["raw"]
    assert report["kept"] == 0 and not report["usable_lane"]


def test_missing_feedback_is_visible_in_movement_status():
    policy = MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig())
    assert policy.step(observations(None, first=True, cones=lane_cones(0.0, 0.0))).drive_action == 0.0
    assert "wheel-speed feedback unavailable" in policy.status_reason


def test_lane_loss_reason_stays_latched_until_explicit_restart():
    policy = MovementPolicy(MpcConfig(lane_loss_timeout_s=0.1), ConeFilterConfig(), ControlConfig())
    policy.step(observations(0.5, first=True, cones=lane_cones(0.0, 0.0)))
    assert policy.step(observations(0.5, dt=0.1, cones=[])).drive_action == 0.0
    reason = policy.action_policy.status_reason
    assert "lane-loss timeout" in reason
    assert policy.step(observations(0.5, cones=lane_cones(0.0, 0.0))).drive_action == 0.0
    assert policy.action_policy.status_reason == reason
    assert policy.step(observations(0.5, first=True, cones=lane_cones(0.0, 0.0))).drive_action > 0.0
