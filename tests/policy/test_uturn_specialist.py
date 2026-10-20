"""Offline learned steering bounds and invalid-inference stop contracts."""
from dataclasses import replace
import numpy as np
import pytest

from policy.action_policy.mpc import MpcConfig,PathTrackingState
from policy.action_policy.policy import MpcPolicy
from policy.action_policy.uturn_specialist import UTurnNetwork,UTurnStudentController
from policy.lane_detection.lane_detection import LaneDetector,LaneDetectionConfig
from tests.policy.test_temporal_lane import observation
from tests.policy.test_lane_detection import lane_cones


def weights(path,bias=0):
    np.savez(path,mean=np.zeros(7),scale=np.ones(7),w1=np.zeros((7,32)),b1=np.zeros(32),
             w2=np.zeros((32,32)),b2=np.zeros(32),w3=np.zeros((32,12)),b3=np.full(12,bias),
             contract=np.array([.48,.92,.1,12]))


@pytest.mark.parametrize('bias,expected', [(100,.48),(-100,-.92),(0,0)])
def test_inference_obeys_asymmetric_limits_after_loading(tmp_path,bias,expected):
    path=tmp_path/'weights.npz';weights(path,bias)
    model=UTurnNetwork(path,MpcConfig())
    assert model.predict(np.zeros(7)) == pytest.approx(np.full(12,expected))


@pytest.mark.parametrize('values', [np.full(7,np.nan),np.full(7,9),np.zeros(3),
                                  np.array([0,0,0,0,-.1,0,0]),np.array([0,0,0,0,0,-.1,0])])
def test_bad_or_out_of_distribution_inputs_are_rejected(tmp_path,values):
    path=tmp_path/'weights.npz';weights(path)
    assert UTurnNetwork(path,MpcConfig()).predict(values) is None


def test_mismatched_steering_calibration_is_rejected(tmp_path):
    path=tmp_path/'weights.npz';weights(path)
    with pytest.raises(ValueError):UTurnNetwork(path,replace(MpcConfig(),max_curvature_per_m=1.2))


def test_invalid_inference_latches_existing_policy_stop(tmp_path):
    path=tmp_path/'weights.npz';weights(path)
    controller=MpcPolicy(MpcConfig())
    controller.controller=UTurnStudentController(MpcConfig(),path)
    obs=observation(0,lane_cones(),speed=9)
    lane=LaneDetector(LaneDetectionConfig()).detect(obs)
    commands=controller.run_policy(obs,lane)
    assert controller.stopped_for_solver_failure and commands[0].speed_m_per_s==0
