"""Offline-only U-turn steering student trained from MPC demonstrations.

No ROS selector or car upload. A small feed-forward network predicts the full curvature horizon from fitted
lane state and measured motion; it never sees the simulator's map/true pose. MpcPolicy retains speed scheduling,
lane-loss, sensor-delay and stop logic. Finite bounded predictions are not a physical safety certificate.
"""
from pathlib import Path
import time

import numpy as np

from policy.action_policy.mpc import MpcSolution


def features(state, previous, age, speed, offset):
    """Sensor-derived inputs, shared exactly by dataset collection and inference."""
    return np.array([state.lateral_error_m, state.heading_error_rad, state.path_curvature_per_m,
                     previous, float(age), float(speed), offset], dtype=float)


class UTurnNetwork:
    """Immutable inference weights for a simulation-trained two-hidden-layer steering network."""

    def __init__(self, path: str | Path, config):
        with np.load(path, allow_pickle=False) as saved:
            self.mean, self.scale = saved['mean'], saved['scale']
            self.w1, self.b1, self.w2, self.b2, self.w3, self.b3 = [saved[k] for k in ('w1','b1','w2','b2','w3','b3')]
            limits = saved['contract']
        right = config.max_right_curvature_per_m or config.max_curvature_per_m
        expected = np.array([config.max_curvature_per_m, right, config.step_s, config.horizon_steps])
        if not np.array_equal(limits, expected):
            raise ValueError('U-turn weights do not match steering/horizon settings')
        shapes = ((7,), (7,), (7,32), (32,), (32,32), (32,), (32,config.horizon_steps), (config.horizon_steps,))
        arrays = (self.mean,self.scale,self.w1,self.b1,self.w2,self.b2,self.w3,self.b3)
        if any(a.shape != shape or not np.isfinite(a).all() for a,shape in zip(arrays,shapes)) or np.any(self.scale<=0):
            raise ValueError('Invalid U-turn weights')
        self.config = config

    def predict(self, inputs):
        inputs = np.asarray(inputs)
        if inputs.shape != (7,) or not np.isfinite(inputs).all() or inputs[4] < 0 or inputs[5] < 0:
            return None
        z = (inputs-self.mean)/self.scale
        if np.max(np.abs(z)) > 8:
            return None
        hidden = np.tanh(z@self.w1+self.b1)
        hidden = np.tanh(hidden@self.w2+self.b2)
        output = .92*(hidden@self.w3+self.b3)
        right = self.config.max_right_curvature_per_m or self.config.max_curvature_per_m
        return np.clip(output,-right,self.config.max_curvature_per_m) if np.isfinite(output).all() else None


class UTurnStudentController:
    """Offline adapter for the existing MpcPolicy stop/speed framework; no optimiser runs here."""

    def __init__(self, config, path):
        self.config, self.network = config, UTurnNetwork(path,config)

    def solve(self, state, previous_curvature_per_m, delay_s, speed_m_per_s=None, *, lateral_offset_m=0.0):
        start = time.perf_counter()
        speed = self.config.target_speed_m_per_s if speed_m_per_s is None else speed_m_per_s
        curve = self.network.predict(features(state,previous_curvature_per_m,delay_s,speed,lateral_offset_m))
        valid = curve is not None
        if not valid:
            curve = np.zeros(self.config.horizon_steps)
        # Existing policy uses this flag to latch a stop for invalid inference. True means bounded finite
        # inference, not optimiser convergence, collision clearance, or physical qualification.
        return MpcSolution(curve,valid,time.perf_counter()-start)
