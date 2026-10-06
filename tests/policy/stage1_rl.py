"""Stage 1 RL design (Card 3, Task A5): a steering policy trained with PPO in the Stage 1 simulation.

The RL policy replaces only the MPC's steering, so MPC and RL are compared on equal terms (Card 1.3).
RlController has MpcController's solve signature and runs inside MpcPolicy, so the cone filter, the lane reference
fitted from the cones, the speed and the lane-loss and lane-end rule (Task A7) are the MPC's. It observes only what
the MPC does (Card 1.2): the lateral and heading error from the cones, the detection age, the speed and its own last
two curvature requests, which stand in for the MPC's delay prediction.

Training (Stage1SteeringEnv) runs the same detection pipeline as stage1_scenarios.run_case, with domain
randomisation over the lane size, start, speed, detector noise, delay, car mass and floor. Lane sizes, starts and
disturbances are drawn from ranges, not from the tuning or unseen cases. The simulator's ground truth is used only
for the reward and to end an episode.

Needs stable-baselines3 (pip install "stable-baselines3>=2.3,<3"). Run from the repository root:
    python3 tests/policy/stage1_rl.py train --reward balanced --steps 300000 --out rl_models
    python3 tests/policy/stage1_rl.py evaluate rl_models/balanced.zip
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import gymnasium
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import stage1_scenarios as s1  # noqa: E402
from policy.action_policy.mpc import MpcConfig, MpcSolution, PathTrackingState  # noqa: E402
from policy.action_policy.reference import reference_from_cones  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones  # noqa: E402
from policy.control.actions import DriveCommand  # noqa: E402
from policy.input_output import ConeBatch, SensorAge  # noqa: E402

MAX_CURVATURE_PER_M = 1.5
"""Curvature in 1/m of a full action (+-1). The tuned MPC never needs more than 0.51 1/m on the tuning cases."""


def observation(state: PathTrackingState | None, previous_curvatures: tuple[float, float], age_s: float,
                speed_m_per_s: float) -> np.ndarray:
    """The policy's observation, scaled to about +-1. The same function is used in training and on the car.

    :param state: The lane reference fitted from the cones, or None when it is not usable at this step.
    :param previous_curvatures: The last two curvature requests in 1/m, newest first.
    :param age_s: Age of the detections in seconds.
    :param speed_m_per_s: The requested speed in m/s.
    :return: [lateral error, heading error, lane usable, last curvature, the one before, age, speed].
    """
    lateral, heading = (0.0, 0.0) if state is None else (state.lateral_error_m, state.heading_error_rad)
    return np.array([lateral / 0.3, heading / 0.3, 0.0 if state is None else 1.0,
                     previous_curvatures[0] / MAX_CURVATURE_PER_M, previous_curvatures[1] / MAX_CURVATURE_PER_M,
                     age_s / 0.4, speed_m_per_s], dtype=np.float32)


def bounded_penalty(value: float, scale: float) -> float:
    """1 - exp(-(value / scale)^2): 0 when value is 0, about 0.63 at the scale and at most 1."""
    return 1.0 - math.exp(-(value / scale) ** 2)


@dataclass(frozen=True)
class RewardWeights:
    """Reward per policy step (Card 1.1 requirements as shaped terms; ground truth, training only).

    r = 1 - lateral * P(y, 0.1 m) - heading * P(psi, 0.1 rad) - curvature_rate * P(dkappa/dt, 1.0 1/(m s)),
    with P the bounded penalty, y the car centre's distance from the lane centre (R1b), psi its heading relative to
    the lane and dkappa/dt the rate of change of the requested curvature (R4, whose threshold is 1.0). The weights add
    up to less than 1, so every step in the lane earns a positive reward and ending the episode early never pays.
    Touching a cone or a body corner past a cone row (R1a, R2) ends the episode with -collision; reaching the last
    cone pair ends it with +finish. Speed (R3) is held by the speed controller and stopping is impossible, so neither
    is in the reward.
    """
    lateral: float
    heading: float
    curvature_rate: float
    collision: float = 20.0
    finish: float = 10.0

    def __post_init__(self):
        if not self.lateral + self.heading + self.curvature_rate < 1.0:
            raise ValueError("the penalty weights must add up to less than 1")


REWARDS = {
    "tracking": RewardWeights(lateral=0.6, heading=0.2, curvature_rate=0.05),
    "balanced": RewardWeights(lateral=0.5, heading=0.2, curvature_rate=0.2),
    "smooth": RewardWeights(lateral=0.35, heading=0.2, curvature_rate=0.4),
}
"""Task A5 reward options, differing in how much smooth steering (R4) is traded against centring (R1)."""


class Stage1SteeringEnv(gymnasium.Env):
    """One policy step per 0.1 s detection batch, action = curvature, on random Stage 1 lanes with random
    disturbances, through the same detection pipeline as stage1_scenarios.run_case."""
    observation_space = gymnasium.spaces.Box(-5.0, 5.0, shape=(7,), dtype=np.float32)
    action_space = gymnasium.spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)

    def __init__(self, reward: RewardWeights, seed: int = 0):
        """
        :param reward: The reward option.
        :param seed: Seed for the random lanes and disturbances.
        """
        self.reward_weights = reward
        self.rng = np.random.default_rng(seed)
        self.env = None

    def _random_case(self) -> s1.Stage1Case:
        rng = self.rng
        spacing = float(rng.choice([0.5, 0.75]))
        lane = s1.Lane(length_m=6.0, width_m=float(rng.choice([0.8, 0.9, 1.0, 1.1, 1.2])),
                       cone_spacing_m=spacing)
        floor = (None, None, None, "dry", "wet")[int(rng.integers(5))]
        disturbances = s1.Disturbances(
            noise_m=float(rng.uniform(0.0, 0.03)), delay_s=float(rng.choice([0.0, 0.0, 0.1, 0.2])),
            car_mass_kg=float(rng.uniform(2.5, 3.6)), floor=floor,
            detector_range_m=float(rng.uniform(3.0, 4.0)))
        return s1.Stage1Case("train", "tuning", "", start_left_m=float(rng.uniform(-0.2, 0.2)),
                             start_heading_deg=float(rng.uniform(-15.0, 15.0)),
                             speed_m_per_s=float(rng.uniform(0.5, 1.2)), disturbances=disturbances,
                             lane=lane, seed=int(rng.integers(1 << 30)))

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        if self.env is not None:
            self.env.close()
        self.case = self._random_case()
        self.env = s1.make_env(self.case)
        self.obs, self.info = self.env.reset(seed=self.case.seed)
        self.det_rng = np.random.default_rng(self.case.seed)
        self.age_s = s1.DETECTION_AGE_S + self.case.disturbances.delay_s
        self.pending = []
        self.curvatures = (0.0, 0.0)
        self.steps = 0
        return self._observe(), {}

    def _pose(self):
        info = self.info
        return (s1.scalar(info["ground_truth_world_position_x_m"]),
                s1.scalar(info["ground_truth_world_position_y_m"]),
                s1.scalar(info["ground_truth_heading_in_world_frame_rad"]))

    def _observe(self) -> np.ndarray:
        self.pending.append(s1.disturbed_detections(self.obs, self._pose(), self.case, self.det_rng))
        delay_steps = round(self.age_s / s1.POLICY_PERIOD_S)
        cones = self.pending.pop(0) if len(self.pending) > delay_steps else None
        state = None
        if cones is not None:
            batch = filter_cones(ConeBatch(cones, SensorAge(self.age_s, None), 0.0), ConeFilterConfig())
            state = reference_from_cones(batch, self.case.lane.width_m)
        return observation(state, self.curvatures, self.age_s, self.case.speed_m_per_s)

    def step(self, action):
        weights = self.reward_weights
        curvature = float(np.clip(action[0], -1.0, 1.0)) * MAX_CURVATURE_PER_M
        rate = (curvature - self.curvatures[0]) / s1.POLICY_PERIOD_S
        self.curvatures = (curvature, self.curvatures[0])
        command = DriveCommand(self.case.speed_m_per_s, curvature)
        for _ in range(round(s1.POLICY_PERIOD_S / s1.ENV_STEP_S)):
            speed = abs(s1.scalar(self.obs["body_longitudinal_velocity_mps"]))
            self.obs, _, _, _, self.info = self.env.step(s1.command_to_action(command, speed))
        self.steps += 1
        x, y, heading = self._pose()
        lane = self.case.lane
        reward = (1.0 - weights.lateral * bounded_penalty(y, 0.1) - weights.heading * bounded_penalty(heading, 0.1)
                  - weights.curvature_rate * bounded_penalty(rate, 1.0))
        corners = s1.body_corners(x, y, heading)
        outside = float(np.max(np.abs(corners[:, 1]))) > lane.width_m / 2
        touched = any(s1.distance_to_body(cx, cy, x, y, heading) < s1.CONE_RADIUS_M
                      for _, _, cx, cy in s1.all_cones(lane) if abs(cx - x) < 0.5)
        terminated = False
        if outside or touched:
            reward -= weights.collision
            terminated = True
        elif x >= lane.length_m:
            reward += weights.finish
            terminated = True
        truncated = self.steps >= 200
        return self._observe(), float(reward), terminated, truncated, {}

    def close(self):
        if self.env is not None:
            self.env.close()


class RlController:
    """Runs a trained policy as the steering inside MpcPolicy, with MpcController's solve signature."""

    def __init__(self, config: MpcConfig, model_path: str | Path):
        """
        :param config: The MPC settings, for the horizon and the speed.
        :param model_path: A stable-baselines3 PPO model saved by train().
        """
        from stable_baselines3 import PPO

        self.config = config
        self.model = PPO.load(str(model_path), device="cpu")
        self.curvatures = (0.0, 0.0)

    def solve(self, state: PathTrackingState, previous_curvature_per_m: float, delay_s: float) -> MpcSolution:
        start = time.perf_counter()
        self.curvatures = (previous_curvature_per_m, self.curvatures[0])
        obs = observation(state, self.curvatures, float(delay_s), self.config.target_speed_m_per_s)
        action, _ = self.model.predict(obs, deterministic=True)
        curvature = float(np.clip(action[0], -1.0, 1.0)) * MAX_CURVATURE_PER_M
        return MpcSolution(np.full(self.config.horizon_steps, curvature), True, time.perf_counter() - start)


def train(reward_name: str, steps: int, out: Path, seed: int = 0, n_envs: int = 8) -> Path:
    """Trains one PPO policy with a reward option and saves it as out/<reward_name>.zip."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv

    reward = REWARDS[reward_name]
    env = make_vec_env(lambda: Stage1SteeringEnv(reward), n_envs=n_envs, seed=seed, vec_env_cls=SubprocVecEnv)
    model = PPO("MlpPolicy", env, seed=seed, verbose=0, n_steps=512, batch_size=256, learning_rate=3e-4,
                gamma=0.98, policy_kwargs=dict(net_arch=[64, 64]), device="cpu")
    model.learn(total_timesteps=steps)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{reward_name}.zip"
    model.save(str(path))
    env.close()
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    train_parser = sub.add_parser("train")
    train_parser.add_argument("--reward", choices=sorted(REWARDS), required=True)
    train_parser.add_argument("--steps", type=int, default=300_000)
    train_parser.add_argument("--seed", type=int, default=0)
    train_parser.add_argument("--out", type=Path, default=Path("rl_models"))
    args = parser.parse_args()
    if args.command == "train":
        start = time.perf_counter()
        path = train(args.reward, args.steps, args.out, args.seed)
        print(f"saved {path} after {time.perf_counter() - start:.0f} s")


if __name__ == "__main__":
    main()
