"""Experimental straight-lane endpoint approach, selected explicitly by simulations.

Not wired into MovementPolicy or ROS parameters. A stable farthest cone pair is
only a hypothesis: persistent occlusion can imitate a finish. These unmeasured
defaults and bounded dead reckoning must be evaluated before runtime integration.
"""
from dataclasses import dataclass, replace
import math

from policy.action_policy.mpc import PathTrackingState
from policy.action_policy.policy import MpcPolicy
from policy.action_policy.reference import reference_from_cones
from policy.input_output import ConeColour


@dataclass(frozen=True)
class EndpointConfig:
    """Offline experiment settings, not admitted robot configuration.

    Distances are metres, durations seconds, speeds m/s, deceleration m/s².
    The 2.5 m confirmation limit assumes the experiment's 4 m camera range.
    stop_offset_m targets the car centre just beyond the final pair, rather
    than declaring completion when its front first reaches the pair.
    """
    confirmation_frames: int = 3
    confirmation_range_m: float = 2.5
    pair_tolerance_m: float = 0.15
    max_observation_age_s: float = 0.35
    approach_speed_m_per_s: float = 0.5
    deceleration_m_per_s2: float = 0.5
    reaction_s: float = 0.3
    stop_offset_m: float = 0.15
    initial_uncertainty_m: float = 0.04
    uncertainty_per_m: float = 0.08
    max_uncertainty_m: float = 0.2
    max_blind_s: float = 4.0
    max_approach_s: float = 10.0
    stop_tolerance_m: float = 0.02

    def __post_init__(self):
        if (isinstance(self.confirmation_frames, bool) or not isinstance(self.confirmation_frames, int)
                or self.confirmation_frames < 2):
            raise ValueError("endpoint confirmation needs at least two distinct frames")
        for name, value in vars(self).items():
            if name != "confirmation_frames" and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"endpoint.{name} must be positive and finite")
        if self.initial_uncertainty_m >= self.max_uncertainty_m:
            raise ValueError("initial endpoint uncertainty must be below the stop limit")


class EndpointTracker:
    """Track distance to a possible final pair with speed-based dead reckoning.

    Only increasing acquisition stamps count as independent confirmations.
    A contradictory farther observation invalidates a confirmed hypothesis.
    Neither an empty frame nor a stale frame confirms an endpoint.
    """
    def __init__(self, config):
        self.config = config
        self.remaining_m = None
        self.uncertainty_m = config.initial_uncertainty_m
        self.confirmations = 0
        self.confirmed = False
        self.contradicted = False
        self.last_stamp = None

    def update(self, cones, state, speed, dt):
        config = self.config
        heading = 0.0 if state is None else state.heading_error_rad
        travelled = speed * dt * math.cos(heading)
        if self.remaining_m is not None:
            self.remaining_m -= travelled
            self.uncertainty_m += abs(travelled) * config.uncertainty_per_m
        if cones is None or state is None or abs(heading) > 0.35:
            if not self.confirmed:
                self.confirmations = 0
            return
        age = cones.sensor_age
        stamp = age.stamp_ns
        if (not math.isfinite(age) or not 0 <= age <= config.max_observation_age_s
                or stamp is None or (self.last_stamp is not None and stamp <= self.last_stamp)):
            return
        self.last_stamp = stamp
        # Projection on the lane direction, in the delayed camera observation.
        distances = {colour: [cone.pos.x * math.cos(heading) - cone.pos.y * math.sin(heading)
                              for cone in cones if cone.colour == colour] for colour in ConeColour}
        farthest = [max(row) for row in distances.values() if len(row) >= 2]
        all_distances = [value for row in distances.values() for value in row]
        delay_travel = speed * float(age) * math.cos(heading)
        if (self.confirmed and all_distances
                and max(all_distances) - delay_travel > self.remaining_m + 2 * config.pair_tolerance_m):
            self.confirmed = False
            self.contradicted = True
            return
        if len(farthest) != 2 or abs(farthest[0] - farthest[1]) > config.pair_tolerance_m:
            if not self.confirmed:
                self.confirmations = 0
            return
        distance = sum(farthest) / 2 - delay_travel
        if not 0 < distance <= config.confirmation_range_m:
            if not self.confirmed:
                self.confirmations = 0
            return
        if self.remaining_m is None or abs(distance - self.remaining_m) > config.pair_tolerance_m:
            if self.confirmed:
                self.confirmed = False
                self.contradicted = True
                return
            self.confirmations = 1
        else:
            self.confirmations += 1
        self.remaining_m = distance
        self.uncertainty_m = config.initial_uncertainty_m
        self.confirmed = self.confirmations >= config.confirmation_frames


def stopping_speed(remaining_m, uncertainty_m, config):
    """Speed cap satisfying v*reaction + v²/(2*a) <= usable stopping distance.

    This is a desired-speed envelope, not a guarantee of physical braking. The
    existing car controller requests effort only and has no calibrated brake.
    """
    usable = remaining_m + config.stop_offset_m - uncertainty_m
    if usable <= config.stop_tolerance_m:
        return 0.0
    reaction_distance_rate = config.deceleration_m_per_s2 * config.reaction_s
    return min(config.approach_speed_m_per_s,
               math.sqrt(reaction_distance_rate ** 2 + 2 * config.deceleration_m_per_s2 * usable)
               - reaction_distance_rate)


class EndpointMpcPolicy(MpcPolicy):
    """Simulation-only extension allowing a bounded approach to a confirmed pair.

    Before confirmation, ordinary lane-loss rules apply. After confirmation,
    fresh empty frames permit bounded dead reckoning of the straight reference.
    Missing/stale streams, missing speed, excessive uncertainty, timeout or
    contradictory detections latch zero. Restart is explicit, as in MpcPolicy.
    """
    def __init__(self, config, endpoint_config=None):
        self.endpoint_config = endpoint_config or EndpointConfig()
        super().__init__(config)

    def reset(self):
        super().reset()
        self.endpoint = EndpointTracker(self.endpoint_config)
        self.approaching_endpoint = False
        self.stopped_at_endpoint = False
        self.endpoint_abort_reason = None
        self.approach_elapsed_s = 0.0
        self.blind_s = 0.0
        self.reference = None
        self.reference_stamp = None

    def _abort(self, reason):
        self.endpoint_abort_reason = reason
        self.stopped_for_lane_loss = True
        return [self._command(0.0, 0.0)]

    def run_policy(self, observations, lanes):
        if observations.policy.is_first_policy_step:
            self.reset()
        if self.stopped_at_endpoint or self.stopped_for_lane_loss or self.stopped_for_solver_failure:
            return [self._command(0.0, 0.0)]
        speed, dt = observations.car.wheel_speed, observations.policy.dt
        if speed is None or not math.isfinite(speed) or speed < 0:
            return self._abort("missing or invalid wheel speed")
        if not math.isfinite(dt) or dt < 0:
            return self._abort("invalid step duration")
        cones = observations.cones
        age = None if cones is None else cones.sensor_age
        if (age is None or not math.isfinite(age) or not 0 <= age <= self.endpoint_config.max_observation_age_s):
            return self._abort("missing or stale detection stream")
        state = reference_from_cones(cones, self.config.lane_width_m)
        self.endpoint.update(cones, state, speed, dt)
        if self.endpoint.contradicted:
            return self._abort("endpoint contradicted")
        # Keep the last straight reference in the current vehicle frame. This
        # approximates motion using wheel speed and commanded, not measured, steering.
        if self.reference is not None:
            distance = speed * dt
            self.reference = PathTrackingState(
                self.reference.lateral_error_m + distance * self.reference.heading_error_rad
                + 0.5 * distance ** 2 * self.previous_curvature_per_m,
                self.reference.heading_error_rad + distance * self.previous_curvature_per_m, 0.0)
        if state is not None and age.stamp_ns is not None and (
                self.reference_stamp is None or age.stamp_ns > self.reference_stamp):
            distance = speed * float(age)
            self.reference = PathTrackingState(
                state.lateral_error_m + distance * state.heading_error_rad
                + 0.5 * distance ** 2 * self.previous_curvature_per_m,
                state.heading_error_rad + distance * self.previous_curvature_per_m, 0.0)
            self.reference_stamp = age.stamp_ns
            self.blind_s = 0.0
        else:
            self.blind_s += dt
        if not self.endpoint.confirmed and not self.approaching_endpoint:
            # reset() above already handles the first step; do not reset the
            # estimator a second time inside the base policy.
            return super().run_policy(replace(observations, policy=replace(
                observations.policy, is_first_policy_step=False)), lanes)
        self.approaching_endpoint = True
        self.approach_elapsed_s += dt
        limits = self.endpoint_config
        if (self.reference is None or self.blind_s > limits.max_blind_s
                or self.approach_elapsed_s > limits.max_approach_s
                or self.endpoint.uncertainty_m > limits.max_uncertainty_m):
            return self._abort("approach confidence or time limit")
        target_speed = min(self.config.target_speed_m_per_s,
                           stopping_speed(self.endpoint.remaining_m, self.endpoint.uncertainty_m, limits))
        if target_speed == 0:
            self.stopped_at_endpoint = True
            return [self._command(0.0, 0.0)]
        solution = self.controller.solve(self.reference, self.previous_curvature_per_m, 0.0, float(speed))
        self.solve_count += 1
        self.last_solve_time_s = solution.solve_time_s
        if not solution.converged:
            self.fallback_count += 1
            self.stopped_for_solver_failure = True
            return [self._command(0.0, 0.0)]
        return [self._command(target_speed, float(solution.curvatures_per_m[0]))]
