"""
run_timing.py

Times the policy steps of one run, so the policy calculation can be checked against the time between the detections
that trigger it (the time budget in docs/stage1-approach.md, section 1.5).

A run lasts from entering "Publishing policy actions" until leaving it. policy_node records each step with
RunTiming.record and logs RunTiming.summary when the run ends. With data logging on, each step's StepTiming is also
written to <stem>_timing.json (log_timing in data_logging.py), so the slow steps can be found in the drive afterwards.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from policy.serialisation import Serialisable


@dataclass(frozen=True)
class StepTiming(Serialisable):
    """How long one policy step's calculation took, and when the step ran."""
    policy_step: int
    """Number of the step in its run, from 0."""
    policy_elapsed_s: float
    """Seconds since entering the publishing-policy state, when the step started."""
    calculation_s: float
    """Monotonic seconds policy_node's calculate_policy_actions took, from just before the call until it returned or
    raised."""
    step_interval_s: float | None
    """Monotonic seconds since the previous step started, or None on the first step of the run."""
    trigger_age_s: float | None
    """Age in seconds of the sample from the sensor that triggers the steps (policy_update_mode) when the step started,
    or None in timer mode, which has no such sensor."""


def _nearest_rank(ordered: list[float], fraction: float) -> float:
    """
    :param ordered: Values in ascending order, at least one.
    :param fraction: Fraction in [0, 1] of the values that are at most the result.
    :return: The nearest-rank percentile of the values.
    """
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def _milliseconds(name: str, values: list[float]) -> str:
    """
    :param name: What the values are, such as "calculation".
    :param values: Durations in seconds, at least one.
    :return: Their median, 95th percentile and maximum in milliseconds, as text.
    """
    ordered = sorted(values)
    return (f"{name} median {_nearest_rank(ordered, 0.5) * 1000:.1f} ms, "
            f"p95 {_nearest_rank(ordered, 0.95) * 1000:.1f} ms, max {ordered[-1] * 1000:.1f} ms")


class RunTiming:
    """Collects the step timings of one run and summarises them when the run ends."""

    def __init__(self):
        self.steps: list[StepTiming] = []
        """The steps of the current run, in order."""

    def start(self):
        """Forgets the previous run. Call it when entering the publishing-policy state."""
        self.steps = []

    def record(self, policy_elapsed_s: float, calculation_s: float, step_interval_s: float | None,
               trigger_age_s: float | None) -> StepTiming:
        """Adds one policy step to the current run.

        :param policy_elapsed_s: Seconds since entering the publishing-policy state, when the step started.
        :param calculation_s: Seconds the policy calculation took.
        :param step_interval_s: Seconds since the previous step started, or None on the first step.
        :param trigger_age_s: Age in seconds of the triggering sensor's sample, or None in timer mode.
        :return: The recorded step.
        """
        step = StepTiming(len(self.steps), policy_elapsed_s, calculation_s, step_interval_s, trigger_age_s)
        self.steps.append(step)
        return step

    def summary(self, run_s: float, end_reason: str, stop_reason: str | None = None) -> str:
        """Describes the current run in one line for the log.

        :param run_s: Seconds the run lasted.
        :param end_reason: Why the node left the publishing-policy state, such as "Operator request".
        :param stop_reason: Why the policy had already stopped the car for the rest of the run
            (MovementPolicy.latched_stop_reason), or None.
        :return: The run's length and end, then the median, 95th percentile and maximum of the calculation time, the
            time between steps and the triggering sample's age.
        """
        text = f"Run summary: {run_s:.1f} s, {len(self.steps)} policy steps, ended by: {end_reason}"
        if stop_reason is not None:
            text += f"; the policy had stopped the car for {stop_reason}"
        if not self.steps:
            return text
        slowest = max(self.steps, key=lambda step: step.calculation_s)
        text += (f"; {_milliseconds('calculation', [step.calculation_s for step in self.steps])}"
                 f" (slowest at {slowest.policy_elapsed_s:.1f} s)")
        intervals = [step.step_interval_s for step in self.steps if step.step_interval_s is not None]
        if intervals:
            text += f"; {_milliseconds('step interval', intervals)}"
        ages = [step.trigger_age_s for step in self.steps if step.trigger_age_s is not None]
        if ages:
            text += f"; {_milliseconds('trigger sample age', ages)}"
        return text
