"""Guard against optimistic sensor/scoring errors in the offline evaluator."""
import numpy as np

from evaluate_mpc import Case, MpcConfig, detect, pareto_front, rollout
from evaluate_mpc_speed import spatial_error


def test_camera_cannot_see_behind_or_past_the_end_of_the_finite_lane():
    assert detect((6.1, 0.0, 0.0), Case("end"), np.random.default_rng(0), 0) == []
    cones = detect((0.0, 0.0, 0.0), Case("start"), np.random.default_rng(0), 0)
    assert cones
    assert all(0 < cone.pos.x <= 4 for cone in cones)


def test_delayed_start_waits_and_early_lane_end_stop_is_not_a_pass():
    metrics, trace = rollout(MpcConfig(), Case("delay", delay_steps=2), 0)
    assert np.all(trace[:2, 4] == 0)
    assert metrics["stopped"]
    if metrics["stop_x_m"] < 6:
        assert not metrics["pass"] and not metrics["completed"]


def test_pareto_excludes_dominated_designs_but_keeps_tradeoffs():
    summaries = {
        "accurate": {"mean_rms_lateral_m": 1, "mean_rms_curvature_rate": 3},
        "smooth": {"mean_rms_lateral_m": 2, "mean_rms_curvature_rate": 1},
        "dominated": {"mean_rms_lateral_m": 2, "mean_rms_curvature_rate": 4},
    }
    assert pareto_front(summaries) == ["accurate", "smooth"]


def test_spatial_score_excludes_stopped_tail_and_uses_identical_finish():
    trace = np.array([[0, 0, 0.2], [1, 3, 0.2], [2, 5, 0.2], [10, 5, 0.0]])
    score, length = spatial_error(trace)
    assert np.isclose(score, 0.2)
    assert length == 4.5


def test_dynamic_rollout_keeps_integrating_after_zero_command():
    metrics, trace = rollout(MpcConfig(), Case("lag", speed_response_s=0.5), 0, measured_speed=True)
    coasting = trace[(trace[:, 4] == 0) & (trace[:, 6] > 0.01)]
    assert len(coasting) > 1
    assert coasting[-1, 1] > coasting[0, 1]
    assert metrics["stopped"]
