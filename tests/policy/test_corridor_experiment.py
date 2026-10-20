"""Tests for offline centreline topology, feasibility screening and experimental control."""
from dataclasses import replace
import math

import pytest

from policy.action_policy.corridor_experiment import OfflineCorridorPolicy
from policy.input_output import ConeColour, ConeDetection, Position
from policy.lane_detection.corridor import CorridorConfig, cone_corridor, left_uturn_span_check, pursuit_curvature
from policy.memory import CarMemory
from policy.memory import ConeTrack, CorridorExperimentMemory
from tests.policy.angular_lanes import AngularUTurn
from tests.policy.curved_lanes import CurveCase, run_curve
from tests.policy.test_lane_detection import lane_cones
from tests.policy.test_temporal_lane import observation


def cone(x, y, colour):
    return ConeDetection(Position(x, y, 0.05), colour, 0.9)


def test_connected_midpoints_follow_a_corner_instead_of_extending_one_arc():
    lane = AngularUTurn(placement_jitter_m=0)
    cones = [cone(x - 1.5, y, colour) for colour, _, x, y in lane.cones()]
    path = cone_corridor(cones)
    assert len(path) > 5
    assert max(y for _, y in path) > 2.5
    assert max(x for x, _ in path) > 1.2
    assert path[-1][0] < path[len(path) // 2][0]  # Return leg runs in the opposite direction.


@pytest.mark.parametrize("cones", [[], [cone(1, 0.4, ConeColour.BLUE)] * 5,
                                  [cone(i, 0.4, ConeColour.BLUE) for i in range(6)],
                                  [cone(i, 0.0, ConeColour.BLUE if i % 2 else ConeColour.YELLOW) for i in range(6)]])
def test_missing_or_degenerate_corridor_has_no_route(cones):
    assert cone_corridor(cones) == ()


def test_gate_width_limit_rejects_pairs_across_disconnected_lanes():
    cones = [cone(x, y, colour) for x in (0.5, 1, 1.5) for y, colour in
             ((2, ConeColour.BLUE), (-2, ConeColour.YELLOW))]
    assert not cone_corridor(cones)


def test_forward_target_sign_and_bounds():
    assert pursuit_curvature(((0.4, 0), (1, 0.4)), 1) > 0
    assert pursuit_curvature(((0.4, 0), (1, -0.4)), 1) < 0
    assert pursuit_curvature(((-1, 0), (-2, 0)), 1) is None
    with pytest.raises(ValueError):
        CorridorConfig(max_cones=1000)


def test_measured_left_steering_limit_rules_out_original_u_even_for_point_car():
    result = left_uturn_span_check(0.48, 3.8)
    assert result["ruled_out"]
    assert result["necessary_span_m"] == pytest.approx(4.1666666667)
    assert not left_uturn_span_check(1.2, 3.8)["ruled_out"]  # Not a feasibility guarantee.
    for limit in (0, math.nan, math.inf):
        with pytest.raises(ValueError):
            left_uturn_span_check(limit, 3.8)


def test_same_u_can_be_followed_with_explicit_hypothetical_sharper_steering():
    case = CurveCase(AngularUTurn(), max_duration_s=40)
    case = replace(case, car=replace(case.car, max_left_curvature_per_m=1.2, max_right_curvature_per_m=1.2))
    memory = CarMemory()
    controller = OfflineCorridorPolicy(1.2, 1.2, memory, CorridorConfig(lookahead_m=1.2))
    result = run_curve(case, controller)
    assert result.lane_kept
    assert max(abs(c.curvature_per_m) for c in result.commands) <= 1.2
    assert CarMemory.deserialise(memory.data.serialise()).data == memory.data


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
def test_slower_hypothetical_profile_keeps_u_across_camera_noise_seeds(seed):
    case = CurveCase(AngularUTurn(), max_duration_s=45, seed=seed, speed_m_per_s=0.4)
    case = replace(case, car=replace(case.car, max_left_curvature_per_m=1.2, max_right_curvature_per_m=1.2))
    controller = OfflineCorridorPolicy(1.2, 1.2, CarMemory(), CorridorConfig(lookahead_m=1.2), 0.4)
    assert run_curve(case, controller).lane_kept


def test_unavailable_motion_clears_unstarted_smoothed_tracks_before_new_local_origin():
    memory = CarMemory()
    track = ConeTrack(1, 0.4, 0.05, "BLUE", 0.9, 0, 0)
    memory.update_corridor_experiment(CorridorExperimentMemory(filtered_tracks=(track,)))
    controller = OfflineCorridorPolicy(1.2, 1.2, memory)
    obs = observation(0.1, lane_cones())
    obs = replace(obs, car=replace(obs.car, ang_velocity=None))
    assert controller.run_policy(obs)[0].speed_m_per_s == 0
    assert not memory.data.corridor_experiment.filtered_tracks


def test_rollout_spacing_is_bounded_at_maximum_admitted_speed(monkeypatch):
    import policy.action_policy.corridor_experiment as module
    original = module.np.linspace
    calls = []

    def record(start, stop, count):
        calls.append((start, stop, count))
        return original(start, stop, count)

    monkeypatch.setattr(module.np, "linspace", record)
    controller = OfflineCorridorPolicy(1.2, 1.2, CarMemory())
    controller.run_policy(observation(0, lane_cones(), speed=2))
    horizons = [(stop, count) for start, stop, count in calls if start == 0 and stop == 1.6]
    assert horizons and all(stop / (count - 1) <= 0.025 for stop, count in horizons)
