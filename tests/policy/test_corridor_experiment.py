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


def route_controller(**settings):
    return OfflineCorridorPolicy(1.,.92,CarMemory(),CorridorConfig(
        lookahead_m=1.1,route_memory_enabled=True,**settings),.4)


def test_saved_route_moves_with_car_and_single_edge_never_renews_anchor():
    controller=route_controller()
    controller.run_policy(observation(0,lane_cones()))
    anchor=controller.memory.data.corridor_experiment.route_seen_s
    initial=controller.memory.data.corridor_experiment.route
    assert anchor==0 and initial
    cones=[replace(c,pos=replace(c.pos,x=c.pos.x-.05)) for c in lane_cones() if c.colour==ConeColour.BLUE]
    assert controller.run_policy(observation(.1,cones))[0].speed_m_per_s>0
    state=controller.memory.data.corridor_experiment
    assert state.route_seen_s==anchor and state.route==initial
    assert controller.path[0][0] < initial[0][0]
    assert CarMemory.deserialise(controller.memory.data.serialise()).data==controller.memory.data


@pytest.mark.parametrize('limit', ['age','distance'])
def test_saved_route_expiry_stops_without_single_edge_renewal(limit):
    settings={'route_max_age_s':.25} if limit=='age' else {'route_max_distance_m':.12}
    controller=route_controller(**settings)
    controller.run_policy(observation(0,lane_cones()))
    for i in range(1,4):
        cones=[replace(c,pos=replace(c.pos,x=c.pos.x-.05*i)) for c in lane_cones() if c.colour==ConeColour.BLUE][:2]
        result=controller.run_policy(observation(i/10,cones))
    assert result[0].speed_m_per_s==0 and controller.memory.data.corridor_experiment.stopped
    assert controller.memory.data.corridor_experiment.route_seen_s==0


def test_one_edge_cannot_invent_saved_route_from_remembered_other_side():
    controller=route_controller()
    controller.run_policy(observation(0,lane_cones()))
    state=controller.memory.data.corridor_experiment
    controller.memory.update_corridor_experiment(replace(state,route=(),route_seen_s=None,route_seen_distance_m=None))
    result=controller.run_policy(observation(.1,[c for c in lane_cones() if c.colour==ConeColour.BLUE][:2]))
    assert result[0].speed_m_per_s==0


def test_unrelated_fresh_colours_do_not_renew_an_old_route():
    controller=route_controller()
    controller.run_policy(observation(0,lane_cones()))
    cones=[cone(x,y,colour) for x in (.5,1.,1.5) for colour,y in ((ConeColour.BLUE,1.3),(ConeColour.YELLOW,-1.3))]
    controller.run_policy(observation(.1,cones))
    assert controller.memory.data.corridor_experiment.route_seen_s==0


def test_missing_motion_clears_route_and_empty_detections_stop():
    controller=route_controller()
    controller.run_policy(observation(0,lane_cones()))
    obs=observation(.1,lane_cones())
    obs=replace(obs,car=replace(obs.car,ang_velocity=None))
    assert controller.run_policy(obs)[0].speed_m_per_s==0
    assert not controller.memory.data.corridor_experiment.route
    other=route_controller()
    other.run_policy(observation(0,lane_cones()))
    assert other.run_policy(observation(.1,[]))[0].speed_m_per_s==0


def test_old_corridor_memory_deserialises_without_route_fields():
    old={'started':False,'stopped':False,'curvature':0.,'filtered_tracks':[]}
    assert CorridorExperimentMemory.deserialise(old).route==()


def test_fresh_colours_on_same_side_cannot_qualify_two_edges():
    from policy.action_policy.corridor_experiment import route_support,route_sides
    path=((0.,0.),(1.,0.),(2.,0.))
    cones=[cone(x,y,colour) for x in (.5,1.) for colour,y in ((ConeColour.BLUE,.4),(ConeColour.YELLOW,.6))]
    assert not route_support(path,cones,CorridorConfig(),require_both=True)
    cones=[cone(x,y,colour) for x in (.5,1.) for colour,y in ((ConeColour.BLUE,.5),(ConeColour.YELLOW,-.5))]
    assert route_support(path,cones,CorridorConfig(),require_both=True,left_colour='BLUE')
    assert not route_support(path,cones,CorridorConfig(),require_both=True,left_colour='YELLOW')
    assert route_sides(path,cones,CorridorConfig())[ConeColour.BLUE]==1


def test_fresh_boundary_continuation_uses_known_width_and_connects_corner():
    from policy.action_policy.corridor_experiment import boundary_centre_path
    cones=[cone(x,y,ConeColour.YELLOW) for x,y in ((.4,-.5),(.9,-.5),(1.4,-.5),(1.4,0.),(1.4,.5),(1.4,1.))]
    path=boundary_centre_path(cones,ConeColour.YELLOW,'BLUE',1.,CorridorConfig())
    assert len(path)>=3 and path[0][1]==pytest.approx(0)
    assert path[-1][1]>.7 and path[-1][0]<1.4
    assert not boundary_centre_path(cones[:2],ConeColour.YELLOW,'BLUE',1.,CorridorConfig())


def test_boundary_nearest_cone_does_not_connect_backwards_at_start():
    from policy.action_policy.corridor_experiment import boundary_centre_path
    # Second-corner observation from failed seed209: the closest cone has a slightly closer neighbour behind it.
    cones=[cone(x,y,ConeColour.YELLOW) for x,y in ((.70,-.78),(.86,-.31),(1.12,.16),(1.35,.69),(1.54,1.21))]
    path=boundary_centre_path(cones,ConeColour.YELLOW,'BLUE',1.,CorridorConfig(lookahead_m=1.1))
    assert len(path)>=2
    assert pursuit_curvature(path,1.1) is not None
    assert path[-1][1]>.5


def test_boundary_simplification_preserves_corner_without_offset_loop():
    from policy.action_policy.corridor_experiment import simplify_boundary,boundary_centre_path
    import numpy as np
    boundary=[(.4,-.5),(.9,-.49),(1.4,-.5),(1.41,0.),(1.4,.5),(1.4,1.)]
    simplified=simplify_boundary(boundary)
    assert len(simplified)==3
    assert simplified[1]==pytest.approx((1.4,-.5))
    path=boundary_centre_path([cone(x,y,ConeColour.YELLOW) for x,y in boundary],
                              ConeColour.YELLOW,'BLUE',1.,CorridorConfig())
    # Correct inside offset of an L: follow east to(.9,0), then north; no extra backwards segment at corner.
    segments=np.diff(np.array(path),axis=0)
    assert (segments>=-1e-6).all()


def test_inward_offset_self_intersection_is_clipped():
    from policy.action_policy.corridor_experiment import remove_offset_loops
    import numpy as np
    points=np.array([[.75,.06],[.97,.01],[.84,-.05],[.88,.11],[.93,.62],[.99,1.08]])
    result=remove_offset_loops(points)
    assert len(result)<len(points)
    assert result[0]==pytest.approx(points[0]) and result[-1]==pytest.approx(points[-1])


def test_close_corner_cones_do_not_strand_forward_boundary_chain():
    from policy.action_policy.corridor_experiment import boundary_centre_path
    points=((.52,-.53),(1.06,-.74),(1.07,-.63),(1.2,-.14),(1.43,.38),(1.54,.89),(1.67,1.38))
    path=boundary_centre_path([cone(x,y,ConeColour.YELLOW) for x,y in points],
                              ConeColour.YELLOW,'BLUE',1.,CorridorConfig(lookahead_m=1.2))
    assert len(path)>=2
    assert pursuit_curvature(path,1.2)>0
