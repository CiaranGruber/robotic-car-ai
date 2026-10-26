"""Sensed T exits, closed mouths, unknown geometry and ordinary-lane preservation."""
from dataclasses import replace

import numpy as np
import pytest

from policy.action_policy.intersection import cross_fits
from policy.input_output import ConeBatch, ConeColour, ConeDetection, LidarCartesianPoint, Position, SensorAge, WheelSpeed
from policy.intersection_state import IntersectionMemory, IntersectionPose
from policy.markers.commands import MarkerCommand
from policy.memory import CarMemory, MpcStop
from tests.policy.intersection_simulation import lidar_frame, rollout
from tests.policy.intersection_turning import track_for
from tests.policy.test_intersection_navigation import navigator, observation


COMMANDS = {"left": MarkerCommand.TURN_LEFT, "right": MarkerCommand.TURN_RIGHT, "straight": MarkerCommand.STRAIGHT}


def sensed_t(closed, x=-2.4):
    track = replace(track_for(.2, corner_anchored=True), closed_exit=closed)
    pose = IntersectionPose(1., x, 0., 0., 3., .4, 0.)
    state = IntersectionMemory(poses=(pose,), frame_heading=0., width_m=2.4, frame_seen_s=1.)
    return track, state, replace(observation(time=1.), lidar_cartesian=lidar_frame(track, (x, 0., 0.)))


@pytest.mark.parametrize("closed", COMMANDS)
def test_t_geometry_closes_only_the_missing_arm(closed):
    track, _, _ = sensed_t(closed)
    cones = track.cones()
    assert len({(round(x, 8), round(y, 8)) for x, y, _ in cones}) == len(cones)
    for arm, pose in (("left", (0., 4., np.pi/2)), ("right", (0., -4., -np.pi/2)), ("straight", (4., 0., 0.))):
        if arm == closed:
            assert track.body_outside_m(pose) > 0.
            assert track.exit_at(pose) is None
        else:
            assert track.body_outside_m(pose) == 0.
            assert track.exit_at(pose) == arm
    assert track.body_outside_m((-4., 0., 0.)) == 0.


@pytest.mark.parametrize("closed", COMMANDS)
def test_lidar_supports_available_t_turns_and_positively_detects_the_cap(closed):
    nav, _ = navigator()
    _, state, obs = sensed_t(closed)
    centre, available, blocked = nav._junction_evidence(obs, state)
    assert centre == pytest.approx(0., abs=.03)
    assert COMMANDS[closed] in blocked and COMMANDS[closed] not in available
    for direction in ("left", "right"):
        if direction != closed:
            assert COMMANDS[direction] in available
    # Continued straight boundaries enter the finite scan later than side walls.
    _, state, obs = sensed_t(closed, -2.1)
    _, available, blocked = nav._junction_evidence(obs, state)
    if closed != "straight":
        assert MarkerCommand.STRAIGHT in available


def test_a_single_far_cone_or_missing_returns_cannot_establish_an_exit():
    nav, _ = navigator()
    _, state, obs = sensed_t("straight")
    points = [p for p in obs.lidar_cartesian.points if not (abs(p.pos.x-3.6) < .07 and p.pos.y > 1.1)]
    # One physical cone contributes three returns spanning only .05 m.
    points += [LidarCartesianPoint(Position(3.6, 1.6+d, .1), i) for i, d in enumerate((-.025, 0., .025))]
    obs = replace(obs, lidar_cartesian=replace(obs.lidar_cartesian, points=points))
    _, available, _ = nav._junction_evidence(obs, state)
    assert MarkerCommand.TURN_LEFT not in available
    assert nav._junction_evidence(replace(obs, lidar_cartesian=replace(obs.lidar_cartesian, points=[])), state) is None


def test_corner_only_support_does_not_claim_a_closed_mouth():
    nav, _ = navigator()
    _, state, obs = sensed_t("straight")
    points = [p for p in obs.lidar_cartesian.points if not (abs(p.pos.x-3.6) < .07 and abs(p.pos.y) < 1.05)]
    obs = replace(obs, lidar_cartesian=replace(obs.lidar_cartesian, points=points))
    _, available, blocked = nav._junction_evidence(obs, state)
    assert MarkerCommand.STRAIGHT not in available and MarkerCommand.STRAIGHT not in blocked


@pytest.mark.parametrize("command", (MarkerCommand.TURN_LEFT, MarkerCommand.STRAIGHT))
def test_lost_candidate_exit_evidence_stops_at_admission_deadline(command):
    nav, mpc = navigator()
    x = -1.6 if command is MarkerCommand.STRAIGHT else -2.3
    pose = IntersectionPose(1., x, 0., 0., 3., 0., 0.)
    state = IntersectionMemory(poses=(pose,), frame_heading=0., width_m=2.4, frame_seen_s=1.,
                               centre_u=0., command=command, command_seen_s=.9)
    memory = CarMemory()
    memory.update_intersection(state)
    obs = observation(time=1.1)
    obs = replace(obs, car=replace(obs.car, wheel_speed=WheelSpeed(0., SensorAge(0., None))))
    actions = nav.step(obs, None, memory, mpc)
    assert actions[0].brake and memory.data.mpc_stop is MpcStop.INTERSECTION
    assert "evidence lost" in memory.data.intersection.stop_reason


@pytest.mark.parametrize("closed", COMMANDS)
@pytest.mark.parametrize("direction", COMMANDS)
def test_t_requested_available_exit_completes_and_missing_exit_stops(closed, direction):
    track, _, _ = sensed_t(closed)
    result = rollout(direction, track=track, navigation=True)
    metrics = result["metrics"]
    assert not metrics["wrong_exit"] and metrics["cone_contacts"] == 0 and metrics["max_body_outside_m"] == 0.
    if direction == closed:
        assert metrics["policy_stop"] == "INTERSECTION" and metrics["actual_exit"] is None
        assert "closed" in metrics["intersection_stop_reason"]
        assert not any(p["phase"] == "turn" for p in result["phases"])
    else:
        assert metrics["navigation_pass"], (metrics, result["phases"])
        assert metrics["handoff_completed"]


def test_route_fit_uses_present_arms_and_legacy_memory_gets_unknown_exit_defaults():
    exits = (MarkerCommand.TURN_LEFT, MarkerCommand.TURN_RIGHT)
    assert cross_fits(0., 2.4, 2.5, MarkerCommand.TURN_LEFT, .3, .3, .25, exits=exits)
    assert not cross_fits(0., 2.4, 2.5, MarkerCommand.STRAIGHT, .3, .3, .25, exits=exits)
    state = replace(IntersectionMemory(), available_exits=exits, closed_exits=(MarkerCommand.STRAIGHT,))
    assert IntersectionMemory.deserialise(state.serialise()) == state
    legacy = {k: v for k, v in state.serialise().items()
              if k not in ("available_exits", "closed_exits", "frame_left_colour", "approach_end_u")}
    old = IntersectionMemory.deserialise(legacy)
    assert old.available_exits == () and old.closed_exits == () and old.frame_left_colour is None
    assert old.approach_end_u is None


def test_finite_incoming_corridor_keeps_command_pending_and_ends_before_the_mouth():
    nav, mpc = navigator()
    _, state, obs = sensed_t("straight", -3.2)
    state = replace(state, centre_u=0., command=MarkerCommand.TURN_LEFT, command_seen_s=.9,
                    approach_end_u=-1.7, frame_left_colour="BLUE")
    memory = CarMemory()
    memory.command_intersection(MarkerCommand.TURN_LEFT, .9)
    commands = nav._approach(obs, state, memory, mpc)
    assert commands[0].speed_m_per_s > 0. and not commands[0].brake
    assert memory.data.intersection.route == () and memory.data.intersection.phase == "approach"
    assert memory.data.intersection.approach_end_u == -1.7
    assert memory.data.intersection_command is MarkerCommand.TURN_LEFT
    assert nav.selected_lane.phase == "approach"
    assert max(p[0] for p in nav.selected_lane.points) <= 1.5+1e-8


@pytest.mark.parametrize("block", ("stopping_envelope", "obstacle", "missing_row"))
def test_temporary_approach_keeps_stopping_and_raw_lidar_checks(block):
    nav, mpc = navigator()
    _, state, obs = sensed_t("straight", -2.15 if block == "stopping_envelope" else -3.2)
    state = replace(state, centre_u=0., command=MarkerCommand.STRAIGHT, command_seen_s=.9,
                    approach_end_u=-1.7, frame_left_colour="BLUE")
    points = list(obs.lidar_cartesian.points)
    if block == "obstacle":
        points.append(LidarCartesianPoint(Position(.25, 0., .1), 0))
    elif block == "missing_row":
        points = [p for p in points if p.pos.y > 0.]
    obs = replace(obs, lidar_cartesian=replace(obs.lidar_cartesian, points=points))
    memory = CarMemory()
    actions = nav._approach(obs, state, memory, mpc)
    assert actions[0].brake and memory.data.mpc_stop is MpcStop.INTERSECTION
    assert nav.selected_lane is None and memory.data.intersection.route == ()


def test_fresh_rows_renew_evidence_without_moving_an_anchored_junction_frame():
    nav, _ = navigator()
    poses = tuple(IntersectionPose(t, 0., 0., 0., 0., 0., 0.) for t in (.8, 1., 1.1))
    state = IntersectionMemory(poses=poses, frame_heading=0., frame_left_colour="BLUE", width_m=2.4,
        centre_u=3.5, approach_end_u=1.8, frame_seen_s=.9)
    cones = [ConeDetection(Position(x, y+.005*x, .1), colour, .9)
             for y, colour in ((1.2, ConeColour.BLUE), (-1.2, ConeColour.YELLOW)) for x in (1., 2., 3.)]
    obs = replace(observation(), cones=ConeBatch(cones, SensorAge(.1, None), .1))
    updated = nav._observe(obs, None, state)
    assert updated.frame_seen_s == 1.
    assert (updated.frame_heading, updated.centre_v, updated.width_m) == (0., 0., 2.4)
    assert (updated.centre_u, updated.approach_end_u) == (3.5, 1.8)


@pytest.mark.parametrize("direction", ("left", "right"))
def test_existing_clean_point_four_metre_cross_remains_navigable(direction):
    result = rollout(direction, track=track_for(.4), navigation=True)
    assert result["metrics"]["navigation_pass"], result["metrics"]
