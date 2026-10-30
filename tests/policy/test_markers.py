"""Checks the ArUco marker commands (policy/markers): which markers count, when a command is given, and what it does to
the MPC's lane side and the next intersection, including on the wide synthetic tracks (wide_tracks.py)."""
import dataclasses
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import wide_tracks as wt  # noqa: E402
from policy.action_policy.intersection import IntersectionConfig  # noqa: E402
from policy.action_policy.mpc import MpcConfig  # noqa: E402
from policy.action_policy.policy import MpcPolicy  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig  # noqa: E402
from policy.control.control import ControlConfig  # noqa: E402
from policy.input_output import (CarObservations, ConeBatch, ConeColour, ConeDetection, FiducialBatch,  # noqa: E402
                                 FiducialDetection, FiducialPose, ImagePoint, ObservedCarState, PolicyState,
                                 Position, Quaternion, SensorAge, WheelSpeed)
from policy.markers.commands import MarkerCommand  # noqa: E402
from policy.markers.markers import MarkerConfig, MarkerReader  # noqa: E402
from policy.memory import CarMemory, Memory  # noqa: E402
from policy.policy_runner import MovementPolicy  # noqa: E402

CONFIG = MarkerConfig(keep_left_id=1, keep_right_id=2, turn_left_id=3, turn_right_id=4, straight_id=5,
                      keep_centre_id=6)
CORNERS = (ImagePoint(0.0, 0.0), ImagePoint(1.0, 0.0), ImagePoint(1.0, 1.0), ImagePoint(0.0, 1.0))


def marker(marker_id, x=1.0, y=0.5, posed=True):
    """A marker detection at (x, y) in base_link, or without a pose."""
    pose = FiducialPose(Position(x, y, 0.2), Quaternion(0.0, 0.0, 0.0, 1.0)) if posed else None
    return FiducialDetection(marker_id, 0.25, CORNERS, pose, 0.5)


def straight_lane(lane_width_m=1.6, markers=None, first=False):
    """Observations of a car on the centre line of a straight lane at 0.5 m/s, with these marker detections (None for
    no fresh fiducial batch)."""
    cones = [ConeDetection(Position(x * 0.5, side * lane_width_m / 2.0, 0.1), colour, 0.9) for x in range(1, 7)
             for side, colour in ((1.0, ConeColour.BLUE), (-1.0, ConeColour.YELLOW))]
    fiducials = None if markers is None else FiducialBatch(markers, SensorAge(0.0, None), "DICT_4X4_50",
                                                           "oak_rgb_camera_optical_frame")
    return CarObservations(ConeBatch(cones, SensorAge(0.0, None), 0.0), fiducials, None, None,
                           ObservedCarState(None, None, None, WheelSpeed(0.5, SensorAge(0.0, None))),
                           PolicyState(0.0 if first else 0.1, 0.0, first))


def read(reader, memory, *batches):
    """Runs the reader over one batch per step and returns the commands it gave."""
    return [reader.step(straight_lane(markers=batch), memory) for batch in batches]


def test_settings_are_checked():
    with pytest.raises(ValueError, match="share an ID"):
        MarkerConfig(keep_left_id=3, turn_left_id=3)
    with pytest.raises(ValueError, match="_id"):
        MarkerConfig(keep_left_id=-2)
    with pytest.raises(ValueError, match="max_distance_m"):
        MarkerConfig(max_distance_m=0.0)
    with pytest.raises(ValueError, match="confirm_steps"):
        MarkerConfig(confirm_steps=0)
    assert MarkerConfig().commands == {}
    assert CONFIG.commands == {1: MarkerCommand.KEEP_LEFT, 2: MarkerCommand.KEEP_RIGHT, 3: MarkerCommand.TURN_LEFT,
                               4: MarkerCommand.TURN_RIGHT, 5: MarkerCommand.STRAIGHT, 6: MarkerCommand.KEEP_CENTRE}


@pytest.mark.parametrize("detection", [
    marker(1, posed=False),  # No pose, so no distance
    marker(1, x=1.6, y=0.0),  # Beyond max_distance_m
    marker(1, x=-0.5, y=0.3),  # Behind base_link
], ids=["no-pose", "too-far", "behind"])
def test_markers_that_do_not_count(detection):
    reader, memory = MarkerReader(CONFIG), CarMemory()
    assert read(reader, memory, *[[detection]] * 4) == [None] * 4
    assert memory.data.marker_sighting is None


def test_a_command_is_given_once_after_confirm_steps():
    reader, memory = MarkerReader(CONFIG), CarMemory()
    assert read(reader, memory, *[[marker(1)]] * 4) == [None, MarkerCommand.KEEP_LEFT, None, None]
    assert memory.data.marker_sighting.steps == 4


def test_a_gap_restarts_the_confirmation():
    reader, memory = MarkerReader(CONFIG), CarMemory()
    # No fresh batch, then an empty batch: both break the run of sightings
    assert read(reader, memory, [marker(2)], None, [marker(2)], [], [marker(2)], [marker(2)]) == \
        [None, None, None, None, None, MarkerCommand.KEEP_RIGHT]


def test_the_nearest_marker_counts_even_if_its_id_is_unsupported():
    reader, memory = MarkerReader(CONFIG), CarMemory()
    batch = [marker(9, x=0.5), marker(4, x=1.2), marker(3, x=0.8, y=-0.4)]
    assert read(reader, memory, batch, batch) == [None, MarkerCommand.STOP]
    assert memory.data.intersection_command is None


def test_only_the_nearest_marker_is_followed():
    reader, memory = MarkerReader(CONFIG), CarMemory()
    both = [marker(1, x=1.2), marker(2, x=0.6)]
    assert read(reader, memory, both, both, both) == [None, MarkerCommand.KEEP_RIGHT, None]
    # Once the nearer one is passed, the farther one is followed
    assert read(reader, memory, [marker(1, x=0.4)], [marker(1, x=0.3)]) == [None, MarkerCommand.KEEP_LEFT]


def test_intersection_commands_replace_each_other_and_a_new_run_forgets_them():
    reader, memory = MarkerReader(CONFIG), CarMemory()
    read(reader, memory, [marker(4)], [marker(4)])
    assert memory.data.intersection_command is MarkerCommand.TURN_RIGHT
    read(reader, memory, [marker(5)], [marker(5)])
    assert memory.data.intersection_command is MarkerCommand.STRAIGHT
    memory.start_marker_run()
    assert memory.data.intersection_command is None and memory.data.marker_sighting is None


def test_memory_with_markers_saves_and_loads():
    reader, memory = MarkerReader(CONFIG), CarMemory()
    read(reader, memory, [marker(3)], [marker(3)])
    assert Memory.deserialise(memory.data.serialise()) == memory.data
    # Recordings made before the markers were kept in memory still load
    old = {key: value for key, value in memory.data.serialise().items()
           if key not in ("marker_sighting", "intersection_command")}
    assert Memory.deserialise(old).intersection_command is None


def test_markers_set_the_lane_side_through_the_movement_policy():
    # The turn marker only sets the intersection command here: this lane never ends, so no turn is driven
    policy = MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig(), marker_config=CONFIG,
                            intersection_config=IntersectionConfig(enabled=False))
    memory = CarMemory()
    policy.step(straight_lane(first=True), memory)
    assert policy.action_policy.lane_side == "CENTRE"
    for _ in range(2):
        policy.step(straight_lane(markers=[marker(1)]), memory)
    assert policy.marker_command is MarkerCommand.KEEP_LEFT and policy.action_policy.lane_side == "LEFT"
    # The side stays after the marker goes, and the line moves over to it
    for _ in range(100):
        policy.step(straight_lane(markers=[]), memory)
    assert policy.action_policy.lane_side == "LEFT"
    assert policy.lane_path.usual_offset_m == pytest.approx(0.4, abs=0.02)
    # A turn marker does not move the car across the lane
    for _ in range(2):
        policy.step(straight_lane(markers=[marker(4)]), memory)
    assert policy.action_policy.lane_side == "LEFT" and policy.intersection_command is MarkerCommand.TURN_RIGHT
    for _ in range(2):
        policy.step(straight_lane(markers=[marker(2)]), memory)
    assert policy.action_policy.lane_side == "RIGHT"
    # The centre marker moves it back to the centre, which also stays
    for _ in range(2):
        policy.step(straight_lane(markers=[marker(6)]), memory)
    for _ in range(100):
        policy.step(straight_lane(markers=[]), memory)
    assert policy.action_policy.lane_side == "CENTRE"
    assert policy.lane_path.usual_offset_m == pytest.approx(0.0, abs=0.02)
    # A new run starts again at mpc.lane_side with no intersection command
    policy.step(straight_lane(first=True), memory)
    assert policy.action_policy.lane_side == "CENTRE" and policy.intersection_command is None


class MarkerSchedule:
    """Shows the MPC a marker beside a wide track's lane, ahead at the distance driven where the track switches side,
    and applies the marker's command as MovementPolicy does. The marker stands 0.2 m outside the cone row on the side to
    switch to, as far beyond the switch as it is ahead when it comes within max_distance_m, so it comes in range where
    SideSchedule would switch."""

    def __init__(self, policy: MpcPolicy, track: wt.WideTrack, config: MarkerConfig = CONFIG):
        self.policy, self.track = policy, track
        self.reader, self.memory = MarkerReader(config), CarMemory()
        marker_ids = {command: marker_id for marker_id, command in config.commands.items()}
        self.marker_id = marker_ids[MarkerCommand.KEEP_LEFT if track.switch_to == "LEFT" else MarkerCommand.KEEP_RIGHT]
        self.beside_m = (track.case.lane.width_m / 2.0 + 0.2) * (1.0 if track.switch_to == "LEFT" else -1.0)
        in_range_m = math.sqrt(config.max_distance_m ** 2 - self.beside_m ** 2)
        self.marker_at_m = wt.cl.RUN_IN_M + track.switch_after_m + in_range_m - 0.05
        self.driven_m = 0.0
        self.switched_at_m = None

    def run_policy(self, observations, lanes):
        self.driven_m += float(observations.car.wheel_speed) * observations.policy.dt
        detection = marker(self.marker_id, self.marker_at_m - self.driven_m, self.beside_m)
        batch = FiducialBatch([detection], SensorAge(0.0, None), "DICT_4X4_50", "oak_rgb_camera_optical_frame")
        command = self.reader.step(dataclasses.replace(observations, fiducials=batch), self.memory)
        if command is not None and command.lane_side is not None:
            self.policy.lane_side = command.lane_side
            self.switched_at_m = self.driven_m
        return self.policy.run_policy(observations, lanes)


@pytest.mark.parametrize("seed", [seed for seed in wt.TRAINING_SEEDS[:30] if wt.wide_track(seed).switch_to][:4])
def test_a_marker_switches_the_side_on_a_wide_track(seed, monkeypatch):
    """On a clean wide track that switches sides, a marker beside the lane at the switch moves the car to the other
    side, as well as asking for the switch directly does (test_wide_tracks.py)."""
    track = wt.clean_track(wt.wide_track(seed))
    schedules = []

    def marker_schedule(policy, side_track):
        schedules.append(MarkerSchedule(policy, side_track))
        return schedules[-1]

    monkeypatch.setattr(wt, "SideSchedule", marker_schedule)
    result, scores = wt.run_track(track)
    schedule = schedules[0]
    switch_m = wt.cl.RUN_IN_M + track.switch_after_m
    assert schedule.switched_at_m == pytest.approx(switch_m, abs=0.15)
    assert result.lane_kept
    assert scores["side_error_median_m"] < 0.07 and scores["scored_m"] > 1.0
    assert math.copysign(1.0, result.lateral_m[-1]) == (1.0 if track.switch_to == "LEFT" else -1.0)
