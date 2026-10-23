# ArUco marker commands

ArUco markers give the car commands: keep to the left, right or centre of the lane, or turn left, turn right or go
straight at the next intersection. Each command's marker ID is set in the `markers` parameters of
[config/ai4r_policy.yaml](../config/ai4r_policy.yaml).

| | |
| --- | --- |
| Code | `policy/markers/` (`MarkerConfig`, `MarkerReader`, `MarkerCommand`), applied in `policy/policy_runner.py` |
| Memory | `Memory.marker_sighting`, `Memory.intersection_command` (`policy/memory.py`) |
| Tests | `tests/policy/test_markers.py` |
| Status | Simulation and unit tests only; not yet run on a car |

## Commands

| Command | ID in the yaml | What it does |
| --- | --- | --- |
| `KEEP_LEFT` | 0 | Keeps to the left side of the lane until another side marker |
| `KEEP_RIGHT` | 9 | Keeps to the right side of the lane until another side marker |
| `KEEP_CENTRE` | 1 | Keeps to `mpc.lane_offset_m` (normally the centre) until another side marker |
| `TURN_LEFT` | 3 | Remembered as the next intersection's command |
| `TURN_RIGHT` | 4 | Remembered as the next intersection's command |
| `STRAIGHT` | 5 | Remembered as the next intersection's command |

Only the nearest marker in range is followed. A side marker's side is kept after the marker is passed, until another
side marker is confirmed; a turn marker does not change it.

The car does not turn at intersections yet. The turn commands are recorded in `Memory.intersection_command` for that
manoeuvre to use, and a later turn marker replaces an earlier one. Every new run (explicit resume) starts again at
`mpc.lane_side` with no intersection command.

## When a marker counts

The `aruco_detector` confirms a marker over `min_consecutive_frames` images (3 in `config/aruco_detector.yaml`).
The policy then counts a mapped marker only when:

- its pose is ahead of `base_link` and at most `markers.max_distance_m` (1.5 m) from it; a marker without a pose
  never counts, and
- it is the nearest such marker for `markers.confirm_steps` (2) policy steps in a row.

The command is given once, at that step. Losing the marker for a step, or a step without a fresh fiducial batch,
restarts the count. The distance is measured straight from `base_link`, so a marker standing 0.9 m to the side of the
car (0.2 m outside the cone row of a 1.4 m lane) only counts once it is about 1.2 m ahead.

The command and the lane side show in the Foxglove live drawing's text beside the car
(`side LEFT  next intersection TURN_RIGHT`), and every command is logged by `policy.markers.markers`.

## Simulation

`test_a_marker_switches_the_side_on_a_wide_track` drives four clean wide tracks that switch sides
([mpc-lane-side.md](mpc-lane-side.md)). A marker stands 0.2 m outside the cone row on the side to switch to. Its
position is given ahead of the car by the distance driven, not from a world pose or a camera model. The marker
switches the side within 0.15 m of where the track asks, and the car keeps
the lane within 0.07 m (median) of the side's line.

## Testing on the car

1. Upload with `scripts/car_management.sh upload_and_build`. The build is needed because `policy/markers/` is a new
   package; `upload` alone does not install it.
2. Check that the `aruco_detector` is running and its RGB image is on (`publish_rgb_image: true` in
   `config/oakd_cone_detector.yaml`; without it the detector gets no image).
3. Print the markers from `DICT_4X4_50` at the size in `default_marker_size_m` (0.25 m), or change the IDs in the
   yaml to match the printed ones.
4. With the Traxxas node disabled (it then sends neutral commands, so the car stays still), put the policy in
   "publishing actions": markers are only read in policy steps. Hold each marker 1 m ahead of the camera and check
   Foxglove's 3D panel (fiducials layer, `id N`) and the text beside the car. Put the policy back to zero actions
   afterwards.
5. Drive a wide lane with a `KEEP_LEFT` or `KEEP_RIGHT` marker beside it, 0.2 m outside the cone row, as in the
   lane side tests.

## Next

- The intersection manoeuvre that uses `Memory.intersection_command`, starting with the hook turn.
- Whether a marker should be placed or read differently at intersections (for example, only markers on the car's
  side of the lane).
