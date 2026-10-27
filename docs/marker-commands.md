# ArUco marker commands

ArUco markers give the car commands: keep to the left, right or centre of the lane, or turn left, turn right or go
straight at an intersection. Each command's marker ID is set in the `markers` parameters of
[config/ai4r_policy.yaml](../config/ai4r_policy.yaml); how the car turns is set in its `intersection` parameters.

| | |
| --- | --- |
| Code | `policy/markers/`, `policy/action_policy/intersection.py` (the turn), `policy/policy_runner.py` |
| Memory | `Memory.marker_sighting`, `Memory.intersection_command`, `Memory.intersection_turn` (`policy/memory.py`) |
| Tests | `tests/policy/test_markers.py`, `tests/policy/test_intersection.py` |
| Simulation | `tests/policy/marker_simulation.py`: the whole MovementPolicy through lane changes and intersections |
| Status | Simulation and unit tests only; the turn has not run on a car |

## Commands

| Command | ID in the yaml | What it does |
| --- | --- | --- |
| `KEEP_LEFT` | 0 | Keeps to the left side of the lane until another side marker |
| `KEEP_RIGHT` | 9 | Keeps to the right side of the lane until another side marker |
| `KEEP_CENTRE` | 1 | Keeps to `mpc.lane_offset_m` (normally the centre) until another side marker |
| `TURN_LEFT` | 3 | Turns left at the intersection the marker stands at |
| `TURN_RIGHT` | 4 | Turns right at the intersection |
| `STRAIGHT` | 5 | Drives straight across the intersection |

Only the nearest marker in range is followed. A side marker's side is kept after the marker is passed, until another
side marker is confirmed. Every new run (explicit resume) starts again at `mpc.lane_side` with no intersection command.

## When a marker counts

The `aruco_detector` confirms a marker over `min_consecutive_frames` images (3 in `config/aruco_detector.yaml`).
The policy then counts a mapped marker only when:

- its pose is ahead of `base_link` and at most `markers.max_distance_m` (2.0 m) from it; a marker without a pose
  never counts, and
- it is the nearest such marker for `markers.confirm_steps` (2) policy steps in a row.

The command is given once, at that step. Losing the marker for a step, or a step without a fresh fiducial batch,
restarts the count (giving the same command again changes nothing). The distance is measured straight from
`base_link`, so a marker beside the road counts later than one straight ahead.

The pose's distance is only right when `default_marker_size_m` in `config/aruco_detector.yaml` is the printed
marker's size (the outer black square): 0.081 m for the markers used on 2026-10-21. A smaller print than configured
looks farther away and may never come within range.

## Lane changes

A side marker sets the MPC's `lane_side`. The MPC then follows the line `mpc.edge_clearance_m` from that side's
cones, moving over at most `mpc.offset_change_per_m` per metre driven.

The defaults were 0.40 m and 0.2 per metre. With 0.40 m an 0.8 m lab lane has no room at all and a 1 m lane only
0.1 m, which is why keeping left was hard to see on car 21. The yaml now uses **0.28 m** and **0.3 per metre**:
0.12 m of offset in an 0.8 m lane and 0.22 m in a 1 m lane (a 24 cm and 44 cm swing from left to right), crossed in
about 1.5 m.

| `edge_clearance_m` / rate | 0.8 m road | 1 m road | Cones touched | Body inside the lane by at least |
| --- | --- | --- | --- | --- |
| 0.40 / 0.2 (old) | 6/6, but no offset | 6/6, 0.10 m offset | 0 | 0.25 m |
| 0.28 / 0.2 | 6/6 | 0/6 (still crossing 1.4 m after the marker) | 0 | 0.13 m |
| **0.28 / 0.3** | **6/6** | **6/6** | **0** | **0.13 m** |
| 0.25 / 0.3 | 6/6 | 6/6 | 0 | 0.10 m |

Each run passes three markers (keep left, then right, then centre, 3 m apart) and passes when the car's median
position over the 0.8 m before the next marker counts is within 0.04 m of the side's line. Over 20 seeds with the
chosen settings: **40/40** runs, worst side error 0.017 m. These roads are straight. The 0.40 m default came from
wide curved lanes, where 0.30-0.35 m touched the outer cones with 3 degrees of camera yaw: put side markers on
straights.

## Intersections

A turn marker stands like a road sign beside the approach road, about 1 m before the intersection
(`intersection.marker_to_entry_m: 1.0`): beside the last cones it was confirmed too late, when the cross road's cones
had already bent the fitted lane and the MPC had stopped for a lost lane. When the marker is confirmed, the policy
places the intersection's entry from the marker's position and the car's pose (the local map's dead reckoning, with
the IMU heading), and remembers the approach lane's centre line and direction. Then, in phases:

1. **Approach.** The MPC keeps following the lane, refining the remembered centre line. Lane detection only gets the
   approach road's cones (not those past the entry or outside the lane's edges). For a turn, the MPC keeps to the
   outside of the lane (left before a right turn, as a Melbourne hook turn does), 0.25 m from its edge: from the middle
   of a lab-width lane the car cannot steer tightly enough to clear both the inside corner and the far side of the
   road it turns into. From 0.8 m before the entry, where the lane runs out, the car holds the approach direction
   at 0.3 m/s.
2. **Arc.** At the planned start, the car follows a quarter circle (1.1 m radius left, 1.2 m right) round the inside
   corner, with pure pursuit on the circle. The start puts the arc's end on the exit lane's centre line where it
   can, and otherwise as close as it can while keeping the car's body 0.03 m clear of the inside corner cone (its
   inner side, `radius - half width` from the arc's centre) and of the exit's far row (its outer front corner,
   `sqrt(front^2 + (radius + half width)^2)`).
3. **Exit.** Lane following takes over once the car has turned 60 degrees and sees a lane lined up with it, or after
   the quarter circle along the exit line. Going straight, the car holds the approach line across the intersection
   until it sees the exit lane. Without an exit lane within 1.5 m, it stops for the rest of the run.

The obstacle stop keeps working throughout, along the path the manoeuvre drives (a straight corridor would take the
cones of the road being turned into for obstacles). Foxglove draws the plan (`/plan/intersection`, orange: the entry,
the arc, the exit line) and the text beside the car shows the phase (`turn approach`, `turn arc`, `turn exit`).

### Simulation

`tests/policy/marker_simulation.py` drives the whole MovementPolicy with `config/ai4r_policy.yaml`: the simulated
camera's cones and markers (in view, 0.3-2.5 m, 0.03 m noise, 10% missed, 0.2 s late), the lidar's points, the wheel
speed and the IMU heading. The car is the lab car with car 21's provisional steering limits: 1.0 1/m left (Yuhao's
reported 1 m radius) and 0.92 1/m right, the camera pan held at zero. Intersections are four-way or T crossings of
roads as wide as the approach road; the roads and the car 21 limits are assumptions, not measurements of the lab.

Over 20 noise seeds each:

| Case | 1 m roads | 0.8 m roads |
| --- | --- | --- |
| Four-way, turn left | 20/20 | 20/20 |
| Four-way, turn right | 20/20 | 19/20 |
| Four-way, straight on | 20/20 | 20/20 |
| T, turn left | 20/20 | 20/20 |
| T, turn right | 20/20 | 18/20 |
| Keep left 1.5 m before, then turn right | | 18/20 |

A pass leaves by the commanded road without touching a cone or the body leaving the roads. In 1 m roads the body
stayed at least 0.07 m inside the roads. The failures in 0.8 m roads clip the first cone of the exit's far row with
the car's front corner: right turns there are at the edge of what car 21's provisional 0.92 1/m right limit allows,
so they need the real limit measured. Policy steps took 5 ms median and 15 ms at the 99th percentile in the
manoeuvre (on a laptop), within the 20 ms budget.

Tuning, each over the turn cases: `corner_clearance_m` 0.05 passed 53/60 in 0.8 m roads against **59/60 at 0.03**
(fresh seeds); moving aside to 0.22 m or using tighter radii made it worse, and holding the line from 1.2 m changed
little.

```bash
python3 tests/policy/marker_simulation.py                     # every case, 3 seeds, figure in logs/marker_simulation/
python3 tests/policy/marker_simulation.py --cases cross_right_1m --seeds 5
python3 tests/policy/marker_simulation.py --set intersection.right_turn_radius_m=1.3
python3 tests/policy/marker_simulation.py --export logs/marker_simulation/scenarios
python3 -m scripts.mcap_export -p logs/marker_simulation/scenarios/cross_right_1m_s0_scenario.json
```

## Testing on the car

1. **Camera image for ArUco.** `aruco_detector` reads the RGB image `oakd_cone_detector` publishes
   (`publish_rgb_image: true` in `config/oakd_cone_detector.yaml`). The usual runtime starts `oakd_gpu_cone_detector`
   instead, and on car 21 on 2026-10-21 no image was published and no marker reached the policy in any of 9 runs.
   Start the runtime with `--aruco`, which stops the GPU detector and runs `oakd_cone_detector`:
   `scripts/car_management.sh dry_copy`, then `scripts/car_management.sh upload_and_build --aruco`. Check that lane
   following still works with that detector before relying on it.
2. **Markers are seen.** On the car: `ros2 topic hz /car/fiducial_detections` while holding a marker about 1.5 m in
   front of the camera, then `ros2 topic echo /car/fiducial_detections`: the `id` and a `position` whose x is about
   the real distance (otherwise fix `default_marker_size_m` and `dream runtime restart aruco_detector`).
3. **Commands are given.** With the Traxxas node disabled (neutral commands, the car stays still), put the policy in
   "publishing actions": markers are only read in policy steps. Hold each marker in front of the camera and watch
   the policy log (`dream runtime logs ai4r_policy --tail 50`: `Marker 0: KEEP_LEFT, lane side LEFT`) and the text
   beside the car in Foxglove. Put the policy back to zero actions afterwards.
4. **Lane change.** Lay a straight road, keep-left and keep-right markers 3 m apart beside it, 0.2 m outside the
   cones, and drive it. The car should move about 0.12 m (0.8 m road) or 0.22 m (1 m road) towards each side.
5. **Intersection.** Lay a crossing as wide as the road, and the turn marker beside the approach road 1 m before the
   crossing (`intersection.marker_to_entry_m`). Start with a left turn in a 1 m road, at walking pace with a hand on
   the RC stop. Watch `/plan/intersection` in Foxglove and the log (`TURN_LEFT: intersection entry ... ahead`,
   `turning`, `exit lane found`). If the arc runs wide or tight, measure the car's tightest turn each way and set
   `left_turn_radius_m` and `right_turn_radius_m` about 10% above it.

## Next

- Measure car 21's tightest left and right turns, and the lab intersection's size, and rerun the simulation with them.
- Join Yuhao's connected-corridor follower (`feature/cone-memory-continuity`), which follows a cone path through 90
  degree corners and U-turns, for exits that bend; it branched from the 19 October `dev` and overlaps the local map.
- Curved roads in the marker simulation, and markers on the left of the road.
