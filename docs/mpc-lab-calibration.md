# MPC: lab car calibration, curved lanes and a lane offset

What Ciaran's recorded drives and Chris's car 20 calibration say about the real car, why the car drifts towards the
cones on curves, what changes in the MPC, and a lane offset towards the hook turn (Tasks A1, A2 and A4 in
[movement-policy-tasks.md](movement-policy-tasks.md)). Simulation only: none of this has been driven on the car yet.

| Item | Where |
| --- | --- |
| Measurements from the recordings, the simulated lab car and lane | `tests/policy/real_car_measurements.py` |
| Lab car in the Stage 1 simulation (`CarProfile`) | `tests/policy/stage1_scenarios.py` |
| Curved lanes: simulation and comparison | `tests/policy/curved_lanes.py` |
| Straight synthetic tracks and scenario export | `tests/policy/synthetic_tracks.py`, `scenarios/data/synthetic_scenarios/` |
| Weight sweep on the straight tracks | `tests/policy/mpc_tuning.py` |
| Tests | `tests/policy/test_mpc_lab_calibration.py`, `tests/policy/test_mpc_curves_and_offset.py` |

## What the team has on other branches (11 October)

This builds on `dev`. Other branches change the same files:

- **`test/lane-following-car20`** (Thanh Long; includes `feature/lane-edges` and Chris's `control/low-level-sysid`):
  the branch car 20 was driven with. The MPC follows `LaneDetection` instead of its own cone fit, Chris's speed
  controller and steering map turn its commands into actions, the camera pan is held at zero, and the settings are
  0.5 m/s, a 0.5 1/m curvature limit both ways, heading weight 5, and straight lane edges
  (`lane_detection.max_curvature_per_m: 0.0`, `cone_filter.check_straight_rows: true`).
- **PR #14** (Yuhao, with Peiyu's open-space policy): the MPC predicts with the measured wheel speed, keeps driving
  briefly when the lane is lost, and stops after a failed solve.

Both change `policy/action_policy/policy.py` and `config/ai4r_policy.yaml`, so merging this needs small conflict
fixes. The changes here are mostly inside `MpcController` and `MpcConfig`, which those branches use unchanged.

## 1. What the recordings measure

Ciaran's four scenarios (`scenarios/data/real_scenarios`: straight_even, long_track, messy_cone, outlier_cone)
drive at a fixed drive effort of 0.55 with the steering centred. Print the table with
`python3 tests/policy/real_car_measurements.py`.

| Quantity | Shared default | Measured | Lab car / lane |
| --- | --- | --- | --- |
| Left row colour | not agreed | blue in 100% of batches (lane detection now finds the side itself) | blue |
| Lane width [m] | 1.0 | 0.77 | 0.8 |
| Cone spacing [m] | 0.5 | 0.20 | 0.2 |
| Camera view [deg] | 80 | +-43 (99%) | 86 |
| Nearest detection [m] | 0 | 0.41 | 0.4 |
| Detection range [m] | 4 | 2.54 (95%) | 2.5 |
| Position noise [m] | 0.01 | 0.040 (row residual) | 0.03 |
| Detection latency [s] | 0 | 0.21 | 0.2 |
| Camera yaw, left [deg] | 0 | 7.3 (long track), pan never commanded | 7 (recordings only) |
| Curvature, steering centred [1/m] | 0 | -0.05 (car 20, Chris) | -0.05 |
| Largest curvature left / right [1/m] | 3.0 / 3.0 | 0.48 / 0.92 (car 20, Chris) | 0.48 / 0.92 |
| Speed time constant [s] | about 0.6 | 1.2 to 1.8; 2.7 (Chris) | 1.5 |

The steering was centred in every recording, so the curvature limits and trim are Chris's car 20 values
(`docs/control/car-calibration.md` on `control/low-level-sysid`). We do not know which car Ciaran recorded with.

## 2. The camera yaw

In every recording the cone rows look turned 6-10 degrees to the right, yet the car's distance from the lane
centre hardly changes: a car really pointing 8 degrees off would drift 0.35 m over the long track's 2.5 m, and it
drifted about 0.03 m. So the detections are rotated, not the car. Thanh Long found the same in lane detection
(`docs/lane-detection-interface.md`) and traced it to the camera pan: the recorded actions never command the pan
(`camera_pan_action` is empty in all four recordings), so the servo stays wherever it was. The car 20 branch now
holds the pan at zero.

So `mpc.camera_yaw_offset_deg` stays available but defaults to 0.0. **Check on the car with the pan held:** put a
cone 2 m straight ahead on the car's centre line; each degree of yaw moves its detected position 3.5 cm sideways
(7 degrees left gives y = -0.25 m). Set the measured value only if it is not near zero. The simulations below give
the camera -3 to +3 degrees of leftover yaw that the MPC does not know about.

## 3. What changes in the MPC

| Setting | Before (`dev`) | Now | Why |
| --- | --- | --- | --- |
| `max_curvature_per_m` | 3.0 both ways | 0.48 (left) | Car 20's left limit, a 2.1 m radius |
| `max_right_curvature_per_m` (new) | - | 0.92 | Car 20's right limit; needed for the 1.5 m right curve (section 4) |
| `curvature_change_weight` | 1.0 | 2.0 | A quarter to a third less steering activity, same lane keeping (sections 4, 6) |
| `heading_error_weight` | 1.0 | 1.0 (unchanged) | 3.0 and 5.0 lost curves and straights (section 4) |
| `target_speed_m_per_s` | 1.0 | 0.5 | Chris's speed controller was checked at 0.5 m/s |
| `lane_width_m` | 1.0 | 0.8 | The lab tracks are 0.73-0.85 m wide |
| `lane_offset_m`, `edge_clearance_m`, `offset_fade_curvature_per_m` (new) | - | 0.0, 0.30, 0.3 | Drive to one side of the lane (section 5; superseded by [mpc-lane-side.md](mpc-lane-side.md)) |
| `camera_yaw_offset_deg` (new) | - | 0.0 | Section 2 |

## 4. Curved lanes: why the car drifts into the cones

`curved_lanes.py` builds lanes from straights and circular arcs, places cones along both edges, and drives the
policy with the lab car: the camera's view, range, noise and 0.2 s delay, -3 to +3 degrees of camera yaw, car 20's
steering limits, and a lagged speed and steering response. The car is kinematic (no tyre slip), which suits
0.5 m/s. A run keeps the lane if no cone is touched, the body stays inside the edges, and the car reaches within
0.75 m of the last cone pair (lane detection reports no lane once only the last cones, nearer than the camera's
0.4 m, are left).

The lanes: a straight, 90 degree curves of 1.5 (right only; car 20 cannot steer it left), 2 and 3 m radius, and an
S bend of two 2.5 m arcs. Each runs with three camera yaws and four noise seeds: 84 runs per row. Every row uses
0.5 m/s and a 0.8 m lane width; only the settings named change.

Chris's steering map has a measured centre (`steering_centre_action`), so a car whose trim is compensated is the
expected case. The -0.05 1/m trim Chris measured is also run, for a car whose trim is reset or not calibrated.

**Lanes kept (of 84), with curvature-rate RMS in brackets:**

| MPC settings | `dev` cone fit | Lane detection, straight edges (car 20 run) | Lane detection, curves on |
| --- | --- | --- | --- |
| `dev` defaults (3.0 both ways, heading 1, change 1) | 49 | 84 (0.33) | 84 (0.27) |
| Car 20 run (0.5 both ways, heading 5, change 1) | 12 | 12 (0.43) | 73 (0.71) |
| **This branch (0.48 left / 0.92 right, heading 1, change 2)** | 47 | **84 (0.22)** | **84 (0.20)** |
| Same, steering trim -0.05 uncompensated | 45 | 73 (0.25) | 72 (0.24) |
| `dev` defaults, trim -0.05 | 49 | 74 (0.36) | 72 (0.33) |
| Car 20 run, trim -0.05 | 13 | 12 (0.44) | 70 (0.71) |

"Lane detection, curves on" is `lane_detection.max_curvature_per_m: 2.0` with `cone_filter.check_straight_rows:
false`. The lane detection columns were run against `test/lane-following-car20` (commit 3319051) with this
branch's `mpc.py`, because lane detection is not on `dev` yet; `python3 tests/policy/curved_lanes.py` runs them
once it is.

**What this shows:**

- **The car 20 run's drift comes mostly from its MPC settings.** With heading weight 5 and the 0.5 1/m limit both
  ways, the car kept 12 of 84 runs with straight lane edges. With this branch's settings and the same straight
  edges it kept all 84. With heading 5 the car lines up with the lane's direction at the cost of its distance from
  the centre, so camera yaw and noise push it sideways, and the 0.5 right limit cannot plan the 1.5 m right curve.
- **Lane detection's curves help a poorly set MPC** (12 to 73 for the car 20 run's settings) and steer slightly
  more smoothly with good settings (0.22 to 0.20). Turning them on is still recommended, but it is not the main fix.
- **`dev`'s own cone path stays unreliable on curves** (45-49 of 84 with any settings). Its straight-row filter
  removes curve cones and its straight fit points the car along a chord. On the car the MPC follows lane detection
  instead, once that merges.
- **Against `dev`'s defaults**, which keep the curves here because the simulated car clips the impossible 3.0
  curvature, the gain is smoother steering (0.27 to 0.20) and more straight lanes kept (section 6). The MPC's plan
  now matches what car 20 can steer.
- **With the trim uncompensated**, every setting with heading weight 1-1.5 loses the 2 m left curve (all 12 of
  this branch's misses). That curve needs 0.50 1/m and car 20 reaches 0.48 left, less with the trim pulling right.
  Heading 2 kept 11 of 12 there but lost straight-edge runs (below). Calibrate the trim rather than tune for it.

**Weights** (0.48 / 0.92 limits; lanes kept of 84 with lane detection's curves on; trim compensated / uncompensated):

| Heading, change weight | Curves on | Straight edges | Curv. rate RMS | Straight synthetic tracks (150) | Card 2 T11 end error [m] |
| --- | --- | --- | --- | --- | --- |
| 1, 1 | 84 / 72 | 84 / 74 | 0.27 | 92% | 0.153 |
| 1.5, 1 | 84 / 76 | 84 / 80 | 0.29 | - | - |
| 2, 1 | 84 / 83 | 68 / 72 | 0.35 | - | - |
| **1, 2** | **84 / 72** | **84 / 73** | **0.20** | **97%** | 0.181 |
| 1, 3 | 84 / 72 | 84 / 72 | 0.17 | 94% | 0.241 |
| 3, 3 (this branch's first draft) | 81 / 80 | 20 / 28 | 0.31 | 80% | 0.222 |

Heading 1 with change 2 keeps every curve and straight-edge run with the trim compensated, steers about a quarter
less than change 1, and keeps the most straight synthetic tracks. Change 3 is smoother still but ends further off
centre on straights and on T11. An earlier draft of this branch shipped heading 3 and change 3. That came from a
smaller comparison using the 0.5 limit both ways, where the missing right limit, not the heading weight, was losing
the curves. The larger comparison here reversed it.

**Right curvature limit** (heading 1, change 1, lane detection's curves on):

| Steering trim | Right limit | Lanes kept (84) | 1.5 m right curve (12) | `dev` cone fit (84) |
| --- | --- | --- | --- | --- |
| 0 | 0.48 (same as left) | 72 | 0 | 21 |
| 0 | **0.92** | **84** | **12** | **51** |
| -0.05 | 0.48 | 60 | 0 | 27 |
| -0.05 | **0.92** | **72** | **12** | **48** |

## 5. Lane offset, towards the hook turn

**Superseded by [mpc-lane-side.md](mpc-lane-side.md)**, which adds `mpc.lane_side`, raises `edge_clearance_m` to
0.40 and removes the curve fade below: through `dev`'s lane detection, the fade shrank the offset to almost
nothing. This section is kept as the record of the first version.

A Melbourne hook turn is a right turn made from the left of the road, so the car must keep to one side and then
turn sharply right. `mpc.lane_offset_m` (positive left) makes the MPC follow a line beside the centre; a manoeuvre
can change `MpcPolicy.lane_offset_m` between steps. On a curve the followed line is concentric with the centre
line, so its curvature is adjusted (`offset_state`). The offset is limited to leave `edge_clearance_m` (0.30 m:
half the car's 0.25 m width, a cone's 0.05 m radius and 0.125 m for tracking error) between the line and each
edge, which is 0.1 m of offset in a 0.8 m lane. It fades out on curves tighter than `offset_fade_curvature_per_m`
(0.3 1/m, a 3.3 m radius), where the car's own tracking error uses up the room.

This branch's settings, lane detection with curves on, 84 runs each; for the run, the offset was passed through
the car 20 branch's lane detection path in a scratch copy:

| Steering trim | Offset asked | Lanes kept (84) | Median position on the straight [m] |
| --- | --- | --- | --- |
| 0 | 0 | 84 | 0.000 |
| 0 | +0.4 (limited to +0.1) | 84 | +0.082 |
| 0 | -0.4 (limited to -0.1) | 84 | -0.080 |
| -0.05 | 0 | 72 | -0.018 |
| -0.05 | +0.4 | 72 | +0.065 |
| -0.05 | -0.4 | 72 | -0.098 |

The offset costs no lanes; with the trim uncompensated the only misses are the 2 m left curve again (section 4),
and the trim moves the car about 2 cm right of where it is asked to be. The offset is only wired into the MPC's own
cone path on `dev`. With lane detection, `run_policy` should pass `lanes.lane_width_m` to `followed_offset_m`, as
the cone path passes `lane_width_m`, and pass the result to `solve`.

## 6. Straight synthetic tracks

`synthetic_tracks.py` draws straight lab-like tracks (0.70-0.95 m wide, staggered, missed, swapped and outlier
cones, -3 to +3 degrees of camera yaw, the lab car's speed response). They run through `dev`'s cone path and check
that the weights chosen on curves still work on straights. Seeds 0-99 and 1000-1049, none of them the unseen
cases. Lane-keeping pass: no cone touched, inside the lane, at most 0.10 m from the centre at the lane end,
curvature-rate RMS at most 1.0, no solver fallback, and the car reaches within 1 m of the end.

| Design | Pass (150) | Lat. err. at end [m] | Curv. rate RMS |
| --- | --- | --- | --- |
| `dev` defaults (3.0 limit, heading 1, change 1) | 91% | 0.043 | 0.36 |
| Car 20 limits, heading 1, change 1 | 92% | 0.043 | 0.35 |
| **This branch: car 20 limits, heading 1, change 2** | **97%** | 0.044 | **0.24** |
| Car 20 limits, heading 1, change 3 | 94% | 0.046 | 0.23 |
| Car 20 limits, heading 3, change 3 (first draft) | 80% | 0.061 | 0.26 |

All 150 tracks were seen when choosing between these five, so 97% is not a held-out estimate. The weights were
chosen on the curves (section 4) and these tracks only confirm them. `python3 tests/policy/mpc_tuning.py` runs
the wider sweep on training seeds and checks its choice on the validation seeds.

The earlier version of this document reported 8% rising to 94% from correcting a 7 degree camera yaw. That
assumed the yaw stays; with the pan held, `dev`'s settings already pass 91% here, so that gain belonged to the
camera, not to the MPC.

The first 8 training tracks, driven by this branch's MPC, are saved as scenario files beside Ciaran's recordings
(`Scenario.from_name(ScenarioType.SYNTHETIC, "synthetic_s0")`; make more with
`python3 tests/policy/synthetic_tracks.py --export N`). They add variety, not evidence about the real car.

## 7. Card 2 cases

T1-T10 still keep the lane, with smoother steering than `dev`'s settings (curvature-rate RMS 0.08-0.33 against
0.12-0.62); the stop zone is still the Task A7 gap for both. T11 (wet floor) stays in the lane without touching a
cone but ends 0.181 m off centre (0.153 with `dev`'s settings; the smoother steering corrects more slowly), so it
stays `xfail` until the tyre model is checked (Task A2). See [stage1-test-cases.md](stage1-test-cases.md).

## Settings for the next car run

On the car 20 branch's `config/ai4r_policy.yaml`:

```yaml
cone_filter:
  check_straight_rows: false
lane_detection:
  max_curvature_per_m: 2.0
mpc:
  max_curvature_per_m: 0.48
  max_right_curvature_per_m: 0.92   # needs this branch's MpcConfig
  heading_error_weight: 1.0         # car 20 run used 5.0
  curvature_change_weight: 2.0
```

Before that, with the pan held at zero: the cone-straight-ahead check (section 2), and a straight run with the
steering centred to confirm Car Control's steering centre removes the trim.

## Open items

- Camera yaw with the pan held, per car (Task A1), and which car each recording used.
- Record curved lanes with data logging on, to check section 4 on real detections; no curved recording exists yet.
- Wire the lane offset into the `LaneDetection` path when lane detection merges, then plan the hook turn itself:
  when to move over, and the right turn at up to car 20's 0.92 1/m (about a 1.1 m radius).
- Agree Chris's speed controller and steering map (Task 1.4), then use them instead of `StandInControl`.
- Lane-end behaviour (Task A7) and the dynamic tyre model (Task A2), as before.
