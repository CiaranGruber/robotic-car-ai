# MPC: keeping to the left or right side of the lane

The first step towards the hook turn: the car keeps to one side of the lane instead of its centre. Set
`mpc.lane_side` to `"LEFT"` or `"RIGHT"` (default `"CENTRE"`), or change `MpcPolicy.lane_side` between policy steps.
Simulation only: none of this has been driven on the car yet.

| Item | Where |
| --- | --- |
| Settings | `MpcConfig.lane_side`, `edge_clearance_m`, `offset_change_per_m` in `policy/action_policy/mpc.py`; `mpc` in `config/ai4r_policy.yaml` |
| Choosing and moving the line | `MpcPolicy.offset_limits_m`, `target_offset_m`, `usual_offset_m` in `policy/action_policy/policy.py` |
| Tests | `tests/policy/test_mpc_curves_and_offset.py`, `tests/policy/test_wide_tracks.py` |
| Simulation | `tests/policy/curved_lanes.py`; wide random tracks in `tests/policy/wide_tracks.py` |
| Scenario files | `scenarios/data/synthetic_scenarios/wide_w*_scenario.json` (table in "Wide synthetic tracks") |

## Whose work it is

Lane detection reports one lane: its two edges, measured width, centre line and where the car is
([lane-detection-interface.md](lane-detection-interface.md)). Where to drive inside that lane is the policy's
choice, as it already is for driving around obstacles (`policy/action_policy/avoidance.py`). So this is MPC work,
and lane detection is unchanged. If the course gets lanes divided by a middle row of cones, lane detection will
need to report more than one lane.

## How it works

The MPC follows a line beside the lane centre (`offset_state`; concentric with it on a curve). With
`lane_side` `"LEFT"` or `"RIGHT"`, that line is `edge_clearance_m` from the chosen edge, using the width lane
detection measures at each step, so the car moves out where the lane widens and back in where it narrows.

- **Edge clearance, default 0.40 m.** The car's side is then 0.275 m from the cone centres. An 0.8 m lane is still
  driven along its centre, a 1.2 m lane gives 0.2 m of offset and a 1.6 m lane 0.4 m.
- **Gradual moves, default 0.2 m sideways per metre driven** (`offset_change_per_m`), when the side changes or the
  lane widens. Where the lane narrows, the line moves in at once.
- **Tight curves.** On the inside of a curve the line stays at most halfway to the turning centre
  (`inside_offset_limit_m`), so its curvature is at most twice the lane's.
- **Obstacles.** Obstacle avoidance plans its ramps from this line (`usual_offset_m`), so the car leaves the side to
  pass an obstacle and returns to it afterwards.

This replaces `offset_fade_curvature_per_m`, which shrank the offset on curves (section "What changed").

## Results

`curved_lanes.py` with dev's lane detection (curves on, `cone_filter.check_straight_rows: false`): car 20's
steering limits, camera view, noise and 0.2 s delay, -3 to +3 degrees of unmeasured camera yaw, steering trim
compensated unless stated. Seven lanes (a straight, 90 degree curves of 1.5-3 m radius, an S bend), three yaws and
four noise seeds: 84 runs per row. "Gap" is the worst distance from the car's side to a cone centre line; a cone's
radius is 0.05 m.

| Side | Lane width [m] | Lanes kept | Median position, straights / curves [m] | Worst gap [m] | Curv. rate RMS |
| --- | --- | --- | --- | --- | --- |
| Centre | 0.8 / 1.2 / 1.6 | 84 / 84 / 84 | 0.00 | 0.167 / 0.324 / 0.517 | 0.22 / 0.24 / 0.30 |
| Left | 0.8 / 1.2 / 1.6 | 84 / 84 / 84 | +0.00 / +0.19 / +0.39 | 0.162 / 0.207 / 0.201 | 0.22 / 0.27 / 0.34 |
| Right | 0.8 / 1.2 / 1.6 | 84 / 84 / 84 | -0.00 / -0.19 / -0.39 | 0.170 / 0.204 / 0.140 | 0.22 / 0.30 / 0.41 |

With the -0.05 1/m trim Chris measured left uncompensated, keeping right in a 1.6 m lane lost 4 of 84 (the S bend)
and every side lost the 2 m left curve in an 0.8 m lane, which car 20 can only just steer (as on `dev`).

**Edge clearance** (two seeds, 42 runs per row; lanes kept at 0.8 / 1.2 / 1.6 m):

| Clearance [m] | Left | Right | Worst gap [m] |
| --- | --- | --- | --- |
| 0.30 | 42 / 42 / 42 | 42 / 42 / 37 | -0.042 |
| 0.35 | 42 / 42 / 42 | 42 / 42 / 40 | +0.053 |
| **0.40** | **42 / 42 / 42** | **42 / 42 / 42** | **+0.141** |

The misses keep right round left curves with +3 degrees of camera yaw, which runs the car about 0.1 m wide of its
line on the outside. Once the camera yaw is measured (cone 2 m straight ahead,
[mpc-lab-calibration.md](mpc-lab-calibration.md) section 2), a smaller clearance may be safe.

**Changing side** (left to right after about 3 m; a straight, a 3 m left curve and a 2 m right curve, 1.2 and
1.6 m lanes, three yaws, two seeds):

| `offset_change_per_m` | Lanes kept (36) | Distance to cross, 1.2 / 1.6 m lane [m] | Largest swing past the new line [m] |
| --- | --- | --- | --- |
| Instant | 21 | 1.2-1.4 / 1.4-2.0 | 1.71 |
| 0.5 | 26 | 1.6 / 2.4 | 0.36 |
| 0.3 | 36 | 1.9-2.0 / 3.3-3.5 | 0.16 |
| **0.2** | **36** | **2.4-2.6 / 4.5** | **0.09** |

## The lab tracks are narrow

On the recorded tracks lane detection measures 0.65-0.95 m (median about 0.8 m), so at the default clearance the
car stays in the centre; at 0.30 m it could move about 0.1 m over. Keeping to a side needs a wider road, about 1.2 m
for 0.2 m of offset, or a measured camera yaw and a smaller clearance. Replaying `fail_curvy_road`,
`obstacle_track_expands`, `fail_obstacle_track_thins` and `straight_even` through `MovementPolicy` with each side
ran without errors; the line moved at most 0.15 m off the centre where those tracks widen.

## Wide synthetic tracks

The team's synthetic tracks (`tests/policy/synthetic_tracks.py`) are lab-width, so they cannot show a lane side.
`tests/policy/wide_tracks.py` draws random tracks the same way, by seed and around the measured lab car, but 1.1-1.6 m
wide and with curves: straights, left curves of 2.5-4 m radius, right curves of 1.5-4 m (drawn twice as often, as
the hook turn is a right turn) and S bends. Each track asks to keep left, keep right, or switch sides 1.5-3 m in.
The car drives through lane detection with curves on, as `config/ai4r_policy.yaml` sets it, with the camera yaw
within ±3°, the steering trim within ±0.03 1/m (left after Car Control's trim) and staggered, swapped-colour and
outlier cones at the rates `synthetic_tracks.py` uses.

```
python3 tests/policy/wide_tracks.py                # training tracks W0-W59
python3 tests/policy/wide_tracks.py --validation   # validation tracks W2000-W2029
python3 tests/policy/wide_tracks.py --export 0 1 2 10 22 33 56   # save these as scenario files
python3 -m scripts.scenario_renderer show -t synthetic -n wide_w22
```

The saved scenarios are the clean tracks (no swapped or outlier cones), so each shows the lane side. The team's
scenario renderer plays them like the recordings: the cones as the car saw them, so the side shows as one row
nearer the car than the other. It does not draw the detected lane or the followed line.

| Scenario | Asked for | Lane |
| --- | --- | --- |
| `wide_w2` | keep left | 1.23 m, left curve of 2.8 m radius |
| `wide_w33` | keep right | 1.32 m, right curve of 3.4 m radius |
| `wide_w56` | keep right | 1.46 m, straight |
| `wide_w1` | keep right | 1.36 m, S bend |
| `wide_w10` | left, then right after 2.9 m | 1.58 m, left curve of 3.5 m radius |
| `wide_w22` | right, then left after 1.6 m | 1.28 m, straight |
| `wide_w0` | right, then left after 2.9 m | 1.42 m, left curve of 3.4 m radius |

"Clean" is the same track without the swapped-colour and outlier cones (staggered cones stay). "Stopped" means the
car stopped short without touching anything, as it does when lane detection reports no lane; "hit" means it
touched a cone or left the lane.

| Tracks | Clean, lane side | As drawn, lane side | As drawn, centred (as dev) |
| --- | --- | --- | --- |
| Training (60) | 59 kept, 1 hit | 23 kept, 32 stopped, 5 hit | 27 kept, 32 stopped, 1 hit |
| Validation (30) | 30 kept | 14 kept, 12 stopped, 4 hit | 13 kept, 16 stopped, 1 hit |

On the clean tracks the car drives a median 0.02 m from the line its side asks for (worst validation track 0.06 m).
The training hit, W28, is a right-to-left switch on an S bend: with the car near the right edge of a 1.53 m lane,
lane detection saw one edge only and, with `left_colour` `"AUTO"`, called it the wrong side. With the true left
colour set, all 60 clean training tracks were kept.

**Swapped-colour and outlier cones are a lane detection problem that lane side makes worse.** In both modes lane
detection often loses the lane, and the car stops safely. But one swapped cone about 2 m ahead can also give a
false lane, about 0.4 m wide and curving at about 1 1/m (W26, W2022). The MPC steers hard on that lane and the car
rolls on while it slows. Centred, the car's side is 0.47 m from the cones of a 1.2 m lane; kept left, only
0.27 m, so the same false lane makes it hit a cone more often (9 runs against 2 over both sets). With the true
left colour set, as many training runs stopped short, and both modes hit in 2 of 60.
`test_one_swapped_cone_does_not_send_the_car_into_the_cones` records W26 as an expected failure until lane
detection rejects such lanes. Until then, keep `lane_side`
`"CENTRE"` except for lane side tests, on tracks laid out with clean colours. The swapped-colour rate (2% of cones)
is the synthetic tracks' assumption, not a measurement.

## Setting the side on the car

- **Now, with no code change.** Set `mpc.lane_side` in `config/ai4r_policy.yaml`, upload it with
  `scripts/car_management.sh upload_and_build` (it restarts `ai4r_policy`), then request policy state 3 as usual.
  Every policy setting is declared read-only and read once at startup (`scripts/policy_node.py`), so
  `ros2 param set` and Foxglove's Parameters panel cannot change it mid-run. Each drive's data log saves the value
  in `<stem>_config.json`.
- **Changing it during a test run (not built).** Make `mpc.lane_side` the one writable parameter: an
  on-set-parameters callback checks the value and sets `MpcPolicy.lane_side`, which already moves the line
  gradually. Then Foxglove's Parameters panel (`foxglove_bridge` already runs on the car) or
  `ros2 param set <namespace>/ai4r_policy mpc.lane_side LEFT` switches sides while driving. A `std_msgs/String`
  topic set from Foxglove's Publish panel, like the policy state button, would also work, but a parameter is checked,
  typed and visible. Either one changes the node's public interface, so it needs a human review, and the data log
  would need to record the side at each step, as it records settings only at startup.
- **For the hook turn.** The manoeuvre sets `lane_side` itself from what the car sees, such as the turn's marker,
  not a person during a scored run.

## What changed

- New `mpc.lane_side` (`"CENTRE"`, `"LEFT"`, `"RIGHT"`) and `mpc.offset_change_per_m` (0.2).
- `mpc.edge_clearance_m` 0.30 to 0.40.
- Removed `mpc.offset_fade_curvature_per_m`. It shrank the offset in proportion to the detected curvature, which
  lane detection reports with noise even on straights: asked to keep left in a 1.6 m lane, the car moved only
  0.05 m over and steered almost three times as much (curvature-rate RMS 0.86 against 0.31). An inside limit from
  car 20's steering limits did the same on curves, flickering on and off and moving the line by up to half a metre
  between steps, so the inside limit is only the geometric one. A line the car cannot steer along is safe anyway:
  it runs wide, towards the centre.
- `mpc.lane_offset_m` now applies while `lane_side` is `"CENTRE"`.

## Next

- Lane detection: reject a lane whose width or curvature jumps far from what it has just measured, and recheck the
  wide tracks as drawn (`python3 tests/policy/wide_tracks.py`).
- Measure the camera yaw with the pan held, then check whether a smaller clearance is safe.
- Try keeping left and right on car 20 in a lane about 1.2 m wide, with the steering trim compensated.
- The hook turn itself: when to move over, then the right turn at up to car 20's 0.92 1/m (about a 1.1 m radius).
