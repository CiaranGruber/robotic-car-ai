# Lane detection: output interface and lane edges from cones

Road Detection's output to the Movement Policy (Task 1.4 in
[movement-policy-tasks.md](movement-policy-tasks.md)). The code is
`policy/lane_detection/`: `lanes.py` holds the types and `lane_detection.py`
finds them. `MovementPolicy.step` runs it after the cone filter and passes the
result to `MpcPolicy.run_policy`.

## Pipeline

```
cone detector -> filter_cones -> detect_lanes -> action policy (MPC/RL) -> control
                 (outliers)      LaneDetection     speed + curvature
```

`MovementPolicy` calls `LaneDetector.detect(observations)`, which runs
`detect_lanes(observations, config, lane_width_m)` and remembers the lane
width measured while both edges are seen. Either returns:

- `None` when there is no cone data (`observations.cones is None`), or
- a `LaneDetection`, whose `status` says what was found.

`detect_lanes` keeps no state; `LaneDetector` keeps only that width, reset on
the first step of each run. On the recorded scenarios, on a laptop, it
takes 0.04 ms (median) for straight edges and 0.65 ms (median, 6.5 ms at most)
for curved edges, inside the 50 ms budget.

## Conventions

| Item | Convention |
| --- | --- |
| Frame | `base_link` when the cones were detected: +x forwards, +y left, origin on the ground below the CG |
| Units | Metres, radians, 1/m |
| Signs | Left of the lane centre, pointing left and turning left are positive, as `PathTrackingState` in `policy/action_policy/mpc.py` |
| Which colour is where | Found automatically (`lane_detection.left_colour: "AUTO"`): the edge further left is the left edge, and a single edge is the left edge when it is on the car's left. The lab floor has had blue on the left on some days and yellow on others. `"BLUE"` or `"YELLOW"` fixes it instead |
| Edge shape | Both edges and the centre line are circular arcs around one centre point (concentric), or parallel straight lines when the curvature is 0. They stay one lane width apart however sharply the lane turns |

The Stage 1 simulation cases ([stage1-test-cases.md](stage1-test-cases.md))
put yellow on the left, which `"AUTO"` handles too. With `"AUTO"`, the car
follows any lane it sees, including one it has wandered into.

## `LaneDetection`

| Field | Type | Meaning | When `NO_LANE` |
| --- | --- | --- | --- |
| `status` | `LaneStatus` | Which edges were found (below) | `NO_LANE` |
| `left_edge` | `LaneEdge \| None` | Left boundary | None |
| `right_edge` | `LaneEdge \| None` | Right boundary | None |
| `centre_line` | `LaneArc \| None` | Middle of the lane, starting at its point nearest the car | None |
| `lane_width_m` | `float \| None` | Measured with both edges, else the width `LaneDetector` remembers (the configured width until both edges are seen) | None |
| `visible_until_m` | `float \| None` | Distance along the centre line to beside the farthest cone used. The lane is unknown beyond it; it falls near the lane end | None |
| `lateral_error_m` | `float \| None` | Car's distance from the centre line; + when the car is left of centre | None |
| `heading_error_rad` | `float \| None` | Car heading minus lane heading beside the car; + when the car points left of the lane | None |
| `path_curvature_per_m` | `float \| None` | Centre line curvature, + turning left; 0.0 with straight edges | None |
| `sensor_age` | `SensorAge` | Age of the cone batch; the car has moved since | Always set |
| `has_lane` | property | `status != NO_LANE` | False |

`lateral_error_m`, `heading_error_rad` and `path_curvature_per_m` are exactly
the three fields of `PathTrackingState`, and the MPC also assumes constant
curvature over its horizon. So the MPC can replace `reference_from_cones`
with:

```python
if lanes is None or not lanes.has_lane:
    ...  # stop, as agreed for no lane or a doubtful lane
state = PathTrackingState(lanes.lateral_error_m, lanes.heading_error_rad, lanes.path_curvature_per_m)
```

### `LaneStatus`

| Status | Found | Centre line from |
| --- | --- | --- |
| `BOTH_EDGES` | Both edges, with a plausible width | Halfway between the edges |
| `LEFT_EDGE_ONLY` | Left edge only | Half of `lane_width_m` right of the left edge |
| `RIGHT_EDGE_ONLY` | Right edge only | Half of `lane_width_m` left of the right edge |
| `NO_LANE` | Nothing usable: too few cones, rows too short to give a direction, cones that still fit badly after dropping outliers, or a doubtful lane (width outside `[min_lane_width_m, max_lane_width_m]`, edges crossed when `left_colour` is fixed, or an edge past the turning centre) | — |

With one edge, the centre line is only as good as the remembered width.

### `LaneEdge` and `LaneArc`

| Type | Field | Meaning |
| --- | --- | --- |
| `LaneEdge` | `arc` | The fitted `LaneArc`, starting beside the car |
| | `colour` | Colour of the cones it was fitted to |
| | `cone_count` | Cones used, at least `min_cones_per_edge`; outliers are left out |
| | `nearest_m`, `farthest_m` | Distance along the edge to its nearest and farthest cone used |
| | `rms_residual_m` | RMS distance of those cones from the edge, at right angles: noise, misplaced cones, or a lane whose curvature changes within view |
| `LaneArc` | `x`, `y` | Start point (m) |
| | `heading_rad` | Direction at the start, + left of the car's +x |
| | `curvature_per_m` | 1 / radius, + turning left; constant along the arc |
| | `point_at(s)`, `heading_at(s)` | Position and direction at distance s along the arc |
| | `signed_distance(x, y)` | Distance from a point to the arc, + on its left |
| | `project(x, y)` | Distance s along the arc to the point nearest a given point |
| | `offset(d)` | The concentric arc d metres to the left (curvature changes to keep the same centre) |

`LaneArc` has no `y = f(x)` form, so a turn of 90° or more within view is
represented correctly. All types are `Serialisable`, so a `LaneDetection` can
be logged and reloaded.

## How the edges are found

1. Each colour's cones are one edge. An edge needs at least
   `min_cones_per_edge` cones, and at least one edge needs two cones 0.25 m
   apart to give a direction. After the fit, the edge with the larger offset
   to the left is the left edge (or `left_colour` fixes it).
2. Both edges are fitted together as concentric arcs: a reference arc starts
   at the car with the lane's direction and curvature, and each edge is at its
   own sideways offset from it. The unknowns are that direction, the
   curvature and the offsets. The fit minimises the cones' distances from
   their edge at right angles, with a robust loss so a few misplaced cones
   count less. A long row steadies a short one, and an edge with too few cones
   for a direction still gives its offset.
3. With `max_curvature_per_m` 0.0 (the default) the curvature is fixed at 0
   and the fit is a linear least-squares fit of parallel lines. Otherwise it
   starts from the better of the best straight lines and the best concentric
   circles, and a small penalty on curvature (`typical_curvature_per_m`)
   keeps detector noise from looking like a curve.
4. Cones farther than `outlier_distance_m` from their edge are dropped and the
   edges fitted again, at most twice.
5. With both edges, the width is checked against
   `[min_lane_width_m, max_lane_width_m]`. Outside it, or with crossed edges,
   the lane is doubtful: `NO_LANE`.

On drawn lanes without noise, curved edges recover the curvature within 2%,
the car's offset within 1 cm and its heading within 0.6°, from a 1 m radius
hairpin (143° of turn in view) to an 8 m radius, both ways. With 3 cm detector
noise a straight lane's median curvature is 0.04 1/m (a 25 m radius).

## Settings

`lane_detection` in `config/ai4r_policy.yaml`:

| Setting | Default | Meaning |
| --- | --- | --- |
| `left_colour` | `"AUTO"` | `"AUTO"` finds each colour's side; `"BLUE"` or `"YELLOW"` fixes the left edge's colour |
| `lane_width_m` | 1.0 | Places the centre line with one edge until both edges have been seen (shared default) |
| `min_cones_per_edge` | 2 | Fewest cones for an edge to count |
| `min_lane_width_m`, `max_lane_width_m` | 0.3, 3.0 | Plausible measured width; the car is about 0.2 m wide |
| `max_curvature_per_m` | 0.0 | 0.0 straight edges (Stage 1); 2.0 for curved roads (0.5 m radius) |
| `typical_curvature_per_m` | 0.5 | Curvature treated as ordinary (2 m radius); smaller pulls harder towards straight |
| `outlier_distance_m` | 0.15 | Cones farther than this from their edge are left out |

**Curved roads need two settings:** `lane_detection.max_curvature_per_m: 2.0`
and `cone_filter.check_straight_rows: false`. The cone filter's row check
assumes straight rows and otherwise removes the cones on a curve.

## Results on the recorded scenarios

Straight edges (the default), after the shipped cone filter. Heading and
lateral error are medians while the car was standing still at the start.

| Scenario | Steps | Both edges | Width (m) | Heading at rest | Lateral at rest (m) | Farthest cone (m) | Edge RMS residual |
| --- | --- | --- | --- | --- | --- | --- | --- |
| straight_even | 52 | 35 | 0.82 ± 0.02 | +9.3° | +0.00 | 1.9 | 2.8 cm |
| messy_cone | 53 | 37 | 0.85 ± 0.02 | +10.2° | -0.04 | 1.9 | 4.6 cm |
| outlier_cone | 102 | 64 | 0.82 ± 0.03 | +8.2° | +0.04 | 2.0 | 5.4 cm |
| long_track | 80 | 67 | 0.74 ± 0.03 | +6.4° | +0.05 | 3.6 | 3.6 cm |

Each run drives to the end of the lane and on. The lane is found with both
edges until only a few cones within about a metre remain; there the
detections are rough and `NO_LANE` is reported, as at a lane end.

**Why straight edges are the default.** With curvature allowed, the same
straight lanes read as curving gently left (median 0.08-0.16 1/m, a 6-12 m
radius) in nearly every step of three runs, and the heading error at the car
roughly doubled (for example straight_even +9.6° ± 2.4° became +20.4° ± 3.8°).
The detected positions bend straight rows slightly, and the curve is extended
back to the car from cones 0.5 m and more ahead, which magnifies the bend.

## Seeing it on the car

`scripts/lane_overlay_node.py` runs the cone filter and lane detection on each
cone batch and publishes the edges, centre line and kept/removed cones as
`visualization_msgs/MarkerArray` for Foxglove's 3D panel, plus the lateral
error, heading error and width for the Plot panel. It never publishes actions.
See its docstring to run it; pass `-p` settings for curved lanes.

## Open issues

- **Heading bias of about 6-10°.** In every run the car drove 2-3 m with zero
  steering while the measured lateral error stayed nearly constant, yet the
  measured heading error was +6° to +10°. A real 6° heading would move the car
  about 0.2 m sideways over 2 m, so the cone positions are rotated rather than
  the car being turned. The bias differs between runs, which points at the
  camera pan servo not being centred (the policy sends no pan command, so it
  stays wherever it was) or a loose mount. Check on the car: centre the car in
  a straight lane, measure it with a tape, and compare `heading_error_rad`.
- **Straight rows look curved** with curvature allowed (above); likely the
  same calibration problem, or depth error growing with distance. Recheck
  after the camera is fixed.
- **Width.** The measured widths are 0.74-0.85 m; compare with the
  tape-measured width of each recorded lane.
- **Range.** Cones were mostly seen to 2 m, not the 4 m shared default.
- **Constant curvature.** Each step fits one curvature over the visible cones,
  so where a straight joins a curve the result is in between, and the edges'
  RMS residual grows. That matches the MPC's constant-curvature horizon.
- **No curved recordings yet.** Record curved lanes with data logging on to
  tune `typical_curvature_per_m` and `outlier_distance_m` on real data.
