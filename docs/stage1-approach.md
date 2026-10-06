# Stage 1 approach: requirements, observations and interfaces

Card 1 in [movement-policy-tasks.md](movement-policy-tasks.md): what Stage 1 (follow a straight lane with even
cones) requires of the movement policy, how each requirement is measured, what the policy observes, how MPC and RL
are compared, what we exchange with Road Detection and Car Control, and the time budget on the car. The test cases
and pass thresholds that use these metrics are in [stage1-test-cases.md](stage1-test-cases.md) (Card 2).

## 1.1 Requirements and metrics

Each requirement has one metric. R1 and R6 each have two sub-requirements with one metric each, because they
cover two separate failures. The metrics are computed by `compute_metrics` in `tests/policy/stage1_scenarios.py`
and checked against `Thresholds` there; keep the two in step with this table.

| Id | Requirement | Metric | Pass if | Harness metric |
| --- | --- | --- | --- | --- |
| R1a | Stay in lane | Largest distance any body corner goes past a cone row while the car is beside the cones | 0.0 m | `body_outside_lane_m` |
| R1b | Follow the lane centre | Distance from the lane centre at the last cone pair, or where the car stopped before it | at most 0.10 m | `lateral_error_at_lane_end_m` |
| R2 | No cone touched | Cones within a cone base radius (0.05 m) of the car body | 0 | `cones_touched` |
| R3 | Hold speed | Mean absolute speed error from 1 m after the start to 1 m before the end, until the first stop request | at most 0.10 m/s | `mean_speed_error_m_per_s` |
| R4 | Smooth | RMS rate of change of the requested curvature | at most 1.0 1/(m s) | `rms_curvature_rate_per_m_s` |
| R5 | Stop at lane end | Car centre's stop position past the last cone pair | 0.0 to 0.5 m | `stop_past_last_cone_m` |
| R6a | Real time | 99th-percentile calculation time per policy step, on the car computer | at most 20 ms (1.5) | `p99_calc_time_s` |
| R6b | Always a usable command | Policy steps where the solver did not converge (fallbacks) | 0 | `fallback_count` |
| R7 | Safe stop when the lane is lost | Time from the first detection batch without the lane reaching the policy to its stop request | at most 0.6 s, and R1a and R2 still hold | `lane_lost_to_stop_request_s` (Task A7) |

**dream-gym's built-in metrics.** dream-gym 0.2.2 has none for cones, stopping or calculation time, so every
metric above is computed by the harness. Three built-in metrics are close and can be used as cross-checks: R1b against
`max_abs_lateral_error_m` (over the whole run rather than at the lane end), R3 against
`mean_abs_speed_error_to_recommended_kph` (against the road's recommended speed, in km/h) and R4 against
`mean_abs_steering_rate_radps` (wheel angle rate rather than requested curvature). Its `compute_pareto_front` is
used for the Pareto fronts in Tasks A4 and A5.

**Coverage.** Every Stage 1 requirement on the card has a row: stay in lane (R1a), no cone touched (R2), hold
speed (R3), smooth (R4), stop at lane end (R5), real time (R6a, R6b) and safe stop when the lane is lost (R7). R1b
is added because a car can stay inside a 1 m lane while holding a steady offset (T11 ends 0.15 m off centre), which
R1a cannot see.

**No two metrics measure the same thing.** R1a is the worst excursion at any time and R1b the final centring. R2
counts contacts, which R1a misses when the car stays between the rows but clips a cone with a corner. R5 is about the
planned lane end, R7 about an unexpected loss mid-lane. R7 is a time rather than a distance because the distance
the car then rolls depends on Car Control's braking (in simulation about 0.6 m from 1 m/s), which the policy
cannot change. It starts when the policy receives the first batch taken after the loss, so the detections' own age
(0.2 s on the car, 1.5) is left out, like the roll-out; 0.6 s is at most 0.5 s driving on the last plan in case the
lane comes back (Task A7) plus one 0.1 s detection gap. R6a is time and R6b convergence: a solver can be fast and
still fail, or converge and be too slow.

**Still to do.** R7 needs a case where the lane disappears mid-run (the camera covered) and its metric in
`compute_metrics`; both come with Task A7. R6a can only be measured on the car computer; in simulation it is
informational.

## 1.2 Observations

One table for both MPC and RL. Everything here exists on the real car; the simulator's ground truth (the car's
world pose and the true lane centre) is used only by the test harness to place the car, create disturbances and
score the run.

| Observation | Units and frame | Provided by | Why the policy needs it |
| --- | --- | --- | --- |
| Lateral error | m, positive when the car is left of the lane centre | Road Detection (Long). Until 1.4 is agreed, `reference_from_cones` fits it from the cones | The quantity to drive to zero (R1a, R1b) |
| Heading error | rad, positive when the car points left of the lane | Road Detection (Long), as above | Predicts where the lateral error is going, so the policy can correct before it grows (R1a, R4) |
| Lane edges | Left and right boundary lines in `base_link` | Road Detection (Long) | Margin to the cone rows (R2), and the lane centre when only one row is visible |
| Lane width | m | Road Detection (Long), or the configured `mpc.lane_width_m` | Finds the centre from a single visible row |
| Observation age | s since the camera acquired the image (`sensor_age`) | DREAM `policy_node` | Predicts the car forwards by the measured 0.2 s delay before planning (R1a, R4) |
| Wheel speed | m/s, unsigned | DREAM Traxxas interface (wheel encoder) | Speed control (R3) and the distance driven while the lane is lost (R7) |
| Time since the previous step and since the run started | s | DREAM `policy_node` | Lane-loss timer (R7) and run timer |
| IMU heading and yaw rate (optional) | rad, rad/s | DREAM BNO08x interface | Not used yet. Would let the policy hold its heading while the lane is lost (R7). Not running on car 20 on 2026-10-06 |

**Not ground truth.** Lateral error, heading error and lane edges come from cone detections, which carry the
detector's noise, 0.2 s delay, limited view and missed cones; the simulation adds the same effects (Card 2.4). Wheel
speed in simulation is currently the true speed without noise, which is a small optimism to remove in Task A2.

## 1.3 MPC versus RL decision rule

MPC and RL are run through the same harness (`run_case` in `tests/policy/stage1_scenarios.py`), on the same tuning
and unseen cases, with the same disturbances and the same metrics from 1.1; neither sees simulator ground truth. A
design is eligible only if it meets every 1.1 threshold on every unseen case. Among eligible designs, the winner is
on the Pareto front of the unseen-case metrics (worst case over the cases), with R1b, R3 and R4 as the trade-off
axes. If both are on the front, or neither is eligible, we prefer the one that is easier to debug on the car: the
MPC, because its plan, cost terms and constraints can be read directly, unless RL is better on a safety metric
(R1a, R2, R5 or R7). The decision and its metrics table are recorded at milestone M1.

## 1.4 Interfaces with Road Detection and Car Control

**Draft, not yet confirmed by Long and Chris.**

| Direction | Item | Definition |
| --- | --- | --- |
| From Long | Lane ahead | Centre line in `base_link` (+x forwards, +y left), as far ahead as the MPC predicts (1.2 to 1.5 m with the current settings) |
| From Long | Car position in the lane | Lateral error (m, + left of centre) and heading error (rad, + pointing left), at the time of the camera image |
| From Long | Lane edges | Left and right boundary lines, with a flag for each saying whether it was seen or inferred |
| From Long | Validity | Whether the lane is usable at this step; the policy treats a missing or doubtful lane as lost |
| From Long | Cone colours | Observed on the lab lane on 2026-10-06 (car 20): **yellow on the left, blue on the right**. To confirm as the rule |
| To Chris | Speed | m/s, never negative; 0.0 requests a stop |
| To Chris | Curvature | 1/m (1 / turning radius), **left turn positive** |
| To Chris | Rate | One command per cone detection batch (10 Hz, 1.5) |
| Both | No lane or a doubtful lane | Stop. A brief loss is bridged by the A7 rule (follow the last plan at reduced speed for at most 0.5 s); after that the car stops and stays stopped until the policy is restarted |

To confirm with Chris: the maximum curvature the car can drive (steering is limited to half the servo range in
`traxxas_vehicle_interface.yaml`, so it is likely well below the 45 degree default), the usable speed range and the
reaction delay. Until Car Control's conversion is ready, `CarControl` in `policy/control/control.py` is a stand-in.

## 1.5 Time budget on the car

Measured on car 20 on 2026-10-06 with the OAK-D cone detector running (`ros2 topic hz` on
`/car/cone_detections`, the same data Foxglove shows):

| Quantity | Value |
| --- | --- |
| Cone detection rate | 10.0 per second; gap between batches 0.09 to 0.11 s (standard deviation 6.5 ms) |
| Acquisition-to-publication latency reported by the detector | 0.197 s |
| Vehicle command timeout | 0.5 s, five detection gaps |
| **Allowed calculation time per policy update** | **20 ms (99th percentile)**, a fifth of the detection gap |
| MPC calculation time on the car computer, horizon 30 | median 0.74 ms, 99th percentile 1.57 ms (Task A6) |

The policy runs once per detection batch, so it must finish well inside the 0.1 s gap; 20 ms leaves room for the
cone filter, the node's own work and Python pauses. The detection age the policy sees is about 0.2 s, mostly the
detector's own latency, and the MPC predicts the car forwards by that age before planning.
