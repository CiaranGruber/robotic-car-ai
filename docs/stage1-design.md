# Stage 1 design: simulation, baseline, MPC and RL

The algorithm tasks of Card 3 in [movement-policy-tasks.md](movement-policy-tasks.md). The requirements and
metrics (R1a-R7) are in [stage1-approach.md](stage1-approach.md), and the test cases, thresholds and harness in
[stage1-test-cases.md](stage1-test-cases.md). Everything here was run with the harness in
`tests/policy/stage1_scenarios.py` on the **tuning cases only**; the unseen cases are kept for milestone M1.

## A6 Calculation time on the car computer

`MpcController.solve` (the MPC in `policy/action_policy/mpc.py`) on car 20's computer (aarch64, Python 3.12.3,
NumPy 1.26.4, SciPy 1.11.4) on 2026-10-06: 300 solves per horizon from random states within the lane (lateral
error within 0.4 m, heading error within 0.5 rad), 0.5 m/s, 0.1 s steps, 0.2 s detection age.

| Horizon (steps) | Distance predicted at 0.5 m/s | Median | 99th percentile | Maximum |
| --- | --- | --- | --- | --- |
| 12 | 0.6 m | 0.31 ms | 0.55 ms | 1.61 ms |
| 20 | 1.0 m | 0.46 ms | 0.53 ms | 0.86 ms |
| 30 | 1.5 m | 0.74 ms | 1.57 ms | 2.32 ms |
| 40 | 2.0 m | 1.17 ms | 2.47 ms | 3.65 ms |
| 60 | 3.0 m | 2.94 ms | 5.96 ms | 8.89 ms |

Every horizon measured fits the 20 ms budget (Task 1.5) with a wide margin, so the horizon is limited by the camera,
not the computer: the predicted distance (speed x step x horizon) should stay within the range the camera reliably
sees cones at, 4 m by the shared default and still to be measured in Task A1. At 1 m/s with 0.1 s steps that is at
most 40 steps.

## A7 Lane end and lost cones

**When the car sees no cones or only one side:**

- Lane end: the last cone pair leaves the camera's view about 0.6 m before the car reaches it (more on wider
  lanes), because a row needs two cones to be fitted and the 80 degree view cannot see a cone beside the lane closer.
- Missed cones: a fallen or missing cone, or a missed detection, in one row (the other row still gives the lane) or
  in both rows for a batch or two.
- Glare or poor light: a shorter view, part of a row or the whole view lost.
- Camera covered, detector stopped or delayed: no cones at all, unexpectedly.
- Only one side visible: the start of a turn, a wide lane or a cone row at the edge of the view.

**Rule** (`MpcPolicy._command_without_lane` in `policy/action_policy/policy.py`; the numbers are the
`mpc` parameters in `config/ai4r_policy.yaml`):

1. One row visible: carry on at full speed, with the lane centre half a lane width from that row.
2. No usable lane, and the farthest cone last seen was at most 1.5 m ahead: the rows have ended. Keep the speed and
   follow the last plan, and request the stop once the car is 0.6 s x speed (its roll-out) from the point 0.25 m
   past that cone, so it stops in the 0.0 to 0.5 m zone (R5).
3. No usable lane otherwise: the lane was lost unexpectedly. Follow the last plan at half speed for at most 0.5 s;
   if the lane returns, carry on, otherwise request the stop (R7: within 0.6 s of the loss).
4. After a stop for a lost lane the car stays stopped until the policy is restarted (zero state, then state 3).
   Distances are dead-reckoned from the larger of the wheel speed and the requested speed, so a missing or stuck
   wheel speed makes the car stop early, never late.

**Result in simulation.** Before the rule the MPC stopped 0.4 to 0.9 m before the last cone pair on every tuning
case. With it, T1-T10 stop 0.19 to 0.30 m past the last pair and pass; T11 stops in the zone but fails R1b for an
unrelated reason (Task A2). T12, the camera covered from 3 m, requests the stop 0.45 s after the loss and rolls 0.8
m to a stop inside the lane. The first version slowed along a constant-deceleration ramp instead, but the simulated
speed controller lags the request, so the car rolled 0.64 to 0.75 m past the last pair and failed R5; requesting
the stop at the roll-out distance fixed it.

**To measure on the car.** The roll-out after a stop request (`stop_distance_per_speed_s`): the temporary Car
Control only coasts, so it is likely longer than the 0.6 s of the simulation. The camera's reliable range (Task
A1), which `lane_end_distance_m` must stay below.
