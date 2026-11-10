# Team overview

This logbook supports the interactive oral. It uses diagrams, plots, tables and short bullets rather than report
prose, and each person moves through their own part while answering questions.

**How to read it.**
- Part 0 is the team: goals, what ran on real cars, the whole-system diagram, who did what, the timeline, the facts
  every part shares, and the experiments still needed.
- Parts 1–8 are the individuals, in the order of the pipeline: data and interfaces, then perception, the movement
  policy and car control. Each follows the five standards: where it fits (1), how it works (2), simulation (3), on
  the car (4), and figures with captions (5).
- Section numbers inside a part refer to that part. "Toby's Section 3.4" or "his 3.4" means Section 3.4 of Toby's
  part.
- Claims name a commit, PR, log or result file where one exists. Simulation results are marked as simulation, with whether
  they were checked on a car.
- A shared model, table or figure appears once, in its owner's part. Other parts point to it.

## 0.1 Goals and requirements

The policy runs on a 1/10 Traxxas car on the DREAM platform. Each step it takes the cones, lidar, wheel speed, IMU
and ArUco markers, and returns drive, steering and camera-pan set-points. It never enables the vehicle.

- **Stage 1** (`docs/movement-policy-tasks.md`): follow a straight lane of evenly spaced cones, hold the speed, stop
  at the lane end, and stop safely when the lane is lost. Both an MPC and an RL policy were developed and compared
  on the same tests before one was chosen (milestone M1, Peiyu 3.4).
- **Later stages:** curves and other lane widths, obstacles in the lane, markers that set the lane side, and turns
  at intersections chosen by markers.

**Stage 1 requirements** (`docs/stage1-approach.md` 1.1; scored by the harness in `tests/policy/stage1_scenarios.py`):

| Id | Requirement | Metric | Pass if |
| --- | --- | --- | --- |
| R1a | Stay in lane | Largest distance any body corner goes past a cone row while beside the cones | 0.0 m |
| R1b | Follow the lane centre | Distance from the lane centre at the last cone pair, or where the car stopped before it | at most 0.10 m |
| R2 | No cone touched | Cones within a cone base radius (0.05 m) of the car body | 0 |
| R3 | Hold speed | Mean absolute speed error from 1 m after the start to 1 m before the end | at most 0.10 m/s |
| R4 | Smooth | RMS rate of change of the requested curvature | at most 1.0 1/(m s) |
| R5 | Stop at the lane end | Car centre's stop position past the last cone pair | 0.0 to 0.5 m |
| R6a | Real time | 99th-percentile calculation time per policy step, on the car computer | at most 20 ms |
| R6b | Always a usable command | Steps where the solver didn't converge | 0 |
| R7 | Safe stop when the lane is lost | First detection batch without the lane to the stop request | at most 0.6 s (0.8 s from the camera, with the 0.2 s detection age) |

## 0.2 What ran on real cars

| Behaviour | Car and date | Result | Where |
| --- | --- | --- | --- |
| Speed control at 0.5 m/s | Car 20, 5 Oct | Feedforward + PI held 0.5 m/s, 15% overshoot, about 0.013 m/s steady error | Chris 4.3 |
| Speed fault fix | Car 20, 26 Oct | 3 runs reached 0.5 m/s in 1.8–3.2 s, peaks 0.57–0.62 m/s (was 0.87 m/s, or no start) | Chris 4.4 |
| MPC within the time budget | Car 20, 6 Oct | 0.46 ms median MPC solve, against a 20 ms budget | Peiyu 4.1 |
| Lane following with the lane estimate | Car 20, 9 Oct | Driven with the branch; no log was kept | Thanh Long 4.5 |
| Lost-lane stop | Car 22, 12 Oct | Stopped 0.8 s after losing the lane | Peiyu 4.2 |
| Keep to the left side of the lane | Car 20, 20 Oct | Kept left, judged by eye, no log | Toby 4.2 |
| Obstacle stops | 22 and 26 Oct, 5 stops | Stopped 0.21–0.33 m short of the box every time | Zhenling E2 |
| Passing obstacles | 26 Oct, run `2026-10-26_14-10-54` | Passed a box on the right and one on the left, then stopped 0.23 m short of a blocking box | Zhenling E3 |
| Marker IDs from the live detector | 19 Oct | `ids=[2]` once the camera image reached the detector | Yifei 4.2 |
| Marker command | Car 13, 27 Oct | "Marker 14: KEEP_RIGHT, lane side RIGHT", 6 times | Toby 4.3 |
| Left turn at an intersection | Car 20, 30 Oct | Marker recognised after a detector reload, then the turn was refused as geometrically infeasible | Yuhao 4.1–4.2 |

**Not shown on a car before access ended on 2 Nov:** a complete turn at an intersection. The intersection results
are simulation only (Toby 3.3–3.5, Yuhao 3.5–3.6, Thanh Long 3.5).

**Videos.** Zhenling's demo video of the 26 Oct obstacle drive (her E3), and the 15 Oct two-board video (Chris 3.2).
_Team to fill in: the submitted video files and which one is the "best" video._

## 0.3 The whole system

![Whole-system block diagram](ciaran/figures/Overall%20block%20diagram.png)

*Ciaran's block diagram of the whole system. The policy (grey box) runs once per new cone batch. Each block's
parameters come from `config/`. The sensors and the wheels each add a delay, and the wheels hold the last command
(ZOH). The table below names who built each block.*

| Block in the diagram | What is in it | Who | Part |
| --- | --- | --- | --- |
| Dream Input and Output | Observation and action data model, data logging, `CarMemory` | Ciaran | 1 |
| Environment detection | Lane estimation from the cones, AUTO colour sides, width memory, camera pan hold | Thanh Long | 2 |
| | Cone filter, lidar obstacles in lane coordinates, local map | Zhenling | 3 |
| | ArUco marker reader | Toby (reader), Yifei (test tools) | 5, 8 |
| MPC algorithm and pathing | MPC framework, Stage 1 design (MPC or RL), lane end and lost lane | Peiyu | 4 |
| | Car 20 tuning, lane side, marker actions, intersection turner | Toby | 5 |
| | Measured-speed prediction, junction navigator, stop persistence | Yuhao | 6 |
| | Obstacle stop and avoidance planner | Zhenling | 3 |
| Control (PID) | `CarController`: feedforward + PI speed control, per-side steering map, brake | Chris (Yifei on the tests; Zhenling added the brake) | 7, 8 |
| Tools beside the loop | Scenario renderer (Ciaran), whole-system simulator and lane overlay (Thanh Long), obstacle simulation (Zhenling), Foxglove export and run report (Peiyu), Stage 1 harness and marker simulation (Toby), car models (Peiyu), camera simulation (Yifei) | | |

## 0.4 Who did what

| Sub-team | Person | Main work | PRs | Part |
| --- | --- | --- | --- | --- |
| Data and interfaces | Ciaran Gruber | Repository, CI, upload script; observation data model, data logging, scenarios and renderer, `CarMemory`; the curve veering investigation | #3, #4, #5, #7, #13, #20, #22 | [1](#ciaran) |
| Road detection | Thanh Long Nguyen | Lane estimation and `LaneDetection`; camera pan bias and hold; AUTO sides and width memory; whole-system simulator and live viewer | commits in #19; `feature/car-simulator` | [2](#long) |
| Obstacles | Zhenling Zhang | Cone filter, lidar obstacle detection, obstacle stop and brake, avoidance planner, local map, obstacle simulation | #11, #19, #21, #28, #33, #34 | [3](#zhenling) |
| Movement policy (MPC) | Peiyu Wang | Stage 1 task list, MPC framework and first car run, Stage 1 design (baseline, MPC, RL, M1), lane end and lost lane, car models, Foxglove and run timing | #1, #8, #16, #25; open #18, #24, #31, #32 | [4](#peiyu) |
| Movement policy (MPC) | Toby Lee | Stage 1 test cases and harness, car 20 MPC tuning, lane side, marker actions, intersection turns, marker simulation | #6, #10, #15, #27, #30, #35 | [5](#toby) |
| Movement policy (MPC) | Yuhao Liu | Measured-speed MPC, final-pair and cone-memory experiments, junction navigator, grouped marker IDs, left-turn diagnosis and stop persistence | #14, #35, #36 | [6](#yuhao) |
| Car control | Chris (Chuhan) Wang | System ID (cars 8 and 20), `CarController`, the 26 Oct speed fix, obstacle-avoidance validation | #17, #23 | [7](#chris) |
| Car control | Yifei Xia | Speed and steering tests with Chris, ArUco ID tests, camera pan calibration and simulation, battery-monitoring feasibility | #26 | [8](#yifei) |

## 0.5 Timeline

Weeks run Monday to Sunday; week 1 is the week of 5 Oct. Merge dates are from GitHub.

| Week | Merged into `dev` | On the cars |
| --- | --- | --- |
| 0 (27 Sep – 4 Oct) | Repository set-up and AI rules (Ciaran). Stage 1 task cards (Peiyu, 27 Sep) | Car 8: system ID and first PI (Chris, 29 Sep) |
| 1 (5–11 Oct) | MPC framework #1, v0.3.0 update #3 #7, upload script #4, CI #5 (5 Oct). Test cases #6 #10, cone filter #11 (6 Oct). Data logging and scenarios #13 (7 Oct) | Car 20: speed and steering maps, PI (Chris and Yifei, 5 Oct); time budget and stand tests (Peiyu, 6 Oct); the four recorded drives (Ciaran, 6 Oct); first drive with the lane estimate (9 Oct) |
| 2 (12–18 Oct) | First car run #8, MPC evaluation #14, car 20 MPC calibration #15, Stage 1 MPC with car control #16, `CarController` #17 (12 Oct). Scenarios with lidar #20 (13 Oct). Obstacle detection with the lane estimate #19 (14 Oct). Avoidance v1 #21 (16 Oct). `CarMemory` #22, two-obstacle fix #23 (18 Oct) | Car 22: floor run, lost-lane stop, OpenCV 5 crash (Peiyu, 12 Oct). Two-board video (15 Oct) |
| 3 (19–25 Oct) | Foxglove #25, ArUco test tools #26 (19 Oct). Lane side #27 #30, multi-obstacle planner #28 (20 Oct). Local map #33 (23 Oct) | Car 20: avoidance runs (Chris, 19 Oct) and lane side (Toby, 20 Oct). S-bend drive (Zhenling, 20 Oct). Car 21: markers (21 Oct). Obstacle stops (22 Oct) |
| 4 (26 Oct – 1 Nov) | Parameters #34 (26 Oct) | Final obstacle drive (Zhenling) and speed fix on car 20 (Chris), 26 Oct. Car 13: marker commands (27 Oct). Car 20: left-turn diagnosis and steering-limit stand test (30 Oct) |
| 5 (2–8 Nov) | Marker actions and intersections #35, intersection stopping and CI #36 (2 Nov). `dev` into `main` #38 (7 Nov) | Car 14 session (Toby, 2 Nov). Car access ended on 2 Nov |
| 6 (9–15 Nov) | This logbook | |

## 0.6 Facts every part uses

These are stated once here. When a part quotes one of them, it should agree with this table.

| Topic | The fact | Source |
| --- | --- | --- |
| Frames and signs | `base_link`: +x forwards, +y left. Lateral error, heading error and curvature are positive to the left | `docs/lane-detection-interface.md` |
| Cone detections | About 10 per second, about 0.2 s old when the policy gets them (0.197 s on car 20, 6 Oct; 0.21 s in the 6 Oct recordings) | Peiyu 4.1, Toby 4.1 |
| Car 20 steering reach | 0.48 1/m left and 0.92 1/m right at ±0.5 steering, measured 5 Oct. With the ±0.8 interval the left limit is set to 0.91 1/m, which hasn't been measured on the ground | Chris 2.1, 4.5 |
| MPC weights | Two versions. Stage 1 design (A4): N = 20, weights 3/1/0.1/3, \|κ\| ≤ 3.0 1/m. `dev` (car 20 calibration, #15): N = 12, weights 10/1/0.1/2, limits 0.48 / 0.92 1/m. Each result says which it used | Peiyu 3.2, Toby 2.1 |
| Marker IDs | `dev`: 1–4 turn left, 5–6 turn right, 7–8 straight, any other ID stops the run. The 26 Oct simulations used 3/4/5. Keep-side commands were used from 23 to 27 Oct | Toby 2.2, Yuhao 2.4 |
| Camera pan | Held at 0 by the policy since 10 Oct (`f49333d`). Set to 0.3 once for a car test on 30 Oct (`bf5e08c`), back to 0 in #36 | Thanh Long 2.3, 4.2 |
| Speed feedforward offset on `dev` | 0.48 (car 20, 6 Oct), then 0.25 (car 22, #16), 0.15 (#34, 26 Oct), and 0.48 again since `bf5e08c` (30 Oct) | Chris 2.2, Zhenling timeline, `config/ai4r_policy.yaml` |
| Stops | An obstacle stop brakes (`brake_action` 0.3 for at most 1 s), and since #36 so does a stop after a refused turn. The lane-end stop is planned for a coast: `stop_distance_per_speed_s` is 0.6 s in simulation and 2.1 s on `dev` (car 22's coast), neither measured on a car | Zhenling 3.3, Yuhao 3.6, Peiyu 2.3 |

## 0.7 Experiments still needed (shared)

These came up in more than one person's work. Each row keeps every owner's method. Experiments that only one person
needs are in that person's part.

| Question | Method | Controlled | Decision it feeds | Who |
| --- | --- | --- | --- | --- |
| How far can car 20 steer at the ±0.8 interval, left and right? | Full lock at 0.3 m/s on the floor, measure the circle's diameter, 3 runs each way (Toby). Repeat at each admitted setting (Yuhao). Re-map steer → curvature on the ground after the 30 Oct stand test (Chris) | speed, floor, battery | `max_curvature_per_m` = 1/R. A radius over 1.8 m means left turns can't fit the lab crossings | Chris, Toby, Yuhao |
| Is the camera yaw zero with the pan held at 0? | A cone 2 m ahead on the centre line: each degree moves it 3.5 cm sideways (Toby, Peiyu). The car on a tape-centred straight lane, compare $e_\psi$ with 0 (Thanh Long). Surveyed targets at several ranges, again after a component restart (Yuhao) | pan 0, the car's pose by tape | `camera_yaw_offset_deg`. Whether car 22's steady +0.22 rad heading was camera yaw | Thanh Long, Toby, Yuhao, Peiyu |
| Car 22's steering direction and centre | On a stand with the car left of the lane centre, the wheels must turn right; then the camera-yaw test above | the car's pose, pan 0 | Steering sign and trim before car 22 drives a lane | Peiyu, Chris |
| How far does the car roll after a stop request? | Drive at 0.5 m/s, request a stop at a tape mark, measure the distance, 3 runs (Peiyu). Several speeds, same surface and battery (Yuhao). Zhenling's lidar method (her E2) already measures braked stops | speed, floor, battery | `stop_distance_per_speed_s` (lane-end stop in 0–0.5 m), `obstacle_stop_distance_per_speed_s` and the brake settings | Peiyu, Yuhao, Zhenling, Chris |
| Does a left turn at an intersection work on the car, and does it match simulation? | A left turn in a 1 m crossing with the marker exactly 1 m before the entry, 5 runs (Toby). Fix the road width, marker distance and start pose; compare the old and new preparation (Yuhao) | marker distance, speed, start pose | Recognition distance, tangent start, body clearance, exit and stop distance; the path in Foxglove against the simulated path | Toby, Yuhao |
