## Part 0: Team overview

**How to use this logbook.** It supports the oral, so it's made of diagrams, equations, plots, tables and short
bullets. Part 0 is the team. Parts 1–8 are the individuals, in pipeline order (interfaces, perception, movement
policy, car control). Each part follows the five standards: where it fits (S1), how it works (S2), simulation (S3),
on the car (S4); figures and captions (S5) are throughout. "See 5.3" means Part 5, Section 3. Results are labelled
**car**, **recording** or **sim**. A shared model, table or figure appears once, in its owner's part.

### 0.1 Goals and requirements

The policy runs on a 1/10 Traxxas car on the DREAM platform. Each step (one per cone batch, about 10 Hz) it takes the
cones, lidar, wheel speed, IMU and ArUco markers, and returns speed, steering and camera-pan set-points. It never
enables the vehicle.

- **Stage 1:** follow a straight lane of cones, hold the speed, stop at the lane end, stop safely if the lane is lost.
  An MPC and an RL policy were compared on the same tests before the MPC was chosen (milestone M1, 4.3).
- **Stage 2:** curves and other lane widths, obstacles in the lane, markers that set the lane side, and turns at
  intersections chosen by markers.

| Id | Requirement (scored by `tests/policy/stage1_scenarios.py`) | Pass if |
| --- | --- | --- |
| R1a / R1b | Body stays inside the cone rows / distance from centre at the last pair | 0.0 m / ≤ 0.10 m |
| R2 | Cones touched | 0 |
| R3 | Mean speed error, 1 m after the start to 1 m before the end | ≤ 0.10 m/s |
| R4 | RMS rate of change of requested curvature | ≤ 1.0 1/(m s) |
| R5 | Stop position past the last cone pair | 0.0–0.5 m |
| R6a / R6b | 99th-percentile step time on the car computer / solver failures | ≤ 20 ms / 0 |
| R7 | Lane lost → stop request | ≤ 0.6 s from the first batch without lane |

### 0.2 What ran on real cars

| Behaviour | Car, date | Result | Part |
| --- | --- | --- | --- |
| Speed control at 0.5 m/s | 20, 5 Oct | Feedforward + PI, 15% overshoot, ≈ 0.013 m/s steady error | 7.4 |
| MPC within the time budget | 20, 6 Oct | 0.46 ms median solve against 20 ms | 4.4 |
| Lane following on the lane estimate | 20, 9 Oct | Driven; no log kept | 2.4 |
| Lost-lane stop | 22, 12 Oct | Stopped 0.8 s after losing the lane | 4.4 |
| Keep to the left of the lane | 20, 20 Oct | Kept left, judged by eye, no log | 5.4 |
| Obstacle stops | 22 and 26 Oct | 5 stops, 0.21–0.33 m short of the box every time | 3.4 |
| Passing obstacles | 26 Oct | Passed a box on the right, one on the left, stopped 0.23 m short of a blocking box | 3.4 |
| Speed fault fix | 20, 26 Oct | 3 runs reached 0.5 m/s in 1.8–3.2 s (before: 0.87 m/s or no start) | 7.4 |
| Marker IDs from the live detector | 19 Oct | `ids=[2]` once the camera image reached the detector | 8.4 |
| Marker command | 13, 27 Oct | "Marker 14: KEEP_RIGHT, lane side RIGHT", logged 6 times | 5.4 |
| Left turn at an intersection | 20, 30 Oct | Marker recognised after a detector reload; turn refused as geometrically infeasible | 6.4 |

**Not shown on a car before access ended on 2 Nov:** a complete turn at an intersection. Intersection results are
simulation only (2.3, 5.3, 6.3).

**Videos (real cars only).** Proposed best video: the 26 Oct obstacle drive (3.4). Also: the 15 Oct two-board stop
(7.3). _Team to confirm the files and add any others._

**AI use.** As the FEIT policy allows for the logbook, members used AI tools to help with code, figures and drafting.
Each part's results come from the named commits, logs and runs.

### 0.3 The whole system and who built it

![Whole-system block diagram](ciaran/figures/Overall%20block%20diagram.png)

*Ciaran's block diagram. The policy (grey) runs once per cone batch; each block's parameters come from `config/`.
The sensors and the wheels each add a delay, and the wheels hold the last command (ZOH).*

| Block | What is in it | Who | Part |
| --- | --- | --- | --- |
| Dream input/output | Observation and action data model, data logging, `CarMemory`; repo, CI, upload script | Ciaran Gruber | 1 |
| Environment detection | Lane estimation, AUTO colour sides, width memory, camera pan hold; whole-system simulator | Thanh Long Nguyen | 2 |
| | Cone filter, lidar obstacles in lane coordinates, local map; obstacle stop and avoidance planner | Zhenling Zhang | 3 |
| MPC and pathing | MPC framework, Stage 1 design (MPC vs RL), lane end and lost lane, car models, Foxglove tools | Peiyu Wang | 4 |
| | Stage 1 tests, car 20 MPC tuning, lane side, marker actions, intersection turner, marker simulation | Toby Lee | 5 |
| | Measured-speed prediction, final-pair and cone-memory experiments, junction navigator, grouped IDs, stop persistence | Yuhao Liu | 6 |
| Control | System ID, `CarController` (feedforward + PI, per-side steering map), speed fix, avoidance validation | Chris (Chuhan) Wang | 7 |
| | Speed and steering tests with Chris, ArUco ID tests, camera pan calibration and simulation | Yifei Xia | 8 |

Sub-teams: interfaces (Ciaran); perception (Thanh Long, Zhenling); movement policy / MPC (Peiyu, Toby, Yuhao); car
control (Chris, Yifei).

### 0.4 Timeline

| Week | Merged into `dev` | On the cars |
| --- | --- | --- |
| 0 (27 Sep) | Repo, AI rules; Stage 1 task cards | Car 8: system ID, first PI |
| 1 (5–11 Oct) | MPC framework #1, upload script #4, CI #5, test cases #6 #10, cone filter #11, data logging #13 | Car 20: speed/steering maps, PI; time budget; recorded drives; first drive on the lane estimate |
| 2 (12–18 Oct) | First car run #8, MPC evaluation #14, car 20 calibration #15, Stage 1 MPC #16, `CarController` #17, obstacle detection #19, avoidance #21, `CarMemory` #22, two-obstacle fix #23 | Car 22: floor run, lost-lane stop, OpenCV 5 crash. Two-board video |
| 3 (19–25 Oct) | Foxglove #25, ArUco tools #26, lane side #27 #30, multi-obstacle planner #28, local map #33 | Car 20: avoidance, lane side, S-bend. Car 21: markers. Obstacle stops |
| 4 (26 Oct – 1 Nov) | Parameters #34 | Final obstacle drive and speed fix (car 20). Car 13: marker commands. Car 20: left-turn diagnosis, steering stand test |
| 5 (2–8 Nov) | Marker actions and intersections #35, intersection stopping #36; `dev` → `main` #38 | Car 14 session. Car access ended 2 Nov |

### 0.5 Facts every part uses

| Topic | The fact |
| --- | --- |
| Frames and signs | `base_link`: +x forwards, +y left. Lateral error, heading error and curvature are positive to the left |
| Cone detections | About 10 per second, about 0.2 s old when the policy gets them (0.197 s measured on car 20) |
| Car 20 steering reach | 0.48 1/m left, 0.92 1/m right at ±0.5 steering (measured 5 Oct). At ±0.8 the left limit is set to 0.91 1/m, not measured on the ground |
| MPC weights | Stage 1 design: N = 20, weights 3/1/0.1/3, \|κ\| ≤ 3.0. `dev` (car 20, #15): N = 12, 10/1/0.1/2, −0.92 ≤ κ ≤ 0.48. Results say which |
| Marker IDs on `dev` | 1–4 turn left, 5–6 turn right, 7–8 straight, any other ID stops. Keep-side IDs were used 23–27 Oct; 26 Oct simulations used 3/4/5 |
| Camera pan | Held at 0 since 10 Oct. Set to 0.3 once for a car test on 30 Oct, back to 0 in #36 |
| Speed feedforward offset | 0.48 (car 20, 6 Oct) → 0.25 (car 22, #16) → 0.15 (#34, 26 Oct) → 0.48 since 30 Oct |
| Stops | Obstacle stop brakes (drive −0.3, ≤ 1 s). Lane-end stop plans a coast: 0.6 s per m/s in sim, 2.1 s on `dev`; neither measured |

### 0.6 Experiments still needed (shared, method ready)

| Question | Method | Decision it feeds | Who |
| --- | --- | --- | --- |
| Car 20's steering reach at ±0.8 | Full lock at 0.3 m/s on the floor, measure the circle, 3 runs each way | `max_curvature_per_m`; a radius over 1.8 m means left turns can't fit lab crossings | Chris, Toby, Yuhao |
| Camera yaw with the pan at 0 | Cone 2 m ahead on the centre line (1° ≈ 3.5 cm); car on a tape-centred lane, compare $e_\psi$ with 0 | `camera_yaw_offset_deg` | Thanh Long, Toby, Yuhao, Peiyu |
| Roll-out after a stop request | Stop request at a tape mark at 0.5 m/s and other speeds, 3 runs | `stop_distance_per_speed_s`, brake settings | Peiyu, Yuhao, Zhenling, Chris |
| Left turn on the car against simulation | 1 m crossing, marker exactly 1 m before, 5 runs, Foxglove path vs simulated | Whether the turner works on a car | Toby, Yuhao |
