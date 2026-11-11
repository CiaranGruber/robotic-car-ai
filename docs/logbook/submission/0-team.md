## Part 0: Team overview

Parts follow the sub-teams in pipeline order: 1 interfaces, 2 perception, 3 movement policy (MPC), 4 car control.
Each part opens with what its members share, then one section per person: **S1** where it fits, **S2** how it
works, **S3** simulation, **S4** real car. Results are labelled **car**, **recording** or **sim**. The full
per-person sections are in `docs/logbook/<name>.md`.

### 0.1 Goals and requirements

The policy runs once per cone batch (≈ 10 Hz) on a 1/10 car on DREAM: cones, lidar, wheel speed, IMU and ArUco
markers in; speed, curvature and camera-pan set-points out. It never enables the vehicle.

- **Stage 1:** follow a straight cone lane, hold 0.5 m/s, stop at the lane end and when the lane is lost. MPC and RL
  were compared before choosing the MPC (milestone M1, 3.2).
- **Stage 2:** curves and other widths, obstacles, markers that set the lane side, turns at intersections.

| Id | Requirement (`tests/policy/stage1_scenarios.py`) | Pass if |
| --- | --- | --- |
| R1a / R1b | Body inside the cone rows / distance from centre at the last pair | 0.0 m / ≤ 0.10 m |
| R2 | Cones touched | 0 |
| R3 | Mean speed error (1 m after start to 1 m before end) | ≤ 0.10 m/s |
| R4 | RMS rate of change of requested curvature | ≤ 1.0 1/(m s) |
| R5 | Stop position past the last cone pair | 0.0–0.5 m |
| R6 | p99 step time on the car / solver failures | ≤ 20 ms / 0 |
| R7 | Lane lost → stop request | ≤ 0.6 s |

### 0.2 What ran on real cars

| Behaviour | Car, date | Result | See |
| --- | --- | --- | --- |
| Speed control at 0.5 m/s | 20, 5 Oct | Feedforward + PI: 15% overshoot, ≈ 0.013 m/s steady error | 4.2 |
| MPC inside the time budget | 20, 6 Oct | 0.46 ms median solve against 20 ms | 3.2 |
| Lost-lane stop | 22, 12 Oct | Stopped 0.8 s after losing the lane | 3.2 |
| Keep to the left of the lane | 20, 20 Oct | Kept left (by eye, no log) | 3.3 |
| Obstacle stops | 22 and 26 Oct | 5 stops, 0.21–0.33 m short of the box | 2.3 |
| Passing obstacles | 20, 26 Oct | Passed a box right, a box left, stopped 0.23 m short of a blocking box | 2.3 |
| Speed fault fix | 20, 26 Oct | 3 runs settled at 0.50–0.54 m/s (before: 0.87 m/s or no start) | 4.2 |
| Marker IDs from the live detector | 19 Oct | `ids=[2]` once the camera image reached the detector | 4.3 |
| Marker command | 13, 27 Oct | "Marker 14: KEEP_RIGHT", 6 times | 3.3 |
| Left turn at an intersection | 20, 30 Oct | Marker read after a detector reload; turn refused as geometrically infeasible | 3.4 |

**Simulation only:** a complete intersection turn. **Videos (real cars):** best video, 26 Oct obstacle drive (2.3);
also the 15 Oct two-board stop (4.2). **AI use:** members used AI tools for code, figures and drafting, as the FEIT
policy allows for the logbook; every result comes from the named commits, logs and runs.

### 0.3 The system and who built it

![Whole-system block diagram](ciaran/figures/Overall%20block%20diagram.png)

*The policy (grey) runs once per cone batch; each block's parameters come from `config/`. Sensors and wheels each add
a delay; the wheels hold the last command (ZOH).*

| Block (sub-team) | Person | Built |
| --- | --- | --- |
| Dream input/output (interfaces) | Ciaran Gruber | Data model, logging, replay, `CarMemory`, repo, CI |
| Environment detection (perception) | Thanh Long Nguyen | Lane estimation, pan hold, whole-system simulator |
| | Zhenling Zhang | Cone filter, lidar obstacles, avoidance planner, obstacle stop, local map |
| MPC and pathing (movement policy) | Peiyu Wang | MPC framework, Stage 1 design (MPC vs RL), lane end, car debugging, Foxglove |
| | Toby Lee | Test harness, MPC settings for car 20, lane side, marker actions, intersection turner, left-turn fix (branch) |
| | Yuhao Liu | Measured-speed MPC, cone-memory U-turn, junction path finding, marker IDs and turn preparation, stop latch, car 20 diagnosis |
| Control (car control) | Chris (Chuhan) Wang | System ID, `CarController`, speed fix, avoidance validation |
| | Yifei Xia | Control tests with Chris, ArUco ID tests, camera pan calibration and simulation |

### 0.4 Facts every part uses

| Topic | Fact |
| --- | --- |
| Frames | `base_link`: +x forwards, +y left; lateral error, heading error and curvature positive to the left |
| Cone detections | ≈ 10 per second, ≈ 0.2 s old (0.197 s measured on car 20) |
| Car 20 steering reach | +0.48 1/m left, −0.92 1/m right at ±0.5 steering (5 Oct). Left limit 0.91 1/m at ±0.8 is not ground-measured |
| Marker IDs (`dev`) | 1–4 turn left, 5–6 turn right, 7–8 straight, other IDs stop; keep-side IDs used 23–27 Oct |
| Camera pan | Held at 0 since 10 Oct (2.2) |
| Stops | Obstacle stop brakes (drive −0.3, ≤ 1 s); lane-end stop plans a coast; roll-out not measured |
