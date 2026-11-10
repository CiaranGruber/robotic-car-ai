## Part 5: Toby Lee — MPC tuning, lane shifting, marker actions and intersection turns

**Role.** MPC sub-team. Card 2 test cases and harness, MPC tuning for car 20, lane-side following, mapping ArUco
markers to policy actions, the intersection turner and marker simulation, then real-robot testing. **PRs** #6 #10
(Stage 1 tests), #15 (car 20 calibration), #27 #30 (lane side), #35 (markers and intersections); open:
`fix/left-turn-steering-and-obstacles`. **On cars:** lane side (car 20) and marker commands (car 13). A full
intersection turn wasn't shown on a car, so turn results are simulation.

### 5.1 Where it fits (S1)

![System diagram](toby/figures/01_system.png)

*Blue: parts I built or tuned; grey: teammates' parts mine depend on. Policy step every 0.1 s within 20 ms.*

| Interface | Carries | Constraint |
| --- | --- | --- |
| `FiducialBatch` in | marker id, pose in `base_link` | Needs the RGB image (`--aruco`); distance right only if `default_marker_size_m` = printed 0.081 m |
| `LaneDetection` in (2.2) | centre line, width | Cross-road cones bend the fit near a junction, so the turner keeps only approach-corridor cones |
| Local map pose | car relative to planned entry | Dead reckoning drifts, but a turn is only ≈ 3 m |
| Lidar obstacles (3.2) | points ahead | Marker stands and lane cones appear as obstacles (car 13) |
| (speed, curvature) → `CarController` (7.2) | command | Must stay within car 20's 0.48 / 0.92 1/m |

- **Only the nearest marker counts** (≤ 2 m, 2 steps in a row): a marker further on must not take over; one noisy frame
  must not give a command.
- **The turn is a separate manoeuvre, not another MPC:** inside a junction there's no lane to follow, so the turner
  drives a planned arc and hands back when it sees the exit lane. The obstacle stop stays on throughout.
- **System consequence:** the car's steering reach decides whether turns work. 0.48 1/m means R = 2.08 m, which can't
  fit a 1 m crossing — a control-team value decides whether my subsystem works.

### 5.2 How it works (S2)

**Car 20 MPC with lane side** (model and least-squares form in 4.2): N = 12 (≈ 0.6 m at 0.5 m/s),

$$\min_{\kappa} \sum_k 10\,(e_k - e^{line}_k)^2 + \psi_k^2 + 0.1\,(\kappa_k - \kappa^{path}_k)^2 + 2\,(\kappa_k - \kappa_{k-1})^2 \quad\text{s.t.}\quad -0.92 \le \kappa_k \le 0.48$$

- Bounds are the only constraints because they are the car's real limit; a soft lane-edge constraint isn't needed
  because the cost tracks a line inside the lane.
- **Lane side:** $e^{line}$ is `edge_clearance_m` (c) in from the chosen edge, so the offset available is $w/2 - c$; it
  moves at most `offset_change_per_m` per metre driven.

| Marker ID (`dev`) | Command | Effect |
| --- | --- | --- |
| 1–4 | `TURN_LEFT` | left turn at the junction the marker stands at |
| 5–6 | `TURN_RIGHT` | right turn |
| 7–8 | `STRAIGHT` | hold the approach line across |
| (23–27 Oct mapping) | `KEEP_LEFT/RIGHT/CENTRE` | set `lane_side` until another side marker |
| other | stop | stop the run |

**Intersection turn: APPROACH → ARC → EXIT.**

![Turn geometry](toby/figures/08_turn_geometry.png)

*Right turn, 0.8 m roads, R = 1.2 m. From the lane centre no start point clears both the inside corner and the far
row; after moving 0.15 m to the outside (a hook turn), one does.*

1. **APPROACH:** the entry is placed from the marker (1 m before it) and the car's pose; the MPC moves to the outside
   of the lane, then holds the approach heading at 0.3 m/s.
2. **ARC:** pure pursuit along the arc, curvature capped at 1.2/R. The start $s$ is clamped into the window where the
   body clears both obstacles ($b$ half width 0.125 m, $f$ front 0.30 m, $r_c$ cone radius, $m$ margin 0.03 m, $a$ move
   aside); empty window → stop ("no clear start"):

$$-\sqrt{(R-b-r_c-m)^2 - (R-a-w/2)^2} \;\le\; s \;\le\; w_{cross} - r_c - m - \sqrt{f^2+(R+b)^2}$$

3. **EXIT:** lane following resumes after 60° of turn once a lane lines up; no exit lane within 1.5 m → stop.

### 5.3 Simulation (S3)

**Which MPC settings suit car 20? (12 Oct)** `curved_lanes.py`: 1.5–3 m curves and an S bend, −3 to +3° camera yaw,
measured noise and delay, 4 seeds → 84 runs per setting.

![MPC tuning](toby/figures/02_mpc_tuning.png)

*Curved-edge lanes kept and steering activity per setting. The dev-defaults bar scores 84 only because the simulator
clips its impossible 3.0 1/m command.*

- Car 20's run settings (0.5 both ways, heading 5): 12/84 with straight lane edges, 73/84 with curved. Chosen
  (0.48 L / 0.92 R, change weight 2): 84/84, steering activity 0.71 → 0.20 1/m per s.
- **Reversed judgement:** a smaller sweep pointed to the heading weight; the larger one showed the missing right limit
  was losing curves.

**Why wasn't the lane change visible? (26 Oct)** Offset available = $w/2 - c$: with c = 0.40 m, 0 cm in a 0.8 m lane,
10 cm in 1 m. Full-policy sweep of c ∈ {0.40, 0.28, 0.25}, rate 0.2/0.3 per m → chose 0.28 m at 0.3: 40/40 runs,
worst side error 0.017 m, body ≥ 0.13 m inside the lane (0.25 m left only 0.10 m margin).

![Lane change simulation](toby/figures/04_lane_change_sim.png)

**Does the whole policy turn correctly? (27 Oct)** `marker_simulation.py` runs the real `MovementPolicy` with a lagged
kinematic car; cones and markers seen 0.3–2.5 m, 3 cm noise, 10% missed, 0.2 s delay. Changes made before car time:
marker 1 m before the entry, move aside before turning, corner clearance 0.03 m (59/60 vs 53/60 at 0.05). Result:
1 m roads 100/100, 0.8 m roads 97/100.

**Reproducing "right works, left fails" (1–2 Nov).** Yuhao's exact commit `bf5e08c` with car 20's left reach,
10 cases × 10 seeds: left 2/40, right 35/40 — the car's pattern. Cause: R = 2.08 m has ≈ 2 cm of room in a 1 m
crossing and none in 0.8 m.

![Turn simulation](toby/figures/05_turn_sim.png)

![Root cause](toby/figures/06_root_cause_bars.png)

- **Fix** (7 variant branches, 10 seeds × 10 cases): ±0.8 steering with left limit 0.91 1/m, lookahead 0.8 m,
  phantom-obstacle filters. All cases 57% → 99%, left turns 2/40 → 39/40.

**What must the car's setup match? (2 Nov)**

![Sensitivity](toby/figures/07_sensitivity.png)

*Left: left-turn pass rate against the car's real left reach. Right: what the config must match; green = measured
value set in config.*

- If the config says 0.91 but the car reaches 0.55–0.7 1/m, every left turn fails. ±4° of camera yaw costs a whole turn
  direction (+4° → left 0%, right 100%, the car's pattern). Marker 1.5 m early: 20%, 100% once configured. Robust to
  doubled latency (92%), 25% dropout (96%); sensitive to 3 cm cone jitter (68%).
- **Can't conclude:** car pass rates (no slip or servo nonlinearity, guessed marker noise, reach set directly, ideal
  crossings). **Can:** reproduce a failure mechanism and rank which calibrations matter.

### 5.4 On the car (S4)

**Simulator parameters from real recordings (12 Oct).** `real_car_measurements.py` on Ciaran's four drives:

| Quantity | Assumed | Measured | Used in sim |
| --- | --- | --- | --- |
| Lane width | 1.0 m | 0.77 m | 0.8 m |
| Cone noise | 0.01 m | 0.040 m | 0.03 m |
| Latency | 0 s | 0.21 s | 0.2 s |
| Range (95%) / nearest | 4 m / 0 | 2.54 m / 0.41 m | 2.5 / 0.4 m |
| Camera yaw | 0° | 7.3° left (pan never commanded) | held at 0, swept ±3° |

**Lane shifting (20 Oct car 20; 21 Oct car 21).** Changed only `lane_side` (CENTRE → LEFT), same track and speed:
the car kept left (by eye, no log). On car 21 the shift was hard to see → the clearance analysis above. Next test: log
the offset in Foxglove against the simulated ±0.12 m (0.8 m lane).

**Is each link of the marker chain working?** Tested bottom up, one variable at a time:

| Link | Check | Result | Decision |
| --- | --- | --- | --- |
| Camera image | runtime status, image topic | 21 Oct: 0 marker batches in 9 runs; GPU detector publishes no RGB | `car_management.sh --aruco` |
| Distance | marker 1.5 m ahead, echo detections | size must match the print (0.081 m); 30 Oct detector still had 0.25 m (6.4) | fixed in config |
| Command | policy logs | 27 Oct car 13: "Marker 14: KEEP_RIGHT", 6 times | works on a car |
| Manoeuvre | Foxglove turn plan | 2 Nov: wrong branch uploaded, then no `--aruco`; time ran out | not shown |

**Phantom obstacles: turns with obstacle detection (27 Oct, car 13).**

![Car 13 phantom avoidance](toby/figures/09_car13_phantom_avoidance.png)

*One run with no obstacle on the track: avoidance planned 16 lines, flipping between −0.44 and +0.28 m.*

- With marker stands added to the simulated lidar, phantom avoidance appeared in 29% of runs, mostly from the lane's
  own cones. Fix: ignore returns within 0.15 m of a seen marker, require 3 sightings in 5 steps, ignore known cones
  while turning → 4%, real-box cases still 12/12. Not yet tested on a car.

**Lessons:** measure before tuning (the 0.48/0.92 asymmetry recorded on 12 Oct explained the 1 Nov failure);
reproduce the car's failure in simulation first; check branch, `CAR_NUM` and `--aruco` before every upload.
