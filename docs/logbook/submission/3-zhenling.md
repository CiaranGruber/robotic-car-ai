## Part 3: Zhenling Zhang — cone filter, obstacles, avoidance and local map

**Role.** Everything between the sensors and the MPC that keeps the car from hitting things: cone filter, lidar
obstacle detection in the lane, obstacle stop and brake, avoidance planner, local map; plus a closed-loop obstacle
simulation and each run's settings saved with its log. **PRs** #11, #19, #21, #28, #33, #34. **On cars:** all five
parts ran; in the final drive (26 Oct) the car passed boxes on the right and left and stopped for one in the middle.

### 3.1 Where it fits (S1)

![My parts in the system](zhenling/figures/Z1_where_my_parts_fit.png)

*One step per cone batch (≈ 10 Hz). Green: car and sensors; blue: teammates; orange: mine. Lidar points become
obstacles only once the lane is fitted, and every output reaches the car through the MPC and car control.*

| Interface | Carries | Constraint |
| --- | --- | --- |
| Cones in | position, colour, confidence | ≈ 10 Hz, 0.2 s late; duplicates and floating detections happen |
| Lidar in | points in `base_link` | New scan only every 0.2–0.3 s, 0.15–0.3 s old; plane ≈ 0.12 m up, so it hits cones |
| `LaneDetection` in (2.2) | centre arc, width | Estimate jumps ≈ 0.1 m step to step |
| `LanePath` d(s) → MPC | line to follow as offset from centre | Chain of quintics: continuous curvature |
| Obstacle distance → MPC stop | m from the car's front | Front taken 0.30 m ahead of `base_link` (not measured) |
| Brake → `CarController` | drive −0.3, ≤ 1 s | Only straight after forward drive |

**Tools.** `tests/policy/obstacle_simulation.py` runs the whole `MovementPolicy` closed loop: the measured lab car,
camera view with boxes hiding cones, 2D lidar (0.5° beams, 1 cm noise, 0.2 s old) and braking travel as a setting;
12 cases × N seeds. On the car, every run logs its exact settings (`_config.json`), so it can be replayed through
any policy version, and the lidar measures the stop gap directly.

### 3.2 How it works (S2)

**Cone filter.** Keep $0 < x$, $|y| \le 2.0$ m, $\sqrt{x^2+y^2} \le 3.5$ m, $-0.15 \le z \le 0.25$ m, confidence ≥ 0.5;
merge same-colour cones closer than 0.10 m.

**Obstacle detection in lane coordinates.** Each lidar point gets $s$ (along the centre arc from the car's front) and
$l$ (offset from the centre line). Keep $0 < s \le 3.5$ m and $|l| < w/2 - 0.15$ m (the margin covers the cone radius
plus ≈ 0.1 m edge-fit error). Cluster returns < 0.15 m apart; ≥ 3 returns per object.

**Obstacle stop.** Stop when an object is in the car's way within

$$d_{stop} = (t_b + t_{scan})\,\max(v_{target}, v_{measured}) + m, \qquad t_b = 0.8\ \text{s},\ m = 0.2\ \text{m}$$

so 0.70 m for a 0.2 s old scan at 0.5 m/s. It must be seen 3 steps in a row (the lane jumps ≈ 0.1 m per step). The stop
latches: only a person restarts the car, since the lidar sees the box poorly when close.

**Avoidance planner** (in the lane's Frenet frame):
1. Each obstacle gets a keep-clear stretch; inside it the body keeps 0.10 m from the obstacle and 0.05 m from the cones.
2. Candidate held offsets in each free gap; overlapping obstacles are passed on one offset.
3. Holds are joined by quintic moves that match offset, slope and curvature at both ends:

$$d(u) = D\,(10u^3 - 15u^4 + 6u^5),\quad u = s/L,\qquad \max|d''| = \frac{10}{\sqrt3}\frac{D}{L^2}\ \Rightarrow\ L = \sqrt{\frac{10}{\sqrt3}\frac{D}{\kappa_{max}}}$$

   v1's half-cosine ramp had curvature $D\pi^2/(2L^2)$ at its ends, so steering jumped at every join; the quintic
   doesn't.
4. Rank: obstacles passed (nearest first), then room for the next, then the lowest cost

$$J = \Delta s \sum \Big[(d - d_{usual})^2 + \kappa_{add}^2 + (d - d_{prev})^2\Big] - 5\cdot\text{room}$$

![Avoidance planner](zhenling/figures/Z7_avoidance_planner_candidates.png)

*Real planner and config, 0.8 m lane, box right then left. (a) Grey: 22 other candidates; blue: the chosen line with
the body (0.25 × 0.60 m) checked along it. (b) Its curvature is continuous and inside the 0.45 1/m ramp limit, below
the car's 0.48 1/m left limit.*

**Local map.** Dead-reckon the pose from wheel speed; match each new cone to the nearest remembered cone of its colour
within 0.12 m; with ≥ 3 confirmed matches, apply half of a least-squares rigid fit each step (capped 0.05 m, 3°).
Recall confirmed cones that left the view; forget 0.5 m behind or after 4 s unseen. Lane detection fits the batch
first and adds recalled cones only if that already gives a lane, so a lane is never made from memory alone.

### 3.3 Simulation: retuning against needless stops and hits (S3, 21 Oct)

- **Problem:** the previous settings stopped needlessly and had no braking travel in the stop distance.
- **Method:** previous vs retuned, 10 noise seeds per case, braking 0.6 s per m/s.

| Setting | Before → after | Problem it fixes |
| --- | --- | --- |
| `corridor_half_width_m` | 0.20 → 0.175 m | lane jumps put passable boxes in the way |
| `lane_edge_margin_m` | 0.10 → 0.15 m | cone returns near a jumping edge counted as objects |
| `obstacle_confirm_steps` | 2 → 3 | two jumpy steps were enough to stop |
| `cone_clearance_m` | 0.10 → 0.05 m | free gap needed beside a box 0.45 → 0.40 m |
| `obstacle_stop_distance_per_speed_s` | 0.0 → 0.8 s | stop distance had no braking travel |

| Case | Previous | Retuned |
| --- | --- | --- |
| Box on the right, 0.5 m gap | 3/10 (7 needless stops) | 10/10 passed |
| Box on the left | 7/10 | 10/10 |
| Box in the middle (must stop) | **0/10: hit the box every time** | 10/10, stopped 0.19 m short |
| Slalom, 2 boxes 2.5 m apart | 3/10 | 10/10 |

![Simulation retune](zhenling/figures/Z5_sim_retune_previous_vs_current.png)

*Each panel one case from above: orange previous, green retuned, × stop requested. `straight_centre`: previous hit the
box 10/10; retuned stopped 0.19 m short.*

- **Can't conclude:** absolute gaps or pass rates (kinematic car, ideal lidar, car size and braking not measured). So
  the stops were checked on real data (3.4).

### 3.4 On the car (S4)

**E1: S-bend lane fit flips (20 Oct).** On the S-bend the fitted curvature jumped between left and right and the car
ran off. Replaying the 28 steps with cone range ∞ / 3.0 / 3.5 m: wrong-sign curvature in 18/28 steps with no limit,
0/28 with 3.5 m. 3.0 m gave 1/28 but only ≈ 12 cones per step and a false stop next to a box. → range limit 3.5 m.

**E2: braking travel, measured from the lidar.** Correcting for the scan's age,

$$t_b = \frac{d_{brake}}{v} = \frac{(d_{scan} - v\,t_{scan}) - d_{final}}{v}$$

| Drive | $v$ | $d_{scan}$ | $t_{scan}$ | $d_{final}$ | $t_b$ |
| --- | --- | --- | --- | --- | --- |
| 22 Oct 15-42-42 | 0.50 m/s | 0.64 m | 0.27 s | 0.30 m | 0.40 s |
| 22 Oct 17-59-26 | 0.50 m/s | 0.53 m | 0.11 s | 0.25 m | 0.46 s |
| 22 Oct 18-00-23 | 0.50 m/s | 0.64 m | 0.26 s | 0.33 m | 0.37 s |
| 22 Oct 18-03-10 | 0.45 m/s | 0.51 m | 0.17 s | 0.21 m | 0.51 s |
| 26 Oct 14-10-54 | 0.60 m/s | 0.69 m | 0.23 s | 0.23 m | 0.53 s |

![Obstacle stop](zhenling/figures/Z6_obstacle_stop_budget_real_vs_sim.png)

*(a) Stop distance at 0.5 m/s: scan age + braking + margin = 0.70 m. (b) Real stops: brake at 0.51–0.64 m readings,
stopped 0.21–0.33 m short. (c) Simulated braking-error tolerance per setting; stars are the real stops.*

- Real $t_b$ = 0.37–0.53 s, below the cautious 0.8 s, so the car always kept 0.21–0.33 m. The brake came late (3
  confirmations, lidar updating about every second step), worth 0.1–0.2 m, absorbed by the margin. Ignoring scan age
  would have counted sensor lag as braking (0.77 instead of 0.53 s on 26 Oct). Odometry claimed 1.1–1.25 m of braking;
  the lidar showed 0.32 m — wheel speed holds its last value near standstill.

**E3: passing boxes (26 Oct, run `2026-10-26_14-10-54`, video).** Passed a box on the right then one on the left
(1.4 m apart) without stopping, then stopped for a middle box whose gaps (0.31 m, 0.12 m) were both under the 0.40 m
needed: braked at a 0.69 m reading, stopped 0.23 m short.

**E4: the local map steadies the lane estimate (22 Oct drives).**

![Local map replay](zhenling/figures/Z10_local_map_real_drives.png)

*(b) Lateral error from the batch alone (orange) jumps up to 0.57 m; with recalled cones (blue) it is steady. (c) The
95th-percentile jump falls by half or more in all four drives.*

**Limits:** car size, steering after the trim change and braking on a stand aren't measured; static obstacles only;
two boxes on opposite sides < ≈ 1.2 m apart in a 0.8–1.1 m lane need more curvature than the car has, so it stops.
