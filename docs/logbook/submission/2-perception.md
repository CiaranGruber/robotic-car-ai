## Part 2: Perception

### 2.1 Shared: from sensors to the lane and the obstacles

![Perception in the system](zhenling/figures/Z1_where_my_parts_fit.png)

*One step per cone batch. Cones → filter (2.3) → lane estimate (2.2) → MPC (3.1). Lidar points become obstacles only
once the lane is fitted (2.3); every output reaches the car through the MPC and car control.*

| Interface | Carries | Constraint |
| --- | --- | --- |
| Cones in | (x, y) in `base_link`, colour, confidence | ≈ 10 Hz, 0.2 s old; duplicates, floating detections |
| Lidar in | points in `base_link` | New scan every 0.2–0.3 s; plane ≈ 0.12 m up, so it hits cones |
| `LaneDetection` → MPC, obstacles, turner | $e_y$ (m), $e_\psi$ (rad), $\kappa_c$ (1/m), width, centre arc | `NO_LANE` stops the car; camera must point straight ahead |
| `LanePath` → MPC | offset $d(s)$ from the centre to follow | Continuous curvature |
| Obstacle distance → stop | m from the car's front | Front taken 0.30 m ahead of `base_link` |

**Shared consequence:** the lane estimate is the frame everything else uses. A 0.35 m jump in it moved an obstacle
into the path and stopped car 20 (4.2); a camera yaw δ shifts the car ≈ 1.2 cm per degree and breaks turns (2.2).

### 2.2 Thanh Long Nguyen — lane estimation, camera pan and the car simulator

**Built:** `LaneDetector` and its `LaneDetection` output, AUTO colour sides and width memory, the pan hold, and a
whole-system simulator. Commits `8b71f8b` `f96ee66` `f49333d` `3319051` (#19). Car 20 drove on it 9 Oct; in every run
since 14 Oct.

**S2: model and fit.**

![Lane estimation](long/figures/02_lane_estimation.png)

*Steps of `LaneDetector.detect` (top) and the model for a lane curving left (bottom).*

- Both edges are concentric arcs about one reference arc (heading ψ, curvature κ) at offsets $d_L$, $d_R$; signed
  distance of cone j, with $s_j = x_j\cos\psi + y_j\sin\psi$, $\ell_j = -x_j\sin\psi + y_j\cos\psi$:

$$D_j = \frac{2\ell_j - \kappa(s_j^2 + \ell_j^2)}{1 + \sqrt{1 - \kappa\,(2\ell_j - \kappa(s_j^2 + \ell_j^2))}}, \qquad \min_\theta \sum_j \rho\big(D_j - d_{edge(j)}\big) + \rho\big((\sigma/\kappa_{typ})\,\kappa\big)$$

- θ = (ψ, κ, $d_L$, $d_R$); soft-L1 loss ρ (0.05 m); curvature prior σ = 0.05 m, κ_typ = 0.5 1/m. Cones > 0.15 m off
  their edge are dropped and refitted (≤ 2 times).
- Outputs $w = d_L - d_R$, $e_y = -(d_L+d_R)/2$, $e_\psi = -\psi$; `NO_LANE` unless 0.3 ≤ w ≤ 3.0 m.
- **AUTO + width memory:** the edge further left is the left edge, whatever its colour; $w_{mem} \leftarrow w_{mem} + 0.2(w - w_{mem})$, so one edge still gives the centre.
- **Camera pan:** the TF assumes zero pan, so a camera turned by δ gives $e_{\psi,meas} = e_{\psi,true} + \delta$ → hold the pan at 0.

**S3: is fitting the edges together better? How accurate is it?** 3 cm noise, 200 seeds: with a 3-cone row,
0.69° vs 3.03° heading RMS (equal at 12 cones). Curved mode: $e_y$ 1.9–3.4 cm for radius ≥ 1.5 m (R1b allows 10 cm).

![Fit accuracy](long/figures/04_fit_accuracy.png)

*Drawn 0.8 m lanes, 30 seeds: straight mode (red) fails on any curve; curved mode (green) stays low until the
tightest bends → straight for Stage 1, curved for curved roads.*

- **Simulator (27 Oct):** real `MovementPolicy` + car 20's maps, camera 86°, 0.4–2.5 m, 3 cm noise, 0.2 s. 76/90 runs
  pass on six tracks, no contacts; junctions least robust (17/25 with 20% missed cones). Uncorrected yaw ±4° fails a
  whole turn direction → **the pan hold is a precondition for turns**. Camera optimistic (20 vs 14 real cones per
  step) → pass rates are an upper bound.

**S4: why did the lane look turned on real drives? (recordings, 6 Oct)** Steering was 0, yet every drive read
$e_\psi$ = +6° to +11°. A real heading would make the car drift; rotated cones wouldn't.

| Drive | Median $e_\psi$ | Drift if real | Measured drift |
| --- | --- | --- | --- |
| straight_even | +9.6° | +0.16 m | −0.05 m |
| messy_cone | +10.9° | +0.19 m | −0.03 m |
| outlier_cone | +8.0° | +0.38 m | −0.01 m |
| long_track | +6.5° | +0.32 m | −0.07 m |

→ the camera was turned, not the car (pan never commanded). **Pan held at 0 from 10 Oct.** Same drives: width
0.74–0.84 m, edge RMS 2.8–4.5 cm, lane seen to ≈ 2 m → cone range cut to 2.5 m. **Limits:** one arc can't fit an
S-bend; a wrong-colour cone in a short row can stop the car; no ground-truth $e_y$ on the car.

### 2.3 Zhenling Zhang — cone filter, obstacles, avoidance and local map

**Built:** cone filter, lidar obstacle detection in lane coordinates, obstacle stop and brake, avoidance planner,
local map, closed-loop obstacle simulation. **PRs** #11 #19 #21 #28 #33 #34. All five ran on cars; the 26 Oct drive
is the best video.

**S2.**
- **Cone filter:** keep $0 < x$, $|y| \le 2.0$ m, range ≤ 3.5 m, $-0.15 \le z \le 0.25$ m, confidence ≥ 0.5; merge
  same-colour cones < 0.10 m apart.
  - *Why range ≤ 3.5 m:* the lane fit describes each edge with one constant-curvature arc, so it can only follow the
    lane a short way ahead. Far cones from the next bend, such as the second half of an S, would pull the fit near the
    car the wrong way. A much shorter range leaves too few cones on each edge, and the fit jumps from step to step.
- **Obstacle detection:** lidar points arrive in `base_link` as $(x, y)$ and are moved into lane coordinates on the
  fitted centre arc (radius $R = 1/\kappa$; both edges are concentric with it): $s = R\,\theta - 0.30$ m is the
  distance along the arc from the car's front, and $l = R - \rho$ is the offset across it, where $\theta$ is the angle
  swept about the arc's centre and $\rho$ the point's distance from that centre (left bend; signs flip for a right
  bend). Both edges then lie at $l = \pm w/2$ for every $s$, so a bend becomes a straight strip.
  - *Range:* only points with $0 < s \le 3.5$ m count.
  - *Inside the lane:* $|l| < w/2 - 0.15$ m, so returns from the cones are ignored.
  - *Objects:* neighbouring returns less than 0.15 m apart form one object, which needs at least 3 returns.

![Obstacle detection](zhenling/figures/Z4_obstacle_detection_real_scan.png)

*Real scan, 22 Oct. (a) Car frame: the lidar also hits the cones, and a straight corridor misses object 1 on this
bend. (b) Lane coordinates: the edges are straight lines; returns within 3.5 m and inside the edge margin form two
objects.*

- **Obstacle stop:** seen 3 steps in a row, within

$$d_{stop} = (t_b + t_{scan})\,\max(v_{target}, v_{measured}) + m, \qquad t_b = 0.8\ \text{s},\ m = 0.2\ \text{m}\ \ (0.70\ \text{m at } 0.5\ \text{m/s})$$

| Term | Value | Meaning |
| --- | --- | --- |
| $t_{scan}\,v$ | $t_{scan}$ = each scan's own age | Distance travelled since the scan was taken |
| $t_b\,v$ | $t_b$ = 0.8 s per m/s | Braking travel. Chosen in simulation: no hit for true braking up to 1.0 s per m/s |
| $\max(v_{target}, v_{measured})$ | larger of the two | Covers a car faster than its target, and a car still speeding up |
| $m$ | 0.2 m | Gap left between the car's front and the obstacle after stopping |

- **Avoidance planner:** plans one avoidance path past all known obstacles, as an offset $d(s)$ from the lane centre
  in lane coordinates, re-planned every step. The MPC follows it.

*1. Path structure.* The path is a chain of segments: straight **holds** at a fixed offset (beside an obstacle, or on
the usual line) and **quintic moves** between two offsets. Obstacles far apart get a hold beside each; for close ones,
one move runs straight into the next.

*2. Quintic move.* A move over $[s_i, s_i + L]$, with $t = s - s_i \in [0, L]$, is

$$d(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4 + c_5 t^5$$

It leaves the start state $(d_0, d_0', d_0'')$ and reaches the end state $(d_1, d_1', d_1'')$: offset, slope and
curvature. These six conditions fix the six coefficients. The three at $t = 0$ give

$$c_0 = d_0, \qquad c_1 = d_0', \qquad c_2 = \tfrac{1}{2}\,d_0''$$

With what is left to reach at $t = L$ after those terms,

$$\Delta d = d_1 - (c_0 + c_1 L + c_2 L^2), \qquad \Delta v = d_1' - (c_1 + 2 c_2 L), \qquad \Delta a = d_1'' - 2 c_2$$

the three at $t = L$ give

$$c_3 = \frac{10\,\Delta d - 4\,\Delta v\,L + \tfrac{1}{2}\Delta a\,L^2}{L^3}, \qquad c_4 = \frac{-15\,\Delta d + 7\,\Delta v\,L - \Delta a\,L^2}{L^4}, \qquad c_5 = \frac{6\,\Delta d - 3\,\Delta v\,L + \tfrac{1}{2}\Delta a\,L^2}{L^5}$$

So the coefficients depend on both end states and on $L$, and offset, slope and curvature are continuous at every
join. Every move ends on a hold ($d_1' = d_1'' = 0$). Only the first move of a step may start mid-turn, from where the
previous step's path is now; the others start on a hold too.

*Move length.* For a move between two holds with offset change $D = d_1 - d_0$, the peak curvature is
$\frac{10}{\sqrt3}\frac{|D|}{L^2}$, so the shortest move within the curvature limit is
$L_{min} = \sqrt{\frac{10}{\sqrt3}\frac{|D|}{\kappa_{max}}}$. A move that starts mid-turn first tries to end as the
car's front reaches the obstacle's stretch, then the lengths $L_{min} \times \{1, 1.25, 1.5, 2, 3\}$, and takes the
first whose peak curvature stays within $\kappa_{max}$.

*3. Choosing the combination.*

- **Joining:** before the first group, hold the current line, then take the shortest move, arriving as the car's
  front reaches the stretch. Between groups, one of four ways: keep the offset; a move centred between them; a move
  spread over the whole gap; or back to the usual line and out again. After the last group, one move back to the
  usual line at the earliest point where the body keeps its clearances.
- **Check and search:** each candidate is checked every 0.05 m with the car's body, a 0.25 × 0.60 m rectangle turned
  along the path. The search goes group by group, keeping the 6 cheapest partial paths, for up to 3 groups.
- **Ranking**, in order: obstacles passed (nearest first) → remaining distance to passing the next → already beside
  the obstacles before $d_{stop}$

![Avoidance planner](zhenling/figures/Z7_avoidance_planner_candidates.png)

*Simulation, 0.8 m lane, box right then left. (a) Grey: 22 candidates; blue: chosen line, body checked along it.
(b) Curvature continuous, inside 0.45 1/m (car's left limit 0.48).*

- **Local map:** dead-reckoned pose, nearest-cone matching (0.12 m), half a rigid fit per step; recalled cones only
  added once the batch already gives a lane.

**S3: retune against needless stops and hits (21 Oct).** Closed loop, 10 seeds per case, braking 0.6 s per m/s.
Narrower corridor (0.175 m), wider edge margin (0.15 m), 3 confirmations, braking travel in $d_{stop}$:

| Case | Previous | Retuned |
| --- | --- | --- |
| Box right, 0.5 m gap | 3/10 (7 needless stops) | 10/10 |
| Box in the middle (must stop) | **0/10: hit every time** | 10/10, 0.19 m short |
| Slalom, 2 boxes 2.5 m apart | 3/10 | 10/10 |

**S4: braking measured from the lidar (22, 26 Oct).** Correcting for scan age,
$t_b = \big((d_{scan} - v\,t_{scan}) - d_{final}\big)/v$.

![Obstacle stop](zhenling/figures/Z6_obstacle_stop_budget_real_vs_sim.png)

*(a) Stop distance at 0.5 m/s = scan age + braking + margin = 0.70 m. (b) 5 real stops: braked at 0.51–0.64 m
readings, stopped 0.21–0.33 m short. (c) Simulated tolerance per setting; stars are the real stops.*

- Real $t_b$ 0.37–0.53 s < the cautious 0.8 s → always kept 0.21–0.33 m. Odometry claimed 1.1 m of braking; the
  lidar showed 0.32 m (wheel speed holds its last value).
- **26 Oct (best video):** passed a box right, then left (1.4 m apart), then stopped for a middle box whose gaps (0.31,
  0.12 m) were under the 0.40 m needed: braked at 0.69 m, stopped 0.23 m short.
- S-bend (20 Oct): curvature flipped sign in 18/28 steps → 0/28 with the 3.5 m range. Local map halved the p95 lane
  jump in all four 22 Oct drives. **Limits:** static obstacles only.
