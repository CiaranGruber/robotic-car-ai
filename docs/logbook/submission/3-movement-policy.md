## Part 3: Movement policy (MPC)

### 3.1 Shared: the MPC all three members build on

![MPC in the system](peiyu/figures/01_system.png)

*Lane (2.2) in, (speed, curvature) to car control (4.1) out, once per cone batch within 20 ms. Markers (3.3) and the
turner (3.3, 3.4) switch what the MPC tracks.*

**Model** (Peiyu). Lateral error $e$, heading error ψ, curvature κ; the car moves $d = v\Delta t$ per step,
Δt = 0.1 s:

$$e_{k+1} = e_k + d\,\psi_k + \tfrac{d^2}{2}(\kappa_k - \kappa^{path}_k), \qquad \psi_{k+1} = \psi_k + d\,(\kappa_k - \kappa^{path}_k)$$

$$\min_{\kappa_0..\kappa_{N-1}} \sum_k q_e\,(e_k - e^{line}_k)^2 + q_\psi\,\psi_k^2 + q_\kappa(\kappa_k - \kappa^{path}_k)^2 + q_\Delta(\kappa_k - \kappa_{k-1})^2 \quad\text{s.t.}\quad \kappa_{min} \le \kappa_k \le \kappa_{max}$$

- **Delay compensation:** predict forward by $d_0 = v\cdot\text{age}$ (≈ 0.2 s) with the previous command; send only
  the first curvature.
- **Bounded least squares.** The fixed-speed MPC precomputes $M$ in $\lVert Mu - c\rVert^2$. With measured wheel
  speed, the MPC rebuilds the speed-dependent terms of $M$ at each solve and reuses the fixed weights and curvature
  penalties. The $4N$ cap in the BVLS solver limits iterations. R6 uses measured step time.

| Setting | Stage 1 design (3.2) | Car 20, `dev` (3.3) |
| --- | --- | --- |
| N | 20 (1 m at 0.5 m/s) | 12 |
| $q_e / q_\psi / q_\kappa / q_\Delta$ | 3 / 1 / 0.1 / 3 | 10 / 1 / 0.1 / 2 |
| κ bounds | ±3.0 1/m | −0.92 to +0.48 1/m (measured, 4.1) |
| $e^{line}$ | 0 (centre) | lane side offset (3.3) |
| $v$ in $d$ | target | measured wheel speed (3.4) |

| Constraint | Handled as | Why |
| --- | --- | --- |
| Curvature limit | hard bound | the real steering limit; bounds always feasible for `bvls` |
| Steering rate | cost $q_\Delta$ | a rate limit needs a general QP; the cost keeps R4 at 0.16 |
| Lane edges | left out | a hard state limit is infeasible when noise puts the car near an edge |
| Speed | not a variable | would make the model bilinear; car control holds speed |

### 3.2 Peiyu Wang — MPC base, MPC vs RL, virtual debug, keeping the code and cars working

**Built:** the MPC framework and first car run, the Stage 1 design (pure pursuit, MPC grid, RL, milestone M1), lane-end
and lost-lane stops, car models, the Foxglove replay tool, car bring-up, merges and reviews. **PRs** #1 #8 #16 #25;
reviewed and merged #27.

**1. The MPC base (S2, S4).** *Before:* no movement policy. I wrote the MPC framework (#1: the model and solver in 3.1)
and the first car run (#8). The team built on it: Toby tuned it for car 20 and added lane side and turns (3.3), Yuhao
added measured speed and stopping (3.4).
- **Lane end and lost lane (S2):** no lane and the farthest cone ≤ 1.5 m ahead → the rows ended: follow the last plan,
  request the stop at roll-out distance (`stop_distance_per_speed_s` × v) from 0.25 m past that cone. No lane
  otherwise → half speed ≤ 0.5 s, then a latched stop (R7: 0.6 s with the 0.2 s age). *Before:* the car stopped
  0.38–0.91 m short (the last pair leaves the 80° view ≈ 0.6 m early, so the lane looked lost); *after:* 0.21–0.24 m
  past (R5).
- **Time budget (S4, car 20, 6 Oct):** batches every 0.1 s, latency 0.197 s → 20 ms per step allowed. Solve N = 20:
  0.46 ms median, 0.53 ms p99; N = 60 still 5.96 ms → the horizon is limited by camera range, not CPU.

<p><img src="peiyu/slide/tiles/mpc_base.png" alt="The MPC base and who built on it" width="49%" style="display:inline-block; margin:0"> <img src="peiyu/slide/tiles/car_timing.png" alt="MPC solve time on car 20" width="44%" style="display:inline-block; margin:0"></p>

*Left: the MPC base and who built on it (blue: mine). Right: MPC solve time on car 20's computer, 300 solves per
horizon, against the 20 ms budget.*

**2. Deciding MPC or RL (S3).** *Why decide:* a learned policy might track better, but it had to stay smooth on lanes
it was never tuned on. *How:* every design ran in the same harness with the same cone filter and lane-end rule; only
the steering differed. First the MPC settings: 324 settings × 24 runs (12 cases, ± 0.03 m noise).

![Design front](peiyu/figures/04_design_front.png)

*Tracking against roughness; Pareto front of passing settings; star = chosen (N 20, 3/1/0.1/3): the smoothest
passing. Pure pursuit tracks 3 mm better but steers 2.3× rougher.*

**RL:** PPO replaces only the solver; bounded penalties $P(x,s) = 1 - e^{-(x/s)^2}$, reward
$r = 1 - aP(y,0.1) - bP(\psi,0.1) - cP(\dot\kappa,1.0)$. Its first reward was exploited (unbounded penalties made
leaving the lane best); with bounded penalties it drove the lane but still weaved. **M1 (unseen cases U1–U3, opened
once, rule fixed before):** RL failed R4 on U1 (1.51) and U2 (1.59); the MPC passed all (U2: 0.51 vs 0.80) →
***MPC chosen***, because it stayed smooth on lanes it was never tuned on.

<p><img src="peiyu/slide/tiles/rl_steering.png" alt="RL and MPC steering on U2" width="49%" style="display:inline-block; margin:0"> <img src="peiyu/slide/tiles/lane_end.png" alt="T1 lane end before and with the rule" width="49%" style="display:inline-block; margin:0"></p>

*Left: the steering each design asks for on U2 (rerun at `47f8838` with `rl_models/balanced_1M.zip`; data in
`peiyu/figures/m1_u2_steering.json`). RL turns hard right, then hard left; the MPC makes one smooth correction.
Right: the end of T1 in the simulator's own top view (`stage1_scenarios.plot_result`), before the lane-end rule
(`01a47a0`, 0.50 m short, FAIL) and with it (`9a7f9d8`, 0.20 m past, PASS; neither with the 0.2 s age).*

**3. Virtual debug (S4, S5).** *Before:* when a car run went wrong, the team had the car's path and text logs: watch
where it drove, then guess why. *Built:* every step's observations and actions are logged; `scripts/mcap_export.py`
replays a data log through the policy into Foxglove, step by step, with the cones the car saw, the lane it found and
the MPC's plan, and the car publishes the same drawings live (#25). *After:* problems are found by replaying runs
instead of rerunning the car; Toby's turn checks read `/plan/intersection` in Foxglove, and Long's on-car live viewer
draws the same markers (their full sections). A replay runs the current code, so car runs are judged by their logged actions (on the run below, the
replay chose the car's own actions on every step). The camera pan prototype (`230c75b`, S3) used the same export:
turning the camera into tight bends keeps the lane in view only when the camera's angle at each image is known; with
the latest command instead, it lost an oval the fixed camera finished, so the pan stays at 0.

<p><img src="peiyu/slide/tiles/car20_replay.png" alt="Car 20 run replayed in Foxglove" width="49%" style="display:inline-block; margin:0"> <img src="peiyu/slide/tiles/foxglove_pan.png" alt="Camera pan prototype in Foxglove" width="49%" style="display:inline-block; margin:0"></p>

*Left: a car 20 run replayed in Foxglove: the car has drifted right and the MPC's plan steers it back. Right: the
camera pan prototype in Foxglove (hairpin, ideal pan): the camera's view turned into the bend and the lane the policy
found.*

**4. Keeping the code and cars working (S1, S4).**
- **Merge conflicts:** eight people pushed to `dev` at once. Chris's controller (#17), Toby's calibration (#15), the
  first car run (#8) and the Stage 1 design (#16) landed together; I resolved six merges (largest 22 conflict hunks,
  `fb2442b`): `dev`'s shared code kept, the MPC features ported onto it, tests re-run; lab weights 10/1/0.1/2 taken,
  since A4's with #15's change weight left a 3 m radius curve.
- **Code review (#27, Toby's lane side):** found and fixed a duplicate setting, a reset that kept the last run's lane
  side, a first step planning from the previous run's line, a missing test with avoidance and a stale docstring;
  merged `dev` in, re-ran the wide-track results (CI 560 passed), then merged.
- **Every car differs:** with car 20's values car 22 drove 1.4 m/s for 0.5; car 22's values stall the car 20 model →
  car models and one setting checked on every model (#18, waiting for review). Car 22 stopped 0.8 s after losing the
  lane; its steering stayed biased (unsolved).
- **Cars that would not start:** car sessions kept starting with a car that would not run, each time for a different
  reason, and each cost restarts, log reading and redeploying the policy: the cone detector crashed under OpenCV 5
  (`KeyError 16`) → `publish_rgb_image: false` (#24); the OAK-D dropped off USB → restart only the detector; DREAM's
  `runtime_runner` processes at 70–80% CPU each (reported upstream); the policy running but the car not moving
  (wrong `left_colour`, stray cones → moved the cones); a flat motor battery (drive applied, wheel speed 0 → check the
  battery first). Car 20 was shared, so every check was read-only first and its source was backed up before deploying.

<p><img src="peiyu/slide/tiles/car_models.png" alt="Every car differs" width="58%"></p>

*Speed on T1 at 0.5 m/s through `CarController` and each measured car model (`45eb015`, data in
`peiyu/figures/car_models_speed.json`): car 22's values stall car 20; car 20's drive car 22 3× too fast; one setting
(#18) holds both.*

### 3.3 Toby Lee — test harness, MPC tuning, lane side, marker actions and intersection turns

**Built:** Stage 1 test cases and harness (#6 #10), car 20 MPC calibration (#15), lane side (#27 #30), marker
actions and the intersection turner (#35, with Yuhao), the turn simulation suite and the left-turn fix
(`fix/left-turn-steering-and-obstacles`, not yet merged); four car sessions (cars 20, 21, 13, 14).

**S2: how each part works.**
- **Test harness** (`tests/policy/stage1_scenarios.py`). Twelve tuning cases T1–T12: start offset ±0.15 m and
  heading up to 10°; 0.8, 1.0 and 1.2 m lanes; detector noise, a missed cone, an extra object, 3 m range, +0.2 s
  delay, a wet floor with a 20% heavier car, and a camera blackout (scored on R7). U1–U3 are held back for M1. Any
  `run_policy()` design (MPC, RL, pure pursuit) drives dream-gym closed loop; ground truth only places the car,
  makes the disturbances and scores R1–R7. `--car` sends the command through the real `CarController` and a car model.
- **Lane side** (`policy.py`). The MPC tracks a line offset $o$ from the lane centre instead of the centre. With
  edge clearance $c$ and lane width $w$, the allowed range and the target are
$$|o| \le \tfrac{w}{2} - c, \qquad o^{*} = \begin{cases} +(\tfrac{w}{2}-c) & \text{LEFT} \\ 0 & \text{CENTRE} \\ -(\tfrac{w}{2}-c) & \text{RIGHT}\end{cases}, \qquad o_k = o_{k-1} + \mathrm{clip}\big(o^{*} - o_{k-1},\ \pm\,0.3\,\Delta s_k\big)$$
  On a curve $o$ is also kept within $0.5/|\kappa|$ of the inside. The MPC sees the offset line as its path:
  $e' = e - o$, $\psi' = \psi$, $\kappa' = \kappa/(1 - \kappa o)$ (concentric). $c$ = 0.28 m gives ±0.12 m in a 0.8 m
  lane and ±0.22 m in a 1 m lane.
- **Markers → actions** (`markers.py`). Each step: keep posed markers with $x > 0$ and range ≤ 2.0 m, take the
  nearest, count consecutive steps it stays nearest, fire once at 2. ID → command as in 0.4. Keep-side commands set
  `lane_side` until the next side marker; turn and straight commands start the turner with the marker's position; an
  unmapped ID latches a braking stop. During a turn, new markers are ignored except STOP.
- **Intersection turn: APPROACH → ARC → EXIT** (`intersection.py`). The entry is placed 1 m past the marker along
  the approach centre line. The car moves to the outside line (0.25 m from the outside edge) by pure pursuit,
  $\kappa = 2y_L/(x_L^2+y_L^2)$ to a point 0.5 m ahead, at 0.3 m/s. The radius is $R = \max(R_{cfg}, 1/\kappa_{max})$.
  The arc is a circle tangent to the car's heading; it starts at $s$ (along the approach, from the entry), which must
  lie in the window below, nearest to the start that ends the arc on the exit centre line, $s^{*} = w_{cross}/2 - R$.
  After ≥ 60° of turn, a lane with $|e| \le w/2 - 0.1$ m and $|\psi| \le 20°$ hands back to the MPC; past 85° the car
  follows the exit line and stops if no lane appears within 1.5 m.

$$\underbrace{-\sqrt{(R-b-r_c-m)^2 - (R-a-w/2)^2}}_{\text{inner side clears the inside corner cone}} \;\le\; s \;\le\; \underbrace{w_{cross} - r_c - m - \sqrt{f^2+(R+b)^2}}_{\text{outer front corner clears the far row}}$$

  With the arc centre at $(s,\ a - R)$ for a right turn (mirror for left), the inner side of the body sweeps radius
  $R - b$, so the corner cone at $(0, -w/2)$ must lie within $R - b - r_c - m$ of the centre: this is the lower bound.
  The outer front corner sweeps $\sqrt{f^2+(R+b)^2}$ and must stay $r_c + m$ short of the far row at $x = w_{cross}$:
  this is the upper bound. Moving aside (larger $a$) widens the window; a larger $R$ narrows it. Values: car half
  width $b$ = 0.125 m, base_link to front $f$ = 0.30 m, cone radius $r_c$ = 0.05 m, margin $m$ = 0.03 m,
  $a = w/2 - 0.25$ m. An empty window, or the car already past the upper bound, is a braking stop (the "geometry stop").

| Window width (start can vary by) | 0.8 m roads | 1.0 m roads |
| --- | --- | --- |
| Right, R = 1.2 m, from the centre ($a$ = 0) | none (−0.05 m) | — |
| Right, R = 1.2 m, moved aside | 0.12 m | 0.45 m |
| Left, R = 2.08 m (0.48 1/m) | none (−0.42 m) | 0.014 m |
| Left, R = 1.1 m (0.91 1/m, the fix) | 0.17 m | 0.48 m |

![Turn geometry](toby/figures/08_turn_geometry.png)

*Right turn, 0.8 m roads, R = 1.2 m: from the centre no start clears both; after moving 0.15 m outside, one does.*

**S3.**
- **MPC for car 20 (12 Oct).** Not a new controller: settings for Peiyu's MPC. Test: `curved_lanes.py`, a straight,
  90° curves of 1.5 (right only), 2 and 3 m radius and an S-bend, × 3 camera yaws (±3°) × 4 noise seeds = 84 runs,
  0.8 m lane, 0.5 m/s, 0.2 s delay, measured noise. Kept = no cone touched, body inside, reaches the last pair.

| Setting | Car 20's first run | Chosen (`dev`) | Why |
| --- | --- | --- | --- |
| Curvature limits | ±0.5 1/m | +0.48 / −0.92 1/m | Chris's measured reach; ±0.5 threw away right steering |
| $q_\psi$ (heading) | 5 | 1 | 3 and 5 lost curves: the car lined up with the lane and drifted from its centre |
| $q_\Delta$ (change) | 1 | 2 | 0.27 → 0.20 1/(m s) for the same lanes kept; 3 raised the end error (0.18 → 0.24 m) |
| Result | 12/84 kept, 0.71 1/(m s) | **84/84, 0.20 1/(m s)** | 150 straight synthetic tracks: 97% (91% before) |

  `mpc_tuning.py` checks the choice on straights: 5 horizons × 3 × 3 × 4 weights = 180 options, chosen on training
  tracks by pass rate then accuracy + smoothness, compared on separate validation tracks. Nothing is learned: the MPC
  solves its optimisation online every step, and only these settings were chosen, in simulation.
- **Lane side (26 Oct):** c = 0.40 m left 0 cm of shift in a 0.8 m lane → c = 0.28 m: 40/40, side error ≤ 0.017 m.
  Offset rate 0.3 m/m: 0.2 had not finished crossing 1.4 m after the marker; 0.5 hit cones in wide lanes.
- **Why do left turns fail on car 20? (1–2 Nov).** The turn simulation suite (`marker_simulation.py`) runs the whole
  `MovementPolicy` closed loop (cones, markers, lidar, IMU, the car's response) through left, right and straight
  crossings at four-way and T intersections in 0.8–1.5 m roads, over noise seeds, and reports pass, stop or contact.
  `--set` changes any setting, which is how steering reach, camera yaw, latency and dropout were swept.

![Root cause](toby/figures/06_root_cause_bars.png)

*Same pattern as the car: left 2/40, right 35/40. At 0.48 1/m the left arc (R = 2.08 m) has a 1.4 cm window in a 1 m
crossing and none in 0.8 m roads (table above). All cases 57% → 99% with the changes below.*

| Change (branch `fix/left-turn-steering-and-obstacles`) | Problem it fixes | Result (sim) |
| --- | --- | --- |
| Steering interval ±0.5 → ±0.8, steering map × 0.625 (Chris, c8988e8) | left saturates beyond −0.75 of ±0.5; the RC turns ≈ 2× further | more left reach on the same map |
| `max_curvature_per_m` 0.48 → 0.91 (left R 2.08 → 1.1 m) | arc too wide for the crossing | left 39/40 in 0.8–1 m roads |
| `intersection.lookahead_m` 0.5 → 0.8 | sharper left reach swung the move-aside too hard | 43/48 vs 33/48 |
| Ignore lidar within 0.15 m of a seen marker; avoid only after 3 of 5 sightings; no new avoidance line at an intersection | marker stands read as obstacles (car 13, 27 Oct) | phantom avoidance 29% → 4% |
| Ignore lidar within 0.12 m of a known cone while turning | the crossing's cones stopped 60% of turns with 3 cm misplacement | 40% → 68% |

  0.91 1/m is not measured on the ground: re-measure with `steering_test` before relying on it (4.2).
- **Sensitivity:** ±4° camera yaw costs a whole turn direction; robust to doubled latency (92%) and 25% dropout (96%).

**S4.**

| Quantity (from 4 recorded drives, 12 Oct) | Assumed | Measured | Used in sim |
| --- | --- | --- | --- |
| Lane width / cone noise | 1.0 m / 0.01 m | 0.77 m / 0.040 m | 0.8 m / 0.03 m |
| Latency / range | 0 s / 4 m | 0.21 s / 2.54 m | 0.2 s / 2.5 m |

| Link in the marker chain | Result |
| --- | --- |
| Lane side (car 20, 20 Oct) | `lane_side` LEFT only changed: car kept left (by eye) |
| Camera image to detector | 0 marker batches in 9 runs → `car_management.sh --aruco` |
| Command (car 13, 27 Oct) | "Marker 14: KEEP_RIGHT", 6 times |
| Phantom obstacles (car 13) | marker stands planned 16 avoidance lines with no box → filters: 29% → 4% in sim |
| Full turn (2 Nov) | not shown: wrong branch, then no `--aruco`; time ran out |

### 3.4 Yuhao Liu — measured-speed MPC, cone memory and junction planning

**Built** measured-speed prediction and solver-failure stops (#14), an offline final-pair estimator, a cone-memory
U-turn and an experimental cross/T-junction navigator. In Toby's default turner, added grouped marker IDs, earlier
turn preparation, heading-aware arc admission and a persistent stop latch (#35, #36). Diagnosed car 20's left-turn failure.

**S2.** Measured wheel speed sets the prediction step distance $d = v\Delta t$ and the delay projection
$d_\tau = v\tau$. A failed solve requests a stop. The default turner starts preparation on the outside of the lane
after confirmation and checks an arc tangent to the actual heading. At the 0.48 1/m left limit, the radius is
2.083 m. A rejected turn keeps its stop latched after the turn object clears.

- **Cone memory.** Wheel and gyro odometry reproject stored cones as the car moves. The U-turn controller holds its
  committed path through short camera dropouts and recomputes steering from the current pose.
- **Junction path finding.** The navigator queues the ArUco command and uses cone rows and current lidar to verify
  the requested exit. An unverified exit allows a finite approach ending before the junction. A closing row blocks
  commitment. The selected centre path feeds MPC until fresh exit geometry and heading alignment allow lane following.

**S3: does measured speed help?** 3 scenarios × 2 inputs × 5 seeds, 0.2 s delay; distance-weighted
$e_{RMS} = \sqrt{L^{-1}\int_0^L e(s)^2 ds}$:

| Scenario | Nominal speed | Measured speed | Reduction |
| --- | --- | --- | --- |
| Acceleration | 0.07158 m | 0.07072 m | 1.2% |
| Half achieved speed | 0.08314 m | 0.08173 m | 1.7% |
| Slow with dropouts | 0.07532 m | 0.07265 m | 3.5% |

- **Final-pair lane end (offline):** 20/20 when the pair is visible, but a hidden pair gives false ends → kept offline.
- **Cone-memory U-turn.** The [branch experiment](https://github.com/CiaranGruber/robotic-car-ai/commit/fd01806)
  completed 10/12 varied simulated layouts. Two triggered clearance stops. The planned 0.50 m radius and
  2.2 1/m left curvature limit were simulation assumptions.
- **Cross/T-junction navigation.** With the [experimental navigator](https://github.com/CiaranGruber/robotic-car-ai/commit/d59e812),
  20/22 cross and 22/24 dense open-exit T cases completed in simulation. All six dense blocked-exit cases stopped.
  The navigator assumes equal-width perpendicular roads and is disabled by default.
- **Stop latch (sim):** at 0.48 1/m, 6/6 runs on 1.0 m roads stop without contact; 6/6 on 1.5 m complete the left
  exit. Suite at `488ec30`: 837 passed.

#### S4. Car 20 left-turn diagnosis (30 Oct)

![Marker range](yuhao/figures/05_marker_range.png)

*All 41 posed ID-1 observations were 3.3–6.4 m away, outside the 2 m gate. The running detector used a 0.250 m marker
size though the print was 0.081 m (range ×3.09); restarting the policy hadn't reloaded it.*

- After restarting only the detector: ID 1 confirmed at 3.0 s, then a **geometry stop** at 5.4 s: 0.174 m outside
  vs 0.182 m needed, heading 4.6° outward. → prepare earlier, tangent admission; limits not relaxed. A zero command
  isn't a physical stop (wheels still at 0.50 m/s).
