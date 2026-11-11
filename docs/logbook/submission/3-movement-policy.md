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

**Built:** Stage 1 test cases and harness (#6 #10), car 20 MPC settings and lane offset (#15), lane side (#27 #30),
marker actions and the intersection turner (#35, with Yuhao), the turn simulation suite and the left-turn fix
(`fix/left-turn-steering-and-obstacles`, **not merged**). Car sessions on cars 20, 21, 13 and 14.

**Where markers and turns fit in the policy (S1).** Markers decide *which line* the MPC follows; the turner takes over only for the arc.

![Where my parts sit](toby/figures/01_system.png)

*Blue = my parts. In: lane (2.2), ArUco poses, lidar. Out: a lane side for the MPC (3.1), or (speed, curvature) from
the turner during a turn, to car control (4.1). Constraint from 4.1: car 20 steers +0.48 1/m left, −0.92 right.*

- **Requirements I tested against:** R1–R7 (0.1) for lane following; for turns, leave by the commanded road without
  touching a cone; otherwise stop.
- **Assumptions:** cones mark a lane 0.8–1.0 m wide; the marker pose is in `base_link`; the camera points straight
  ahead (pan held, 2.2). A wrong camera yaw or steering reach breaks turns first (S3).

**How the car picks its line and plans a turn (S2): test harness, lane side, markers, arc-start window.**

- **Test harness** (`stage1_scenarios.py`). 12 tuning cases T1–T12 (start offset and heading, 0.8–1.2 m lanes,
  noise, missed or extra cones, delay, a heavier car, a camera blackout); U1–U3 held back. Any `run_policy()`
  design (MPC, RL, pure pursuit) drives dream-gym closed loop and is scored on R1–R7.
- **Lane side** (`policy.py`). The MPC tracks a line offset $o$ from the centre. With edge clearance $c$ = 0.28 m:

$$|o| \le \tfrac{w}{2} - c, \qquad o^{*} = \pm(\tfrac{w}{2}-c)\ \text{or}\ 0, \qquad o_k = o_{k-1} + \mathrm{clip}\big(o^{*} - o_{k-1},\ \pm 0.3\,\Delta s_k\big)$$

  ±0.12 m in a 0.8 m lane, ±0.22 m in a 1 m lane. The MPC sees the offset line as its path:
  $e' = e - o$ and $\kappa' = \kappa/(1 - \kappa o)$ for the offset line.
- **Markers → actions** (`markers.py`). Nearest posed marker ahead within 2 m, fired after 2 steps in a row. Keep-side
  IDs set the lane side; turn and straight IDs start the turner; an unknown ID latches a stop.
- **Turner: APPROACH → ARC → EXIT** (`intersection.py`). Move to the outside line, then drive an arc of radius
  $R = \max(R_{cfg}, 1/\kappa_{max})$ by pure pursuit, $\kappa = 2y/(x^2+y^2)$. $R_{cfg}$: preferred radius in
  the config (1.1 m left, 1.2 m right); $\kappa_{max}$: the car's steering limit that way (car 20: 0.48 left,
  0.92 right 1/m), so the radius is never tighter than the car can drive; $(x, y)$: the pursuit target a lookahead ahead on the
  path (0.5 m on `dev`, 0.8 m on the fix branch), in the car's frame. Hand back to the MPC once the lane is
  found again after ≥ 60°. The arc may only start where the body clears both corners:

$$\underbrace{-\sqrt{(R-b-r_c-m)^2 - (R-a-w/2)^2}}_{\text{inner side clears the inside corner cone}} \;\le\; s \;\le\; \underbrace{w_{cross} - r_c - m - \sqrt{f^2+(R+b)^2}}_{\text{outer front corner clears the far row}}$$

  $s$: arc start along the approach (m, 0 = entry); $a$: how far the car moved aside; $w$, $w_{cross}$: approach
  and crossing road widths; $b$ = 0.125 m half width;
  $f$ = 0.30 m to the front; $r_c + m$ = 0.08 m cone plus margin. The code also corrects for heading error
  (Yuhao, 3.4). No valid $s$ → braking stop (the "geometry stop").

<img src="toby/figures/08_turn_geometry.png" alt="Turn geometry" style="width: 85%;">

| Window width | 0.8 m roads | 1.0 m roads |
| --- | --- | --- |
| Right, R = 1.2 m, from the centre / moved aside | none / 0.12 m | — / 0.45 m |
| Left, R = 2.08 m (0.48 1/m, car 20) | none | 0.014 m |
| Left, R = 1.1 m (0.91 1/m, the fix) | 0.17 m | 0.48 m |

*Moving aside widens the window; a large radius closes it. This is why left turns failed (S3).*

**Simulation: tuning the MPC for car 20, lane side, and why left turns failed (S3).**

- **Which MPC settings suit car 20? (12 Oct).** Settings for Peiyu's MPC, not a new controller. 84 curved-lane runs
  per setting (straight, 1.5–3 m radius curves, S-bend × 3 camera yaws × 4 seeds), 0.8 m lane, 0.2 s delay.

![MPC tuning](toby/figures/02_mpc_tuning.png)

| Changed | From → to | Why |
| --- | --- | --- |
| Curvature limits | ±0.5 → +0.48 / −0.92 1/m | Chris's measured reach; a 0.48 right limit kept 0/12 of the 1.5 m right curves, 0.92 kept 12/12 |
| Heading weight $q_\psi$ | 5 → 1 | 5 lined the car up with the lane but let it drift off centre |
| Change weight $q_\Delta$ | 1 → 2 | steering 0.27 → 0.20 1/(m s); 3 was smoother but ended 0.24 m off centre vs 0.18 |

  Result: 73 → 84 of 84 lanes kept, steering 0.71 → 0.20 1/(m s); 150 straight tracks 97% (91% `dev`). **Limit:**
  those 150 tracks were also used to choose, so 97% is not held-out. Nothing is learned; only settings are chosen.

- **Lane side (26 Oct).** c = 0.40 m left no room to move in a 0.8 m lane → 0.28 m: 40/40, side error ≤ 0.017 m.

<img src="toby/figures/04_lane_change_sim.png" alt="Lane change simulation" style="width: 80%;">

*Keep left, right, then centre. Marker IDs 13, 14, 16 are the simulation's mapping, not `dev`'s.*

- **Why do left turns fail on car 20? (1–2 Nov).** `marker_simulation.py` runs the whole policy closed loop through
  left, right and straight crossings (0.8–1.5 m roads, noise seeds); `--set` sweeps one setting at a time.

<img src="toby/figures/06_root_cause_bars.png" alt="Root cause" style="width: 62%;">

*Reproduces the car: left 2/40, right 35/40. Car evidence: Yuhao's 30 Oct geometry stop (3.4).*

| Fix (branch, not merged) | Problem | Result (sim) |
| --- | --- | --- |
| Steering ±0.5 → ±0.8 (Chris) and left limit 0.48 → 0.91 1/m | arc too wide for the crossing | left 39/40 |
| Pure pursuit lookahead 0.5 → 0.8 m | sharper steering swung the move-aside | 43/48 vs 33/48 |
| Ignore lidar near a seen marker or a known cone while turning | stands and cones read as obstacles | phantom avoidance 29% → 4%; turns 40% → 68% |

![Sensitivity](toby/figures/07_sensitivity.png)

*Left: if the real left reach is 0.7 1/m or less, every left turn fails, so 0.91 (not ground-measured, 4.2) must be
measured before use. Right: camera yaw and marker position must be measured; latency and dropout matter less.*

**Car tests: does each link from marker to action work? (S4)**

| Question (car, date) | Method | Result → decision |
| --- | --- | --- |
| How far is sim from the lab? (4 drives, 12 Oct) | recorded lane width, cone noise, delay, range | 0.77 m, 0.040 m, 0.21 s, 2.54 m → sim uses 0.8 m, 0.03 m, 0.2 s, 2.5 m |
| Does lane side work? (20, 20 Oct) | only `lane_side` changed to LEFT | kept left, by eye only (no log) |
| Do markers reach the policy? (9 runs) | count marker batches | 0 → start with `car_management.sh --aruco` |
| Is the command right? (13, 27 Oct) | log the command | "Marker 14: KEEP_RIGHT", 6 times |
| Full turn (2 Nov) | — | not shown: wrong branch, then no `--aruco` |

<img src="toby/figures/09_car13_phantom_avoidance.png" alt="Car 13 phantom avoidance" style="width: 62%;">

*Car 13, no box on the track: marker stands planned 16 avoidance lines. This led to the lidar filters in S3.
**Limit:** they hide a real object within 0.15 m of a marker or 0.12 m of a cone.*

### 3.4 Yuhao Liu — measured-speed MPC, cone memory and junction planning

**Built** measured-speed prediction and solver-failure stops (#14), an offline final-pair estimator, a cone-memory
U-turn and an experimental cross/T-junction navigator. In Toby's default turner, added grouped marker IDs, earlier
turn preparation, heading-aware arc admission and a persistent stop latch (#35, #36). Diagnosed car 20's left-turn failure.

**S2.** Measured wheel speed sets the prediction step distance $d = v\Delta t$ and the delay projection
$d_\tau = v\tau$. A failed solve requests a stop. The default turner starts preparation on the outside of the lane
after confirmation and checks an arc tangent to the actual heading. At the 0.48 1/m left limit, the radius is
2.083 m. A rejected turn keeps its stop latched after the turn object clears.

**Calibrated MPC integration.** I integrated Toby's asymmetric car 20 steering limits and lane offsets with my
measured-speed prediction and stopping logic in PR #15. In PR #16 I resolved the follow-on CarController merge and
added checks that reject non-finite control inputs.
[Calibration repair](https://github.com/CiaranGruber/robotic-car-ai/commit/32723db) · [CarController integration](https://github.com/CiaranGruber/robotic-car-ai/commit/7390c92)

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

**Cone-row association and replay.** I separated the incoming cone rows from transverse rows at junctions so the
lane fitter could track the approach. The fitter still accepts coherent curves, and obstacle detection keeps all
checked cones. Across the same 7,032 synthetic approach frames, accepted misaligned fits fell from 2,265 to 170 and
aligned fits rose from 2,528 to 4,628. I also added separate observed-lane and selected-path views in Foxglove and MCAP
replay. [Evaluation](https://github.com/CiaranGruber/robotic-car-ai/blob/37b7b39/docs/acceptance.md#cone-grouping-and-lane-formation-update-2026-10-26) · [Implementation](https://github.com/CiaranGruber/robotic-car-ai/commit/d59e812)

![Selected MPC paths for left, right and straight exits at a simulated cross-junction](yuhao/figures/04_junction_selection.png)

*The green lines show the selected MPC paths for three commands at the same simulated crossing. The marker IDs
reflect the earlier experimental mapping.*

#### S4. Car 20 deployment and diagnosis

**23–26 Oct. Deployment and ArUco recovery.** The 23 Oct deployment passed the pinned-interface ROS gate with 186
pytest cases and 189 colcon checks. At startup, all 82 observed drive-and-steering commands were zero in policy state 2.
On 26 Oct I traced the missing ArUco stream to disabled camera RGB output. Restoring RGB and CameraInfo yielded 100
advancing ArUco batches in 10 seconds. A later capture confirmed a valid ID 4 pose while the vehicle was disarmed.
[Deployment record](https://github.com/CiaranGruber/robotic-car-ai/blob/37b7b39/docs/acceptance.md#car-104325433-intersection-deployment-2026-10-23) · [ArUco recovery](https://github.com/CiaranGruber/robotic-car-ai/blob/37b7b39/docs/acceptance.md#car-104325433-rgb-and-junction-configuration-repair-2026-10-26)

**30 Oct. Left-turn diagnosis.**

![Marker range](yuhao/figures/05_marker_range.png)

*All 41 posed ID-1 observations were 3.3–6.4 m away, outside the 2 m gate. The running detector used a 0.250 m marker
size though the print was 0.081 m (range ×3.09); restarting the policy hadn't reloaded it.*

- After restarting only the detector: ID 1 confirmed at 3.0 s, then a **geometry stop** at 5.4 s: 0.174 m outside
  vs 0.182 m needed, heading 4.6° outward. → prepare earlier, tangent admission; limits not relaxed. A zero command
  isn't a physical stop (wheel-speed feedback was still 0.50 m/s).

![Recorded drive, steering and wheel-speed feedback after the detector reload](yuhao/figures/06_post_reload_trace.png)

*The 30 Oct trace marks when the policy confirmed ID 1 and when it latched a geometry stop. Drive command and
wheel-speed feedback are shown separately.*
