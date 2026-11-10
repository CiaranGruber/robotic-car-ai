## Part 2: Thanh Long Nguyen — lane estimation, the camera pan and the car simulator

**Role.** Road detection owner: the block that turns cones into the lane, and its output `LaneDetection`, used by the
MPC, obstacle avoidance and the intersection turns. Then the whole-system simulator and live 3D viewer.
**Commits** `8b71f8b`, `f96ee66` (9 Oct), `f49333d`, `3319051` (10 Oct), in `dev` via #19; `6c4cd17` on
`feature/car-simulator` (no PR). **On cars:** car 20 drove on it 9 Oct; on `dev` in every team run since 14 Oct.

### 2.1 Where it fits (S1)

![System diagram](long/figures/01_system.png)

*Blue: what I built (lane estimation, overlay node, pan hold, and the simulator band that replaces every grey box).
Three blocks steer the car from the same lane.*

| Interface | Carries (units) | Constraint |
| --- | --- | --- |
| Filtered cones in (3.2) | (x, y) in `base_link` (m), colour; 10 Hz, ≈ 0.2 s old | Camera must point straight ahead (pan 0) |
| `LaneDetection` → MPC (4.2) | $e_y$ (m), $e_\psi$ (rad), $\kappa_c$ (1/m), + = left | Constant curvature over the view; `NO_LANE` makes the MPC stop |
| → obstacles, avoidance (3.2) | centre arc, width, `visible_until_m` | Obstacles are placed on my arc, so a jump in it moves them |
| → intersection turner (5.2) | lane while approaching | The cross road bends one arc; the turner keeps only approach cones |
| Camera pan set-point | 0.0 | DREAM's camera TF is fixed at zero pan |

**How my errors reach the system:**
- A camera yaw δ becomes a lane offset: the MPC zeroes the measured $e_\psi$, so the car settles about 1.2 cm per
  degree off centre, and turns are planned from the wrong place (2.3).
- A curvature error δκ becomes a heading error ≈ δκ · s̄ (s̄ ≈ 1–1.5 m, the mean cone distance).
- A jump in the estimate moves every obstacle: a 0.35 m jump made a board look in-path and stopped car 20 (7.4).
- `NO_LANE` stops the car: a doubtful lane is a stop, not a guess.

### 2.2 How it works (S2)

![Lane estimation](long/figures/02_lane_estimation.png)

*Top: the steps of `LaneDetector.detect` (blue = mine, dashed = teammates' later additions); step 5 loops back to 4 to
drop outliers and refit. Bottom: the model for a lane curving left.*

- **Model:** a reference arc from the car with heading ψ and curvature κ; each edge is the concentric arc offset $d$,
  curvature κ/(1 − κd). Straight is κ = 0, not a separate model.
- **Signed distance of cone j** (exact as κ → 0), with $s_j = x_j\cos\psi + y_j\sin\psi$,
  $\ell_j = -x_j\sin\psi + y_j\cos\psi$:

$$D_j = \frac{2\ell_j - \kappa(s_j^2 + \ell_j^2)}{1 + \sqrt{1 - \kappa\,(2\ell_j - \kappa(s_j^2 + \ell_j^2))}}$$

- **Fit** over θ = (ψ, κ, $d_L$, $d_R$), soft-L1 loss ρ (scale 0.05 m), with a curvature prior (σ = 0.05 m,
  κ_typ = 0.5 1/m) so noise alone can't bend a straight lane:

$$\min_\theta \sum_j \rho\big(D_j - d_{edge(j)}\big) + \rho\big((\sigma/\kappa_{typ})\,\kappa\big)$$

- Started from the better of two linear fits; at most 50 evaluations; cones > 0.15 m from their edge are dropped and
  the fit repeated (≤ 2 times). Straight mode (κ = 0) is one linear solve.
- **Outputs:** $w = d_L - d_R$, $d_c = (d_L+d_R)/2$, $e_y = -d_c$, $e_\psi = -\psi$, $\kappa_c = \kappa/(1-\kappa d_c)$.
  `NO_LANE` unless 0.3 ≤ w ≤ 3.0 m, edges not crossed and κd < 0.9.
- **AUTO sides and width memory (10 Oct):** the edge further left is the left edge, whatever its colour. While both
  edges are seen, $w_{mem} \leftarrow w_{mem} + 0.2(w - w_{mem})$; with one edge, the centre is $w_{mem}/2$ from it.
- **Lever arm:** $\psi(0) = \psi(\bar s) - \kappa\bar s$, so a false bend of 0.16 1/m at s̄ = 1.1 m predicts +10.1° of
  heading error (+10.8° observed, 2.4).
- **Camera pan:** with the TF fixed at zero pan, a camera turned by δ gives $e_{\psi,meas} = e_{\psi,true} + \delta$ and
  almost no change in $e_y$ (rotation is about the camera, 0.10 m behind `base_link`). So the policy holds the pan at
  0 (`f49333d`).

| Decision | Alternative | Why | Cost |
| --- | --- | --- | --- |
| Fit both edges together | Each row its own line (the MPC's old fit) | Short row borrows the long row's direction; one edge still gives an offset | Iterative solve in curved mode |
| Straight edges for Stage 1 | Curvature free | Free curvature added 4–11° on straight lab lanes | Curves need a setting (switched 15 Oct) |
| Robust loss, drop > 0.15 m, refit | RANSAC (tried by Zhenling 19 Oct, reverted) | Only 3–6 cones per row; refit keeps good cones | Fails when wrong cones are a large share of a short row |
| Stateless, one batch at a time | Track the lane | Deterministic, testable, bounded time | Jumps when cones leave view; local map added (3.2) |
| Hold the pan at 0 | Measure and correct yaw | Bias changed drive to drive; DREAM assumes zero pan | Servo must be mounted centred |

**Simulator (27 Oct).** Runs the real `MovementPolicy` from `config/ai4r_policy.yaml` with the node's start/stop rules,
a kinematic car with car 20's measured drive and steering maps, camera cones (86°, 0.4–2.5 m, 3 cm noise, 0.2 s
old), lidar, wheel speed, gyro, markers, and a browser 3D view. Trade-off: decoded detections, not pixels, so it is
fast but the camera is optimistic (2.3).

### 2.3 Simulation (S3)

**Is fitting the edges together better than per row?** Drawn straight lane, 3 cm noise, 200 seeds, right row 2–12
cones. With a 3-cone row: 0.69° vs 3.03° heading RMS; equal (0.51°) at 12 cones each. On the even real drives,
jitter was about the same. **Decision:** the joint fit pays off where rows are short (lane ends, turns, hidden cones).

![Fit accuracy](long/figures/04_fit_accuracy.png)

*Drawn 0.8 m lanes through the measured camera, 30 seeds, car 0.10 m left and 5° left. Red (straight mode) leaves
the plot on any curve; green (curved mode with noise) stays low until the tightest bends.*

- **Accuracy, curved mode:** $e_y$ within 1.9–3.4 cm and $e_\psi$ 1.7–3.0° for radius ≥ 1.5 m left / ≥ 3 m right — well
  inside R1b's 0.10 m. At 1 m radius only 6–8 cones are seen and $e_\psi$ is 7–12° out.
- **Straight mode:** best on straights (1.4 cm, 0.4°) but 10 cm and 10° out at 8 m radius. → straight for Stage 1,
  curved for curved roads.
- **One edge, narrower lane:** a fixed 1.0 m width put the centre 20 cm off in a 0.6 m lane; width memory 1.9 cm. A
  fixed `"BLUE"` found no lane with yellow on the left; AUTO 100%. → AUTO and width memory (10 Oct).

![Wrong-colour cone](long/figures/06_wrong_colour_cone.png)

*Failure case: one yellow cone in the blue row. Green cells: the wrong cone is left out from that many cones per
row. Red: false lanes, with the false heading.*

- Straight mode rejects it from 4 cones per row; curved mode only from 7–9, and can give a false lane turned 59–78°.
  Rows are short exactly where this matters (lane ends, turns). Proposed fix (not built): treat a lone cone of one
  colour inside the other row as unknown.

**Does the whole policy drive six tracks?** Car-20 model, 5 seeds, every cone seen and with 20% missed.

![Simulator tracks](long/figures/07_simulator_tracks.png)

| Track | Seed 0 | 5 seeds | 20% missed |
| --- | --- | --- | --- |
| lab_straight (3.2 m) | stops 0.39 m past the last pair | 5/5 | 5/5 |
| curvy_road (11.6 m) | drives it all | 5/5 | 5/5 |
| obstacle_lane | passes the box on the left | 3/5 | 4/5 |
| aruco_lane (keep left, keep centre) | 0.3 m left, then back | 5/5 | 5/5 |
| cross: left / right / straight | +89.5°, −89.0°, +0.4° | 5/5, 4/5, 4/5 | 3/5 each |
| T: left / straight into the closed arm | +91.3° / stops "exit closed" | 4/5, 5/5 | 4/5, 4/5 |

- 76/90 seeded runs pass; **no run touched a cone or box** (every failure is a stop). Junctions are least robust
  (22/25 → 17/25 with missed cones), matching the car (no full turn shown).

![Camera yaw](long/figures/09_camera_yaw.png)

*Uncorrected camera yaw −8° to +8° (policy not told), 3 seeds. Both turns pass only near 0°; the shaded band is the
yaw the real 6 Oct drives had.*

- lab_straight passes at every yaw but settles ≈ 1.2 cm/° off centre. curvy_road fails all runs at +8°. Left turns
  pass only 0 to +4°, right only −4 to 0. → **the pan hold is a precondition for turns.**
- **Sim vs real camera (2.4 drives):** real 14 cones per step vs 20 simulated, farthest 1.92 vs 2.34 m, residual
  3.4 vs 2.6 cm. The simulated camera is optimistic: pass rates are an upper bound.
- **Can't conclude:** car pass rates (no pixels, no tyre slip, ideal tracks); built on the 26 Oct policy.

### 2.4 On the car and in real recordings (S4)

**What does the estimate measure on real drives?** Ciaran's four 6 Oct straight drives: drive fixed at 0.55,
steering 0, pan never commanded.

| Drive | Both edges | Width (mean ± sd) | Worse-edge RMS | Seen to | $e_y$ step change (median / p95) |
| --- | --- | --- | --- | --- | --- |
| straight_even | 67% | 0.82 ± 0.02 m | 2.8 cm | 1.8 m | 1.3 / 5.3 cm |
| messy_cone | 72% | 0.84 ± 0.03 m | 4.5 cm | 1.8 m | 1.6 / 4.3 cm |
| outlier_cone | 63% | 0.80 ± 0.02 m | 2.8 cm | 1.7 m | 2.6 / 7.2 cm |
| long_track | 84% | 0.74 ± 0.03 m | 3.6 cm | 2.6 m | 1.3 / 5.4 cm |

→ noise well inside R1b; lane seen to ≈ 2 m, not 4 m, so the cone range was cut to 2.5 m for car 20. Limit: widths
weren't tape-measured.

**Why did the lane look turned? (hypothesis test)** Every drive read $e_\psi$ = +6° to +11°. With steering at 0 the
real heading can't change: a real heading would make the car drift distance × sin(heading); rotated cones wouldn't.

![Pan bias](long/figures/11_pan_bias.png)

| Drive | Median $e_\psi$ | Drift if real | Measured drift |
| --- | --- | --- | --- |
| straight_even | +9.6° | +0.16 m | −0.05 m |
| messy_cone | +10.9° | +0.19 m | −0.03 m |
| outlier_cone | +8.0° | +0.38 m | −0.01 m |
| long_track | +6.5° | +0.32 m | −0.07 m |

- The cones were rotated, not the car; the pan was never commanded. **Decision (10 Oct):** hold the pan at 0, with a
  test. After the fix (13 Oct, hand-placed car), heading at rest −5.0° to +2.8° — consistent, not a controlled test.
  It came back once: the 30 Oct car test set pan 0.3 (`bf5e08c`; the test flagged it), restored in #36.

**Straight or curved for Stage 1?** With curvature free, the straight drives read a 6–12 m bend in 92–97% of steps.
The lever arm predicts the extra heading error within 1°:

| Drive | False κ | s̄ | Predicted κ·s̄ | Observed |
| --- | --- | --- | --- | --- |
| straight_even | 0.162 1/m | 1.09 m | +10.1° | +10.8° |
| outlier_cone | 0.158 1/m | 1.08 m | +9.8° | +9.1° |
| long_track | 0.080 1/m | 1.50 m | +6.9° | +6.9° |

**First lane following (car 20, 9 Oct).** MPC followed `LaneDetection` at 0.5 m/s, curvature limited to 0.5 1/m.
The fit measured a 0.65 m floor lane, which led to AUTO and width memory the next day. No log or video was kept.
Step time on recorded drives: 0.4 ms straight, 4.3 ms curved at p99 (laptop).

**What later team runs showed about my block:**

| Date, car | Seen | Cause | Done |
| --- | --- | --- | --- |
| 19 Oct, car 20 | Estimate jumped 0.35 m in 40 ms; false obstacle stop | One batch at a time; near cone left the view | Lane and lidar frames must agree (7.4) |
| 20 Oct, S-bend | Curvature flipped left/right | One arc can't fit an S | 3.5 m cone range (3.2) |
| 22 Oct, 4 drives | $e_y$ jumps up to 0.57 m | As on 19 Oct | Local map recalled cones; p95 jump halved (3.4) |

**Limits:** one arc can't fit an S-bend; short rows can't reject a wrong-colour cone; no ground-truth $e_y$ on the car;
the simulator hasn't been compared with a logged car run.
