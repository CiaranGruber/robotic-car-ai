## Part 4: Peiyu Wang — MPC framework, Stage 1 design and tools to check car runs

**Role.** MPC sub-team. Stage 1 task list, the MPC framework and first car run, the Stage 1 design (pure pursuit
baseline, MPC grid, RL, milestone M1), lane-end and lost-lane stops, measured car models, Foxglove export and run
timing. **PRs** #1, #8, #16, #25 merged; #18, #24, #31, #32 open. **On cars:** time budget and solve time (car 20);
speed and lost-lane stop (car 22); traced the car 22 detector crash. Lane following itself wasn't shown in my
sessions (car 22's steering failed); M1 and the lane-end stop are simulation.

### 4.1 Where it fits (S1)

![System diagram](peiyu/figures/01_system.png)

*Blue: my parts; grey: teammates'; dashed: simulation only. Runs once per cone batch within 20 ms.*

| Interface | Carries | Constraint |
| --- | --- | --- |
| `LaneDetection` in (2.2) | $e$, $\psi$, κ_path, width | ≈ 0.2 s old; the MPC predicts forward by that age |
| Wheel speed in | m/s, unsigned, 44 Hz | Gaps up to 0.32 s; lost-lane distance uses max(wheel, requested) so a stuck reading stops early |
| (speed, curvature) → `CarController` (7.2) | m/s ≥ 0, 1/m left + | One command per batch |
| Data logs, Foxglove markers | every step's inputs, outputs, plans | Replay runs today's code |

- **One command per batch, well inside 20 ms:** batches every 0.1 s, command timeout 0.5 s → a small bounded
  least-squares problem with an iteration cap, not a general solver.
- **Never move on a guess:** wait stopped for the first usable lane; a lost lane gets a short bridge, then a latched
  stop.
- **Compare designs on one harness:** baseline, MPC and RL all run inside `MpcPolicy` with the same filter, reference
  and lane-end rule; only the steering differs.
- **System consequence:** the lane-end stop depends on the car's roll-out (`stop_distance_per_speed_s`), a car
  control property not yet measured (0.6).

### 4.2 How it works (S2)

**MPC (shared; also used in 5.2 and 6.2).** Lateral error $e$, heading error ψ, curvature κ; the car drives
$d = v\Delta t$ per step, Δt = 0.1 s, N = 20 (1 m at 0.5 m/s):

$$e_{k+1} = e_k + d\,\psi_k + \tfrac{d^2}{2}(\kappa_k - \kappa^{path}_k), \qquad \psi_{k+1} = \psi_k + d\,(\kappa_k - \kappa^{path}_k)$$

$$\min_{\kappa_0..\kappa_{N-1}} \sum_k 3\,e_k^2 + \psi_k^2 + 0.1\,(\kappa_k - \kappa^{path}_k)^2 + 3\,(\kappa_k - \kappa_{k-1})^2 \quad\text{s.t.}\quad |\kappa_k| \le 3.0$$

- **Delay compensation:** predict forward by $d_0 = v\cdot\text{age}$ with the previous command:
  $e \leftarrow e + d_0\psi + \tfrac{d_0^2}{2}(\kappa_{prev} - \kappa^{path})$, $\psi \leftarrow \psi + d_0(\kappa_{prev} - \kappa^{path})$.
  Only the first curvature is sent.
- **As least squares:** stack $x = Fx_0 + G(u - \kappa^{path}\mathbf 1)$; the cost is $\lVert Mu - c\rVert^2$ with

$$M = \begin{bmatrix} W^{1/2} G \\ \sqrt{0.1}\, I \\ \sqrt{3}\, D \end{bmatrix}, \qquad c = \begin{bmatrix} -W^{1/2}\,(F x_0 - G\,\kappa^{path}\mathbf{1}) \\ \sqrt{0.1}\,\kappa^{path}\mathbf{1} \\ \sqrt{3}\,\kappa_{prev}\,e_1 \end{bmatrix}$$

  $M$ depends only on settings (built once); each step only changes $c$. Solved with `lsq_linear(method="bvls")`,
  at most 4N iterations (caps time, R6a).
- **Weights:** only ratios matter. 0.1 m off centre costs 3 × 0.1² = 0.03, the same as ≈ 10° of heading or a
  0.1 1/m curvature change between steps.

| Constraint | Handled as | Why |
| --- | --- | --- |
| Curvature limit | hard bound | The steering can't turn further; bounds are all `bvls` takes, and the problem always has a solution |
| Steering rate | cost (change weight) | A rate limit constrains differences, needing a general QP; the cost keeps R4 at 0.16 mean |
| Lane edges | left out | A hard state limit becomes infeasible when a noisy observation already puts the car near an edge |
| Speed | not a variable | Speed × heading would make the model bilinear; car control holds speed |

| Assumption | Where it breaks |
| --- | --- |
| Small heading error | Large errors after a lost lane |
| Constant speed = target | Car 22 drove 1.4 m/s for 0.5 (4.4); later predicted with measured speed (6.2) |
| No slip | T11 wet floor: 6° sideslip leaves the car 0.25 m off centre |
| Constant path curvature over horizon | A curve's start isn't anticipated |

**Lane end and lost lane (A7).** One row visible: keep going, centre half a width from it. No lane and the farthest
cone was ≤ 1.5 m ahead: the rows ended — follow the last plan and request the stop at roll-out distance
(`stop_distance_per_speed_s` × v) from 0.25 m past that cone (mid R5 zone). No lane otherwise: follow the plan at half
speed for ≤ 0.5 s, then stop (latched). Stop request on the 5th empty batch → 0.6 s with the 0.2 s age (R7).

**RL (A5).** PPO replaces only the solver. Observation: $e$, ψ, lane usable, last two curvatures, age, speed. Reward
with bounded penalty $P(x,s) = 1 - e^{-(x/s)^2}$: $r = 1 - aP(y,0.1) - bP(\psi,0.1) - cP(\dot\kappa,1.0)$, −20 for a
cone, +10 at the end. Domain randomisation over width, spacing, start, speed, noise, delay, mass and floor.

### 4.3 Simulation (S3)

**A3/A4: which MPC settings, and does it beat pure pursuit? (6 Oct)** Baseline:
$\kappa = g\,\frac{2y_L}{L^2 + y_L^2} + \kappa^{path}$, L = 1.0 m, g = 1.0. MPC grid: 324 options (horizon 12/20/30,
lateral 3/10/30, heading 0.3/1/3, change 0.3/1/3/10, limit 3/1/0.5), each on 12 tuning cases with and without
0.03 m noise (24 runs); 73 pass all 24.

![Design front](peiyu/figures/04_design_front.png)

*Left: tracking (lateral RMS) against roughness (curvature-rate RMS), mean of 24 runs; lower left is better. Blue line:
Pareto front of passing MPC options; star: the A4 choice, the smoothest on it. Right: runs passed of 24.*

- The starting weights failed T11 (R1b) and, with noise, T5 (R4 1.02). Chosen: N = 20, 3/1/0.1/3 — R4 0.16 mean,
  0.45 worst, no fallbacks. Pure pursuit tracks 3 mm better but steers 2.3× rougher.
- Lateral weight 30 never passes (amplifies noise); 12 steps can touch cones on T11.
- **Judgement that reversed:** I expected the MPC to win as it was; it lost until the lateral weight came down and the
  change weight went up.

**A5: RL.** Iteration 1 found a reward exploit: unbounded penalties made leaving the lane the best return (episodes
lasted 9 steps). Bounded penalties fixed it. Best policy: 18/24 tuning runs; it still weaves (fails R4).

**M1: MPC or RL? (unseen cases U1–U3, opened only here).** A design is eligible only if it passes every unseen case.

![M1 on the unseen cases](peiyu/figures/05_m1_unseen.png)

*U2 (15° start, two missed cones, one wrong colour): both pass, MPC smoother (R4 0.51 vs 0.80). Right: R4 per case;
RL is above 1.0 on U1 (1.51) and U2 (1.59).*

→ **MPC chosen for Stage 1.** Re-run at `47f8838` for this logbook: every value matches `docs/stage1-design.md`.

**Lane end: why did the car stop short? (A7)** The last cone pair leaves the 80° view ≈ 0.6 m before the car reaches
it, so the lane counted as lost early.

![Lane end](peiyu/figures/03_lane_end.png)

*Left: T1–T10 stop positions before (red) and with (green) the A7 rule; shaded is R5. Right: T12, camera covered at
3 m: stop request 0.60 s later; the car rolls to a stop inside the lane.*

- Before: 0.38–0.91 m short; with the rule and 0.2 s age: 0.21–0.24 m past. **Reversed judgement:** a constant-decel
  ramp rolled 0.64–0.75 m past (controller lag); requesting the stop at the roll-out distance fixed it.

**Car models (12 Oct).** First-order speed and inverted steering maps for car20 and two fits of car 22. With car 20's
values, the car 22 models drive 1.4–1.5 m/s for 0.5 (as car 22 did); with `dev`'s car-22 values the car20 model
passes 0/12. The #18 setting passes 10, 11, 11 of 12 → one shared setting, checked against every model (#18, open).

**Camera-pan prototype (20 Oct).** On a 1.5 m loop the fixed camera lost the lane after 8.6 s; the ideal pan kept it,
but a naive pan (latest command, not the angle at image time) lost the lane on the oval. → panning helps only with
the angle at image time known; the team kept pan 0.

**Can't conclude:** car results (dream-gym tyres not scaled for 1:10, roll-out not measured, one calibration per model,
RL one seed).

### 4.4 On the car (S4)

**Time budget (car 20, 6 Oct).** Cone batches 10.0 per second (gaps 0.09–0.11 s), detector latency 0.197 s, command
timeout 0.5 s → **20 ms at p99** allowed per step.

![Timing on the car](peiyu/figures/07_car_timing.png)

*Left: 300 solves per horizon on car 20's computer. Right: run summary of a 22 Oct log.*

- N = 20: 0.46 ms median, 0.53 ms p99; even N = 60 is 5.96 ms at p99 → horizon is limited by camera range, not CPU.
- 22 Oct logs (another branch, with avoidance): steps 57 ms apart median, up to 368 ms; avoidance 10–30× costlier
  than lane following (laptop replay).

**Car 22 on the floor (12 Oct).**

| What | Result | Decision |
| --- | --- | --- |
| Speed | car 20 values: ≈ 1.4 m/s (up to 1.7) for 0.5; car 22 settings: 0.46–0.49 m/s | car models, one setting (#18) |
| Lost-lane stop | stopped 0.8 s after losing the lane | A7 works on a car |
| Steering | MPC asked for more right (+0.06 → +0.36) but heading stayed ≈ +0.22 rad; lost the lane after 3.4 s | unsolved: inverted steering, offset centre or camera yaw (0.6) |

**Detector crash (car 22).** The cone detector exited ≈ 13 s after every start: OpenCV 5 in the venv changed
`CV_8UC3`, so publishing the RGB image raised `KeyError 16`. Fix: `publish_rgb_image: false` (#24). Cost: the ArUco
detector then gets no image (conflicts with 5.4's `--aruco`) until the venv has OpenCV 4.

**Foxglove replay (car 20, 19–20 Oct).** Any logged step can be drawn with the cones, lane and MPC path. Limit: a replay
runs today's code (other actions on 181 of 183 steps), so judge runs from logged actions.

| Sim vs car | Simulation | Car | Meaning |
| --- | --- | --- | --- |
| Speed, car 20 values | 1.4–1.5 m/s | 1.4 m/s, up to 1.7 | agree (but models were fitted to car 22) |
| Lost-lane stop | 0.60 s (T12) | 0.8 s | rule works; car has no known cover moment |
| Step interval | 0.1 s | 57 ms median, up to 368 ms | budget needs re-measuring with avoidance |
| Roll-out after stop | 0.6 s | not measured | lane-end distances unproven on a car |

**Keeping the cars running.** Faults were checked read-only first (`dream runtime status/logs`, `top`, USB events,
`ros2 topic hz`): OpenCV 5 crash (above); OAK-D dropping off USB (restart only the detector); DREAM `runtime_runner`
using 70–80% CPU each (reported upstream); wrong `left_colour` giving no lane; a flat motor battery. Also resolved
the 12 Oct merge of four branches into `dev` (#8, #15, #16, #17).
