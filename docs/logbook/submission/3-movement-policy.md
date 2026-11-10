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
- **Bounded least squares:** stacking gives $\lVert Mu - c\rVert^2$; $M$ depends only on settings (built once), each
  step only changes $c$. `lsq_linear(method="bvls")`, ≤ 4N iterations → bounded time (R6).

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

### 3.2 Peiyu Wang — MPC design, MPC vs RL, lane end and car bring-up

**Built:** the MPC framework and first car run, the Stage 1 design (pure pursuit, MPC grid, RL, milestone M1), lane-end
and lost-lane stops, car models, Foxglove export, car debugging. **PRs** #1 #8 #16 #25.

**S2: lane end and lost lane.** No lane and the farthest cone ≤ 1.5 m ahead → the rows ended: follow the last plan,
request the stop at roll-out distance (`stop_distance_per_speed_s` × v) from 0.25 m past that cone. No lane otherwise →
half speed ≤ 0.5 s, then a latched stop (R7: 0.6 s with the 0.2 s age). **RL:** PPO replaces only the solver; bounded
penalties $P(x,s) = 1 - e^{-(x/s)^2}$, reward $r = 1 - aP(y,0.1) - bP(\psi,0.1) - cP(\dot\kappa,1.0)$.

**S3: which MPC, and does it beat pure pursuit? (6 Oct)** 324 settings × 24 runs (12 cases, ± 0.03 m noise).

![Design front](peiyu/figures/04_design_front.png)

*Tracking against roughness; Pareto front of passing settings; star = chosen (N 20, 3/1/0.1/3): the smoothest
passing. Pure pursuit tracks 3 mm better but steers 2.3× rougher.*

**M1: MPC or RL? (unseen cases U1–U3, opened once).** RL failed R4 on U1 (1.51) and U2 (1.59); the MPC passed all
(U2: 0.51 vs 0.80) → **MPC chosen**. RL's first reward was exploited (unbounded penalties made leaving the lane best).
**Lane end:** before, the car stopped 0.38–0.91 m short (the last pair leaves the 80° view ≈ 0.6 m early); with the
rule, 0.21–0.24 m past (R5).

**S4.**
- **Time budget (car 20, 6 Oct):** batches every 0.1 s, latency 0.197 s → 20 ms per step allowed. Solve N = 20:
  0.46 ms median, 0.53 ms p99; N = 60 still 5.96 ms → the horizon is limited by camera range, not CPU.
- **Car 22 (12 Oct):** car 20's settings drove 1.4 m/s for 0.5 → one setting checked on every car model (#18).
  Lost-lane stop 0.8 s after losing the lane. Steering stayed biased (unsolved).
- **Keeping the cars running:** cone detector crashed under OpenCV 5 (`KeyError 16`) → `publish_rgb_image: false`
  (#24); OAK-D dropping off USB; DREAM processes at 70–80% CPU; a flat battery. Checked read-only first.
- **Foxglove:** any logged step drawn with cones, lane and MPC plan; used to debug the car runs.

### 3.3 Toby Lee — car 20 tuning, lane side, marker actions and intersection turns

**Built:** Stage 1 test cases (#6 #10), car 20 MPC calibration (#15), lane side (#27 #30), marker actions and the
intersection turner (#35), marker simulation; four car sessions (cars 20, 21, 13, 14).

**S2.**
- **Lane side:** $e^{line}$ sits `edge_clearance_m` (c) in from the chosen edge, so the shift available is $w/2 - c$;
  it changes ≤ 0.3 m per metre driven.
- **Markers → actions:** only the nearest marker within 2 m, seen 2 steps in a row, counts. IDs as in 0.4.
- **Intersection turn: APPROACH → ARC → EXIT.** Entry placed 1 m before the marker; move to the outside of the lane;
  drive an arc at 0.3 m/s; hand back to the MPC after 60° once the exit lane lines up, or stop. The arc start $s$ must
  clear the inside corner and the far row (half width $b$, front $f$, cone radius $r_c$, margin $m$, shift $a$):

$$-\sqrt{(R-b-r_c-m)^2 - (R-a-w/2)^2} \;\le\; s \;\le\; w_{cross} - r_c - m - \sqrt{f^2+(R+b)^2}$$

![Turn geometry](toby/figures/08_turn_geometry.png)

*Right turn, 0.8 m roads, R = 1.2 m: from the centre no start clears both; after moving 0.15 m outside, one does.*

**S3.**
- **MPC for car 20 (12 Oct):** 84 runs per setting (curves, S-bend, ±3° yaw, measured noise and delay). Measured
  asymmetric limits (0.48 L / 0.92 R) and change weight 2: 84/84, steering activity 0.71 → 0.20 1/(m s).
- **Lane side (26 Oct):** c = 0.40 m left 0 cm of shift in a 0.8 m lane → c = 0.28 m: 40/40, side error ≤ 0.017 m.
- **Why do left turns fail on car 20? (1–2 Nov)** At 0.48 1/m, R = 2.08 m leaves ≈ 2 cm in a 1 m crossing.

![Root cause](toby/figures/06_root_cause_bars.png)

*Same pattern as the car: left 2/40, right 35/40. Fix (±0.8 steering, left 0.91 1/m, phantom-obstacle filters):
all cases 57% → 99%. The 0.91 1/m left limit is not yet measured on the ground (4.2).*

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

### 3.4 Yuhao Liu — measured-speed MPC, turn preparation and stopping

**Built:** measured-speed prediction and solver-failure stops (#14), grouped marker IDs, earlier and tangent-aware turn
preparation, persistent stop latch (#35 #36), offline lane-end and junction experiments, car 20 diagnosis.

**S2.** Rebuild the motion matrices each step with measured speed in $d = v\Delta t$; delay projection $d_\tau = v\tau$.
A failed solve requests a stop. Turns: prepare on the outside straight after confirmation; admit an arc tangent to the
**actual** heading ($R = 1/0.48 = 2.083$ m); reject an infeasible start rather than relax clearance; keep the stop
reason outside the turn's lifetime.

**S3: does measured speed help?** 3 scenarios × 2 inputs × 5 seeds, 0.2 s delay; distance-weighted
$e_{RMS} = \sqrt{L^{-1}\int_0^L e(s)^2 ds}$:

| Scenario | Nominal speed | Measured speed | Reduction |
| --- | --- | --- | --- |
| Acceleration | 0.07158 m | 0.07072 m | 1.2% |
| Half achieved speed | 0.08314 m | 0.08173 m | 1.7% |
| Slow with dropouts | 0.07532 m | 0.07265 m | 3.5% |

- **Final-pair lane end (offline):** 20/20 when the pair is visible, but a hidden pair gives false ends → kept offline.
- **Stop latch (sim):** at 0.48 1/m, 6/6 runs on 1.0 m roads stop without contact; 6/6 on 1.5 m complete the left
  exit. Suite at `488ec30`: 837 passed.

**S4: why did ID 1 never request a turn? (car 20, 30 Oct)**

![Marker range](yuhao/figures/05_marker_range.png)

*All 41 posed ID-1 observations were 3.3–6.4 m away, outside the 2 m gate. The running detector used a 0.250 m marker
size though the print was 0.081 m (range ×3.09); restarting the policy hadn't reloaded it.*

- After restarting only the detector: ID 1 confirmed at 3.0 s, then a **geometry stop** at 5.4 s: 0.174 m outside
  vs 0.182 m needed, heading 4.6° outward. → prepare earlier, tangent admission; limits not relaxed. A zero command
  isn't a physical stop (wheels still at 0.50 m/s).
