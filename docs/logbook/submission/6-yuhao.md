## Part 6: Yuhao Liu — measured-speed MPC, cone memory and intersection stopping

**Role.** MPC sub-team. Measured-speed prediction and solver-failure handling, final-pair stopping and cone-memory
experiments, an experimental junction navigator, grouped marker IDs, left-turn diagnosis on car 20 and stop
persistence. **Changes:** #14, `32723db`, `e908d73`/`fd01806`, `d59e812`, #35, #36. **Status:** measured speed and
marker/stop changes are integrated; final-pair and U-turn work are offline experiments; the junction navigator is
disabled by default; no physical left turn completed.

### 6.1 Where it fits (S1)

![System diagram](yuhao/figures/00_system.png)

*Where prediction, route selection and stop decisions sit. Blue boxes contain my contributions within shared
components. The default marker turner (5.2) and the experimental junction route are alternative paths; dashed =
offline.*

| Interface | Used / produced | Constraint |
| --- | --- | --- |
| Cones and observation age | positions, colour, acquisition time | Old measurements describe an earlier pose |
| Wheel speed, IMU | m/s (unsigned), heading | Prediction assumes forward travel; memory drifts and needs expiry |
| Posed ArUco detections | id, position, freshness | Pose scale depends on the detector's admitted marker size; editing the YAML doesn't reload a running component |
| `CarMemory` | plan, pending command, phase, stop reason | Losing a manoeuvre object must not erase a stop |

- Track within the car's steering reach: a curve outside the measured limits isn't made feasible by a weight.
- Obey the requested exit: a closed or unsupported left exit must not silently become straight travel.
- Keep stopping separate from success: a guarded stop can be correct fault handling while failing the task.

### 6.2 How it works (S2)

- **Measured-speed MPC.** The model is 4.2's; I rebuild the motion matrices each step with the fresh measured speed in
  $d = v\Delta t$ (constant over the horizon), and the delay projection uses $d_\tau = v\tau$ with the previous
  curvature. A failed solve (non-finite input, bad status, shape or bounds) requests a stop; a cached plan stops at
  exhaustion instead of repeating its last steering.
- **Final-pair stopping (offline).** Both rows ≥ 2 cones, farthest cones agree within 0.15 m and within 2.5 m,
  confirmed across 3 increasing acquisition times; then cap speed at 0.5 m/s and enforce the assumed envelope

$$v\,t_r + \frac{v^2}{2a} \le d_{usable}, \qquad t_r = 0.3\ \text{s},\ a = 0.5\ \text{m/s}^2$$

- **Cone memory and geometric steering.** Cones are moved into a remembered frame with wheel/gyro odometry and
  propagated when a row is briefly lost; steering to a lookahead target $(x_L, y_L)$ in the body frame is
  $\kappa = 2y_L/(x_L^2 + y_L^2)$. No learning; memory age and travel are bounded.
- **Junction navigator (26 Oct, disabled by default).** Needs positive outgoing-row support; an interior closing row
  vetoes the exit; an unknown exit allows only a finite approach that stops before the junction mouth.
- **Contributions to the default turner (#35/#36):** grouped IDs (table in 5.2); prepare on the outside immediately
  after confirmation; admit an arc tangent to the **actual** heading ($R = 1/|\kappa_{max}| = 2.083$ m at 0.48 1/m);
  reject an infeasible start rather than relax clearance; keep the stop reason outside the turn object's lifetime and
  request the bounded brake.

### 6.3 Simulation (S3)

**Could weight tuning fix the early stop?** 60 tuning runs (baseline, longer horizon, smoother) + 10 validation runs:
all candidates 0/20 completion — all stopped before the 6 m finish as rows left the view. Smoother: curvature-rate RMS
−32%, lateral RMS +12%. → not a weight problem; investigate lane-end evidence.

**Does measured speed improve prediction?** 3 scenarios × 2 speed inputs × 5 seeds = 30 runs; 0.2 s delay, 0.01 m
noise; distance-weighted $e_{RMS} = \sqrt{L^{-1}\int_0^L e(s)^2ds}$ over the first 4.5 m (so slow travel can't flatter
the score).

| Scenario | Nominal speed RMS | Measured speed RMS | Reduction |
| --- | --- | --- | --- |
| Acceleration | 0.07158 m | 0.07072 m | 1.2% |
| Half achieved speed | 0.08314 m | 0.08173 m | 1.7% |
| Slow with dropouts | 0.07532 m | 0.07265 m | 3.5% |

![Measured speed](yuhao/figures/01_measured_speed.png)

*Seed 200, lateral position against distance; orange measured speed, blue nominal.*

→ small, consistent benefit, no fallbacks. Limit: uncalibrated longitudinal dynamics, no `CarController`.

**Can a final cone pair identify the finish?** 7 scenarios × baseline/prototype × 5 seeds = 70 runs.

| Scenario | Baseline | Prototype | Meaning |
| --- | --- | --- | --- |
| Final pair becomes visible (4 cases) | 0/20 | 20/20 | stops in the 6.0–6.5 m gate |
| Final pair hidden | 0/5 | 0/5 | 5/5 false endpoints at 5.583 m |
| All cones from 4 m hidden | 0/5 | 0/5 | 5/5 false endpoints at 3.583 m |
| Detection stream fails at 5 s | 0/5 | 0/5 | expected fault aborts |

![Endpoint cases](yuhao/figures/02_endpoint_profiles.png)

*Speed against distance, seed 300; green prototype, blue baseline, dashed the true finish. A previous visible pair can
look like the end.*

→ temporal agreement can't prove hidden cones don't continue the lane: **kept offline**, not claimed as lane-end
detection.

**Cone-memory U-turn.** 12 varied layouts: 10/12 complete, 2 guarded stops, 0 contacts. **Critical assumption:**
2.2 1/m left capability, far tighter than car 20's — so this can't establish a physical U-turn.

**Does positive exit evidence select the requested road? (26 Oct, IDs 3/4/5)**

![Junction selection](yuhao/figures/04_junction_selection.png)

| Experiment | Outcome |
| --- | --- |
| Cross, 0.2/0.4 m spacing | 20/22 completed (2 noisy approaches stopped) |
| Dense T, exit available | 22/24 completed |
| Dense T, exit closed | 6/6 stopped |
| Sparse T, 0.5/0.6 m spacing | 18/18 stopped (approach lane lost before the junction) |

**Does stopping survive a rejected turn?** At the 0.48 1/m left limit: 6/6 at 1.0 m width stop without contact; 6/6
at 1.5 m complete the left exit. Policy suite at `488ec30`: 837 passed, 5 skipped, 1 expected failure.

### 6.4 On the car (S4)

**Why did ID 1 never request a turn? (30 Oct, car 20)** Compared two recordings, settings, installed hashes and the
detector's **active runtime snapshot**, without widening the 2 m gate.

| Run | Policy steps | Posed ID-1 observations | Reported range | Within 2 m | Turn states |
| --- | --- | --- | --- | --- | --- |
| 14:46:15 | 157 | 14 | 3.373–4.594 m | 0 | 0 |
| 14:50:00 | 62 | 27 | 3.279–6.369 m | 0 | 0 |

![Marker range](yuhao/figures/05_marker_range.png)

*All 41 posed ID-1 observations are beyond the unchanged 2 m gate.*

- **Finding:** the running detector used 0.250 m although the print and YAML were 0.081 m (restarting the policy hadn't
  reloaded it): an expected range scale of 0.250/0.081 = 3.09. **Intervention:** with the car stationary, restart only
  the detector and verify 0.081 m in both snapshot and live readback.

**Why did the left turn still stop after the reload? (15:02:43 run, 705 aligned records)**

| Event | Time | Evidence |
| --- | --- | --- |
| ID 1 confirmed | 3.025 s | `TURN_LEFT`, entry 2.483 m ahead, lane width 1.013 m |
| Geometry stop | 5.438 s | stop latch, left radius 2.083 m; drive zero, wheel speed still 0.503 m/s |
| `ARC` phase | 6.017 s | stop and zero drive remain — a phase name doesn't mean the arc was driven |

![Post-reload trace](yuhao/figures/06_post_reload_trace.png)

*Wheel speed and normalised drive/steering for the first 10 s; vertical lines at confirmation and the geometry stop.*

- At rejection: outside offset 0.1738 m vs 0.1823 m required, heading 4.61° outward, 26.5 mm past the latest clear
  start. **Decision:** prepare immediately, tangent admission, exit search from the arc end; don't relax limits.
- **Limits:** not a matched trial; no external pose ground truth; the log has a 0.3 pan command against a zero-pan TF
  (restored in #36).

| Assumption | Car evidence | Decision |
| --- | --- | --- |
| A posed marker in the log reaches the command selector | every range outside the gate (stale snapshot) | verify admitted runtime settings before changing logic |
| An aligned feasible arc is enough | arrived late and 4.61° outward | actual-heading tangent admission, earlier preparation |
| A zero command is a stop metric | residual wheel speed after zero drive | keep software stop separate from physical stopping |
