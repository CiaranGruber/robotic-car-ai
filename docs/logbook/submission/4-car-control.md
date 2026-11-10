## Part 4: Car control

### 4.1 Shared: the car model, controller and limits everyone uses

Everything above car control works in m/s and 1/m; car control is the one block that knows the car (Chris's
`CarController`, `policy/control/control.py`, #17).

| Interface | Carries | Constraint |
| --- | --- | --- |
| (speed, curvature) from the MPC | m/s, 1/m (+ left), brake; 10 Hz | curvature within car 20's +0.48 / −0.92 1/m |
| Wheel speed | m/s, 10 Hz | unsigned, quantised at low speed, holds its last value 3–4 s after stopping |
| drive, steer → vehicle | normalised; drive ≤ 0.75 | platform slew limits; steer ±0.5 after trim |

**Car model from system ID** (first order plus dead time):

$$v(t) = v_{ss}\left(1 - e^{-(t - t_0 - \theta)/\tau}\right), \qquad v_{ss} = K\,(u - u_0)$$

| Quantity | Car 8 (29 Sep) | Car 20 (5 Oct) |
| --- | --- | --- |
| $K$ (m/s per unit drive) | 3.97 | 9.18 (±10%) |
| $u_0$ (drive offset) | 0.697 | 0.475 |
| τ / θ | 1.1 s / 0.5 s | 2.7 s / 0.2 s |

**Controller:** feedforward for the fast part, low-gain PI for the rest; per-side inverse steering map.

$$u = \mathrm{clip}\Big(u_0 + \frac{v_{ref}}{K} + k_p e + I_k,\ 0,\ 0.75\Big), \quad I_k = \mathrm{clip}(I_{k-1} + k_i e \Delta t, \pm I_{max}), \quad e = v_{ref} - v$$

$$s(\kappa) = \begin{cases} (0.038 - \kappa)/0.959 & \kappa < 0 \text{ (right)} \\ \max\big((0.008 - \kappa)/0.630,\ -0.75\big) & \kappa \ge 0 \text{ (left, saturates} \approx 0.47\text{ 1/m)} \end{cases}$$

- Car 20: $k_p$ 0.15, $k_i$ 0.04 (0.2 after the fix), PI off below 0.4 m/s. **No D term:** quantised wheel speed would
  be amplified. Anti-windup: no integration while clipped; brake bypasses the PI.
- **System consequence:** the left limit (R ≈ 2.1 m) caps the MPC and avoidance: it explains the failed left turns
  (3.3) and how close two obstacles can be (4.2).

### 4.2 Chris (Chuhan) Wang — system ID, speed fix and avoidance validation

**Built:** system ID on cars 8 and 20, `CarController` (#17), two-obstacle fixes (#23), the 26 Oct speed fix, avoidance
validation on car 20.

**S4: system ID (car 20, 5 Oct, with Yifei).** One drive level per run, steer 0, 7–8 m straight, bagged; dead-band
sweep 0.50–0.60. Battery checked by repeating 0.75 at start and end (1.19 vs 1.14 m/s at 2 s). Steering: PI at
0.5 m/s, κ = yaw rate / speed over the last 3 s; ±0.5 repeated after a reboot within 3%.

![Car 20 maps and PI](chris/figures/03_car20_maps_pi.png)

*(a) drive → speed (shaded: no motion). (b) steer → curvature per side. (c) step to 0.5 m/s: PI v1 27% overshoot,
v2 (#17) 15%, ≈ 0.013 m/s steady error.*

**S3 + S4: the 26 Oct speed fault.** The car held 0.87 m/s for a 0.5 target, then (feedforward 0.15) didn't start.
Cause: the integral only ran for $|e| < 0.2$ m/s, so a large error froze it.

![FF error sweep](chris/figures/04_ff_error_sweep.png)

*Sim, true $u_0$ swept with feedforward fixed at 0.48: old rule reaches 0.5 m/s only for errors −0.02 to +0.03; new
rule (integrate when above target or not moving) across −0.12 to +0.28. Trade-off: more overshoot (0.62 vs 0.53 m/s).*

| Car 20, 26 Oct, fixed | Moving after | Reached 0.5 m/s | Peak | Before stop |
| --- | --- | --- | --- | --- |
| 15:20:27 | 0.8 s | 1.9 s | 0.62 m/s | 0.50 m/s |
| 15:21:42 | 0.4 s | 3.2 s | 0.57 m/s | 0.54 m/s |
| 15:23:01 | 0.9 s | 1.8 s | 0.60 m/s | 0.52 m/s |

**Avoidance validation.**
- **Sim (15 Oct video, car stopped beside the first of two boards):** two planner bugs (check band widened on both
  sides; line re-centred on board 1's gap) → all spacings pass with ≥ 0.11 m clearance (#23).
- **Car 20 (19 Oct):** still braked: the lane estimate jumped 0.35 m in 40 ms as a near cone left view (2.1), putting
  board 1 in the path → block within 0.6 m only if lane and lidar frames agree.
- **How close can two boxes be? (sim, 30 Oct):** left bends worst (≈ 3 m needed at 3 m radius vs 1.2 m straight).
- **Stand test (30 Oct):** RC steered ≈ 2× further than autonomous; ±0.8 gives +60% left travel. The ground re-map
  (0.91 1/m) wasn't done.

### 4.3 Yifei Xia — control tests, marker recognition and camera pan

**Built:** speed and steering tests with Chris (results in 4.1–4.2), ArUco ID test tools (#26), camera pan
calibration and a compensation simulation, battery-monitoring study.

**S2: heading compensation.** With vehicle heading $\psi_v$ and target camera heading $\psi_t$, wrap $w$ to the shortest
angle:

$$\theta_d = w(\psi_t - \psi_v), \qquad u_d = c + g\,\theta_d, \qquad c = 0.32,\ g = -0.43\ \text{per rad}$$

Range ±0.15 about centre (exceeding latches a fault), rate ≤ 0.1 per s, stale IMU (> 0.5 s) → hold.

**S3.** Interactive simulation (servo τ 0.15 s assumed, delayed heading, 20 s run, CSV export); 11 controller checks
pass, no comparative results yet. Offline ArUco decode of stored images: IDs 0, 2, 5, 8 (examples, not accuracy).

**S4.**
- **Live ArUco (19 Oct):** the detector ran but its image topic had zero publishers. Once the image pipeline ran:
  `ids=[2]`, `TARGET_SEEN_POSE_OK` (same root cause as 3.3's `--aruco`).
- **Camera pan calibration (30 Oct – 2 Nov):**

![Camera pan calibration](Yifei/figure/camera-pan-calibration.png)

*Left ≈ +90° at −0.35, forward at 0.32, right ≈ −90° at 1.00: a normalised command is not an angle. No repeats.*

- **Battery:** motor 8.4 V NiMH and computer power bank are separate; no voltage source without modification →
  not implemented; wheel-speed feedback (4.1) absorbs the change instead.
