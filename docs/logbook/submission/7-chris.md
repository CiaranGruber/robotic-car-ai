## Part 7: Chris (Chuhan) Wang — system ID, speed control and avoidance validation

**Role.** Car control sub-team with Yifei: system ID on the real cars, the low-level `CarController` (speed and
steering), the 26 Oct speed fix, and validation of obstacle avoidance. **PRs** #17 (`CarController`), #23
(two-obstacle fixes); open: `fix/speed-overspeed-integral` (tested on car 20), `test/steering-limit` (±0.8).
**On cars:** system ID (cars 8, 20), PI tuning, speed fix and avoidance runs (car 20). 7.3's prototype and spacing
study are simulation.

### 7.1 Where it fits (S1)

![System diagram](chris/figures/01_system.png)

*Blue: mine. Everything above car control works in m/s and 1/m, so it can be tuned in simulation and moved between
cars; car control is the one block that knows the car's physics.*

| Interface | Carries | Constraint |
| --- | --- | --- |
| `DriveCommand` from the MPC | speed (m/s), curvature (1/m, + left), brake, 10 Hz | Curvature within car 20's +0.48 / −0.92 1/m |
| Wheel speed | m/s, 10 Hz | Unsigned, quantised at low speed, holds its last value 3–4 s after stopping |
| drive, steer → vehicle | normalised; drive ≤ 0.75 | Platform slew 10/s steer, 50/s drive; steer ±0.5 after trim |
| Curvature limits → MPC, avoidance | +0.48 / −0.92 1/m | Measured on car 20 at ±0.5; used in #15 (5.2) |

- Track 0.5 m/s with little overshoot despite ≈ 0.2 s dead time and τ ≈ 2.7 s → feedforward for the fast part,
  low-gain PI for the rest.
- Work on any car and battery → feedforward is only a start; the integral must absorb map errors.
- **System consequence:** car 20's left limit (R ≈ 2.1 m) caps the MPC and avoidance: it explains the failed left turns
  (5.3) and limits how close two obstacles can be (7.3).

### 7.2 How it works (S2)

**Car model from system ID:**

$$v(t) = v_{ss}\left(1 - e^{-(t - t_0 - \theta)/\tau}\right), \qquad v_{ss} = K\,(u - u_0)$$

| Quantity | Meaning | Car 8 (29 Sep) | Car 20 (5 Oct) |
| --- | --- | --- | --- |
| $K$ | speed per unit drive (m/s) | 3.97 | 9.18 (±10%) |
| $u_0$ | drive offset | 0.697 | 0.475 |
| τ | time constant | 1.1 s | 2.7 s |
| θ | dead time | 0.5 s | 0.2 s |
| breakaway | drive at which the car moves | — | 0.52–0.53 (≈ 0.68 on 26 Oct) |

Steering (κ = yaw rate / wheel speed, positive steer turns right): right $\kappa = -0.959s + 0.038$, left
$\kappa = -0.630s + 0.008$, saturating at κ ≈ 0.47 1/m beyond $s = -0.75$.

**Controller** (`policy/control/control.py`):

$$u_{ff} = u_0 + \frac{v_{ref}}{K}, \quad e = v_{ref} - v, \quad I_k = \mathrm{clip}(I_{k-1} + k_i e \Delta t, \pm I_{max}), \quad u = \mathrm{clip}(u_{ff} + k_p e + I_k,\ 0,\ 0.75)$$

$$s(\kappa) = \begin{cases} (0.038 - \kappa)/0.959 & \kappa < 0 \\ \max\big((0.008 - \kappa)/0.630,\ -0.75\big) & \kappa \ge 0 \end{cases}$$

| Parameter | Car 20 | Why |
| --- | --- | --- |
| $u_0$, $K$ | 0.48, 9.18 | Off by > ≈ 0.03 and the old controller fails (7.3); changes with car and battery |
| $k_p$ | 0.15 | 0.2 s delay and τ 2.7 s: more gain overshoots |
| $k_i$, $I_{max}$ | 0.04, 0.2 (fix: 0.2, 0.3) | How fast a map error is removed |
| `speed_feedback_min` | 0.4 m/s | Wheel speed unreliable below this, so PI is off |

- **No D term:** wheel speed is quantised (a staircase), so a derivative would amplify the steps. Anti-windup: no
  integration while clipped; reset at a 0 target; a brake request bypasses the PI.

| Integral rule | Integrates when | Effect |
| --- | --- | --- |
| Before (`dev`) | $\lvert e\rvert$ < 0.2 m/s | An overspeed > 0.2 or a car standing still (e = 0.5) freezes the integral; only P acts |
| After the fix | e < band, or above target, or wheels < 0.05 m/s | Overspeed always pulled down; a car that hasn't started always pushed up |

### 7.3 Simulation (S3)

**For which feedforward errors does the speed controller still work? (26 Oct, re-run 5 Nov)** `sim_speed_integral.py`:
car 20's model plus a breakaway threshold, a copy of the controller at 10 Hz; true $u_0$ swept 0.20–0.60 with the
feedforward fixed at 0.48.

![FF error sweep](chris/figures/04_ff_error_sweep.png)

- Old rule reaches 0.5 m/s only for errors −0.02 to +0.03; the fix across −0.12 to +0.28 (settles 3.5–7.6 s). 26 Oct
  breakaway case: old never starts, fix settles in 5.3 s. **Trade-off:** with a correct feedforward the fix overshoots
  more (0.62 vs 0.53 m/s), as the car did. → $k_i$ 0.2, $I_{max}$ 0.3, feedforward back to 0.48.
- **Can't conclude:** no stick-slip randomness or battery drift; τ and delay not swept; a copy of the code.

**Why did the car stop beside the first of two boards? (15 Oct video)** Repo closed-loop avoidance test, 1.0 m lane,
boards 0.2 m in, spacing 0.8–2.5 m.

![Two boards](chris/figures/06_two_boxes.png)

- Bug 1: the obstacle check band was widened on both sides by the car's 0.064 m offset, so it touched board 1 → widen
  only on the car's side. Bug 2: after locking, the line re-centred on board 1's gap → keep the current line while it
  clears both. All spacings pass with ≥ 0.11 m clearance; regression test fails before, passes after (#23).

**How close can two obstacles be? (30 Oct)** Sweeps on `obstacle_simulation.py`, mirrored boards, 6–8 seeds per point.

![Spacing](chris/figures/07_spacing.png)

- The key variable is the lateral shift between the passing lines. Raising the left limit 0.47 → 0.93 alone changes
  nothing on a straight, because the planner's symmetric 0.45 cap binds first. Left bends are worst (3 m radius:
  ≈ 3 m needed vs 1.6 m right, 1.2 m straight). Per-side caps (prototype, not merged): 2.25 → 2.0 m straight.
- **Failure found:** an unconfirmed obstacle inside stopping distance plus lane loss in the same step latches a
  coasting lane-loss stop; the car hits the board in sim.

### 7.4 On the car (S4)

**Drive → speed and steer → curvature (car 8 29 Sep; car 20 5 Oct, with Yifei).** One drive level per run, steer 0,
7–8 m straight, everything bagged; dead-band sweep 0.50–0.60 in 0.02 steps. Safety stops: 1.5 m/s, max distance,
stall 0.6 s, IMU > 3.5 m/s². Controlled: same straight and direction, trim fixed, battery checked by repeating 0.75 at
start and end (1.19 vs 1.14 m/s at 2 s). 0.80–1.00 dropped (0.85 hit 1.5 m/s in 1.8 s). Steering: PI at 0.5 m/s, one
level per run, κ averaged over the last 3 s; trim −0.18 first; ±0.5 repeated after a reboot within 3%.

![Car 20 maps and PI](chris/figures/03_car20_maps_pi.png)

*(a) drive → speed (shaded: no motion; line: model). (b) steer → curvature per side. (c) step to 0.5 m/s, PI v1 vs v2.*

| PI version | $k_p$ | $k_i$ | Overshoot | Steady error at 0.5 m/s |
| --- | --- | --- | --- | --- |
| v1 | 0.10 | 0.07 | 27% | – |
| v2 (#17) | 0.15 | 0.04 | 15% | ≈ 0.013 m/s |

**Speed fix (car 20, 26 Oct).** Before: 0.87 m/s steady for a 0.5 target; with feedforward 0.15 the car didn't move
(drive 0.28; it needed ≈ 0.60–0.68 that day). One no-lane frame at standstill also latched LANE_LOSS (plan length =
measured speed × step = 0) → plan length ≥ target speed.

| Run | Moving after | Reached 0.5 m/s | Peak | Before stop |
| --- | --- | --- | --- | --- |
| 15:20:27 | 0.8 s | 1.9 s | 0.62 m/s | 0.50 m/s |
| 15:21:42 | 0.4 s | 3.2 s | 0.57 m/s | 0.54 m/s |
| 15:23:01 | 0.9 s | 1.8 s | 0.60 m/s | 0.52 m/s |

Limits: 3 runs on one day; if the encoder fails while moving, the standstill integral could push drive to 0.75
(needs a time limit).

**Avoidance on car 20 (19 Oct).** Still braked beside board 1: as the nearest blue cone left the view the lane estimate
jumped 0.35 m in 40 ms (2.4), putting board 1 in the path; in the raw lidar frame it was 0.19 m clear. Fix (local):
within 0.6 m of the front, block only if lane and lidar frames agree; replay: runs 1 and 3 false-stop on old code,
pass on new. Ruled out dropped commands (2145 reached the Traxxas, ≈ 50 ms lag). Measured: lidar 0.30 m and front
0.40 m ahead of the rear axle (config had 0.30).

**Is the left saturation mechanical? (30 Oct stand test).** RC turned ≈ 2× further than autonomous. With the interval
at ±0.8, applied = trim −0.25 + 0.8 s, clipped at −0.8: left travel +60%. The ground re-map wasn't done, so the 0.91
1/m left limit used for turns (5.3) is unmeasured (0.6).

**Lessons:** a feedforward tuned on one day isn't a car property (breakaway 0.52 → ≈ 0.68 in three weeks), so feedback
must absorb it. Not measured: braking, PI at other speeds, car width, the left limit at ±0.8.
