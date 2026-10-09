# Low-level control

The last stage of the policy (`policy_runner.py`). It turns the movement policy's instruction into the car's
normalised actions. Code: [control.py](control.py); settings: `control:` in
[config/ai4r_policy.yaml](../../config/ai4r_policy.yaml); tests: [tests/policy/test_control.py](../../tests/policy/test_control.py).
It has no ROS imports, so it runs in pytest and dream-gym.

## Interface

**Input:** the last `DriveCommand` in the policy's instructions ([actions.py](actions.py)):

| Field | Units | Meaning |
| --- | --- | --- |
| `speed_m_per_s` | m/s | Forwards speed. `0.0` (or less) stops: drive 0 and the integral is reset. |
| `curvature_per_m` | 1/m | 1 / turning radius. **Positive turns left**, negative right, 0 straight. |

No `DriveCommand` at all also stops the car with the steering centred. `MoveCameraTo` is not handled yet, so the
camera pan stays `None` (no target sent).

The controller also reads `observations.car.wheel_speed` (unsigned m/s) and `observations.policy.dt` /
`is_first_policy_step` (the integral resets at the start of every run).

**Output:** `CarActions` with

| Field | Range | How it is calculated |
| --- | --- | --- |
| `drive_action` | `[0, max_drive_action]` | `speed_ff_offset + v_ref / speed_ff_gain + speed_kp * e + I`, with `e = v_ref - wheel_speed` |
| `steering_action` | `[-max_steering_action, max_steering_action]` | `steering_centre_action + slope * curvature`, with a separate slope for left and right |

Details of the speed controller:
- Feedback (P and I) is only used for targets `>= speed_feedback_min_m_per_s` (0.4 m/s); below that the drive is
  feedforward only, because the encoder-based wheel speed lags by seconds at low speed.
- The integral only accumulates while `|e| < speed_i_band`, is limited to `+-speed_i_max`, and stops integrating
  into the drive limit (anti-windup).
- If feedback is needed but the wheel speed is missing or expired, the drive is **zero**. Put `wheel_speed` in
  `required_sensors` when driving with this controller.
- Drive is forwards only. Wheel speed is unsigned, so reverse cannot be controlled.

## Car 20 calibration (5 Oct)

These are the defaults in `ControlConfig` and the YAML. Every car is different; measure a new car with
[docs/control/car-calibration.md](../../docs/control/car-calibration.md).

| Quantity | Value | Source |
| --- | --- | --- |
| Speed map | `v_ss = 9.18 * (drive - 0.475)` m/s | open-loop speed map, `c20_speedmap` |
| `speed_ff_offset` / `speed_ff_gain` | 0.48 / 9.18 | fit, offset rounded up by the PI's integral correction |
| `speed_kp` / `speed_ki` / `speed_i_band` | 0.15 / 0.04 / 0.2 | PI step to 0.5 m/s: 15 % overshoot, steady error about 0.01 m/s (`c20_pi05`, `c20_pi05_v2`) |
| Open-loop dynamics | first order, tau about 2.7 s, delay about 0.2 s | hold at least 8 s to measure a steady speed |
| Breakaway drive | about 0.52 to 0.53 | static friction; targets below 0.4 m/s may not start the car |
| Steering trim (vehicle interface) | **-0.18** | must be set again after the main board restarts (it resets to 0.0) |
| Steering sign | **positive steering turns right** | debug2 curvature check |
| Right map (steering >= 0) | `curvature = -0.959 * steering + 0.038` | `c20_steer`, at 0.5 m/s, trim -0.18 |
| Left map (-0.75 <= steering < 0) | `curvature = -0.630 * steering + 0.008` | saturates beyond -0.75 |
| Minimum turning radius | left about 2.1 m (saturated), right about 1.1 m (steering 1.0) | |
| Wheelbase | not measured | |

The inverse map used here: `steering_left_action_per_curvature = -1/0.630 = -1.587`,
`steering_right_action_per_curvature = -1/0.959 = -1.043`, `steering_centre_action = 0.02` (both fits give
roughly zero curvature there).

**Note for the MPC:** `mpc.max_curvature_per_m` defaults to 3.0 1/m, but car 20 can only reach about 0.48 1/m left
and 0.92 1/m right. Larger requests saturate the steering, so the MPC's prediction will be wrong in tight turns
until its limit is lowered.

## System identification tools and records

The open-loop sysID sequencer (timed drive/steer segments with speed, distance, stall and impact guards) is in the
GitLab `ai4r_policy` repository on branch `feature/sysid-test-sequencer`; it is **not** ported here. This repository
keeps the analysis tools and the records (in Chinese):

| File | Contents |
| --- | --- |
| [tools/record_run.ps1](../../tools/record_run.ps1) | One-command `ros2 bag record` on the car over ssh, copy back and analyse |
| [tools/sysid_analyze.py](../../tools/sysid_analyze.py) | Dependency-free MCAP analysis: per-segment speed, curvature, step response and speed-map fit |
| [docs/control/car-calibration.md](../../docs/control/car-calibration.md) | Per-car calibration procedure and the results table (car 8, car 20) |
| [docs/control/sysid-test-plan.md](../../docs/control/sysid-test-plan.md) | Original test plan and car 8 results |
| [docs/control/run-checklist.md](../../docs/control/run-checklist.md) | Step-by-step checklist for one recorded run |
| [docs/control/code-walkthrough.md](../../docs/control/code-walkthrough.md) | Walkthrough of the policy node (v0.1.0) |

Rosbags are not committed (`bags/` and `*.mcap` are ignored).
