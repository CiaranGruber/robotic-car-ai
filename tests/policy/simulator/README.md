# Car simulator and live 3D viewer

A real-time 3D simulator of the car for developing and testing the policy without the car, and the same 3D viewer
for watching the real car's surroundings when it is connected. Both open in your browser.

- **Without the car:** your local policy code (`MovementPolicy`, exactly as `policy_node` calls it) drives a
  simulated car among 3D cones, ArUco signs and boxes. The sensors, timing and node rules come from the repository's
  settings and measurements.
- **With the car:** the viewer reads the car's live streams through its Foxglove bridge, read-only. It rebuilds the
  car's surroundings in 3D from its detections (cones, decoded ArUco markers, lidar), with the car's own lane and
  plan drawings, seen from any view including the car's camera.

## Run it without the car

From the repository root, with the project's `.venv`:

```bash
.venv/bin/python -m tests.policy.simulator cross --marker-id 3
```

Your browser opens `http://localhost:8000`. Press Ctrl+C in the terminal to stop.

Other examples:

```bash
.venv/bin/python -m tests.policy.simulator lab_straight --loop
```

```bash
.venv/bin/python -m tests.policy.simulator curvy_road --speed 2 --set mpc.target_speed_m_per_s=0.4
```

```bash
.venv/bin/python -m tests.policy.simulator my_track.yaml --camera-yaw-deg 7 --miss 0.2
```

| Option | What it does |
| --- | --- |
| `TRACK` | A bundled track name (below) or a track file. Default `lab_straight` |
| `--marker-id N` | The ID printed on the track's first sign, for example 3, 4 or 5 at the cross |
| `--set NAME=VALUE` | Any policy or node setting by full name, on top of `config/ai4r_policy.yaml` and the track. Repeatable |
| `--speed X` | Simulated seconds per real second (also in the viewer) |
| `--loop` | Start again a few seconds after each run ends |
| `--camera-yaw-deg D` | How far left the simulated camera really points; the policy only corrects `mpc.camera_yaw_offset_deg` |
| `--miss P`, `--no-noise`, `--seed N` | Cone misses per frame, exact sensors, and the noise seed |
| `--car-model car20/car22a/car22b` | The measured car response from `car_models.py`. Default `car20` |
| `--headless` | No browser: run once as fast as possible and print what happened |
| `--foxglove [PORT]`, `--record FILE.mcap` | Also serve the run to Foxglove, or record it for Foxglove |
| `--host 0.0.0.0` | Let others on the network open the viewer |

### In the viewer

| Control | Key |
| --- | --- |
| Pause or resume | Space |
| Restart (or clear the remembered scene with the car) | R |
| Chase, car camera, top and orbit views | 1, 2, 3, 4 (drag to orbit in view 4) |
| Car camera inset (the chase view when the main view is the camera) | I |
| Camera view area on the floor, lidar points, policy drawings | V, L, P |
| Drive by hand: the policy still runs and draws, but your keys drive | M, then the arrow keys or WASD |
| Track, first sign's ID and speed | The menus on the right |

The car camera view is drawn from the camera mount's defaults (`config/camera_mount.yaml`: 0.10 m behind
base_link, 0.30 m up, 20 degrees down) with the detector's measured 86 degree view. It leaves out the car's own body
so the surroundings stay visible. The left panel shows the node state (as `policy_fsm_state_string` would),
the lane the policy found, its errors, the sign commands and the junction phase. Under "Drawing layers" you can hide
each of the policy's drawing layers, which are the same ones `policy_node` publishes on the car.

## Run it with the car

The laptop must reach the car (the university VPN on this Mac). The car's `foxglove_bridge` listens on port 1234; check
on the car with `ss -ltnp | grep foxglove`.

```bash
.venv/bin/python -m tests.policy.simulator --car 10.43.254.26:1234
```

It only subscribes: nothing is published, no service is called and no parameter is set, so it cannot move the car
or change its settings. It reads `cone_detections`, `fiducial_detections`, `scan`, `wheel_speed_m_per_sec`,
`imu/data`, `policy_fsm_state_string`, `drive_and_steer_set_point_normalized`, `policy_visualisation/markers` and
`/tf_static`. The camera and lidar mount transforms come from `/tf_static`, or the mount defaults if it has none.

- Cones are kept for 4 s and markers for 6 s after they were last seen, placed with an odometry pose from wheel speed
  and gyro. That pose drifts; it is only for drawing. "Clear memory" (R) starts again from the car's current place.
- The car's lane, obstacle and plan drawings only appear while its policy runs (state 3).
- The right panel lists each stream's rate. A stream at "none" is not arriving.

## Bundled tracks

On 27 Oct 2026 (commit `d1d8096` plus this simulator), headless runs gave:

| Track | What it tests | Result |
| --- | --- | --- |
| `lab_straight` | The Stage 1 lab lane: 3.2 m, 0.8 m wide, cones every 0.2 m | Stops 0.39 m past the last pair, no contact |
| `curvy_road` | 0.8 m lane bending left (2.5 m radius) then right (2 m) | Drives all 11 m, no contact |
| `obstacle_lane` | A box right of centre halfway along a 5 m lane | Passes it, no contact |
| `aruco_lane` | 1.4 m lane with a keep-left (ID 0) then keep-centre (ID 1) sign | Moves 0.3 m left, then back to the centre |
| `cross` | The 2.4 m four-way cross with one turn sign; navigator on | IDs 3/4/5 take the left/right/straight exits |
| `t_junction` | The cross with its straight arm closed | ID 3 turns left; ID 5 stops: "requested exit is closed" |

The cross and T tracks turn on `intersection.enabled` with the sensors it needs (`imu_angular_velocity` required,
`cone_empty_timeout_s: 4.0`), like the car's own override file.

### Track files

A track is a YAML file in world coordinates: metres, +x east, +y north, angles in degrees anticlockwise from +x.
Copy a bundled track to start.

```yaml
name: My test area
description: What it tests and what should happen.
start: {x: -0.3, y: 0.0, heading_deg: 0.0}   # the car's start
duration_s: 30.0                             # longest run
roads:                                       # any number of these
  - lane:                                    # curved_lanes.CurvedLane
      start: [0.0, 0.0]                      # first cone pair
      heading_deg: 0.0
      width_m: 0.8
      cone_spacing_m: 0.2
      left_colour: BLUE
      pieces: [[straight, 1.5], [arc, 0.4, 90], [straight, 1.0]]   # arc: curvature 1/m (+ left), sweep degrees
  - cross: {centre: [0.0, 0.0], heading_deg: 0.0, width_m: 2.4, arm_length_m: 6.0, cone_spacing_m: 0.2,
            closed_exit: null}               # null, left, right or straight (a T)
cones: [[1.0, 0.9, YELLOW]]                  # loose cones
markers:                                     # ArUco signs, DICT_4X4_50
  - {id: 3, x: -3.8, y: -0.7, facing_deg: 180, height_m: 0.2}   # size_m defaults to config/aruco_detector.yaml
obstacles:
  - {x: 2.6, y: -0.15, length_m: 0.2, width_m: 0.25, height_m: 0.25, yaw_deg: 0}
policy_settings:                             # settings this track needs, by full name
  intersection.enabled: true
```

A sign only gives its command within `markers.max_distance_m` (1.5 m) of base_link and ahead of it, and only once the
camera has seen it in `min_consecutive_frames` frames. So place it where the car passes close by.

## What is simulated, and where each value comes from

| Part | Model | Source in the repository |
| --- | --- | --- |
| Policy | `MovementPolicy`, built as `policy_node` builds it | `config/ai4r_policy.yaml`, the track, `--set` |
| Node | One step per cone batch (`policy_update_mode`), stale sensors become `None`, start refused without fresh required sensors or recent nonempty cones, drop to state 2 at `supervision_rate_hz` | `scripts/policy_node.py`, node settings in `config/ai4r_policy.yaml` |
| Operator | Requests state 3 after 1 s, retrying every 0.5 s until the node starts; never restarts it after a stop | The run checklist |
| Car response | Measured speed and steering maps, kinematic (no tyre slip) | `tests/policy/car_models.py` (car 20 by default) |
| Vehicle interface | Drive and steering slew limits, 0.5 s command timeout | `config/traxxas_vehicle_interface.yaml` |
| Braking | 0.8 m/s² while the drive is negative | NOT measured, as in `intersection_simulation.py` |
| Camera cones | 86 degree view, 0.4-2.5 m, 0.03 m noise, 10 Hz, 0.2 s old; boxes hide cones | `real_car_measurements.LAB_CAR`, `obstacle_simulation.py` |
| ArUco | Decoded poses from the same frames once seen in 3 frames, within the view and within 60 degrees of the sign's face | `config/aruco_detector.yaml`; the 60 degrees is not measured |
| Lidar | 0.5 degree beams to 8 m, 0.01 m noise, 10 Hz, 0.15-0.25 s old; cones and boxes | `obstacle_simulation.lidar_points`, `config/lidar_mount.yaml` defaults |
| Wheel speed and gyro | 50 Hz and 100 Hz; yaw rate is speed times driven curvature | |

## What it does not simulate

- Camera pixels: the GPU cone detector and the ArUco detector belong to DREAM and need the real camera. Both
  simulators feed the policy detections, not images.
- Tyre slip, actuator dead time, battery voltage, uneven floors, lighting, or sensor dropouts beyond `--miss`.
- ROS itself: the node rules are copied from `scripts/policy_node.py`, not run. Use the CONTRIBUTING fast gate on a
  ROS machine for the real node.
- Passing in simulation does not show the car will pass on the floor. Use it to find problems early.

## Files

| File | Contents |
| --- | --- |
| `__main__.py` | The command line |
| `track.py`, `tracks/` | Track files and the world they describe |
| `vehicle.py` | The car: measured response, vehicle interface limits, body |
| `sensors.py` | Camera cones, ArUco, lidar, gyro |
| `simulation.py` | One run: sensor clocks, the node's rules, contacts and when a run ends |
| `server.py`, `viewer/` | The live 3D viewer: web server, page and three.js scene |
| `live.py`, `ros2_cdr.py` | With the car: read-only bridge client and ROS 2 message decoder |
| `scene.py` | Optional Foxglove output (`--foxglove`, `--record`) |
| `../test_simulator.py` | Its tests: `.venv/bin/python -m pytest tests/policy/test_simulator.py` |
