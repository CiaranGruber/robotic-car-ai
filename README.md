# ai4r_policy

Student ROS 2 policy package for the DREAM robot, targeting Ubuntu 24.04 and
ROS 2 Jazzy. Version 0.2.0 adds ArUco observations to the first source release;
a published annotated tag and successful release CI establish release identity.
It is not a physically qualified driving controller.
[Acceptance](docs/acceptance.md) records observed software checks and remaining
robot checks, and [the v0.2.0 release preparation](docs/release-v0.2.0.md)
defines promotion, tag, artifact, and publication evidence.

**Start with [scripts/policy_node.py](scripts/policy_node.py)** and its
`INSERT POLICY CODE` section. It contains the observation/action explanations
and a zero-action starter. The [policy YAML](config/ai4r_policy.yaml) explains
every setting and includes lidar-only and wheel-speed-controller examples.

| Update mode | Executes your policy when |
| --- | --- |
| `cone_detection` | A fresh cone-detection batch arrives |
| `fiducial_detection` | A fresh fiducial-detection batch arrives |
| `lidar` | A fresh lidar scan arrives |
| `timer` | The configured policy timer runs |

Other callbacks cache observations. Declare `required_sensors` for your exercise;
missing/stale required data stops the policy. Required cones also have an
independent empty-detection timeout. Recovery needs an explicit start request.
Cone batches carry `acquisition_to_publish_latency_s`, measured from OAK-D's
end-of-exposure timestamp until immediately before the detector's `publish()`
call. Exposure duration is excluded; this is not exposure-start-to-publication
latency. The student starter exposes
`cone_acquisition_to_publish_latency_s` and `cone_measurement_age_s` alongside
the existing coordinate/colour/confidence lists. The latter is the current age
from the same end-of-exposure acquisition reference, already including latency;
never add the two together.
Internally, `observations["cone_detections"]` is now a batch dictionary with
`detections` (the existing five-value tuples) and
`acquisition_to_publish_latency_s`, or `None` when unavailable/stale. This
replaces the previous raw-list value; the student function signature is unchanged.
Rebuild the OAK-D producer and policy against the same updated cone message.
Fiducial batches include dictionary/source-frame metadata and marker records;
valid poses are transformed into `policy_frame_id` at image acquisition time.
A fresh empty marker list is valid data, with no automatic visibility timeout.
The exact student Python record fields are documented beside
`calculate_policy_actions` in the policy script. A valid pose is a successful
geometric solve, not proof that the marker is real or safe to drive toward.
Your policy selects expected IDs and evaluates distance, reprojection error
and consistency across observations; background false detections are possible.

## Build and run

The robot must first have a compatible DREAM underlay and a built student
overlay at `~/ai4r_student_workspace`. In particular, `dream_interfaces` v0.2.0
must provide `FiducialDetections` and the units-bearing `DriveAndSteer`;
older definitions are incompatible. The compatible dependency is pinned in
[ci/dependencies.repos](ci/dependencies.repos). Staff/developer standalone build
and test instructions are in [CONTRIBUTING.md](CONTRIBUTING.md).

For managed DREAM runs, keep this package in the external student workspace and
set `~/.config/dream/ai4r.yaml` to:

```yaml
student_workspace: ~/ai4r_student_workspace
```

DREAM uses that workspace's installed overlay only for `ai4r_policy`. Its
[AI4R runtime guide](https://gitlab.unimelb.edu.au/dream/dream_system/-/blob/main/docs/ai4r-runtime.md)
describes the explicit build, source-YAML validation and restart commands.

With the compatible underlay and student overlay sourced:

```bash
ros2 launch ai4r_policy ai4r_policy.launch.py namespace:=car
```

The launch supports optional `namespace` and `params_file` arguments. Shipped
policy settings load first, then the explicit overlay. It starts only the
`ai4r_policy` node and does not respawn it. DREAM owns hardware services and
mounting transforms separately. Its camera transform describes the fixed
zero-pan pose; physically panning the camera invalidates that transform until a
measured dynamic TF is supplied.

The policy starts in state **2: publishing zero drive and steering**. It holds
the pan target by sending no pan command. Vehicle Enable/Disable/Disarm is
separate: wait for the vehicle's Enabled state after its explicit Enable request,
then request policy state 3. Ongoing zero commands satisfy the policy side of
the vehicle's fresh-zero pre-enable handshake. The policy never enables it.

```bash
# Start policy calculations after required sensors and the vehicle are ready:
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'
# Stop policy driving actions and publish zeros continuously:
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 2}'
```

State **1** ceases all action publication; it is neither Disarm nor an immediate
vehicle stop. A blocked Python calculation also blocks this node's watchdog;
the independent vehicle command watchdog remains necessary. Sending a zero
command does not prove physical stopping or take control away from manual RC.

## Interfaces and configuration

All names below are relative to the selected namespace. The fixed node name is
`ai4r_policy`. Sensor subscriptions keep the latest observations; IMU fields have
independent validity/freshness. Cone positions are metres in `policy_frame_id`
(normally `base_link`). DREAM places the normal `base_link` origin on nominal
ground directly below the agreed vehicle centre of gravity: +x forwards, +y
left, +z up. A cone position's `z` is the detected point's nominal height above
that ground plane, not the cone's total height. The fixed-pose transform does
not compensate for terrain or vehicle pitch. Raw lidar remains in its message
frame with its existing origin; Cartesian points use mounting TF to reach the
body frame. Body IMU vectors use mounting TF, and relative
heading resets only when entering policy state 3.

Every accepted scan prepares `lidar_scan` and attempts `lidar_cartesian` in the
subscriber callback, before a lidar-triggered policy step. There is no conversion
switch. Students receive `lidar_ranges`, `lidar_intensities`, `lidar_points_xyz`
and `lidar_scan_indices`, with separate `lidar_scan_available` and
`lidar_cartesian_available` flags. Cartesian points are finite, in-range `(x,y,z)`
tuples in metres; index `j` corresponds to raw ray `lidar_scan_indices[j]`.
The Cartesian dictionary contains `points_xyz`, `scan_indices` and `frame_id`.
Missing TF or failed conversion leaves the raw scan usable and immediately
discards the previous Cartesian result. A fresh empty Cartesian list is valid.
Snapshots retain matching acquisition stamps and cannot expose Cartesian data
beyond either its own timeout or the raw scan timeout. One nonblocking transform
lookup uses the first ray's timestamp for the entire scan; there is no per-ray
motion correction or IMU levelling.

**Configuration migration:** rename the old observation/required-sensor name
`lidar` to `lidar_scan`, and `sensor_timeout_s.lidar` to
`sensor_timeout_s.lidar_scan`. The update mode remains `lidar`. That mode must
require `lidar_scan`, `lidar_cartesian`, or both. Cartesian driving policies
should require `lidar_cartesian`; its timeout defaults to 0.5 s. Timer and other
sensor modes reuse the same cached representations and freshness handling.

| Topic | Type | Direction / QoS |
| --- | --- | --- |
| `cone_detections` | `dream_interfaces/ConeDetections` | Input; reliable, depth 1 |
| `fiducial_detections` | `dream_interfaces/FiducialDetections` | Input; reliable, depth 1 |
| `scan` | `sensor_msgs/LaserScan` | Input; best effort, depth 1 |
| `wheel_speed_m_per_sec` | `std_msgs/Float32` | Input; reliable, depth 1; unsigned m/s |
| `imu/data` | `sensor_msgs/Imu` | Input; best effort, depth 5 |
| `rc_drive_and_steer_normalized` | `dream_interfaces/DriveAndSteer` | Input; reliable, depth 1; operator RC sticks, normalized; stored only, never a trigger |
| `policy_fsm_transition_request` | `std_msgs/UInt16` | Input; reliable, depth 10; states 1/2/3 |
| `drive_and_steer_set_point_normalized` | `dream_interfaces/DriveAndSteer` | Output; reliable, depth 1; explicit normalized units |
| `pan_set_point_normalized` | `std_msgs/Float32` | Output; reliable, depth 1; optional normalized target |
| `policy_fsm_state_value`, `policy_fsm_state_string` | `std_msgs/Int8`, `std_msgs/String` | Output; reliable, depth 10; current state and reason |
| `imu_heading_angle` | `std_msgs/Float32` | Output; reliable, depth 10; valid tared heading in degrees |
| `debug1`, `debug2` | `std_msgs/Float32` | Output; reliable, depth 10; optional student scalars |
| `policy_visualisation/markers` | `visualization_msgs/MarkerArray` | Output; reliable, depth 10; the policy's cones, lane, obstacles and plans in `policy_frame_id`, at most `visualisation.publish_rate_hz` (`docs/foxglove/live_layout.json`) |

All QoS is volatile. Standard ROS parameter/introspection services and TF
subscriptions also exist. Finite actions are clipped to `[-1,1]`; invalid
outputs or student exceptions stop the policy. A pan target of `None` holds;
zero recentres. Drive is motor effort, not a speed setpoint. Full meanings and
examples are beside the variables in the Python file.

Five commented ROS parameter files are installed:

- [ai4r_policy.yaml](config/ai4r_policy.yaml): triggers, required sensors and timing.
- [traxxas_vehicle_interface.yaml](config/traxxas_vehicle_interface.yaml): editable steering limits/trim, timeouts, slew rates, RC calibration and wheel/encoder settings; commented identity and pan-joint references.
- [oakd_cone_detector.yaml](config/oakd_cone_detector.yaml): perception and debug settings.
- [bno08x_imu_interface.yaml](config/bno08x_imu_interface.yaml): selected IMU products and accuracy.
- [aruco_detector.yaml](config/aruco_detector.yaml): dictionary, marker sizes and detection filtering.

ArUco filtering runs in `aruco_detector`, before observations reach the policy.
Keep the shipped `allowed_marker_ids: []` so unsupported IDs reach the policy's
stop check as well as direction IDs. `min_consecutive_frames: 3` requires an
ID in three consecutive processed images before publishing the current result,
adding about 0.2 s at 10 Hz. Set it to `1` to disable confirmation. A missed
detection removes the ID immediately; reacquisition must confirm again. A gap
over 0.5 s between image acquisitions resets confirmation, so rates below 2 Hz
require confirmation to be disabled. This filters IDs, not individual copies
of the same ID, and cannot exclude a persistent false detection of an allowed
ID. Repeated-ID observations remain separate and poses are never replayed.
An image with no accepted markers still supplies a fresh empty batch; the
student continues to decide the driving response to marker absence.

This branch maps **IDs 1–4 to left, 5–6 to right and 7–8 to straight at the next
intersection**, using `markers.turn_left_ids`, `turn_right_ids` and `straight_ids`.
Every other confirmed, posed marker ahead within `markers.max_distance_m` stops
the current run, including during a turn. The stop remains after the marker
disappears or a valid marker appears; only explicitly starting a new run clears
it. A confirmed valid direction is remembered when its marker leaves view.
The legacy scalar/keep-side assignments are disabled in the shipped configuration
and remain available for replaying older settings.
The current `intersection` manoeuvre is enabled in the shipped settings. It
places the junction from the marker's position, retains the approach direction,
follows a planned turn and searches for the outgoing lane within a bounded
distance; it stops if that exit lane is not found. See
[marker commands](docs/marker-commands.md).

The separate experimental `intersection_navigator` connects a confirmed
command to a finite path through an **equal-width perpendicular cross or T-junction**.
It estimates incoming rows from motion-compensated cone tracks and uses current
lidar boundary evidence for the requested exit. Supported closing rows block a
direction; missing returns leave it unknown. The existing
MPC steers through the turn, then fresh outgoing boundaries and heading alignment
admit handoff to normal lane following. Camera pan and steering limits are unchanged;
no trained model is used. It is **disabled by default and locally simulated only**.

With that experimental navigator, the ArUco command selects the **formed lane centre path** once sensor evidence
admits an exit. IDs 1–4 connect the approach to the left exit, 5–6 to the right,
and 7–8 to the straight exit. A sign ahead of an ordinary straight or curved
road only queues the command; its measured lane fit and normal control remain
the source, subject to command expiry and sensor-stop checks. At a sensed junction,
fresh incoming cone geometry plus current paired lidar rows can support a slower,
finite straight approach while exit evidence is still unknown. This path ends
before the road mouth, keeps the command pending and uses the configured stopping
envelope; its endpoint cannot advance with repeated predictions. During that
approach or a committed junction, `policy.selected_lane` supplies both the centre
path drawn in Foxglove's `lane` layer and the curvature reference used by MPC.
Measured acquisition-time edges remain separate in the faded `lane/observed`
layer. The selected path is cleared on stop/completion; fresh outgoing cone
support admits normal lane following with observed colour sides (`AUTO`).

Run the turning scenarios, including cone loss triggered during the actual turn:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.intersection_turning --output build/intersection-turning-20261026
```

It runs 22 controller-driven cases: left/right on a 2.4 m cross at .2/.4 m cone
spacing with nominal/noisy detections, a missing blue boundary, an .8 s complete
camera gap and missing corner cones, plus nominal/noisy straight commands.
It writes `turning.gif`, `turning.png`, `signal_selection.png`, detailed frames,
trajectories and a source-hashed report. Only the local simulation enables the
optional navigator; this command neither enables nor contacts a physical car.
See the [turn-selection evidence](docs/acceptance.md#aruco-selected-lanes-and-turning-scenarios-2026-10-26).

To increase the distance **along each cone row**, keeping the road 2.4 m wide:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.intersection_turning --cone-spacings 0.5 0.6 --plot-spacing 0.6 \
  --corner-anchored --output build/intersection-spaced-cones-20261026
```

Distances are between cone centres. Corner anchoring preserves the four corner
positions and spaces cones symmetrically outwards, so a wider gap does not change
the junction opening. The default command retains the earlier layout for
reproduction. `cone_layout.png` compares the requested gaps; the other figures
show the selected .6 m cases. This sparse matrix currently stops on approach
lane loss in all 22 cases, before a route forms; it does not establish successful
turning or recovery from turn-triggered camera loss. See the
[sparse-cone evidence](docs/acceptance.md#sparser-cone-rows-2026-10-26).

Run all three T orientations (no straight road, no left road or no right road):

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.t_intersection_simulation --output build/t-intersection-20261026
```

The same physical layout receives IDs 3/4/5 separately. Available exits must
reach the requested arm and complete outgoing-lane handoff; a command toward
the closing row must stop without selecting another exit. The 48-case matrix
includes noisy detections, finite turn-triggered cone loss, and .5/.6 m sparse
approach probes. It writes `t_intersection.png`, `t_intersection.gif`, detailed
frames/trajectories and a source-hashed report. These are local decoded-sensor
simulations; the GPU model and car are unchanged. See the
[T-junction evidence](docs/acceptance.md#t-junction-exit-support-2026-10-26).

Run the local navigation evaluation with:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.intersection_simulation --navigate --output build/aruco-navigation-final
```

It writes `report.json`, trajectory CSVs and `intersection.png` using a 2.4 m-wide
cross and .2 m cone spacing. Omit `--navigate` to reproduce the earlier marker-only
1 m-cross baseline. These are decoded marker poses, not camera-pixel recognition.
The navigator's parameters and required gyro/wheel/lidar/empty-camera settings are
documented beside `intersection.enabled` in `config/ai4r_policy.yaml`. See the
[navigation evidence](docs/acceptance.md#aruco-intersection-navigation-prototype-2026-10-23)
for limits and checks. That simulation evaluation did not apply car settings;
the subsequent authorised car deployment is recorded in
[the deployment evidence](docs/acceptance.md#car-104325433-intersection-deployment-2026-10-23).

To inspect the current cone filter and lane fitter independently of vehicle control:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.intersection_identification --output build/intersection-association-20261026
```

This writes an animated top-down view, a six-frame storyboard, frame details and
a source-hashed report. It uses synthetic decoded cones during a centred straight
passage through a four-way cross, including noise and missing boundaries. It also
compares the same straight rows with cross-road cones removed. This diagnoses
lane association; it does not test GPU image recognition or successful driving.
Add `--baseline build/intersection-identification-20261026` when the earlier local
capture is available to generate `comparison.gif` with matching before/after
poses. `stages.png` shows decoded cone positions, checked/grouped cones and the
resulting lane separately.

The GPU detector remains the source of cone measurements. The policy rejects
non-finite positions/confidence and resolves equal-confidence duplicate detections
consistently. `lane_detection.associate_branches` then separates supported incoming
straight rows from transverse rows before lane formation. Coherent circular rows
with a plausible forward tangent keep the existing arc fit and outlier rejection.
A T's transverse segment may share a corner with paired incoming rows; a
near-transverse tangent cannot win solely because fitted offsets straddle the car. All checked
cones remain available for obstacle and intersection processing; selecting lane
members does not delete a side road. Uncertain transverse-only observations can
return no lane rather than a confident sideways lane. This is a geometric
heuristic; noisy/sparse observations can still produce an incorrect fit.
See the [updated association evidence](docs/acceptance.md#cone-grouping-and-lane-formation-update-2026-10-26).

The optional [camera mount file](config/camera_mount.yaml) is also installed,
but it is DREAM source configuration, not ROS parameters. The shipped `{}` uses
the defaults: camera optical centre `0.30 m` above nominal ground, `0.10 m`
behind the CG (`forward_from_cg_m: -0.10`), centred at `y = 0`, and tilted
`20 degrees` down. It accepts only `height_above_ground_m` (finite and
nonnegative), `forward_from_cg_m` (finite and signed), and
`downward_tilt_deg` (finite, strictly between `-90` and `90`, positive down).
Fields may be omitted to inherit their defaults; a missing file also uses all
defaults.

The optional [lidar mount file](config/lidar_mount.yaml) likewise ships `{}` to
inherit DREAM's baseline: scan origin 0.20 m forward of the CG, centred laterally,
0.12 m above nominal ground, level with 180-degree yaw. It admits
`forward_from_cg_m`, `left_from_cg_m`, `height_above_ground_m`, `roll_deg`,
`pitch_deg` and `yaw_deg`. Distances are finite metres, height is nonnegative,
and angles are finite degrees in [-180, 180]. Rotations are absolute,
right-handed `Rz(yaw) * Ry(pitch) * Rx(roll)` from the lidar frame into
`base_link`, not offsets added to the default. The file documents signs and
measurement references. Apply edits using `dream runtime restart rplidar_c1`;
policy restart alone does not reload the pose. Invalid settings leave the
current lidar run intact. Cartesian points use the updated TF; raw scan ranges
are unchanged. Install the matching DREAM lidar-mount support first; older
DREAM versions ignore this optional file. No dynamic/IMU compensation is added.

Policy settings are startup-only. DREAM reads the current source YAML under
`~/ai4r_student_workspace/src/ai4r_policy/config/` on each new start or restart,
so editing these files needs no student-workspace rebuild. Restart only the
affected unit: `dream runtime restart ai4r_policy` for policy settings, or
`dream runtime restart traxxas_vehicle_interface`, `oakd_cone_detector`,
`bno08x_imu_interface`, or `aruco_detector` for that component's YAML.
Camera-mount changes take effect through `dream runtime restart oakd_cone_detector`. DREAM validates and
snapshots the effective camera configuration before stopping a running detector;
malformed YAML, unknown keys, or invalid values therefore leave it running with
its previous snapshot. An idempotent `dream runtime start oakd_cone_detector`
does not restart or reload an already-running unit. Restarting policy alone does
not reload a hardware unit. The standalone policy launch loads only its own ROS
parameter file and never consumes `camera_mount.yaml` or `lidar_mount.yaml`; DREAM admits component
requests before passing them to independently launched units. The active vehicle
settings override DREAM's baselines; use measured calibration and geometry for
your car. Omitted settings inherit the baseline. This requires DREAM's expanded
Traxxas delegation; older versions reject the newly exposed settings. Commented
reference values remain system-owned. See the [runtime guide](https://gitlab.unimelb.edu.au/dream/dream_system/-/blob/feature/classroom-ros-environment/docs/ai4r-runtime.md)
for the complete source-YAML workflow.

For this physical-car setup, select **`oakd_gpu_cone_detector`**. Keep the CPU
`oakd_cone_detector` unit stopped so it does not compete for the camera. Start
the GPU unit with `dream runtime start oakd_gpu_cone_detector`; the management
helper's `start_runtime` also selects GPU, stops an active CPU unit first, and
aborts if that stop or GPU startup fails. It starts the IMU and lidar alongside
the vehicle interface, because intersection navigation requires fresh gyro,
wheel-speed and Cartesian lidar observations. The CPU component's YAML namespace
and configuration contract above remain specific to that component; selecting
GPU does not rename them or change camera/vehicle calibration.

ArUco needs the GPU's clean `camera/rgb/image_rect` and matching
`camera/rgb/camera_info` streams. On the deployed car, DREAM's optional source
override `config/oakd_gpu_cone_detector.yaml` enables `publish_rgb_image: true`;
its inherited settings remain 1280x720 at 10 Hz. This is independent of cone
debug images, which may stay off. The car-only override is retained on the car;
the repository's five ROS parameter files are unchanged by that addition.

Direct-node equivalent for integrations that need one (Python ROS launch):

```python
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from pathlib import Path
policy = Node(
    package="ai4r_policy", executable="policy_node.py", name="ai4r_policy",
    namespace="car", output="screen", respawn=False,
    parameters=[str(Path(get_package_share_directory("ai4r_policy")) /
                    "config" / "ai4r_policy.yaml")],
)
# Append one explicit policy overlay after the shipped file if needed.
```

## Integration status and development

This is the standalone source repository. DREAM's AI4R integration defines the
managed policy and component runtime units and admits source-YAML overrides.
Its `base_link` origin is nominal ground directly below the agreed CG. The
default zero-pan camera optical pose is `[-0.10, 0.00, 0.30]` m relative to that
origin, with a 20-degree downward tilt; the optional camera mount file overrides
height, forward offset and tilt. The nominal IMU translation now has
`z = 0.10 m` in this ground-level frame, with its existing rotation unchanged.
Sensor observations retain their contracts except that transformed
cone `z` is rebased to the ground-level origin; raw lidar data is unchanged.
Physical pan motion requires a measured dynamic transform before cone positions
can be trusted. The fixed flat-ground pose does not compensate for terrain or
vehicle pitch, and physical frame validation remains separate. The policy does
not own those services, perform hardware discovery, or install student
dependencies.

`dev` is the active development branch; `main` is the release-ready line.
Annotated immutable tags identify candidates and releases. Use feature branches
and reviewed MRs; safety, public-interface and CI-policy changes require recorded
human review.
See [contribution checks](CONTRIBUTING.md), [change history](CHANGELOG.md),
[acceptance](docs/acceptance.md), and the [MIT license](LICENSE).

Traxxas output mapping and final PWM bounds are documented beside the eight
integer microsecond settings in `config/traxxas_vehicle_interface.yaml`. DREAM
admits and validates them before vehicle-interface restart. They affect manual
RC fallback too; restarting only the policy does not reload them. Matching
firmware/interface 0.5.0 is required.
