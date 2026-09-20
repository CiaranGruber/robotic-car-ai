# ai4r_policy

Student ROS 2 policy package for the DREAM robot, targeting Ubuntu 24.04 and
ROS 2 Jazzy. Version 0.1.0 is an initial development candidate, not a release or
a physically qualified driving controller. [Acceptance](docs/acceptance.md)
records observed software checks and remaining robot checks.

**Start with [scripts/policy_node.py](scripts/policy_node.py)** and its
`INSERT POLICY CODE` section. It contains the observation/action explanations
and a zero-action starter. The [policy YAML](config/ai4r_policy.yaml) explains
every setting and includes lidar-only and wheel-speed-controller examples.

| Update mode | Executes your policy when |
| --- | --- |
| `cone_detection` | A fresh cone-detection batch arrives |
| `lidar` | A fresh lidar scan arrives |
| `timer` | The configured policy timer runs |

Other callbacks cache observations. Declare `required_sensors` for your exercise;
missing/stale required data stops the policy. Required cones also have an
independent empty-detection timeout. Recovery needs an explicit start request.

## Build and run

The robot must first have a compatible DREAM underlay and a built student
overlay at `~/ai4r_student_workspace`. In particular, `dream_interfaces` must
provide `DriveAndSteer.units` and `UNITS_NORMALIZED`; older definitions are
incompatible. The compatible `dream_interfaces` v0.1.0 dependency is pinned in
[ci/dependencies.repos](ci/dependencies.repos). Staff/developer standalone build
and test instructions are in [CONTRIBUTING.md](CONTRIBUTING.md).

With the compatible underlay and student overlay sourced:

```bash
ros2 launch ai4r_policy ai4r_policy.launch.py namespace:=car
```

The launch supports optional `namespace` and `params_file` arguments. Shipped
policy settings load first, then the explicit overlay. It starts only the
`ai4r_policy` node and does not respawn it. DREAM supplies hardware services and
mounting/pan TF separately. Managed policy execution and component-configuration
loading still require the DREAM integration described below.

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
(normally `base_link`); lidar stays in its message frame. Body IMU vectors use
mounting TF, and relative heading resets only when entering policy state 3.

| Topic | Type | Direction / QoS |
| --- | --- | --- |
| `cone_detections` | `dream_interfaces/ConeDetections` | Input; reliable, depth 1 |
| `scan` | `sensor_msgs/LaserScan` | Input; best effort, depth 1 |
| `wheel_speed_m_per_sec` | `std_msgs/Float32` | Input; reliable, depth 1; unsigned m/s |
| `imu/data` | `sensor_msgs/Imu` | Input; best effort, depth 5 |
| `policy_fsm_transition_request` | `std_msgs/UInt16` | Input; reliable, depth 10; states 1/2/3 |
| `drive_and_steer_set_point_normalized` | `dream_interfaces/DriveAndSteer` | Output; reliable, depth 1; explicit normalized units |
| `pan_set_point_normalized` | `std_msgs/Float32` | Output; reliable, depth 1; optional normalized target |
| `policy_fsm_state_value`, `policy_fsm_state_string` | `std_msgs/Int8`, `std_msgs/String` | Output; reliable, depth 10; current state and reason |
| `imu_heading_angle` | `std_msgs/Float32` | Output; reliable, depth 10; valid tared heading in degrees |
| `debug1`, `debug2` | `std_msgs/Float32` | Output; reliable, depth 10; optional student scalars |

All QoS is volatile. Standard ROS parameter/introspection services and TF
subscriptions also exist. Finite actions are clipped to `[-1,1]`; invalid
outputs or student exceptions stop the policy. A pan target of `None` holds;
zero recentres. Drive is motor effort, not a speed setpoint. Full meanings and
examples are beside the variables in the Python file.

Four commented parameter files are installed:

- [ai4r_policy.yaml](config/ai4r_policy.yaml): triggers, required sensors and timing.
- [traxxas_vehicle_interface.yaml](config/traxxas_vehicle_interface.yaml): vehicle tuning and commented calibration/identity references.
- [oakd_cone_detector.yaml](config/oakd_cone_detector.yaml): perception and debug settings.
- [bno08x_imu_interface.yaml](config/bno08x_imu_interface.yaml): selected IMU products and accuracy.

Policy settings are startup-only. Restart the affected node after editing its
startup YAML. The policy launch loads only its own file; the other files are
requests for DREAM to admit and pass to the independently launched components.
Commented reference values do not override robot calibration.

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

This is the standalone source repository. DREAM still needs compatible component
selection, the managed policy runtime unit, admission/loading of the three
component YAMLs, and body/IMU/panning-camera TF. The policy does not own those
services, perform hardware discovery, or install student dependencies.

`dev` is the active development branch; `main` is the release-ready line.
Immutable tags identify releases. Use feature branches and reviewed MRs;
safety, public-interface and CI-policy changes require recorded human review.
See [contribution checks](CONTRIBUTING.md), [change history](CHANGELOG.md),
[acceptance](docs/acceptance.md), and the [MIT license](LICENSE).
