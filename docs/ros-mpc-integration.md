# ROS–MPC integration diagnosis

The shipped MPC profile follows cones continuously (`mpc.run_duration_s: 0.0`).
The previous value, 8.0, was a demo duration: it requested zero drive after eight
seconds even with a valid lane. A positive duration can still be set explicitly.
This edit is source configuration only; it has not been deployed to a car.

Read `/car/policy_fsm_state_string`, not only the numeric state or a shortened
Foxglove label. In state 3 it now includes the MPC reason, raw and retained cone
counts, and a warning if wheel-speed feedback is unavailable. It distinguishes
following, waiting for a usable lane, cached-plan continuation, an inferred
endpoint, an exhausted plan, a lane-loss timeout, solver failure and a duration
limit. State 2 continues to report the ROS stop reason, such as stale required
sensors or no recent nonempty detections. These checks and explicit restart
requirements are unchanged. Vehicle Disarmed/Disabled is a separate state;
the policy never enables the vehicle.

With the vehicle held disarmed, inspect:

```bash
ros2 topic echo /car/policy_fsm_state_string
ros2 topic hz /car/cone_detections
ros2 topic echo /car/cone_detections --once
ros2 topic hz /car/wheel_speed_m_per_sec
```

A cone batch must use `base_link`, have a fresh acquisition stamp and valid
coordinates/colour/confidence. The ROS node logs rejected batches. Detector
presence, valid depth, mounting TF, accepted ROS data, filtered cones and a
usable lane are separate checks. The policy filter currently excludes points
beyond 4 m forwards, even though the detector accepts camera depth up to 5 m.
A plot containing detections therefore does not prove the MPC received a lane.
Do not infer cone colours from a plot that draws every point in red.

The current straight-lane fit needs at least two same-colour cones with 0.25 m
of forward separation. A final pair alone does not provide a new row fit.
Endpoint inference from disappearing rows can also confuse occlusion with the
finish. Neither this integration change nor the screenshot establishes camera
depth accuracy, mounting geometry, or car-specific steering calibration.

Replay a converted observation recording without ROS or hardware:

```bash
python -m tests.policy.diagnose_cones scenarios/data/real_scenarios/straight_even_scenario.json
```

This reports raw, plausible/unique and row-filtered cone counts, retained colours,
points beyond the forward limit, acquisition age and lane-fit availability using
the default settings. It does not simulate the ROS watchdog or certify the camera.
The existing straight recording has 36/52 usable lane batches, 7 empty batches
and 9 nonempty batches without a fit. The long-track recording has 67/80 usable
batches, 7 empty batches and 6 nonempty batches without a fit. These recordings
are not the run shown in the screenshot.

After an authorized deployment, restart only the policy for its YAML/code change.
Detector image/depth/TF investigation requires the matching detector logs and
camera view; no detector settings or live robot services were changed here.
