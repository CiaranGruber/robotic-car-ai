# Changelog

## [0.2.0] - 2026-09-28

- Always prepare body-frame Cartesian lidar points alongside the original scan,
  retaining usable rays' original indices and independent raw availability.
  Rename the raw observation/required-sensor/timeout key from `lidar` to
  `lidar_scan`; add `lidar_cartesian`, while keeping update mode `lidar`.
  Failed conversion discards old points; required-data stopping and explicit
  resume apply independently to the chosen representations.
- Expose measured cone publication latency alongside current measurement age.
  Cone observations now store a batch dictionary; student coordinate lists and
  the policy function signature are unchanged. Consume the updated cone IDL.
- Reject fiducial batches whose pose transformation overflows without refreshing
  observations or triggering the policy; clarify the fiducial-mode YAML comments.
- Add student YAML controls for detector-owned allowed IDs and three-frame
  confirmation, without changing the policy observation structure.
- Add validated ArUco observations, an optional acquisition-triggered policy
  mode, and acquisition-time camera-to-policy-frame pose transformation.
- Add the student ArUco detector source-settings example. Empty marker batches
  remain valid observations; marker visibility policy stays student-owned.

## 0.1.0 - 2026-09-22

- Add a single-file student policy with cone-detection, lidar and timer update
  modes; configurable required sensors; independent freshness/zero supervision;
  explicit resume; heading tare on policy entry; normalized drive/steer and
  optional pan commands.
- Provide a zero-action starter, detailed inline teaching comments, and four
  documented parameter files with robot-owned values shown as references.
- Add installed-package ROS checks and the DREAM repository/CI conventions.
- Add release-candidate/final tag identity, exact promotion/synchronization
  checks, tagged installed builds, and deterministic source artifacts carrying
  the exact `dream_interfaces` v0.1.0 pin.

This is a new package, not an in-place replacement of the old `ai4r_pkg` node.
It consumes the units-bearing DREAM actuator message and standard IMU messages;
legacy percentage commands and old cone message/colour conventions are not
supported. Managed DREAM launch/configuration integration remains separate.
