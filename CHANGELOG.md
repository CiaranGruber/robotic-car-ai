# Changelog

## Unreleased

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
