# Changelog

## Unreleased

## 0.1.1 - 2026-09-28

- Add a bounded cone outlier filter for Stage 1: view-volume and confidence
  gating, duplicate merging, and rejection of cones off their colour row using a
  shared-direction row fit. Settings are under `cone_filter` in the policy YAML;
  policy code receives `filtered_cones` and `debug1` reports rejected cones.

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
