# AI4R policy acceptance

Status: initial implementation passes the software gate on `jah`; no release or
physical vehicle acceptance. Version 0.1.0 does not identify a published release.

## Evidence ownership

The offline gate in CONTRIBUTING.md builds and tests the installed policy with
the exact units-bearing `dream_interfaces` revision in ci/dependencies.repos.
Synthetic observations and a synthetic ROS peer establish software behavior,
not actual sensor/vehicle response.

## Observed software evidence

On 2026-09-21 (Australia/Sydney), the assistant ran `tools/verify_fast.sh` on
`jah`: ARM64 Ubuntu 24.04.4, ROS 2 Jazzy, Python 3.12.3. The isolated build used
`dream_interfaces` v0.1.0 commit `9b6ef917c0b8bc31efe6ca07b8a3d25f29c35fdd` and
policy commit `f7bf564bf3c3b6a2fda98318dc643954be9f0e98` (before this evidence
update; tree `6fc93b1fcc9b0cd234cdbb77db8d64cd3dab5bb3`). The consumed cone and
actuator messages are unchanged from the earlier tested development dependency.
Tracked executable modes were preserved by transferring a Git archive.

- Both packages built and installed successfully.
- All 38 pytest cases passed in 6.50 seconds. Colcon reports 39 checks because
  it also counts the enclosing CTest case: zero failures, errors or skips.
- The tests exercised real ROS messages, selected-trigger behavior, sensor
  expiry and recovery, state controls, IMU transforms/tare, invalid outputs and
  calculation overruns, namespaced YAML loading, synthetic DDS/timers, and the
  installed launch process starting and shutting down successfully.
- The installed launch arguments check passed. Local staged whitespace checks
  passed, and public component parameter names were checked against source.
- GitLab CI lint accepted `.gitlab-ci.yml` without errors or warnings.

The remote log is `/tmp/ai4r-policy-20260921.v1YztL/verify-fast.log`; a copy was
retained outside this repository as `tmp/ai4r-policy-jah-verify-fast.log` in the
development collection. These temporary logs are supporting evidence, not
release artifacts. No sensor driver, vehicle interface or physical car ran.
GitLab pipeline results are linked from the implementation merge request.

## Initial GitLab setup audit

Read back on 2026-09-21 for `dream/ai4r_policy` (project 7128): default branch
`dev`, semi-linear merges, encouraged squash, required successful pipelines and
resolved discussions, source deletion by default, obsolete-pipeline cancellation,
and the DREAM squash/merge message templates.

The project protection list contains exactly `main` and `dev`, with no wildcard
branch rule. Neither permits direct or force pushes. Developers/Maintainers may
merge to `dev`; only Maintainers may merge to `main` or create `v*` tags. The
instance reports GitLab 19.4.0, `enterprise: false`; group-protection and approval
APIs return 404. No separate second-person approval rule was configured. Human
diff review for this safety/public-interface/CI change remains a recorded MR
requirement, not a claim of server-enforced selective approval.

## Robot acceptance checklist

All currently **not run** with a physical car. Record source/dependency commits,
tester/date, robot configuration and observed result when these are performed:

- Zero startup, explicit vehicle/policy controls, neutral handshake and stop.
- Lidar-only and wheel-speed-only operation with no cone detector running.
- Empty/missing cones and other required-sensor loss, explicit recovery request.
- Actual command timing/vehicle watchdog and physical stop behavior.
- Pan hold on policy stop, correct pan-dependent cone frame and measured pose.
- Correct IMU mounting, body axes, relative heading and reset semantics.
- Admitted component YAML loading and component-specific restart through DREAM.

Managed DREAM runtime/configuration integration, physical TF calibration and
GUI/workspace workflow are separate follow-ups. Release preparation must add
tag packaging/identity CI and recorded human review before creating a release.
