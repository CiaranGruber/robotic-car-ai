# AI4R policy acceptance

Status: the implementation passes the software gate on `jah`. Version 0.1.0
identifies a release only through its published annotated tag and successful
release CI. Physical vehicle acceptance remains unperformed.

## Evidence ownership

The offline gate in CONTRIBUTING.md builds and tests the installed policy with
the exact units-bearing `dream_interfaces` revision in ci/dependencies.repos.
Synthetic observations and a synthetic ROS peer establish software behavior,
not actual sensor/vehicle response.
Keep revision, command, result and limitations here; retain detailed logs in
CI artifacts or merge requests rather than a tracked evidence directory.

## Observed software evidence

### Release preparation, 2026-09-22

The isolated `jah` checkout at
`4d16bc025a47a0b287d801977c5eee92b405bf1b` passed the full installed-package
gate with exact standalone `dream_interfaces` v0.1.0. Both packages built;
all 38 pytest cases / 39 colcon checks passed, with no failures or skips.
The installed public launch arguments were present. Four portable release
contract tests passed, and source packaging produced the archive, provenance
manifest and matching checksums. Logs are retained outside this repository in
`/home/poi/build/dream-release-20260922/logs`; MR !2 holds CI and review evidence.

The first run at `135f4efe4b008d393b56c27b5e93d4dc0976c0f9` exposed a fixed
two-second wait in the launch smoke test: it interrupted the node before its
startup message under parallel build load. The test now waits for that actual
startup event with a bounded deadline, then checks graceful shutdown. Policy
behavior was unchanged. All ROS peers in this gate were synthetic and isolated
from the ordinary robot domain.

### Ground-origin camera documentation and settings, 2026-09-22

On `jah`, the required `AI4R_INTERFACES_SOURCE=PATH bash tools/verify_fast.sh`
gate passed for the executable and configuration files committed in
`7c0ca0c050a2aefc24a68a7e12d6099692216d4a`, using exact-clean `dream_interfaces`
v0.1.0 at `9b6ef917c0b8bc31efe6ca07b8a3d25f29c35fdd`. Verification used a
working-tree snapshot before final documentation-only edits; tested source
hashes were checked before committing.

Both packages built successfully. All 38 pytest cases passed (39 colcon checks,
zero errors, failures or skips). Installed launch arguments remained `namespace`
and `params_file`, and installed `camera_mount.yaml` matched its source.
Only synthetic ROS peers ran; physical sensor, vehicle and camera mounting
accuracy were not tested.

### Initial policy gate, 2026-09-21

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

### Source-live runtime observation, 2026-09-22

Policy commit `b8591507b9bdcf8faa4f13da78aa673aa6e0a333` was checked from source
against the DREAM system using `dream_interfaces` v0.1.1. The observation found
the managed AI4R policy and Foxglove services starting, and Foxglove decoded
zero drive/steer actions with normalized units in policy state 2. Duplicate
start was idempotent, the detached services persisted, and graceful stop was
observed. The standalone behavior gate was not rerun during this deployment;
no calculation request, vehicle enable, firmware flash, sensor exercise, or
powered-vehicle acceptance occurred.

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

The DREAM runtime/configuration source contract now exists, but its behavior and
physical frame accuracy have not been accepted on a robot. Measured dynamic
camera TF is needed before physical pan motion. GUI workflow remains a separate
follow-up. Release publication requires promotion, final-tag CI, immutable
assets and recorded human review; their completion is recorded in GitLab MRs
and the release record.
