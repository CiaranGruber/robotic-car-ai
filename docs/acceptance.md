# AI4R policy acceptance

Status: the implementation passes the software gate on `jah`. Version 0.1.0
identifies a release only through its published annotated tag and successful
release CI. Physical vehicle acceptance remains unperformed.

## Turn slowing and failed-fit fallback, 2026-10-19

Follow-up simulation requested by the owner: ran 30 closed-loop comparisons
(six layouts, five policy modes) using
`PYTHONPATH="$PWD" .venv/bin/python build/turn-simulation-check/run.py`.
`build/turn-simulation-check/summary.json` records the source hashes and metrics;
`trajectories.png` and `requested-speeds.png` show geometry and commanded speeds.
The current slowdown mode completes four smooth layouts without contact or
excursion, requests 0.4 m/s in tight turns, and uses four saved-lane steps during
the smooth U's inner-edge occlusion. Both angular layouts still fail to finish,
with footprint excursions of 0.324 m and 0.326 m; the occluded angular case
contacts one cone. All six current-policy solves avoid solver-failure latches.
These deterministic nominal-car cases include valid simulated gyro and speed;
they do not reproduce the physical recordings' missing gyro or overspeed.


User requested lower speed on turns and improved saved-lane fallback when lane
fitting fails. The local policy YAML now enables `mpc.turn_slowdown_enabled`.
Requested speed interpolates from the normal 0.5 m/s at absolute curvature
0.15 1/m to 0.4 m/s at 0.45 1/m, for either turn direction. Both detected lane
curvature and the maximum absolute planned steering curvature constrain speed.
The turn cap never raises a lower cruise target. The 0.4 m/s cap keeps existing
wheel-speed feedback active; below its configured 0.4 threshold the current
controller uses feedforward only. Actual MPC prediction continues using measured
speed, including overspeed, and support/stopping checks retain conservative
measured/previous/requested speed. Lane-loss following cannot accelerate above
its preceding slowed request or the remaining cached plan's turn speed cap.
The requested speed is now shown in the existing Foxglove summary.

A failed fresh arc fit (`NO_LANE`) can now validate current cones directly
against the original accepted two-edge boundaries. At least one side needs
fresh same-colour support, residual tolerance, minimum count and 0.25 m span;
no new arc fit is required. This preserves the original motion, age, distance,
remaining extent and controller horizon/stopping bounds. It never renews the
original reference. Empty detections still use the bounded cached-plan loss/end
behavior and stop; no unlimited blind lane extension was introduced.

The five recent physical recordings, from 18:30:04 through 18:32:57, contain
**no gyro samples in any frame** (54, 112, 116, 1582 and 918 recovered frames).
Consequently their motion-compensated lane memory cannot activate. The recent
terminal buffer records the IMU service being stopped; no service was started
or configuration changed on the car during this work. Foxglove now explicitly
shows `lane memory unavailable: no gyro` in this condition. Recorded peak wheel
speeds in the four moving logs are 1.425, 1.538, 1.617 and 1.487 m/s, despite a
0.5 m/s cruise setting. Lower requested speed is not physical speed validation.

Open-loop candidate replay is in `build/turn-speed-review/summary.json`, with
settings per run, source hashes and reproducible driver beside it. Slower
positive commands occur in 0, 20, 19, 22 and 25 frames respectively; no memory
hold activates without gyro. The candidate still latches lane loss in three
moving recordings and an obstacle stop in 18:31:36. Recorded trajectories do not
respond to the candidate commands, so no collision-avoidance claim follows.

Local closed-loop comparison is in `build/turn-speed-simulation/summary.json`
and `trajectories.png`. Straight, smooth 90-degree, smooth U and occluded smooth
U cases complete without cone contact or corridor excursion. The angular U
still fails to complete: both angular cases leave the corridor and the occluded
case touches one cone. Lower speed cannot remove the existing steering geometry
limit. Physical testing and upload remain not run.

Software verification: `574 passed, 3 skipped, 1 existing xfailed` from the
policy tests. New cases exercise signed curvature scheduling, planned-steering
slowing, measured-speed prediction, lower cruise targets, invalid parameters,
no acceleration after lane loss and saved-boundary support after failed fitting.
The pinned-interface fast gate was attempted and remains blocked by absent
`.verification/dependencies/dream_interfaces`. A read-only Astra review found
no concrete remaining defects; it is not hardware qualification.

## Missing-side lane fallback, 2026-10-19

User requested reuse of the pre-generated lane when one side disappears. The
existing per-colour edge fitter is unchanged. With
`lane_detection.single_edge_hold_enabled`, fresh-only fitted evidence determines
BOTH/ONE/NONE; remembered cones cannot manufacture two visible sides. An accepted
two-edge centre arc, width and colour roles are stored in serializable lane
tracking memory. One-edge observations transport this arc using acquisition-time
wheel/gyro odometry, re-anchor it nearest the car, and require fresh same-colour
support within the configured residual tolerance and at least 0.25 m span.
The saved side colours prevent a single-edge AUTO side flip.

The original two-edge acquisition bounds prediction to 1.5 s and 0.8 m; duplicate
or single-edge frames never renew these bounds. Missing motion, inconsistent
support, no initial two-edge reference, or exhausted bounds do not authorize
prediction. Neither edge visible uses existing bounded lane-loss/end handling;
it does not prove goal completion. The MPC additionally requires enough saved
arc for camera latency, its whole planning horizon and coasting stopping travel,
using the maximum of target, previous requested and measured speed. Insufficient
support latches a zero-command lane-loss stop and clears the plan. Fresh input
cones remain unchanged for the controller and endpoint evidence, and its last
seen distance is capped by supported arc extent during a hold.

The local policy YAML enables this fallback; the config-class default remains
false for backward-compatible replay of older saved configurations. No live
robot configuration, upload, service restart or vehicle enable was performed.
Foxglove summary identifies when the previous two-edge lane is being followed.

Software evidence: `558 passed, 3 skipped, 1 existing xfailed` from
`.venv/bin/python -m pytest tests/policy -q`, including 13 fallback regressions
covering turning-frame transport, colour roles, expiry, missing support/motion,
serialization, reacquisition and controller support bounds including stationary
starts. The required pinned-interface fast gate was attempted but could not
start: `.verification/dependencies/dream_interfaces` is absent. Hardware
qualification remains not run.

Local kinematic results are in `build/single-edge-hold-simulation/summary.json`
and `trajectories.png`. The smooth U with inner-edge occlusion completes with
five held steps, no cone contact and no footprint excursion. The requested
three-segment angular U remains unsuccessful under the car-20 steering profile:
one case touches a cone and both angular cases leave the corridor. This feature
does not resolve that steering/geometry limitation.

Open-loop replay of the 261 recovered complete frames from car .21 is recorded
in `build/car21-obstacle-review/single-edge-hold.json`. With the feature disabled,
all actions still match the original recording; original obstacle stop is frame
28 (1.531 s). With the feature enabled, the saved lane is selected at frames
25..33, but the controller support bound stops at frame 25 (1.364 s): the saved
extent cannot cover prediction and stopping travel at the recorded speed. There
is no obstacle latch in this candidate replay, but it does not keep driving or
complete the U-turn. The observations file was truncated during copying; this
is a prefix comparison, not a changed-action physical outcome.

## Experimental cone memory and lane continuity, 2026-10-19

### Connected-centreline experiment and steering feasibility

The owner authorized continued work against the same angular U. The unchanged
3 m cross leg and 0.8 m corridor span only 3.8 m vertically. Car 20's admitted
simulation left curvature is 0.48 1/m. Any intended forward left-winding turn
from heading 0 to pi requires at least `2 / 0.48 = 4.167 m` vertical travel,
even for a point car: between the last heading-zero point and first heading-pi
point, `sin(heading) >= 0` and `dheading/ds <= k`, so integrating
`dy/ds >= sin(heading) * dheading/ds / k` gives `dy >= 2/k`. Intermediate right
steering within that heading interval does not evade the bound. This rules out
this intended route under that profile, not arbitrary opposite-winding routes
or a differently calibrated real vehicle. `left_uturn_span_check` implements
this necessary screen; a pass is not a clearance/feasibility certificate.

The offline `OfflineCorridorPolicy` experiment retains the existing cone filter
and lane-edge observer, but follows a connected polyline of opposite-colour
gate midpoints rather than sending another single-arc reference to the MPC.
Mixed-colour Delaunay triangles establish gate connectivity, with bounded cone
counts, widths and same-colour gaps. Local world-frame tracks are smoothed,
transformed to the current frame using measured motion, and used for forward
lookahead. Candidate curvatures obey explicit steering and change limits;
sampled cone/rectangle rollouts reject collisions, and missing route/motion
after driving starts latches a stop. All lasting state is serialisable through
CarMemory; origin resets also clear the smoothed map. Source modules are
excluded from car upload and are **not selectable through MovementPolicy or
ROS**. This follower lacks runtime lidar/obstacle integration, braking-distance
qualification, swept-corridor certification and measured timing guarantees.

The expanded comparison keeps identical angular cone positions, spacing and
placement jitter, and evaluates five camera-noise seeds, with/without 0.4 s
inner-edge occlusion at camera times 8.0..8.4 s, at both 0.4 and 0.5 m/s. It
preserves the original 0.48/0.92 left/right limits as a failing control and
separately labels a **hypothetical, uncalibrated** 1.2/1.2 1/m steering profile.
The latter changes only the model's steering limits; camera noise/latency,
steering/speed lag, footprint, trim, layout and collision/completion checks stay
the same. The comparison includes the original constrained arc pipeline and
an unconstrained arc pipeline with the same 2 s / 1 m memory budget as the
connected follower. The connected follower uses 1.2 m lookahead, a 0.3 position
update fraction and 4 1/(m s) curvature-change limit. These are experiment
settings, not accepted robot defaults.

Run `MPLCONFIGDIR=/private/tmp/ai4r-temporal-mpl .venv/bin/python -m
tests.policy.corridor_experiment` for the comparison and source hashes under
`build/corridor-experiment`. The measured-limit case remains infeasible and
the experimental follower can contact cones there: the numerical experiment
does not waive the analytical bound. Successful hypothetical cases establish
only that a connected path can handle the angular layout with sufficiently
tight steering; they do not establish a physical fix.

With the hypothetical 1.2/1.2 profile, connected-centreline following at 0.4 m/s
passed **10/10** cases (five seeds, with/without occlusion), with no cone
contacts or corridor excursions; worst body clearance was about 0.053 m.
Both arc baselines passed 0/10. The longer-memory unconstrained arc baseline
contacted cones even where it reached the endpoint, so endpoint position alone
is not counted as a pass. At 0.5 m/s the connected follower passed only 4/10;
contacts and corridor excursions remain in the results. Under measured car-20
limits every pipeline passed 0/10 at both speeds, as expected from the bound.
The prototype therefore improves this explicit hypothetical experiment but
is not a robust high-speed or measured-profile solution. Plot trajectories
show seed 0 at 0.4 m/s; the JSON includes all 120 pipeline/case runs.

The final local policy suite passed 545 tests, with 3 optional skips and
1 existing expected failure. Sixteen new experiment regressions check route
topology, degeneracy/gap rejection, necessary steering bounds, positive
hypothetical cases across five noise seeds, serialization, map reset and
rollout spacing at the maximum admitted speed. An Astra read-only review
checked the feasibility proof, frame/reset contracts and rollout bounds.
The required fast gate was attempted again and cannot run without the absent
pinned `dream_interfaces` checkout. No physical robot configuration, deployment
or driving was changed. The actual car's left-turn calibration and gyro-enabled
recording remain prerequisites for a meaningful physical comparison.

### Angular-layout correction

The owner clarified that physical cones form three straight lines rather than
a smooth turn. The follow-up simulation assumes a squared U: two parallel
legs joined by a perpendicular segment, with two three-segment boundaries.
This is an explicit provisional interpretation, not a measured reproduction;
three parallel rows with a shared middle divider would be a different layout.
Each centreline leg is 3 m, corridor width 0.8 m, cone spacing nominally 0.5 m
with independent 0.8..1.2 spacing factors, and placement offsets up to 0.025 m
per axis, fixed placement seed 17. Camera noise is applied separately.

`AngularUTurn` uses a polygonal scoring corridor with open entrance/exit
extensions and vehicle-edge intersection checks, including concave inner-corner
excursions when every vehicle corner lies inside. Positive intrusion magnitudes
are sampled lower bounds, not exact maximum body penetration. The sharp
polyline is a scoring reference, not a feasible infinite-curvature command.

Re-running `python -m tests.policy.temporal_lane_simulation --output
build/temporal-lanes-angular` compares the original four cases with angular U
and angular U with inner-edge occlusion during camera times 4.0..4.4 s.
Both angular cases **fail completion** in every mode. Baseline contacts two
cones, memory alone contacts one, and memory plus constraints contacts none
but stops after about 2.1 m, before the first turn. The constraints therefore
do not solve this layout. The current single-arc fit and the measured car-20
turning limits remain material limitations. Do not infer physical feasibility
from these assumed dimensions or the previous smooth-turn successes.

Artifacts and source hashes are in `build/temporal-lanes-angular/summary.json`
and `trajectories.png`. The final local policy suite passed 529 tests, with
3 optional skips and 1 existing expected failure, including 11 new angular
geometry/scoring regressions. No runtime policy tuning or car configuration
changed in this simulation correction. The previously missing pinned ROS
interfaces checkout still prevents the required fast gate.

### Initial temporal implementation

The local candidate on `feature/cone-memory-continuity` retains the existing
cone filtering and concentric-arc edge fitter. It adds opt-in forward-only
planar cone memory, with same-colour position association, measured wheel
speed/body gyro integration, acquisition-time pose interpolation, and bounded
track age/travel/count. Original cone ages are preserved; fresh-only cones
still supply the MPC's endpoint/horizon evidence. A fresh inlier must support
the fitted lane. Remembered cones are labelled in the existing Foxglove layer.

The optional continuity guard transports the accepted arc to each new image's
frame and checks lateral, wrapped heading and curvature changes against noise
allowances plus rates times elapsed acquisition time. It compares only shared
supported extent and rejects whole fits, rather than creating inconsistent
clipped geometry. Rejections do not renew accepted evidence or learned width.
Expired references remain rejected until temporal reset. Missing motion,
stream loss, reverse RC, clock discontinuity and policy entry clear temporal
evidence. Existing bounded lane-loss plan following may precede a latched stop;
this is not an immediate-stop mechanism. Unsigned speed cannot detect rollback
or independently establish forward travel. Both features default **off**.

On local macOS/Python 3.12, `.venv/bin/python -m pytest tests/policy -q` passed
518 tests, with 3 optional RL tests skipped and 1 existing expected failure.
Twenty-four temporal regressions cover frame motion/latency, floating-point
history boundaries, duplicate expiry, motion loss, resets, fresh-inlier support,
association bounds, partial occlusion, whole-fit rejection, recovery, rates and
backward-compatible memory serialisation. An Astra read-only review identified
fresh-support, supported-extent, interpolation and reset issues; regression
tests cover the fixes. Implementation remained Solo.

Reproduce the local comparison with:

```bash
MPLCONFIGDIR=/private/tmp/ai4r-temporal-mpl .venv/bin/python \
  -m tests.policy.temporal_lane_simulation --output build/temporal-lanes \
  --recording .test-artifacts/lane-replay/2026-10-19_13-30-55/2026-10-19_13-30-55_observations.json
```

The fixed-seed kinematic simulation uses the existing measured car-20 response,
0.5 m/s target, 2.5 m turn radius, camera noise/latency, and measured-motion
simulation inputs. Baseline, memory alone and memory plus constraints each
completed straight, 90-degree, 180-degree and briefly occluded-inner-edge
180-degree cases without cone contacts or leaving the lane. These easy cases
do not reproduce the reported physical failure. Memory alone increased curve
error; it is not an established improvement. The local summary, trajectory
plot and comparison MCAPs are under `build/temporal-lanes`; the summary records
source hashes and dependency versions. This is a no-slip kinematic model, not
physical qualification or a worst-case timing bound.

The 109-step physical recording has **no gyro observations**. Default replay
matches all recorded drive/steering actions, but the recording cannot evaluate
motion compensation: memory remains unavailable and the enabled guard reports
no lane throughout. No gyro was invented from steering requests. Record fresh
body gyro and wheel-speed observations before using this experiment on a car.

The required `AI4R_INTERFACES_SOURCE="$PWD/.verification/dependencies/dream_interfaces"
bash tools/verify_fast.sh` was attempted and failed because the pinned checkout
is absent. No ROS build, deployment, live configuration change or physical
driving was performed. Human diff review remains required before merge.

Repository rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.00_adding_sensors_and_observations.md`, `2.01_branching_strategy.md`,
`2.02_adding_parameters.md`, `2.03_adding_and_using_test_data.md`,
`2.05_car_memory.md` and `3.02_creating_real_scenario_data.md`.

## Evidence ownership

The offline gate in CONTRIBUTING.md builds and tests the installed policy with
the exact units-bearing `dream_interfaces` revision in ci/dependencies.repos.
Synthetic observations and a synthetic ROS peer establish software behavior,
not actual sensor/vehicle response.
Keep revision, command, result and limitations here; retain detailed logs in
CI artifacts or merge requests rather than a tracked evidence directory.

## PR target conflict resolution, 2026-10-10

Merged `dev` at `238fe38` into the combined MPC/prototype branch based on
`3b950e6`. Kept the Stage 1 documentation and dev's updated figures, logging,
scenario tools and replacement car-management script. Combined logging and RC
observations with serialisable RC records and compatibility for recordings
made before the RC field existed. Updated evaluation imports for the new test
package layout; the MPC and open-space behavior remains selected as before.

With the declared dependencies and bundled DREAMGym 0.2.2 installed in the local
Windows Python 3.12.14 environment, the policy suite passed 207 tests, with
12 existing expected failures (lane-end stopping and dynamic tyre-model cases)
and three intentionally withheld unseen cases. Command:
`.venv/Scripts/python.exe -B -m pytest tests/policy/ -q -rs -p no:cacheprovider
--basetemp=build/pytest-dev-merge-2`.
The ROS fast command again could not start through Windows Bash
(`Bash/E_ACCESSDENIED`); the pinned interfaces checkout is absent. No physical
acceptance or live configuration changes are claimed. Recorded human review
and the installed ROS gate remain outstanding before PR merge.

## MPC and RC/open-space integration, 2026-10-10

Combined MPC branch `f7052b2` with prototype branch `7e30078`. The conflict
resolution retains measured-speed prediction, solver-failure latching and
distance-bounded lane-loss fallback alongside the prototype's run-duration
stop and RC/open-space selection. Travelled distance is accumulated once.
Endpoint inference remains an offline experiment, not a ROS-selected policy.

On Windows with Python 3.12.14, `.venv/Scripts/python.exe -B -m pytest
tests/policy/ -q -p no:cacheprovider` passed all 139 tests. This includes
timed-stop/restart and invalid-duration regressions plus the existing MPC,
endpoint, filtering, control and open-space tests. The saturation test now
sets its effort limit explicitly; the control default is float `1.0` so ROS
declares the same parameter type as the shipped YAML's `0.2` override.

The CONTRIBUTING fast command was attempted through Windows Bash but could
not start (`Bash/E_ACCESSDENIED`). The pinned interfaces checkout is also
absent. These portable tests include import stubs and do not establish ROS
integration, physical stopping or frame accuracy. The installed ROS gate
and recorded human diff review remain required before PR merge. No robot
configuration or hardware was changed.

## MPC kinematic screening, 2026-10-06

Final-pair endpoint prototype, also uncommitted: the owner chose the final cone
pair as finish cue. The explicitly selected offline EndpointMpcPolicy completed
20/20 finish-visible cases in a 70-run baseline/prototype experiment. Hidden
final cones and persistent occlusion each caused 5/5 false early endpoint
declarations; these are retained failures, not accepted behavior. Missing-stream
cases latched zero. 96 policy tests pass; the ROS fast gate remains blocked by
the absent pinned interfaces checkout. Results, source hashes and settings are
in `build/mpc-endpoint-evaluation/`, explained in the
[iteration report](mpc-simulation-evaluation.md). The endpoint extension is not
selected by MovementPolicy or the ROS node. No hardware acceptance is claimed.

Subsequent uncommitted algorithm iteration on `codex/mpc-simulation-evaluation`:
MPC uses available wheel speed for prediction, solver failure latches zero until
an explicit new run, and lane-loss fallback stops at cached-plan exhaustion.
The [iteration report](mpc-simulation-evaluation.md) records 30 speed-sensitivity
runs with zero solver failures and 1.2–3.5% lower spatial lateral RMS using
measured speed. Completion still fails; no physical or dynamic-model acceptance
is claimed. `.venv/bin/python -m pytest tests/policy -q` passes 83 tests. The fast
gate was reattempted and is still blocked by the absent pinned interfaces
checkout. Source hashes, metrics and plots are in local
`build/mpc-speed-evaluation/`. Human diff review remains required before merge.

Initial screening evidence (before those runtime changes):

On the local macOS host, starting from `0e48627`, an offline evaluator compared
three unchanged-runtime MPC configurations in 60 tuning runs and ran 10
validation cases of the lowest-error Pareto candidate. All stopped before the
6 m finish; none passed the provisional completion gate. Solver fallbacks were
zero. The [evaluation report](mpc-simulation-evaluation.md) records assumptions,
metrics, reproduction and limitations. Local `build/mpc-evaluation/summary.json`
records exact source hashes and dependency versions.

`.venv/bin/python -m pytest tests/policy -q` passed 66 tests. The CONTRIBUTING
fast command was attempted but could not run: the pinned interfaces checkout
at `.verification/dependencies/dream_interfaces` is absent. This is ideal
speed/curvature kinematic screening, not a DREAMGym dynamics run, ROS gate,
car-computer timing qualification, or physical vehicle acceptance. Runtime
policy and car parameters were not changed.

## Student-facing review, 2026-09-28

The owner reviewed `scripts/policy_node.py`, `config/ai4r_policy.yaml` and
`config/aruco_detector.yaml` at commit
`5d81db2353e798d687b0a65dfb6a0d3862805e80` and approved them without changes in
the development conversation. The reviewed Git blobs, respectively, are
`f6b2eee08e259fe08f77aab9a4f0eafbc45f84f8`,
`35b77406b6af2c938f8999cd2809b0ec6b1fe197` and
`c47dbe135b816282a82977d307a956ec7299c9b7`.
This records approval of those three student-facing files. Review of the wider
cross-repository changes and final MR verification remains separate.

## Observed software evidence

### Student lidar mounting configuration, 2026-09-30

The uncommitted candidate on `feature/student-lidar-mount`, based on
`d244b246984d0cc6d64876ffd9e506b610494da5`, was checked on the supplied Jetson
Orin Nano (Ubuntu 24.04.5 LTS, aarch64, Python 3.12.3, ROS Jazzy), using exact
clean `dream_interfaces` revision `5f50902ccee44e8370d6e2be85607b3054ffbf98`.
`AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh` passed all 64 pytest
cases / 65 colcon checks, with no errors, failures or skips. Installed launch
arguments were checked, including installation of the empty `lidar_mount.yaml`
mapping. The node changes are comments only; executable behavior is unchanged.

The normalized SHA-256 of `config/lidar_mount.yaml` was
`2e4463e748064ef2a3e13916e285f1d88bb819123efa0f165ac4c3e745d7348f`;
that of `scripts/policy_node.py` was
`cd5b5a4fafc14afe09d410633edd98b45b1cc2f70f3e48219fde99541ecbcdee`.
Logs and a source manifest are retained under
`/tmp/student-lidar-verify.3P2Yuw3r` on the test host and in the local
`.test-artifacts/student-lidar-mount` directory. Subsequent acceptance edits
change documentation only. DREAM's matching acceptance record owns the
configuration/restart contracts and live lidar/TF checks. The policy gate used
synthetic peers in localhost domain 218; no policy-controlled vehicle test was
performed. Deployment and physical mounting accuracy remain **not tested**.

The implementation was subsequently committed as
`ffe7d3d0233af745b0d8a19ac1a6210a893882d4`. A fresh live scan captured at
2026-09-30 01:29:56 AEST was converted by calling that source's
`PolicyNode._convert_lidar_scan` directly, producing 600 valid points from 720
rays. The owner explicitly confirmed that the 180-degree plot matches the
car's current physical surroundings, supporting the unchanged default yaw.
DREAM's acceptance record retains the capture identity and operator evidence.
This records the default orientation; metric calibration and MR diff review
remain separate.

### Student Traxxas settings and policy comments, 2026-09-29/30

The working-tree Traxxas YAML was checked on Windows with Python 3.12 and
PyYAML 6.0.2 against the matching DREAM parameter resolver. All 19 active
settings (the original four plus 15 newly delegated settings) were admitted,
retained their types and existing defaults, and resolved to the baseline when
omitted. This was a source-configuration check, not an installed ROS test.

The assistant subsequently checked exact clean policy commit
`a6b7cbfef839adf140da43efe356b900514fee76` on the supplied Jetson Orin Nano,
Ubuntu 24.04.5 LTS / aarch64, Python 3.12.3 and ROS Jazzy. The required installed
fast gate passed: 64 pytest cases / 65 colcon checks, zero errors, failures or
skips, and installed launch arguments checked. All four release-contract tests
and source packaging passed. The installed Traxxas YAML was byte-identical to
source; the matching DREAM resolver admitted all 19 settings, preserved their
types/defaults in launch YAML and inherited the baseline for omitted values.

The owner's comment edits were committed as `4ada8d0`, followed by a small
comment correction/whitespace pass. An AST comparison confirmed unchanged
executable behavior; the only docstring change is a grammar correction.
Exact clean candidate `4c8b55480a2dcc84618982784d56d635df2aea39` was then
verified again on that Jetson using exact clean `dream_interfaces`
`5f50902ccee44e8370d6e2be85607b3054ffbf98` from `ci/dependencies.repos`:

- `AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh`: 64 pytest cases /
  65 colcon checks passed, no errors, failures or skips; installed launch
  arguments checked.
- `python3 -B tests/test_release_contract.py -v`: all four tests passed.
- `python3 -B tools/package_source.py --output ...`: source-package preview
  passed in a fresh output directory. The first attempt correctly refused to
  overwrite the previous run's existing `build/release` directory.

Logs and the verification manifest are retained under
`/tmp/student-traxxas-verify.NpwVqxx9/final` on the test host and attached to the
implementation MR. Subsequent acceptance edits change documentation only.
Tests used localhost domain 218 and synthetic peers; no physical driver or
actuator ran. Deployment and physical vehicle checks remain **not run**.
Install the matching DREAM delegation update before using the new active
student YAML; older DREAM versions reject these newly admitted settings.

### Cartesian lidar observations, 2026-09-27

The candidate based on `890857e` prepares body-frame Cartesian points for every
accepted raw scan, with original ray indices and separate freshness requirements.
The assistant ran `AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh` on jah
against exact-clean interfaces `d95da38bbea66ff2a91f6f52ab24ea18ba116f8c`:
64 pytest cases / 65 colcon checks passed, no failures or skips. Both packages
built and installed launch arguments were checked. Cases cover three-axis TF,
invalid-ray filtering, index correspondence, empty results, failed conversion,
rejected scans, copied snapshots, both freshness deadlines and explicit recovery.

The public student build passed. A synthetic check used the installed policy
and actual DREAM mounting-TF launch action in isolated localhost domain 219,
omitting the physical driver. For 720 rays the callback median was 3.182 ms,
p95 3.291 ms and maximum 33.623 ms across 200 samples on jah. This includes raw
validation, storage and conversion, but excludes DDS delivery and student policy
execution. It is a short observation, not a worst-case timing bound. Runtime,
configuration and test hashes matched the local feature source after CRLF
normalization. The accompanying commit identifies the implementation; only
acceptance documentation was added after verification. Logs and the probe remain
under `/home/poi/lidar_cartesian_20260927/evidence/` and the matching DREAM system
evidence record. Physical lidar, mounting accuracy and driving were **not run**.

### Cone publication latency, 2026-09-27

The candidate based on `bed86cf` consumes interface
`d95da38bbea66ff2a91f6f52ab24ea18ba116f8c` and keeps cone publication latency
attached to the corresponding batch. Student coordinate lists and the function
signature remain unchanged; the internal cone observation is now a dictionary.
The assistant ran the required `AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh`
on jah against the exact clean interface checkout: 53 pytest cases / 54 colcon
checks passed, no failures or skips, and installed launch arguments were checked.
The new cases cover nonempty/empty batches, independent acquisition-based age,
copied snapshots, stale availability and rejection of invalid latency without
refreshing data or triggering a policy step. The earlier overflow regression
also remains in the passing suite. Source hashes matched the local changes
after CRLF normalization; the accompanying feature commit identifies the tested
implementation. The log is `/home/poi/cone_latency_20260927/evidence/policy-fast.log`.
Only synthetic ROS peers ran; physical driving remains untested.

### Fiducial overflow and comment audit fixes, 2026-09-27

The assistant verified the executable, configuration and test sources committed
in `d6395cc2dddf3cff69a18c670fa67a7723b16619` on jah, using a disposable workspace
and exact-clean interfaces `6711f6ce7799f0d97dfa4bf9f6bc40c52b39857b`.
The required `AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh` passed:
48 pytest cases / 49 colcon checks, zero failures or skips, and installed launch
arguments checked. Source hashes matched the local files after CRLF normalization;
the disposable script's executable mode was restored from Git's source archive.

The new regression failed against the preceding runtime source with the expected
unhandled transform `ValueError`. With the fix it confirms whole-batch rejection,
unchanged cached freshness, no policy trigger, expiry to zero publication, and
continued explicit-resume requirements after fresh data returns. The two YAML
comment omissions are corrected. Logs are retained on jah under
`/home/poi/aruco_test_20260926/evidence/policy-audit-fast.log` and
`policy-audit-regression-before.log`, with local development copies. These checks
use synthetic ROS data; no camera, vehicle interface or physical actuator ran.

### ArUco filter configuration, 2026-09-27

Policy source `0adf7d5` adds the detector-owned allowed-ID and consecutive-frame
settings to the student YAML and its installed-configuration check. The required
`AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh` passed on jah against
exact-clean interfaces `6711f6ce7799f0d97dfa4bf9f6bc40c52b39857b`: 47 pytest
cases / 48 colcon checks, no failures or skips. The log is retained at
`/home/poi/aruco_test_20260926/evidence/policy-filters-fast.log`. The policy runtime
Python source is unchanged by this filter configuration update; these are
synthetic checks, not physical driving evidence.

### ArUco development snapshot, 2026-09-26

The uncommitted 0.2.0 snapshot based on `ff6049d` built on jah against the new
working-tree fiducial interfaces and passed all 47 installed pytest cases.
Synthetic ROS cases cover camera-to-policy pose transformation, dictionary and
batch validation, fresh empty detections, stale required-input stopping,
recovery and the fiducial trigger. They establish software behavior only.

Logs remain in `/home/poi/aruco_test_20260926/evidence/policy-tests.log` and the
system development evidence record. The standalone gate subsequently passed
against exact-clean interface commit `6711f6ce7799f0d97dfa4bf9f6bc40c52b39857b`:
47 pytest cases / 48 colcon checks, zero failures or skips, with both packages
built and installed launch arguments inspected. Four portable release/provenance
regressions passed after source packaging was changed to record the dependency
manifest's exact commit without inferring the old v0.1.0 release tag. No physical
vehicle response or camera mounting accuracy was tested.

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

### MPC and CarController integration review, 2026-10-12

PR #16 was integrated with `dev` at `2fb92f0`, preserving the calibrated MPC
settings, asymmetric curvature limits, camera-yaw correction and lane offsets
from #15, alongside CarController, lane-end inference and lane-state debug
outputs. The Stage 1 simulator retains both the calibrated camera/car profiles
and the optional measured CarController response models. Non-finite control
inputs are now rejected before clipping, so they cannot become full drive or
steering actions; speed-input rejection also clears the integral.

On macOS/Python 3.12 with dreamgym 0.2.2, the merged policy suite passed
310 tests, with 3 optional RL tests skipped and 1 existing dynamic wet-floor
case expected to fail. The stop-zone test checks stopping separately from the
lane-tracking test; the latter retains that known failure. The yaw-correction
regression disables lane-end inference to isolate tracking from the new stop
behavior. Historical tuning tables are not evidence for the merged defaults.

The Stage 1 CLI ran T1–T12 with `--speed 0.5`, the shipped car 22 control
settings (`speed_ff_offset=0.25`, `speed_ff_gain=12.0`, `speed_kp=0.2`,
`speed_ki=0.15`, `speed_i_band=0.5`, `max_drive_action=0.45`) and
`--mpc stop_distance_per_speed_s=2.1`, with the new MPC defaults and no run-time
limit. Car22a passed 12/12; car22b passed 11/12. Car22b T11 still failed:
6 cone contacts, 0.394 m outside the lane, 0.461 m endpoint lateral error and
stopping 1.28 m before the last cone. No tuning or threshold relaxation was
applied to hide this failure. These models reuse car 20's steering map;
car 22 steering calibration remains unresolved.

The required `AI4R_INTERFACES_SOURCE="$PWD/.verification/dependencies/dream_interfaces"
bash tools/verify_fast.sh` could not run because that pinned checkout is absent.
No ROS build or physical driving was verified. Lane-end inference can mistake
occlusion for a final pair; it is not a confirmed endpoint detector. Human diff
review remains required before merge.
