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

## Intersection and local-map conflict resolution, 2026-10-26

The owner requested resolution of the in-progress merge of `cd7df7f` into
`feature/aruco-marker-actions` at `d59e812`, for their own upload and team
follow-up. Six conflicted files now preserve both intersection navigation and
the incoming cone/obstacle memory. Node startup and MCAP replay load both
configuration families; legacy memory deserialisation supplies defaults for
both. Visualisation retains remembered cones/obstacles, the selected lane and
the separate observed-lane layer. The merged summary uses the incoming line
list, avoiding the conflicting reference to an undefined `lane` variable.
On intersection takeover the old avoidance path and remembered lane-coordinate
obstacles are cleared, so they cannot reappear in the outgoing lane's frame.
Combined-memory round-trip and takeover checks cover this integration.

`OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m pytest
-q tests/policy` completed with **730 passed, 5 failed, 3 skipped, 1 xfailed,
6 subtests passed** in 33.84 s. The five failures are nominal straight travel
through the .2 m-spaced cross and the two T orientations with straight exits,
and left/right turns at the .4 m-spaced cross. All stop on the approach with
`incoming corridor stopping envelope reached before exit support`.
These remain explicit failing tests for team follow-up. Read-only ablation
of the cross cases showed that disabling recalled cones did not recover them;
using the historical `obstacle_stop_distance_per_speed_s: 0.0` instead of the
incoming `0.8` did. The incoming stopping setting and all stop checks are
preserved in the resolved source; the diagnostic does not establish that the
historical value is physically appropriate.

All 44 policy/helper Python files parsed, and the diff whitespace check passed.
The required CONTRIBUTING fast command was attempted, but the pinned
`.verification/dependencies/dream_interfaces` checkout is absent; no installed
ROS gate or physical driving result is claimed. A bounded read-only Astra
review found no omission in the combined conflict resolutions. Ignored
`build/merge-conflict-resolution-20261026/` retains the conflicted working files,
all three original index stages and the ablation results. No car connection,
upload, commit, push or completed merge was performed for this request.
Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.01_branching_strategy.md`, `2.02_adding_parameters.md`, `2.05_car_memory.md`
and `3.00_codebase_structure.md`.

## Car 10.43.254.33 RGB and junction configuration repair, 2026-10-26

The owner explicitly requested changing the car after recent logs showed the
junction navigator disabled and no ArUco observations. The five recent sensor
windows were replayed offline using the saved parameters: their first new
`LANE_LOSS` latches and requested actions match to floating-point precision.
The latest run stopped at 3.1448 s after .5302 s without a usable lane fit,
despite seven fresh cones. GPU runtime snapshots had `publish_rgb_image: false`
and its optional source override was missing, leaving the ArUco image and
calibration topics without publishers. ArUco's process was running during those
tests; the missing clean camera feed prevented observations.

The authorised repair patched only the current car's source configuration,
preserving its comments and other settings. `intersection.enabled` is now true;
`imu_angular_velocity` joins cones, wheel speed and Cartesian lidar in required
sensors. Cone, raw/Cartesian lidar, wheel and gyro deadlines are .3 s to match
the navigator. `cone_empty_timeout_s` is 4.0 s, covering the existing 4.0 s
camera hold without changing stream freshness. The optional
`config/oakd_gpu_cone_detector.yaml` was restored with only
`publish_rgb_image: true`. GPU annotated debug images remain enabled, with
inherited clean 1280x720 RGB/CameraInfo at 10 Hz. ArUco remains DICT_4X4_50,
8.1 cm markers, three-frame detector confirmation.

The policy was confirmed at ZERO before writing either file. Source hashes
guarded against overwriting concurrent changes, and the previous policy YAML
and an absent-GPU-override record were saved under
`/home/ai4r/policy_configuration_backups/20261026T160046+1100/`. Only the GPU
detector and policy were restarted, through DREAM; ArUco resumed processing
when its input returned. No student-code upload/build, vehicle-enable request,
policy state-3 request, calibration, steering-limit or other service change was
made. This is an explicit car-specific configuration; repository defaults
remain the opt-in teaching defaults.

A ten-second steady observation confirmed effective enabled settings, **101
matched advancing RGB/CameraInfo pairs**, **100 advancing ArUco batches**,
250 cone batches, 100 lidar scans, 487 wheel samples and **499 valid gyro
samples**. Maximum observed acquisition ages were .091 s for cones, .106 s
for lidar, .025 s for valid gyro and .164 s for ArUco. Camera calibration was
valid and camera/IMU/lidar transforms to `base_link` were available. The vehicle
reported `Disarmed`, the policy reported state 2, wheel speed was zero, and
all **200 observed policy drive/steer commands were exactly zero**. No marker
was visible in this initial observation.

The owner subsequently presented a marker. A first ten-second capture still
contained empty fiducial batches. A later clean RGB capture showed the marker
on a laptop display; the configured DICT_4X4_50 pixel decoder recognised **ID 4
(right)** in all twelve retained frames. A separate live ten-second observation
then confirmed **ID 4 with `pose_valid: true`**, 101 advancing fiducial batches,
and 100 matched RGB/CameraInfo acquisition pairs. Its last transformed position
was approximately (.749, -.175, .194) m in `base_link`, or .769 m planar range.
That is a pose estimate using the configured .081 m black-square size; the
displayed square's physical dimensions were not independently measured.
The vehicle remained `Disarmed`, wheel speed was zero, policy state was 2 and
all 200 observed policy commands were zero. No additional detector setting or
service restart was needed for recognition. The user was told recognition was
confirmed and the marker could be removed. Driving and junction completion
were not tested.

The targeted YAML changes passed parsing, constructor and startup-contract
checks; runtime parameter readback confirms the car loaded them. The required
CONTRIBUTING fast command was attempted locally but the pinned
`.verification/dependencies/dream_interfaces` checkout is absent. No installed
ROS gate is claimed. Read-only configuration review found no blocker for startup
and observation at zero. Ignored artifacts under
`build/car-intersection-config-20261026/` retain the before/after files, diff,
backup/hashes, runtime snapshots and observations. The preceding log diagnosis
is under `build/car-intersection-stop-20261026/`. Rules followed:
`0.00_root_rule.md`, `0.01_coding_standards.md`, `2.02_adding_parameters.md` and
`3.01_using_car_management_script.md`.

## T-junction exit support, 2026-10-26

The owner requested T-intersection support. The optional navigator now supports
equal-width perpendicular crosses and three T orientations: straight closed
(left/right available), left closed (right/straight), and right closed
(left/straight). IDs 3/4/5 retain their left/right/straight meanings. A closing
row blocks the requested direction; the policy never silently substitutes a
different exit. The GPU detector, hardware services, steering limits and
physical car were not changed or contacted. The saved navigator remains disabled.

Current lidar establishes a candidate from a strong near wall on either side.
A selected side requires that wall plus separated far-boundary returns, which
may share a physical corner but must extend beyond it. The weaker far check
requires .15 m extent and at least two occupied .12 m bins; one cone's .05 m
three-return surface cannot satisfy it. Straight requires both continued rows
beyond the junction. Interior closing rows override availability, with corner
endpoints excluded. Missing returns remain unknown, and an unsupported command
stops at its admission deadline. The body-fit check uses only incoming and
positively supported outgoing arms under the equal-width orthogonal prior.

While exit support is unknown, a measured incoming cone frame and current paired
incoming lidar rows can support a slower, finite straight approach with the
existing MPC. The first candidate anchors its frame and endpoint at centre
minus half road width, car front and configured stopping margin, before the
junction mouth. Fresh matching cone rows renew evidence without rotating that
frame or advancing the endpoint. A configured stopping envelope includes sensor,
actuation and policy-step allowances. The turn retains its earlier radius-based
admission cutoff. Missing corridor/exit evidence, expiry, tracking error, solver
failure or a raw return in the swept footprint stops this approach. Its selected
lane is labelled `approach`; the committed route remains empty and the marker
pending until an exit is admitted. No existing stop latch is cleared.

Incoming-frame colour membership is retained so opposite-arm cones cannot join
the wrong incoming boundary. A false near-transverse arc at a T corner no longer
wins merely because its offsets straddle the car: tangents beyond 45 degrees
need a supported forward association or give no lane. A transverse segment may
share a corner with a compatible pair of incoming rows. These remain geometric
heuristics; shared-corner association can admit previously uncertain short noisy
curves as straight fragments. Tests preserve accepted ordinary bends, noisy
curves with an outlier, full U-turns and recovery from outside a curved lane.
New frame/exit/approach fields live in serialisable `IntersectionMemory`, with
defaults for older recordings. Motion, command/route lifetimes, outgoing
confirmation, independent zero publication and pan hold are retained.

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.t_intersection_simulation --output build/t-intersection-20261026
```

The matrix completed **48 cases / 5,822 policy frames** on a 2.4 m road with
2.5 m turn radius. The dense .2 m cases include nominal/noisy commands on every
T, plus 1.2 s missing-blue, .8 s all-camera loss and 1.2 s missing-corner windows
on the four available turn choices. All 12 loss windows actually trigger at
20 degrees heading and recover after their finite durations plus .2 s delivery
latency. Both .5 m and .6 m layouts supply nine nominal sparse probes each.

All **six clean available exits** complete navigation and outgoing handoff.
All **six dense closed-road commands** (nominal/noisy) stop with explicit closing
row evidence and no committed turn. Overall **22/24 dense available-exit cases**
pass. The noisy left turn at a terminating T reaches the correct moving exit
but has not completed outgoing confirmation at the scoring gate, so it remains
a failure. The noisy straight command with the left road closed stops because
the current paired incoming lidar corridor is no longer supported before
straight-exit admission. All **18 sparse probes** stop on approach `LANE_LOSS`;
they establish neither T admission nor closed-exit handling. No case records
a cone/object contact, wrong exit or body excursion. These failures remain
limitations; no stop threshold or outgoing confirmation was relaxed.

The unchanged cross matrix was rerun into
`build/intersection-cross-t-regression-20261026/`: **20/22 navigation passes /
4,039 frames**, retaining the previous pass count. The two noisy .4 m approaches
stop on expired incoming-frame evidence. Existing clean .4 m left/right
navigation now has explicit regression coverage.

The complete policy suite passed **702 tests**, with 3 optional skips, 1 existing
expected failure and 6 passing subtests. New checks cover all T choices, physical
caps/road scoring, single-cone rejection, unknown evidence, finite approach
endpoints and pending commands, stopping/obstacle/missing-row checks, fixed frame
anchors with advancing acquisition evidence, legacy memory and ordinary curves.
`git diff --check` passed. The required CONTRIBUTING fast command was attempted
against the pinned path but `.verification/dependencies/dream_interfaces` is
absent; no installed ROS gate is claimed. Read-only design/code review found
no blocking issue. Human diff review is still required before merge.

Ignored artifacts in `build/t-intersection-20261026/` contain a nine-case nominal
storyboard, six-case animation (turn camera gaps and nominal straight exits),
frame archive, trajectory CSVs and a source-hashed report. Executed source hashes,
case/frame totals and disturbance activation/recovery were checked. Inputs are
synthetic decoded detections, a fixed forward 86-degree/.4–2.5 m camera, ideal
three-return 4 m lidar, fresh motion and a no-slip plant. No pixel detector,
measured lidar occlusion, actuator deadtime, tyre dynamics, physical stopping
or arbitrary/unequal-width junction capability is established. No deployment
occurred. Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.02_adding_parameters.md`, `2.03_adding_and_using_test_data.md`,
`2.05_car_memory.md` and `3.00_codebase_structure.md`.

## Sparser cone rows, 2026-10-26

The owner clarified that wider spacing meant distance between neighbouring
cones along each row. The turning harness now accepts `--cone-spacings` and
`--plot-spacing`. An explicit `--corner-anchored` layout retains the four
corners at (+/-1.2, +/-1.2) m and spaces cones symmetrically outwards. This keeps
the 2.4 m road opening fixed rather than changing its apparent width or symmetry
with the sampling phase. Previous default layouts remain available. Distances
are between cone centres. No runtime policy, GPU detector, steering limit,
saved car configuration or physical vehicle was changed in this update.

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.intersection_turning --cone-spacings 0.5 0.6 --plot-spacing 0.6 \
  --corner-anchored --output build/intersection-spaced-cones-20261026
```

This completed **22 cases / 889 policy frames**, with **0 navigation passes**:
ten left/right turn cases per gap, plus nominal/noisy straight cases at .6 m.
All latch `LANE_LOSS` on approach before marker command activation or junction
admission. In clean cases the camera initially sees two cones on each boundary,
then only one as the nearest cone leaves its field of view; there is insufficient
support for a fresh lane fit. The .6 m noisy cases stop sooner. No route is
selected, wrong exit reached, contact recorded or body excursion scored. The
12 scheduled turn-loss cases never reach the 20-degree heading trigger and
therefore provide no evidence of loss-during-turn recovery. The sparse approach
failure remains unresolved; stop thresholds were not relaxed.

The unchanged assumptions are the 2.4 m equal-width perpendicular cross, 2.5 m
turn radius, fixed forward 86-degree/.4–2.5 m camera, .2 s sensor latency, ideal
cone lidar, fresh gyro/wheel signals and the no-slip CAR_20 plant. These are
decoded detections, not GPU pixel recognition or physical qualification. Equal
seeds reproduce each case, but different cone counts consume different random
draws, so noisy layouts are not paired per-cone comparisons.

Artifacts under `build/intersection-spaced-cones-20261026/` contain the annotated
`cone_layout.png`, .6 m scenario figures/animation, detailed frame archive,
trajectory CSVs and source-hashed report. Reports count stops and whether the
scheduled disturbance actually triggered; figure labels do not imply a route
or camera-loss event occurred when the car stopped beforehand.

Focused geometry, lane-selection and navigation regressions passed **46 tests**.
They cover unique fixed corners, symmetric uniform rows, serialized layout
reconstruction and legacy sampling as well as existing navigation contracts.
`git diff --check` passed. The required CONTRIBUTING fast command was attempted
with `AI4R_INTERFACES_SOURCE="$PWD/.verification/dependencies/dream_interfaces"`
but that pinned checkout is absent; no installed ROS gate is claimed. Read-only
Astra review found no blocking geometry or selection issue. No deployment
occurred. Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.03_adding_and_using_test_data.md` and `3.00_codebase_structure.md`.

## ArUco selected lanes and turning scenarios, 2026-10-26

The owner requested turning scenarios and clarified that confirmed ArUco
left/right/straight commands must change lane formation while normal straight
and curved lanes continue to work. The GPU detector and physical car were not
changed or contacted. The optional junction mode remains disabled in the saved
configuration; the scenarios explicitly enable it locally.

The existing cone-fit `LaneDetection` remains measured boundary evidence at
camera acquisition time. It cannot represent a connected straight–turn–straight
junction path as one constant-curvature arc. A new `SelectedIntersectionLane`
in `policy/lane_detection/selection.py` represents the admitted centre path in
the estimated current body frame. The remembered, confirmed marker selects the
route only after the existing incoming-cone frame, bilateral side-road lidar,
feasible width/radius, command freshness and clearance checks admit the cross.
ID 3 selects left, ID 4 right and ID 5 straight. A marker on an ordinary road
queues a command and does not replace that road's straight/curved fit.

The new selected path and the MPC curvature preview are sampled from the same
admitted route/progress. It is exposed as `MovementPolicy.selected_lane`, drawn
as the current `lane` centre, and cleared on stop/completion. Measured edges
remain separate in the faded `lane/observed` drawing layer. The active
intersection solver's MPC prediction is drawn even when the ordinary lane
solver's counter has not advanced. Active-turn debug scalars use the route's
current lateral/heading tracking errors. Foxglove MCAP export now reads the
recorded intersection configuration and includes `/lane/observed`; previous
settings without that feature still use its disabled default.

The committed choice survives marker disappearance and bounded camera loss
under the existing motion, lidar and route-lifetime limits. Fresh outgoing
bilateral tracks and heading confirmation still admit handoff. Normal measured
lane fitting then uses observed colour sides (`AUTO`), accommodating blue on
the right after the southbound exit without recolouring physical cones. The
handoff now updates remembered width once per step instead of twice. A route
sample join also now removes duplicate adjacent positions, which could otherwise
produce a zero-length segment and NaN during projection at an exact turn start.

The reproducible command is:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.intersection_turning --output build/intersection-turning-20261026
```

It runs **22 closed-loop cases / 4,205 policy frames** using the existing
MovementPolicy → MPC → CarController → CAR_20 plant. The cross is 2.4 m wide,
radius 2.5 m, with .2/.4 m cone spacing. Left/right cases each cover nominal
cones, 20% misses with .02 m noise, a missing blue boundary for 1.2 s, all camera
cones missing for .8 s and missing corner cones for 1.2 s. The three injected
loss profiles start when the actual simulated heading first reaches 20 degrees
in either direction; finite acquisition-time windows restore detections even
after world x stops advancing. Nominal/noisy straight commands provide the
other two cases. Disturbances affect only sensor generation, never the
detector's branch identities or control targets.

**20/22 reached the requested moving exit**, with completed handoff, no wrong
exit, cone/object contact or body excursion. All ten .2 m-spacing turn cases,
the two straight cases and eight .4 m-spacing turn cases pass. The two failures
are the noisy .4 m-spacing approaches: both retain the correct command but
latch `LANE_LOSS` before the cross is admitted. They stop without a route,
contact or body excursion; they are not counted as navigation passes. The
new branch reference does not turn a sign into permission to drive without
enough geometry. These failures remain a limitation requiring more work on
the sparse/noisy approach, not wider acceptance thresholds.

Inputs are synthetic decoded detections, an 86-degree/.4–2.5 m camera with .2 s
latency, fresh wheel/gyro samples, and ideal three-return cone lidar within 4 m.
Policy rate is 10 Hz; the no-slip kinematic plant is integrated at 100 Hz. The
equal-width perpendicular-cross prior, configured radius, borrowed motor time
constants and synthetic negative-drive deceleration remain assumptions. No
pixel detection, measured lidar occlusion, tyre dynamics, actuator deadtime,
ROS watchdog timing or physical stopping/frame accuracy is established.
Simulator poses are used for sensors, disturbances, scoring and drawing; the
controller receives motion estimates and decoded sensor observations only.

Focused regressions verify that all three marker commands preserve ordinary
straight and both curved-road fits/actions versus a no-marker baseline before
admission (existing expiry/sensor-stop conditions remain active), the selected path
and MPC curvature agree, current-pose transforms are correct, stop/completion
clear selection, fresh turn predictions draw, reversed exit colours reacquire,
width is learned once and disturbances recover after finite windows. The full
policy suite passed **662 tests**, with 3 optional skips, 1 existing expected
failure and 6 passing subtests. The required CONTRIBUTING fast command was
attempted but remains blocked by the absent pinned
`.verification/dependencies/dream_interfaces` checkout; no installed ROS gate
is claimed. Read-only Astra review checked the frame/source separation and
failure-report robustness. Human diff review remains required before merge.

Ignored artifacts under `build/intersection-turning-20261026/` include
`signal_selection.png`, `turning.png`, `turning.gif`, `runs.json.gz`, individual
trajectory CSVs and a source-hashed report. Green is the selected centre path
used by MPC, dotted grey is the measured local cone fit, and dark is the actual
controller-driven simulated trajectory. No deployment occurred.
Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.02_adding_parameters.md`, `2.03_adding_and_using_test_data.md`,
`2.05_car_memory.md` and `3.00_codebase_structure.md`.

## Cone grouping and lane formation update, 2026-10-26

The owner requested a fix and a repeat simulation, then clarified that the GPU
detector must stay and the change should improve its decoded positions/grouping
before lane formation. The GPU model, camera, sensor services and vehicle were
not changed or contacted. The update uses current observations without training
a model or introducing additional persistent state.

`filter_cones` now rejects non-finite positions/confidence and confidence outside
the allowed range. Equal-confidence duplicate detections use a geometric tie
break, keeping the same measurement irrespective of packet order. Valid positions
are not moved or smoothed. Existing sensor age, stamp and publication latency are
preserved. The existing frame/yaw correction remains before lane association.

`lane_detection.associate_branches` is enabled in the local policy configuration
and defaults to true in `LaneDetectionConfig`. The node automatically declares
and loads it through the existing read-only dataclass parameter loop. The old
concentric fitter is retained. A coherent arc explaining at least 85% of the
batch retains its fit when width and local geometry are plausible, including
ordinary outlier rejection and recovery from outside a forward-facing curve.
Otherwise, bounded hypotheses from the nearest 12 cones per colour identify
supported incoming straight segments. Nearby support, residual and forward
tangent contribute to selection; compatible paired boundaries must agree in
direction within 15 degrees and satisfy colour/width checks. Hypotheses more
than 45 degrees from forward are not incoming-straight candidates. A separate
transverse segment must support association; a one-boundary proposal requires
three cones. Similarly supported competing directions or transverse-only views
can return `NO_LANE` rather than recover the corrupted all-cones fit.

Associated straight fragments use the existing fitter's straight model, avoiding
curvature extrapolation from short noisy rows. This does not globally flatten
curves: genuine circular rows retain the curved model. All checked cones remain
in the observation batch for obstacle detection and the intersection navigator;
only lane-fit membership changes. The shared `_fit_lane_edges` entry point is
used by both production detection and diagnostic inlier reporting.

The repeat command was:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.intersection_identification --output build/intersection-association-20261026 \
  --baseline build/intersection-identification-20261026
```

It completed the same **72 cases / 12,528 frames** plus the separate 174-frame
straight-row control. The baseline report/frame hashes are recorded; scenario
keys, camera assumptions, acquisition poses, timing and raw counts are checked
before admitting the comparison. The clean nominal 1.4 m road now produces
**100/100 aligned approach fits**, versus 23 aligned, 54 accepted misaligned and
23 no-lane frames before. The nominal missing-blue-boundary case also gives
100/100 aligned approach fits. The straight-row control remains 100/100.

Across the whole matrix, these are **approach-frame geometry counts**, not
independent navigation trials. The unchanged alignment thresholds are 15 degrees
heading and .2 m centre offset; junction-interior frames have no unique-exit score.

| Profile | Aligned before / after | Accepted misaligned before / after | No lane before / after |
| --- | ---: | ---: | ---: |
| Clean, 1,172 frames | 442 / 927 | 378 / 0 | 352 / 245 |
| Noise and misses, 2,344 frames | 769 / 1,455 | 814 / 135 | 761 / 754 |
| Missing blue boundary, 1,172 frames | 445 / 857 | 458 / 0 | 269 / 315 |
| Missing corner cones, 1,172 frames | 430 / 556 | 312 / 35 | 430 / 581 |
| Complete detection gap, 1,172 frames | 442 / 833 | 303 / 0 | 427 / 339 |

Accepted misaligned approach fits fall from **2,265 to 170 out of 7,032**;
aligned fits increase from 2,528 to 4,628, while no-lane frames change from
2,239 to 2,234. The residual 135 noisy and 35 missing-corner errors are real
limitations. In particular, two unused cones suffice as transverse evidence,
and coherent but incorrectly associated fits can still survive. This is a
geometric heuristic, not a general intersection recognizer or a proven physical
driving policy. The matrix is still a centred straight sensor sweep; rotated
and offset approaches are additionally covered by focused tests, not by the GIF.
No camera-pixel detector, measured occlusion or closed-loop driving is simulated.

The 37 new regressions cover invalid GPU-output values, deterministic duplicate
selection, shuffled dense crossings, rotated/offset approaches, wide-road
transverse fits, single/ambiguous support, both full U-turn directions with an
outlier, and coherent curve recovery from outside the lane. The full policy
suite passed **644 tests**, with 3 optional skips, 1 existing expected failure
and 6 passing subtests. A curved-obstacle fixture now explicitly enables curved
fitting instead of applying the straight-only default to a curved road. The
existing swapped-colour-cone test now passes, so its obsolete strict expected
failure marker was removed without changing its collision assertion.

Read-only Astra review identified the U-turn, order and extreme-value regressions
during development; their reproductions are covered by tests. Its additional
1,200-case curve comparison found no large curvature/drop regressions before
the final recovery guard, which is covered by four explicit outside-curve tests.
A local 17-cone fit benchmark measured approximately 2.9 ms at the 95th percentile
with association versus 2.3 ms without; this is a desktop observation, not a
robot timing guarantee. The required CONTRIBUTING fast command was attempted
but stopped because `.verification/dependencies/dream_interfaces` is absent.
No installed ROS gate or physical stopping/frame accuracy is claimed.

Ignored local artifacts are in `build/intersection-association-20261026/`:
`comparison.gif`, `stages.png`, `identification.png`, `report.json` and
`frames.json.gz`. The first two separate the before/after behaviour and the
decoded-detection → cone-check/group → lane-fit stages. No deployment occurred.
Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.02_adding_parameters.md`, `2.03_adding_and_using_test_data.md`,
`2.05_car_memory.md` and `3.00_codebase_structure.md`.

## Intersection identification diagnostic, 2026-10-26

The owner requested an offline simulation using the current cone detection and
lane formation. `tests/policy/intersection_identification.py` feeds synthetic
decoded cone observations through the unchanged local cone filter, camera-yaw
correction and public `LaneDetector.detect`. It separately records exact inlier
membership from the fitter's private candidate routine; a candidate can exist
even when the public result is `NO_LANE`. Scoring-only cone identities and arm
labels are never supplied as observation fields. No runtime policy source,
vehicle configuration or live car service was changed during this task.

The command documented in README completed **72 scenarios / 12,528 frames**.
The matrix covers road widths 1.0/1.4/2.4 m, cone spacing .4/.6 m and assumed
camera ranges 2.5/3.5 m. Profiles cover clean detections, 20% random misses with
.02 m position noise (two seeds), missing blue boundaries, missing corner cones
and a complete detection gap. All poses follow the road centre eastwards at
.4 m/s, with zero heading and lateral offset. Assumed camera FOV is 86 degrees,
minimum forward visibility .4 m, measurement age .2 s and publication latency
.08 s; these are simulation assumptions, not measured camera calibration.

For the clean 1.4 m-wide cross, .4 m spacing and 3.5 m visibility, there are
100 approach frames before the body enters the junction:

| Input geometry | Aligned fit | Accepted but misaligned | No lane |
| --- | ---: | ---: | ---: |
| Four-way cross | 23 | 54 | 23 |
| Same straight rows and junction gap, cross-road cones removed | 100 | 0 | 0 |

Alignment here checks local heading within 15 degrees of the incoming arm and
centre offset within .2 m. It does not establish full boundary/width correctness
or safe navigation. The separate control has 174 frames and is excluded from
the 72-scenario count. All 54 accepted misaligned approach fits in the nominal
cross combine inliers from the straight and cross-road boundaries. A selected
clean frame returns a heading of +82.2 degrees while still approaching along
the straight arm. The control isolates the added cross-road cones as the cause
of this difference for this geometry. Inside the junction, where several exits
are legitimate, no unique straight-lane success score is assigned.

Selected historical Oct 23 cone frames were also re-evaluated with today's local
filter/fitter. The 17:17:13 first frame again gives `NO_LANE` despite 17 filtered
cones: its candidate points +77.3 degrees and has reversed BLUE-left width.
The 17:21:18 prefix replay produces `BOTH_EDGES`, `NO_LANE`, then
`RIGHT_EDGE_ONLY` at frames 18–20, preserving prior width state. Those captures
have no labelled road ground truth and do not establish the current deployed
teammate LocalMap policy's behaviour.

Artifacts are under ignored `build/intersection-identification-20261026/`:
`identification.gif`, `identification.png`, `report.json` and `frames.json.gz`.
The report records effective configuration and hashes of policy source, geometry
helper and harness. Counts, scoring partitions, junction exclusions and hashes
were checked; nonzero yaw correction and unlabelled-frame handling were exercised.
The existing cone-filter/lane-detector suite passed **77 tests**. Read-only
independent review found no scoring-data leakage or blocking harness issue.
The required CONTRIBUTING fast command was attempted but cannot run because
`.verification/dependencies/dream_interfaces` is absent; no ROS gate is claimed.

This is an identification diagnostic using synthetic decoded cones, not GPU
pixel detection, intersection classification or closed-loop navigation. Rotated
approaches and turning trajectories are not covered. It supports adding road-arm
association before fitting a single arc; changing the filter's acceptance alone
does not establish the intended branch. No deployment was performed.

Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.03_adding_and_using_test_data.md`, `2.05_car_memory.md` and
`3.00_codebase_structure.md`.

## Replacement-policy intersection stop diagnosis, 2026-10-23

After the teammate's deployment, the owner explicitly requested renewed car
diagnosis. Checks were read-only: no upload, configuration change, enable
request or runtime restart was performed by this agent. The teammate changed
the source and restarted the policy during the check, so two builds were
examined separately. The owner confirmed that the car moves briefly and stops.

Three preceding-build captures at 17:33 replayed with all 121 captured drive
and steering commands matching. Two runs stopped after approximately 0.51 s
without a usable fitted lane, at run times 1.663 and 2.391 s; the other stopped
after one stationary zero-spacing plan and the following missing-lane frame.
These results describe that preceding build, not the subsequent cone-memory
deployment.

The subsequent process, PID 33298, started after the new cone-memory source
was uploaded. Three 17:36 captures from that build replayed all **169 captured
drive and steering commands** without mismatches using its separately captured
source and configuration. Both recorded internal stops were **OBSTACLE**, at
run times 1.769 and 8.749 s, with usable lane estimates and nonempty plans.
The configuration uses remembered cones, .5 m/s target speed, .3 brake effort,
.8 s obstacle stopping factor, .2 m margin and .175 m corridor half-width.
Those are the teammate's deployed settings; this agent did not apply them.

The immediate geometry conflict is reproducible:

- At 17:36:50, 23 blocking returns lie approximately .92 m forward and .32 m
  right of the CG. A current filtered yellow cone is .102 m from the cluster,
  but .435 m inside the fitted yellow boundary, beyond the .15 m fit-outlier
  threshold.
- At 17:36:23, another 23-return cluster lies approximately 1.00 m forward and
  .57 m left. A recalled yellow cone falls within its footprint but is .479 m
  inside the selected yellow boundary and rejected by the fit.

The resulting obstacle distances, .616 and .654 m ahead of the car's front,
are within their age-adjusted stopping thresholds, approximately .714 and
.734 m. Cone memory therefore does not resolve boundary association: a cone
can be remembered and still be excluded from the selected arc. Its lidar
returns then remain inside the inferred driving lane. Camera/lidar ages differ
by .116–.178 s, so exact semantic correspondence still requires motion alignment;
the evidence supports a cone/geometry conflict but does not independently
identify either physical object. The obstacle stop was retained.

The captured runner filters cones, updates its local map, fits the lane, plans
avoidance and runs MPC. It has no connected intersection-route selection using
the ArUco direction. Recommended next work is explicit incoming/outgoing branch
association and direction-selected junction paths, retaining the existing arc
fitter within individual road segments. Increasing cone memory or accepting
all colour arrangements does not establish the correct branch.

Local ignored evidence is under `build/replacement-policy-20261023/`; `current/`
contains the cone-memory source, run excerpts and matching replay results.
The parent capture includes source files changed after its older logs, so its
older-run replay uses a separate baseline snapshot matching the initial remote
hashes. Read-only Astra review independently examined the latest stop geometry.
Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md` and
`3.01_using_car_management_script.md`.

## Offline examination after vehicle handover, 2026-10-23

The owner stopped this agent's car testing and handed the vehicle to a teammate
for a different policy deployment. The following examination used only already
captured local files: no SSH, live Foxglove interaction, upload, restart or car
configuration change occurred during the offline work. Existing policy edits
were preserved and no runtime policy source was changed.

The six captures contain **599 sensor/action steps**. Replaying the recorded
configuration reproduced all 599 drive and steering commands within `1e-9`;
33 local policy/node source hashes matched the earlier deployed manifest.
The first three captures contain only the first 150 steps of longer runs;
the last three are complete short runs. Memory records precede calculation,
so their initial previous-run latch is not a new failure in that run.

| Run, local log stem time | Captured lane result | First new stop in excerpt |
| --- | --- | --- |
| 17:15:57 | 140/150 no lane | Obstacle, 6.801 s, 0.154 m ahead |
| 17:17:13 | 150/150 no lane | None; waiting for the first usable lane |
| 17:17:51 | 149/150 no lane | Lane loss, 7.934 s, after one positive command |
| 17:21:05 | 41/41 both boundaries | Intersection lidar age, 1.172 s |
| 17:21:11 | 55/56 both boundaries | Obstacle, 3.055 s, 0.384 m ahead |
| 17:21:18 | 51/52 usable lane | None; all 52 drive commands positive |

The two 17:17 excerpts contain median 19–20 raw cones per frame. Their principal
failure is row association: 148/150 candidate fits in each assign a negative
BLUE-left lane width. One candidate points approximately 77 degrees across the
road while incorporating same-colour cones from multiple crossing boundaries.
Offline `AUTO` colour assignment accepts 148–149 frames but can accept that
wrong branch. A two-metre forward crop alone also fails to resolve it. These
counterfactuals are diagnostics, not successful changed-policy navigation.
The recommended next design work is selection of connected incoming row
segments with forward alignment and temporal continuity, before the existing
arc fitter. A colour change alone is not supported by this evidence.

In 17:17:51, the only valid lane frame occurs while wheel speed is zero, so its
MPC plan has zero metre spacing. The very next missing-lane frame cannot reuse
that plan and immediately latches lane loss. This is safe stopping but a
startup/fallback limitation. Consistent startup evidence and a validated moving
prediction would need separate design and tests; repeating a zero-length plan
through missing lanes is not a supported remedy.

Only one of the 599 policy snapshots has lidar age at least 0.3 s. At the
17:21:05 stop, the unchanged scan ages from 0.168520 to 0.320141 s during a
0.151620 s policy interval. Wheel age also rises while cone acquisitions
advance. The next lidar observation is fresh, but the explicit restart latch
correctly persists. Aggregate age cannot identify transport versus callback or
calculation delay; no freshness limit was relaxed.

The 17:21:11 stop involves 23 lidar returns at body coordinates
`x = 0.639–0.770 m`, `y = -0.469–-0.399 m`. They are inside the fitted right
boundary and are not matched to a contemporaneous detected cone centre; the
nearest centre is 0.703 m away. The capture cannot distinguish another object
from an undetected cone. The two obstacle-confirmation evaluations reuse one
scan timestamp, so they are not two independent scans. Obstacle detection and
stopping were retained; no colour-based exemption was applied.

The latest short recorded run reports wheel speed up to 0.521568 m/s. A
separate later 50-second observer reports requested drive up to 0.609466 and
applied output up to 0.592 while wheel speed remains zero throughout that
observation. Its raw remote record was not retrieved after handover; only the
tool-returned summary is available locally. It cannot establish ESC power,
drivetrain response or encoder correctness, nor does applied PWM prove motion.
The last observed vehicle state was Disabled with zero applied drive, and the
agent sent a single state-2 ZERO request when the owner stopped testing.

Before handover, the IMU was observed streaming under a new boot session
`15987582438160780185`, with eligible fresh gyro diagnostics around 50 Hz;
the preceding latched firmware fault had therefore recovered. It still reported
CALIBRATING and unreliable gyro accuracy under the unchanged permissive
configuration. This is not calibrated-accuracy or physical-navigation acceptance.

Ignored evidence is in `build/car-no-motion-20261023/`: captured excerpts,
camera image, reproducible offline analysis, per-step CSV traces,
`offline_report.json`, `observed_drive_summary.json` and
`offline_findings.png`. The camera image is not synchronized with either stop.
Read-only Astra reviews independently reproduced the command and timing/lane
findings. Replay does not establish stopping distance, collision clearance or
successful intersection traversal. Rules followed: `0.00_root_rule.md`,
`0.01_coding_standards.md`, `2.05_car_memory.md` and
`3.00_codebase_structure.md`.

## Remaining hardware services started, 2026-10-23

The owner requested activation of the remaining services on `10.43.254.33`.
The IMU, lidar and Traxxas vehicle interface were started through DREAM; all
seven selected runtime units reported running. The CPU detector remained
interrupted and the GPU remained the sole cone publisher. A read-only check
observed 50 scans, 245 wheel-speed messages, 97 GPU cone batches and 50 fiducial
batches. Lidar had 337 finite in-range returns and its transform to `base_link`
was available. Policy state stayed 2 with 45 exact-zero policy commands; the
vehicle reported **Disarmed**, zero wheel speed and zero applied motor output.
Applied steering followed the RC input and is distinct from policy zero output.

The running IMU process published **no IMU messages**. Its diagnostics identify
`FAULT_LATCHED / dynamic_calibration_failed`, with host publication disabled.
All requested products were already enabled and the permissive accuracy settings
were unchanged. This is incomplete sensor readiness, not a successful driving
startup. The required gyro gate and explicit-resume behavior remain enforced.
The provider exposes no reset API and documents that manual USB power recovery
may be necessary; no firmware, calibration or reset workaround was applied.

The owner could not identify the IMU USB cable. A full car power-off/restart
was requested as the next physical recovery step and was not yet confirmed
when this initial result was recorded. Detailed observations are retained in
ignored `build/car-services-20261023-1792733889373/` and the existing remote
deployment backup directory. The startup helper now includes
`bno08x_imu_interface` before the policy; Bash syntax and diff checks pass.
Read-only Astra review confirmed the missing-gyro gate and startup dependency.
No Enable or state-3 request was sent. Rules followed: `0.00_root_rule.md`,
`0.01_coding_standards.md` and `3.01_using_car_management_script.md`.

After the owner's reported power-off/on, the updated GPU-only helper
successfully restarted the six sensor/vehicle/policy units, retaining the
already-running Foxglove bridge. The fault and zero IMU publication persisted.
The host still reported 4191 s uptime and the IMU boot session was unchanged,
indicating that its USB power had apparently remained on. A separate power
disconnect of the onboard computer was requested; that next step remained
pending at this observation. The vehicle remained Disarmed with zero applied
motor output. No software reset, calibration change or gate bypass was used.

Further owner-authorised recovery identified the IMU at USB hub `1-2`, port
1, serial `5766176857522D66`; Traxxas is on port 2 and lidar on port 4.
The hub advertises per-port power switching. Ubuntu's `uhubctl 2.5.0-1` was
downloaded and extracted only into `/tmp`, without installation or permission
changes. Its [documented companion-hub handling](https://github.com/mvp/uhubctl#usb-30-duality-note)
also selected the empty port 1 on the USB3 counterpart `2-1`.
Immediately before one targeted ten-second cycle, the observer received 77
exact-zero policy commands, state 2, Disarmed diagnostics and 241 zero applied
motor observations. The IMU driver was stopped through DREAM, with an exit
trap restoring port power and restarting the driver on failure.

The utility reported port power off/on and the device re-enumerated, but the
IMU boot session remained `14637590233235807966`, the firmware remained
`FAULT_LATCHED / dynamic_calibration_failed`, and another five-second observer
received zero IMU messages. This did **not** establish physical power removal
or sensor recovery; no broader port cycle was attempted. After the owner's
subsequent reported physical power disconnect/reconnect, host uptime was still
6010.70 s and the IMU boot session and fault were unchanged. The vehicle
remained Disarmed with zero motor output. Identifying the actual board power
connection or confirming the physical unit is now required. The firmware,
calibration and gyro admission checks remain unchanged. Read-only Astra review
confirmed the bounded recovery and the need to distinguish re-enumeration from
a new MCU boot. Rules followed remain `0.00_root_rule.md`,
`0.01_coding_standards.md` and `3.01_using_car_management_script.md`.

## GPU cone detector selection, 2026-10-23

The owner specified the GPU detector for physical-car tests. On
`10.43.254.33`, only `oakd_gpu_cone_detector` was started; the CPU
`oakd_cone_detector` remained interrupted. The initial read-only observation
received 125 advancing cone batches from the GPU as the sole cone publisher,
while the policy remained in state 2.

The GPU's clean RGB stream was initially disabled, so ArUco had no image input.
DREAM explicitly admits an optional GPU source override. A single-file staged
helper dry run and upload created the previously absent car-only
`config/oakd_gpu_cone_detector.yaml` with `publish_rgb_image: true`, retaining
the provider's 1280x720/10 Hz defaults and all mounting/calibration settings.
Only the GPU detector was restarted to apply this, then `aruco_detector` was
started with the owner's .081 m marker setting. No provider repository was
edited, and no vehicle, lidar or IMU service was started.

A second read-only check received **139 advancing cone batches**, **56 matched
RGB/CameraInfo acquisition pairs**, and **52 advancing fiducial batches**.
The GPU was the only cone publisher. No marker ID was visible during this
observation; marker recognition and physical navigation remain untested. The
policy stayed in state **2**, with **108/108 exact zero drive/steering commands**.

The local startup helper now stops an active CPU detector before starting the
GPU sequence and aborts on status, CPU-stop or GPU-start failure. Debug-image
forwarding is preserved, with no CPU fallback. Four mocked startup/error
rehearsals, Bash syntax and diff checks passed; no policy runtime source changed.
Read-only Astra review found no helper blocker. Evidence and the car-only
override are in ignored `build/car-gpu-rgb-20261023-1792732984925/` and the
existing remote deployment backup directory. Rules followed:
`0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.02_adding_parameters.md` and `3.01_using_car_management_script.md`.

## Car 10.43.254.33 intersection deployment, 2026-10-23

The owner authorised SSH deployment and confirmed an **8.1 cm outer black
ArUco square**. The uncommitted intersection implementation was uploaded with
the car-management helper, using a staged copy targeting car 20 (`.33`), after
reviewing its dry-run list. The repository helper's existing target was left
unchanged. Before upload, the complete prior source and installed package were
archived on the car under
`/home/ai4r/policy_backups/intersection-20261023-1792731819351/`.

The car's policy overlay enables intersection navigation, retaining its timer
trigger at 10 Hz and required cone/lidar/wheel/gyro observations. Turns request
.4 m/s with a 2.5 m radius; marker IDs 3/4/5 select left/right/straight. The
existing steering/speed calibration and braking trio are preserved:
`control.brake_action: 0.0`, `mpc.stop_distance_per_speed_s: 0.6`, and
`mpc.obstacle_stop_distance_per_speed_s: 0.6`. The cone-empty watchdog remains
10.2 s. Only policy and ArUco configuration files were uploaded; the latter
changes the marker size from .25 to **.081 m**. Five other hardware/mount
configuration files matched their pre-deployment hashes. No provider source
or hardware calibration was modified.

The exact deployment policy/configuration passed a new 40-run local evaluation:
18/18 positive navigation/handoff cases, all ten negative cases stopping, and
zero cone/object contacts or body departure across all runs. The existing
width/spacing visibility limitations remain; these are single-junction,
decoded-marker, no-slip simulations, not physical driving evidence.

On the car, the CONTRIBUTING fast command passed in a separate workspace and
localhost-only ROS domain 218 against exact-clean `dream_interfaces`
`5f50902ccee44e8370d6e2be85607b3054ffbf98`: **186 pytest cases / 189 colcon
checks**, zero errors, failures or skips, and installed launch arguments
verified. Obsolete starter-test assumptions about marker size, update mode,
and active neutral steering/pan were corrected; explicit state-2 exact zeros
and pan hold remain asserted. The managed incremental student build then
passed. All 46 uploaded files matched the isolated tested source; all 36
installed runtime/configuration files checked matched the upload manifest,
and the new modules imported from the student overlay.

Only `ai4r_policy` was started. A read-only ROS observer verified 19 effective
parameters, **82/82 zero drive-and-steering commands**, state **2**, the startup
explicit-request reason, and **zero pan commands**. Foxglove was already
running; the vehicle, IMU, lidar, camera and ArUco services remained interrupted
and were not started/restarted. The .081 m detector setting will take effect
when that component is started. No Enable request, state-3 request, driving,
physical stopping or frame acceptance was performed.

Local ignored evidence, manifests, overlay, logs and simulation trajectories
are under `build/car-deploy-intersection-20261023-1792731465444/`. The successful
remote gate log is
`/home/ai4r/policy_deploy_checks/intersection-20261023-1792731819351/verify-fast.log`.
The shipped repository YAML remains opt-in; the car uses the recorded enabled
deployment overlay. Read-only Astra review checked the deployment configuration
and test changes. Rules followed: `0.00_root_rule.md`,
`0.01_coding_standards.md`, `2.02_adding_parameters.md` and
`3.01_using_car_management_script.md`.

## ArUco intersection navigation prototype, 2026-10-23

The owner authorised connecting the marker command to turning and widening the
simulated road if needed. The opt-in navigator in
`policy/action_policy/intersection.py` now uses the confirmed command to select
a straight route or 90-degree turn that aligns with the outgoing lane. The
shipped YAML keeps `intersection.enabled: false`; this is local prototype
evidence, not a physical deployment or ROS acceptance record.

The implemented family is an **orthogonal, equal-width, four-way cross**.
Incoming width and heading are fitted jointly to distinct, acquisition-aligned
straight cone tracks; the existing curved-lane detector remains the normal
lane follower. Current lidar must positively show wall segments on both side
roads. The near wall and measured common width locate the crossing under that
explicit family assumption; there is no general junction-topology inference or
proof of an unseen far boundary. A marker alone or missing incoming cones
cannot admit a turn. The padded reference footprint must fit the assumed
cross, and starting too late, stale frame support or an expired command stops
admission.

The finite route uses a **2.5 m radius**, **.4 m/s through the crossing**, and
the branch's unchanged **+.48/-.92 /m** curvature limits. Normal lane following
requests .5 m/s. The existing MPC now optionally previews varying route
curvature; each tick uses the current projected lateral/heading error. It
passes ordinary speed/curvature commands to the unchanged `CarController`.
The confirmed command is consumed once and cannot change during the manoeuvre.
Pose history, world cone tracks, choice, route, anchors and phases are
serialisable `CarMemory` data. Wheel speed and yaw rate transport camera and
lidar data using acquisition poses; no simulator pose or selected reference
path enters the policy. All sensor ages must be finite and nonnegative.

Route retention is bounded to **24 s/10 m** from original acquisition support,
with a **4 s all-cones-empty hold**. Predictions do not refresh these anchors.
Every raw finite lidar point participates in rectangular swept checks along
the route and constant-curvature/reaction-then-straight stopping envelopes;
cone colours do not exempt returns. A configured run-duration stop is still
enforced. Stops latch until an explicit new run. Handoff requires both outgoing
rows, sufficient observed extent, small heading error, and **advancing camera
acquisitions over .3 s**. Repeated cached evidence cannot complete handoff.
The normal fitter then establishes colour sides spatially (`AUTO`), because
the right exit reverses the incoming colour convention. Multiple-junction
navigation has not been established by these single-junction runs.

The nominal simulation uses **2.4 m-wide arms, .2 m cone spacing**, 8 m arms,
an 86-degree/.4–2.5 m camera with .2 s latency and ideal current three-return
cone lidar within 4 m. **18/18** runs pass navigation, chosen-exit heading and
fresh-lane handoff: each direction with clean detections, 20% misses/.02 m
noise, sign disappearance, brief cone loss, one hidden boundary and short
complete camera loss. The longest raw camera-empty interval is **3.7 s**.
All 10 negative cases stop without contacts or road departure: missing,
unmapped, unposed or single-image marker; prolonged camera loss; stale gyro,
wheel or lidar; missing terminal lane; and a physical object in the turn's
swept path. Retaining a command alone never earns a navigation pass.

Additional width/start-pose/noise probes use new seeds: **2.2 m passes 3/3**;
**2.6 m stops 3/3** without contact when its longer camera-empty interval
exceeds the 4 s hold. The larger width improves geometric room but worsens
visibility. Neither width result establishes a general minimum or maximum
road width. No physical steering limit was increased to obtain a pass.
Coarser nominal cone-spacing probes pass **3/3 at .4 m**, while **.5 m stops
3/3** at the camera-empty bound. Those failures are retained, not navigation
passes. These probes do not include noisy detections at the coarser spacings.

The plant is the same no-slip kinematic `CAR_20` response as the historical
baseline below, with ideal wheel/gyro feedback and illustrative braking.
The time constants are borrowed from car 22. Decoded sign poses do not test
the ArUco pixel detector. Dense cones, the equal-width geometric prior, static
obstacles, fixed camera and ideal lidar are material assumptions. Final goal
scoring occurs while moving, not at a final stop. Installed ROS behavior,
vehicle timing, frames and stopping remain unverified. The source YAML's
default cone-empty watchdog is unchanged: enabling this feature additionally
requires gyro/wheel/cone/lidar requirements and `cone_empty_timeout_s` at least
the selected hold interval; the node rejects inconsistent combinations.

Reproduce locally using the README command. Source-hashed reports, all
trajectories, negative outcomes, width probes and the six-panel plot are in
ignored, upload-excluded `build/aruco-navigation-final/` (**40 runs** total).
Earlier development results remain in `build/aruco-navigation-development/`
and `build/aruco-navigation-reviewed/`. No car connection,
configuration application, vehicle enable, upload, push or release was done.

Software verification: **606 passed, 3 skipped, 2 existing xfailed, 6 subtests
passed** in the policy suite. New tests cover acquisition transport, invalid
ages, swept-return blocking, explicit reset/serialization, run-duration and
route expiry, cached-evidence rejection, curvature preview and end-to-end
exit alignment. Read-only Astra review identified frame-age/distance,
acquisition, handoff and disabled-feature issues; they were repaired and the
tests/evaluation rerun. In particular, refitting cached rows cannot renew
the frame-distance anchor. Compilation and diff checks pass. The required
pinned-interface fast command was attempted for the final source and is
blocked by the missing `.verification/dependencies/dream_interfaces` checkout.
The installed ROS gate and physical stopping/frame accuracy remain unverified.

Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.02_adding_parameters.md`, `2.05_car_memory.md` and
`3.00_codebase_structure.md`.

## Historical ArUco four-way intersection diagnostic, 2026-10-23

On `feature/aruco-marker-actions`, based on
`327a2801cf9429435643a274e2b5595c12319df8`, the owner requested a four-way
intersection simulation using this branch's existing policy. Runtime policy,
marker mapping and configuration are unchanged. The shipped mappings are
**3 = TURN_LEFT, 4 = TURN_RIGHT, 5 = STRAIGHT**; confirmation takes two policy
calls after detector confirmation. The latest confirmed command is stored in
`Memory.intersection_command` until another command or an explicit new run.
There is no intersection detector, route selection, command consumption or
expiry yet; `config/ai4r_policy.yaml` already documents that limitation.

`tests/policy/intersection_simulation.py` runs the complete `MovementPolicy`
(filtering, marker reader, lane detection, obstacles, avoidance, MPC and
`CarController`) with the shipped policy/detector YAML. Normalised drive and
steering go through the repository's `CAR_20` response, not a mistaken
drive-as-speed conversion. Four matched profiles—clean, noisy cones with 20%
misses/.02 m noise, a disappearing sign, and brief cone loss—each run with all
three direction IDs. Physical geometry and disturbance seeds remain identical
within each three-run comparison; no selected branch or ground-truth path is
given to the policy. Four additional marker-negative cases exercise missing,
unmapped, unposed and single-image detections.

In **12/12 positive runs**, the correct command was confirmed on approach and
retained after the sign disappeared. **Every matched group produced identical
actions and trajectories for all three IDs.** Clean, disappearing-sign and
brief-cone-loss cases stop for `LANE_LOSS`, around x = -.578 m, before the
crossing entry at -.5 m. All three noisy cases exit straight, including the
left/right requests. Only the noisy straight case reaches its requested exit;
that **1/12 navigation match does not establish marker-controlled selection**.
Left and right complete **0/4 each**. The four negative cases produce no
intersection command. All 16 runs have zero physical cone contacts and zero
body departure from the simulated road.

The assumed road is a 1 m-wide four-way cross with 5 m arms, .5 m cone spacing,
a sign at (-2.4, -.7) m, and exit gates 2 m past the centre. An aligned exit
must persist for .3 s; retaining a command or crossing the junction centre is
not completion. Body scoring splits edges at road boundaries to detect cuts
across non-convex inner corners, and checks every physical cone at 100 Hz.
The camera has an assumed 86-degree field, .4–2.5 m range and .2 s latency at
10 Hz. Decoded marker poses pass a simplified consecutive-image publication
gate using the detector YAML; no ArUco pixel recognition is tested. Lidar is
ideal, current, three-point returns per cone within 4 m. Wheel speed is ideal;
the plant has no tyre slip or actuator deadtime. `CAR_20` speed-response time
constants were borrowed from car 22, not measured on this vehicle. Negative
effort uses illustrative .8 m/s² deceleration without reverse. Marker stands,
ROS watchdogs/state transitions and physical stopping are not modelled. The
narrow cross is a diagnostic assumption, not evidence that a physical turn
fits. A goal gate is navigation while moving, not a final stop.

Reproduce with the command in README. Ignored, upload-excluded
`build/aruco-intersection/` contains the source-hashed `report.json`, all 16
trajectory CSVs and a six-panel `intersection.png`. The new simulator and
checks are covered by the existing `tests/policy/*` upload exclusion. No
hardware, car configuration, SSH, enable or upload was used.

Verification: **585 passed, 3 skipped, 2 existing xfailed, 6 subtests passed**
in `tests/policy`. This includes five new harness checks for non-convex body
scoring, exit selection, lidar scan adjacency, detector confirmation and
retained-command scoring. A read-only Astra review found the synthetic lidar
indices initially followed cone creation order; they now follow angular ray
order, matching the obstacle grouping contract, and the final simulation was
rerun. Source hashes match the executed report and diff checks pass. The
required CONTRIBUTING fast command was attempted and is blocked by the missing
`.verification/dependencies/dream_interfaces` checkout. No installed ROS or
hardware acceptance is claimed.

Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md` and
`3.00_codebase_structure.md`.

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
