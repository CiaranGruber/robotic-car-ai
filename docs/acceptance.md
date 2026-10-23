# AI4R policy acceptance

Current local work uses the owner's corrected **connected blue outer boundary**
and prefers online geometric feedback with no trained weights. The earlier
open-apron result is superseded. The selected ROS mode remains the earlier
corridor candidate; the connected-turn experiment is not selected or uploaded.
The installed ROS gate is blocked locally; physical vehicle acceptance remains
unperformed. Earlier records below describe their historical snapshots.

## Connected blue boundary: preferred online feedback, 2026-10-21

The owner prefers a controller based on current observations because real cone
placement will differ. `ConnectedTurnPolicy(config)` already defaults to
`network=None`; the preferred evaluation now directly uses that branch and
does not load checkpoints or train a model. Cone rows and the connected cap are
estimated from live detections transported by wheel/gyro odometry. Analytic
lookahead steering is recomputed from the current estimated pose each tick.
Tests forbid `numpy.load` during five end-to-end navigation/occlusion cases.

This is **online feedback, not online learning**. The circular-cap geometry
family, planned radius and inward margin remain assumptions. The fitted frame
and turn freeze after commitment, and bounded memory carries that turn through
brief occlusion. Only sufficiently supported fresh return geometry may replace
the return path. It does not continuously reshape the committed turn from
partial observations. Camera pan remains zero.

`tests/policy/evaluate_connected_realtime.py` uses a single fixed controller
configuration across **12 new varied layouts**: .9–1.1 m row gaps, 2.6–3.8 m
divider lengths, .9/1.2 m straight extensions, .4/.55 m cone spacing, .025/.04 m
placement jitter, changed start poses, temporary camera loss and physical
missing cones. Placement seeds are 14000..14011 and detector seeds
14200..14211; course geometry is not supplied to the policy. **10/12 complete**;
two .9 m-gap cases stop at the lidar/braking-footprint guard. All have **zero
cone contacts and zero full-body floor excursions**. Guarded stops are failures
to complete, not navigation passes.

Four out-of-model probes change the physical blue cap and its continuous
scoring boundary together, scaling cap depth to .8/.9/1.1/1.2 while preserving
row spacing. **3/4 complete**; the .8-depth cap stops without a supported path.
These results show limited tolerance to shape variation, not arbitrary-layout
generalisation. All four negative cases (prolonged empty camera stream, stale
gyro, stale wheel speed, unsupported terminal geometry) latch stops without
contact or excursion. Both cone-memory-disabled comparisons stop without
completing. The return goal gate still measures navigation while moving, not
final stopping.

All completion results above use **hypothetical +2.2/-0.92 /m steering** and a
.5 m planned radius, .30 m/s approach and .15 m/s through turns. The earlier
+1/-0.92 /m assumption remains infeasible and stops before driving. The fixed
10 Hz simulation uses the same no-slip, odometry, sensor and braking assumptions
as the historical comparison below. Tick timing is a local diagnostic and does
not establish a hardware deadline. Physical steering capability and stopping
remain unverified; no car configuration or upload was changed.

Reproduce the preferred checkpoint-free experiment:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.evaluate_connected_realtime --hypothetical-tight-steering \
  --output build/connected-realtime
```

`build/connected-realtime/` contains the source-hashed `report.json`, varied-case
trajectory CSVs and `evaluation.png`. It contains no weight checkpoint. These
ignored artifacts and the experiment are excluded from car upload. A read-only
Astra review found no blocking issue in the runner, checkpoint-free claim or
ellipse scoring; its figure-label correction is applied. Rules followed:
`0.00_root_rule.md`, `0.01_coding_standards.md`, `2.05_car_memory.md` and
`3.00_codebase_structure.md`.

Software verification for this checkpoint-free evaluation: **700 passed,
3 skipped, 1 existing xfailed** in `tests/policy`; diff checks pass and executed
source hashes match the report. The required pinned-interface fast command was
attempted and stops because `.verification/dependencies/dream_interfaces` is
absent. The installed ROS gate remains unverified on this Mac.

## Connected blue boundary: historical trained comparison, 2026-10-21

The owner confirmed the curved connection and clarified that the two straight
blue rows continue **1 m beyond the last yellow cone before the curve begins**.
The nominal course has 1 m between adjacent rows, a 2 m total outer width and
a semicircular blue cap of radius 1 m. Its furthest blue point is therefore
2 m beyond the yellow tip. The car must remain inside this boundary. There is
no open apron. The scorer checks the whole body against a continuous closed
outer floor and middle divider, plus contact with every physical jittered cone.
Missing camera detections do not remove physical obstacles. Physical removal
profiles remove those cones from both lidar and contact scoring.

**The earlier +1.0/-0.92 /m steering assumption is infeasible here.** A necessary
strip-width bound for a forward-only rectangular car is
`hypot(R + width/2, rear) + hypot(R + width/2, front)`, where `R=1/max_curvature`.
The derivation allows countersteering and nonmonotone heading. The .30/.30/.25 m
guard body needs at least **2.329 m** at 1/m even without extra clearance or a
divider. The default closed-course policy therefore latches a stop before
driving. Code cannot establish greater physical steering capability. A nominal
offline sensitivity sweep passes at hypothetical left limits 1.8, 2.0 and
2.2 /m, with respective planned radii .556, .5 and .5 m; tested lower limits
stop. These are single-layout sensitivity results, not calibration or a general
minimum-steering guarantee.

The separate conditional model trains at **hypothetical +2.2/-0.92 /m**, with
a **.5 m planned left radius**, .30 m/s approach and **.15 m/s through turns**.
`policy/action_policy/connected_uturn.py` reuses the existing straight-row
observer, sensor admission and motion-compensated cone memory. A longer observed
divider can refine a noisy startup angle by at most .005 rad per step before
the turn is committed. Cached endpoint/start positions remain in odometry
coordinates and are reprojected without refreshing their observation times.
Unsupported legacy coordinates disable refinement. Curved blue detections are
excluded from the straight-row fit. At least three distinct, recent, spatially
separated cap observations are required; no simulator pose, row IDs or course
endpoints enter the policy or network. The turn uses the straight extension,
placing its centre .35 m inward from the sensed cap centre, with observed
clearance past the divider tip. Both fitted frame and row offsets freeze when
the turn route is committed.

Route time and distance use the same original acquisition pose and expire at
36 s/12 m. A **fresh-empty filtered cone stream lasting 3 s** stops independently
of that route limit. Any surviving filtered detection resets the empty-stream
timer; this is not a test that both full boundaries remain usable. Surviving
detections cannot refresh the retained turn anchor. Fresh two-boundary return
paths cannot discard an unsupported terminal tail and give it a new anchor.
Motion/sensor failures and path loss after driving latch stops until explicit
new-run reset. Camera pan stays fixed at zero.

The guard retains the .30/.30/.25 m footprint, .075 m padding, measured initial
curvature, acquisition-time lidar transport, .15 s spatial steering settling and
reaction-then-centred-steering braking sweeps. Its speed bound includes measured
speed, the next commanded speed and maxima of queued commands covering .35 s,
stored in bounded .05 s buckets. An older .30 m/s command cannot be forgotten
while it is still inside the .2 s actuator delay. The plant still assumes no
slip, ideal measured odometry, an 86° camera with .2 s latency/.03 m detection
noise, ideal three-point cone lidar within 4 m and requested-speed lag. Active
braking assumes .2 s dead time plus .8 s linear deceleration. These are offline
assumptions, not physical frame, actuation or stopping evidence.

The 17→32→32→1 tanh steering network is trained from scratch with behaviour
cloning and one DAgger round. The final dataset contains **9,802 samples**;
training RMSE is **.00160 /m**. Both optimisations reached their 650-iteration
cap rather than declaring convergence; closed-loop tests determine acceptance.
Planning, cap fitting, cone memory and stop checks remain deterministic. Teacher
and learned steering both pass **12/12** training rollouts; learned validation
passes **4/4**. Separate physical placement seeds **10000..10007** and detector
seeds **10200..10207**, with layout combinations absent from training, pass
**8/8** frozen teacher and student navigation tests. Contacts and full-body floor
excursions are **zero**. Cases include 25% random misses, temporary divider/outer
loss, .4 s complete loss, physical interior removals, a missing divider-tip cone
and a missing cap cone. The longest final raw-camera empty interval is .7 s;
the oldest usable evidence while generating previews is .9 s.

All four negative tests—prolonged camera loss, stale gyro, stale wheel speed and
unobserved terminal geometry—latch stops without contact or excursion. Both
cone-memory-disabled comparisons stop without completing. Navigation completion
is the existing return goal gate while moving; it does not establish final
stopping. Teacher/student equality of outcomes does not establish an advantage
of learned steering. Earlier failed closed-course reports remain in
`build/connected-uturn-development/` (6/8) and
`build/connected-uturn-slow-development/` (7/8); those test sets became
development evidence. The 8000-series run in `build/connected-uturn-verified/`
predates the final coordinate/delay hardening and is not final source evidence.

Final artifacts live in `build/connected-uturn-final-reviewed/`: `report.json`,
`hypothetical-steering.npz`, `demonstrations.npz`, `evaluation.png` and per-case
trajectories. Reports freeze executed-source/configuration and checkpoint hashes;
the closed-cap weights reject incompatible geometry, speed and memory contracts.
The files are ignored by Git and excluded from car upload. Reproduce locally:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.train_connected_uturn --train-hypothesis \
  --output build/connected-uturn-final-reviewed
```

Read-only Astra review identified and helped repair acquisition-distance,
coordinate-frame, queued-speed and terminal-tail issues. Physical steering
measurement and vehicle testing are **not run**. No car configuration, SSH,
upload, service, enable, commit, push, merge or release was performed.

Software verification: **696 passed, 3 skipped, 1 existing xfailed** in the policy
suite, including **63** open/connected learner checks; **143 passed** in the
actual locally filtered upload copy and **4 release-contract tests passed**.
Compilation and diff checks pass. The required pinned-interface fast gate was
attempted for the final source and cannot start: the configured
`.verification/dependencies/dream_interfaces` checkout is absent, and this Mac
lacks ROS Jazzy. No physical interfaces were used to work around that limit.

Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.03_adding_and_using_test_data.md`, `2.05_car_memory.md`, and
`3.00_codebase_structure.md`.

## Superseded open-apron model: 8/8 frozen holdouts, 2026-10-21

The earlier interpretation used three finite, straight, parallel cone rows and
omitted the curved blue connection. The owner subsequently corrected that
interpretation. This run's 8/8 result applies only to its open apron; it does
not validate the connected course above. The simulated car enters the lower
gap, rounds the divider's end and returns in the upper gap.

`policy/action_policy/three_row_uturn.py` provides an offline NumPy steering
student and deterministic observer/planner. Training from scratch uses a
17→32→32→1 tanh network, behaviour cloning and one DAgger round: **11,320**
sensor-derived feature/action samples, final training RMSE **0.00219 /m**. The
teacher uses the same sensed/remembered preview path and analytic steering;
learning approximates steering, while cone memory and turn formation remain
explicit code. Teacher and student both pass the same final 8 cases; this
experiment does not demonstrate an advantage over analytic steering.

Motion compensation uses valid, fresh wheel speed and gyro yaw, acquisition-time
poses, distinct cone tracks and bounded pose history. A sparse startup view with
two divider cones needs a separate parallel outer boundary, plausible gap,
consistent spatial support and a second of stationary advancing observations.
An initially truncated plan can be replaced before turning only with newly
revealed extent and newer endpoint observations from all three rows. A new
return path needs fresh separated support from both return boundaries. Lost
paths after any admitted drive command latch a brake/zero command until an
explicit new-run reset. Predictions and single-row observations cannot renew
the retained turn anchor. Prolonged empty frames expire it after 36 s or 12 m.
These extended fresh-empty limits belong only to this offline experiment.

Training placement seeds 30..41, validation 500..503 and final frozen holdout
4000..4007 use separate episodes and layout combinations. Normalisation, fitting
and DAgger use training episodes only. Validation passes **4/4**; frozen student
and teacher navigation each pass **8/8**, with **zero physical cone contacts and
zero full-body floor excursion**. Tests cover nominal detections, 25% random
misses, temporary divider/outer-row loss, .4 s all-cone loss, physical interior
cone removal and a missing divider-tip cone. Both no-memory ablations fail to
complete and stop. Four negative cases—prolonged camera loss, stale gyro, stale
wheel speed and unobserved terminal geometry—latch stops without contacts or
excursions. Navigation ends at the return goal gate while moving; final stopping
is not part of that completion score.

Failures remain recorded: one training/DAgger episode stops before sufficient
turn evidence exists (11/12 completions in each collection round). An earlier
frozen set with placement 3000..3007 passed 7/8; its physically sparse startup
stall exposed the admission/planning defects repaired above. That set became
development evidence and remains in `build/three-row-uturn-frozen/report.json`.

The 1 m gap between adjacent lane centres would need a .5 m radius for a simple
semicircle. The assumed +1.0/-0.92 /m steering instead uses a forward R-L-R path
with planning radius 1.18 m. It requires the explicitly declared **4 m deep
clear apron beyond the outer-row ends**, with .9 m extra lateral space on each
side. The largest final raw-camera empty interval is 19.4 s; the oldest usable
cone evidence during preview generation reaches 20.3 s. This depends on ideal
no-slip odometry through a long blind interval. Apron space and full normalized
steering response are unverified on the vehicle; admitted physical steering
calibration has not changed. Yellow divider/blue outers are assumed. The camera
stays fixed at zero pan, 86° FOV, .2 s latency and .03 m detection noise. Lidar
has ideal three-point cone surfaces within 4 m, without physical occlusion or
noise. Speed follows a lagged requested-speed plant, not an ESC model. Active
braking assumes .2 s dead time plus .8 s linear deceleration. The guard reuses
the runtime swept footprint with measured initial curvature, .15 s spatial
steering settling, scan-age motion compensation and reaction-to-centred-braking
interpolation; those assumptions still require physical measurement.

Final artifacts are `build/three-row-uturn-verified/steering.npz`, `report.json`,
`demonstrations.npz`, `evaluation.png` and per-holdout trajectories. The report
freezes model, configuration and executed-source hashes; separate evaluation
rejects mismatches. Artifacts are ignored by Git. Reproduce from the repo root:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.train_three_row_uturn --workers 2 --output build/three-row-uturn-verified
```

Verification: **672 passed, 3 skipped, 1 existing xfailed** in the policy suite,
including **39** three-row checks; **143 passed** in an actual locally filtered
upload copy; **4 release-contract tests passed**. The filtered copy excludes the
new learner, trainer and weights. The required pinned-interface fast gate was
attempted and cannot start because `.verification/dependencies/dream_interfaces`
is absent; this Mac also lacks ROS Jazzy. Read-only Astra review found the
stop/resume, provenance and sparse-view issues repaired above, with no remaining
blocking finding. No ROS selector, car configuration, pan/steering calibration,
SSH session, upload, service or vehicle enable changed. Physical testing is
**not run**. No commit, push, merge or release was performed.

Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.03_adding_and_using_test_data.md`, `2.05_car_memory.md`, and
`3.00_codebase_structure.md`.

## Deployable corridor candidate: 30/30 runtime simulations, 2026-10-20

The selected YAML now runs `action_policy: corridor` through `MovementPolicy`,
the existing normalized `CarController` and ROS parameter declarations. The
upload includes the corridor engine/geometry, package manifest and registered
runtime test; simulation and learned-model files remain excluded. The helper
retains the owner's `CAR_NUM=20`, resolving to confirmed target `.33`, and starts
the required IMU with lidar. No SSH, upload, service change, vehicle enable,
commit, merge or release was performed for this preparation.

The existing cone/edge observer remains. The middle path connects cone geometry,
rounds observed corners at the planning radius, and retains the generated route
in motion-compensated memory. Anchors expire after 6.5 s or 2.6 m; current signed
boundary support remains required. Supporting/duplicate frames cannot renew a
retained anchor. A newly fitted full path can establish a new anchor, including
from one known boundary; a fresh stronger turn can replace a retained straight.
Short terminal straights need fresh separated cones, a learned colour role,
and a bounded backward bridge; they never extend beyond the measured endpoint
or re-anchor a full route. Loss of motion clears both raw and generated routes.

Target speed is .4 m/s with a latched measured-speed stop above .7 m/s. Pan stays
zero. The explicit `full_range_steering` test option maps provisional left
1.0/m and right .92/m to the existing normalized endpoints. This assumes linear
response and the owner's approximate 1 m left radius; it is not calibration or
an increase in hardware limits. False retains the previous measured control
slopes; `action_policy: mpc` restores the previous controller.

All lidar returns are checked without cone-colour exemptions, using a .30 m
front/.30 m rear/.25 m wide footprint and .075 m padding. Candidate checks cover
driving and reaction-then-centred-steering braking, including intermediate stop
times. Spatial interpolation uses separate longitudinal/lateral bounds, explicit
integration error and a pointwise stop-delay bound. The .2 m preferred cone
clearance and weight 3 are soft costs that cannot override hard rejection.
Command deadline is .2 s, with .25 s reaction allowance including supervision.
The node preserves explicit resume, independent zero publication, IMU field
freshness and pan hold. A blocked calculation still blocks its own supervisor.
Node state-2/watchdog zeros coast; the modelled active brake is a policy stop,
not evidence of physical watchdog stopping.

Final frozen mapped-steering evaluation passes **30/30**, with zero contacts
and no full-footprint excursion. Worst polygon clearance is **.11235 m** in
this model. Twenty regression cases use placement seeds 17/127 and detector
seeds 201..205/801..805; ten untouched holdouts use placement 131 and detector
seeds 901..905. Each runs with/without inner-edge occlusion at 6.0..6.4 s.
Completion retains the existing criterion: within .75 m of the end, zero
contacts and no footprint excursion. The scorer was not relaxed.

These are three straight cone segments with independent .025 m placement
jitter, assumed 3 m legs/cross, owner-reported 1 m width/left radius, .2 s camera
latency and .03 m detection noise. The camera model has a body-origin planar
pivot. Lidar provides ideal three-point cone surfaces within 4 m, with no scan
noise or physical occlusion. The mapped mode inverts the configured steering
map; it is not independent actuator calibration. Drive effort is checked, but
the plant follows requested speed rather than simulating ESC/encoder feedback.
Active braking is an explicit assumption: .2 s dead time followed by .8 s
linear deceleration, giving distance/speed .6 s. The guard instead assumes a
.15 s spatial first-order steering response; its interpolation bounds do not
establish agreement with real time-varying braking/steering or frame accuracy.

Reports/plots: `build/corridor-deploy-final-nominal/`,
`build/corridor-deploy-final-regression127/`, and
`build/corridor-deploy-final-holdout/`. Each contains `report.json` with matching
executed-source hashes and `trajectories.png`. Aggregate evidence is
`build/corridor-deploy-final-summary.json`. Reproduce using
`OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m
tests.policy.corridor_runtime_simulation --count 5 --mapped-only --output DIR`;
append `--seed-start 801 --placement-seed 127`, or
`--seed-start 901 --placement-seed 131`, for the other sets.

Software verification: **633 passed, 3 skipped, 1 existing xfailed** in the
policy suite; **143 passed** from the actual locally filtered upload copy;
**4 release-contract tests passed**; Python compilation, shell syntax and diff
checks passed. The required `CONTRIBUTING.md` fast gate was attempted against
the pinned path and cannot start: `.verification/dependencies/dream_interfaces`
is absent (this host also lacks ROS Jazzy). Installed ROS watchdog tests were
added but have not run here. Read-only Astra review found and helped repair
path reversal, anchor reset, deadline and swept-footprint issues; no blocking
issue remained in the final reviewed changes. Physical testing is **not run**.

Rules followed: `0.00_root_rule.md`, `0.01_coding_standards.md`,
`2.00_adding_sensors_and_observations.md`, `2.02_adding_parameters.md`,
`2.05_car_memory.md`, `3.00_codebase_structure.md`, and
`3.01_using_car_management_script.md`.

## Camera path preview: 30/30 versus fixed-camera 26/30, 2026-10-20

The owner suggested panning the camera to keep cones visible during the U.
An opt-in simulated camera now previews the follower's cone-derived path 2 m
ahead, using pursuit curvature times .6 m as its pan angle in radians. The
assumed cap is +/-25 degrees, slew 60 degrees/s and first-order lag .15 s.
Small preview curvatures up to .15/m recenter while moving. Missing paths fall
back to requested steering curvature; stops retain the previous pan target.
Neither true track vertices nor simulator pose enter the camera target.

The existing camera delay queue carries each image's actual acquisition pan.
Cone coordinates are corrected using that angle into acquisition `base_link`,
before existing motion compensation. Default `camera=None` retains the original
detector and scorer. A zero-pan regression checks identical trajectories and
metrics. The detector's simplified planar pivot remains the body origin; real
mount translation, optical/shaft kinematics, blur and physical occlusion are not
modeled. Angle limits and servo response are assumptions, not measured facts.

On **30 matched nominal cases**, path-preview pan passes **30/30**, with zero
cone contacts and no footprint excursion; the fixed camera passes **26/30**.
Worst path-pan polygon clearance is .1586 m. Frame-weighted visibility of both
cone colours during turns increases from **46.5% to over 99.9%**. This measures
raw colour presence when absolute requested steering curvature exceeds .3/m,
not sufficient cone counts/span for a validated lane fit. Comparisons use the
unchanged provisional 1 m width/left-radius profile, .4 m/s, .2 s image latency,
.03 m detector noise and strict full-footprint/contact/completion scoring.

Twenty regression cases use placements 17/89 and detector seeds 501..505, each
with/without the 6.0..6.4 s inner-edge occlusion. These new cases exposed a shared
seed505 failure with fixed or steering-reactive pan; earlier path preview fixes
those cases without changing the steering controller. Frozen evaluation on
placement 113 and detector seeds 601..605 adds ten fresh cases, again with and
without occlusion; both fixed and path-pan pass those ten. The earlier fixed
camera's 100/100 result below remains scoped to its recorded case set and is
not a universal robustness claim.

Steering-reactive pan alone passes 16/20 regression cases, despite improving
visibility to 97.7%; looking ahead before steering is material here. Deliberately
omitting acquisition-pan coordinate correction fails all 20 regression cases.
This error control is not a model of correct physical-camera integration.
No ROS selector, normalized pan command, live service or configuration changed.
Physical use requires calibrated optical/shaft geometry, measured pan feedback
and acquisition-time dynamic camera TF in the detector/DREAM producer. Existing
runtime fixed/zero-pan behavior remains in place.

Aggregate metrics/hashes: `build/camera-path-pan-summary.json`; per-case reports
and plots in `build/camera-path-pan-regression/` and
`build/camera-path-pan-fresh/` (`report.json`, `trajectories.png`, `visibility.png`).
Reproduce with `OPENBLAS_NUM_THREADS=1 .venv/bin/python -m
tests.policy.camera_pan_simulation --output build/camera-path-pan-regression`,
then append `--output build/camera-path-pan-fresh --seed-start 601 --placements
113 --modes fixed path_pan25` for the fresh set. Source hashes match across both
evaluations. Software tests: `603 passed, 3 skipped, 1 existing xfailed`.
Read-only Astra review found no concrete issue or route oracle. Required ROS
fast gate attempted; pinned `dream_interfaces` checkout is still absent.

## Three-line U-turn follow-up: 100/100 nominal runs, 2026-10-20

The connected-corridor candidate now completes **100/100 nominal simulations**:
80 regression cases and 20 fresh holdouts, all with zero cone contacts and no
full-footprint corridor excursion. Existing scoring is unchanged: completion
means within .75 m of the end, together with those two clearance conditions.
The worst simulated polygon clearance is .1059 m. This is simulation evidence,
not a physical clearance or stopping qualification.

The failure fixes keep the existing cone/edge observer. Fresh single-boundary
ordering now starts forward, groups close corner samples, fits straight stretches
while preserving corners, and removes loops caused by inward offsets. Grouping
changes the fitted boundary only; the collision inputs retain the original cone
samples. The follower uses 1.2 m lookahead. Saved routes retain their original
time/travel limits, require current supporting observations and cannot be renewed
by a single edge. Work is bounded by the existing cone-count limit.

The profile remains provisional: owner-reported 1 m lane width and 1 m left
turning radius, inherited .92/m right curvature, assumed 3 m legs/cross,
forward-only .4 m/s, .2 s camera latency, .03 m detector noise, valid gyro and
measured-speed feedback. The scorer, footprint, steering response and geometry
were not relaxed during this follow-up. The old car20 profile and runtime YAML
remain unchanged. This offline follower is excluded from upload and has no ROS
selector; physical testing remains not run.

Regression placement seed 17 uses detector seeds 201..210; placement seeds
23/41/67 use detector seeds 301..310. Each is run with/without inner-edge
occlusion at 6.0..6.4 s. These 80 cases informed the fixes. After freezing the
candidate, placement seed 89 with detector seeds 401..410 provides 20 fresh
cases, again with/without occlusion. Source hashes match across all families.
The separate +/-10% radius/width sensitivity grid passes 6/9; it is not included
in the nominal 100. Approximate geometry is not established calibration.
The same-profile arc baseline still fails. The learned network is not used.

Aggregate metrics and hashes: `build/uturn-resolve/final-summary.json`.
Individual reports/plots: `final-regression/report.json` and `validation.png`,
`expanded/report.json` and `trajectories.png`, `fresh/report.json` and
`trajectories.png`, all below `build/uturn-resolve/`.
Reproduce the first 20 plus sensitivity/baseline with
`.venv/bin/python -m tests.policy.car21_uturn_simulation --output
build/uturn-resolve/final-regression`; the other exact drivers are
`PYTHONPATH="$PWD" .venv/bin/python build/uturn-resolve/expanded.py` and
`PYTHONPATH="$PWD" .venv/bin/python build/uturn-resolve/fresh.py`.

Verification: `597 passed, 3 skipped, 1 existing xfailed` for the policy suite.
New regressions cover forward boundary ordering, corner preservation, offset-loop
removal and close corner samples. Read-only Astra review found no concrete
remaining issue. The required pinned-interface fast gate was attempted again;
it cannot start because `.verification/dependencies/dream_interfaces` is absent.

## Provisional car21 width/radius: earlier 18/20 stage, 2026-10-20

The owner reports a path about 1 m wide and a turning radius about 1 m, and has
manually driven the real U. These are provisional user measurements, not the
car20 calibration previously used by the simulator. The new evaluation uses
lane width 1 m and left maximum curvature 1/m. Right curvature remains .92/m
pending its own measurement. Three-segment shape, 3 m legs/cross, .025 m cone
placement jitter and placement seed 17 are retained; the 3 m dimensions remain
assumed. The old car20 profile, original .8 m track defaults and runtime YAML
are unchanged. The previous infeasibility calculation remains scoped to the
old profile and does not describe the owner's reported capability.

The offline connected-corridor follower now has opt-in world-frame path memory
and fresh single-boundary continuation. Saved routes require fresh support from
both colours on opposite signed sides, minimum counts and .25 m spatial span.
Colour roles come from this fresh geometry, not the augmented observer. Cached
routes retain original acquisition time/distance and are transported/trimmed
as the car moves; invalid motion or empty detections clears them and stops.
The evaluation bounds route memory at 6.5 s/2.6 m. Single-edge observations never
renew that anchor. When at least three fresh same-colour points form a connected
boundary over the lookahead distance, its current measured polyline can be
offset inward using the known 1 m width and learned colour role. This is fresh
geometry, independent of old-route expiry. It uses bounded cone gaps and
corner-direction changes; existing per-colour arc observation remains available.
The feature defaults off, and the whole corridor follower remains excluded from
car upload and absent from the ROS selector.

Historical report from `.venv/bin/python -m tests.policy.car21_uturn_simulation
--output build/car21-provisional-uturn-heldout`, using the source hashes saved
in that report. The current driver uses the follow-up tuning above and does not
reproduce this earlier result. Target speed .4 m/s, lookahead 1.1 m;
forward-only kinematic model with .2 s camera latency, .03 m detector noise,
valid gyro and measured-speed feedback. Development used seeds 0..4 and
101..110; final held-out seeds 201..210 were frozen before evaluation, each
with/without inner-cone occlusion at 6.0..6.4 s. **18/20 complete the U** according
to the unchanged lane-kept scoring (within .75 m of end), **all 20 have zero cone
contacts and no footprint excursion**. The seed209 pair stops early around
6.235 m progress. Worst corridor clearance across the cases is about .0289 m;
this is polygon clearance, not a physical stopping/clearance qualification.
The same provisional profile's arc-based baseline fails its comparison case.

Report and source hashes: `build/car21-provisional-uturn-heldout/report.json`;
plot: `validation.png`. The radius/width sensitivity grid shows that +/-10%
parameter changes do not consistently pass, so the approximate measurements
must not be promoted as exact calibration. This result belongs to the connected
geometry follower, not the earlier learned steering network (whose saved
steering contract remains .48/.92). Runtime integration, lidar safety and
physical testing are not performed; no vehicle services were contacted.

Verification: `593 passed, 3 skipped, 1 existing xfailed` for the policy suite,
including saved-route expiry, non-renewal, motion reset, one-edge evidence,
serialization, signed-colour support and fresh corner-boundary offset tests.
Required pinned-interface fast gate attempted; dependency checkout still absent.
Read-only Astra review found no remaining concrete defects in the offline scope.

## U-turn specialist and steering-range investigation, 2026-10-20

The owner requested continued forward-only investigation, then authorized a
local U-turn-only learned model and code-level steering-limit investigation.
No hardware configuration, calibrated fact, physical steering mapping, track
geometry or pass criteria was changed. The new student module is excluded from
car upload and is not selectable by the ROS policy.

`python -m tests.policy.train_uturn_specialist --output build/uturn-specialist`
trains a NumPy/SciPy 7->32->32->12 tanh network from scratch on 3,470 MPC steering
horizon demonstrations. Twelve teacher episodes (smooth U radii 2.5, 3.2, 4 m;
left/right; +/-0.08 m start offset; seeds 0..11) all meet the existing lane-kept
criterion. Only fitted lane state, previous requested curvature, sensor age,
measured speed and requested lane offset enter the network. Training-only
normalization and episode-disjoint radii/seeds prevent evaluation leakage.
No external pretrained checkpoint was supplied or downloaded. L-BFGS uses a
400-iteration budget; it reaches that limit with objective 2.696e-5, so numerical
optimizer convergence is not claimed.

The saved `build/uturn-specialist/uturn-steering.npz` contains 15,976 bytes of
weights and the steering/horizon contract. Invalid/nonfinite/negative motion
inputs, a normalization-range excursion beyond 8, or incompatible model settings
reject inference; invalid inference uses the existing latched stop framework.
Curvatures stay within unchanged asymmetric .48/.92 1/m limits. Finite bounded
network outputs do not certify collision clearance, optimality or hardware
safety. Existing speed, supported-lane and lane-loss logic remains in MpcPolicy.
No MPC optimizer runs inside the student controller's inference call.

Closed-loop holdouts use radii 2.8 and 3.6 m with seeds 101..104, including a
0.4-second inner-edge occlusion. Student and teacher both pass 4/4 smooth cases
with no contact or footprint excursion. On the unchanged original three-line
U, seeds 105/106, the student fails to finish: its input-range check stops it at
1.86/1.82 m progress, before the corner, with no contacts or excursion. The
teacher fails both cases with contacts. This is not a learned solution to the
original angular U or evidence of improvement over MPC on feasible layouts.
Report, source hashes and plot: `build/uturn-specialist/report.json` and
`evaluation.png`. Tests: `584 passed, 3 skipped, 1 existing xfailed`, including
10 new inference-contract regressions. The required pinned-interface fast gate
was attempted but cannot start without `.verification/dependencies/dream_interfaces`.
Astra performed a read-only review; no concrete remaining defects were found.

Forward feasibility audit: `build/uturn-forward-feasibility/audit.json` and
`feasibility.png`. The .25 m footprint tightens the necessary vertical span to
`2/.48 + .25 = 4.4167 m`, exceeding the available 3.8 m by .6167 m. This rules out
forward paths reaching unwrapped heading +pi, including intermediate steering
reversals. The necessary left curvature .5634 1/m is not sufficient to establish
corner clearance. Opposite winding (-pi) is outside this simple proof. Two
bounded forward-only, full-map lattice searches found no route at 5 cm/5 degree
and 2.5 cm/2.5 degree resolution; the latter exhausts 16,690 retained states.
Search failure is not a proof of all forward-only alternatives. Search collision
checks were cross-checked against the simulation polygon/body scorer on 500
random poses. None of these bounds establish the uncalibrated physical car21's
capability; they describe the current car20-based simulation profile.

Software steering-range test: control already permits normalized steering +/-1.
The linear control map extrapolates a full-left action to .642722 1/m and
full-right to .939597 1/m, whereas car20's measured left saturation is .48 1/m.
Increasing the planner cap alone while retaining the measured physical model
does not solve the U. An explicitly hypothetical physical response matching
that extrapolation also fails the tested MPC and connected-corridor followers
(lookaheads 1.2..2 m, speed .4 m/s). The candidate was not promoted to local
runtime YAML or uploaded. Results and driver:
`build/steering-range-experiment/summary.json` and `run.py`. Setting a software
number larger cannot establish an increased mechanical steering range.

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
