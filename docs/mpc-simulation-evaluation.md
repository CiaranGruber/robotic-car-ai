# MPC simulation screening — 2026-10-06

Three configurations were compared in 60 tuning runs, followed by 10 validation
runs. All tracked the lane without solver fallback, but all stopped before the
6 m finish. **No configuration passes the provisional completion gate.** Runtime
MPC code and car configuration were unchanged in that initial experiment.
The subsequent algorithm iteration below changes MPC runtime behavior.

## Reproduce

From the repository root with Python 3.12 or later:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pytest tests/policy -q
.venv/bin/python tests/policy/evaluate_mpc.py --output build/mpc-evaluation
```

The evaluator writes `metrics.csv`, `summary.json` (configurations, cases,
versions and source hashes), `trajectories.png`, and `pareto.png`. Outputs are
ignored local build artifacts. Source is `tests/policy/evaluate_mpc.py`.

## Assumptions

This is kinematic screening, not DREAMGym or a calibrated vehicle simulation.
It exercises the existing cone filter and MpcPolicy. Speed and curvature are
applied ideally at 100 Hz; policy updates are 10 Hz. It excludes CarControl,
motor effort, coasting/braking, steering slew, tyre dynamics, ROS timeouts and
vehicle watchdogs. Speed tracking cannot be evaluated independently here.
Empty batches represent fresh empty detections, not detector-stream loss.

The 6 m lane is 1 m wide with 0.5 m cone spacing. The camera has an 80-degree
view and 4 m radial range, following movement-policy-tasks.md starting values.
Ground truth is used only for sensor generation and scoring. Collision checks
use illustrative circular radii of 0.18 m for the car and 0.05 m for cones.

T1 starts centred; T2 is 0.15 m left; T3 has 10-degree heading error. T4 combines
0.15 m offset, 0.01 m Gaussian noise, 0.2 s delay, 10% individual misses and
every third batch empty. Tuning uses 1 m/s and seeds 0–4; world-cone random draws
match across candidates. Noiseless repetitions do not add statistical power.
Validation uses separate scenarios and seeds 100–104, after candidate selection.
These are public scenarios with held-out seeds, not blinded unseen cases.
Validation results were not used to modify candidates. Retuning from them
requires new independent final cases.

The provisional gate requires positive cone clearance, the illustrative
footprint inside the lane, zero fallbacks, p99 update below 50 ms, and a latched
stop with the car centre at x = 6.0–6.5 m. The Pareto objectives are lateral RMS
and curvature-rate RMS. Select the lowest-error Pareto candidate only if all
its tuning runs pass. Otherwise validate the lowest-error Pareto candidate
diagnostically and keep `selected` null. Thresholds were fixed before execution.

## Results

Baseline uses existing MpcConfig defaults. Longer changes horizon from 12 to
20 steps; smooth uses 20 steps and increases curvature-change weight from 1 to
4. Other weights remain lateral 10, heading 1, curvature 0.1. At 1 m/s and
0.1 s steps, horizons cover 1.2 or 2 m, below the assumed camera range (not a
guarantee that the horizon remains observed near the lane end).

| Candidate | Mean lateral RMS (m) | Mean curvature-rate RMS (1/m/s) | Worst run p99 update (ms) | Fallbacks | Passing runs |
| --- | ---: | ---: | ---: | ---: | ---: |
| Baseline | 0.02974 | 0.27902 | 2.20 | 0 | 0/20 |
| Longer | 0.02962 | 0.28323 | 1.20 | 0 | 0/20 |
| Smooth | 0.03329 | 0.19038 | 1.05 | 0 | 0/20 |

All three lie on this empirical Pareto front. Smooth reduces curvature-rate RMS
by about 32%, with about 12% higher lateral RMS than baseline. Longer's tiny
tracking advantage does not justify promotion. Timing covers filtering and
policy calculation on this Mac, including first-use costs, not sensor generation.
It is not a car-computer timing bound or evidence that longer horizons run faster.

Tuning stops occur at x = 5.19–5.34 m. Longer was evaluated diagnostically in
validation: 0/10 pass, stops at 5.13–5.29 m. Minimum illustrative cone clearance
is 0.12 m in tuning and 0.069 m in validation. All remain within the lane bound.

The finite camera view loses a usable two-cone row before reaching the last
cones. The policy then briefly follows the old plan at half speed and latches
stopped. Weight tuning does not solve this failure. Next, represent remaining
lane distance and distinguish an observed lane end from uncertain lane loss.
Increasing the blind-driving timeout simply to reach the finish is unsupported.

## Verification and next work

66 hardware-free policy tests pass, including finite visibility, delayed
startup and Pareto checks. The required ROS fast gate was attempted but stopped
because the pinned `.verification/dependencies/dream_interfaces` checkout is
absent. No ROS, DREAMGym dynamic, or physical acceptance is claimed. The DREAM
guide URL was unavailable through browsing; local repository rules were read.

Next independent evaluation should include DREAMGym dynamics, measured actuator
conversion, wrong-colour detections, permanent stream loss and an agreed lane-end
rule. These results do not establish physical stopping or frame accuracy.

## Algorithm iteration: measured speed and bounded failure behavior

The MPC now rebuilds its linear prediction matrices using fresh wheel speed
when available, holding that speed constant over the horizon. Missing feedback
retains the nominal-speed model; a measured zero is not replaced with target
speed. Delay propagation uses the same measured speed, still assuming the
previous commanded curvature was applied during the delay. This is not a
history-based state estimator, and unsigned wheel speed assumes forward travel.
The matrix construction is vectorised; it avoids repeated matrix powers.

A failed solve now requests zero speed/curvature and remains stopped until an
explicit new run resets the policy. Solver outputs must have the correct shape,
finite values and satisfy the curvature bound. Numerical solve failures return
an invalid plan rather than letting arbitrary iterates become commands.
Invalid motion inputs raise an exception for the node's existing stop handling.

During brief lane loss, cached-plan progress uses wheel speed over the last
policy interval when available (approximate odometry). The plan retains the
spatial spacing from its solve. Reaching its distance limit stops the policy
instead of indefinitely repeating its final curvature. A plan made at zero
speed cannot authorise moving after lane loss. The existing lane-loss timeout
and explicit-restart behavior remain. No live configuration was changed; YAML
changes only explain the new behavior.

### Speed sensitivity experiment

```bash
.venv/bin/python tests/policy/evaluate_mpc_speed.py
```

Outputs are in `build/mpc-speed-evaluation/`. This compares the same new
controller with nominal-speed versus measured-speed inputs, isolating that
choice rather than comparing two different solver-failure implementations.
Both use MpcConfig defaults (1 m/s, horizon 12); this is not a test of every
setting in the car YAML (0.5 m/s, horizon 30). Five seeds (200–204) are run per
model and scenario, for 30 runs. These are development sensitivity scenarios,
not independent final validation, and no settings were tuned from their results.

S1 uses a 0.5 s first-order speed response. S2 adds a steady achieved-speed
ratio of 0.5. S3 uses a 0.3 s response, ratio 0.6, and every third detection batch
empty. All use 0.2 s perception delay and 0.01 m cone noise, with initial lateral
and heading errors specified in the script and output JSON. These are invented
sensitivity values, not identified car dynamics. The simulator continues to
integrate residual speed after a zero command, until speed is below 0.01 m/s.
Curvature still responds ideally; CarControl and DREAMGym are not simulated.

Tracking is scored over the same first 4.5 m for every run using a spatial
integral of squared lateral error. This avoids rewarding a slower trajectory
or a longer stationary tail through time weighting. All runs reached that
scoring distance. The endpoint completion gate is unchanged and still fails.

| Scenario | Nominal-speed RMS (m) | Measured-speed RMS (m) | Reduction |
| --- | ---: | ---: | ---: |
| Acceleration | 0.07158 | 0.07072 | 1.2% |
| Half achieved speed | 0.08314 | 0.08173 | 1.7% |
| Slow with dropouts | 0.07532 | 0.07265 | 3.5% |

All 30 runs had zero solver fallbacks. Worst observed per-run p99 filtering plus
policy time was 1.58 ms on this Mac, not a vehicle-computer bound. Stops remained
short of the finish (approximately 5.32–5.88 m across these dynamics cases).
There is a small consistent tracking benefit here, not evidence of broad
robustness or successful lane-end handling.

83 policy tests pass, including independent rollout-cost optimality, zero-speed
prediction, invalid inputs/results, numerical solver failure, explicit restart,
plan exhaustion, measured plan progress, and coasting in the evaluator. The
required ROS fast gate was attempted again and remains blocked by the missing
pinned interfaces checkout. Before merge, these runtime safety changes require
recorded human diff review under AGENTS.md.

## Algorithm iteration: final-cone-pair stopping prototype

The owner selected the **final cone pair** as the finish cue. The new
`policy/action_policy/endpoint.py` implements an experimental EndpointMpcPolicy
that is explicitly selected by the offline evaluator. MovementPolicy and the
ROS node still select the existing MpcPolicy. EndpointConfig values are offline
experiment arguments, not new ROS parameters or calibrated vehicle settings.

The estimator compares each colour's farthest cone along a usable straight-lane
reference. Both rows need at least two cones. The farthest distances must agree
within 0.15 m and lie within 2.5 m, comfortably inside the assumed 4 m camera
range. Three distinct, increasing acquisition timestamps must yield consistent
endpoint positions after wheel-speed motion and observation-age compensation.
Empty frames, one-sided rows, repeated stamps and stale data cannot confirm it.
A farther contradictory detection aborts the approach. This establishes a
consistent hypothesis, not proof that no hidden cones exist farther ahead.

Once confirmed, the experiment caps requested speed at 0.5 m/s and uses
`v * reaction_time + v² / (2 * deceleration) <= remaining_usable_distance`.
The assumed reaction time is 0.3 s and deceleration is 0.5 m/s². The target car
centre is 0.15 m beyond the pair, reduced by position uncertainty and a 0.02 m
stop tolerance. These are design assumptions, not measured brake performance.
This is a longitudinal speed envelope around the lateral MPC, not a coupled
longitudinal/lateral optimisation. CarControl has no calibrated braking model.

During fresh empty detections after confirmation, wheel speed and commanded
curvature propagate the last straight reference. Only this opt-in prototype
replaces the ordinary brief lane-loss plan with that terminal approach. It
permits at most 4 s without a fresh usable reference, 10 s in the approach, and
0.2 m heuristic uncertainty (initially 0.04 m, growing 0.08 m per metre without
endpoint refresh). Those bounds are not statistical confidence guarantees.
Missing/stale detector streams, missing speed, contradictory evidence, solver
failure or exceeded bounds request zero and remain stopped. A new explicit
run is needed to reset either an abort or an endpoint stop.

### Reproduce and results

```bash
.venv/bin/python tests/policy/evaluate_endpoint.py
```

The 70-run development experiment compares measured-speed baseline and endpoint
mode on seven cases, five seeds (300–304) each. Outputs, exact settings, source
hashes and dependency versions are in `build/mpc-endpoint-evaluation/`. The
finish coordinate is used only by the scorer and sensor generator, not passed
to the policy. Identical seeds in noiseless cases are deterministic repetitions.
Settings were held fixed across the batch; these are not blinded final cases.

| Scenario | Baseline completion | Endpoint completion | Endpoint stop x (m) |
| --- | ---: | ---: | ---: |
| Ideal, initial offset | 0/5 | 5/5 | 6.005 |
| Noise, 0.2 s delay, 0.3 s speed lag | 0/5 | 5/5 | 6.082–6.093 |
| Brief early occlusion | 0/5 | 5/5 | 6.082–6.093 |
| Final pair hidden | 0/5 | 0/5 | 5.583 |
| All cones from x=4 m hidden | 0/5 | 0/5 | 3.583 |
| Detection stream fails at t=5 s | 0/5 | 0/5 | 4.497 |
| One-second occlusion during approach | 0/5 | 5/5 | 6.083–6.095 |

All 20 endpoint runs in the four finish-visible scenarios meet the existing
completion gate (centre at 6.0–6.5 m, stopped, footprint in lane, positive cone
clearance, no solver fallback and p99 below 50 ms). No solver failed in any of
the 70 runs. Worst observed per-run p99 filtering plus calculation was 1.01 ms
on this Mac. The model retains ideal curvature actuation, an illustrative
circular footprint and first-order speed response; it is not DREAMGym or
hardware acceptance.

The two persistent-visibility failures each produce **5/5 false endpoint
declarations**: the preceding visible pair is indistinguishable from a genuine
finish. Temporal confirmation cannot eliminate this ambiguity. Stream failure
requests zero and latches the abort in all five endpoint runs; failure to
complete that scenario is expected fault handling, not a successful finish.

96 hardware-free policy tests pass, including endpoint evidence, stopping
envelope, contradiction, confidence timeout, explicit restart, delayed/coasting
completion and the missing-final-pair counterexample. The ROS fast gate was
attempted again and remains blocked by the absent pinned interfaces checkout.
`git diff --check` passes. No live robot changes or physical tests were made.

Before considering ROS integration, test the prototype against DREAMGym and
record measured stopping behavior. Final-pair inference requires accepting its
visibility assumption or adding independent evidence; it must not be presented
as robust to persistent occlusion. The existing runtime remains unchanged by
this endpoint iteration.
