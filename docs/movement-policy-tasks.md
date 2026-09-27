# Movement Policy tasks: Stage 1

What the Movement Policy (MPC/RL) group has to do for **Stage 1: follow a
straight lane with even cones**, written as Planner cards. Each task can be
done by one person without waiting for anyone else; use the shared defaults
below where a number is still unknown and say so in your result. Each `- [ ]`
line is one checklist item.

## What this group does

Road Detection gives us the lane ahead and where the car is in it. We decide
**where to drive and how fast**, and pass a speed and a curvature (how sharply
to turn) to Car Control.

**Quality bar.** At least the level of the Upskilling log-book: requirements
become metrics, observations are justified, several MPC and RL options are
compared on a Pareto front, and the chosen design is evaluated on cases never
used for tuning. The final policy uses no simulator ground truth, and Stage 1
must also pass on the real car.

| | Upskilling assignment | This project |
| --- | --- | --- |
| Vehicle | Tesla-sized simulated car | 1/10 Traxxas car, plus a simulation of it |
| Our output | Drive and steering commands | Speed and curvature, converted by Car Control |
| Observations | Simulated sensors | Cone perception: short range, noise, delay, missed cones |
| Requirements | R1-R4 (speed, smoothness) | Stage 1, real time, safe stop when the lane ends or is lost |
| Evaluation | Unseen road in simulation | Unseen cases in simulation, then the real car |

**Plan:** develop both MPC and RL for Stage 1 to the quality bar, choose one
with the same tests and metrics, then run it on the car.

## Shared defaults

Starting values from dream-gym's cone-following example
(`examples/cone_following_with_simple_policy.py`). Replace them when measured.

| Item | Default | Confirmed by |
| --- | --- | --- |
| Car model | 1/10 Traxxas Slash parameters from that example | Chris |
| Maximum steering angle | 45 degrees (likely larger than the real car) | Chris |
| Lane width | 1.0 m | Task 2.2 |
| Cone spacing | 0.5 m | Task 2.2 |
| Which colour is on which side | No default; dream-gym treats colours as labels per road | Long |
| Camera view | 80 degrees wide, 4 m range, 0.01 m position noise | Task A1 |
| Target speed | 1.0 m/s | Chris |
| Time allowed per policy update | 50 ms | Task 1.5 |

## Log-book entries

Every design task records its work the same way:

- Starting values and the reason or source for each.
- For each iteration: what changed, why, and what was observed.
- Final values, why they were kept, and the supporting metrics table and trajectory figure.
- Any unexpected or failed result, what caused it and what it changed.

## Card 1: Determine general approach (PW, due 07/10)

### 1.1 Turn Stage 1 into requirements and metrics

- [ ] Write Stage 1 requirements: stay in lane, no cone touched, hold speed, smooth, stop at lane end
- [ ] Add real time and safe stop when the lane is lost
- [ ] Give each requirement one measurable metric, using dream-gym built-in metrics where possible
- [ ] Add solver fallback count and 99th-percentile calculation time as metrics
- [ ] Check every requirement is covered and no two metrics measure the same thing

**Output:** requirement and metric table in `docs/`.

### 1.2 Choose and justify the observations

- [ ] List what the real car can provide: lateral error, heading error, wheel speed
- [ ] Add lane edges and IMU heading if useful; note which group provides each
- [ ] Write why the policy needs each one
- [ ] Confirm nothing on the list is simulator ground truth

**Output:** one observation table, used by both MPC and RL.

### 1.3 Set the MPC vs RL decision rule

- [ ] State that both are tested on the same cases, metrics and disturbances
- [ ] Define the winner: on the Pareto front and meets every Stage 1 threshold
- [ ] Say what happens on a tie, e.g. prefer the easier one to debug on the car

**Output:** one paragraph in `docs/`.

### 1.4 Agree inputs and outputs with Long and Chris

- [ ] List what we receive from Long: lane ahead, car position in it, lane edges
- [ ] Agree which cone colour is on which side of the lane
- [ ] List what we send to Chris: speed in m/s and curvature in 1/m, left turn positive
- [ ] State what happens with no lane or a doubtful lane: stop
- [ ] Get Long and Chris to confirm in writing

**Output:** one confirmed table in `docs/`.

### 1.5 Measure the time budget on the car

- [ ] With the cone detector running, read the detection rate in Foxglove
- [ ] Note the gap between detections and the vehicle's 0.5 s command timeout
- [ ] Set the time the policy may take per update, well below the detection gap

**Output:** detections per second and allowed calculation time; update the defaults.

## Card 2: Design test cases for the car to follow (TL, due 07/10)

Two case sets, as in the assignment: **tuning cases** for design and **unseen
cases** kept back for the final comparison. Starting point:

| Case | Set | Start | Speed | Disturbance |
| --- | --- | --- | --- | --- |
| T1 | Tuning | Centred, straight | 1.0 m/s | None |
| T2 | Tuning | 0.15 m to the left | 1.0 m/s | None |
| T3 | Tuning | 10 degrees off | 1.0 m/s | None |
| T4 | Tuning | Centred, straight | 1.0 m/s | Detector noise and one missed cone |
| U1 | Unseen | 0.2 m to the right, 5 degrees off | 0.7 m/s | Detector noise and delay |
| U2 | Unseen | 0.1 m to the left, 15 degrees off | 1.0 m/s | Two missed cones and a wrong colour |

All cases use a 6 m straight lane with even cones. Example pass rule: no cone
touched, inside the lane to the end, and stopped within 0.5 m after the last
cone.

### 2.1 Write the full test case table

- [ ] Write at least four tuning cases and two unseen cases
- [ ] Give each case pass thresholds on the Card 1.1 metrics
- [ ] Keep the unseen cases away from whoever tunes MPC and RL

**Output:** one table in `docs/`; unseen cases stored separately.

### 2.2 Draw the physical cone layout

- [ ] Draw the Stage 1 lane to scale: length, lane width, cone spacing
- [ ] Mark the start line, start offsets and where the car must stop
- [ ] Check it fits the lab floor; count the cones needed per colour

**Output:** a printable drawing in `docs/`; update the defaults.

### 2.3 Build the same lane in simulation

- [ ] Recreate the drawing in the dream-gym Road construction notebook
- [ ] Start from the small-scale cone track in dream-gym's road examples
- [ ] Save a picture of the simulated lane beside the drawing

**Output:** lane settings and picture in `docs/`.

### 2.4 List the disturbances to test

- [ ] List real problems: missed cone, wrong colour, extra object, fallen cone, glare
- [ ] Write how to create each on the floor (remove a cone, swap colours...)
- [ ] Give each a simulation setting: detector noise, range, delay, car mass or grip

**Output:** a disturbance checklist for real and simulated tests.

### 2.5 Make the test record sheet

- [ ] Make a shared sheet: date, case, design, speed, pass/fail, video, notes
- [ ] Fill in one example row from a dry run

**Output:** a shared spreadsheet, linked from `docs/`.

## Card 3: Add appropriate algorithm-related tasks (PW, due 07/10)

- [ ] Create one Planner card for each task A1-A7 and milestone M1-M2 below
- [ ] Assign an owner to each; owners confirm they can start with the shared defaults
- [ ] Review progress weekly and update the shared defaults

### A1 Measure how far and how well the camera sees cones

- [ ] Place cones in front of the car at 0.5 m steps up to 5 m, ahead and to the sides
- [ ] Record which cones are detected and their reported positions in Foxglove
- [ ] Measure the true positions with a tape and compare
- [ ] Repeat with cones near the edge of the camera view

**Output:** seeing distance, view width and position error; update the defaults and
share with Long.

### A2 Set up a simulation that behaves like our car

- [ ] Start from the car in dream-gym's cone-following example
- [ ] Set the cone detector view, range and noise to the shared defaults
- [ ] Add a road-information delay equal to the detection gap from Task 1.5
- [ ] Make the policy output speed and curvature, with a simple conversion like Chris's
- [ ] Share the setup with everyone doing A3-A5

**Output:** one agreed simulation setup that every design task uses.

### A3 Get a first baseline with the cone detection notebook

- [ ] Run the dream-gym cone detection notebook with the shared car and detector
- [ ] Try the lookahead and gain values it suggests on the Stage 1 lane
- [ ] Record the Card 1.1 metrics with and without detector noise

**Output:** baseline numbers that MPC and RL must beat.

### A4 Design the MPC (log-book)

- [ ] Start from our assignment MPC; record starting values and reasons
- [ ] Make at least three options that differ in horizon, weights or constraints
- [ ] Keep the horizon within the camera range and the time budget
- [ ] Tune on tuning cases only, with the Card 2.4 disturbances
- [ ] Plot the options on a Pareto front; choose one with zero fallbacks

**Output:** MPC log-book entries and the chosen design.

### A5 Design the RL policy (log-book)

- [ ] Write reward options from the Card 1.1 requirements, with shapes and weights
- [ ] Train at least three options that differ in reward or hyperparameters
- [ ] Vary detector noise, delay and car parameters during training
- [ ] Watch for reward exploits: hugging an edge, weaving, stopping
- [ ] Plot the options on a Pareto front and choose one

**Output:** RL log-book entries and the chosen design.

### A6 Measure the calculation time on the car computer

- [ ] Run the assignment MPC at several horizon lengths on the car computer
- [ ] Record typical and 99th-percentile calculation time for each
- [ ] Find the longest horizon that fits the time budget

**Output:** horizon against time table; limits A4.

### A7 Decide what to do when the lane ends or cones disappear

- [ ] List when the car sees no cones or only one side: lane end, missed cones, glare
- [ ] Decide how long to keep going, how much to slow down and when to stop
- [ ] Write the rule in 3-5 lines with the numbers

**Output:** a short rule used by both MPC and RL.

## Milestones

These combine earlier tasks, so they come later.

### M1 Choose MPC or RL

Needs 1.3, 2.1, A4 and A5.

- [ ] Run both chosen designs on the unseen cases with the same disturbances
- [ ] Put the results in one metrics table with trajectory figures
- [ ] Apply the Card 1.3 rule and record the decision

### M2 Stage 1 on the car

Needs 1.4, 2.2, 2.5, M1 and Car Control's speed and curvature conversion.

- [ ] Run every case three times and fill in the record sheet
- [ ] Compare each run with the same case in simulation; explain differences

## Timeline

- By 07/10: Cards 1 and 2, A2 and A3.
- Next two weeks: A1 and A4-A7, ending with M1.
- Then: M2.

## Later stages

Not part of this phase. Stages 2-6 (lane widths, curves, intersection, QR
commands, obstacles) reuse the same process; plan them after M2.

## Questions for other groups

- **Long:** which cone colour is on which side?
- **Chris:** maximum steering angle, usable speed range and reaction delay.
- **General setup:** the repository keeps all policy code in one file
  (`scripts/policy_node.py`). How do the planned folders fit that?
