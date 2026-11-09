# AI4R System Project: team logbook (draft)

The logbook supports the interactive oral, so it uses diagrams, plots, tables and short bullets rather than report
prose. It's shown on the projector, and each person moves through their own section while answering questions.

**Status:** this draft began on 4 Nov 2026 with Toby's section. Completed individual sections are linked below;
the overview and remaining sections await their owners. Before submission, the team should check the
[things to agree](#things-to-agree-before-we-submit) list so no two sections propose conflicting things.

## How to use it

- Each person writes their own file in this folder, `<name>.md`, with their figures in `<name>/figures/`.
- Each section follows the same order as the marking standards, so a question about any standard has an obvious place
  to go:

  | Part | What it shows | Standard |
  | --- | --- | --- |
  | Where it fits | Block diagram, inputs, outputs, interfaces, assumptions, constraints | 1 System-level |
  | How it works | Equations, algorithms, parameters and what they mean | 2 Theory |
  | Simulation | The question, method, comparisons, failure cases, and what we can and can't conclude | 3 Simulation |
  | On the car | The question, method, controlled variables, results and limits | 4 Real world |
  | Figures | Every figure has labels, units, a legend and a caption that says what to look at | 5 Visual |

- Give every claim a commit, PR, log or result file, so a "how do you know?" question can be answered by pointing at it.
- Mark simulation-only results as simulation, and say whether each one has been checked on a car.

## 1. Team overview (team lead, max 3 slides in the talk)

_To fill in:_
- Goals and requirements: the stage 1 and stage 2 behaviours and the cards.
- What the system achieved on the car, with the "best" video.
- Who did what, as sub-teams and individuals over time.
- The integration timeline: releases, PRs and car sessions.
- A whole-system block diagram. [Toby's diagram](toby-lee.md#1-where-my-parts-fit-standard-1) is a starting point;
  replace the "teammates" boxes with each owner's part.

## 2. Individual sections

| Person | Sub-team / topics (owner to confirm) | Section |
| --- | --- | --- |
| Toby Lee | MPC sub-team: Card 2 test cases and harness, car 20 MPC tuning, lane-side following, ArUco marker commands, intersection turns, marker simulation | [toby-lee.md](toby-lee.md) |
| Ciaran Gruber | | _to add_ |
| Peiyu Wang | MPC sub-team: Stage 1 task list and Card 1, MPC framework and first car run, Stage 1 design (baseline, MPC, RL, M1), lane-end and lost-lane stops, car models and control settings, Foxglove export and run timing | [peiyu-wang.md](peiyu-wang.md) |
| Yuhao Liu | MPC sub-team: measured-speed prediction, offline endpoint and cone-memory experiments, experimental junction routes, grouped marker commands and left-turn stopping/diagnosis | [yuhao-liu.md](yuhao-liu.md) |
| Chris (Chuhan Wang) | Car control sub-team: system ID (cars 8, 20), CarController (FF + PI speed, per-side steering map), 26 Oct speed fix, obstacle-avoidance validation and spacing study | [chris-wang.md](chris-wang.md) |
| Thanh Long Nguyen | | _to add_ |
| Zhenling Zhang | Obstacle handling: cone filter, lidar obstacle detection, obstacle stop and brake, avoidance planner, local map, obstacle simulation | [zhenling-zhang.md](zhenling-zhang.md) |
| Yifei Xia | | _to add_ |

The names are taken from the repository's commit authors. Add anyone who's missing and correct the spelling.

## Things to agree before we submit

These points come up in more than one person's work. The team should state each one the same way everywhere.

| Topic | What Toby's section says | Who else should check it |
| --- | --- | --- |
| Car 20 steering limits | Measured 0.48 1/m left and 0.92 1/m right at ±0.5 steering (Chris). With Chris's ±0.8 interval, the left limit is set to 0.91, but that number hasn't been measured on the car. | Chris, Yuhao |
| Why left turns failed | A 0.48 1/m left limit gives a 2.08 m arc, which doesn't fit a 0.8–1 m crossing. Simulation of `bf5e08c` passed 2 of 40 left turns and 35 of 40 right turns (10 cases, 10 seeds). | Yuhao (reported the car result), Chris |
| Marker IDs | `dev` uses grouped IDs: 1–4 turn left, 5–6 turn right, 7–8 straight, and any other ID stops the run. On `dev`, no ID maps to a keep-side command, so the lane-change demo needs the 23–27 Oct mapping. | Yuhao |
| Lane shifting on a car | On car 20 on 20 Oct, `lane_side: LEFT` kept the car to the left (judged by eye; no log). The 21 Oct shift on car 21 was hard to see, and the clearance was retuned afterwards. | Whoever was at the 20 and 21 Oct sessions |
| What ran on a real car | Detection and commands: car 13 logged "Marker 14: KEEP_RIGHT" on 27 Oct. No complete left turn at an intersection was shown before car access ended on 2 Nov. | Everyone who was at the sessions |
| Phantom obstacles | Car 13 log: 16 avoidance lines with no obstacle on the track. The fixes are on `fix/left-turn-steering-and-obstacles`, which has no PR yet. | Zhenling Zhang (obstacle owner) |
| Camera pan / yaw | The pan is held at 0, so the camera's yaw isn't measured. In simulation, ±4° of yaw costs one whole turn direction. | Lane detection owner, Thanh Long |
| Which videos we submit | They must show real cars. The team should list which features have video. | Everyone |
| RGB image and OpenCV 5 | Peiyu: with OpenCV 5 in the DREAM venv, publishing the RGB image crashes the cone detector (car 22, 12 Oct), so #24 turns it off. Toby's markers need the RGB image (`--aruco`). `dev` still has it on. | Toby, Yuhao |
| Which MPC weights | Stage 1 design (A4) chose 3/1/0.1/3 with a 3.0 1/m limit. `dev` uses the lab calibration 10/1/0.1/2 with 0.48 / 0.92 1/m (#15). Each result should say which one it used. | Toby |
| Control settings per car | `dev`'s speed control values suit car 22 only (the car 20 model reaches 0/12 cases). #18 proposes one setting for both cars; not yet reviewed. | Chris |
