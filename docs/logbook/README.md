# AI4R System Project: team logbook (draft)

The logbook supports the interactive oral, so it uses diagrams, plots, tables and short bullets rather than report
prose. It's shown on the projector, and each person moves through their own section while answering questions.

**Status:** this is a draft started on 4 Nov 2026 by Toby. Only Toby's section is written. The overview and the other
sections are headings for each owner to fill in. Before submission, the team should check the
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
| Peiyu Wang | | _to add_ |
| Yuhao Liu | | _to add_ |
| Chris | | _to add_ |
| Linna Zhang | | _to add_ |
| Thanh Long Nguyen | | _to add_ |
| Zhenling | | _to add_ |
| Yifei Xia | | _to add_ |

The names are taken from the repository's commit authors. Add anyone who's missing and correct the spelling.

## Things to agree before we submit

These points come up in more than one person's work. The team should state each one the same way everywhere.

| Topic | What Toby's section says | Who else should check it |
| --- | --- | --- |
| Car 20 steering limits | Measured 0.48 1/m left and 0.92 1/m right at ±0.5 steering (Chris). With Chris's ±0.8 interval, the left limit is set to 0.91, but that number hasn't been measured on the car. | Chris, Yuhao |
| Why left turns failed | A 0.48 1/m left limit gives a 2.08 m arc, which doesn't fit a 0.8–1 m crossing. Simulation of `bf5e08c` gave 0% of left turns and 100% of right turns (8 runs each). | Yuhao (reported the car result), Chris |
| Marker IDs | `dev` uses grouped IDs: 1–4 turn left, 5–6 turn right, 7–8 straight, and any other ID stops the run. On `dev`, no ID maps to a keep-side command, so the lane-change demo needs the 23–27 Oct mapping. | Yuhao |
| Lane shifting on a car | On car 20 on 20 Oct, `lane_side: LEFT` kept the car to the left (judged by eye; no log). The 21 Oct shift on car 21 was hard to see, and the clearance was retuned afterwards. | Whoever was at the 20 and 21 Oct sessions |
| What ran on a real car | Detection and commands: car 13 logged "Marker 14: KEEP_RIGHT" on 27 Oct. No complete left turn at an intersection was shown before car access ended on 2 Nov. | Everyone who was at the sessions |
| Phantom obstacles | Car 13 log: 16 avoidance lines with no obstacle on the track. The fixes are on `fix/left-turn-steering-and-obstacles`, which has no PR yet. | Obstacle owners (Linna?) |
| Camera pan / yaw | The pan is held at 0, so the camera's yaw isn't measured. In simulation, ±4° of yaw costs one whole turn direction. | Lane detection owner, Thanh Long |
| Which videos we submit | They must show real cars. The team should list which features have video. | Everyone |
