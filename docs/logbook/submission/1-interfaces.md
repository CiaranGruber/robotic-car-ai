## Part 1: Interfaces

### 1.1 Ciaran Gruber — data model, logging and replay

**Built:** the observation/action data model between DREAM and the policy, data logging, scenarios from logs and
their renderer, `CarMemory` (state carried between steps: lane side, marker command, stops, local map), the repo, CI
and the car upload script. **PRs** #3 #4 #5 #7 #13 #20 #22.

**S1.** "Dream Input" and "Dream Output" in 0.3. Design goal: every step's observations and actions are
serialisable, so any car run can be logged, turned into a scenario and replayed off the car.

**S2.** Each step: DREAM observations → serialisable dataclass → (log) → policy → actions → (log) → car actions.

**S3: changing the architecture without changing the car's output.** Replay recorded scenarios step by step through
the modified code and compare every action with the baseline; accept only if all match within $10^{-6}$. Used for
the `CarMemory` refactor without a physical track.

**S4: why did the car veer into cones on curves? (26 Oct)** Unit tests and simulation passed, but on the curvy track
the car hit cones. Logged the run, converted it to a scenario (`fail_curvy_road`), replayed it with that day's policy
(`d1d8096`) and config, and overlaid the lane the policy produced. Obstacle removed to isolate curvature.

![fail_curvy_road frame 60](ciaran/figures/fail_curvy_road_frame60_centreline_oct26.png)

*Frame 60: far blue cones conflict with the near yellow row and the detector reports `NO_LANE`.*

→ Lane keeping now follows the yellow row when the blue row is missing (2.2); gentler curvature also helped.
