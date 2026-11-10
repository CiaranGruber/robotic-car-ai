## Part 1: Interfaces

### 1.1 Ciaran Gruber — data model, logging and replay

**Built:** the observation/action data model between DREAM and the policy, data logging, scenarios from logs and
their renderer, `CarMemory` (state carried between steps: lane side, marker command, stops, local map), the repo, CI
and the car upload script. **PRs** #3 #4 #5 #7 #13 #20 #22.

**Standard 1**: Explanation to be done in interactive discussion with block diagram in [0.3 The system and who built it](#03-the-system-and-who-built-it).

**Standard 2.**: Most evidence for this is observed through the design of the overall policy architecture.

<img src="ciaran/figures/block%20diagram%20flowchart.png" alt="Block diagram flowchart" style="width: 75%; max-width: 75%;">

_Figure: The algorithm used for managing logging in the car_

![Modified concurrent camera design](ciaran/figures/Modified%20concurrent%20camera%20design.png)

_Figure: Modification of program architecture designed for concurrent execution of policy nodes._

**Standard 3. Using single-step simulation to ensure consistent behaviour during architectural redesigns**



<img src="ciaran/figures/Simulation%20diagram%20flowchart.png" alt="Simulation diagram flowchart" style="width: 75%; max-width: 60%;">

_Figure: The process followed when making significant architectural changes involving single-step simulation to ensure every action matches within $10^{-6}$._

**Standard 4: Resolving issues with car movement**

**Engineering question**: Why does the car veer into the cones on curved tracks? (See figure below)

Unit tests and simulation passed, but on the curvy track the car hit cones. Logged the run, converted it to a scenario (`fail_curvy_road`), replayed it with that day's policy (`d1d8096`) and config, and overlaid the lane the policy produced. Obstacle seen in the below figure was removed to isolate curvature.

<table style="width:100%; border-collapse:collapse; table-layout:fixed; margin:6pt 0;">
  <tr>
    <td style="width:33.33%; border:none; padding:0 4pt; text-align:center; vertical-align:bottom;">
      <img src="ciaran/figures/fail_curvy_road.jpg"
           alt="Track photo for fail_curvy_road"
           style="width:100%; margin:0 auto;">
      <p style="margin:4pt 0 0; font-size:9pt; color:#444c56;">
        Track (obstacle later removed)
      </p>
    </td>
    <td style="width:33.33%; border:none; padding:0 4pt; text-align:center; vertical-align:bottom;">
      <img src="ciaran/figures/fail_curvy_road_frame1_centreline_oct26.png"
           alt="fail_curvy_road frame 1"
           style="width:100%; margin:0 auto;">
      <p style="margin:4pt 0 0; font-size:9pt; color:#444c56;">
        Frame 1
      </p>
    </td>
    <td style="width:33.33%; border:none; padding:0 4pt; text-align:center; vertical-align:bottom;">
      <img src="ciaran/figures/fail_curvy_road_frame60_centreline_oct26.png"
           alt="fail_curvy_road frame 60"
           style="width:100%; margin:0 auto;">
      <p style="margin:4pt 0 0; font-size:9pt; color:#444c56;">
        Frame 60: <code>NO_LANE</code>
      </p>
    </td>
  </tr>
</table>

*Far blue cones conflict with the near yellow row at frame 60 and the detector reports `NO_LANE`.*

→ Lane keeping now follows the yellow row when the blue row is missing (2.2); gentler curvature also helped.
