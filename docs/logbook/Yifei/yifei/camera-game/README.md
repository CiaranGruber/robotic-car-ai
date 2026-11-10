# Camera Heading Laboratory (Local Simulation)

## Launch

From this `camera-game` folder, double-click `scripts/start_pan_game.bat`, or open PowerShell in this folder and run:

```powershell
python scripts/pan_game.py
```

Python, Tkinter and the project's existing Python dependencies are required. These were available on the development computer; other computers may need environment setup. SSH, ROS and a physical vehicle are not required. Do not upload this tool to the vehicle.

The control names below are English descriptions of the existing interface. This README translation does not change the application's interface language.

## Getting started

1. The application starts in **Ideal mode**. Adjust **Camera target heading**, or click the circular map, to select the camera's desired direction in the world frame.
2. Adjust **Vehicle heading**, or use the left and right arrow keys. The dashed gold arrow shows the target, the pink arrow shows the camera heading, and the cyan arrow shows the vehicle heading.
3. In ideal mode, the camera maintains the selected direction as the vehicle rotates. Zero degrees points towards the top of the display, and leftward rotation is positive. Both heading controls are relative to the world, not the vehicle.
4. Select **Project mode** to observe the existing controller's compensation-range and command-rate limits together with the simulated actuator response. The default compensation range is approximately +/-20 degrees between the vehicle and target headings. Exceeding this range latches a fault and holds the last command. Return within range and select **Reset / Unlock** to resume.
5. Reset preserves the slider settings, clears the records and returns the simulated servo to the vehicle's forward-facing position. It does not guarantee that the selected target is within the permitted range.
6. Select **No-compensation baseline** to observe a camera fixed to the vehicle and rotating with it.

Changing mode or resetting clears the current records. Export any required CSV data first. Pause the simulation to inspect or export results. Do not adjust the heading sliders during a fixed-trajectory comparison.

## Connection to the project and modelling assumptions

- `policy/pan_simulation.py` directly calls `step` in `policy/pan_stabilisation.py` and uses the existing `CarMemory`. It does not substitute a separate compensation algorithm.
- An adapter converts vehicle heading minus selected target heading into the relative IMU heading expected by the controller. Target selection is a simulation feature; it does not imply that the vehicle's ROS node already supports absolute heading targets.
- The model uses the previously reported calibration: centre command **0.32**, with a signed gain of **-0.43 command units per radian** of leftward camera rotation. The default compensation command range is **+/-0.15** around the centre, with a command-rate limit of **0.1 units/s**.
- Camera world heading equals vehicle heading plus camera angle relative to the vehicle. Tracking error is the shortest signed angular difference between camera world heading and target heading.
- Project mode converts the command back into a relative camera angle using the calibration and simulates a first-order servo response. The default time constant of **0.15 s is assumed, not measured on the vehicle**. IMU delay is also a user-selected simulation condition.
- Once IMU data has been received, a data age of **0.5 s or more** latches a fault. Restoring data does not automatically unlock the controller. Before the first measurement arrives, the model waits for data. This models data age and fault latching, not the complete ROS state machine, coordinate transforms or hardware drivers.
- Ideal mode has no travel, rate or delay constraints. Its calculated command may exceed hardware limits and must not be used as a recommendation for real-vehicle settings.
- Each simulation step represents **0.02 s**. Display updates depend on computer performance, so simulation time may differ from elapsed wall-clock time.
- All resets, motion and faults occur only in the model. No vehicle or camera commands are transmitted.

## Repeatable experiments for the logbook

Example research question: **For the same vehicle rotation, how much does compensation reduce camera heading error, and how do command-rate limits, actuator response and IMU delay affect the result?**

The **Fixed trajectory: 20 seconds** control sets the target to zero degrees and applies the following vehicle heading, with time `t` in seconds:

```text
vehicle_heading(t) = 15 * sin(2*pi*t/10) degrees
```

The simulation pauses automatically after 20 simulation seconds. Use this same input for each comparison:

| Experiment | Mode | Command-rate limit | Servo time constant | IMU delay |
|---|---|---|---|---|
| A | No-compensation baseline | Not applied | Not applied | Not applied |
| B | Ideal compensation | Not applied | Not applied | Not applied |
| C | Project controller | 0.1 units/s | 0.15 s | 0 s |
| D | Project controller | 0.1 units/s | 0.15 s | 0.2 s |

Export a separate CSV after each run. Compare maximum absolute error, mean absolute error and error-versus-time curves. The CSV contains simulation time, vehicle/target/camera headings, heading error, relative pan angle, command, IMU data age, mode, model parameters and status. Up to the most recent 10 minutes are retained; displayed statistics also refer to those retained records.

For additional boundary tests, keep the target at zero and manually rotate the vehicle to 30 degrees with the default compensation range to trigger a range fault. Disable **IMU data available** after normal operation begins to investigate stale-data fault latching. Record the triggering conditions and command-hold behaviour. Analyse fault cases separately from normal tracking results.

Distinguish ideal references, simulation predictions and physical observations in the report. A successful photo-detection example or an animation alone is not a complete simulation investigation. Support the discussion with a clear question, controlled comparisons, result plots and explicit assumptions and limitations.

## Implementation and verification

The implementation follows the applicable repository guidance in rules 0.00 and 0.01, README, CONTRIBUTING and acceptance documentation. Controller state uses the existing memory implementation; the compensation algorithm is reused. In the original development repository, the desktop scripts and simulation model are excluded from the vehicle-upload workflow.

The following command runs the relevant tests **from the original project root**, where the test files are available. The copied `camera-game` folder does not contain that test suite.

```powershell
python -m pytest tests/policy/test_pan_simulation.py tests/policy/test_pan_stabilisation.py -q
```

Hardware-free tests do not establish physical compensation accuracy, coordinate-frame accuracy or vehicle stopping performance.
