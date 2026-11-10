## Part 8: Yifei Xia — car control testing, marker recognition and camera pan

**Role.** Car control sub-team with Chris: speed and steering tests on car 20 (results and controller in 7.2–7.4).
Own work: ArUco ID testing and diagnosis, camera pan calibration and a compensation simulation, and a battery-monitoring
feasibility study. **PR** #26: `scripts/aruco_detection_test.py` (read-only ID subscriber) and
`scripts/camera_pan_test.py` (manual pan test, dry-run unless `--live`), with notes and tests.

### 8.1 Where it fits (S1)

| Subsystem | Input → output | Constraint |
| --- | --- | --- |
| Speed and steering tests (with Chris) | drive / steer commands → measured speed and curvature | A fixed drive is not a fixed speed; steering angle ≠ curvature |
| ArUco detection test | `/car/fiducial_detections` (or a stored image) → marker IDs | Detector needs an image source and `DICT_4X4_50` |
| Camera compensation | vehicle heading, target direction → pan command | Calibration is per car; IMU freshness, command range and rate limits |
| Battery monitoring | motor battery → charge estimate | No hardware modification allowed |

- My marker output was the recognised ID; mapping IDs to manoeuvres was 5.2.
- Physically panning the camera breaks the fixed zero-pan camera TF, so a moving camera needs a measured dynamic
  transform (why the team held pan 0, 2.2).

### 8.2 How it works (S2)

**Marker IDs.** The printed markers are ArUco (`DICT_4X4_50`), not QR codes. On the car, the test subscribes to the
existing detector's results; offline, OpenCV's ArUco detector returns IDs and saves outlines. An ID is an input to
decision logic, not a command.

**Heading compensation.** With vehicle heading $\psi_v$, target camera heading $\psi_t$ (left positive), wrap $w$ to the
shortest signed angle:

$$\theta_d = w(\psi_t - \psi_v), \qquad u_d = c + g\,\theta_d$$

| Parameter | Value | Basis |
| --- | --- | --- |
| Centre command $c$ | 0.32 | measured forward alignment (8.4) |
| Gain $g$ | −0.43 per rad | from the left / forward / right observations |
| Range about centre | ±0.15 (≈ ±20°) | exceeding it latches a fault rather than clipping |
| Rate limit | 0.1 per s | restricts command changes |
| Servo time constant (sim) | 0.15 s | assumed, not measured |
| IMU stale threshold (sim) | 0.5 s | stale input → latched hold |

### 8.3 Simulation and offline tests (S3)

**How does the camera hold a world direction?** Interactive camera lab: ideal, none, and the project controller with a
first-order servo and delayed heading; fixed trajectory $\psi_v(t) = 15\sin(2\pi t/10)$° for 20 s, CSV export. 11
controller checks passed; GUI checks covered mode changes, range faults and reset. **No comparative error results yet**
— planned: compare the three modes, then vary IMU delay 0–0.2 s. Can't show physical stabilisation (assumed servo, no
ROS/TF pipeline).

**Can stored images be decoded without car access?**

![Offline ArUco ID outputs](Yifei/figure/aruco-photo-detection-results.png)

*Four stored-image inputs returned IDs 0, 2, 5 and 8, one marker each. Examples, not an accuracy benchmark.*

### 8.4 On the car (S4)

**Live ArUco detection (19 Oct).** Read-only subscriber, markers shown to the camera. **Failure found:** the detector
was running but its image topic had zero publishers — an upstream image problem, not decoding. Once the image
pipeline ran: `ids=[2]` and `TARGET_SEEN_POSE_OK`, other IDs too. (Same root cause as 5.4's `--aruco`.) Lesson:
switching camera services affects every component that shares the camera.

**Camera pan calibration (30 Oct – 2 Nov).** Commanded the pan and compared its direction with the car's forward, left
and right.

![Camera pan calibration](Yifei/figure/camera-pan-calibration.png)

| Direction | Angle | Pan command |
| --- | --- | --- |
| Left | ≈ +90° | −0.35 |
| Forward | ≈ 0° | 0.32 |
| Right | ≈ −90° | 1.00 |

- The forward command is 0.32, not 0: a normalised command is not an angle. The camera centred but didn't visibly
  compensate when the body turned (sensor freshness, controller state, limits investigated). No repeat measurements or
  servo dynamics; car access ended before more testing.

**Battery monitoring (30 Oct).** Motor: 8.4 V 5000 mAh NiMH; computer: a separate 5 V power bank, so computer-supply data
says nothing about the motor battery. No accessible voltage/current source, and the battery can't go straight to a
GPIO pin; a sensing module and ADC would be needed, which the no-modification rule forbids. → not implemented; wheel-
speed feedback (7.2) handles changing response without a charge estimate.

**Lessons:** calibrate before interpreting; diagnose the whole data path (a running detector without an image does
nothing); keep evidence levels separate (four decoded images aren't an accuracy result).
