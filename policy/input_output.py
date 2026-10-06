from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Iterable


@dataclass(frozen=True)
class Position:
    """A point in the car's body frame (base_link), in metres.

    These are CAR coordinates; they are NOT world positions. The origin is on the nominal ground
    directly below the vehicle centre of gravity (CG).
    """
    x: float
    """Distance forwards of the origin in metres (+x is forwards)."""
    y: float
    """Distance left of the origin in metres (+y is left)."""
    z: float
    """Height above the nominal ground plane in metres (+z is up).

    For a cone, this is the detected point's nominal height above the ground plane; it is NOT the
    cone's total height.
    """


class ConeColour(Enum):
    """The colour of a detected cone.

    The ROS message uses ConeDetection.COLOR_YELLOW / COLOR_BLUE (1/2). policy_node converts those
    ids into this enum, so these values should not be assumed to match the message ids.
    """
    YELLOW = 0
    """A yellow cone."""
    BLUE = 1
    """A blue cone."""


class SensorAge(float):
    """How old a sensor's last accepted sample is, in seconds.

    This acts as a float equal to the age in seconds, with the sample's ROS stamp attached as
    stamp_ns. The age is measured when the policy step starts, and is the larger of:
    - the time since policy_node received the sample, and
    - the time since the sample's own ROS stamp, for sensors whose messages have a header.

    If an optional observation has expired, its value is None, but its age is still available.
    A missing age (no accepted sample exists) is stored as 0.0, so 0.0 alone does not prove that
    a sample is fresh.
    """
    stamp_ns: int | None
    """ROS stamp of the sample in nanoseconds.

    None for wheel speed, which has no header, and when no accepted sample exists.
    """

    def __new__(cls, age_s: float | None, stamp_ns: int | None):
        """
        :param age_s: Age in seconds of the sensor's last accepted sample, or None if none exists.
        :param stamp_ns: ROS stamp of that sample in nanoseconds, or None if it has no stamp.
        """
        age = super().__new__(cls, age_s or 0.0)
        age.stamp_ns = stamp_ns
        return age


@dataclass(frozen=True)
class ConeDetection:
    """One cone from the cone detector.

    policy_node checks every cone in a batch before accepting it: the position must be finite,
    the colour must be yellow or blue and the confidence must be within [0, 1]. If any cone fails,
    the whole batch is ignored.
    """
    pos: Position
    """Position of the detected point in the car's body frame.

    The fixed camera pose does NOT compensate for terrain or vehicle pitch.
    """
    colour: ConeColour
    """Classified colour of the cone."""
    confidence: float
    """Classification confidence, from 0.0 to 1.0."""


class Detections[T](list[T]):
    """A list of detections from one sensor message, with that message's sensor age attached.

    It behaves as a normal list of detections.
    """
    __sensor_age: SensorAge

    def __init__(self, detections: Iterable[T], sensor_age: SensorAge):
        """
        :param detections: The detections from one sensor message.
        :param sensor_age: Age of that sensor message.
        """
        super().__init__(detections)
        self.__sensor_age = sensor_age

    @property
    def sensor_age(self) -> SensorAge:
        """
        :return: Age of the sensor message these detections came from.
        """
        return self.__sensor_age


class ConeBatch(Detections[ConeDetection]):
    """Cone detections from one accepted batch, with measured publication latency.

    A valid empty frame is an empty list with cone data still available. It is None on
    CarObservations only when cone detections are optional and no fresh batch exists.
    """
    __acquisition_to_publish_latency_s: float

    def __init__(
        self,
        detections: Iterable[ConeDetection],
        sensor_age: SensorAge,
        acquisition_to_publish_latency_s: float,
    ):
        """
        :param detections: The cones from one detection batch.
        :param sensor_age: Age of that batch, including transport and time spent waiting for this
            policy step. It uses the same end-of-exposure reference for OAK-D as the latency.
        :param acquisition_to_publish_latency_s: Fixed time from camera acquisition (end-of-exposure
            for OAK-D) to immediately before the detector called publish(). Exposure duration is
            excluded. Do not add this to sensor_age.
        """
        super().__init__(detections, sensor_age)
        self.__acquisition_to_publish_latency_s = acquisition_to_publish_latency_s

    @property
    def acquisition_to_publish_latency_s(self) -> float:
        """
        :return: Acquisition-to-publication latency in seconds for this batch.
        """
        return self.__acquisition_to_publish_latency_s


@dataclass(frozen=True)
class ImagePoint:
    """A pixel location in a camera image."""
    u: float
    """Horizontal pixel coordinate."""
    v: float
    """Vertical pixel coordinate."""


@dataclass(frozen=True)
class Quaternion:
    """A unit quaternion orientation in (x, y, z, w) order.

    This matches the ROS quaternion convention used by policy_node (identity is 0, 0, 0, 1).
    """
    x: float
    """X component of the quaternion."""
    y: float
    """Y component of the quaternion."""
    z: float
    """Z component of the quaternion."""
    w: float
    """W (scalar) component of the quaternion."""


@dataclass(frozen=True)
class FiducialPose:
    """A fiducial marker pose in policy_frame_id.

    policy_node only supplies this when the detector's pose solve succeeded. A valid pose is a
    successful geometric solve, not proof that the marker is real or safe to drive toward. Position
    and orientation are transformed into policy_frame_id at image acquisition time.
    """
    position: Position
    """Marker position in metres in policy_frame_id (normally base_link)."""
    orientation: Quaternion
    """Marker orientation in policy_frame_id."""


@dataclass(frozen=True)
class FiducialDetection:
    """One ArUco / fiducial marker from an accepted detection batch.

    Repeated IDs remain separate records in their received order.
    """
    id: int
    """Marker ID scoped by the batch dictionary name."""
    marker_size_m: float
    """Physical marker side length in metres."""
    corners: tuple[ImagePoint, ImagePoint, ImagePoint, ImagePoint]
    """Four canonical-order corners as source-image pixels.

    Corners remain source-image pixels after poses are transformed. The batch's source_frame_id is
    the camera optical frame these pixels belong to.
    
    TODO: The node documents the order only as canonical; it does not name the roles of each index. The actual order
          should be found out and this datatype can be turned into a better-structured dataclass instead (e.g. with top-right, etc) 
    """
    pose: FiducialPose | None
    """Marker pose in policy_frame_id, or None when the detector did not supply a usable pose."""
    reprojection_error_px: float
    """Reprojection error in pixels. NaN means unavailable."""


@dataclass(frozen=True)
class FiducialBatch:
    """One fiducial-detection batch, with dictionary/source-frame metadata.

    An accepted empty batch has detections == [] and is still fresh data. Marker absence has no
    automatic timeout or remembered pose.
    """
    detections: list[FiducialDetection]
    """Marker records in received order."""
    sensor_age: SensorAge
    """Age of the fiducial batch."""
    dictionary_name: str
    """ArUco dictionary name that scopes every marker ID in this batch."""
    source_frame_id: str
    """Camera optical frame for the corners."""


@dataclass(frozen=True)
class LidarDetection:
    """One ray from a lidar scan.

    The ray is in the lidar's own frame (LidarScan.frame_id). The scan is NOT rotated into
    base_link, and its origin is the lidar device, unchanged by the base_link ground-origin
    convention.
    """
    angle: float
    """Angle of the ray in radians in the lidar's frame: angle_min + i * angle_increment for ray i."""
    ld_range: float
    """Measured distance in metres.

    Infinity can mean no return, and NaN is not a zero-distance obstacle. Only finite ranges
    between the scan's range_min and range_max are real hits.
    """
    intensity: float | None
    """Return intensity reported by the lidar for this ray, in device-specific units.

    None when the scan has no intensities, which is valid for a lidar scan.
    """


@dataclass(frozen=True)
class LidarScan:
    """One lidar scan, with its rays in scan order and the scan's details.

    policy_node checks each scan before accepting it: it has at least one ray, a valid frame,
    finite details, a positive angle_increment, range_max greater than range_min, and either no
    intensities or one per ray. The scan stamp is the first ray's acquisition time.
    """
    detections: list[LidarDetection]
    """The rays of the scan in scan order; ray i is at angle_min + i * angle_increment."""
    sensor_age: SensorAge
    """Age of the lidar scan."""
    frame_id: str
    """The lidar's own frame, which the rays are measured in."""
    angle_min: float
    """Angle of the first ray in radians."""
    angle_max: float
    """Angle of the last ray in radians."""
    angle_increment: float
    """Angle between consecutive rays in radians."""
    time_increment: float
    """Time between consecutive rays in seconds."""
    scan_time: float
    """Time between scans in seconds."""
    range_min: float
    """Minimum valid range in metres. Shorter ranges are not real hits."""
    range_max: float
    """Maximum valid range in metres. Longer ranges are not real hits."""


@dataclass(frozen=True)
class LidarCartesianPoint:
    """One usable lidar return converted into policy_frame_id.

    Only finite returns within the scan's inclusive range limits are included. There are no
    placeholders. One mounting transform is used for the whole scan; points have no per-ray motion
    correction or IMU levelling.
    """
    pos: Position
    """Point in metres in policy_frame_id (normally body x forward, y left, z up)."""
    scan_index: int
    """Original ray index in the matching LidarScan, so the raw range is
    lidar_scan.detections[scan_index].ld_range.
    """


@dataclass(frozen=True)
class LidarCartesian:
    """Body-frame Cartesian lidar points prepared alongside the raw scan.

    Cartesian points always match the latest accepted raw scan. If transform/conversion fails, the
    raw scan remains available and the previous Cartesian result is discarded. A fresh empty point
    list is valid. This observation expires at either representation's timeout.
    """
    points: list[LidarCartesianPoint]
    """Usable points in policy_frame_id, with their original scan indices."""
    sensor_age: SensorAge
    """Age of the Cartesian conversion sample."""
    frame_id: str
    """Frame of the points; normally policy_frame_id (base_link)."""


@dataclass(frozen=True)
class CarOrientation:
    """The car's orientation from the IMU, in the car's body frame after the mounting TF is applied.

    This is only created when roll, pitch and relative heading are all available. Relative heading
    needs an orientation sample that was fresh when the policy entered the publishing-policy state.
    Without one, the orientation stays None for that whole run, even if later samples arrive.
    """
    roll: float
    """Rotation about the forwards (+x) axis in radians, relative to magnetic ENU (east/north/up)."""
    pitch: float
    """Rotation about the left (+y) axis in radians, relative to magnetic ENU (east/north/up)."""
    heading_angle: float
    """Yaw about the up (+z) axis in radians, relative to the heading on policy entry.

    It is re-zeroed ONLY when entering the publishing-policy state, and is wrapped to [-pi, pi].
    Absolute orientation uses magnetic ENU (east/north/up), and angles can drift.
    """
    sensor_age: SensorAge
    """Age of the IMU orientation sample."""


@dataclass(frozen=True)
class AngularVelocity:
    """The car's rate of rotation from the IMU gyroscope, in the car's body frame.

    Values are in rad/s after the mounting TF is applied. This is independent of the other IMU
    fields: a new gyro sample is not a new heading.
    """
    roll: float
    """Rotation rate about the forwards (+x) axis in rad/s."""
    pitch: float
    """Rotation rate about the left (+y) axis in rad/s."""
    yaw: float
    """Rotation rate about the up (+z) axis in rad/s."""
    sensor_age: SensorAge
    """Age of the IMU angular velocity sample."""


@dataclass(frozen=True)
class SpecificForce:
    """Specific force from the IMU accelerometer, in the car's body frame.

    Values are in m/s^2 after the mounting TF is applied. Specific force INCLUDES GRAVITY; it is
    not pure driving acceleration. When the car is still and level, up_acc is about +9.81 and the
    other components are about zero.
    """
    forwards_acc: float
    """Specific force along the forwards (+x) axis in m/s^2."""
    left_acc: float
    """Specific force along the left (+y) axis in m/s^2."""
    up_acc: float
    """Specific force along the up (+z) axis in m/s^2, including gravity."""
    sensor_age: SensorAge
    """Age of the IMU specific force sample."""


class WheelSpeed(float):
    """The wheel speed in m/s, acting as a float, with its sensor age attached.

    Wheel speed is UNSIGNED: it does not tell you forward versus reverse.
    - The vehicle's Traxxas node estimates wheel speed from sparse encoder periods, so low speeds
      take longer to measure.
    - The Traxxas node's encoder_timeout_seconds controls when it concludes that no encoder ticks
      means the wheel has stopped.
    - The policy_node sensor timeout instead detects missing wheel-speed telemetry.
    - A missing wheel speed (None) is not a measured zero.
    """
    sensor_age: SensorAge
    """Age of the wheel-speed sample.

    Its stamp_ns is always None because wheel-speed messages have no header.
    """

    def __new__(cls, wheel_speed: float, sensor_age: SensorAge):
        """
        :param wheel_speed: Unsigned wheel speed in m/s.
        :param sensor_age: Age of the wheel-speed sample.
        """
        speed = float.__new__(cls, wheel_speed)
        speed.sensor_age = sensor_age
        return speed


@dataclass(frozen=True)
class ObservedCarState:
    """The measured state of the car.

    Each field is None when that observation is optional and missing or expired. Partial IMU
    messages are normal, so each IMU field may independently be None.
    """
    orientation: CarOrientation | None
    """Roll, pitch and relative heading, or None."""
    ang_velocity: AngularVelocity | None
    """Body rotation rates, or None."""
    specific_force: SpecificForce | None
    """Body specific force including gravity, or None."""
    wheel_speed: WheelSpeed | None
    """Unsigned wheel speed in m/s, or None. None is not a measured zero."""


@dataclass(frozen=True)
class PolicyState:
    """Timing information about the current policy step and run."""
    dt: float
    """ACTUAL monotonic seconds since the previous policy step.

    It is 0.0 on the first step, so guard against dividing by it. In timer mode, a sensor sample
    can be reused across several steps, so a policy step is not necessarily a new measurement.
    """
    policy_elapsed_s: float
    """Seconds since entering the publishing-policy state."""
    is_first_policy_step: bool
    """True on the first step after entering the publishing-policy state.

    Use it to reset an integrator (or similar) once per run.
    """


@dataclass(frozen=True)
class CarObservations:
    """All observations available to the policy for one policy step.

    All lengths and angles use metres and radians.
    """
    cones: ConeBatch | None
    """Cones from the most recent fresh detection batch, or None.

    It is None when cone detections are optional for policy_node and no fresh batch is available.
    A valid empty frame gives an empty list rather than None, so always handle having no cones.
    """
    fiducials: FiducialBatch | None
    """Fiducial markers from the most recent fresh batch, or None.

    It is None when fiducials are optional for policy_node and no fresh batch is available.
    A valid empty batch gives detections == [] rather than None.
    """
    lidar_scan: LidarScan | None
    """The most recent fresh raw lidar scan, or None.

    It is None when lidar_scan is optional for policy_node and no fresh scan is available.
    """
    lidar_cartesian: LidarCartesian | None
    """Body-frame Cartesian points from the matching scan, or None.

    It is None when conversion is unavailable/stale, including when the raw scan is present but
    its transform failed. A fresh empty point list is valid and is not None.
    """
    car: ObservedCarState
    """The measured state of the car."""
    policy: PolicyState
    """Timing information about the current policy step and run."""


@dataclass
class CarActions:
    """The actions requested by the policy for one step.

    All actions are NORMALISED values in [-1, 1]. policy_node clips values outside [-1, 1], and
    NaN, infinity or programming errors stop the policy. Invalid data never becomes a motor command.
    """
    drive_action: float
    """Requested motor effort (ESC), NOT a speed in m/s."""
    steering_action: float
    """Position within the calibrated steering interval; zero is centre."""
    camera_pan_action: float | None
    """Independent camera pan servo target, NOT an angle in radians.

    Zero is centre. None sends no target, so the camera keeps its current position. The camera
    can move even while vehicle drive is disabled.
    """
    debug1: Any
    """Optional scalar published on the debug1 topic, or None to publish nothing.

    A value must be a finite number that fits in a Float32, otherwise the policy stops.
    """
    debug2: Any
    """Optional scalar published on the debug2 topic, or None to publish nothing.

    A value must be a finite number that fits in a Float32, otherwise the policy stops.
    """


def default_actions() -> CarActions:
    """Create actions that request no movement.

    Drive is zero, steering is centred, no camera pan target is sent (so the camera holds its
    position) and nothing is published on the debug topics.

    :return: The default actions.
    """
    return CarActions(0.0, 0.0, None, None, None)


def convert_observations(
    cone_data_available: bool,
    num_cones: int,
    x_coords: list[float],
    y_coords: list[float],
    z_coords: list[float],
    cone_colour: list[ConeColour],
    cone_confidence: list[float],
    cone_acquisition_to_publish_latency_s: float | None,
    fiducials_available: bool,
    fiducial_dictionary_name: str | None,
    fiducial_source_frame_id: str | None,
    fiducials: list[dict],
    wheel_speed_in_meters_per_second: float | None,
    lidar_scan: dict | None,
    lidar_cartesian: dict | None,
    roll_angle_in_radians: float | None,
    pitch_angle_in_radians: float | None,
    heading_angle_in_radians: float | None,
    angular_velocity_rad_per_sec: tuple[float, float, float] | None,
    specific_force_m_per_sec_squared: tuple[float, float, float] | None,
    sensor_age_s: dict[str, float | None],
    sensor_stamp_ns: dict[str, int | None],
    dt: float,
    policy_elapsed_s: float,
    is_first_policy_step: bool,
) -> CarObservations:
    """Convert the raw observations from policy_node into a CarObservations object.

    An observation that is None (optional and missing or expired) becomes None in the result.

    :param cone_data_available: True when a fresh cone-detection batch exists. A valid empty frame
        gives True with num_cones == 0.
    :param num_cones: Number of cones in the current detection batch. It is 0 when no fresh batch
        is available.
    :param x_coords: Forward cone positions in metres (base_link). Index i describes one cone in
        all the cone lists.
    :param y_coords: Leftward cone positions in metres (base_link).
    :param z_coords: Nominal heights of the detected points above the ground in metres. These are
        NOT the cones' total heights.
    :param cone_colour: Cone colours, already converted from the message ids to ConeColour.
    :param cone_confidence: Cone classification confidences, from 0.0 to 1.0.
    :param cone_acquisition_to_publish_latency_s: Acquisition-to-publication latency in seconds, or
        None when no fresh cone batch is available.
    :param fiducials_available: True when a fresh fiducial batch exists. A valid empty batch gives
        True with an empty fiducials list.
    :param fiducial_dictionary_name: ArUco dictionary name for the batch, or None when unavailable.
    :param fiducial_source_frame_id: Camera optical frame for corners, or None when unavailable.
    :param fiducials: Marker dictionaries from policy_node. Each has id, marker_size_m, corners,
        pose_valid, position_xyz, orientation_xyzw and reprojection_error_px. When pose_valid is
        false, position_xyz and orientation_xyzw are None and become pose=None on FiducialDetection.
    :param wheel_speed_in_meters_per_second: Unsigned wheel speed in m/s, or None.
    :param lidar_scan: Full raw lidar scan dict, or None when unavailable. It contains ranges,
        intensities (empty or one per range), frame_id, angle_min/max/increment, time_increment,
        scan_time and range_min/max.
    :param lidar_cartesian: Cartesian conversion dict, or None when unavailable. It contains
        points_xyz, scan_indices and frame_id.
    :param roll_angle_in_radians: Roll about +x, or None.
    :param pitch_angle_in_radians: Pitch about +y, or None.
    :param heading_angle_in_radians: Heading relative to policy entry, wrapped to [-pi, pi], or
        None.
    :param angular_velocity_rad_per_sec: Body rotation rates about (x, y, z) in rad/s, or None.
    :param specific_force_m_per_sec_squared: Body specific force along (x, y, z) in m/s^2
        including gravity, or None.
    :param sensor_age_s: Age in seconds of each sensor's last accepted sample, keyed by sensor
        name, or None if none exists.
    :param sensor_stamp_ns: ROS stamp in nanoseconds of each sensor's last accepted sample, keyed
        by sensor name. None for wheel speed, which has no header, or if no sample exists.
    :param dt: Actual monotonic seconds since the previous policy step; 0.0 on the first step.
    :param policy_elapsed_s: Seconds since entering the publishing-policy state.
    :param is_first_policy_step: True on the first step after entering the publishing-policy state.
    :return: The observations for this policy step.
    """
    def get_sensor_age(sensor: str) -> SensorAge:
        """
        :param sensor: Name of the sensor, as used in sensor_age_s and sensor_stamp_ns.
        :return: The age and ROS stamp of that sensor's last accepted sample.
        """
        return SensorAge(sensor_age_s[sensor], sensor_stamp_ns[sensor])

    # Set up cone detections
    cones = None
    if cone_data_available:
        assert cone_acquisition_to_publish_latency_s is not None
        cones = ConeBatch(
            [
                ConeDetection(
                    pos=Position(x_coords[i], y_coords[i], z_coords[i]),
                    colour=cone_colour[i],
                    confidence=cone_confidence[i],
                )
                for i in range(num_cones)
            ],
            sensor_age=get_sensor_age("cone_detections"),
            acquisition_to_publish_latency_s=cone_acquisition_to_publish_latency_s,
        )

    # Set up fiducial detections
    fiducial_batch = None
    if fiducials_available:
        assert fiducial_dictionary_name is not None
        assert fiducial_source_frame_id is not None
        converted_fiducials = []
        for marker in fiducials:
            corner_uvs = marker["corners"]
            corners = (
                ImagePoint(corner_uvs[0][0], corner_uvs[0][1]),
                ImagePoint(corner_uvs[1][0], corner_uvs[1][1]),
                ImagePoint(corner_uvs[2][0], corner_uvs[2][1]),
                ImagePoint(corner_uvs[3][0], corner_uvs[3][1]),
            )
            pose = None
            if marker["pose_valid"]:
                orientation = marker["orientation_xyzw"]
                pose = FiducialPose(
                    position=Position(*marker["position_xyz"]),
                    orientation=Quaternion(
                        orientation[0], orientation[1], orientation[2], orientation[3],
                    ),
                )
            converted_fiducials.append(FiducialDetection(
                id=marker["id"],
                marker_size_m=marker["marker_size_m"],
                corners=corners,
                pose=pose,
                reprojection_error_px=marker["reprojection_error_px"],
            ))
        fiducial_batch = FiducialBatch(
            detections=converted_fiducials,
            sensor_age=get_sensor_age("fiducial_detections"),
            dictionary_name=fiducial_dictionary_name,
            source_frame_id=fiducial_source_frame_id,
        )

    # Set up raw lidar scan
    lidar_scan_obs = None
    if lidar_scan is not None:
        intensities = lidar_scan["intensities"] or [None] * len(lidar_scan["ranges"])
        lidar_scan_obs = LidarScan(
            detections=[
                LidarDetection(lidar_scan["angle_min"] + i * lidar_scan["angle_increment"], ld_range, intensity)
                for i, (ld_range, intensity) in enumerate(zip(lidar_scan["ranges"], intensities))
            ],
            sensor_age=get_sensor_age("lidar_scan"),
            frame_id=lidar_scan["frame_id"],
            angle_min=lidar_scan["angle_min"],
            angle_max=lidar_scan["angle_max"],
            angle_increment=lidar_scan["angle_increment"],
            time_increment=lidar_scan["time_increment"],
            scan_time=lidar_scan["scan_time"],
            range_min=lidar_scan["range_min"],
            range_max=lidar_scan["range_max"],
        )

    # Set up Cartesian lidar points
    lidar_cartesian_obs = None
    if lidar_cartesian is not None:
        lidar_cartesian_obs = LidarCartesian(
            points=[
                LidarCartesianPoint(pos=Position(*point), scan_index=index)
                for point, index in zip(lidar_cartesian["points_xyz"], lidar_cartesian["scan_indices"])
            ],
            sensor_age=get_sensor_age("lidar_cartesian"),
            frame_id=lidar_cartesian["frame_id"],
        )

    # Detect car orientation
    orientation = None
    if roll_angle_in_radians is not None and pitch_angle_in_radians is not None and heading_angle_in_radians is not None:
        orientation = CarOrientation(
            roll=roll_angle_in_radians,
            pitch=pitch_angle_in_radians,
            heading_angle=heading_angle_in_radians,
            sensor_age=get_sensor_age("imu_orientation")
        )
    # Detect car angular velocity
    angular_velocity = None
    if angular_velocity_rad_per_sec is not None:
        angular_velocity = AngularVelocity(
            roll=angular_velocity_rad_per_sec[0],
            pitch=angular_velocity_rad_per_sec[1],
            yaw=angular_velocity_rad_per_sec[2],
            sensor_age=get_sensor_age("imu_angular_velocity")
        )
    # Detect car specific force
    specific_force = None
    if specific_force_m_per_sec_squared is not None:
        specific_force = SpecificForce(
            forwards_acc=specific_force_m_per_sec_squared[0],
            left_acc=specific_force_m_per_sec_squared[1],
            up_acc=specific_force_m_per_sec_squared[2],
            sensor_age=get_sensor_age("imu_specific_force")
        )
    wheel_speed = None
    if wheel_speed_in_meters_per_second is not None:
        wheel_speed = WheelSpeed(wheel_speed_in_meters_per_second, get_sensor_age("wheel_speed"))
    # Return a filled car observations object
    return CarObservations(
        cones=cones,
        fiducials=fiducial_batch,
        lidar_scan=lidar_scan_obs,
        lidar_cartesian=lidar_cartesian_obs,
        car=ObservedCarState(
            orientation=orientation,
            ang_velocity=angular_velocity,
            specific_force=specific_force,
            wheel_speed=wheel_speed,
        ),
        policy=PolicyState(
            dt=dt,
            policy_elapsed_s=policy_elapsed_s,
            is_first_policy_step=is_first_policy_step,
        ),
    )


def convert_actions(
    output: CarActions,
) -> tuple[float, float, float | None, float | None, float | None]:
    """Convert the policy's actions into the tuple returned to policy_node.

    A warning is printed for each action that is not strictly inside (-1, 1), but the value is
    not changed. policy_node clips drive, steering and camera pan to [-1, 1] afterwards.

    :param output: The actions chosen by the policy.
    :return: The drive, steering, camera pan, debug1 and debug2 actions, in that order.
    """
    if not -1 < output.drive_action < 1:
        print(f"Drive action `{output.drive_action}` is out of range")
    if not -1 < output.steering_action < 1:
        print(f"Steering action `{output.steering_action}` is out of range")
    if output.camera_pan_action is not None and not -1 < output.camera_pan_action < 1:
        print(f"Camera pan action `{output.camera_pan_action}` is out of range")
    return (
        output.drive_action,
        output.steering_action,
        output.camera_pan_action,
        output.debug1,
        output.debug2,
    )
