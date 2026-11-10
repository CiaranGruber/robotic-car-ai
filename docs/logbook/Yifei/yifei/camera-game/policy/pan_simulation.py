"""Hardware-free plant model around the production pan controller.

All angles are degrees at the UI boundary. North is zero, left/CCW positive.
Servo time constant is an assumption, not a measurement of the real car.
"""
import math
from types import SimpleNamespace as NS

from policy import pan_stabilisation
from policy.memory import CarMemory


def wrap(angle):
    return (angle + 180) % 360 - 180


class PanSimulation:
    """Virtual car, sensor delay and first-order servo; no ROS or hardware IO."""

    def __init__(self):
        self.mode = 'project'
        self.gain = -0.43
        self.centre = 0.32
        self.limit = 0.15
        self.rate = 0.1
        self.lag = 0.15
        self.delay = 0.0
        self.body = 0.0
        self.target = 0.0
        self.sensor_on = True
        self.reset()

    def reset(self):
        self.time = 0.0
        self.pan = 0.0
        self.command = self.centre
        self.memory = CarMemory()
        self.first = True
        self.latched = False
        self.status = '运行中'
        self.samples = []
        self.records = []
        self.last_received = None
        self.last_heading = None

    def step(self, dt=0.02):
        if not math.isfinite(dt) or not 0 < dt <= .2:
            raise ValueError('simulation dt must be in (0, 0.2]')
        self.time += dt
        if self.sensor_on:
            self.samples.append((self.time, self.body))
        while self.samples and self.samples[0][0] <= self.time - self.delay + 1e-9:
            self.last_received, self.last_heading = self.samples.pop(0)
        age = math.inf if self.last_received is None else self.time - self.last_received
        if self.mode == 'ideal':
            self.pan = wrap(self.target-self.body)
            self.command = self.centre + self.gain * math.radians(self.pan)
            self.status = '理想补偿（无限转幅、瞬时响应）'
        elif self.mode == 'off':
            self.pan = 0.0
            self.command = self.centre
            self.status = '无补偿：镜头固定在车头'
        else:
            if not self.latched:
                if age >= .5:
                    if self.first and self.last_received is None:
                        self.status = '等待 IMU'
                    else:
                        self.latched = True
                        self.status = '已锁停：IMU 数据超过 0.5 秒'
                else:
                    # Target selection is a simulation adapter: the existing
                    # controller sees body heading relative to the chosen target.
                    obs = NS(car=NS(orientation=NS(heading_angle=math.radians(
                        wrap(self.last_heading-self.target)))),
                        policy=NS(dt=0.0 if self.first else dt,
                                  is_first_policy_step=self.first))
                    try:
                        action = pan_stabilisation.step(obs, self.memory,
                            self.gain, self.centre, self.limit, self.rate)
                        assert action.drive_action == action.steering_action == 0
                        self.command = action.camera_pan_action
                        self.first = False
                        self.status = '运行中'
                    except ValueError:
                        self.latched = True
                        self.status = '已锁停：超过补偿范围或输入无效'
            desired_pan = math.degrees((self.command-self.centre)/self.gain)
            alpha = 1.0 if self.lag == 0 else 1-math.exp(-dt/self.lag)
            self.pan += alpha*(desired_pan-self.pan)
        camera = wrap(self.body+self.pan)
        row = dict(time_s=self.time, mode=self.mode, body_deg=self.body,
                   target_deg=self.target, camera_deg=camera,
                   error_deg=wrap(camera-self.target), pan_deg=self.pan,
                   command=self.command, imu_age_s=age,
                   gain=self.gain, centre=self.centre, limit=self.limit,
                   rate=self.rate, servo_tau_s=self.lag, imu_delay_s=self.delay,
                   sensor_on=self.sensor_on, status=self.status)
        self.records.append(row)
        if len(self.records) > 30000:
            self.records.pop(0)
        return row
