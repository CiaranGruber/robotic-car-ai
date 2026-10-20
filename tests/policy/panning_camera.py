"""Opt-in planar camera experiment; angles, pivot and servo response are unmeasured assumptions.

The camera rotates about the body origin, as in the existing detector model. This omits real mounting
translation, shaft/optical kinematics, pitch, image blur and visibility occlusion. Nothing commands hardware.
"""
from dataclasses import dataclass, replace
import math

from policy.input_output import Position
from policy.lane_detection.corridor import pursuit_curvature
from tests.policy.curved_lanes import detections


@dataclass(frozen=True)
class PanFrame:
    """A simulated image's detections and camera angle at acquisition."""
    cones: tuple
    pan_deg: float
    time_s: float


class PanningCamera:
    """Look into requested turns, with bounded slew/lag and acquisition-aligned coordinate correction."""

    def __init__(self, max_pan_deg=25., preview_m=.6, slew_deg_per_s=60., lag_s=.15,
                 compensate=True, path_lookahead_m=None):
        if (not math.isfinite(max_pan_deg) or not 0 <= max_pan_deg <= 60
                or not all(math.isfinite(v) and v > 0 for v in (preview_m, slew_deg_per_s, lag_s))):
            raise ValueError("Invalid experimental pan limits")
        if path_lookahead_m is not None and (not math.isfinite(path_lookahead_m) or path_lookahead_m <= 0):
            raise ValueError("Invalid camera path preview")
        self.max_pan_deg = max_pan_deg
        self.preview_m = preview_m
        self.slew_deg_per_s = slew_deg_per_s
        self.lag_s = lag_s
        self.compensate = compensate
        self.path_lookahead_m = path_lookahead_m
        self.path_curvature = None
        self.angle_deg = 0.
        self.target_deg = 0.
        self.last_curvature = 0.
        self.samples = []

    def capture(self, cones, pose, car, rng, time_s):
        camera_car = replace(car, camera_yaw_deg=car.camera_yaw_deg + self.angle_deg)
        raw = detections(cones, pose, camera_car, rng)
        frame = PanFrame(tuple(raw), self.angle_deg, time_s)
        self.samples.append({"time_s": time_s, "pan_deg": self.angle_deg,
                             "requested_curvature": self.last_curvature,
                             "colours": tuple(c.colour.name for c in raw)})
        return frame

    def occlude(self, frame, colour):
        kept = tuple(c for c in frame.cones if c.colour != colour)
        self.samples[-1]["colours"] = tuple(c.colour.name for c in kept)
        return replace(frame, cones=kept)

    def to_body(self, frame):
        # Use the exposure's pan, never the current servo angle or its commanded target. In this simplified
        # model R(pan) undoes R(-pan). Any legacy static camera_yaw_deg remains unchanged for fair comparison.
        angle = math.radians(frame.pan_deg if self.compensate else 0.)
        cosine, sine = math.cos(angle), math.sin(angle)
        return [replace(c, pos=Position(cosine*c.pos.x-sine*c.pos.y,
                                       sine*c.pos.x+cosine*c.pos.y, c.pos.z)) for c in frame.cones]

    def aim_path(self, path):
        # Preview the policy's cone-derived route, without access to true simulator geometry or pose.
        self.path_curvature = (pursuit_curvature(path, self.path_lookahead_m)
                               if self.path_lookahead_m is not None else None)

    def advance(self, command, dt):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Invalid camera integration interval")
        self.last_curvature = command.curvature_per_m
        if command.speed_m_per_s > 0:
            requested = self.path_curvature if self.path_curvature is not None else command.curvature_per_m
            curvature = requested if abs(requested) > .15 else 0.
            target = math.degrees(curvature * self.preview_m)
            self.target_deg = max(-self.max_pan_deg, min(self.max_pan_deg, target))
        # A stop sends no new target. The servo can finish moving toward its previous target;
        # it does not recenter or instantaneously freeze its shaft.
        change = (self.target_deg-self.angle_deg) * (1-math.exp(-dt/self.lag_s))
        limit = self.slew_deg_per_s*dt
        self.angle_deg += max(-limit, min(limit, change))

    def metrics(self):
        turning = [s for s in self.samples if abs(s["requested_curvature"]) > .3]
        return {"max_abs_pan_deg": max((abs(s["pan_deg"]) for s in self.samples), default=0.),
                "turn_camera_frames": len(turning),
                "turn_both_colours_fraction": sum(len(set(s["colours"])) == 2 for s in turning)/max(1, len(turning)),
                "turn_mean_cones": sum(len(s["colours"]) for s in turning)/max(1, len(turning))}
