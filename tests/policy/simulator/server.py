"""The live 3D viewer's server: runs the simulation (or listens to the car) and streams it to the browser.

Plain standard library: http.server serves viewer/index.html and viewer/app.js, GET /scene gives what does not
change (the track, the car's and camera's geometry), GET /stream sends FRAME_RATE_HZ frames as server-sent events,
and POST /control takes the viewer's buttons and keys (pause, restart, speed, track, marker ID, manual driving).
"""
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
import math
from pathlib import Path
import queue
import threading
import time
from urllib.parse import urlparse

from policy import visualisation
from tests.policy.car_models import CAR_20, CarModel
from tests.policy.obstacle_simulation import CAMERA_FORWARD_M, LIDAR_FORWARD_M, LIDAR_HEIGHT_M
from tests.policy.simulator.scene import CAMERA_HEIGHT_M, CAMERA_TILT_DEG, marker_bits, marker_names
from tests.policy.simulator.sensors import SensorSettings
from tests.policy.simulator.simulation import PLANT_STEP_S, POLICY_STATE, Simulation, run_settings
from tests.policy.simulator.track import TRACKS_DIR, load_track
from tests.policy.simulator.vehicle import FRONT_AXLE_M, REAR_AXLE_M, WHEEL_TRACK_M, WHEELBASE_M

_LOGGER = logging.getLogger(__name__)

VIEWER_DIR = Path(__file__).resolve().parent / "viewer"
FRAME_RATE_HZ = 30.0
"""Frames sent to the viewer per second of wall-clock time."""
MAX_STEPS_PER_LOOP = 60
"""Most simulation steps between two frames, so a slow computer slows the run down rather than skipping frames."""
END_HOLD_S = 3.0
"""Wall-clock seconds a finished run stays on screen before --loop starts it again."""
MANUAL_FORWARD_DRIVE = 0.56
"""Drive action of the manual forwards key: about 0.75 m/s with car 20's speed map."""
MANUAL_BRAKE_DRIVE = -0.5


def layer_json(layer: visualisation.Layer) -> dict:
    """
    :return: One policy drawing layer (in base_link) as plain lists for the viewer.
    """
    return {"lines": [{"points": [[round(x, 3), round(y, 3)] for x, y in line.points], "colour": list(line.colour),
                       "closed": line.closed} for line in layer.lines],
            "spheres": [[round(s.x, 3), round(s.y, 3), s.diameter_m, list(s.colour)] for s in layer.spheres],
            "boxes": [{"centre": list(b.position), "size": list(b.size), "colour": list(b.colour),
                       "orientation": list(b.orientation)} for b in layer.boxes],
            "texts": [{"position": list(t.position), "text": t.text} for t in layer.texts]}


def car_geometry(car) -> dict:
    """
    :return: The car body's and wheels' sizes and the sensor positions, in base_link metres.
    """
    return {"front": car.body_front_m, "rear": car.body_rear_m, "width": car.body_width_m, "wheelbase": WHEELBASE_M,
            "front_axle": FRONT_AXLE_M, "rear_axle": REAR_AXLE_M, "wheel_track": WHEEL_TRACK_M,
            "wheel_diameter": car.wheel_diameter_m, "lidar": [LIDAR_FORWARD_M, 0.0, LIDAR_HEIGHT_M]}


def camera_geometry(settings: SensorSettings) -> dict:
    """
    :return: Where the camera is and what it sees, for the viewer's camera view.
    """
    return {"x": CAMERA_FORWARD_M, "z": CAMERA_HEIGHT_M, "tilt_deg": CAMERA_TILT_DEG, "hfov_deg": settings.camera_fov_deg,
            "yaw_deg": settings.camera_yaw_deg, "near": settings.camera_min_range_m, "far": settings.camera_range_m}


def bundled_tracks() -> list[dict]:
    """
    :return: Each bundled track's file name and title.
    """
    tracks = []
    for path in sorted(TRACKS_DIR.glob("*.yaml")):
        track = load_track(path)
        tracks.append({"file": path.stem, "name": track.name, "markers": len(track.markers)})
    return tracks


class SimulationRunner(threading.Thread):
    """Runs simulations in real time (or faster or slower) and makes the viewer's frames."""

    def __init__(self, track: str, marker_id: int | None = None, overrides: dict | None = None, seed: int = 0,
                 sensor_settings: SensorSettings | None = None, speed: float = 1.0, loop: bool = False,
                 foxglove_writer=None, car_model: CarModel = CAR_20):
        """
        :param track: A bundled track's name or a track file.
        :param marker_id: The ID printed on the track's first marker instead of the file's.
        :param overrides: Settings by full parameter name on top of the YAML and the track.
        :param speed: Simulated seconds per wall-clock second.
        :param loop: Start the run again after it ends.
        :param foxglove_writer: A function making a scene.SceneWriter for each new run, or None.
        :param car_model: The car's measured drive and steering response.
        """
        super().__init__(daemon=True, name="simulation")
        self.car_model = car_model
        self.track_name, self.marker_id, self.overrides = track, marker_id, overrides or {}
        self.seed, self.sensor_settings, self.speed, self.loop = seed, sensor_settings, speed, loop
        self.foxglove_writer = foxglove_writer
        self.controls: queue.Queue = queue.Queue()
        self.lock = threading.Lock()
        self.paused, self.manual, self.manual_keys = False, False, (0, 0)
        self.scene_version = 0
        self.frame_json = "{}"
        self.frame_ready = threading.Condition()
        self.frame_number = 0
        self.stop_requested = False
        self._new_run()

    def _new_run(self):
        track = load_track(self.track_name, self.marker_id)
        self.sim = Simulation(track, run_settings(track, self.overrides), self.car_model, self.sensor_settings,
                              self.seed)
        self.names = marker_names(self.sim.settings)
        self.drawer = visualisation.StepDrawer()
        self.layers: dict = {}
        self.writer = self.foxglove_writer(self.sim) if self.foxglove_writer else None
        self.ended_at: float | None = None
        with self.lock:
            self.scene_version += 1
            self.scene = self._scene()

    def _scene(self) -> dict:
        sim, track = self.sim, self.sim.track
        return {
            "mode": "sim", "version": self.scene_version,
            "track": {"file": Path(str(self.track_name)).stem, "name": track.name, "description": track.description,
                      "cones": [[c.x, c.y, c.colour.name] for c in track.cones],
                      "markers": [{"id": m.id, "x": m.x, "y": m.y, "facing_deg": m.facing_deg, "size_m": m.size_m,
                                   "height_m": m.height_m, "command": self.names.get(m.id, "no command"),
                                   "bits": None if marker_bits(m.id) is None else marker_bits(m.id).tolist()}
                                  for m in track.markers],
                      "obstacles": [{"x": o.x, "y": o.y, "length_m": o.length_m, "width_m": o.width_m,
                                     "height_m": o.height_m, "yaw_deg": o.yaw_deg} for o in track.obstacles],
                      "roads": [road.round(3).tolist() for road in track.roads], "start": list(track.start)},
            "car": car_geometry(sim.car), "camera": camera_geometry(sim.sensors.settings),
            "tracks": bundled_tracks(), "marker_names": {str(k): v for k, v in self.names.items()},
        }

    # ---- Controls ---------------------------------------------------------------------------------------------
    def control(self, request: dict):
        self.controls.put(request)

    def _apply(self, request: dict):
        op = request.get("op")
        if op == "pause":
            self.paused = True
        elif op == "resume":
            self.paused = False
        elif op == "restart":
            self._new_run()
        elif op == "speed":
            self.speed = max(0.1, min(8.0, float(request.get("value", 1.0))))
        elif op == "track":
            self.track_name, self.marker_id = str(request["value"]), None
            self._new_run()
        elif op == "marker_id":
            self.marker_id = int(request["value"])
            self._new_run()
        elif op == "manual":
            self.manual = bool(request.get("value"))
            self.manual_keys = (0, 0)
        elif op == "keys":
            self.manual_keys = (int(request.get("forward", 0)), int(request.get("turn", 0)))

    def _manual_command(self) -> tuple[float, float]:
        forward, turn = self.manual_keys
        model = self.sim.car.model
        drive = MANUAL_FORWARD_DRIVE if forward > 0 else MANUAL_BRAKE_DRIVE if forward < 0 else 0.0
        curvature = 0.9 * (model.max_left_curvature_per_m if turn > 0 else -model.max_right_curvature_per_m) \
            if turn else 0.0
        slope = model.steering_left_action_per_curvature if curvature >= 0 else model.steering_right_action_per_curvature
        return drive, model.steering_centre_action + slope * curvature

    # ---- The loop ---------------------------------------------------------------------------------------------
    def run(self):
        sim_target, last_wall = 0.0, time.monotonic()
        next_frame = last_wall
        while not self.stop_requested:
            while not self.controls.empty():
                self._apply(self.controls.get())
                if self.sim.time_s == 0.0:
                    sim_target = 0.0
            now = time.monotonic()
            if not self.paused and not self.sim.finished:
                sim_target = min(sim_target + (now - last_wall) * self.speed, self.sim.time_s + MAX_STEPS_PER_LOOP
                                 * PLANT_STEP_S)
                self.sim.manual_command = self._manual_command() if self.manual else None
                while self.sim.time_s + 1e-9 < sim_target and not self.sim.finished:
                    happened = self.sim.step()
                    if happened["policy"] and self.sim.observations is not None:
                        self.layers = {name: layer_json(layer) for name, layer in
                                       self.drawer.draw(self.sim.policy, self.sim.observations,
                                                        self.sim.actions).items()}
                    if self.writer is not None:
                        self.writer.publish(happened)
            else:
                sim_target = self.sim.time_s
            last_wall = now
            if self.sim.finished:
                if self.ended_at is None:
                    self.ended_at = now
                    summary = self.sim.summary()
                    _LOGGER.info("Run ended after %.1f s: %s; %.2f m, %d cones touched", summary.time_s,
                                 summary.ended_because, summary.distance_m, summary.cones_touched)
                elif self.loop and now - self.ended_at >= END_HOLD_S:
                    self._new_run()
                    sim_target = 0.0
            if now >= next_frame:
                next_frame = now + 1.0 / FRAME_RATE_HZ
                self._publish_frame()
            time.sleep(0.002)

    def _publish_frame(self):
        frame = to_json(self.frame())
        with self.frame_ready:
            self.frame_json, self.frame_number = frame, self.frame_number + 1
            self.frame_ready.notify_all()

    def frame(self) -> dict:
        sim, state = self.sim, self.sim.car.state
        policy, lanes = sim.policy, sim.policy.lanes
        finite = lambda v, digits=3: round(v, digits) if v is not None and math.isfinite(v) else None
        lidar = [] if sim.lidar.latest is None else [[round(p.pos.x, 3), round(p.pos.y, 3), round(p.pos.z, 3)]
                                                    for p in sim.lidar.latest[1]]
        intersection = sim.memory.data.intersection
        return {
            "mode": "sim", "scene_version": self.scene_version, "t": round(sim.time_s, 3), "paused": self.paused,
            "speed_factor": self.speed, "manual": self.manual, "ended": sim.ended_because,
            "car": {"x": state.x, "y": state.y, "heading": state.heading, "speed": state.speed_m_per_s,
                    "curvature": state.curvature_per_m, "wheel_angle": sim.car.front_wheel_angle_rad,
                    "drive": state.drive_applied, "steer": state.steering_applied, "distance": state.distance_m},
            "node": {"state": sim.node_state, "reason": sim.node_reason, "driving": sim.node_state == POLICY_STATE},
            "touched": sorted(sim.cones_touched), "obstacles_touched": sorted(sim.obstacles_touched),
            "markers_seen": sim.sensors.markers_seen(state.pose), "layers": self.layers, "lidar": lidar,
            "hud": {
                "lane": None if lanes is None else lanes.status.name,
                "lateral_error_m": None if lanes is None or not lanes.has_lane else finite(lanes.lateral_error_m),
                "heading_error_deg": None if lanes is None or not lanes.has_lane
                else finite(math.degrees(lanes.heading_error_rad), 1),
                "cones_detected": 0 if sim.observations is None or sim.observations.cones is None
                else len(sim.observations.cones),
                "cones_used": len(policy.cones or []),
                "marker_command": None if policy.marker_command is None else policy.marker_command.name,
                "next_intersection": None if policy.intersection_command is None else policy.intersection_command.name,
                "intersection_phase": None if intersection is None else intersection.phase,
                "lane_side": getattr(policy.action_policy.lane_side, "name", str(policy.action_policy.lane_side)),
                "obstacle_distance_m": finite(policy.obstacle_distance_m),
                "policy_drive": None if sim.actions is None else finite(sim.actions.drive_action),
                "policy_steer": None if sim.actions is None else finite(sim.actions.steering_action),
                "policy_steps": sim.policy_steps,
                "mpc_stop": None if sim.memory.data.mpc_stop is None else sim.memory.data.mpc_stop.name},
            "events": [f"{e.time_s:6.2f} s  {e.text}" for e in sim.events[-8:]],
        }


def _finite(value):
    """Replaces every NaN or infinite number with None, which JSON can hold and browsers accept."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(item) for item in value]
    return value


def to_json(value) -> str:
    """
    :return: Compact JSON for the viewer, with non-finite numbers as null and other types as text.
    """
    return json.dumps(_finite(value), separators=(",", ":"), allow_nan=False, default=str)


class LiveRunner:
    """Gives the viewer the car's live, rebuilt surroundings (live.LiveCar)."""

    def __init__(self, host: str, port: int):
        from tests.policy.simulator.live import LiveCar
        settings = run_settings(load_track("lab_straight"))
        self.names = marker_names(settings)
        self.car = LiveCar(host, port, self.names)
        self.frame_ready = threading.Condition()
        self.frame_json, self.frame_number = "{}", 0
        self.stop_requested = False
        self.address = f"{host}:{port}"
        sensors = SensorSettings()
        from tests.policy.simulator.vehicle import SimulatedCar
        from tests.policy.car_models import CAR_20
        self.scene = {"mode": "car", "version": 1, "address": self.address,
                      "track": {"file": "", "name": f"Live car {self.address}", "description": "", "cones": [],
                                "markers": [], "obstacles": [], "roads": [], "start": [0, 0, 0]},
                      "car": car_geometry(SimulatedCar(CAR_20, (0.0, 0.0, 0.0))), "camera": camera_geometry(sensors),
                      "tracks": [], "marker_names": {str(k): v for k, v in self.names.items()}}

    def start(self):
        self.car.start()
        threading.Thread(target=self._frames, daemon=True, name="live-frames").start()

    def _frames(self):
        bits = {}
        while not self.stop_requested:
            frame = self.car.frame()
            for marker in frame["markers"]:
                if marker["id"] not in bits:
                    found = marker_bits(marker["id"])
                    bits[marker["id"]] = None if found is None else found.tolist()
                marker["bits"] = bits[marker["id"]]
            frame["scene_version"] = 1
            text = to_json(frame)
            with self.frame_ready:
                self.frame_json, self.frame_number = text, self.frame_number + 1
                self.frame_ready.notify_all()
            time.sleep(1.0 / FRAME_RATE_HZ)

    def control(self, request: dict):
        self.car.control(request)


def make_handler(runner):
    """
    :return: A request handler class serving the viewer for this runner.
    """
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            _LOGGER.debug("viewer: " + fmt, *args)

        def _send(self, body: bytes, content_type: str, status: int = 200):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                self._send((VIEWER_DIR / "index.html").read_bytes(), "text/html; charset=utf-8")
            elif path == "/app.js":
                self._send((VIEWER_DIR / "app.js").read_bytes(), "text/javascript; charset=utf-8")
            elif path == "/scene":
                with getattr(runner, "lock", threading.Lock()):
                    scene = runner.scene
                self._send(to_json(scene).encode(), "application/json")
            elif path == "/stream":
                self._stream()
            else:
                self._send(b"not found", "text/plain", 404)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            seen = -1
            try:
                while not runner.stop_requested:
                    with runner.frame_ready:
                        runner.frame_ready.wait_for(lambda: runner.frame_number != seen, timeout=1.0)
                        seen, frame = runner.frame_number, runner.frame_json
                    self.wfile.write(f"data: {frame}\n\n".encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return

        def do_POST(self):
            if urlparse(self.path).path != "/control":
                self._send(b"not found", "text/plain", 404)
                return
            length = int(self.headers.get("Content-Length", 0))
            try:
                request = json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError:
                self._send(b"bad request", "text/plain", 400)
                return
            runner.control(request)
            self._send(b"{}", "application/json")

    return Handler


def serve(runner, host: str, port: int) -> ThreadingHTTPServer:
    """Starts the runner and the viewer's web server in the background.

    :return: The server; call shutdown() to stop it.
    """
    server = ThreadingHTTPServer((host, port), make_handler(runner))
    server.daemon_threads = True
    runner.start()
    threading.Thread(target=server.serve_forever, daemon=True, name="viewer-http").start()
    return server
