"""With the car: rebuilds the car's surroundings in 3D from its live sensor streams, through its Foxglove bridge.

Read-only: LiveCar subscribes to topics and never publishes, calls a service or sets a parameter, so it cannot move
the car or change its settings. It listens to:
- cone_detections (in base_link) and fiducial_detections (in the camera's optical frame, moved to base_link with
  the camera transform from /tf_static, or the mount's default);
- scan (the lidar's LaserScan, moved to base_link with its /tf_static transform or the mount's default);
- wheel_speed_m_per_sec and imu/data, integrated into an odometry pose so the rebuilt cones and markers stay put
  while the car drives. The pose drifts; it is only for drawing;
- policy_fsm_state_string, drive_and_steer_set_point_normalized and policy_visualisation/markers (the policy's own
  lane, obstacle and plan drawings, as policy_node publishes them).
Cones and markers are remembered for CONE_MEMORY_S and MARKER_MEMORY_S after they were last seen.
"""
from __future__ import annotations

import base64
from collections import deque
import json
import logging
import math
import os
import socket
import struct
import threading
import time

from tests.policy.simulator.ros2_cdr import MessageDecoder

_LOGGER = logging.getLogger(__name__)

SUBPROTOCOLS = ("foxglove.sdk.v1", "foxglove.websocket.v1")
"""WebSocket subprotocols to try, newest first. foxglove-bridge 3.x only accepts foxglove.sdk.v1."""
TOPICS = {"cones": "cone_detections", "fiducials": "fiducial_detections", "scan": "scan",
          "wheel": "wheel_speed_m_per_sec", "imu": "imu/data", "state": "policy_fsm_state_string",
          "drawings": "policy_visualisation/markers", "command": "drive_and_steer_set_point_normalized",
          "tf_static": "tf_static"}
"""What LiveCar subscribes to, by the end of each topic's name."""
CONE_MEMORY_S = 4.0
"""Seconds a cone stays in the rebuilt scene after it was last detected."""
MARKER_MEMORY_S = 6.0
"""Seconds a marker stays in the rebuilt scene after it was last decoded."""
MERGE_M = 0.12
"""A new detection this close to a remembered cone of the same colour, in metres, is the same cone."""
RECONNECT_S = 2.0
"""Seconds to wait before connecting again after the bridge drops."""
DEFAULT_TRANSFORMS = {
    # The GPU cone detector's static transform on the car (camera_mount.yaml defaults: 0.10 m behind base_link,
    # 0.30 m up, 20 degrees down), as base_link -> optical frame: translation, quaternion x, y, z, w.
    "camera": ((-0.10, 0.0, 0.30), (-0.579228, 0.579228, -0.405580, 0.405580)),
    # lidar_mount.yaml's defaults: 0.20 m ahead, 0.12 m up, turned 180 degrees.
    "lidar": ((0.20, 0.0, 0.12), (0.0, 0.0, 1.0, 0.0)),
}


def rotate(q: tuple[float, float, float, float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    :return: The vector v turned by the unit quaternion q (x, y, z, w).
    """
    x, y, z, w = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx)


def multiply(a: tuple, b: tuple) -> tuple[float, float, float, float]:
    """
    :return: The quaternion product a * b, both (x, y, z, w).
    """
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


class BridgeClient:
    """A minimal read-only Foxglove WebSocket client."""

    def __init__(self, host: str, port: int, timeout_s: float = 5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout_s)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                           f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
                           f"Sec-WebSocket-Protocol: {', '.join(SUBPROTOCOLS)}\r\n\r\n").encode())
        self.buffer = b""
        while b"\r\n\r\n" not in self.buffer:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("the bridge closed the connection during the handshake")
            self.buffer += chunk
        head, self.buffer = self.buffer.split(b"\r\n\r\n", 1)
        if b" 101 " not in head.split(b"\r\n")[0]:
            raise ConnectionError(f"the bridge refused the connection: {head.decode(errors='replace')[:120]}")
        self.sock.settimeout(1.0)

    def _exact(self, n: int) -> bytes:
        while len(self.buffer) < n:
            chunk = self.sock.recv(1 << 20)
            if not chunk:
                raise ConnectionError("the bridge closed the connection")
            self.buffer += chunk
        out, self.buffer = self.buffer[:n], self.buffer[n:]
        return out

    def send(self, opcode: int, payload: bytes):
        mask = os.urandom(4)
        n = len(payload)
        length = bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack(">H", n) if n < 65536 \
            else bytes([0x80 | 127]) + struct.pack(">Q", n)
        self.sock.sendall(bytes([0x80 | opcode]) + length + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def send_json(self, message: dict):
        self.send(1, json.dumps(message).encode())

    def messages(self):
        """Yields (opcode, payload) for each complete message, or (None, b'') each second of silence."""
        data, kind = b"", None
        while True:
            try:
                b0, b1 = self._exact(2)
            except (socket.timeout, TimeoutError):
                yield None, b""
                continue
            opcode, n = b0 & 0x0F, b1 & 0x7F
            n = struct.unpack(">H", self._exact(2))[0] if n == 126 else struct.unpack(">Q", self._exact(8))[0] \
                if n == 127 else n
            payload = self._exact(n)
            if opcode == 9:
                self.send(10, payload)
            elif opcode == 8:
                raise ConnectionError("the bridge closed the connection")
            elif opcode in (1, 2):
                data, kind = payload, opcode
            elif opcode == 0:
                data += payload
            if opcode in (0, 1, 2) and b0 & 0x80 and kind is not None:
                yield kind, data
                data, kind = b"", None

    def close(self):
        try:
            self.send(8, b"")
            self.sock.close()
        except OSError:
            pass


class LiveCar(threading.Thread):
    """Listens to one car and keeps a picture of its surroundings for the viewer."""

    def __init__(self, host: str, port: int, marker_names: dict[int, str] | None = None):
        super().__init__(daemon=True, name="live-car")
        self.host, self.port = host, port
        self.marker_names = marker_names or {}
        self.lock = threading.Lock()
        self.stop_requested = False
        self.status = f"connecting to {host}:{port}"
        self.started_at = time.monotonic()
        self.pose = [0.0, 0.0, 0.0]
        self.speed, self.yaw_rate = 0.0, 0.0
        self.last_imu_at: float | None = None
        self.cones: list[dict] = []
        self.markers: dict[int, dict] = {}
        self.lidar: list[list[float]] = []
        self.layers: dict[str, dict] = {}
        self.node_state, self.command = "", {}
        self.transforms: dict[str, dict[str, tuple]] = {}
        self.colour_names: dict[int, str] = {}
        self.counts: dict[str, deque] = {name: deque(maxlen=200) for name in TOPICS}
        self.paused = False

    # ---- Connection -------------------------------------------------------------------------------------------
    def run(self):
        while not self.stop_requested:
            try:
                self._session()
            except (OSError, ConnectionError, ValueError) as exc:
                with self.lock:
                    self.status = f"no connection to {self.host}:{self.port}: {exc}"
                _LOGGER.warning("%s", self.status)
                time.sleep(RECONNECT_S)

    def _session(self):
        client = BridgeClient(self.host, self.port)
        channels, subscribed, decoders = {}, {}, {}
        advertised_at = None
        try:
            for kind, payload in client.messages():
                if self.stop_requested:
                    return
                if kind == 1:
                    message = json.loads(payload)
                    if message.get("op") == "advertise":
                        for channel in message["channels"]:
                            channels[channel["id"]] = channel
                        advertised_at = time.monotonic()
                    elif message.get("op") == "unadvertise":
                        for channel_id in message.get("channelIds", []):
                            channels.pop(channel_id, None)
                elif kind == 2 and payload and payload[0] == 1:
                    subscription = struct.unpack_from("<I", payload, 1)[0]
                    if subscription in subscribed:
                        name, decoder = subscribed[subscription], decoders[subscription]
                        try:
                            self._handle(name, decoder, decoder.decode(payload[13:]))
                        except (ValueError, struct.error, KeyError, IndexError) as exc:
                            _LOGGER.debug("could not decode %s: %s", name, exc)
                # Subscribe once the advertisements have settled, and again to new topics
                if advertised_at is not None and time.monotonic() - advertised_at > 0.5:
                    advertised_at = None
                    subscriptions = []
                    # Prefer the car's own namespace when the same topic exists twice
                    for channel in sorted(channels.values(), key=lambda c: not c["topic"].startswith("/car/")):
                        name = self._topic_name(channel["topic"])
                        if name is None or channel.get("encoding") != "cdr" or name in subscribed.values():
                            continue
                        subscription = len(subscribed) + 1
                        subscribed[subscription] = name
                        decoders[subscription] = MessageDecoder(channel["schemaName"], channel.get("schema", ""))
                        subscriptions.append({"id": subscription, "channelId": channel["id"]})
                    if subscriptions:
                        client.send_json({"op": "subscribe", "subscriptions": subscriptions})
                    with self.lock:
                        found = sorted(set(subscribed.values()))
                        self.status = f"connected to {self.host}:{self.port}: {', '.join(found) or 'no known topics'}"
        finally:
            client.close()

    @staticmethod
    def _topic_name(topic: str) -> str | None:
        for name, suffix in TOPICS.items():
            if topic.rstrip("/").endswith("/" + suffix) or topic.lstrip("/") == suffix:
                return name
        return None

    def stop(self):
        self.stop_requested = True

    # ---- Messages ---------------------------------------------------------------------------------------------
    def _transform(self, frame: str, default: str) -> tuple[tuple, tuple]:
        """
        :return: base_link -> frame as (translation, quaternion), from /tf_static or the mount default.
        """
        chain = self._chain("base_link", frame)
        return chain if chain is not None else DEFAULT_TRANSFORMS[default]

    def _chain(self, parent: str, child: str, depth: int = 0):
        direct = self.transforms.get(parent, {}).get(child)
        if direct is not None or depth > 3:
            return direct
        for middle, first in self.transforms.get(parent, {}).items():
            rest = self._chain(middle, child, depth + 1)
            if rest is not None:
                translation = tuple(a + b for a, b in zip(first[0], rotate(first[1], rest[0])))
                return translation, multiply(first[1], rest[1])
        return None

    def _handle(self, name: str, decoder: MessageDecoder, message: dict):
        now = time.monotonic()
        with self.lock:
            self.counts[name].append(now)
            if self.paused and name not in ("tf_static",):
                return
            getattr(self, f"_on_{name}")(message, decoder, now)

    def _on_tf_static(self, message, decoder, now):
        for transform in message.get("transforms", []):
            t, r = transform["transform"]["translation"], transform["transform"]["rotation"]
            parent = transform["header"]["frame_id"].lstrip("/")
            self.transforms.setdefault(parent, {})[transform["child_frame_id"].lstrip("/")] = (
                (t["x"], t["y"], t["z"]), (r["x"], r["y"], r["z"], r["w"]))

    def _on_wheel(self, message, decoder, now):
        self.speed = float(message.get("data", 0.0))
        if self.last_imu_at is None or now - self.last_imu_at > 0.5:
            self._integrate(now, 0.0)

    def _on_imu(self, message, decoder, now):
        w = message["angular_velocity"]
        frame = message["header"]["frame_id"].lstrip("/")
        chain = self._chain("base_link", frame)
        rate = rotate(chain[1], (w["x"], w["y"], w["z"]))[2] if chain is not None else w["z"]
        self._integrate(now, rate)
        self.last_imu_at = now

    def _integrate(self, now, yaw_rate):
        last = getattr(self, "_last_integrated_at", None)
        self._last_integrated_at = now
        if last is None or now - last > 0.5:
            return
        dt = now - last
        self.yaw_rate = yaw_rate
        heading = self.pose[2] + yaw_rate * dt / 2
        self.pose[0] += self.speed * dt * math.cos(heading)
        self.pose[1] += self.speed * dt * math.sin(heading)
        self.pose[2] = math.atan2(math.sin(self.pose[2] + yaw_rate * dt), math.cos(self.pose[2] + yaw_rate * dt))

    def _world(self, x: float, y: float) -> tuple[float, float]:
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        return self.pose[0] + c * x - s * y, self.pose[1] + s * x + c * y

    def _on_cones(self, message, decoder, now):
        if not self.colour_names:
            for value in decoder.constants.values():
                for key, number in value.items():
                    if key.startswith("COLOR_"):
                        self.colour_names[int(number)] = key.removeprefix("COLOR_")
        for cone in self.cones:
            cone["now"] = False
        for detection in message.get("detections", []):
            p = detection["position"]
            colour = self.colour_names.get(int(detection.get("color", -1)), "UNKNOWN")
            x, y = self._world(p["x"], p["y"])
            match = next((c for c in self.cones if c["colour"] == colour
                          and math.hypot(c["x"] - x, c["y"] - y) < MERGE_M), None)
            if match is None:
                self.cones.append({"x": x, "y": y, "colour": colour, "seen": now, "now": True})
            else:
                match.update(x=(match["x"] + x) / 2, y=(match["y"] + y) / 2, seen=now, now=True)
        self.cones = [c for c in self.cones if now - c["seen"] <= CONE_MEMORY_S]

    def _on_fiducials(self, message, decoder, now):
        frame = message["header"]["frame_id"].lstrip("/")
        translation, rotation = self._transform(frame, "camera") if frame != "base_link" else \
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        for detection in message.get("detections", []):
            if not detection.get("pose_valid"):
                continue
            p, q = detection["pose"]["position"], detection["pose"]["orientation"]
            body = tuple(a + b for a, b in zip(translation, rotate(rotation, (p["x"], p["y"], p["z"]))))
            normal = rotate(rotation, rotate((q["x"], q["y"], q["z"], q["w"]), (0.0, 0.0, 1.0)))
            x, y = self._world(body[0], body[1])
            facing = math.degrees(self.pose[2] + math.atan2(normal[1], normal[0]))
            self.markers[int(detection["id"])] = {"id": int(detection["id"]), "x": x, "y": y, "z": body[2],
                                                  "facing_deg": facing, "size_m": float(detection["marker_size_m"]),
                                                  "seen": now,
                                                  "command": self.marker_names.get(int(detection["id"]), "no command")}
        self.markers = {k: m for k, m in self.markers.items() if now - m["seen"] <= MARKER_MEMORY_S}

    def _on_scan(self, message, decoder, now):
        translation, rotation = self._transform(message["header"]["frame_id"].lstrip("/"), "lidar")
        points, step = [], max(1, len(message["ranges"]) // 720)
        low, high = message["range_min"], message["range_max"]
        for i in range(0, len(message["ranges"]), step):
            r = message["ranges"][i]
            if not (math.isfinite(r) and low <= r <= high):
                continue
            angle = message["angle_min"] + i * message["angle_increment"]
            x, y, z = rotate(rotation, (r * math.cos(angle), r * math.sin(angle), 0.0))
            points.append([round(x + translation[0], 3), round(y + translation[1], 3), round(z + translation[2], 3)])
        self.lidar = points

    def _on_state(self, message, decoder, now):
        self.node_state = message.get("data", "")

    def _on_command(self, message, decoder, now):
        self.command = {k: v for k, v in message.items() if isinstance(v, (int, float))}

    def _on_drawings(self, message, decoder, now):
        layers: dict[str, dict] = {}
        for marker in message.get("markers", []):
            if marker.get("action") == 3:  # DELETEALL
                continue
            layer = layers.setdefault(marker.get("ns", "") or "policy",
                                      {"lines": [], "spheres": [], "boxes": [], "texts": []})
            pose, scale, colour = marker["pose"], marker["scale"], marker["color"]
            position = (pose["position"]["x"], pose["position"]["y"], pose["position"]["z"])
            q = (pose["orientation"]["x"], pose["orientation"]["y"], pose["orientation"]["z"],
                 pose["orientation"]["w"])
            if math.hypot(*q) < 1e-6:
                q = (0.0, 0.0, 0.0, 1.0)
            rgba = [colour["r"], colour["g"], colour["b"], colour["a"]]
            place = lambda p: [a + b for a, b in zip(position, rotate(q, (p["x"], p["y"], p["z"])))]
            points = [place(p) for p in marker.get("points", [])]
            kind = marker.get("type")
            if kind in (4, 5) and points:  # LINE_STRIP, LINE_LIST
                pairs = [points] if kind == 4 else [points[i:i + 2] for i in range(0, len(points) - 1, 2)]
                layer["lines"] += [{"points": [p[:2] for p in line], "colour": rgba, "closed": False} for line in pairs]
            elif kind in (2, 7):  # SPHERE, SPHERE_LIST
                for p in points or [list(position)]:
                    layer["spheres"].append([p[0], p[1], scale["x"], rgba])
            elif kind in (1, 6):  # CUBE, CUBE_LIST
                for p in points or [list(position)]:
                    layer["boxes"].append({"centre": p, "size": [scale["x"], scale["y"], scale["z"]],
                                           "colour": rgba, "orientation": list(q)})
            elif kind == 9 and marker.get("text"):  # TEXT_VIEW_FACING
                layer["texts"].append({"position": list(position), "text": marker["text"]})
        self.layers = layers

    # ---- For the viewer ---------------------------------------------------------------------------------------
    def rates(self, now: float) -> dict[str, dict]:
        rates = {}
        for name, times in self.counts.items():
            recent = [t for t in times if now - t <= 2.0]
            rates[name] = {"rate_hz": round(len(recent) / 2.0, 1),
                           "age_s": None if not times else round(now - times[-1], 2)}
        return rates

    def frame(self) -> dict:
        now = time.monotonic()
        with self.lock:
            return {"mode": "car", "status": self.status, "t": now - self.started_at, "paused": self.paused,
                    "car": {"x": self.pose[0], "y": self.pose[1], "heading": self.pose[2], "speed": self.speed,
                            "wheel_angle": 0.0, "drive": self.command.get("drive"), "steer": self.command.get("steer"),
                            "yaw_rate": self.yaw_rate},
                    "cones": [[round(c["x"], 3), round(c["y"], 3), c["colour"], round(now - c["seen"], 2), c["now"]]
                              for c in self.cones],
                    "markers": [{**{k: v for k, v in m.items() if k != "seen"}, "age_s": round(now - m["seen"], 2)}
                                for m in self.markers.values()],
                    "lidar": self.lidar, "layers": self.layers,
                    "hud": {"node": self.node_state, "topics": self.rates(now)}}

    def control(self, request: dict):
        op = request.get("op")
        with self.lock:
            if op == "pause":
                self.paused = True
            elif op == "resume":
                self.paused = False
            elif op in ("restart", "clear"):
                self.cones, self.markers, self.pose = [], {}, [0.0, 0.0, 0.0]
