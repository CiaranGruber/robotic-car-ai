"""Runs the car simulator with its live 3D viewer, or shows the real car's surroundings in the same viewer.

Without the car, from the repository root (then open http://localhost:8000):
    python -m tests.policy.simulator cross --marker-id 3
    python -m tests.policy.simulator lab_straight --speed 2 --loop
    python -m tests.policy.simulator my_track.yaml --set mpc.target_speed_m_per_s=0.4
With the car connected (read-only, through its Foxglove bridge):
    python -m tests.policy.simulator --car 10.43.254.26:1234
Without a browser (prints how the run went):
    python -m tests.policy.simulator cross --headless
"""
from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys
import time
import webbrowser

import yaml

from tests.policy.car_models import CARS
from tests.policy.simulator.sensors import SensorSettings
from tests.policy.simulator.server import LiveRunner, SimulationRunner, bundled_tracks, serve
from tests.policy.simulator.simulation import Simulation, aruco_settings, run_settings
from tests.policy.simulator.track import load_track

_LOGGER = logging.getLogger("tests.policy.simulator")


def _setting(text: str) -> tuple[str, object]:
    name, separator, value = text.partition("=")
    if not separator or not name:
        raise argparse.ArgumentTypeError(f"use NAME=VALUE, such as mpc.target_speed_m_per_s=0.4, not {text!r}")
    return name.strip(), yaml.safe_load(value)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m tests.policy.simulator", description=__doc__.split("\n")[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     epilog="Bundled tracks: " + ", ".join(t["file"] for t in bundled_tracks()))
    parser.add_argument("track", nargs="?", default="lab_straight", help="A bundled track or a track file.")
    parser.add_argument("--car", metavar="HOST:PORT",
                        help="Show the real car's surroundings from its Foxglove bridge instead of simulating.")
    parser.add_argument("--marker-id", type=int, help="ID printed on the track's first marker, such as 3, 4 or 5.")
    parser.add_argument("--set", action="append", default=[], type=_setting, metavar="NAME=VALUE",
                        help="A policy or node setting by full name, on top of config/ai4r_policy.yaml and the track.")
    parser.add_argument("--speed", type=float, default=1.0, help="Simulated seconds per real second. Default 1.")
    parser.add_argument("--loop", action="store_true", help="Start the run again a few seconds after it ends.")
    parser.add_argument("--seed", type=int, default=0, help="Sensor noise seed. Default 0.")
    parser.add_argument("--no-noise", action="store_true", help="Exact sensors, for repeatable checks.")
    parser.add_argument("--miss", type=float, default=0.0, help="Chance a cone is missing from a camera frame.")
    parser.add_argument("--camera-yaw-deg", type=float, default=0.0,
                        help="How far left the simulated camera really points, degrees. Default 0.")
    parser.add_argument("--car-model", choices=sorted(CARS), default="car20", help="Measured car response. Default car20.")
    parser.add_argument("--host", default="127.0.0.1", help="Viewer address. Use 0.0.0.0 to share on the network.")
    parser.add_argument("--port", type=int, default=8000, help="Viewer port. Default 8000.")
    parser.add_argument("--no-browser", action="store_true", help="Do not open the viewer in a browser.")
    parser.add_argument("--foxglove", type=int, nargs="?", const=8765, metavar="PORT",
                        help="Also serve the run to Foxglove (Open connection, ws://localhost:PORT). Default 8765.")
    parser.add_argument("--record", type=Path, metavar="FILE.mcap", help="Also record the run for Foxglove.")
    parser.add_argument("--headless", action="store_true", help="Run once as fast as possible and print the result.")
    return parser


def _sensor_settings(args) -> SensorSettings:
    aruco = aruco_settings()
    return SensorSettings(camera_yaw_deg=args.camera_yaw_deg, cone_miss_probability=args.miss,
                          noise=not args.no_noise, min_consecutive_frames=int(aruco["min_consecutive_frames"]),
                          allowed_marker_ids=tuple(aruco.get("allowed_marker_ids", []) or ()))


def _headless(args) -> int:
    track = load_track(args.track, args.marker_id)
    sim = Simulation(track, run_settings(track, dict(args.set)), CARS[args.car_model], _sensor_settings(args), args.seed)
    started = time.perf_counter()
    while not sim.finished:
        sim.step()
    summary = sim.summary()
    print(f"{summary.track}: {summary.ended_because} after {summary.time_s:.1f} s "
          f"({time.perf_counter() - started:.1f} s to compute)")
    print(f"  drove {summary.distance_m:.2f} m, touched {summary.cones_touched} cones and "
          f"{summary.obstacles_touched} obstacles, at most {summary.max_off_road_m:.2f} m off the road")
    for event in summary.events:
        print(f"  {event.time_s:6.2f} s  {event.text}")
    return 0


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = build_parser().parse_args(argv)
    if args.headless:
        return _headless(args)
    if args.car:
        host, _, port = args.car.rpartition(":")
        if not host or not port.isdigit():
            raise SystemExit("--car needs HOST:PORT, such as 10.43.254.26:1234 (dream's foxglove_bridge port)")
        runner = LiveRunner(host, int(port))
    else:
        load_track(args.track, args.marker_id)  # Report a bad track before starting anything
        writer = None
        if args.foxglove is not None or args.record is not None:
            import foxglove
            from tests.policy.simulator.scene import SceneWriter
            if args.foxglove is not None:
                foxglove.start_server(name="AI4R car simulator", host=args.host, port=args.foxglove)
                _LOGGER.info("Foxglove: Open connection > Foxglove WebSocket > ws://localhost:%d", args.foxglove)
            if args.record is not None:
                foxglove.open_mcap(args.record, allow_overwrite=True)
                _LOGGER.info("Recording to %s", args.record)
            writer = lambda sim: SceneWriter(sim, time.time_ns())
        runner = SimulationRunner(args.track, args.marker_id, dict(args.set), args.seed, _sensor_settings(args),
                                  args.speed, args.loop, writer, CARS[args.car_model])
    server = serve(runner, args.host, args.port)
    url = f"http://{'localhost' if args.host in ('127.0.0.1', '0.0.0.0') else args.host}:{args.port}"
    _LOGGER.info("Viewer: %s  (Ctrl+C to stop)", url)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        runner.stop_requested = True
        server.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
