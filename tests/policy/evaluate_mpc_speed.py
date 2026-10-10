"""Compare nominal and measured-speed MPC with fixed, uncalibrated speed dynamics.

Run: .venv/bin/python tests/policy/evaluate_mpc_speed.py
These sensitivity cases are development cases, not held-out final validation.
Speed is simulated telemetry; pose is never given to the policy. This isolates
the model change; it does not emulate CarControl or DREAMGym dynamics.
"""
import csv
import dataclasses
import hashlib
import json
import math
import os
from pathlib import Path
import platform

import numpy as np
import scipy

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tests.policy.evaluate_mpc import Case, MpcConfig, ROOT, rollout


CASES = [
    Case("S1-acceleration", offset=0.2, delay_steps=2, noise=0.01, speed_response_s=0.5),
    Case("S2-half-speed", offset=-0.2, heading=math.radians(-5), delay_steps=2,
         noise=0.01, speed_response_s=0.5, speed_gain=0.5),
    Case("S3-slow-dropouts", offset=0.15, heading=math.radians(10), delay_steps=2,
         noise=0.01, speed_response_s=0.3, speed_gain=0.6, empty_every=3),
]


def spatial_error(trace, finish_m=4.5):
    """Integrate lateral error over the same distance, interpolating the final segment."""
    finish = min(finish_m, float(trace[-1, 1]))
    section = trace[trace[:, 1] < finish]
    xs = np.append(section[:, 1], finish)
    ys = np.append(section[:, 2], np.interp(finish, trace[:, 1], trace[:, 2]))
    length = float(xs[-1] - xs[0])
    return (float(np.sqrt(np.trapezoid(ys ** 2, xs) / length)) if length > 0 else None), length


def main():
    output = ROOT / "build/mpc-speed-evaluation"
    output.mkdir(parents=True, exist_ok=True)
    rows, traces = [], {}
    for case in CASES:
        for feedback in (False, True):
            name = "measured-speed" if feedback else "nominal-speed"
            for seed in range(200, 205):
                metrics, trace = rollout(MpcConfig(), case, seed, measured_speed=feedback)
                # Spatial integral prevents slower trajectories or longer stopped
                # tails from making tracking appear better through time weighting.
                spatial_rms, length = spatial_error(trace)
                rows.append(dict(model=name, spatial_rms_m=spatial_rms,
                                 scored_length_m=length, **metrics))
                if seed == 200:
                    traces[(case.name, name)] = trace
    with (output / "metrics.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summaries = []
    for case in CASES:
        for name in ("nominal-speed", "measured-speed"):
            group = [row for row in rows if row["case"] == case.name and row["model"] == name]
            summaries.append(dict(case=case.name, model=name,
                spatial_rms_m=float(np.mean([row["spatial_rms_m"] for row in group])),
                min_scored_length_m=min(row["scored_length_m"] for row in group),
                worst_p99_ms=max(row["p99_update_ms"] for row in group),
                fallbacks=sum(row["fallbacks"] for row in group),
                min_stop_x=min(row["stop_x_m"] for row in group),
                max_stop_x=max(row["stop_x_m"] for row in group),
                passes=sum(row["pass"] for row in group)))
    report = dict(summaries=summaries, cases=[dataclasses.asdict(case) for case in CASES],
                  config=dataclasses.asdict(MpcConfig()), seeds=list(range(200, 205)),
                  python=platform.python_version(), platform=platform.platform(),
                  numpy=np.__version__, scipy=scipy.__version__,
                  source_sha256={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in [Path(__file__).resolve(), Path(__file__).with_name("evaluate_mpc.py"),
                                              *sorted((ROOT / "policy").rglob("*.py"))]})
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    os.environ.setdefault("MPLCONFIGDIR", str(output / ".matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    for ax, case in zip(axes, CASES):
        for name in ("nominal-speed", "measured-speed"):
            trace = traces[(case.name, name)]
            ax.plot(trace[:, 1], trace[:, 2], label=name)
        ax.axhline(0, color="gray", linewidth=0.5)
        ax.axvline(6, color="gray", linestyle="--")
        ax.set(title=case.name, xlabel="Distance along lane (m)", ylabel="Lateral position (m)")
        ax.legend(fontsize=8)
    fig.suptitle("Speed-model sensitivity — illustrative dynamics, seed 200")
    fig.savefig(output / "trajectories.png", dpi=160)
    plt.close(fig)
    print(json.dumps(summaries, indent=2))


if __name__ == "__main__":
    main()
