"""Final-cone-pair experiment; no simulator finish coordinate is passed to policy.

Run: .venv/bin/python tests/policy/evaluate_endpoint.py
The baseline is measured-speed MpcPolicy. Endpoint mode is explicitly selected
here only. Cases and defaults are fixed before this batch; these are development
scenarios, not blinded validation. Missing/occluded final cones deliberately
demonstrate the unresolvable ambiguity of the selected finish cue.
"""
import csv
import dataclasses
import hashlib
import json
import os
from pathlib import Path
import platform

import numpy as np
import scipy

from evaluate_mpc import Case, MpcConfig, MpcPolicy, ROOT, rollout
from policy.action_policy.endpoint import EndpointConfig, EndpointMpcPolicy


CASES = [
    Case("E1-ideal", offset=0.15),
    Case("E2-noise-delay-lag", offset=0.15, noise=0.01, delay_steps=2, speed_response_s=0.3),
    Case("E3-brief-occlusion", noise=0.01, delay_steps=2, speed_response_s=0.3,
         occlusion_start_step=20, occlusion_end_step=23),
    Case("E4-missing-final-pair", hide_from_x_m=6.0, speed_response_s=0.3),
    Case("E5-persistent-occlusion", hide_from_x_m=4.0, speed_response_s=0.3),
    Case("E6-stream-failure", stream_loss_step=50, speed_response_s=0.3),
    Case("E7-approach-occlusion", noise=0.01, delay_steps=2, speed_response_s=0.3,
         occlusion_start_step=50, occlusion_end_step=60),
]


def main():
    output = ROOT / "build/mpc-endpoint-evaluation"
    output.mkdir(parents=True, exist_ok=True)
    rows, traces = [], {}
    for case in CASES:
        for name, factory in (("baseline", MpcPolicy), ("endpoint", EndpointMpcPolicy)):
            for seed in range(300, 305):
                metrics, trace = rollout(MpcConfig(), case, seed, measured_speed=True, policy_factory=factory)
                # Course completion is deliberately separate from fault handling.
                # A fault case may correctly stop without completing the course.
                rows.append(dict(policy=name, **metrics))
                if seed == 300:
                    # Include the post-integration endpoint, particularly the
                    # instantaneous zero-speed step in the ideal plant case.
                    final = trace[-1].copy()
                    final[0] += 0.1
                    final[1] = metrics["stop_x_m"]
                    final[6] = metrics["stop_speed_m_per_s"]
                    trace = np.vstack((trace, final))
                    traces[(case.name, name)] = trace
    with (output / "metrics.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summaries = []
    for case in CASES:
        for name in ("baseline", "endpoint"):
            group = [row for row in rows if row["case"] == case.name and row["policy"] == name]
            summaries.append(dict(case=case.name, policy=name, passes=sum(row["pass"] for row in group),
                                  endpoint_stops=sum(row["endpoint_stop"] for row in group),
                                  min_stop_x=min(row["stop_x_m"] for row in group),
                                  max_stop_x=max(row["stop_x_m"] for row in group),
                                  min_clearance_m=min(row["min_cone_clearance_m"] for row in group),
                                  fallbacks=sum(row["fallbacks"] for row in group),
                                  worst_p99_ms=max(row["p99_update_ms"] for row in group)))
    report = dict(summaries=summaries, cases=[dataclasses.asdict(case) for case in CASES],
                  mpc=dataclasses.asdict(MpcConfig()), endpoint=dataclasses.asdict(EndpointConfig()),
                  seeds=list(range(300, 305)), python=platform.python_version(), platform=platform.platform(),
                  numpy=np.__version__, scipy=scipy.__version__,
                  source_sha256={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in [Path(__file__).resolve(), Path(__file__).with_name("evaluate_mpc.py"),
                                              *sorted((ROOT / "policy").rglob("*.py"))]})
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    os.environ.setdefault("MPLCONFIGDIR", str(output / ".matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 4, figsize=(16, 7), constrained_layout=True)
    for ax, case in zip(axes.flat, CASES):
        for name in ("baseline", "endpoint"):
            trace = traces[(case.name, name)]
            ax.plot(trace[:, 1], trace[:, 6], label=name)
            ax.plot(trace[-1, 1], trace[-1, 6], "x")
        ax.axvline(6, color="gray", linestyle="--")
        ax.set(title=case.name, xlabel="Distance along lane (m)", ylabel="Actual simulated speed (m/s)",
               xlim=(0, 6.6), ylim=(-0.03, 1.1))
        ax.legend(fontsize=8)
    axes.flat[-1].set_visible(False)
    fig.suptitle("Final-pair approach — seed 300; missing cones can cause a false finish")
    fig.savefig(output / "stopping-profiles.png", dpi=160)
    plt.close(fig)
    print(json.dumps(summaries, indent=2))


if __name__ == "__main__":
    main()
