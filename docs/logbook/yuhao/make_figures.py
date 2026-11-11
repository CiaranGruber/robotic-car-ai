"""Draw the system diagram and redraw Figures 7–8 from committed CSV extracts.

Run from any directory with Python 3 and matplotlib:
    python3 docs/logbook/yuhao/make_figures.py

No policy, simulator, vehicle service or network connection runs here.
Figures 3–6 are preserved historical plots; see data/sources.json for origins.
"""

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data"
FIGURES = ROOT / "figures"


def rows(name):
    with (DATA / name).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def system_diagram():
    fig, ax = plt.subplots(figsize=(15, 10.5), layout="constrained")
    ax.set(xlim=(0, 12.2), ylim=(-0.8, 10.3))
    ax.axis("off")
    nodes = {
        "s": (4.8, 9.6, 8.6, 0.8, "DREAM sensor services and acquisition-time transforms", False),
        "o": (4.8, 8.2, 8.6, 0.9, "Cones and posed markers (m), lidar (m)\nWheel speed (m/s), IMU heading and yaw motion (rad, rad/s)", False),
        "l": (1.5, 6.5, 2.6, 1.1, "Teammates\nLane and obstacle\nestimates", False),
        "m": (4.5, 6.5, 2.6, 1.1, "My contribution\nMeasured-speed prediction\nMotion-compensated memory", True),
        "c": (7.5, 6.5, 2.6, 1.1, "Shared marker pipeline\nReader and grouped\ndirection IDs", True),
        "x": (10.5, 6.5, 2.7, 1.1, "Offline alternatives\nFinal-pair stopping\nConnected U-turn", True),
        "p": (3.0, 4.5, 4.0, 1.1, "MPC lane following\nor an experimental selected route", True),
        "t": (7.3, 4.5, 4.0, 1.1, "Default marker turner\nPrepare, arc, exit", True),
        "g": (5.2, 2.7, 8.5, 0.9, "Shared guards and my stop-persistence changes\nValidity, obstacles and explicit new run after a latch", True),
        "a": (5.2, 1.3, 8.5, 0.75, "Outputs: speed (m/s), curvature (1/m), brake request", False),
        "v": (5.2, -0.1, 8.5, 0.9, "Chris: CarController maps drive and steering actions\nDREAM vehicle interface owns explicit vehicle enable", False),
    }
    for name, (x, y, w, h, label, mine) in nodes.items():
        ax.add_patch(Rectangle((x-w/2, y-h/2), w, h,
                              facecolor="#fff3d6" if name == "x" else ("#e3f0fb" if mine else "#eef0f2"),
                              edgecolor="#7d6531" if name == "x" else "#344b5f", linewidth=1.3,
                              linestyle="--" if name == "x" else "-"))
        ax.text(x, y, label, ha="center", va="center", fontsize=11 if w < 3 else 13,
                color="#142b41", linespacing=1.4)
    for start, end in [("s", "o"), ("o", "l"), ("o", "m"), ("o", "c"),
                       ("o", "x"), ("l", "p"), ("l", "t"), ("m", "p"),
                       ("c", "t"), ("p", "g"), ("t", "g"), ("g", "a"), ("a", "v")]:
        sx, sy, sw, sh, *_ = nodes[start]
        tx, ty, tw, th, *_ = nodes[end]
        ax.add_patch(FancyArrowPatch((sx, sy-sh/2-0.03), (tx, ty+th/2+0.03),
                                     arrowstyle="-|>", mutation_scale=15,
                                     linewidth=1.15, color="#455e72",
                                     linestyle="--" if end == "x" else "-"))
    ax.text(10.55, 8.55, "Simulation\ninterfaces", ha="center", va="center", fontsize=11, color="#7d6531")
    fig.savefig(FIGURES / "00_system.png", dpi=160)
    plt.close(fig)


def main():
    FIGURES.mkdir(exist_ok=True)
    plt.rcParams.update({"font.size": 12, "axes.spines.top": False,
                         "axes.spines.right": False, "savefig.facecolor": "white"})
    system_diagram()
    marker = rows("marker-range.csv")
    fig, ax = plt.subplots(figsize=(11, 4.7), layout="constrained")
    for run, colour in [("2026-10-30_14-46-15", "#00695e"),
                        ("2026-10-30_14-50-00", "#c05b24")]:
        subset = [r for r in marker if r["run"] == run]
        ax.plot([float(r["elapsed_s"]) for r in subset],
                [float(r["distance_m"]) for r in subset],
                "o-", color=colour, markersize=5,
                label=f"Run {run[-8:].replace('-', ':')} ({len(subset)} observations)")
    ax.axhline(2.0, color="#142b41", linestyle="--", label="Policy range gate: 2.0 m")
    ax.set(xlabel="Time from each run's own start (s)",
           ylabel="Reported planar marker range (m)", ylim=(0, 7),
           title="Car 20, 30 Oct: all 41 posed ID-1 observations exceed the gate")
    ax.legend(loc="lower right", frameon=False)
    ax.grid(alpha=0.18)
    fig.savefig(FIGURES / "05_marker_range.png", dpi=160)
    plt.close(fig)

    trace = [r for r in rows("post-reload-trace.csv") if float(r["elapsed_s"]) <= 10]
    events = json.loads((DATA / "post-reload-events.json").read_text())
    main_run = next(r for r in events if r["run"].endswith("15-02-43"))
    confirm = next(e["elapsed_s"] for e in main_run["transitions"] if e["index"] == 32)
    stop = next(e["elapsed_s"] for e in main_run["transitions"] if e["index"] == 63)
    times = [float(r["elapsed_s"]) for r in trace]
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.4), sharex=True,
                             layout="constrained")
    axes[0].plot(times, [float(r["wheel_speed_m_per_s"]) for r in trace],
                 color="#00695e", label="Recorded wheel-speed feedback")
    axes[0].set(ylabel="Wheel speed (m/s)", ylim=(0, 0.75),
                title="Car 20, 15:02:43 after detector reload: recognition, then a stop decision")
    axes[0].legend(loc="lower right", frameon=False)
    axes[1].step(times, [float(r["drive_action"]) for r in trace],
                 where="post", color="#2458a6", label="Drive action")
    axes[1].step(times, [float(r["steering_action"]) for r in trace],
                 where="post", color="#c05b24", label="Steering action (includes centre trim)")
    axes[1].axhline(0, color="grey", linewidth=0.7)
    axes[1].set(xlabel="Time after run start (s)", ylabel="Normalised action (unitless)")
    axes[1].legend(loc="lower right", frameon=False)
    for ax in axes:
        ax.axvline(confirm, color="#656565", linestyle=":")
        ax.axvline(stop, color="#a93232", linestyle="--")
        ax.axvspan(stop, 10, color="#a93232", alpha=0.055)
        ax.grid(alpha=0.18)
        ax.set_xlim(0, 10)
    axes[0].text(confirm + 0.10, 0.96, f"ID 1 confirmed\n{confirm:.3f} s",
                 transform=axes[0].get_xaxis_transform(), va="top", fontsize=10)
    axes[0].text(stop + 0.12, 0.96, f"Geometry stop latched\n{stop:.3f} s",
                 transform=axes[0].get_xaxis_transform(), va="top", fontsize=10,
                 color="#a93232")
    fig.savefig(FIGURES / "06_post_reload_trace.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
