"""Makes the figures for Chris's logbook section (docs/logbook/chris/figures/).

    python3 docs/logbook/chris/make_figures.py      # needs matplotlib + numpy

Data sources (all in this folder or typed from the cited results):
- data/speed_points.csv: car 20 drive -> speed points, 5 Oct bags c20_*, from progress-assessment/plot_speed_map.py
- steering table: car 20, 5 Oct bags c20_steer + c20_steer4 (curvature = IMU yaw rate / wheel speed, last 3 s)
- sim_speed_integral.py: speed-controller simulation (figures 04, 05)
- two-box results: avoidance sim, 17-18 Oct (PR #23); spacing results: obstacle_simulation.py sweeps, 30 Oct
figures/03_car20_maps_pi.png is the 12 Oct Progress Assessment figure (real data, plot_speed_map.py on the c20_* bags).
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from sim_speed_integral import NEW, OLD, TRUE_OFFSETS, simulate, summary

HERE = Path(__file__).resolve().parent
FIG = HERE / "figures"
BLUE, GREY, DARK, RED, GREEN, AMBER = "#1F5FD1", "#9A9A9A", "#222222", "#C0392B", "#2A7F62", "#B26B00"
LIGHT = "#E8EFFC"
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})

# Car 20 steering map, 5 Oct (steer command -> curvature 1/m, + = left)
STEER = [(-1.0, 0.466), (-0.75, 0.46), (-0.5, 0.345), (-0.25, 0.15), (0.0, -0.04), (0.25, -0.203),
         (0.5, -0.445), (0.75, -0.67), (1.0, -0.93)]


def box(ax, x, y, w, h, title, lines=(), mine=False, fs=9.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                fc=LIGHT if mine else "white", ec=BLUE if mine else GREY, lw=2 if mine else 1.2))
    ax.text(x + w / 2, y + h - 0.22, title, ha="center", va="top", fontsize=fs, weight="bold", color=DARK)
    for i, line in enumerate(lines):
        ax.text(x + w / 2, y + h - 0.55 - 0.27 * i, line, ha="center", va="top", fontsize=fs - 1.5, color="#444")


def arrow(ax, x0, y0, x1, y1, label=None, dx=0.08, dy=0.0, ha="left"):
    ax.annotate("", (x1, y1), (x0, y0), arrowprops=dict(arrowstyle="-|>", color="#555", lw=1.2))
    if label:
        ax.text((x0 + x1) / 2 + dx, (y0 + y1) / 2 + dy, label, fontsize=8.5, color="#333", ha=ha, va="center")


def fig_system():
    fig, ax = plt.subplots(figsize=(11, 7.2))
    ax.set_xlim(0, 11.8), ax.set_ylim(0, 7.4), ax.axis("off")
    box(ax, 0.4, 6.3, 3.0, 0.8, "OAK-D cone detector", ["cones 10 Hz"])
    box(ax, 7.4, 6.3, 3.0, 0.8, "RPLidar C1", ["2D scan 10 Hz"])
    box(ax, 0.4, 4.9, 3.0, 1.0, "Lane estimation (Long)", ["cones -> centre line, width"])
    box(ax, 7.4, 4.75, 3.0, 1.15, "Obstacles + avoidance", ["Zhenling; my fixes: PR #23,", "lane-jump stop, spacing study"], mine=True)
    box(ax, 3.4, 3.5, 4.2, 1.0, "MPC path following (Peiyu, Toby, Yuhao)", ["curvature caps +0.48 / -0.92 1/m from my map"])
    box(ax, 1.0, 1.75, 9.0, 1.15, "Low-level car control (mine, with Yifei): policy/control/", [
        "speed: feedforward + PI on wheel speed      steering: per-side inverse map kappa -> steer"], mine=True, fs=10.5)
    box(ax, 1.0, 0.2, 4.2, 1.1, "DREAM / Traxxas interface", ["slew 10/s steer, 50/s drive; ~50 ms lag",
                                                             "steer clipped to +-0.5 after trim"])
    box(ax, 5.8, 0.2, 4.2, 1.1, "Car 20 (Traxxas Slash)", ["speed: tau 2.7 s, delay 0.2 s, starts at u 0.52",
                                                          "steer: R_min 1.1 m right, 2.1 m left"])
    arrow(ax, 1.9, 6.3, 1.9, 5.9), arrow(ax, 8.9, 6.3, 8.9, 5.9)
    arrow(ax, 3.4, 5.4, 7.4, 5.4, "lane frame", dx=-0.4, dy=0.15)
    arrow(ax, 1.9, 4.9, 4.2, 4.5), arrow(ax, 8.9, 4.75, 6.8, 4.5)
    ax.text(8.3, 4.45, "planned line, brake", fontsize=8.5, color="#333")
    arrow(ax, 5.5, 3.5, 5.5, 2.9, "v_ref (m/s), kappa_ref (1/m, + left), brake @ 10 Hz", dx=0.1)
    arrow(ax, 3.1, 1.75, 3.1, 1.3, "drive u [0, 0.75], steer s [-1, 1]", dx=0.1)
    arrow(ax, 5.2, 0.75, 5.8, 0.75)
    ax.annotate("", (10.0, 2.3), (10.0, 0.75), arrowprops=dict(arrowstyle="-|>", color="#555", lw=1.2,
                                                                connectionstyle="arc,angleA=0,angleB=0,armA=40,armB=40,rad=0"))
    ax.text(10.55, 1.5, "wheel speed\n(encoder)", fontsize=8.5, color="#333", ha="left")
    ax.set_title("Where my parts fit: blue = my work, grey = teammates' parts", loc="left", fontsize=12, weight="bold")
    fig.savefig(FIG / "01_system.png", dpi=150, bbox_inches="tight"), plt.close(fig)


def fig_car_control():
    fig, ax = plt.subplots(figsize=(11, 5.6))
    ax.set_xlim(0, 11), ax.set_ylim(0, 5.8), ax.axis("off")
    # speed path
    ax.text(0.2, 5.45, "Speed path (10 Hz)", fontsize=10.5, weight="bold", color=BLUE)
    box(ax, 0.2, 3.8, 1.6, 1.2, "v_ref", ["from MPC", "m/s"])
    box(ax, 2.4, 4.2, 2.3, 1.2, "Feedforward", ["u_ff = u0 + v_ref / K", "u0 0.48, K 9.18"], mine=True)
    box(ax, 2.4, 2.75, 2.3, 1.2, "PI on error e", ["kp e + I,  I += ki e dt", "integral rule (Sec. 2.3)"], mine=True)
    box(ax, 5.3, 3.5, 1.9, 1.2, "Clip", ["0 <= u <= 0.75", "anti-windup"], mine=True)
    box(ax, 7.8, 3.5, 1.5, 1.2, "Slew", ["50 /s", "~50 ms lag"])
    box(ax, 9.6, 3.5, 1.3, 1.2, "Motor", ["tau 2.7 s", "delay 0.2 s"])
    arrow(ax, 1.8, 4.5, 2.4, 4.8), arrow(ax, 1.8, 4.2, 2.4, 3.4)
    ax.text(1.05, 3.35, "e = v_ref - v", fontsize=8.5, color="#333")
    arrow(ax, 4.7, 4.8, 5.3, 4.2), arrow(ax, 4.7, 3.35, 5.3, 3.9)
    arrow(ax, 7.2, 4.1, 7.8, 4.1), arrow(ax, 9.3, 4.1, 9.6, 4.1)
    ax.plot([10.25, 10.25, 3.55], [3.5, 2.3, 2.3], color="#555", lw=1.2)
    arrow(ax, 3.55, 2.3, 3.55, 2.75)
    ax.text(6.8, 2.05, "wheel speed v (encoder, 10 Hz, unsigned, quantised; PI off below 0.4 m/s)", fontsize=8.5,
            ha="center", color="#333")
    # steering path
    ax.text(0.2, 1.75, "Steering path (10 Hz)", fontsize=10.5, weight="bold", color=BLUE)
    box(ax, 0.2, 0.2, 1.6, 1.2, "kappa_ref", ["from MPC", "1/m, + left"])
    box(ax, 2.4, 0.2, 2.9, 1.2, "Inverse map, per side", ["right: s = (0.038 - k)/0.959", "left: s = (0.008 - k)/0.630"],
        mine=True)
    box(ax, 5.8, 0.2, 1.4, 1.2, "Clamp", ["s >= -0.75", "s <= 1"], mine=True)
    box(ax, 7.8, 0.2, 1.5, 1.2, "Trim + clip", ["trim -0.18", "+-0.5 interval"])
    box(ax, 9.6, 0.2, 1.3, 1.2, "Servo", ["slew 10 /s", "left sat. 0.47"])
    for x0, x1 in ((1.8, 2.4), (5.3, 5.8), (7.2, 7.8), (9.3, 9.6)):
        arrow(ax, x0, 0.8, x1, 0.8)
    ax.set_title("Inside car control: blue = my code (policy/control/control.py), grey = platform and car",
                 loc="left", fontsize=12, weight="bold")
    fig.savefig(FIG / "02_car_control.png", dpi=150, bbox_inches="tight"), plt.close(fig)


def fig_ff_sweep():
    errs, old, new = [], [], []
    for o in TRUE_OFFSETS:
        errs.append(0.48 - o)
        old.append(summary(simulate(o, o + 0.045, 0.48, **OLD))["v_end"])
        new.append(summary(simulate(o, o + 0.045, 0.48, **NEW))["v_end"])
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.axvspan(-0.02, 0.03, color="#EEEEEE")
    ax.axhline(0.5, ls="--", color=GREY, lw=1)
    ax.plot(errs, old, "o-", color=GREY, lw=2, label="old integral (band, ki 0.04): dev / main")
    ax.plot(errs, new, "o-", color=BLUE, lw=2.5, label="26 Oct fix (ki 0.2, integrate above target / at rest)")
    ax.text(0.005, 1.0, "old works\nonly here", ha="center", fontsize=8.5, color="#555")
    ax.text(-0.12, 0.08, "FF too low:\ncar never starts", fontsize=8.5, color="#555")
    ax.text(0.27, 1.2, "FF too high:\nstuck overspeed", fontsize=8.5, color="#555", ha="right")
    ax.text(-0.125, 0.53, "target 0.5 m/s", fontsize=8.5, color="#555")
    ax.set(xlabel="feedforward offset error: set u0 minus true u0 (drive units)", ylabel="speed after 20 s (m/s)",
           ylim=(-0.05, 1.7))
    ax.legend(loc="lower right", frameon=False, fontsize=8.5)
    ax.set_title("Simulation: the old integral only reaches 0.5 m/s if the feedforward is within ~0.03",
                 loc="left", fontsize=11, weight="bold")
    fig.savefig(FIG / "04_ff_error_sweep.png", dpi=150, bbox_inches="tight"), plt.close(fig)


def fig_fix_timeseries():
    cases = [("A  FF correct (u0 0.475)", 0.475, 0.52), ("B  FF too high (true u0 0.30)", 0.30, 0.345),
             ("C  breakaway 0.68 (like 26 Oct)", 0.50, 0.68)]
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6), sharey=True)
    for ax, (name, o, ub) in zip(axes, cases):
        for cfg, colour, label in ((OLD, GREY, "old"), (NEW, BLUE, "26 Oct fix")):
            out = simulate(o, ub, 0.48, T=16.0, **cfg)
            ax.plot([x[0] for x in out], [x[1] for x in out], color=colour, lw=2, label=label)
        ax.axhline(0.5, ls="--", color=GREY, lw=1)
        ax.set(title=name, xlabel="time (s)")
    axes[0].set_ylabel("wheel speed (m/s)"), axes[0].legend(frameon=False)
    fig.suptitle("Simulation, step to 0.5 m/s: the fix removes both failures but overshoots more when FF is right "
                 "(peak 0.62 vs 0.53)", x=0.01, ha="left", fontsize=11, weight="bold")
    fig.tight_layout()
    fig.savefig(FIG / "05_fix_timeseries.png", dpi=150, bbox_inches="tight"), plt.close(fig)


def fig_two_boxes():
    spacing = [0.8, 1.2, 1.6, 2.0, 2.5]
    before = ["pass", "pass", "stop", "stop", "pass"]
    after = ["pass"] * 5
    fig, ax = plt.subplots(figsize=(7.5, 2.2))
    for row, (name, res) in enumerate((("before (dev 6124c1c)", before), ("after PR #23", after))):
        for i, r in enumerate(res):
            ax.add_patch(plt.Rectangle((i, 1 - row), 0.92, 0.85, color=GREEN if r == "pass" else RED, alpha=0.75))
            ax.text(i + 0.46, 1.42 - row, r, ha="center", va="center", color="white", weight="bold")
        ax.text(-0.1, 1.42 - row, name, ha="right", va="center")
    ax.set_xlim(-2.4, 5), ax.set_ylim(-0.1, 2.1), ax.axis("off")
    for i, s in enumerate(spacing):
        ax.text(i + 0.46, -0.05, f"{s} m", ha="center", va="top")
    ax.text(2.3, -0.42, "spacing between the two boards (1.0 m lane, each board 0.2 m into the lane)", ha="center",
            fontsize=9, color="#444")
    ax.set_title("Simulation: the 15 Oct video's stop beside board 1 reproduced, then fixed", loc="left",
                 fontsize=11, weight="bold")
    fig.savefig(FIG / "06_two_boxes.png", dpi=150, bbox_inches="tight"), plt.close(fig)


def fig_spacing():
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 3.8))
    shift, gap = [0.25, 0.35, 0.45, 0.55], [1.1, 1.35, 2.0, 2.1]
    a.plot(shift, gap, "o-", color=BLUE, lw=2)
    for s, g in zip(shift, gap):
        a.text(s, g + 0.08, f"{g} m", ha="center", fontsize=9)
    a.set(xlabel="lateral shift between the two passing lines (m)", ylabel="smallest gap the planner passes (m)",
          ylim=(0, 2.6), title="(a) Planner only, curve cap 0.45 1/m")
    names = ["straight,\nshift 0.55 m", "left bend R 3 m"]
    cur, proto = [2.25, 3.0], [2.0, 2.5]
    x = range(len(names))
    a2 = b.bar([i - 0.18 for i in x], cur, 0.36, color=GREY, label="current planner (dev)")
    b.bar([i + 0.18 for i in x], proto, 0.36, color=BLUE, label="prototype: each half capped by its own side")
    b.text(1 - 0.18, 3.05, "> 3", ha="center", fontsize=9)
    b.text(0 - 0.18, 2.3, "2.25", ha="center", fontsize=9), b.text(0.18, 2.05, "2.0", ha="center", fontsize=9)
    b.text(1.18, 2.55, "2.5", ha="center", fontsize=9)
    b.set_xticks(list(x), names), b.set(ylabel="smallest reliable gap (m)", ylim=(0, 4.3),
                                        title="(b) Closed loop, 6-8 seeds per point")
    b.legend(frameon=False, fontsize=8.5, loc="upper left")
    fig.suptitle("Simulation, 30 Oct: the planner's curve cap, not the steering map, sets how close two obstacles "
                 "can be", x=0.01, ha="left", fontsize=11, weight="bold")
    fig.tight_layout()
    fig.savefig(FIG / "07_spacing.png", dpi=150, bbox_inches="tight"), plt.close(fig)
    del a2


def slide_figs():
    rows = list(csv.DictReader(open(HERE / "data" / "speed_points.csv")))
    u = [float(r["drive_cmd"]) for r in rows]
    v = [float(r["v_ss_m_per_s"]) for r in rows]
    fig, ax = plt.subplots(figsize=(4.4, 2.3))
    ax.axvspan(0.45, 0.52, color="#F2E6D8")
    ax.scatter([a for a, b in zip(u, v) if b > 0], [b for b in v if b > 0], color=BLUE, s=18, zorder=3)
    ax.scatter([a for a, b in zip(u, v) if b == 0], [0] * v.count(0.0), color=RED, marker="x", s=25, zorder=3)
    ax.plot([0.475, 0.9], [0, 9.18 * (0.9 - 0.475)], color=DARK, lw=1.3)
    ax.text(0.6, 3.3, "v = 9.18 (u - 0.475)", fontsize=8)
    ax.set(xlabel="drive command", ylabel="speed (m/s)", title="Car 20, 5 Oct, real data", xlim=(0.45, 0.9))
    ax.title.set_fontsize(9)
    fig.savefig(FIG / "slide_sysid.png", dpi=200, bbox_inches="tight"), plt.close(fig)

    fig, ax = plt.subplots(figsize=(4.4, 2.3))
    ax.axhline(0, color="#ccc", lw=0.8), ax.axvline(0, color="#ccc", lw=0.8)
    ax.scatter([s for s, _ in STEER], [k for _, k in STEER], color=BLUE, s=18, zorder=3)
    ax.plot([0, 1], [0.038, -0.921], color=AMBER, lw=1.5, label="right: k = -0.96 s")
    ax.plot([-0.75, 0], [0.48, 0.008], color=GREEN, lw=1.5, label="left: k = -0.63 s")
    ax.plot([-1, -0.75], [0.47, 0.47], color=GREEN, lw=1.5, ls=":")
    ax.legend(frameon=False, fontsize=7)
    ax.set(xlabel="steer command", ylabel="curvature (1/m)", title="Inverse map per side (car 20)")
    ax.title.set_fontsize(9)
    fig.savefig(FIG / "slide_steer.png", dpi=200, bbox_inches="tight"), plt.close(fig)

    fig, ax = plt.subplots(figsize=(4.4, 2.3))
    for cfg, colour, label in ((OLD, GREY, "old: stuck 1.2 m/s"), (NEW, BLUE, "fix: 0.5 m/s in 7 s")):
        out = simulate(0.30, 0.345, 0.48, T=16.0, **cfg)
        ax.plot([x[0] for x in out], [x[1] for x in out], color=colour, lw=1.8, label=label)
    ax.axhline(0.5, ls="--", color=GREY, lw=1)
    ax.legend(frameon=False, fontsize=7)
    ax.set(xlabel="time (s)", ylabel="speed (m/s)", title="Sim: feedforward 0.18 too high")
    ax.title.set_fontsize(9)
    fig.savefig(FIG / "slide_fix.png", dpi=200, bbox_inches="tight"), plt.close(fig)

    fig, ax = plt.subplots(figsize=(4.4, 2.3))
    spacing = [0.8, 1.2, 1.6, 2.0, 2.5]
    before = [1, 1, 0, 0, 1]
    for i, ok in enumerate(before):
        ax.add_patch(plt.Rectangle((i, 1), 0.9, 0.8, color=GREEN if ok else RED, alpha=0.75))
        ax.add_patch(plt.Rectangle((i, 0), 0.9, 0.8, color=GREEN, alpha=0.75))
        ax.text(i + 0.45, -0.25, f"{spacing[i]}", ha="center", va="top", fontsize=8)
    ax.text(-0.1, 1.4, "before", ha="right", va="center", fontsize=8), ax.text(-0.1, 0.4, "PR #23", ha="right",
                                                                                   va="center", fontsize=8)
    ax.text(2.25, -0.75, "board spacing (m); green pass, red stop", ha="center", fontsize=7.5, color="#444")
    ax.set_xlim(-1.3, 5), ax.set_ylim(-1.0, 2.1), ax.axis("off")
    ax.set_title("Two boards, 1 m lane (sim)", fontsize=9)
    fig.savefig(FIG / "slide_avoid.png", dpi=200, bbox_inches="tight"), plt.close(fig)


if __name__ == "__main__":
    FIG.mkdir(exist_ok=True)
    fig_system(), fig_car_control(), fig_ff_sweep(), fig_fix_timeseries(), fig_two_boxes(), fig_spacing()
    slide_figs()
    print("figures written to", FIG)
