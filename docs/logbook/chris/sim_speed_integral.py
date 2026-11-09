"""Car 20 longitudinal model + speed controller, old vs new integral logic (Chris, logbook Section 3.2).

Plant (5 Oct sysID, car 20): first order plus dead time, v_ss = K (u - u0), K = 9.18 m/s, tau = 2.7 s, 0.2 s delay,
plus a breakaway threshold: the car stays still until the applied drive exceeds u_break. 10 Hz, like the policy.
Controller: a line-for-line copy of SpeedController in policy/control/control.py (FF + PI, anti-windup), with the
integral rule before (`new=False`, dev/main) and after the 26 Oct fix (`new=True`, branch fix/speed-overspeed-integral).
No dependencies.  python3 docs/logbook/chris/sim_speed_integral.py   prints the sweep used for figures 04 and 05.
"""
DT, K, TAU, DELAY_STEPS = 0.1, 9.18, 2.7, 2
OLD = dict(new=False, ki=0.04, imax=0.2)   # dev / main defaults
NEW = dict(new=True, ki=0.2, imax=0.3)     # fix/speed-overspeed-integral


def simulate(o_true, u_break, ff_offset, new, ki, imax, kp=0.15, band=0.2, vref=0.5, T=20.0, umax=0.75):
    """Returns [(t, wheel speed, drive)] for a step from rest to vref."""
    v, integral, buf, out = 0.0, 0.0, [0.0] * DELAY_STEPS, []
    for k in range(int(T / DT)):
        e = vref - v
        ff = ff_offset + vref / K
        unclipped = ff + kp * e + integral
        pushing_high = unclipped >= umax and e > 0
        pushing_low = unclipped <= 0 and e < 0
        in_band = (e < band or v < 0.05) if new else abs(e) < band
        if k and in_band and not (pushing_high or pushing_low):
            integral = max(-imax, min(imax, integral + ki * e * DT))
        u = max(0.0, min(umax, ff + kp * e + integral))
        buf.append(u)
        applied = buf.pop(0)
        v_ss = 0.0 if (v < 0.05 and applied < u_break) else max(0.0, K * (applied - o_true))
        v += (v_ss - v) * DT / TAU
        out.append((k * DT, v, u))
    return out


def summary(out, vref=0.5):
    vs = [x[1] for x in out]
    settle = next((out[i][0] for i in range(len(vs)) if all(abs(x - vref) < 0.05 for x in vs[i:])), None)
    return dict(peak=max(vs), v_end=vs[-1], settle_s=settle)


TRUE_OFFSETS = [0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.475, 0.50, 0.55, 0.60]

if __name__ == "__main__":
    print("FF error (0.48 - true u0) | old: peak, v after 20 s, settle | new: peak, v after 20 s, settle")
    for o in TRUE_OFFSETS:
        row = [summary(simulate(o, o + 0.045, 0.48, **c)) for c in (OLD, NEW)]
        print(f"{0.48 - o:+.3f}", *[f"{r['peak']:.2f} {r['v_end']:.2f} {r['settle_s']}" for r in row], sep=" | ")
