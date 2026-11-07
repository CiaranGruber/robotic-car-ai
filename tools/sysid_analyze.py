#!/usr/bin/env python3
"""Summarise a system-ID sequencer rosbag (.mcap) without ROS or extra packages.

Usage:  python tools/sysid_analyze.py sysid_run/sysid_run_0.mcap [more.mcap ...]

For every hold segment (debug1 >= 0) it prints the commanded and applied drive,
steering, the steady-state wheel speed (mean of the last 3 s), time to first
movement, time to 63 % of steady state, and the mean path curvature from
debug2. It then fits v_ss = K * (drive - u0) over segments that moved.
Only uncompressed MCAP (the rosbag2 default here) is supported.
"""
import collections
import struct
import sys


def _records(data, offset, end):
    while offset < end:
        opcode = data[offset]
        length, = struct.unpack_from("<Q", data, offset + 1)
        yield opcode, data[offset + 9:offset + 9 + length]
        offset += 9 + length


def read_mcap(path):
    data = open(path, "rb").read()
    if data[:8] != b"\x89MCAP0\r\n":
        raise ValueError(f"{path} is not an MCAP file")
    channels, messages = {}, collections.defaultdict(list)

    def handle(opcode, body):
        if opcode == 0x04:  # channel
            channel_id, = struct.unpack_from("<H", body, 0)
            length, = struct.unpack_from("<I", body, 4)
            channels[channel_id] = body[8:8 + length].decode()
        elif opcode == 0x05:  # message
            channel_id, _, log_time, _ = struct.unpack_from("<HIQQ", body, 0)
            messages[channels.get(channel_id)].append((log_time / 1e9, body[22:]))
        elif opcode == 0x06:  # chunk
            offset = 28
            length, = struct.unpack_from("<I", body, offset)
            compression = body[offset + 4:offset + 4 + length].decode()
            offset += 4 + length
            if compression:
                raise ValueError(f"compressed chunks ({compression}) are not supported")
            size, = struct.unpack_from("<Q", body, offset)
            inner = body[offset + 8:offset + 8 + size]
            for inner_opcode, inner_body in _records(inner, 0, len(inner)):
                handle(inner_opcode, inner_body)

    for opcode, body in _records(data, 8, len(data) - 8):
        handle(opcode, body)
    return messages


def float32(messages, topic):
    return [(t, struct.unpack_from("<f", raw, 4)[0]) for t, raw in messages.get(topic, [])]


def drive_and_steer(messages, topic):
    # CDR: 4-byte header, uint8 units padded to 4, float32 drive, float32 steer.
    return [(t, *struct.unpack_from("<ff", raw, 8)) for t, raw in messages.get(topic, [])]


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def segments(phase):
    """Return (index, start, end) for each contiguous debug1 value >= 0."""
    result, current = [], None
    for t, value in phase:
        if current and value == current[0]:
            current[2] = t
            continue
        if current and current[0] >= 0:
            result.append(tuple(current))
        current = [value, t, t]
    if current and current[0] >= 0:
        result.append(tuple(current))
    return result


def analyse(path):
    messages = read_mcap(path)
    speed = float32(messages, "/car/wheel_speed_m_per_sec")
    curvature = float32(messages, "/car/debug2")
    command = drive_and_steer(messages, "/car/drive_and_steer_set_point_normalized")
    applied = drive_and_steer(messages, "/car/drive_and_steer_applied_normalized")
    rows = []
    for index, start, end in segments(float32(messages, "/car/debug1")):
        inside = lambda series, a=start, b=end: [s for s in series if a <= s[0] <= b]
        v = inside(speed)
        tail = [x for t, x in v if t >= end - 3.0]
        v_ss = mean(tail)
        moved = next((t - start for t, x in v if x > 0.0), float("nan"))
        t63 = next((t - start for t, x in v if v_ss > 0 and x >= 0.632 * v_ss), float("nan"))
        rows.append({
            "seg": int(index), "hold": end - start,
            "cmd": mean([d for _, d, _ in inside(command)]),
            "applied": mean([d for t, d, _ in inside(applied) if t > start + 0.3]),
            "steer": mean([s for _, _, s in inside(command)]),
            "v_ss": v_ss, "moved": moved, "t63": t63,
            "kappa": mean([k for t, k in inside(curvature) if t >= end - 3.0]),
        })
    return rows


def fit(rows):
    points = [(r["cmd"], r["v_ss"]) for r in rows if r["v_ss"] > 0.05]
    if len(points) < 2:
        return None
    mx, my = mean([p[0] for p in points]), mean([p[1] for p in points])
    sxx = sum((x - mx) ** 2 for x, _ in points)
    if sxx == 0.0:
        return None
    gain = sum((x - mx) * (y - my) for x, y in points) / sxx
    return gain, mx - my / gain, len(points)


def main(paths):
    all_rows = []
    for path in paths:
        rows = analyse(path)
        all_rows += rows
        print(f"\n{path}")
        print(" seg  hold  cmd    applied steer  v_ss   moved  t63    kappa")
        for r in rows:
            print(f" {r['seg']:3d} {r['hold']:5.1f}  {r['cmd']:.3f}  {r['applied']:.3f}  "
                  f"{r['steer']:+.2f}  {r['v_ss']:.3f}  {r['moved']:5.2f}  {r['t63']:5.2f}  "
                  f"{r['kappa']:+.3f}")
    result = fit(all_rows)
    if result:
        gain, offset, n = result
        print(f"\nFit over {n} moving segments: v_ss = {gain:.2f} * (drive - {offset:.3f}) m/s")
        print(f"Feedforward: drive = {offset:.3f} + v_ref / {gain:.2f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
