"""Plan a synchronized joint-space move offline, using degrees and seconds."""
import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import sys

MAX_SAMPLES = 100_000


def checked(value, label, low, high):
    # bool is a subclass of int in Python; do not accept JSON true as 1.
    if type(value) not in (int, float) or not low <= value <= high:
        raise ValueError(f"{label} must be a number in [{low}, {high}]")
    return float(value)


@dataclass(frozen=True)
class Axis:
    name: str
    start: float
    goal: float
    speed: float
    accel: float

    @property
    def delta(self):
        return self.goal - self.start


@dataclass(frozen=True)
class Move:
    axes: tuple
    shape: str
    ramp: float
    cruise: float
    peak: float
    accel: float
    speed_axis: str | None
    accel_axis: str | None

    @property
    def duration(self):
        return 2 * self.ramp + self.cruise

    def progress(self, t):
        """Return path fraction and its nonnegative rate, holding at endpoints."""
        if not math.isfinite(t):
            raise ValueError("sample time must be finite")
        if t <= 0:
            return 0.0, 0.0
        if t >= self.duration:
            return 1.0, 0.0
        if t < self.ramp:
            return self.accel * t * t / 2, self.accel * t
        remaining = self.duration - t
        if remaining < self.ramp:
            return 1 - self.accel * remaining * remaining / 2, self.accel * remaining
        return self.peak * (t - self.ramp / 2), self.peak

    def sample(self, t):
        if not math.isfinite(t):
            raise ValueError("sample time must be finite")
        if self.shape == "hold" or t <= 0:
            return [(axis.start, 0.0) for axis in self.axes]
        if t >= self.duration:
            return [(axis.goal, 0.0) for axis in self.axes]
        fraction, rate = self.progress(t)
        return [(axis.start + axis.delta * fraction, axis.delta * rate) for axis in self.axes]


def plan(axes):
    moving = [axis for axis in axes if axis.delta != 0]
    if not moving:
        return Move(tuple(axes), "hold", 0, 0, 0, 0, None, None)
    v_axis = min(moving, key=lambda axis: axis.speed / abs(axis.delta))
    a_axis = min(moving, key=lambda axis: axis.accel / abs(axis.delta))
    velocity = v_axis.speed / abs(v_axis.delta)
    accel = a_axis.accel / abs(a_axis.delta)
    unconstrained_peak = math.sqrt(accel)
    if velocity >= unconstrained_peak:
        peak = unconstrained_peak
        cruise = 0.0
        shape = "triangle"
    else:
        peak = velocity
        cruise = max(0.0, 1 / peak - peak / accel)
        shape = "trapezoid" if cruise > 0 else "triangle"
    return Move(tuple(axes), shape, peak / accel, cruise, peak, accel, v_axis.name, a_axis.name)


def read_axes(path):
    obj = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(obj, dict) or not isinstance(obj.get("joints"), list):
        raise ValueError("expected a JSON object with a joints array")
    items = obj["joints"]
    if not 1 <= len(items) <= 12:
        raise ValueError("enter one to twelve joints")
    names = set()
    axes = []
    for index, item in enumerate(items, start=1):
        if not isinstance(item, dict):
            raise ValueError(f"joint {index} must be an object")
        name = item.get("name")
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,19}", name):
            raise ValueError(f"joint {index} needs a simple name, e.g. elbow_2")
        if name in names:
            raise ValueError(f"duplicate joint name: {name}")
        names.add(name)
        axis = Axis(
            name,
            checked(item.get("start_deg"), f"{name}.start_deg", -36000, 36000),
            checked(item.get("goal_deg"), f"{name}.goal_deg", -36000, 36000),
            checked(item.get("max_deg_s"), f"{name}.max_deg_s", 0.001, 100000),
            checked(item.get("max_deg_s2"), f"{name}.max_deg_s2", 0.001, 100000),
        )
        if 0 < abs(axis.delta) < 1e-6:
            raise ValueError(f"{name} travel must be zero or at least 0.000001 deg")
        axes.append(axis)
    return axes


def write_samples(path, move, dt, source):
    # Refuse to open any existing file, including a symlink. Avoid input clobber.
    if path.resolve() == source.resolve():
        raise ValueError("CSV must be a new path, different from the input")
    ticks = math.ceil(move.duration / dt)
    if ticks + 1 > MAX_SAMPLES:
        raise ValueError(f"CSV exceeds {MAX_SAMPLES:,} rows; increase --step")
    header = ["time_s"]
    for axis in move.axes:
        header.extend((f"{axis.name}_deg", f"{axis.name}_deg_s"))
    with path.open("x", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(header)
        def write_row(t):
            writer.writerow([t, *(value for pair in move.sample(t) for value in pair)])
        for i in range(ticks):
            t = i * dt
            if t < move.duration:
                write_row(t)
        write_row(move.duration)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", type=Path)
    parser.add_argument("--csv", type=Path, help="optional new output file")
    parser.add_argument("--step", type=float, default=0.05, help="sample interval in seconds")
    args = parser.parse_args(argv)
    try:
        dt = checked(args.step, "--step (s)", 0.0001, 3600)
        move = plan(read_axes(args.scenario))
        if args.csv:
            write_samples(args.csv, move, dt, args.scenario)
    except (ValueError, OSError, OverflowError) as exc:
        parser.exit(2, f"error: {exc}\n")
    print(f"{move.shape}: {move.duration:.4f} s total "
          f"({move.ramp:.4f} s ramp, {move.cruise:.4f} s cruise)")
    if move.speed_axis:
        print(f"Path speed cap: {move.speed_axis}; path acceleration cap: {move.accel_axis}")
    for axis in move.axes:
        peak = axis.delta * move.peak
        print(f"{axis.name}: {axis.start:g} -> {axis.goal:g} deg; signed peak {peak:.3f} deg/s")
    if args.csv:
        print(f"Wrote {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
