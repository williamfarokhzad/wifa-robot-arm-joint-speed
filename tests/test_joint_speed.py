import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from joint_speed import Axis, plan, read_axes

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    return plan(read_axes(ROOT / "examples" / f"{name}.json"))


class PlannerTests(unittest.TestCase):
    def test_two_different_axes_set_normalized_caps(self):
        m = load("two-caps")
        self.assertEqual((m.shape, m.speed_axis, m.accel_axis),
                         ("trapezoid", "elbow", "shoulder"))
        self.assertAlmostEqual(m.duration, 11/6)
        self.assertAlmostEqual(m.cruise, 7/6)
        for (q, v), (target_q, target_v) in zip(m.sample(11/12), [(45, 60), (-22.5, -30)]):
            self.assertAlmostEqual(q, target_q)
            self.assertAlmostEqual(v, target_v)

    def test_triangle_follows_hand_calculated_ramp(self):
        m = load("short-move")
        self.assertEqual(m.shape, "triangle")
        self.assertAlmostEqual(m.duration, 1)
        # At 0.25 s, shoulder advances 0.5 * 48 * 0.25**2 = 1.5 degrees.
        self.assertEqual(m.sample(.25), [(11.5, 12), (4.25, -6), (15, 0)])
        self.assertEqual(m.sample(.75), [(20.5, 12), (-.25, -6), (15, 0)])

    def test_exact_speed_boundary_has_no_plateau(self):
        m = load("at-boundary")
        self.assertEqual((m.shape, m.duration, m.cruise), ("triangle", 2, 0))
        self.assertEqual(m.sample(1), [(15, 30), (7.5, 15)])

    def test_all_stationary_has_one_rest_state(self):
        m = load("hold")
        self.assertEqual((m.shape, m.duration), ("hold", 0))
        self.assertEqual(m.sample(-5), [(20, 0)])
        self.assertEqual(m.sample(5), [(20, 0)])

    def test_signed_integrated_velocity_recovers_distance(self):
        for name, targets in [("two-caps", [90, -45]), ("short-move", [12, -6, 0])]:
            with self.subTest(name=name):
                m = load(name)
                # Composite trapezoid integration, independent of its position sampler.
                steps = 3000
                dt = m.duration / steps
                areas = [0.0] * len(targets)
                prev = m.sample(0)
                for j in range(1, steps + 1):
                    point = m.sample(j * dt)
                    for i in range(len(targets)):
                        areas[i] += (prev[i][1] + point[i][1]) * dt / 2
                    prev = point
                for area, target in zip(areas, targets):
                    self.assertAlmostEqual(area, target, delta=2e-5)

    def test_trace_holds_the_line_and_respects_axis_bounds(self):
        for name in ("two-caps", "short-move", "at-boundary"):
            with self.subTest(name=name):
                m = load(name)
                dt = m.duration / 113
                prev = m.sample(0)
                self.assertEqual(prev, [(a.start, 0) for a in m.axes])
                for j in range(1, 114):
                    trace = m.sample(j * dt)
                    fractions = []
                    for a, (q, v), (_, last_v) in zip(m.axes, trace, prev):
                        self.assertLessEqual(abs(v), a.speed + 1e-9)
                        self.assertLessEqual(abs(v - last_v) / dt, a.accel + 1e-8)
                        if a.delta:
                            self.assertGreaterEqual(v * a.delta, 0)
                            fractions.append((q - a.start) / a.delta)
                        else:
                            self.assertEqual((q, v), (a.start, 0))
                    self.assertAlmostEqual(max(fractions), min(fractions))
                    prev = trace
                self.assertEqual(m.sample(m.duration), [(a.goal, 0) for a in m.axes])

    def test_simultaneous_finish_alone_does_not_preserve_line(self):
        # Stretch independently planned profiles to the same duration.
        # They finish together but generally have different path fractions.
        shoulder = Axis("shoulder", 0, 90, 90, 180)
        elbow = Axis("elbow", 0, -45, 30, 45)
        common = plan([shoulder, elbow])
        solo_shoulder, solo_elbow = plan([shoulder]), plan([elbow])
        final = max(solo_shoulder.duration, solo_elbow.duration)
        slow_shoulder = lambda t: solo_shoulder.sample(t * (solo_shoulder.duration / final))[0][0]
        slow_elbow = lambda t: solo_elbow.sample(t * (solo_elbow.duration / final))[0][0]
        self.assertEqual((slow_shoulder(final), slow_elbow(final)), (90, -45))
        self.assertGreater(abs(slow_shoulder(.4)/90 - slow_elbow(.4)/-45), .001)
        self.assertAlmostEqual(common.sample(.4)[0][0]/90, common.sample(.4)[1][0]/-45)

    def test_rejects_nonfinite_time(self):
        with self.assertRaises(ValueError):
            load("short-move").sample(math.inf)


class InputOutputTests(unittest.TestCase):
    def cli(self, *args):
        return subprocess.run([sys.executable, str(ROOT / "joint_speed.py"), *map(str, args)],
                              capture_output=True, text=True)

    def test_schema_errors(self):
        item = dict(name="arm", start_deg=0, goal_deg=20, max_deg_s=60, max_deg_s2=120)
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "input.json"
            for joints, fragment in [
                ([item, item], "duplicate"), ([dict(item, max_deg_s=True)], "max_deg_s"),
                ([dict(item, max_deg_s2=0)], "max_deg_s2"), ([dict(item, goal_deg=float("nan"))], "goal_deg"),
                ([dict(item, goal_deg=1e-9)], "travel"), ([], "one to twelve"),
            ]:
                with self.subTest(fragment=fragment):
                    f.write_text(json.dumps({"joints": joints}))
                    with self.assertRaisesRegex(ValueError, fragment):
                        read_axes(f)

    def test_csv_includes_exact_finish_when_step_does_not_divide_time(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "trace.csv"
            result = self.cli(ROOT / "examples/two-caps.json", "--csv", out, "--step", ".25")
            self.assertEqual(result.returncode, 0, result.stderr)
            with out.open(newline="") as handle:
                rows = list(csv.DictReader(handle))
            times = [float(row["time_s"]) for row in rows]
            self.assertEqual(len(rows), 9)
            self.assertTrue(all(x < y for x, y in zip(times, times[1:])))
            self.assertAlmostEqual(times[-1], 11/6)
            self.assertEqual([float(rows[-1][k]) for k in ("shoulder_deg", "shoulder_deg_s", "elbow_deg", "elbow_deg_s")],
                             [90, 0, -45, 0])

    def test_csv_hold_and_near_multiple_use_unique_timestamps(self):
        with tempfile.TemporaryDirectory() as d:
            for name, step in [("hold", ".1"), ("at-boundary", ".19999999999999998")]:
                with self.subTest(name=name):
                    out = Path(d) / (name + ".csv")
                    result = self.cli(ROOT / "examples" / (name + ".json"), "--csv", out, "--step", step)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    with out.open(newline="") as handle:
                        times = [float(row["time_s"]) for row in csv.DictReader(handle)]
                    self.assertTrue(all(x < y for x, y in zip(times, times[1:])))
                    self.assertEqual(times[-1], load(name).duration)
                    if name == "hold":
                        self.assertEqual(times, [0])

    def test_csv_existing_file_stays_intact_and_sample_cap_prevents_creation(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "trace.csv"
            out.write_text("preserve\n")
            example = ROOT / "examples/two-caps.json"
            result = self.cli(example, "--csv", out)
            self.assertEqual(result.returncode, 2)
            self.assertIn("File exists", result.stderr)
            self.assertEqual(out.read_text(), "preserve\n")
            slow = Path(d) / "slow.json"
            slow.write_text(json.dumps({"joints": [dict(name="arm", start_deg=0,
                                                       goal_deg=100, max_deg_s=1, max_deg_s2=1)]}))
            new = Path(d) / "new.csv"
            result = self.cli(slow, "--csv", new, "--step", ".0001")
            self.assertEqual(result.returncode, 2)
            self.assertIn("100,000", result.stderr)
            self.assertFalse(new.exists())
