# Robot Arm Joint Speed: Plan a Synchronized Move

Plan a motion for two rotary joints so they leave and reach their endpoints together and follow a straight line in joint coordinates. The planner calculates a shared progress curve from the joint travel, speed limits and acceleration limits. It can save angle and signed speed as CSV.

The [full WIFA Robotics lesson](https://wifarobotics.com/learn/robot-arm-joint-speed/) explains the model and works through the numbers. This project is an offline kinematic exercise. It has no controller connection or physical robot data.

## Start from an empty computer

1. Install [Python 3.11 or newer](https://www.python.org/downloads/). Check it in Terminal with `python3 --version`; in Windows PowerShell use `py -3 --version`.
2. In this repository, choose **Code → Download ZIP** and extract it. Open Terminal or PowerShell in the extracted folder. Use `ls` or `dir` to confirm that `joint_speed.py` and `examples` are present.
3. Run these commands from that folder. Replace `python3` with `py -3` on Windows. There is nothing to install with pip.

```text
python3 joint_speed.py examples/two-caps.json --csv trace.csv --step 0.25
python3 joint_speed.py examples/short-move.json
python3 joint_speed.py examples/at-boundary.json
python3 joint_speed.py examples/hold.json
python3 -m unittest discover -s tests -v
```

| Scenario | Profile | Time (s) | Signed peaks in file order (deg/s) |
| --- | --- | ---: | --- |
| two-caps | Trapezoid | 1.8333 | +60, −30 |
| short-move | Triangle | 1.0000 | +24, −12, 0 |
| at-boundary | Triangle touching the speed cap | 2.0000 | +30, +15 |
| hold | Hold | 0.0000 | 0 |

Open `trace.csv` with a text editor or spreadsheet. It contains nine data rows. The last row is at about 1.83333333333333 seconds, with angles 90 and −45 degrees and both speeds zero. To plot it, use time as the x coordinate and the signed speed columns as y coordinates. The last interval is shorter than 0.25 seconds; keep the time column when plotting.

## Set up your own move

Copy a JSON scenario and edit the joint objects. `start_deg` and `goal_deg` are angles of the output joint, `max_deg_s` is a positive speed magnitude and `max_deg_s2` is a positive acceleration magnitude. Set positive directions consistently. The planner takes the difference literally: 350° to 10° means −340°, with no automatic wraparound. It accepts one to twelve axes, angles from −36,000 to 36,000 degrees, travel of zero or at least 0.000001 degrees, and limits between 0.001 and 100,000 in the specified units.

Try changing the elbow limit from 30 to 45 deg/s in `two-caps.json`. You should get 1.5000 seconds, shoulder peak +90 and elbow peak −45. At an exact tie the CLI reports the first axis with the shared cap. A reported cap can be above the actual peak in a short move.

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| Python cannot open the script | Open the terminal in the folder where `joint_speed.py` sits. |
| Invalid JSON | Use double quotes and omit trailing commas. Use positive numeric caps even for reverse motion. |
| CSV already exists | Give `--csv` a new name. The command refuses to overwrite files. |
| CSV would exceed 100,000 data rows | Increase `--step`. This reduces output resolution without changing the planned time. |
| A spreadsheet puts every value in one column | Import the file as comma separated text with a decimal point. |
| The tool tip does not move along a straight line | Joint space and Cartesian space are different. This program plans joint angles only. |

The tests check signed velocity area against known travel, analytically known samples, shared progress, speed and acceleration bounds, and CSV edge cases. The SVGs in `diagrams` can be regenerated with `python3 diagrams/render.py` and edited as vectors. The WebPs are raster exports of those SVGs.

This model assumes zero initial and final speed and instantaneous changes in acceleration. It omits jerk limits, robot geometry, torque, collisions, tracking error and safety checks. It cannot set safe limits or validate a real move. Do not send its CSV to a controller.

## Primary references

* Lynch and Park, [Modern Robotics: point to point trajectories, part 1](https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-1-and-9-2-point-to-point-trajectories-part-1-of-2/), for path versus time scaling.
* Lynch and Park, [part 2](https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-1-and-9-2-point-to-point-trajectories-part-2-of-2/), for trapezoidal time scaling and alternatives.
* WPILib, [Trapezoidal Motion Profiles](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html), for applying velocity and acceleration constraints to setpoint generation. This lab is its own Python implementation and does not depend on WPILib.
