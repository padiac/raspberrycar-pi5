#!/usr/bin/env python3
"""Every mode on the page, started and run, with nothing attached.

Each of these was found by somebody pressing a button on the car, not by a
test: the missions died on their first tick because they lacked two
attributes the control loop reads, and "Search and approach" died the
moment it reached its approach leaf because it read a counter the follower
had stopped having. Unit tests of each piece passed throughout. What none
of them did was the thing a person does: pick a mode and let it run.

So this does exactly that, for every behaviour the page offers, in a room
with the target in it and in one without -- and fails if anything raises
or returns something that is not a velocity.

Run directly: python3 tests/test_every_mode.py
"""

import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from behaviors import build_behaviors                       # noqa: E402
from control import PathRecorder                            # noqa: E402
from control.safety import SafetyLayer                      # noqa: E402
from hal import Velocity                                    # noqa: E402
from perception.types import Detection, WorldState          # noqa: E402

FAILS = []
WIDTH, HEIGHT = 1280, 720
TICKS = 120          # six seconds at the loop's 20Hz


class Calibration(dict):
    """The measured values the car runs on, so the code paths are the real ones."""


CALIBRATION = Calibration(
    camera_half_fov_deg=12.5, turn_rate_deg_s=30.0, turn_coast_deg=3.7,
    turn_floor=0.2, drive_floor=0.2, car_width_cm=15.0, car_length_cm=28.0,
    car_margin_cm=4.0, ir_fitted=True,
)


class Robot:
    """What behaviours reach for on a robot, and nothing else."""

    def __init__(self):
        self.calibration = CALIBRATION
        self.speed_limit = 0.4
        self.pan = None
        self.safety = SafetyLayer()
        self.world = None

    def _over(self, magnitude, key):
        floor = self.calibration.get(key) or 0.0
        return min(1.0, max(magnitude, floor / self.speed_limit)) if floor else magnitude

    def enough_to_turn(self, magnitude):
        return self._over(magnitude, "turn_floor")

    def enough_to_drive(self, magnitude):
        return self._over(magnitude, "drive_floor")


def world(tick, with_target):
    # The ball drifts across the picture and back, so following has
    # something to do, and turning has something to correct.
    detections = []
    if with_target:
        x = WIDTH / 2 + 300 * (((tick // 20) % 2) * 2 - 1) * ((tick % 20) / 20.0)
        detections = [Detection("ball", 0.85, (x - 90, 260, x + 90, 440), 32)]
    return WorldState(
        dt=0.05, frame_size=(WIDTH, HEIGHT), detections=detections,
        range_cm=120.0, range_expected=True, range_forward=True,
        forward_cm=120.0, forward_age=0.0,
        turned=tick * 0.5, turn_rate=10.0,
    )


def run(behaviour, with_target):
    robot = Robot()
    if getattr(behaviour, "accepts_target", False) and hasattr(behaviour, "configure"):
        behaviour.configure(target="ball")
    behaviour.start(robot)
    try:
        for tick in range(TICKS):
            robot.world = world(tick, with_target)
            robot.safety.filter(Velocity.STOP, robot.world)
            out = behaviour.update(robot, robot.world)
            if not isinstance(out, Velocity):
                return "tick %d returned %r, not a Velocity" % (tick, out)
            if hasattr(behaviour, "status"):
                behaviour.status()
    finally:
        behaviour.stop(robot)
    return None


for with_target in (True, False):
    print("\n%s" % ("with a ball in view" if with_target else "with nothing in view"))
    for name, behaviour in build_behaviors(recorder=PathRecorder()).items():
        try:
            problem = run(behaviour, with_target)
        except Exception:                                   # noqa: BLE001
            problem = traceback.format_exc().strip().splitlines()[-1]
        print("  %-6s %-8s %s" % ("PASS" if not problem else "FAIL", name, problem or ""))
        if problem:
            FAILS.append("%s (%s)" % (name, "target" if with_target else "empty"))

print()
if FAILS:
    print("%d FAILURE(S): %s" % (len(FAILS), FAILS))
    sys.exit(1)
print("every mode starts and runs")
