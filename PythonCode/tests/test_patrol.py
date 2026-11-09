#!/usr/bin/env python3
"""Checks for what patrol decides, given a sweep.

Decision-level rather than unit-level: the chooser has its own suite in
test_vfh.py and the heading arithmetic has one in test_turning.py. What is
left, and what these cover, is whether the branches around them fire in the
right order -- which is where this behaviour has gone wrong before.

Run directly: python3 tests/test_patrol.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from behaviors.patrol import PatrolBehavior
from control.safety import SafetyLayer
from perception.types import WorldState

PASSES = []
FAILS = []


def check(label, condition):
    (PASSES if condition else FAILS).append(label)
    print("  %-6s %s" % ("PASS" if condition else "FAIL", label))


def section(title):
    print("\n" + title)
    print("-" * len(title))


class FakeCalibration:
    def __init__(self, **kw):
        self.values = {"car_width_cm": 15.0, "car_length_cm": 28.0,
                       "car_margin_cm": 4.0, "turn_floor": 0.0}
        self.values.update(kw)

    def get(self, key):
        return self.values.get(key)


class FakeRobot:
    def __init__(self, **kw):
        self.calibration = FakeCalibration(**kw)
        self.safety = SafetyLayer()
        self.speed_limit = 0.4
        self.pan = None

    def enough_to_turn(self, magnitude):
        return magnitude


def decide(clearance, unknown=(), previous=None, **chassis):
    robot = FakeRobot(**chassis)
    patrol = PatrolBehavior()
    patrol.world_now = WorldState(range_cm=300.0, range_expected=True)
    patrol.clearance = dict(clearance)
    patrol.unknown = set(unknown)
    patrol.last_choice = previous
    velocity = patrol._decide(robot, 100.0)
    return velocity, patrol


section("the rotation check follows the chassis it was measured from")

robot = FakeRobot()
patrol = PatrolBehavior()
check("a 15 by 28 car sweeps a 15.9cm radius",
      abs(patrol._turn_clearance(robot) - math.hypot(7.5, 14.0)) < 0.01)

narrow = FakeRobot(car_width_cm=10.0, car_length_cm=10.0)
check("a smaller car needs less room",
      patrol._turn_clearance(narrow) < patrol._turn_clearance(robot))

unmeasured = FakeRobot(car_width_cm=0.0, car_length_cm=0.0)
check("an unmeasured chassis falls back rather than claiming zero",
      patrol._turn_clearance(unmeasured) > 10.0)


section("sideways before backwards, because backwards is ground already covered")

# Too tight to turn -- 12cm ahead against a 15.9cm swept radius -- but sixty
# centimetres of room off to the left.
velocity, patrol = decide({-90: 60, -60: 55, -30: 14, 0: 12,
                           30: 14, 60: 20, 90: 22})
check("it slides rather than reversing", velocity.vy < 0.0)
check("towards the side with room", velocity.vy < 0.0 and velocity.vx == 0.0)
check("and says so", "sliding left" in patrol.state)
check("then looks again", patrol.follow is not None)

# Mirror image: the room is on the right.
velocity, _ = decide({-90: 22, -60: 20, -30: 14, 0: 12,
                      30: 14, 60: 55, 90: 60})
check("and the other way when the room is there", velocity.vy > 0.0)

# Nowhere to the side either: reversing is what is left.
velocity, patrol = decide({-90: 14, -60: 14, -30: 13, 0: 12,
                           30: 13, 60: 14, 90: 14})
check("pressed in on all sides it backs out", velocity.vx < 0.0)
check("and does not pretend to slide", velocity.vy == 0.0)

# Room beside, but only just: the step itself has to land somewhere.
velocity, _ = decide({-90: 13, -60: 13, -30: 13, 0: 12,
                      30: 13, 60: 13, 90: 13})
check("a gap too small for the step is not used", velocity.vy == 0.0)

# Silence off the beam is usually something soft standing right there, so a
# side that heard nothing is not room even though it reads as a long way.
velocity, patrol = decide({-90: 300, -60: 14, -30: 14, 0: 12,
                           30: 14, 60: 14, 90: 14}, unknown=(-90.0,))
check("a side that heard nothing does not count as room",
      "sliding" not in patrol.state)
check("so it reverses instead", velocity.vx < 0.0)

# The same geometry, heard rather than guessed, is used.
velocity, patrol = decide({-90: 300, -60: 14, -30: 14, 0: 12,
                           30: 14, 60: 14, 90: 14})
check("and a side that was measured is", velocity.vy < 0.0)


section("the slot that started all this")

# A gap the beam goes through and the car does not. Posts 20cm away at 30
# degrees off are 2 * 20 * sin(30) = 20cm apart; the car is 15cm wide and
# asks for 23cm with its margin, so it does not fit.
#
# The first version of this used 25cm posts, which is a 25cm slot -- and
# once the chassis was actually measured at 15cm rather than guessed at 22,
# the car fitted through it with a centimetre to spare on each side. The
# module was right and the test was describing the old, wider, imaginary
# car. Worth leaving the arithmetic written down here.
velocity, patrol = decide({-90: 60, -60: 22, -30: 20, 0: 300,
                           30: 20, 60: 22, 90: 60})
check("the deep reading does not win", velocity.vx <= 0.0)
check("it goes to the side instead", abs(velocity.vy) > 0.0)
check("naming the reason", "no gap wide enough" in patrol.state)

# Widen the slot past what the car needs and it should simply drive.
velocity, patrol = decide({-90: 30, -60: 28, -30: 70, 0: 300,
                           30: 70, 60: 50, 90: 70})
check("a gap it fits through is driven at", velocity.vx > 0.0)


print("\n" + "=" * 62)
if FAILS:
    print("FAILED %d of %d:" % (len(FAILS), len(FAILS) + len(PASSES)))
    for name in FAILS:
        print("  - " + name)
    sys.exit(1)
print("all %d patrol checks passed" % len(PASSES))
