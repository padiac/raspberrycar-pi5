#!/usr/bin/env python3
"""Checks for undoing the heading wrap, and for what it buys.

The board reports plus or minus a hundred and eighty degrees. Everything
here exists because subtracting two of those numbers is wrong in exactly the
situation that matters: a car turning through the wrap, which is one heading
in four hundred and will therefore happen on the day somebody is watching.

Run directly: python3 tests/test_turning.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from behaviors.patrol import PatrolBehavior, STALL_AFTER, STALL_RATE
from control.turning import TurnTracker, shortest_delta
from hal import Velocity
from perception.types import WorldState

PASSES = []
FAILS = []


def check(label, condition):
    (PASSES if condition else FAILS).append(label)
    print("  %-6s %s" % ("PASS" if condition else "FAIL", label))


def section(title):
    print("\n" + title)
    print("-" * len(title))


section("the wrap comes out")

check("an ordinary step", shortest_delta(10.0, 15.0) == 5.0)
check("and backwards", shortest_delta(15.0, 10.0) == -5.0)
check("across the wrap going right", shortest_delta(178.0, -178.0) == 4.0)
check("across the wrap going left", shortest_delta(-178.0, 178.0) == -4.0)
check("the far side is never the answer", abs(shortest_delta(179.0, -179.0)) < 10.0)

# A full turn, in twenty steps of eighteen degrees, crossing the wrap twice.
tracker = TurnTracker()
when = 0.0
heading = 0.0
# Seed it. The first frame can only be remembered, not differenced -- there
# is nothing to difference it against -- so a loop of twenty updates carries
# nineteen steps. Forgetting that was this test being wrong, not the tracker.
tracker.update(heading, when)
for step in range(20):
    heading = ((heading + 18.0 + 180.0) % 360.0) - 180.0
    when += 0.05
    tracker.update(heading, when)
check("twenty steps of eighteen degrees is one turn",
      abs(tracker.turned - 360.0) < 0.5)

# And back again: the total is signed, not a distance.
for step in range(20):
    heading = ((heading - 18.0 + 180.0) % 360.0) - 180.0
    when += 0.05
    tracker.update(heading, when)
check("turning back unwinds it", abs(tracker.turned) < 0.5)


section("the rate says whether the car is moving")

tracker = TurnTracker()
when, heading = 0.0, 0.0
tracker.update(heading, when)
for _ in range(20):
    when += 0.05
    heading = ((heading + 5.0 + 180.0) % 360.0) - 180.0
    tracker.update(heading, when)
check("a brisk spin reads about a hundred degrees a second",
      tracker.rate is not None and 90.0 < tracker.rate < 110.0)

for _ in range(20):
    when += 0.05
    tracker.update(heading, when)
check("a stopped car reads zero", abs(tracker.rate) < 1.0)

# The distinction the whole thing turns on: not turning, versus cannot tell.
# Advance the clock rather than passing when + something: the tracker
# ignores a frame stamped earlier than the one before it, which is correct
# and which this test was quietly relying on not happening.
when += 0.05
tracker.update(None, when)
check("a silent IMU is not the same as a stopped car", tracker.rate is None)
check("and the total it had is kept", abs(tracker.turned - 100.0) < 1.0)

when += 0.05
tracker.update(heading, when)
check("and it picks up again without a jump",
      abs(tracker.turned - 100.0) < 1.0)

mark = tracker.turned
for _ in range(9):
    when += 0.05
    heading = ((heading + 10.0 + 180.0) % 360.0) - 180.0
    tracker.update(heading, when)
check("since() measures from where you marked",
      abs(tracker.since(mark) - 90.0) < 0.5)


section("a turn ends when the car has turned, not when the clock says")


def world(turned=0.0, rate=0.0):
    return WorldState(range_cm=300.0, range_expected=True,
                      turned=turned, turn_rate=rate)


patrol = PatrolBehavior()
patrol.move = Velocity(omega=0.7)
patrol.move_since = 0.0
patrol.turn_goal = (0.0, 90.0)          # from zero, wanting ninety

done, _ = patrol._manoeuvre_over(world(turned=40.0, rate=100.0), 0.3)
check("part way round it keeps going", not done)

done, note = patrol._manoeuvre_over(world(turned=91.0, rate=100.0), 0.9)
check("at the angle it stops", done)
check("quietly, because nothing went wrong", note is None)

# Turning the other way must satisfy a goal of the other sign, and not one
# of this sign.
patrol.turn_goal = (0.0, -90.0)
done, _ = patrol._manoeuvre_over(world(turned=-91.0, rate=-100.0), 0.9)
check("and the same turning the other way", done)

# With no IMU the goal is never set, and the clock is all there is.
patrol.turn_goal = None
done, _ = patrol._manoeuvre_over(world(turned=0.0, rate=None), 0.9)
check("no IMU means the timed hold stands", not done)


section("commanded to turn and not turning is a stall")

patrol = PatrolBehavior()
patrol.move = Velocity(omega=0.7)
patrol.move_since = 0.0

done, _ = patrol._manoeuvre_over(world(rate=0.0), STALL_AFTER / 2)
check("it is given a moment to break friction", not done)

done, note = patrol._manoeuvre_over(world(rate=0.0), STALL_AFTER + 0.1)
check("then a motionless turn is caught", done)
check("and named", note is not None and "stalled" in note)
check("and counted as a block, so it escalates", len(patrol.blocks) == 1)

patrol = PatrolBehavior()
patrol.move = Velocity(omega=0.7)
patrol.move_since = 0.0
done, _ = patrol._manoeuvre_over(world(rate=STALL_RATE + 20.0), STALL_AFTER + 0.1)
check("a turn that is turning is left alone", not done)

# The case that would make this worse than useless: a silent IMU reads as
# no rotation, and every manoeuvre would be declared stalled.
patrol = PatrolBehavior()
patrol.move = Velocity(omega=0.7)
patrol.move_since = 0.0
done, _ = patrol._manoeuvre_over(world(rate=None), STALL_AFTER + 0.1)
check("a silent IMU does not stall every turn", not done)

# Nor should driving in a straight line, which commands no rotation at all.
patrol = PatrolBehavior()
patrol.move = Velocity(vx=0.8)
patrol.move_since = 0.0
done, _ = patrol._manoeuvre_over(world(rate=0.0), STALL_AFTER + 0.1)
check("driving straight is not a stalled turn", not done)


print("\n" + "=" * 62)
if FAILS:
    print("FAILED %d of %d:" % (len(FAILS), len(FAILS) + len(PASSES)))
    for name in FAILS:
        print("  - " + name)
    sys.exit(1)
print("all %d turning checks passed" % len(PASSES))
