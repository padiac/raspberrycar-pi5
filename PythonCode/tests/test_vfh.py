#!/usr/bin/env python3
"""Checks for the direction chooser.

The cases here are the ones the old scoring got wrong, written as the shapes
they have in a room rather than as abstractions: a gap narrower than the car,
a doorway, a corner, two equally good ways out.

Run directly: python3 tests/test_vfh.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control import vfh

PASSES = []
FAILS = []


def check(label, condition):
    (PASSES if condition else FAILS).append(label)
    print("  %-6s %s" % ("PASS" if condition else "FAIL", label))


def section(title):
    print("\n" + title)
    print("-" * len(title))


#: Half the chassis plus a margin, which is what the enlargement needs.
R = 15.0


section("an obstacle blocks a wider arc the closer it is")

check("far away it is nearly a point", vfh.blocked_arc(400.0, R) < 3.0)
check("at a metre it is noticeable",
      7.0 < vfh.blocked_arc(100.0, R) < 10.0)
check("at 30cm it is a serious bite", vfh.blocked_arc(30.0, R) > 29.0)
check("inside the radius nothing clears it", vfh.blocked_arc(10.0, R) == 90.0)
check("and a reading of zero is the same", vfh.blocked_arc(0.0, R) == 90.0)

# This is the distance weighting VFH does with a term in its histogram,
# arriving here out of the geometry instead.
check("closer always blocks more",
      vfh.blocked_arc(50.0, R) > vfh.blocked_arc(120.0, R))


section("a gap narrower than the car is not a way out")

# The chair-castor case, and the reason for all of this. The sweep sees a
# long reading straight between two legs, because the beam goes through the
# gap and bounces off the wall behind. It is genuinely 300cm of air -- in a
# slot 12cm wide, which the car cannot use.
castors = {-30.0: 25.0, -15.0: 25.0, 0.0: 300.0, 15.0: 25.0, 30.0: 25.0}
angle, trace = vfh.choose(castors, R)
check("the deep reading between two legs is refused",
      angle is None or abs(angle) > 20.0)

# The old rule, for contrast: pick the angle whose own reading is largest.
naive = max(castors, key=lambda a: castors[a])
check("whereas picking the deepest reading would have aimed at it",
      naive == 0.0)

# Same shape, but the gap is a doorway the car does fit through.
#
# The first version of this test put the door posts at 25cm and 30 degrees
# off, which is a gap of 2 * 25 * sin(30) = 25cm. The car needs 2R = 30cm, so
# the module was right to refuse it and the test was wrong to ask. Posts at
# 60cm and 30 degrees off are 60cm apart, which it fits through twice over.
doorway = {-30.0: 60.0, -15.0: 120.0, 0.0: 300.0, 15.0: 120.0, 30.0: 60.0}
angle, trace = vfh.choose(doorway, R)
check("a gap the car does fit through is taken",
      angle is not None and abs(angle) <= 20.0)


section("it prefers room, then straight on, then what it chose last")

choices = {-90.0: 40.0, -45.0: 60.0, 0.0: 30.0, 45.0: 350.0, 90.0: 50.0}
angle, _ = vfh.choose(choices, R)
check("the roomiest side wins", angle is not None and angle > 20.0)

# A corridor looks the same both ways. The old code broke that tie at random
# every time it was asked, which is a car twitching in a doorway.
corridor = {-90.0: 300.0, -45.0: 300.0, 0.0: 20.0, 45.0: 300.0, 90.0: 300.0}
left, _ = vfh.choose(corridor, R, previous=-60.0)
right, _ = vfh.choose(corridor, R, previous=60.0)
check("a tie goes the way it went last time", left < 0.0 < right)

first, _ = vfh.choose(corridor, R, previous=None)
again, _ = vfh.choose(corridor, R, previous=first)
check("and having chosen, it stays chosen", first == again)

# ...but not at any price: room has to be able to outvote the memory.
moved = {-90.0: 20.0, -45.0: 22.0, 0.0: 25.0, 45.0: 300.0, 90.0: 300.0}
angle, _ = vfh.choose(moved, R, previous=-90.0)
check("unless the room is clearly elsewhere", angle > 0.0)


section("nowhere to go is an answer, not a crash")

boxed = {-90.0: 12.0, -45.0: 12.0, 0.0: 12.0, 45.0: 12.0, 90.0: 12.0}
angle, trace = vfh.choose(boxed, R)
check("pressed in on all sides it returns nothing", angle is None)
check("and says why", "grown to its own width" in trace)

angle, trace = vfh.choose({}, R)
check("no readings at all is handled", angle is None and "no readings" in trace)

one = {0.0: 300.0}
angle, _ = vfh.choose(one, R)
check("a single reading still gives an answer", angle is not None)


section("the forbidden list is respected")

# A way out that was tried and led straight back.
twice = {-45.0: 300.0, 0.0: 20.0, 45.0: 60.0}
angle, _ = vfh.choose(twice, R)
check("normally it takes the open side", angle < 0.0)
blocked_angles = [a for a in vfh.fine_angles((-90.0, -20.0))]
angle, _ = vfh.choose(twice, R, forbid=blocked_angles)
check("with that side forbidden it goes the other way",
      angle is None or angle > -20.0)


section("valleys are found where they are")

bins = vfh.histogram({-90.0: 300.0, -45.0: 300.0, 0.0: 20.0,
                      45.0: 300.0, 90.0: 300.0}, R)
found = vfh.valleys(bins)
check("a blocked middle splits the arc in two", len(found) == 2)
check("and neither half contains straight ahead",
      all(not (low <= 0.0 <= high) for low, high in found))

bins = vfh.histogram({-90.0: 300.0, 0.0: 300.0, 90.0: 300.0}, R)
check("an empty room is one valley", len(vfh.valleys(bins)) == 1)


print("\n" + "=" * 62)
if FAILS:
    print("FAILED %d of %d:" % (len(FAILS), len(FAILS) + len(PASSES)))
    for name in FAILS:
        print("  - " + name)
    sys.exit(1)
print("all %d direction checks passed" % len(PASSES))
