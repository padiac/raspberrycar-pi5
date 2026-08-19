"""What the follow behaviour decides, with no camera and no car attached.

This is the layering paying off. A behaviour reads a WorldState and returns a
Velocity; both are plain data, so every decision it makes can be put under a
microscope by handing it a made-up world -- including the ones that are a
nuisance to arrange in a room, like an ultrasonic that hears nothing back.
"""

import sys


import sys
from pathlib import Path

# Find the robot from wherever this is run. Relying on the working directory
# instead -- which is what "sys.path.insert(0, '.')" amounts to -- works until
# somebody runs the file from anywhere else, and then fails with an ImportError
# about `hal` that says nothing about the actual problem.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from behaviors.follow import (  # noqa: E402
    AHEAD_DEADBAND, DISTANCE_DEADBAND_CM, FollowBehavior, STANDOFF_CM,
)
from perception.types import Detection, WorldState  # noqa: E402

FAILURES = []
WIDTH, HEIGHT = 640, 360


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not ok:
        print(f"        got  {got!r}")
        print(f"        want {want!r}")
        FAILURES.append(name)


def check_sign(name, value, sign):
    """sign: +1 forward, -1 back, 0 still."""
    got = 0 if abs(value) < 1e-9 else (1 if value > 0 else -1)
    check(f"{name} (v={value:+.3f})", got, sign)


class FakeRobot:
    """Everything FollowBehavior touches on a robot, which is nothing."""


def world(offset=0.0, fill=0.30, range_cm=None, range_forward=True):
    """A frame with one bottle in it, at a given bearing and apparent size."""
    box_h = fill * HEIGHT
    centre = WIDTH / 2 + offset * (WIDTH / 2)
    box = (centre - 40, (HEIGHT - box_h) / 2, centre + 40, (HEIGHT + box_h) / 2)
    return WorldState(
        dt=0.05,
        frame_size=(WIDTH, HEIGHT),
        detections=[Detection("bottle", 0.9, box, 39)],
        range_cm=range_cm,
        range_expected=True,
        range_forward=range_forward,
    )


def follower():
    behaviour = FollowBehavior(target="bottle")
    behaviour.start(FakeRobot())
    return behaviour


print("distance comes from the rangefinder when it can be trusted")

far = follower().update(FakeRobot(), world(range_cm=STANDOFF_CM + 40))
check_sign("further than the standoff: drive forward", far.vx, +1)

near = follower().update(FakeRobot(), world(range_cm=STANDOFF_CM - 25))
check_sign("closer than the standoff: back off", near.vx, -1)

at = follower().update(FakeRobot(), world(range_cm=STANDOFF_CM))
check_sign("at the standoff: hold still", at.vx, 0)

edge = follower().update(
    FakeRobot(), world(range_cm=STANDOFF_CM + DISTANCE_DEADBAND_CM - 1))
check_sign("inside the deadband: hold still", edge.vx, 0)

print()
print("a reading is only about the target while the target is ahead")
# The rangefinder points down the middle of the car. With the target off to one
# side it is measuring whatever happens to be straight ahead, which is not the
# thing being followed -- and a wall at 30cm would read as "arrived".
side = follower().update(
    FakeRobot(), world(offset=AHEAD_DEADBAND + 0.3, range_cm=STANDOFF_CM + 60))
check_sign("target off to the side: do not drive on that reading", side.vx, 0)
check("  but do turn towards it", abs(side.omega) > 0.01, True)

print()
print("falling back when the ultrasonic hears nothing")
# A small soft ball can return no echo at all. Vaguer beats stopped.
quiet = follower().update(FakeRobot(), world(fill=0.20, range_cm=None))
check_sign("no echo, target looks small: still approaches", quiet.vx, +1)

big = follower().update(FakeRobot(), world(fill=0.80, range_cm=None))
check_sign("no echo, target looks huge: still backs off", big.vx, -1)

# The head being turned away says nothing about the camera, which is bolted to
# the chassis and still looking where the car is going. So this falls back to
# the apparent-size estimate rather than giving up -- and the request to go
# forward is refused by the safety layer, which is where that rule lives.
# Restating it here would be the start of the two disagreeing.
aimed_away = follower().update(
    FakeRobot(), world(fill=0.20, range_cm=STANDOFF_CM + 40, range_forward=False))
check_sign("head turned away: ignores the reading, uses the camera", aimed_away.vx, +1)
# Which signal is driving has to be visible, or a car that is following badly
# looks the same whether the rangefinder is being used or quietly ignored.
b = follower()
b.update(FakeRobot(), world(fill=0.20, range_cm=STANDOFF_CM + 40, range_forward=False))
check("  and says it is on the camera", "no echo" in b.status()["state"], True)

b = follower()
b.update(FakeRobot(), world(range_cm=72.0))
check("a real reading is reported in centimetres", "72cm" in b.status()["state"], True)

print()
print("bearing")
right = follower().update(FakeRobot(), world(offset=+0.6, range_cm=STANDOFF_CM))
check_sign("target to the right: turn right", right.omega, +1)

left = follower().update(FakeRobot(), world(offset=-0.6, range_cm=STANDOFF_CM))
check_sign("target to the left: turn left", left.omega, -1)

print()
print("nothing to follow")
empty = WorldState(dt=0.05, frame_size=(WIDTH, HEIGHT), detections=[],
                   range_cm=100.0, range_expected=True)
behaviour = follower()
out = behaviour.update(FakeRobot(), empty)
check("no target: stopped", (out.vx, out.vy, out.omega), (0.0, 0.0, 0.0))

blind = WorldState(dt=0.05, frame_size=(0, 0))
check("no camera: stopped", follower().update(FakeRobot(), blind).moving, False)

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S): {FAILURES}")
    sys.exit(1)
print("all follow checks passed")
