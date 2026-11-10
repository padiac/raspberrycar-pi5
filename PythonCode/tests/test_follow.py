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
#
# This used to be enforced by refusing to move at all while the target was
# off to one side, which produced a car that turned, started, drifted back
# over the edge of the band, and stopped -- reported as going forward "in
# fits and starts". The invariant was never "do not move". It was "do not
# believe that number". So the check is now the invariant itself: off to
# the side, what the rangefinder says must make no difference whatsoever.
aside = AHEAD_DEADBAND + 0.3
near = follower().update(FakeRobot(), world(offset=aside, range_cm=20.0))
far = follower().update(
    FakeRobot(), world(offset=aside, range_cm=STANDOFF_CM + 60))
check("target off to the side: a reading of 20cm and one of 105cm give "
      "exactly the same answer", round(near.vx, 9), round(far.vx, 9))
check("  but it does turn towards it", abs(far.omega) > 0.01, True)

# ...and it approaches more cautiously than it would with the target in
# front, because the part of the motion that is not towards the target is
# wasted and the turn is the more urgent half.
head_on = follower().update(FakeRobot(), world(offset=0.0, range_cm=None))
check("  and more slowly than it would head-on", far.vx < head_on.vx, True)
check("  which is still forwards, not a stop", far.vx > 0.0, True)

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
print("holding on to the one it is following")

# The complaint that started this: "if there are two things in the picture,
# which one does it follow?" The old answer was "whichever measures bigger
# this frame", decided afresh every tick, so two similar targets could swap
# without anything in the code noticing.
behaviour = follower()
behaviour.update(FakeRobot(), world(offset=-0.3, range_cm=STANDOFF_CM))
both = WorldState(
    dt=0.05, frame_size=(WIDTH, HEIGHT),
    detections=[
        Detection("bottle", 0.9, (WIDTH / 2 - 0.28 * WIDTH / 2 - 40,
                                  120, WIDTH / 2 - 0.28 * WIDTH / 2 + 40, 240), 39),
        # Bigger, and across the room. Bigger is not the question.
        Detection("bottle", 0.9, (WIDTH / 2 + 0.6 * WIDTH / 2 - 70,
                                  60, WIDTH / 2 + 0.6 * WIDTH / 2 + 70, 300), 39),
    ],
    range_cm=STANDOFF_CM, range_expected=True, range_forward=True)
behaviour.update(FakeRobot(), both)
check("a second, larger target does not steal the car",
      behaviour.track.offset < 0, True)

# A detector that blinks for a frame or two used to mean stopping dead and
# starting again. Now the belief coasts, so the car keeps turning towards
# where the target was going.
blinking = follower()
for step in range(6):
    blinking.update(FakeRobot(), world(offset=-0.1 - 0.06 * step,
                                       range_cm=STANDOFF_CM))
moving_left = WorldState(dt=0.05, frame_size=(WIDTH, HEIGHT), detections=[],
                         range_cm=STANDOFF_CM, range_expected=True)
coasting = blinking.update(FakeRobot(), moving_left)
check("one missed look does not stop the turn", abs(coasting.omega) > 0.01, True)
check("  but it does stop driving at something nobody can see",
      coasting.vx, 0.0)
check("  and it says so", "coasting" in blinking.status()["state"], True)

gone = follower()
gone.update(FakeRobot(), world(offset=0.2, range_cm=STANDOFF_CM))
for _ in range(40):
    gone.update(FakeRobot(), moving_left)
check("a target that never comes back is given up", gone.track.alive, False)
check("  and not remembered as a stale guess", gone.track.offset, None)

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S): {FAILURES}")
    sys.exit(1)
print("all follow checks passed")
