"""Pointing the head, and knowing when it is pointing at the car.

The sensor rides on a servo, which turns one number -- how far away is
something -- into two: how far, and in which direction. Everything that
follows from that lives here. Aiming, the trim that makes the middle of
the slider the middle of the car, the direction the linkage turns, and the
map of what parts of this chassis are in the sensor's own way.

All of it is calibration in the strict sense: measurements of one
assembled machine that no amount of reasoning recovers. Somebody has to
point the thing at a wall and write down what it says.
"""

import time

from hal import Velocity


def blind_at(robot, angle):
    """The floor for that angle, or None if nothing was recorded."""
    blind = robot.calibration.get("pan_blind") or {}
    if angle is None or not blind:
        return None
    nearest = min(blind, key=lambda k: abs(float(k) - angle))
    if abs(float(nearest) - angle) > 10.0:
        return None
    return float(blind[nearest])


def calibrate_pan_blind(robot, samples=9, dwell=0.9, angles=None):
    """Learn what parts of the car the sensor can see, angle by angle.

    Wants open space around it, because the method cannot tell a bracket
    from a wall except by distance and steadiness -- so anything far off
    or wavering is discarded rather than recorded.

    What this buys is the difference between "there is an obstacle that
    way" and "I cannot see that way". A wire eighteen centimetres from the
    head reads as an obstacle at every sweep, drags down the score of the
    angles either side of it, and quietly condemns that whole side of the
    car for good -- the way out is always somewhere else, and the room the
    car will not enter is not blocked at all.
    """
    if robot.pan is None or robot.rangefinder is None:
        return None
    angles = angles or (-90.0, -45.0, 0.0, 45.0, 90.0)
    found = {}
    for angle in angles:
        robot.pan.look(angle)
        time.sleep(dwell)
        taken = []
        for _ in range(samples):
            reading = robot.link.telemetry.range_cm if robot.link else None
            if reading is not None and reading > 0:
                taken.append(reading)
            time.sleep(0.12)
        if len(taken) < samples * 0.6:
            continue                      # heard nothing: sees past itself
        taken.sort()
        median = taken[len(taken) // 2]
        if median > robot.BLIND_MAX_CM:
            continue                      # too far to be part of the car
        if taken[-1] - taken[0] > robot.BLIND_SPREAD_CM:
            continue                      # wandering, so it is the room
        found[str(int(angle))] = round(median, 1)
    robot.pan.centre()
    robot.calibration.set("pan_blind", found)
    return found


def set_pan_reversed(robot, reversed_):
    """Say which way the head runs, and remember it."""
    if robot.pan is None:
        return None
    robot.pan.set_reversed(reversed_)
    robot.calibration.set("pan_reversed", robot.pan.reversed)
    return robot.pan.reversed


def set_pan_trim(robot, trim):
    """Aim the head, and remember where forward turned out to be.

    The saving lives here rather than in PanServo because a HAL device
    should not know what a filesystem is; it knows how to point a servo.
    """
    if robot.pan is None:
        return None
    robot.pan.set_trim(trim)
    robot.calibration.set("pan_trim", robot.pan.trim)
    return robot.pan.trim


def _link_came_back(robot, was_reset):
    """Put back the state that lives on the far end of the serial link.

    Velocity looks after itself: the control loop sends one every tick and
    the firmware stops on its own if it hears nothing. The servo does not.
    A hobby servo has no feedback, so the only record of where the head is
    pointing is the number this side remembers -- and a firmware that has
    just booted has centred the head without telling anyone. Left alone,
    every range reading after a reset is filed under an angle the head is
    not at, and the scan table quietly becomes fiction.
    """
    if not was_reset or robot.pan is None:
        return
    robot.pan.look(robot.pan.angle)
    print("[hw] board reset, re-aimed head to %.0f deg" % robot.pan.angle, flush=True)

#: How near a reading has to be to the recorded floor to count as looking
#: at the car rather than past it. Wide enough for the centimetre or two
#: of jitter a rigid target still produces, narrow enough that a real
#: obstacle at a similar distance is not waved away.
BLIND_MARGIN_CM = 3.0
#: Nothing further than this is taken to be part of the car. Furniture
#: standing still during the calibration would otherwise be recorded as a
#: permanent blind spot and then ignored for the rest of the car's life.
BLIND_MAX_CM = 40.0
#: A rigid target attached to the chassis gives the same figure every
#: time. Anything that wanders more than this is the room, not the car.
BLIND_SPREAD_CM = 4.0
