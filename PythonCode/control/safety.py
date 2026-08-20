"""The reflex layer: what the robot will not let you do.

Every velocity request -- from a behaviour or from a human at the keyboard --
passes through here on the way to the wheels. That is the point: obstacle
avoidance stops being a *mode you choose* and becomes a property of the
machine, so lane following, object following and manual driving are all
protected by the same code.

One rule shapes the whole design: **this layer only ever removes motion, never
adds any.** It can slow you down, clamp an axis or refuse outright, but it will
not steer for you. A safety layer that injects its own movement ends up
fighting the behaviour above it -- both push, neither wins, and the result is
oscillation nobody can debug. Deciding *where to go instead* is intent, and
intent belongs to behaviours.

The original project put obstacle avoidance in a `main_obstacle_avoidance.py`
of its own, which meant you got avoidance or you got lane following, never
both.
"""

from dataclasses import dataclass, field

from hal import Velocity

#: Closer than this (cm) and forward motion is refused entirely.
STOP_CM = 20.0
#: Between STOP_CM and this, forward motion is scaled down with the room left.
SLOW_CM = 80.0
#: Speed still allowed at exactly STOP_CM, so the car creeps rather than
#: slamming to a halt at the threshold.
CREEP = 0.15
#: How old a forward reading may be and still describe where the car is.
#:
#: The head is on a servo, and there is more to look at than straight
#: ahead: a chair leg shows up as a single ten-degree spike in a sweep, so
#: a car that only ever looks forward is blind to everything its corners
#: are about to hit. Requiring the head to be forward *at this instant*
#: made glancing sideways cost a full stop, which meant it never happened.
#:
#: The bound is what makes this safe rather than convenient. Approaching
#: an obstacle the car was measured at 2 to 3 cm/s, because this layer is
#: already throttling it there, so a second-old reading is wrong by about
#: three centimetres against a stopping distance of twenty. Anything older
#: than this is treated exactly as it was before: no reading, no forward.
FORWARD_MEMORY_S = 1.0


@dataclass
class SafetyReport:
    """What the layer did to a request, and why."""

    requested: Velocity = Velocity.STOP
    allowed: Velocity = Velocity.STOP
    interventions: list = field(default_factory=list)

    @property
    def intervened(self):
        return bool(self.interventions)

    def as_dict(self):
        return {
            "intervened": self.intervened,
            "reasons": list(self.interventions),
            "requested": self.requested.as_dict(),
            "allowed": self.allowed.as_dict(),
        }


class SafetyLayer:
    def __init__(self, stop_cm=STOP_CM, slow_cm=SLOW_CM, enabled=True):
        self.stop_cm = stop_cm
        self.slow_cm = slow_cm
        self.enabled = enabled
        self.last = SafetyReport()

    def status(self):
        return {
            "enabled": self.enabled,
            "stop_cm": self.stop_cm,
            "slow_cm": self.slow_cm,
            **self.last.as_dict(),
        }

    def filter(self, requested, world):
        """Return the velocity the robot is actually allowed to execute."""
        reasons = []
        vx, vy, omega = requested.vx, requested.vy, requested.omega

        if not self.enabled:
            self.last = SafetyReport(requested, requested, ["safety disabled"])
            return requested

        # -- forward clearance, from the ultrasonic ------------------------
        # Only ever limits *forward* motion: reversing away from a wall in
        # front must stay possible, or the car can trap itself.
        # Not whatever the head is pointing at now -- the most recent thing it
        # saw while pointing forward, provided that was recent enough to still
        # be true. The head is allowed to look away; it is not allowed to look
        # away for long.
        distance = world.forward_cm
        forward_reading = (
            distance is not None
            and world.forward_age is not None
            and world.forward_age <= FORWARD_MEMORY_S
        )

        if vx > 0 and world.range_expected and not forward_reading:
            # Three ways to have no forward reading and one correct response.
            # The board has gone quiet, or the head is aimed elsewhere, or it
            # is still swinging back. None of them is open road, and all three
            # used to read as one: a miss came back as the last good value, or
            # as zero before there had been a good one, and zero fails the
            # `> 0` test below, so the whole check was skipped.
            # Which of the two it is matters to whoever is reading this. A
            # stale reading means the head is busy elsewhere and will be back;
            # no reading at all means the sensor is not answering, and the
            # difference is a glance versus a fault. Reporting the age says so
            # without having to guess: the last value hangs around after the
            # sensor stops answering, so its mere existence proves nothing.
            if world.forward_cm is None:
                reasons.append("no range reading, forward blocked")
            else:
                age = world.forward_age
                reasons.append(
                    "last look ahead was %.1fs ago, forward blocked" % age
                    if age is not None else "no range reading, forward blocked"
                )
            vx = 0.0
        elif vx > 0 and forward_reading and distance > 0:
            if distance < self.stop_cm:
                reasons.append(f"obstacle {distance:.0f}cm ahead, forward blocked")
                vx = 0.0
            elif distance < self.slow_cm:
                span = self.slow_cm - self.stop_cm
                ceiling = CREEP + (1.0 - CREEP) * (distance - self.stop_cm) / span
                if vx > ceiling:
                    reasons.append(f"obstacle {distance:.0f}cm ahead, forward limited")
                    vx = ceiling

        # -- side clearance, from the infrared pair -------------------------
        # These only see to the sides, so they gate strafing rather than
        # forward travel. On a 2-DOF chassis vy is discarded downstream
        # anyway, which is harmless.
        proximity = world.proximity
        if proximity is not None:
            if vy < 0 and proximity.left:
                reasons.append("obstacle to the left, strafe blocked")
                vy = 0.0
            if vy > 0 and proximity.right:
                reasons.append("obstacle to the right, strafe blocked")
                vy = 0.0
            # Something touching both sides means a gap too narrow to drive
            # into; hold position rather than wedging the car.
            if vx > 0 and proximity.left and proximity.right:
                reasons.append("blocked both sides, forward blocked")
                vx = 0.0

        allowed = Velocity(vx, vy, omega)
        self.last = SafetyReport(requested, allowed, reasons)
        return allowed
