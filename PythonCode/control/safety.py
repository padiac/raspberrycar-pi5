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
        distance = world.range_cm
        if vx > 0 and distance is not None and distance > 0:
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
