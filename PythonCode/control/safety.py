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

import time

from hal import Velocity
from settings import Spec, group

#: Closer than this (cm) and forward motion is refused entirely.
STOP_CM = 20.0
#: Between STOP_CM and this, forward motion is scaled down with the room left.
SLOW_CM = 80.0
#: Speed still allowed at exactly STOP_CM, so the car creeps rather than
#: slamming to a halt at the threshold.
CREEP = 0.15
#: Fastest the car may go with no echo coming back.
#:
#: A timed-out ping is not the same quality of information as a measurement,
#: and in a house it usually is not "nothing within four metres" at all. Sound
#: bounces off a pane of glass like light off a mirror: dead on it returns, at
#: any angle it leaves and never comes back. A floor-to-ceiling window is
#: therefore invisible from almost every approach, and so is a smooth wall met
#: obliquely. Soft things -- curtains, a sofa -- swallow the pulse instead.
#: All of them report exactly what an empty room reports.
#:
#: So it may drive, because refusing to cross open space would make the car
#: useless, but only at a crawl. Set above the duty at which these motors
#: actually start turning (measured at 0.16 forward, 0.20 rotating) so this is
#: a slow car rather than a stalled one.
NO_ECHO_CEILING = 0.22

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

#: How long a jolt keeps forward travel shut after it is felt.
#:
#: The accelerometer already knew about collisions and nothing was listening:
#: a bump lit a box on the web page and changed no decision anywhere. That is
#: the one sensor reading that is never a false alarm about whether an
#: obstacle exists -- the car is touching it.
#:
#: Short, because this is a reflex and not a plan. It buys the layer above
#: one second to notice the contact and decide where to go instead, which is
#: its job; a reflex that held for five seconds would be making that decision
#: by default.
BUMP_FREEZE_S = 1.2

#: The numbers above are the defaults; these are the knobs. Declared here,
#: beside the paragraphs that explain what each one is for, and collected
#: centrally so that one page can list and change every setting on the car.
REFLEX = group(
    "safety",
    enabled=Spec(
        True,
        help="Whether the reflex layer acts at all. Stored because turning "
             "it off is a decision somebody makes knowingly, and having it "
             "come back on mid-experiment is its own hazard."),
    stop_cm=Spec(
        STOP_CM, unit="cm", bounds=(0.0, 200.0),
        help="Forward motion refused closer than this. Stopping distance "
             "sets it, not taste, so it is not the knob to turn when the "
             "car feels over-protective."),
    slow_cm=Spec(
        SLOW_CM, unit="cm", bounds=(0.0, 400.0),
        help="Forward motion throttled between stop_cm and here. This is "
             "the knob to turn: set wide, the car is throttled all the time "
             "in a flat and holding the trigger down does nothing."),
    no_echo_ceiling=Spec(
        NO_ECHO_CEILING, bounds=(0.0, 1.0),
        help="Speed allowed when the ping times out. One means no limit -- "
             "a timeout taken as open road. Glass, a curtain and a wall met "
             "at an angle all return what an empty room returns."),
    bump_guard=Spec(
        "auto", choices=("off", "auto", "always"),
        help="What a felt collision may do. auto blocks forward travel for "
             "a behaviour and not for a human, who felt the same collision "
             "and does not need their controls frozen."),
)


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
    #: Which of this layer's attributes come from the register.
    TUNED = ("enabled", "stop_cm", "slow_cm", "no_echo_ceiling", "bump_guard")

    def __init__(self, **overrides):
        """Thresholds follow the register unless given explicitly.

        The override exists for tests, which need a layer with particular
        thresholds and must not reach into the settings every other part of
        the process is reading -- a test that changes a global is a test
        that changes the next test. The running robot passes nothing, so
        what the web page sets is what the car does, with one copy of each
        number rather than one here and one in a file.
        """
        self._overrides = {}
        for key, value in overrides.items():
            if key not in self.TUNED:
                raise TypeError("SafetyLayer has no setting %r" % key)
            if value is not None:
                self._overrides[key] = value
        #: Set by the owner each tick, because whether a person is driving is
        #: a fact about the control stack and not about the world, and
        #: smuggling it into WorldState to avoid one assignment would make
        #: WorldState a lie.
        self.human_driving = False
        self.last = SafetyReport()
        self._bumped_at = None

    def __getattr__(self, key):
        # Only reached for attributes that are not set on the instance, so
        # the overrides and everything else resolve normally first.
        if key in SafetyLayer.TUNED:
            overrides = self.__dict__.get("_overrides") or {}
            if key in overrides:
                return overrides[key]
            return getattr(REFLEX, key)
        raise AttributeError(key)

    def __setattr__(self, key, value):
        if key in SafetyLayer.TUNED:
            # Changing a threshold on the layer would last until the next
            # restart and tell nobody, which is the bug this register was
            # built to end. Treated as an override so a test can still do
            # it, while the robot goes through set_safety and persists.
            self.__dict__.setdefault("_overrides", {})[key] = value
            return
        object.__setattr__(self, key, value)

    @property
    def bump_active(self):
        """Whether a felt jolt is currently holding the car back."""
        if self._bumped_at is None:
            return False
        return (time.monotonic() - self._bumped_at) < BUMP_FREEZE_S

    def status(self):
        return {
            "enabled": self.enabled,
            "stop_cm": self.stop_cm,
            "slow_cm": self.slow_cm,
            "bump_guard": self.bump_guard,
            "bump_active": self.bump_active,
            "no_echo_ceiling": self.no_echo_ceiling,
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
        elif vx > 0 and forward_reading and distance < 0:
            # Heard nothing back. Could be an empty room; could equally be
            # glass, or a wall at a glancing angle, or something soft.
            if vx > self.no_echo_ceiling:
                reasons.append("nothing came back, creeping")
                vx = self.no_echo_ceiling
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

        # -- the front corners, from the infrared pair -----------------------
        #
        # These sit where the ultrasonic cannot help. Its cone is about fifteen
        # degrees wide and the car is wider than that, so a chair leg off the
        # bow is invisible to it until a wheel finds it -- and it is aimed one
        # way at a time, so while it is looking left it is not looking anywhere
        # else. The infrared is fixed, instant, and covers exactly that.
        #
        # So a blocked corner stops forward travel, one corner being enough.
        # Waiting for both, as this used to, means the only obstacle it ever
        # objected to was a gap narrower than the car -- and clipping a table
        # leg with one corner is the collision that actually happens.
        proximity = world.proximity
        if proximity is not None:
            if vy < 0 and proximity.left:
                reasons.append("obstacle to the left, strafe blocked")
                vy = 0.0
            if vy > 0 and proximity.right:
                reasons.append("obstacle to the right, strafe blocked")
                vy = 0.0
            if vx > 0 and (proximity.left or proximity.right):
                where = ("both corners" if proximity.left and proximity.right
                         else "left corner" if proximity.left else "right corner")
                reasons.append(f"{where} blocked, forward blocked")
                vx = 0.0
            # Turning is gated too, one direction at a time.
            #
            # Spinning on the spot does not move the corners together: the
            # forward speed of a point on the chassis is omega times how far
            # that point sits to the side of centre, so the two front corners
            # advance in opposite directions. Turning right carries the left
            # corner forward and pulls the right corner back.
            #
            # Which gives a rule that needs to know nothing about where these
            # sensors are aimed -- a fact worth having, since a knock is enough
            # to change that: forbid the rotation that drives a blocked corner
            # forward, and permit the one that withdraws it.
            #
            # Escape survives, which is the property that matters. A blocked
            # left corner can still turn left and can still reverse; only with
            # both corners blocked does everything but reverse close, and
            # backing out is the right answer there anyway.
            if omega > 0 and proximity.left:
                reasons.append("left corner blocked, turning right blocked")
                omega = 0.0
            if omega < 0 and proximity.right:
                reasons.append("right corner blocked, turning left blocked")
                omega = 0.0

        # -- contact, from the accelerometer ---------------------------------
        #
        # Last, because it is the only input here that is not about an
        # obstacle ahead but about one already met, and because it overrides
        # a clear reading: the ultrasonic can be reporting two metres of open
        # road while a chair leg the beam missed entirely is against the
        # bumper. That is the exact case the other two checks cannot cover.
        #
        # It removes motion and does not add any, so it keeps the rule this
        # layer is built on. Only forward: reversing off whatever was hit is
        # the way out, and turning away from it has to stay possible too, so
        # neither is touched.
        if world.bumped:
            self._bumped_at = time.monotonic()
        if self._blocking_bump():
            if vx > 0:
                reasons.append("just hit something, forward blocked")
                vx = 0.0

        allowed = Velocity(vx, vy, omega)
        self.last = SafetyReport(requested, allowed, reasons)
        return allowed

    def _blocking_bump(self):
        if self.bump_guard == "off":
            return False
        if self.bump_guard == "auto" and self.human_driving:
            # A person felt the same collision through the floor and is
            # already reacting; freezing their controls for a second fights
            # them. An autonomous behaviour felt nothing and would keep
            # pushing, which is what this exists to stop.
            return False
        return self.bump_active
