"""Turning a measured number of degrees, on a car that cannot turn gently.

Reported from the room, repeatedly: the ball is in the picture, the car
swings a long way past it, comes back, and hunts. Three measurements, all
taken on this car with the ball in front of it, say why.

    smallest turn the car can make      1.7 degrees  (a 0.08s pulse)
    turn rate once it is moving         46 degrees/second
    half the camera's picture           16 degrees

The first two are one fact. Below a duty of 0.2 these motors do not rotate
the chassis at all -- rotating in place fights the rollers on four wheels
-- so a request for a gentle turn comes out as 0.2 anyway, and the only
thing that can actually be varied is how long it lasts.

The third is what makes the old loop impossible rather than merely coarse.
A ball a quarter of the way to the edge is four degrees off. At 46 degrees
a second the car covers that in under a tenth of a second, while the
camera, the detector and the control loop together take rather longer than
that to notice. The feedback arrives describing a car that has already
turned five or six degrees further. A loop whose delay is longer than the
error it is correcting will oscillate at any gain, which is why nothing
done to the gains helped and why it was never a tuning problem.

So the car stops steering continuously and starts moving in steps: turn by
a measured amount, stop, wait long enough for a genuinely new look, decide
again. The gyro closes each step, so the size of the step is known rather
than hoped for -- and when there is no gyro the clock does it instead,
which is worse and still works.

This is deliberately not a controller. It has no gains and no memory of
past errors. It is the thing underneath one: a way to ask for a rotation
and have it happen.
"""

import math

from settings import Spec, group

#: What a turn command has to be to move the chassis at all.
#:
#: Not a choice. `Robot.enough_to_turn` raises anything smaller to this,
#: because under it the car does not rotate and a behaviour is left
#: convinced it manoeuvred. Kept here as the value this module commands,
#: so that nothing else has to raise it afterwards.
EFFORT = 0.20

STEP = group(
    "aim",
    settle_s=Spec(
        0.22, unit="s", bounds=(0.0, 3.0),
        help="How long to hold still after a step before acting on what is "
             "seen. Shorter than the camera and detector take together and "
             "the car decides twice on the same picture, which is how it "
             "swung past the target and came back."),
    smallest_deg=Spec(
        1.5, unit="deg", bounds=(0.1, 30.0),
        help="Turns smaller than this are not attempted. Measured on this "
             "car: the shortest pulse that moves it at all is worth about "
             "1.7 degrees, so asking for less is asking for nothing."),
    longest_s=Spec(
        0.55, unit="s", bounds=(0.05, 5.0),
        help="Longest single step. A big correction is taken in several "
             "steps with a look between them rather than one long swing, "
             "because the thing being corrected may move."),
    tolerance_deg=Spec(
        1.2, unit="deg", bounds=(0.1, 20.0),
        help="How close to the requested angle counts as arrived. Below "
             "roughly one step's worth there is nothing the car can do "
             "about the difference."),
    brake_s=Spec(
        0.14, unit="s", bounds=(0.0, 1.0),
        help="How long to push the other way when a step ends. Cutting "
             "the power does not stop this car: the driver's output stage "
             "switches off and the wheels free-wheel, a median of 8.6 "
             "degrees. A counter push of this length brings that to 3.7. "
             "Too long and it sets off backwards, which shows up as the "
             "coast measuring negative. Zero disables it."),
    patience=Spec(
        2.0, unit="x", bounds=(1.0, 6.0),
        help="How much longer than the expected time a step may take "
             "before it is abandoned. Catches a wheel against a chair leg, "
             "where the gyro target would otherwise never arrive."),
)


class Aim:
    """Ask for a rotation; get told what to command, tick by tick.

    Three states, and the caller can see which: stepping, settling, and
    idle. Only in idle is it worth asking for anything new, because a
    decision taken while the car is mid-step is taken on a picture from
    before the step began.
    """

    def __init__(self, rate_deg_s, effort=EFFORT, coast_deg=0.0, **overrides):
        #: Degrees per second once moving, measured on this chassis. Used
        #: to guess how long a step should take; the gyro decides when it
        #: is actually finished, so an inaccurate rate costs a little
        #: efficiency rather than accuracy.
        self.rate = float(rate_deg_s) if rate_deg_s else 45.0
        self.effort = effort
        #: How far this chassis keeps going after a step is called off,
        #: braking included. Measured; see calibration.py. Zero for a car
        #: nobody has measured, which simply means it will overshoot by
        #: however much it coasts, as it did before this existed.
        self.coast = max(0.0, float(coast_deg or 0.0))

        unknown = sorted(set(overrides) - set(SETTINGS))
        if unknown:
            raise TypeError("no such aim setting: %s (have: %s)"
                            % (", ".join(unknown), ", ".join(SETTINGS)))
        self._settings = dict(overrides)

        self.goal = None            # heading we are turning towards
        self.direction = 0
        self.deadline = None        # give up on this step at this time
        self.settled_at = None      # when the settle after a step ends
        self.asked_deg = 0.0
        self.steps = 0
        self.abandoned = 0
        #: How long between calls, measured rather than assumed. A
        #: command can only be withdrawn on a tick boundary, so this is
        #: exactly how far the car still travels after the decision to
        #: stop -- and the loop that calls this is not running at the
        #: rate it asks for, because the work on top of its sleep
        #: includes copying a whole camera frame.
        self._interval = None
        self._called = None
        #: While this is set, the car is being pushed the other way to
        #: kill the momentum a cut command leaves it with.
        self._braking_until = None
        self._braked_way = 0

    def _get(self, name):
        chosen = self._settings.get(name)
        return getattr(STEP, name) if chosen is None else chosen

    # -- what the caller sees ----------------------------------------------

    @property
    def smallest_deg(self):
        """The floor on a step, before anything has been observed."""
        return self._get("smallest_deg")

    @property
    def lead_deg(self):
        """How far the car still travels after a step is called off.

        One more tick of commanded turning, because the command stands
        until the loop comes round again, plus the coast, because cutting
        the power does not stop this chassis. The tick is measured from
        the interval between calls and is zero until there have been two.
        """
        return self.coast + self.rate * (self._interval or 0.0)

    @property
    def useless_below(self):
        """Smaller than this and the step would end on the tick it began.

        The goal is reached when what remains is inside the tolerance plus
        the lead, so a request already inside that commands nothing at
        all. Refusing it is the difference between a car that declines to
        make a turn it cannot make, and a car with a rising step counter
        that is not moving.
        """
        return max(self.smallest_deg,
                   self._get("tolerance_deg") + self.lead_deg)

    @property
    def stepping(self):
        return self.deadline is not None

    def settling(self, now):
        return self.settled_at is not None and now < self.settled_at

    def ready(self, now):
        """Idle, and the picture is as new as it is going to get."""
        return not self.stepping and not self.settling(now)

    # -- asking for a turn --------------------------------------------------

    def request(self, degrees, turned, now):
        """Turn this far, positive to the right. True if a step started.

        Refused while a step is running or settling, because a decision
        taken then was taken on a stale picture -- and acting on a stale
        picture twice in a row is the whole fault this replaces.
        """
        if not self.ready(now):
            return False
        if degrees is None or not math.isfinite(degrees):
            return False
        if abs(degrees) < self.useless_below:
            return False

        longest = self._get("longest_s")
        wanted = min(abs(degrees), self.rate * longest)
        self.direction = 1 if degrees > 0 else -1
        self.asked_deg = wanted * self.direction
        # The gyro ends the step when it can. The clock is the backstop for
        # a car whose board has gone quiet, and for a wheel against a chair
        # leg where the angle will never arrive.
        self.goal = None if turned is None else turned + self.asked_deg
        self.deadline = now + max(
            0.05, wanted / max(self.rate, 1.0) * self._get("patience"))
        self.settled_at = None
        self.steps += 1
        return True

    def omega(self, now, turned):
        """What to command this tick. Zero while settling or idle."""
        if self._called is not None:
            gap = now - self._called
            # A gap of zero is a test calling twice at the same instant;
            # a huge one is the loop having been elsewhere. Neither says
            # anything about how long a tick is.
            if 0.0 < gap < 1.0:
                self._interval = (gap if self._interval is None
                                  else self._interval * 0.7 + gap * 0.3)
        self._called = now

        if self._braking_until is not None:
            if now < self._braking_until:
                # The step is over; this is what stopping costs on a car
                # whose idea of "stop" is to let go of the wheels.
                return self.effort * -self._braked_way
            self._braking_until = None
            self._braked_way = 0

        if not self.stepping:
            return 0.0

        if self.goal is not None and turned is not None:
            remaining = (self.goal - turned) * self.direction
            # Stop early by everything the car is still going to travel:
            # one more tick of commanded turning, because the command
            # stands until the loop comes round again, and then the
            # coast, because cutting the power does not stop it. Both are
            # measured. Without them a request for 3.3 degrees delivered
            # 8.7, and the step after it had to undo the difference.
            if remaining <= self._get("tolerance_deg") + self.lead_deg:
                self._done(now)
                return 0.0
        if now >= self.deadline:
            # Out of time. With a gyro that means the car did not get
            # there -- a wheel caught, or the chassis is on carpet today --
            # and it is worth counting, because a car that silently fails
            # to turn looks exactly like one that decided not to.
            if self.goal is not None and turned is not None:
                self.abandoned += 1
            self._done(now)
            return 0.0
        return self.effort * self.direction

    def _done(self, now):
        brake = self._get("brake_s")
        self._braking_until = (now + brake) if brake > 0 else None
        self._braked_way = self.direction
        self.goal = None
        self.deadline = None
        self.direction = 0
        # The settle is for the camera: it starts once the car has
        # actually stopped, which is after the braking, not after the
        # decision to stop.
        self.settled_at = now + brake + self._get("settle_s")

    def forget(self):
        self.goal = None
        self.deadline = None
        self.direction = 0
        self.settled_at = None
        self._braking_until = None
        self._braked_way = 0
        # The interval is a property of the loop, not of the step, so it
        # survives: forgetting one target does not make the car's control
        # loop a different speed.

    def status(self, now=None):
        return {
            "stepping": self.stepping,
            "settling": bool(now is not None and self.settling(now)),
            "asked_deg": round(self.asked_deg, 1),
            "steps": self.steps,
            "abandoned": self.abandoned,
            "rate_deg_s": round(self.rate, 1),
            "coast_deg": round(self.coast, 1),
            "useless_below": round(self.useless_below, 1),
            "braking": self._braking_until is not None,
            "tick_s": None if self._interval is None else round(self._interval, 3),
            "lead_deg": round(self.lead_deg, 1),
        }


#: Every name in STEP, which is also every keyword the constructor takes.
#: Declared because a Group forwards attribute access to the register and
#: cannot be asked what it holds -- and because forgetting to add one to
#: both places has already cost an evening elsewhere in this tree.
SETTINGS = ("settle_s", "smallest_deg", "longest_s", "tolerance_deg",
            "patience", "brake_s")
