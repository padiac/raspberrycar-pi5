"""Turning the firmware's heading into something a behaviour can use.

The board reports a heading in degrees and wraps it into plus or minus a
hundred and eighty, which is the right thing for a number a human reads off
a dashboard and the wrong thing for anything that wants to know how far the
car has turned. Subtracting two of them gives 359 degrees for a car that
moved one degree across the wrap, and a manoeuvre asked to turn 90 degrees
either finishes instantly or never.

So the wrap is undone the only way it can be: by watching, and accumulating
the small differences. Twenty hertz of telemetry means each step is a degree
or two even in a brisk spin, which is nowhere near the ambiguity at 180.

What this buys, which is more than tidiness:

**Turns can be closed-loop.** "Turn ninety degrees" instead of "turn for one
and a bit seconds and hope". The timed version is wrong whenever anything
changes -- the speed limit, the battery, the carpet -- and it is wrong
silently, which is how the car ends up facing a wall it meant to turn away
from. Gyro integration drifts over minutes and is excellent over the two
seconds a turn takes, so this is the one job the sensor is genuinely good at.

**Stalls become visible.** Commanded to turn and not turning is the signature
of a wheel on a cable, a castor wedged against a chair, a carpet edge -- and
of the quieter failure where the commanded duty is simply below what moves
this car at all. None of it is detectable from the rangefinder when there is
nothing in front of the car to watch, and all of it looks identical from
inside the control loop, where the wheels are being told what to do and the
behaviour is content.
"""


def shortest_delta(previous, current):
    """Signed degrees from previous to current, taking the short way round.

    The short way is the right way here because the two readings are a
    twentieth of a second apart. A car that really did turn 350 degrees
    between two frames would be spinning at 7000 degrees a second, which the
    gyroscope could not have measured anyway: its range is 250.
    """
    delta = (current - previous) % 360.0
    if delta > 180.0:
        delta -= 360.0
    return delta


class TurnTracker:
    """Accumulated rotation and current rate, from a wrapping heading."""

    #: Rate is averaged over about this long. A single pair of frames at 20Hz
    #: gives a rate quantised to roughly 20 degrees a second, which is the
    #: same size as the thing being measured.
    WINDOW_S = 0.35

    def __init__(self):
        #: Total rotation since this started, unwrapped and signed. Keeps
        #: growing; the point is differences, not the value.
        self.turned = 0.0
        self.rate = None            # degrees per second, None until known
        self._last = None           # (heading, when)
        self._history = []          # (when, turned)

    def update(self, heading, now):
        """Feed one telemetry frame. Heading may be None when the IMU is out."""
        if heading is None:
            # Not zero and not the last value: unknown. A behaviour that
            # cannot tell "not turning" from "cannot tell" will eventually
            # declare a perfectly good manoeuvre stalled.
            self._last = None
            self.rate = None
            self._history = []
            return self.turned

        if self._last is not None:
            previous, when = self._last
            if now > when:
                self.turned += shortest_delta(previous, heading)
        self._last = (heading, now)

        self._history.append((now, self.turned))
        self._history = [h for h in self._history if now - h[0] <= self.WINDOW_S]
        if len(self._history) >= 2:
            (t0, a0), (t1, a1) = self._history[0], self._history[-1]
            self.rate = (a1 - a0) / (t1 - t0) if t1 > t0 else self.rate
        return self.turned

    def since(self, mark):
        """How far the car has turned since a recorded `turned` value."""
        return self.turned - mark

    def status(self):
        return {
            "turned": round(self.turned, 1),
            "rate": None if self.rate is None else round(self.rate, 1),
        }
