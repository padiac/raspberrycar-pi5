"""What a piece of hardware on this robot has to look like.

The test of a hardware abstraction layer is not whether the files are tidy.
It is this: when a part is swapped for a different part that does the same
job, how much of the codebase has to be edited? Before this file the answer
was "the robot", because the choice between an ultrasonic on the Pi's own
pins and one reported over the serial link was an `if` statement in
robot.py, and everything downstream was written against whichever class
came out of it. Two implementations of the same idea, related by nothing a
compiler or a reader could check, and a third would have meant a third
branch in the same `if`.

So there are two things here.

**Contracts.** One abstract class per job a device can do: measuring a
distance, reporting a corner, pointing the head, watching the supply. A new
driver subclasses one and implements a handful of methods, and the rest of
the robot keeps working because the rest of the robot only ever knew the
contract. The contracts were already there in spirit -- both rangefinders
had `read_cm`, both proximity arrays had `read` -- but a convention nobody
has written down is one that drifts.

**A registry.** Drivers announce themselves and say when they can be built.
Choosing between them stops being a branch in the robot and becomes data:
the registry is asked for a range sensor, each candidate says whether it can
be built from what is available, and the best-placed one wins. Adding
hardware is adding a file. Removing it is deleting one.

A driver may say no. A Pi with no camera ribbon, an ESP32 that is not
plugged in, a module that imports a library nobody installed -- all of these
are ordinary, and the robot is expected to run without them. `build` returns
None rather than raising, and every caller already handles None because the
hardware was always allowed to be missing.
"""

from abc import ABC, abstractmethod


class Device(ABC):
    """Anything physical the robot owns.

    Three things are asked of all of them, because three things are done to
    all of them: say what you are, say how you are, and let go of whatever
    you hold.
    """

    #: The job this device does. One of the KINDS below. Used by the registry
    #: and nowhere else; code that has a device already knows what it asked
    #: for.
    kind = "device"

    #: Human-readable, for logs and the web page. Says which implementation
    #: it is, not which job -- "ultrasonic via ESP32", not "rangefinder" --
    #: because the whole point is that several can fill the same role.
    label = "device"

    def status(self):
        """A plain dict for the web page. JSON-safe values only."""
        return {"label": self.label, "kind": self.kind}

    def close(self):
        """Release pins, threads, file handles. Must tolerate being called
        twice, and must not raise: it runs on the way out of a process that
        is already shutting down, and a failure here hides the real one."""


class RangeSensor(Device):
    """Measures the distance to whatever is in front of it.

    The three-state return is the contract, and getting it wrong has cost
    this project more than any other single mistake:

        positive  a measurement, in centimetres
        negative  the pulse went out and nothing came back inside range
        None      the sensor is not answering

    The second and third are not the same and must never be folded together.
    A timed-out ping indoors is usually glass, a curtain or a sofa -- things
    that return exactly what an empty room returns -- while None means the
    hardware is silent. Treating silence as open road drives into walls;
    treating a timeout as a fault refuses to cross an empty room.
    """

    kind = "range"

    @abstractmethod
    def read_cm(self):
        """One reading, with the three-state meaning above."""

    def read_smoothed_cm(self):
        """A reading worth acting on, for sensors that are noisy enough to
        need it. The default is to trust the raw one."""
        return self.read_cm()


class ProximitySensor(Device):
    """Says whether something is beside the car, without saying how far.

    `fitted` exists because an unconnected input pin sitting at its pull-up
    reads exactly like a clear road, and a sensor nobody has confirmed is
    wired must not be allowed to vote.
    """

    kind = "proximity"
    fitted = True

    @property
    def reporting(self):
        """Whether readings are arriving at all, fitted or not."""
        return True

    @abstractmethod
    def read(self):
        """A Proximity, or None when there is nothing to report."""


class FloorSensor(Device):
    """Looks down. Line following, and one day not falling down the stairs."""

    kind = "floor"

    @abstractmethod
    def read(self):
        """A Floor, or None."""


class PanMount(Device):
    """Points something -- here, the rangefinder -- at a chosen angle.

    Angles are degrees off straight ahead, positive to the right, which is
    the same sense as vy and omega everywhere else in this robot. A mount
    has no feedback, so `angle` is what was last commanded and reachable,
    not what was measured.
    """

    kind = "pan"

    #: Where it is pointing, in degrees off forward. A plain attribute and
    #: not a property on purpose: implementations assign to it from look(),
    #: and an abstract property here would make that assignment illegal for
    #: no gain -- there is nothing to compute, only something to remember.
    angle = 0.0

    @abstractmethod
    def look(self, angle):
        """Aim. Returns the angle actually reachable, which may be clamped."""

    @property
    def limit(self):
        """How far either side of forward it can be asked to look."""
        return 90.0

    def centre(self):
        return self.look(0.0)

    @property
    def looking_forward(self):
        """Near enough to straight ahead for the reading to count as
        forward clearance."""
        return abs(self.angle) < 10.0


class PowerMonitor(Device):
    """Watches the supply, so a car browning out says so rather than
    behaving strangely."""

    kind = "power"

    @property
    def sagging(self):
        return False

    @property
    def unhappy(self):
        return False


class FrameSource(Device):
    """Produces images. A camera, or anything that can pretend to be one."""

    kind = "camera"

    @abstractmethod
    def capture(self):
        """One frame as a BGR array, or None if there is nothing to give."""


#: Every job a device can fill. Checked when a driver registers, so a typo
#: in a kind is caught at import rather than by a missing sensor at runtime.
KINDS = ("drivetrain", "range", "proximity", "floor", "pan", "power", "camera")


class Context:
    """What a driver is allowed to build itself from.

    Deliberately small. A driver that needs something not in here is reaching
    past the abstraction -- it wants to know about the robot, and the robot
    is exactly what must not be known about down here.
    """

    def __init__(self, link=None, calibration=None, preferences=None):
        #: The serial link to the microcontroller, or None if there is not
        #: one. Drivers for devices wired to that board require it.
        self.link = link
        #: Measured facts about this machine.
        self.calibration = calibration
        #: Choices a human made.
        self.preferences = preferences

    def measured(self, key, fallback=None):
        if self.calibration is None:
            return fallback
        value = self.calibration.get(key)
        return fallback if value is None else value

    def chosen(self, key, fallback=None):
        if self.preferences is None:
            return fallback
        try:
            value = self.preferences.get(key)
        except KeyError:
            # Asking about a setting that no module has declared is a
            # driver asking whether something is configured, not a bug.
            return fallback
        return fallback if value is None else value


class _Driver:
    def __init__(self, kind, name, build, priority, needs_link):
        self.kind = kind
        self.name = name
        self.build = build
        self.priority = priority
        self.needs_link = needs_link

    def suits(self, context):
        if self.needs_link and context.link is None:
            return False
        return True


#: kind -> [driver], best first.
_DRIVERS = {}


def driver(kind, name, priority=0, needs_link=False):
    """Register a way of building a device.

    `priority` breaks ties: a device wired to the microcontroller is
    preferred over one on the Pi's own pins, because when both are possible
    the board is where this robot actually puts its sensors. `needs_link`
    is the common reason a driver cannot be used, declared rather than
    discovered by watching it throw.
    """
    if kind not in KINDS:
        raise ValueError("unknown device kind %r" % kind)

    def register(factory):
        entry = _Driver(kind, name, factory, priority, needs_link)
        drivers = _DRIVERS.setdefault(kind, [])
        drivers[:] = [d for d in drivers if d.name != name] + [entry]
        drivers.sort(key=lambda d: -d.priority)
        return factory

    return register


def catalogue(kind=None):
    """What could be built, for the web page and for the tests that check
    every registered driver honours its contract."""
    kinds = KINDS if kind is None else (kind,)
    return {
        k: [{"name": d.name, "priority": d.priority, "needs_link": d.needs_link}
            for d in _DRIVERS.get(k, [])]
        for k in kinds
    }


def build(kind, context, prefer=None, on_error=None):
    """Build the best available device of this kind, or None.

    None is an answer, not a failure: this robot is expected to run with the
    camera unplugged, with no ESP32, and on a desk with no hardware at all.
    `on_error` is called with (name, exception) for anything that was tried
    and did not work, so the reason reaches the log instead of disappearing.
    """
    if kind not in KINDS:
        raise ValueError("unknown device kind %r" % kind)
    candidates = [d for d in _DRIVERS.get(kind, []) if d.suits(context)]
    if prefer:
        candidates.sort(key=lambda d: (d.name != prefer, -d.priority))
    for entry in candidates:
        try:
            device = entry.build(context)
        except Exception as exc:        # noqa: BLE001 - absent hardware is normal
            if on_error is not None:
                on_error(entry.name, exc)
            continue
        if device is not None:
            return device
    return None
