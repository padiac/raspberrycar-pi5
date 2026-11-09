"""What the robot believes about the world right now.

A single `WorldState` is built once per control tick and handed to the active
behaviour. Behaviours read it and nothing else -- they never touch a sensor
directly -- which is what lets them be swapped, tested or replayed without any
hardware attached.
"""

from dataclasses import dataclass, field

from hal.sensors import Floor, Proximity


@dataclass(frozen=True)
class Detection:
    """One thing the detector found."""

    label: str
    confidence: float
    box: tuple          # (x1, y1, x2, y2) in frame pixels
    class_id: int

    @property
    def centre_x(self):
        return (self.box[0] + self.box[2]) / 2

    @property
    def centre_y(self):
        return (self.box[1] + self.box[3]) / 2

    @property
    def width(self):
        return self.box[2] - self.box[0]

    @property
    def height(self):
        return self.box[3] - self.box[1]

    @property
    def area(self):
        return self.width * self.height

    def as_dict(self):
        return {"label": self.label, "conf": round(self.confidence, 2)}


@dataclass
class WorldState:
    dt: float = 0.05                    # seconds since the previous tick
    frame: object = None                # BGR numpy array, or None if camera off
    frame_size: tuple = (0, 0)          # (width, height)
    detections: list = field(default_factory=list)   # list[Detection]
    range_cm: float = None              # ultrasonic; None unknown, <0 clear
    range_expected: bool = False        # a rangefinder is fitted, so None
                                        # means it went quiet, not absent
    range_angle: float = None           # where the head was looking, degrees
                                        # off forward, positive right
    range_forward: bool = True          # was that near enough to straight
                                        # ahead to count as clearance?
    forward_cm: float = None            # the most recent reading that WAS
                                        # taken looking forward
    forward_age: float = None           # how many seconds ago, or None if
                                        # there has never been one
    range_blind: bool = False           # the head is looking at part of the
                                        # car, so this says nothing about the
                                        # room beyond it
    proximity: Proximity = None         # side infrared
    floor: Floor = None                 # downward line sensors
    turned: float = 0.0                 # cumulative rotation in degrees,
                                        # unwrapped, signed, positive right
    turn_rate: float = None             # degrees per second right now, or
                                        # None when the IMU is not answering
                                        # -- which is NOT the same as zero
    turned: float = 0.0                 # cumulative rotation in degrees,
                                        # unwrapped, signed, positive right
    turn_rate: float = None             # degrees per second right now, or
                                        # None when the IMU is not answering
                                        # -- which is NOT the same as zero
    bumped: bool = False                # the accelerometer felt a jolt
                                        # recently; the firmware holds it
                                        # long enough to survive a 20Hz poll

    def __post_init__(self):
        """Fill in the forward reading when the caller only gave a range.

        A world built by hand -- in a test, or by a chassis with no servo to
        point anywhere else -- says how far away something is and nothing
        about where the head was, because there was only one place it could
        have been. Reading that as "has never seen anything forward" switches
        the whole clearance check off without a word, which is the same shape
        of mistake as the one where a timed-out ping and a silent board became
        the same value. Two fields that nearly mean the same thing want the
        near case handled here, once, rather than remembered everywhere.
        """
        if (self.forward_cm is None and self.range_forward
                and self.range_cm is not None):
            self.forward_cm = self.range_cm
            self.forward_age = 0.0

    # -- convenience the behaviours would otherwise all reimplement ---------

    def largest(self, label):
        """Nearest instance of `label`, judged by box area. None if absent."""
        best = None
        for det in self.detections:
            if det.label.lower() != label.lower():
                continue
            if best is None or det.area > best.area:
                best = det
        return best

    def labels(self):
        return sorted({d.label for d in self.detections})

    @property
    def has_frame(self):
        return self.frame is not None and self.frame_size[0] > 0
