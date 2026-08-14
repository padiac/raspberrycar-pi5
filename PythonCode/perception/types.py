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
    range_cm: float = None              # ultrasonic, None if unavailable
    proximity: Proximity = None         # side infrared
    floor: Floor = None                 # downward line sensors

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
