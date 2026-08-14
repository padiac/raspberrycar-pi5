"""Drive between lane lines.

Where the lane *is* is perception's problem (see perception/lane.py); this file
only decides what to do about the offset it reports. The original mapped the
same information onto a handful of discrete manoeuvres, so the car switched
between hard turns; a continuous error lets it ease into bends instead.
"""

from perception.lane import LaneFinder
from perception.lane import draw as draw_lane

from control import PID
from hal import Velocity

from .base import Behavior

CRUISE = 0.45
MAX_TURN = 0.9
#: How hard to cast about when both lane edges have disappeared.
SEARCH_TURN = 0.5
SEARCH_CRAWL = 0.2


class LaneBehavior(Behavior):
    name = "lane"
    label = "Lane following"
    description = "Drives between dark lane lines on a light floor."
    requires = ("motors", "camera")

    def __init__(self):
        self.finder = LaneFinder()
        self.pid = PID(kp=1.8, kd=0.25, setpoint=0.0)
        self.reading = None
        self.last_offset = 0.0
        self.state = "waiting"

    def configure(self, **kwargs):
        if "threshold" in kwargs:
            try:
                self.finder.threshold = max(5, min(250, int(kwargs["threshold"])))
            except (TypeError, ValueError):
                pass
        return self.status()

    def start(self, robot):
        self.pid.reset()
        self.reading = None
        self.last_offset = 0.0

    def update(self, robot, world):
        if not world.has_frame:
            self.state = "no camera"
            return Velocity.STOP

        reading = self.finder.find(world.frame)
        self.reading = reading

        if not reading.found:
            # Lost both edges. Keep turning the way the lane last went, which
            # usually picks it up again on the far side of a bend.
            self.state = "lane lost, searching"
            direction = 1.0 if self.last_offset > 0 else -1.0
            return Velocity(vx=SEARCH_CRAWL, omega=direction * SEARCH_TURN)

        self.last_offset = reading.offset
        omega = -self.pid.update(reading.offset, world.dt)
        omega = max(-MAX_TURN, min(MAX_TURN, omega))

        # Ease off the faster we are turning, so bends do not throw the car
        # out of the lane.
        vx = CRUISE * (1.0 - 0.5 * min(1.0, abs(omega)))
        self.state = f"offset {reading.offset:+.2f} ({reading.rows_seen} rows)"
        return Velocity(vx=vx, omega=omega)

    def overlay(self, frame):
        return draw_lane(frame, self.reading) if self.reading else frame

    def status(self):
        return {
            "offset": None if not self.reading or not self.reading.found
            else round(self.reading.offset, 3),
            "threshold": self.finder.threshold,
            "state": self.state,
        }
