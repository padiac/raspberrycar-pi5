"""Record a driven path so it can be played back.

Drive the car by hand with recording armed and every command is captured as a
timed segment; replaying walks those segments back out at the same speeds for
the same durations.

This is dead reckoning, and it is worth being clear about the consequence:
there are no wheel encoders, so playback repeats *what was commanded*, not
where the car actually went. Battery sag, carpet, a wheel catching on
something -- any of it makes the replay drift from the original path, and the
error accumulates rather than correcting. It is good for a repeatable little
routine, not for getting back to an exact spot.
"""

import threading
import time
from dataclasses import dataclass

from hal import Velocity

#: Commands shorter than this are noise from a button press, not intent.
MIN_SEGMENT = 0.12
#: Velocities within this of each other are treated as the same command, so a
#: held button becomes one segment instead of a hundred.
SAME = 0.02


@dataclass
class Segment:
    velocity: Velocity
    seconds: float

    def as_dict(self):
        return {**self.velocity.as_dict(), "seconds": round(self.seconds, 2)}


def _alike(a, b):
    return (abs(a.vx - b.vx) < SAME and abs(a.vy - b.vy) < SAME
            and abs(a.omega - b.omega) < SAME)


class PathRecorder:
    def __init__(self):
        self.segments = []
        self.recording = False
        self.lock = threading.RLock()
        self._open = None          # segment being extended
        self._open_seconds = 0.0

    # -- recording ---------------------------------------------------------

    def arm(self):
        with self.lock:
            self.recording = True
            self._open, self._open_seconds = None, 0.0
        return self.status()

    def disarm(self):
        with self.lock:
            self._close()
            self.recording = False
        return self.status()

    def clear(self):
        with self.lock:
            self.segments = []
            self._open, self._open_seconds = None, 0.0
        return self.status()

    def observe(self, velocity, dt):
        """Called once per control tick with whatever is being driven."""
        if not self.recording:
            return
        with self.lock:
            if self._open is not None and _alike(self._open, velocity):
                self._open_seconds += dt
                return
            self._close()
            self._open, self._open_seconds = velocity, dt

    def _close(self):
        if self._open is not None and self._open_seconds >= MIN_SEGMENT:
            self.segments.append(Segment(self._open, self._open_seconds))
        self._open, self._open_seconds = None, 0.0

    # -- reading -----------------------------------------------------------

    def snapshot(self):
        """A stable copy to replay, so recording can continue independently."""
        with self.lock:
            return list(self.segments)

    @property
    def total_seconds(self):
        with self.lock:
            return sum(s.seconds for s in self.segments)

    def status(self):
        with self.lock:
            return {
                "recording": self.recording,
                "segments": len(self.segments),
                "seconds": round(self.total_seconds, 1),
                "moves": [s.as_dict() for s in self.segments[-8:]],
            }
