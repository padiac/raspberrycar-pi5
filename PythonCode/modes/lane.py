"""Follow a lane marked by dark lines on a light floor.

Ported from the original main_lane_tracking.py. The detection is the same idea
-- threshold the red channel, then scan a few rows inward from each side until
a dark pixel turns up -- but the original mapped the result onto a handful of
discrete manoeuvres. Here the two edges give a continuous centring error, so
the car eases into bends instead of switching between hard turns.
"""

import cv2
import numpy as np

from .base import DriveMode

NUM_ROWS = 4             # how many heights to sample the lane at
ROW_SPACING = 15         # pixels between sampled rows, counting up from bottom
THRESHOLD = 60           # below this is "dark" == lane marking
CRUISE = 0.45
MAX_STEER = 0.9
SEARCH_STEER = 0.5       # how hard to cast about when both edges vanish


class LaneMode(DriveMode):
    name = "lane"
    label = "Lane following"
    description = "Drives between dark lane lines on a light floor."
    requires = ("motors", "camera")

    def __init__(self):
        self.points = []
        self.offset = 0.0
        self.state = "waiting"
        self.last_offset = 0.0

    def start(self, robot):
        self.points = []
        self.offset = 0.0
        self.last_offset = 0.0

    def _scan(self, frame):
        """Return (offset -1..1, debug points). offset>0 means drift left."""
        blurred = cv2.blur(frame, (5, 5))
        # The original used the red channel: their tape was blue/black, which
        # is dark in red regardless of the floor's colour cast.
        red = blurred[:, :, 2]
        _, binary = cv2.threshold(red, THRESHOLD, 255, cv2.THRESH_BINARY)

        height, width = binary.shape
        half = width // 2
        points = []
        offsets = []

        for i in range(NUM_ROWS):
            y = height - ROW_SPACING * (i + 1)
            if y < 0:
                break
            row = binary[y]

            dark_left = np.where(row[:half] == 0)[0]
            dark_right = np.where(row[half:] == 0)[0]

            left_x = int(dark_left.max()) if dark_left.size else None
            right_x = int(dark_right.min()) + half if dark_right.size else None

            if left_x is not None:
                points.append((left_x, y))
            if right_x is not None:
                points.append((right_x, y))

            if left_x is not None and right_x is not None:
                centre = (left_x + right_x) / 2
                offsets.append((centre - half) / half)
            elif left_x is not None:
                # Only the left edge: hug it at a fixed distance.
                offsets.append((left_x - (half - width * 0.25)) / half)
            elif right_x is not None:
                offsets.append((right_x - (half + width * 0.25)) / half)

        if not offsets:
            return None, points
        # Nearer rows matter more -- they're what the car is about to reach.
        weights = np.linspace(1.0, 0.5, len(offsets))
        return float(np.average(offsets, weights=weights)), points

    def update(self, robot, view):
        if view.frame is None:
            self.state = "no camera"
            return 0.0, 0.0

        offset, points = self._scan(view.frame)
        self.points = points

        if offset is None:
            # Lost both edges: keep turning the way the lane last went, which
            # is usually enough to pick it up again on the next bend.
            self.state = "lane lost, searching"
            direction = 1.0 if self.last_offset > 0 else -1.0
            return 0.2, direction * SEARCH_STEER

        self.offset = offset
        self.last_offset = offset
        steer = max(-MAX_STEER, min(MAX_STEER, offset * 1.8))
        # Slow down the harder we're turning, so bends don't throw the car out.
        throttle = CRUISE * (1.0 - 0.5 * min(1.0, abs(steer)))
        self.state = f"offset {offset:+.2f}"
        return throttle, steer

    def overlay(self, frame):
        for x, y in self.points:
            cv2.circle(frame, (x, y), 4, (0, 255, 255), -1)
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2, h), (w // 2, h - 60), (255, 255, 0), 1)
        if self.points:
            target = int(w // 2 + self.offset * (w // 2))
            cv2.line(frame, (target, h), (target, h - 60), (0, 0, 255), 2)
        return frame

    def status(self):
        return {"offset": round(self.offset, 3), "state": self.state}
