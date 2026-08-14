"""Find a lane made of dark lines on a light floor.

The method is the original project's: threshold the red channel, then scan a
few rows inward from each edge until a dark pixel turns up. Kept as its own
module because it is *perception* -- it answers "where is the lane relative to
me", and says nothing about what to do about it. The behaviour decides that.
"""

from dataclasses import dataclass, field

import cv2
import numpy as np

NUM_ROWS = 4            # how many heights to sample
ROW_SPACING = 15        # pixels between rows, counting up from the bottom
THRESHOLD = 60          # below this is "dark" == a lane marking
# When only one edge is visible, aim to sit this far from it, as a fraction of
# half the frame width.
SINGLE_EDGE_OFFSET = 0.5


@dataclass
class LaneReading:
    #: -1 .. +1. Negative means the lane centre is to the left of the camera,
    #: i.e. the car has drifted right. None when no lane was found at all.
    offset: float = None
    #: Sampled edge points, for drawing.
    points: list = field(default_factory=list)
    #: How many rows contributed an offset -- a crude confidence.
    rows_seen: int = 0

    @property
    def found(self):
        return self.offset is not None


class LaneFinder:
    def __init__(self, threshold=THRESHOLD, rows=NUM_ROWS, spacing=ROW_SPACING):
        self.threshold = threshold
        self.rows = rows
        self.spacing = spacing

    def find(self, frame):
        if frame is None:
            return LaneReading()

        blurred = cv2.blur(frame, (5, 5))
        # The original used the red channel: their tape was blue or black,
        # which reads as dark in red whatever colour cast the floor has.
        red = blurred[:, :, 2]
        _, binary = cv2.threshold(red, self.threshold, 255, cv2.THRESH_BINARY)

        height, width = binary.shape
        half = width // 2
        points, offsets = [], []

        for i in range(self.rows):
            y = height - self.spacing * (i + 1)
            if y < 0:
                break
            row = binary[y]

            dark_left = np.where(row[:half] == 0)[0]
            dark_right = np.where(row[half:] == 0)[0]

            # Innermost dark pixel on each side is the lane edge.
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
                target = half - half * SINGLE_EDGE_OFFSET
                offsets.append((left_x - target) / half)
            elif right_x is not None:
                target = half + half * SINGLE_EDGE_OFFSET
                offsets.append((right_x - target) / half)

        if not offsets:
            return LaneReading(points=points)

        # Nearer rows matter more; they are what the car is about to reach.
        weights = np.linspace(1.0, 0.5, len(offsets))
        offset = float(np.average(offsets, weights=weights))
        return LaneReading(offset=offset, points=points, rows_seen=len(offsets))


def draw(frame, reading):
    for x, y in reading.points:
        cv2.circle(frame, (x, y), 4, (0, 255, 255), -1)
    height, width = frame.shape[:2]
    centre = width // 2
    cv2.line(frame, (centre, height), (centre, height - 60), (255, 255, 0), 1)
    if reading.found:
        target = int(centre + reading.offset * centre)
        cv2.line(frame, (target, height), (target, height - 60), (0, 0, 255), 2)
    return frame
