"""Autonomous wandering that avoids obstacles.

Ported from the original main_obstacle_avoidance.py. The original picked one
of a few fixed speeds; here the same decisions come out as continuous
throttle/steer so they blend rather than jerk between states.
"""

import time

from .base import DriveMode

TURN_HOLD = 0.5          # keep turning this long after deciding to, so the car
                         # actually clears the obstacle instead of dithering
STOP_DISTANCE = 20.0     # cm: closer than this, turn on the spot
SLOW_DISTANCE = 100.0    # cm: below this, scale speed with room ahead


class AvoidMode(DriveMode):
    name = "avoid"
    label = "Obstacle avoidance"
    description = "Drives forward on its own, turning away from whatever it finds."
    requires = ("motors",)

    def __init__(self):
        self.turn_until = 0.0
        self.turn_dir = 0.0
        self.reason = "idle"

    def start(self, robot):
        self.turn_until = 0.0
        self.turn_dir = 0.0

    def update(self, robot, view):
        now = time.monotonic()

        # Still completing a turn started on an earlier tick.
        if now < self.turn_until:
            return 0.0, self.turn_dir

        left_clear, right_clear = 1, 1
        if view.obstacle:
            left_clear, right_clear = view.obstacle  # 0 means blocked

        # Side IR sensors take priority: they see things the forward-facing
        # ultrasonic cone misses entirely.
        if not left_clear and right_clear:
            self.reason = "obstacle left, turning right"
            self._hold(now, +1.0)
            return 0.0, 1.0
        if left_clear and not right_clear:
            self.reason = "obstacle right, turning left"
            self._hold(now, -1.0)
            return 0.0, -1.0
        if not left_clear and not right_clear:
            self.reason = "blocked both sides, backing up"
            return -0.5, 0.0

        distance = view.distance_cm
        if distance is None or distance <= 0:
            self.reason = "no range reading, creeping"
            return 0.3, 0.0

        if distance < STOP_DISTANCE:
            self.reason = f"wall at {distance:.0f}cm, turning"
            self._hold(now, -1.0)
            return 0.0, -1.0
        if distance < SLOW_DISTANCE:
            # Ease off as the wall approaches instead of running flat out.
            speed = 0.4 + 0.5 * (distance - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE)
            self.reason = f"clear for {distance:.0f}cm"
            return speed, 0.0

        self.reason = "open road"
        return 0.9, 0.0

    def _hold(self, now, direction):
        self.turn_until = now + TURN_HOLD
        self.turn_dir = direction

    def status(self):
        return {"reason": self.reason}
