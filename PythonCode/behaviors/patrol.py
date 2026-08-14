"""Wander around without hitting anything.

This is what the original called `main_obstacle_avoidance.py`, renamed to say
what it actually is. "Obstacle avoidance" was never a goal -- it is a
constraint, and it now lives in the safety layer where it applies to every
behaviour. What is left here is the *intent*: keep exploring.

The division of labour is worth noticing, because it is the whole point of the
refactor. This behaviour just asks to drive forward. When something is in the
way the safety layer clamps that to zero on its own; this code then sees that
it is not making progress and decides which way to turn -- because choosing a
new direction is intent, and intent does not belong in the safety layer.
"""

import random
import time

from hal import Velocity

from .base import Behavior

CRUISE = 0.8
TURN_SPEED = 0.7
#: Keep turning at least this long once committed, so the car actually clears
#: the obstacle instead of twitching left and right on the spot.
TURN_HOLD = 0.6
#: How long forward progress must be blocked before deciding to turn.
STUCK_AFTER = 0.25


class PatrolBehavior(Behavior):
    name = "patrol"
    label = "Patrol"
    description = "Drives around on its own, turning away when the way is blocked."
    requires = ("motors",)

    def __init__(self):
        self.turn_until = 0.0
        self.turn_direction = 0.0
        self.blocked_since = None
        self.state = "idle"

    def start(self, robot):
        self.turn_until = 0.0
        self.turn_direction = 0.0
        self.blocked_since = None

    def update(self, robot, world):
        now = time.monotonic()

        # Mid-turn from an earlier tick: see it through.
        if now < self.turn_until:
            self.state = f"turning {'right' if self.turn_direction > 0 else 'left'}"
            return Velocity(vx=0.0, omega=self.turn_direction * TURN_SPEED)

        # Ask the safety layer what happened to our last request rather than
        # reading the sensors again here -- it already decided, and duplicating
        # the thresholds is how the two end up disagreeing.
        blocked = robot.safety.last.allowed.vx <= 0.0 < robot.safety.last.requested.vx

        if blocked:
            if self.blocked_since is None:
                self.blocked_since = now
            elif now - self.blocked_since > STUCK_AFTER:
                self.blocked_since = None
                self.turn_direction = self._choose_direction(world)
                self.turn_until = now + TURN_HOLD
                self.state = "blocked, picking a way round"
                return Velocity(vx=0.0, omega=self.turn_direction * TURN_SPEED)
        else:
            self.blocked_since = None

        self.state = "cruising"
        return Velocity(vx=CRUISE)

    def _choose_direction(self, world):
        """Turn away from whichever side is occupied; guess if both or neither."""
        proximity = world.proximity
        if proximity is not None:
            if proximity.left and not proximity.right:
                return +1.0
            if proximity.right and not proximity.left:
                return -1.0
        # Nothing to go on: pick at random so the car cannot get stuck in a
        # corner repeating the same failed turn.
        return random.choice((-1.0, 1.0))

    def status(self):
        return {"state": self.state}
