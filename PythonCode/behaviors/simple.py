"""The two behaviours driven from outside rather than by sensors."""

import time

from hal import Velocity

from .base import Behavior

#: If no command arrives within this long, stop. Without it a browser crash, a
#: phone locking or wifi dropping mid-command would leave the car driving away
#: at whatever it was last told.
COMMAND_TIMEOUT = 0.6


class IdleBehavior(Behavior):
    name = "idle"
    label = "Stopped"
    description = "Motors off. Camera and telemetry keep running."

    def update(self, robot, world):
        return Velocity.STOP


class ManualBehavior(Behavior):
    name = "manual"
    label = "Manual"
    description = "You drive: touch buttons, WASD or a gamepad."
    requires = ("motors",)

    def __init__(self):
        self.wanted = Velocity.STOP
        self.last_command = 0.0

    def command(self, velocity):
        self.wanted = velocity.clamped()
        self.last_command = time.monotonic()

    def start(self, robot):
        self.wanted = Velocity.STOP
        self.last_command = 0.0

    def update(self, robot, world):
        if not self.wanted.moving:
            return Velocity.STOP
        if time.monotonic() - self.last_command > COMMAND_TIMEOUT:
            self.wanted = Velocity.STOP
            return Velocity.STOP
        return self.wanted

    def status(self):
        age = time.monotonic() - self.last_command if self.last_command else None
        return {"command_age": None if age is None else round(age, 2)}
