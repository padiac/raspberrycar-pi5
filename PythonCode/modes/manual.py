"""Manual driving: the browser, keyboard or gamepad steers."""

import time

from .base import DriveMode

# If no command arrives within this long, cut the motors. Without it a browser
# crash / phone locking / wifi drop mid-command would leave the car driving
# away at its last commanded speed.
COMMAND_TIMEOUT = 0.6


class IdleMode(DriveMode):
    name = "idle"
    label = "Stopped"
    description = "Motors off. Camera and telemetry keep running."

    def update(self, robot, view):
        return 0.0, 0.0


class ManualMode(DriveMode):
    name = "manual"
    label = "Manual"
    description = "You drive, with touch buttons, WASD or a gamepad."
    requires = ("motors",)

    def __init__(self):
        self.throttle = 0.0
        self.steer = 0.0
        self.last_command = 0.0

    def command(self, throttle, steer):
        self.throttle = max(-1.0, min(1.0, float(throttle)))
        self.steer = max(-1.0, min(1.0, float(steer)))
        self.last_command = time.monotonic()

    def start(self, robot):
        self.throttle = self.steer = 0.0
        self.last_command = 0.0

    def update(self, robot, view):
        if self.throttle == 0.0 and self.steer == 0.0:
            return 0.0, 0.0
        if time.monotonic() - self.last_command > COMMAND_TIMEOUT:
            self.throttle = self.steer = 0.0
            return 0.0, 0.0
        return self.throttle, self.steer

    def status(self):
        age = time.monotonic() - self.last_command if self.last_command else None
        return {"command_age": None if age is None else round(age, 2)}
