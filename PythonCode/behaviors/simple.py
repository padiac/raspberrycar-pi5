"""The two behaviours driven from outside rather than by sensors."""

import time

from hal import Velocity

from .base import Behavior

#: If no command arrives within this long, stop. Without it a browser crash, a
#: phone locking or wifi dropping mid-command would leave the car driving away
#: at whatever it was last told.
COMMAND_TIMEOUT = 0.6

#: Turning is scaled down against driving, because the camera cannot keep up
#: with it. A pan across a room at full rate smears every frame -- the exposure
#: is long enough indoors that anything moving that fast is a streak -- and the
#: recording is unwatchable exactly where something interesting is happening.
#:
#: Driving forward at the same speed is fine: the scene grows rather than
#: sweeping, so the image stays sharp.
TURN_SCALE = 0.5


class IdleBehavior(Behavior):
    name = "idle"
    label = "Stopped"
    description = "Motors off. Camera and telemetry keep running."
    category = "control"

    def update(self, robot, world):
        return Velocity.STOP


class ManualBehavior(Behavior):
    name = "manual"
    label = "Manual"
    description = "You drive: touch buttons, WASD or a gamepad."
    category = "control"
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

        if not self.wanted.omega:
            return self.wanted

        # Halved, but not into nothing. At a limit of 0.40 a full stick comes
        # to exactly the duty these motors start turning at, so scaling
        # without the floor would make the slowest useful turn the fastest
        # impossible one.
        gentle = abs(self.wanted.omega) * TURN_SCALE
        gentle = robot.enough_to_turn(gentle)
        gentle = min(gentle, abs(self.wanted.omega))
        return Velocity(
            vx=self.wanted.vx, vy=self.wanted.vy,
            omega=gentle if self.wanted.omega > 0 else -gentle,
        )

    def status(self):
        age = time.monotonic() - self.last_command if self.last_command else None
        return {"command_age": None if age is None else round(age, 2)}
