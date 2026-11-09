"""The two behaviours driven from outside rather than by sensors."""

import time

from hal import Velocity

from .base import Behavior

#: If no command arrives within this long, stop. Without it a browser crash, a
#: phone locking or wifi dropping mid-command would leave the car driving away
#: at whatever it was last told.
COMMAND_TIMEOUT = 0.6

#: How much of a requested turn to actually give, under manual control.
#:
#: This was halved for a while so the recording stayed watchable -- a pan
#: across a room at full rate smears every frame, because the exposure indoors
#: is long enough that anything moving that fast is a streak. Driving forward
#: is fine at any speed, since the scene grows rather than sweeping.
#:
#: Back to full at the driver request: the slider is the one governor now, and
#: a slider at 100 percent that still turns at half is a lie about itself.
TURN_SCALE = 1.0


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
    # Somebody is watching, so the trigger on the pad may reach past the
    # speed limit here. Nowhere else.
    hand_driven = True

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

        # Order matters here, and getting it wrong is invisible until the
        # scale reaches 1.0. Scaling may only ever slow a turn down, so the
        # request caps it -- but the floor may speed it up, because below the
        # duty these motors start turning at the car does not turn at all, and
        # a stick pushed a little should move the car a little rather than
        # produce a stall. So cap first against what was asked for, then raise
        # to the floor. Capping last silently undoes the floor.
        gentle = abs(self.wanted.omega) * TURN_SCALE
        gentle = min(gentle, abs(self.wanted.omega))
        gentle = robot.enough_to_turn(gentle)
        return Velocity(
            vx=self.wanted.vx, vy=self.wanted.vy,
            omega=gentle if self.wanted.omega > 0 else -gentle,
        )

    def status(self):
        age = time.monotonic() - self.last_command if self.last_command else None
        return {"command_age": None if age is None else round(age, 2)}
