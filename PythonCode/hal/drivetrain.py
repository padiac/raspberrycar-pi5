"""How a velocity request becomes wheel speeds.

Everything above this file speaks one language: a `Velocity` of
(vx, vy, omega). Behaviours produce one, the safety layer filters it, and a
`Drivetrain` turns it into whatever the wheels need. Swapping the chassis means
swapping the Drivetrain subclass and nothing else.

Two are implemented:

* `DifferentialDrive` -- two driven sides, like the original code assumed. Two
  degrees of freedom: it can drive and it can turn, but it cannot move
  sideways, so `vy` is ignored.
* `MecanumDrive` -- four independently driven mecanum wheels. Three degrees of
  freedom (holonomic): it can drive, strafe and rotate at the same time, in any
  combination. This is what the actual chassis has.

The pin numbers below are placeholders inherited from upstream; the real motor
driver board has not arrived yet. Set the wiring in one place here and nothing
above needs to know.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass

from gpiozero import Motor

from . import gpio  # noqa: F401 - selects the right gpiochip on import


def clamp(value, low=-1.0, high=1.0):
    return max(low, min(high, float(value)))


@dataclass(frozen=True)
class Velocity:
    """A movement request, each component -1.0 ~ 1.0.

    vx    forward (+) / backward (-)
    vy    strafe left (-) / right (+)   -- mecanum only, ignored on 2-DOF
    omega turn left (-) / right (+)
    """

    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0

    STOP = None  # filled in below; a shared "no motion" instance

    def clamped(self):
        return Velocity(clamp(self.vx), clamp(self.vy), clamp(self.omega))

    def scaled(self, factor):
        return Velocity(self.vx * factor, self.vy * factor, self.omega * factor)

    @property
    def moving(self):
        return bool(self.vx or self.vy or self.omega)

    def as_dict(self):
        return {
            "vx": round(self.vx, 3),
            "vy": round(self.vy, 3),
            "omega": round(self.omega, 3),
        }


Velocity.STOP = Velocity()


def _normalise(wheels):
    """Scale wheel speeds down together if any exceeds full power.

    Clipping each wheel on its own would quietly change the direction of
    travel: a request to drive forward while turning hard would come out as
    driving straight, because both sides would clip to the same value.
    """
    peak = max(1.0, max(abs(w) for w in wheels))
    return [w / peak for w in wheels]


class Drivetrain(ABC):
    """A chassis. Takes a Velocity, drives the wheels."""

    #: How many of (vx, vy, omega) this chassis can actually honour. The UI
    #: uses it to decide whether to offer strafe controls.
    dof = 2
    #: Human-readable, for the web UI.
    label = "drivetrain"
    #: Names of the wheels, in the order `wheel_speeds` returns them.
    wheel_names = ()

    def __init__(self):
        self.last = Velocity.STOP
        self.speeds = ()
        #: Every Motor this chassis owns, so the pins can be handed back when
        #: swapping to a different chassis at runtime.
        self.motors = ()

    @abstractmethod
    def wheel_speeds(self, velocity):
        """Inverse kinematics: Velocity -> per-wheel speeds, each -1..1."""

    @abstractmethod
    def _drive(self, speeds):
        """Push already-computed speeds to the motors."""

    def close(self):
        """Release the GPIO pins. Required before building another chassis."""
        try:
            self.stop()
        except Exception:  # noqa: BLE001 - closing must not depend on the motors working
            pass
        for motor in self.motors:
            try:
                motor.close()
            except Exception:  # noqa: BLE001
                pass
        self.motors = ()

    def apply(self, velocity):
        velocity = velocity.clamped()
        speeds = _normalise(self.wheel_speeds(velocity))
        self.last = velocity
        self.speeds = tuple(round(s, 3) for s in speeds)
        self._drive(speeds)
        return self.speeds

    def stop(self):
        self.apply(Velocity.STOP)

    def status(self):
        return {
            "label": self.label,
            "dof": self.dof,
            "wheels": dict(zip(self.wheel_names, self.speeds)),
        }


class DifferentialDrive(Drivetrain):
    """Two driven sides. Cannot strafe, so vy is discarded."""

    dof = 2
    label = "differential (2 motors)"
    wheel_names = ("left", "right")

    # BCM pin numbers, inherited from the upstream code.
    LEFT = (18, 23)     # (forward, backward)
    RIGHT = (24, 25)

    def __init__(self):
        super().__init__()
        self.left = Motor(forward=self.LEFT[0], backward=self.LEFT[1])
        self.right = Motor(forward=self.RIGHT[0], backward=self.RIGHT[1])
        self.motors = (self.left, self.right)

    def wheel_speeds(self, v):
        # vy deliberately unused: this chassis has no sideways axis.
        return [v.vx + v.omega, v.vx - v.omega]

    def _drive(self, speeds):
        self.left.value = clamp(speeds[0])
        self.right.value = clamp(speeds[1])


class MecanumDrive(Drivetrain):
    """Four mecanum wheels: holonomic, so all three axes work at once.

    The rollers on a mecanum wheel sit at 45 degrees, so each wheel pushes
    diagonally. Summing four diagonal pushes gives motion in any direction
    without the chassis having to turn to face it, which is what makes
    sideways travel possible.
    """

    dof = 3
    label = "mecanum (4 motors)"
    wheel_names = ("front_left", "front_right", "rear_left", "rear_right")

    # Placeholders until the real driver board arrives -- these are the only
    # lines that need editing once the wiring is known.
    FRONT_LEFT = (18, 23)
    FRONT_RIGHT = (24, 25)
    REAR_LEFT = (5, 6)
    REAR_RIGHT = (13, 19)

    def __init__(self):
        super().__init__()
        self.front_left = Motor(forward=self.FRONT_LEFT[0], backward=self.FRONT_LEFT[1])
        self.front_right = Motor(forward=self.FRONT_RIGHT[0], backward=self.FRONT_RIGHT[1])
        self.rear_left = Motor(forward=self.REAR_LEFT[0], backward=self.REAR_LEFT[1])
        self.rear_right = Motor(forward=self.REAR_RIGHT[0], backward=self.REAR_RIGHT[1])
        self.motors = (self.front_left, self.front_right, self.rear_left, self.rear_right)

    def wheel_speeds(self, v):
        # Standard mecanum inverse kinematics. vy flips sign between the left
        # and right pairs, and between front and rear, because that is how the
        # roller angles are mirrored across the chassis.
        return [
            v.vx - v.vy - v.omega,   # front left
            v.vx + v.vy + v.omega,   # front right
            v.vx + v.vy - v.omega,   # rear left
            v.vx - v.vy + v.omega,   # rear right
        ]

    def _drive(self, speeds):
        for motor, speed in zip(
            (self.front_left, self.front_right, self.rear_left, self.rear_right), speeds
        ):
            motor.value = clamp(speed)


#: Which chassis to build. The car is mecanum, but until the four-motor board
#: is wired the two-motor differential model is what actually matches reality,
#: so that stays the default. Override with CAR_DRIVETRAIN=mecanum.
DRIVETRAINS = {"differential": DifferentialDrive, "mecanum": MecanumDrive}


def build_drivetrain(kind=None):
    import os

    kind = kind or os.environ.get("CAR_DRIVETRAIN", "differential")
    try:
        factory = DRIVETRAINS[kind]
    except KeyError:
        raise ValueError(
            f"unknown drivetrain '{kind}', expected one of {sorted(DRIVETRAINS)}"
        ) from None
    return factory()
