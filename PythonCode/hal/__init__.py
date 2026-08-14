"""Hardware abstraction layer.

Everything that touches a pin, a bus or the camera lives here, and nothing here
knows what the robot is trying to *do*. Wiring details -- pin numbers, sensor
polarity, chassis geometry -- stop at this boundary.
"""

from .drivetrain import (
    DifferentialDrive,
    Drivetrain,
    MecanumDrive,
    Velocity,
    build_drivetrain,
)

__all__ = [
    "DifferentialDrive", "Drivetrain", "MecanumDrive", "Velocity",
    "build_drivetrain",
]
