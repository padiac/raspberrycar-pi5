"""Hardware abstraction layer.

Everything that touches a pin, a bus or the camera lives here, and nothing here
knows what the robot is trying to *do*. Wiring details -- pin numbers, sensor
polarity, chassis geometry -- stop at this boundary.
"""

from .base import (
    Context,
    Device,
    FloorSensor,
    FrameSource,
    KINDS,
    PanMount,
    PowerMonitor,
    ProximitySensor,
    RangeSensor,
    build,
    catalogue,
    driver,
)
from .drivetrain import (
    DifferentialDrive,
    Drivetrain,
    MecanumDrive,
    Velocity,
    build_drivetrain,
)

# Importing these registers their drivers, which is the whole mechanism:
# the registry is populated by the modules existing, so adding a piece of
# hardware is adding a file and naming it here.
from . import camera, sensors, supply        # noqa: E402,F401

__all__ = [
    "Context", "Device", "DifferentialDrive", "Drivetrain", "FloorSensor",
    "FrameSource", "KINDS", "MecanumDrive", "PanMount", "PowerMonitor",
    "ProximitySensor", "RangeSensor", "Velocity", "build", "build_drivetrain",
    "catalogue", "driver",
]
