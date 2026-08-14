"""Driving modes.

To add one: subclass DriveMode, then list it in build_modes() below. Nothing
else needs to change -- the robot and the web UI pick it up from the registry.
"""

from .avoid import AvoidMode
from .base import DriveMode, Perception
from .follow import FollowMode
from .lane import LaneMode
from .manual import IdleMode, ManualMode

__all__ = [
    "AvoidMode", "DriveMode", "FollowMode", "IdleMode", "LaneMode",
    "ManualMode", "Perception", "build_modes",
]


def build_modes():
    """Fresh instances of every mode, keyed by name. Order sets the UI order."""
    return {m.name: m for m in (IdleMode(), ManualMode(), AvoidMode(), FollowMode(), LaneMode())}
