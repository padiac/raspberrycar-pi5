"""Perception: sensors and camera in, a description of the world out.

Nothing here decides anything. Every module answers a question about what is
out there -- what objects are visible, where the lane is -- and leaves what to
do about it to the behaviours.
"""

from .types import Detection, WorldState

__all__ = ["Detection", "WorldState"]
