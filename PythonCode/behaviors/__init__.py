"""Behaviours: what the robot is trying to do.

To add one, subclass `Behavior` and list it in `build_behaviors()`. The robot,
the API and the web UI all read that registry, so nothing else needs changing.
"""

from .base import Behavior
from .composite import CompositeBehavior, ScanBehavior, Stage, build_search_and_approach
from .follow import FollowBehavior
from .lane import LaneBehavior
from .patrol import PatrolBehavior
from .simple import IdleBehavior, ManualBehavior

__all__ = [
    "Behavior", "CompositeBehavior", "FollowBehavior", "IdleBehavior",
    "LaneBehavior", "ManualBehavior", "PatrolBehavior", "ScanBehavior",
    "Stage", "build_behaviors", "build_search_and_approach",
]


def build_behaviors():
    """One instance of each behaviour, keyed by name. Order sets the UI order."""
    behaviors = [
        IdleBehavior(),
        ManualBehavior(),
        PatrolBehavior(),
        FollowBehavior(),
        build_search_and_approach(),
        LaneBehavior(),
    ]
    return {b.name: b for b in behaviors}
