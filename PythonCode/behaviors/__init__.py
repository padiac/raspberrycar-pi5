"""Behaviours: what the robot is trying to do.

Two kinds live here, and the robot cannot tell them apart:

* a **behaviour** is a policy -- read the world, return a velocity, forever
  (`manual`, `patrol`, `follow`, `lane`);
* a **mission** is a behaviour tree composing several of those into a task
  with a beginning and an end (`fetch`, `seek`), built in `tree/missions.py`.

To add either, write it and list it in `build_behaviors()`. The robot, the API
and the web UI all read that registry, so nothing else needs changing.
"""

from .base import Behavior
from .follow import FollowBehavior
from .lane import LaneBehavior
from .patrol import PatrolBehavior
from .simple import IdleBehavior, ManualBehavior
from .tree import Status, TreeBehavior, build_missions

__all__ = [
    "Behavior", "FollowBehavior", "IdleBehavior", "LaneBehavior",
    "ManualBehavior", "PatrolBehavior", "Status", "TreeBehavior",
    "build_behaviors", "build_missions",
]


def build_behaviors():
    """One instance of each behaviour, keyed by name. Order sets the UI order."""
    entries = [
        IdleBehavior(),
        ManualBehavior(),
        PatrolBehavior(),
        FollowBehavior(),
        LaneBehavior(),
        *build_missions(),
    ]
    return {b.name: b for b in entries}
