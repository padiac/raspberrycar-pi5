"""Behaviour trees: composing behaviours into tasks.

Read `core.py` first -- it is short, and the three-status rule it describes is
the whole idea. `nodes.py` is the vocabulary, `missions.py` the examples.
"""

from .core import Context, Node, Status, Tick, TreeBehavior
from .missions import build_fetch, build_missions, build_seek
from .nodes import (
    Condition,
    Fallback,
    Inverter,
    Move,
    Parallel,
    Repeat,
    Retry,
    RunBehavior,
    Sequence,
    Succeeder,
    Timeout,
    Wait,
    closer_than,
    fills_frame,
    sees,
)

__all__ = [
    "Condition", "Context", "Fallback", "Inverter", "Move", "Node", "Parallel",
    "Repeat", "Retry", "RunBehavior", "Sequence", "Status", "Succeeder",
    "Tick", "Timeout", "TreeBehavior", "Wait", "build_fetch", "build_missions",
    "build_seek", "closer_than", "fills_frame", "sees",
]
