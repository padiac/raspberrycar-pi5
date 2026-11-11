"""Concrete tasks, written as trees.

Each of these is a few lines of composition over behaviours that already
existed. Nothing in `follow.py` or `patrol.py` changed to make them possible --
that is the point of keeping "how to move" and "what to do" apart.
"""

from ..follow import FollowBehavior
from ..patrol import PatrolBehavior
from .core import Status, TreeBehavior
from .nodes import (
    Fallback,
    Repeat,
    RunBehavior,
    Sequence,
    Timeout,
    Wait,
    fills_frame,
    sees,
)


class ScanBehavior:
    """Turn on the spot. Deliberately dumb -- when to stop is the tree's call."""

    name = "scan"
    label = "Scan"
    description = "Turns in place."
    requires = ("motors",)
    wants_detection = True

    def __init__(self, speed=0.45):
        from hal import Velocity

        self.speed = speed
        self._stop = Velocity.STOP

    def configure(self, **kwargs):
        return self.status()

    def start(self, robot):
        pass

    def stop(self, robot):
        pass

    def update(self, robot, world):
        from hal import Velocity

        return Velocity(omega=self.speed)

    def status(self):
        return {"state": "turning"}

    def overlay(self, frame):
        return frame


def build_fetch(target="person", arrive_fill=0.40, search_seconds=25):
    """Look for something, drive to it, stop.

        Sequence
        ├─ Timeout(25s) -> Scan until seen      "where is it"
        ├─ Approach until it fills the frame    "go there"
        └─ Wait 1s                              "arrived"

    The `Timeout` is what makes this a task rather than a demo: if the target
    never appears the mission fails and stops, instead of spinning all day.
    """

    scan = RunBehavior(
        ScanBehavior(),
        until=lambda ctx, _b: Status.SUCCESS if sees().check(ctx) else None,
        name="Scan for target",
    )

    approach = RunBehavior(
        FollowBehavior(target=target),
        until=lambda ctx, b: (
            Status.SUCCESS if fills_frame(arrive_fill).check(ctx)
            # Give up once the follower itself has: it looks around for a lost
            # target before conceding, and the tree, not the behaviour,
            # decides that conceding counts as failure. This used to read a
            # `lost_frames` counter the follower stopped having when it got a
            # tracker, and the mission died with an AttributeError the first
            # time it reached this leaf.
            else Status.FAILURE if b.search.gave_up
            else None
        ),
        name="Approach target",
    )

    root = Sequence(
        [Timeout(scan, search_seconds), approach, Wait(1.0, name="Arrived")],
        name="Fetch",
    )

    return TreeBehavior(
        name="fetch",
        label="Search and approach",
        description="Spins to find the target, drives to it, stops when close.",
        root=root,
        requires=("motors", "camera", "detector"),
        wants_detection=True,
        blackboard={"target": target},
    )


def build_seek(target="person"):
    """Follow the target when you can see it, wander when you cannot.

        Repeat(forever)
        └─ Fallback
           ├─ Sequence: [sees target] -> Follow it
           └─ Patrol

    `Fallback` re-decides every tick and always retries from the top, so the
    switch is automatic in both directions: walk into view and the car picks
    you up mid-patrol; walk away and it goes back to wandering. Neither
    `FollowBehavior` nor `PatrolBehavior` knows the other exists.
    """

    follow = RunBehavior(FollowBehavior(target=target), name="Follow target")
    patrol = RunBehavior(PatrolBehavior(), name="Patrol")

    root = Repeat(
        Fallback(
            [
                Sequence([sees(), follow], name="If seen, follow"),
                patrol,
            ],
            name="Follow or wander",
        ),
        name="Seek",
    )

    return TreeBehavior(
        name="seek",
        label="Seek and follow",
        description="Follows the target when visible, patrols when it is not.",
        root=root,
        requires=("motors", "camera", "detector"),
        wants_detection=True,
        blackboard={"target": target},
    )


def build_missions():
    return [build_fetch(), build_seek()]
