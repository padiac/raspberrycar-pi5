"""Building a task out of several behaviours.

The behaviours next door each do one thing forever. Real jobs are not like
that -- "go and find a bottle" is *search, then approach, then stop*, and which
one should be running depends on what has happened so far.

`CompositeBehavior` is a small state machine over other behaviours. Each stage
names a behaviour to run and a rule for when to move on. Because a composite is
itself a `Behavior`, the robot cannot tell the difference, the web UI lists it
alongside the rest, and composites can nest.

`SearchAndApproach` below is a worked example: spin on the spot until the
target appears, follow it in, stop when close enough.
"""

from dataclasses import dataclass

from hal import Velocity

from .base import Behavior
from .follow import FollowBehavior


@dataclass
class Stage:
    """One step: a behaviour to run, and when to leave it."""

    name: str
    behavior: Behavior
    #: (robot, world, behavior) -> name of the next stage, or None to stay.
    transition: object = None
    #: Shown in telemetry so it is obvious what the task is doing.
    note: str = ""


class CompositeBehavior(Behavior):
    name = "composite"
    label = "Composite"

    def __init__(self, stages, first=None):
        self.stages = {stage.name: stage for stage in stages}
        self.first = first or stages[0].name
        self.current = self.first
        self.history = []

    @property
    def stage(self):
        return self.stages[self.current]

    def _enter(self, robot, name):
        if name == self.current:
            return
        self.stage.behavior.stop(robot)
        self.current = name
        self.history.append(name)
        del self.history[:-6]
        self.stage.behavior.start(robot)

    def start(self, robot):
        self.current = self.first
        self.history = [self.first]
        self.stage.behavior.start(robot)

    def stop(self, robot):
        self.stage.behavior.stop(robot)

    def update(self, robot, world):
        stage = self.stage
        velocity = stage.behavior.update(robot, world)

        if stage.transition is not None:
            nxt = stage.transition(robot, world, stage.behavior)
            if nxt and nxt in self.stages:
                self._enter(robot, nxt)
                # Run the new stage this same tick so a completed stage does
                # not leave the car coasting on the old one's last command.
                velocity = self.stage.behavior.update(robot, world)

        return velocity

    def overlay(self, frame):
        return self.stage.behavior.overlay(frame)

    def status(self):
        return {
            "stage": self.current,
            "note": self.stage.note,
            "path": " > ".join(self.history),
            **{f"{self.current}.{k}": v for k, v in self.stage.behavior.status().items()},
        }

    def configure(self, **kwargs):
        # Settings are forwarded to every stage that understands them, so
        # "target" reaches both the search and the approach stage at once.
        for stage in self.stages.values():
            stage.behavior.configure(**kwargs)
        return self.status()


class ScanBehavior(Behavior):
    """Rotate on the spot, looking for a named object."""

    name = "scan"
    label = "Scan"
    description = "Turns in place until the target comes into view."
    requires = ("motors", "camera", "detector")
    wants_detection = True

    def __init__(self, target="person", speed=0.45):
        self.target = target
        self.speed = speed
        self.found = False

    def configure(self, **kwargs):
        if kwargs.get("target"):
            self.target = str(kwargs["target"]).strip().lower()
        return self.status()

    def start(self, robot):
        self.found = False

    def update(self, robot, world):
        self.found = world.largest(self.target) is not None
        if self.found:
            return Velocity.STOP
        return Velocity(omega=self.speed)

    def status(self):
        return {"target": self.target, "found": self.found}


def build_search_and_approach(target="person", arrive_fill=0.40):
    """Spin until the target is visible, drive to it, then stop.

    `arrive_fill` is how much of the frame height the target must fill before
    the task calls itself done -- a stand-in for "close enough", since a single
    camera cannot measure distance directly.
    """

    scan = ScanBehavior(target=target)
    approach = FollowBehavior(target=target)

    def leave_scan(robot, world, behavior):
        return "approach" if world.largest(behavior.target) else None

    def leave_approach(robot, world, behavior):
        found = world.largest(behavior.target)
        if found is None:
            # Lost it for good: go back to looking rather than sitting still.
            return "scan" if behavior.lost_frames > 12 else None
        _width, height = world.frame_size
        if height and found.height / height >= arrive_fill:
            return "arrived"
        return None

    def leave_arrived(robot, world, behavior):
        return None

    class Arrived(Behavior):
        name = "arrived"
        label = "Arrived"

        def update(self, robot, world):
            return Velocity.STOP

        def status(self):
            return {"done": True}

    composite = CompositeBehavior(
        [
            Stage("scan", scan, leave_scan, "turning to look for the target"),
            Stage("approach", approach, leave_approach, "driving towards it"),
            Stage("arrived", Arrived(), leave_arrived, "close enough, holding"),
        ]
    )
    composite.name = "fetch"
    composite.label = "Search and approach"
    composite.description = "Spins to find the target, drives to it, stops when close."
    composite.requires = ("motors", "camera", "detector")
    composite.wants_detection = True
    return composite
