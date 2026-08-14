"""Behaviour trees: how a task is assembled out of behaviours.

A behaviour decides how to move *right now*. A task is a shape over time --
"look for it, then drive to it, then stop" -- and a state machine written by
hand for each task gets unmaintainable fast. Behaviour trees are the standard
answer; ROS 2's Nav2 orchestrates navigation with one, so the concepts here
carry over directly.

The whole idea is one rule. Every node, when ticked, returns one of three
things:

    SUCCESS   I am done and it worked
    FAILURE   I am done and it did not
    RUNNING   ask me again next tick

Everything else -- sequences, fallbacks, retries, timeouts -- is just a
different rule for combining children's answers. Leaves do the actual work.

The one addition here is that a tick also carries a `Velocity`, because in this
robot ticking the tree *is* the control loop: the tree is asked what to do, and
its answer goes to the safety layer and then the wheels.
"""

from dataclasses import dataclass, field
from enum import Enum

from hal import Velocity


class Status(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


@dataclass(frozen=True)
class Tick:
    """What a node reports: how it went, and how it wants to move."""

    status: Status
    velocity: Velocity = Velocity.STOP

    @classmethod
    def running(cls, velocity=Velocity.STOP):
        return cls(Status.RUNNING, velocity)

    @classmethod
    def success(cls, velocity=Velocity.STOP):
        return cls(Status.SUCCESS, velocity)

    @classmethod
    def failure(cls, velocity=Velocity.STOP):
        return cls(Status.FAILURE, velocity)


@dataclass
class Context:
    """Everything a node is allowed to look at."""

    robot: object
    world: object
    #: Scratch space shared across the tree, for one node to leave a note
    #: another can read -- a target name chosen at runtime, say.
    blackboard: dict = field(default_factory=dict)


class Node:
    """Base class. Subclasses implement `tick`; composites also hold children."""

    #: Shown in the UI.
    kind = "node"

    def __init__(self, name=None, children=()):
        self.name = name or type(self).__name__
        self.children = list(children)
        self.last_status = None
        self.running = False

    # -- lifecycle ---------------------------------------------------------

    def tick(self, ctx):
        """Run one step. Subclasses override `run`, not this."""
        result = self.run(ctx)
        self.last_status = result.status
        was_running, self.running = self.running, result.status is Status.RUNNING
        # Leaving RUNNING means this node finished; clear its state so a later
        # re-entry starts fresh instead of resuming halfway through.
        if was_running and not self.running:
            self.on_exit(ctx)
        return result

    def run(self, ctx):
        raise NotImplementedError

    def reset(self):
        """Forget any progress. Called when an ancestor restarts this subtree."""
        self.last_status = None
        self.running = False
        for child in self.children:
            child.reset()

    def on_exit(self, ctx):
        """Called when this node stops being RUNNING."""

    # -- introspection, so the UI can show what the tree is doing ----------

    def describe(self):
        return {
            "name": self.name,
            "kind": self.kind,
            "status": self.last_status.value if self.last_status else None,
            "children": [c.describe() for c in self.children],
        }

    def active_path(self):
        """Names of the nodes currently RUNNING, outermost first."""
        if not self.running:
            return []
        for child in self.children:
            if child.running:
                return [self.name] + child.active_path()
        return [self.name]

    def running_leaf(self):
        """The deepest RUNNING node -- the one actually driving right now."""
        if not self.running:
            return None
        for child in self.children:
            if child.running:
                return child.running_leaf() or child
        return self


class TreeBehavior:
    """Presents a tree to the robot as an ordinary Behavior.

    The robot cannot tell a tree from a hand-written behaviour, which is what
    lets missions sit in the same list as `manual` and `patrol` and be selected
    the same way.
    """

    wants_detection = False
    #: A tree is always a mission: it has a goal and it finishes.
    category = "mission"

    def __init__(self, name, label, root, description="", requires=(),
                 wants_detection=False, blackboard=None):
        self.name = name
        self.label = label
        self.description = description
        self.requires = tuple(requires)
        self.wants_detection = wants_detection
        self.root = root
        self.blackboard = dict(blackboard or {})
        self.result = None
        # A mission takes a target if anything inside it does, so picking one
        # on the video reaches the right leaf without the UI knowing the shape
        # of the tree.
        self.accepts_target = any(
            getattr(getattr(node, "behavior", None), "accepts_target", False)
            or getattr(node, "kind", "") == "condition" and "target" in node.name
            for node in self._walk(root)
        )

    def start(self, robot):
        self.root.reset()
        self.result = None

    def stop(self, robot):
        # Give whatever leaf was mid-run a chance to clean up.
        ctx = Context(robot=robot, world=None, blackboard=self.blackboard)
        leaf = self.root.running_leaf()
        if leaf is not None:
            leaf.on_exit(ctx)
        self.root.reset()

    def update(self, robot, world):
        ctx = Context(robot=robot, world=world, blackboard=self.blackboard)
        tick = self.root.tick(ctx)

        if tick.status is Status.RUNNING:
            self.result = None
            return tick.velocity

        # The tree finished. Hold still and remember the outcome rather than
        # looping forever -- restarting is the operator's call.
        self.result = tick.status.value
        return Velocity.STOP

    def configure(self, **kwargs):
        # Settings land on the blackboard, where any node can read them, and
        # are also forwarded to wrapped behaviours that understand them.
        for key, value in kwargs.items():
            if value is not None:
                self.blackboard[key] = value
        for node in self._walk(self.root):
            behavior = getattr(node, "behavior", None)
            if behavior is not None and hasattr(behavior, "configure"):
                behavior.configure(**kwargs)
        return self.status()

    @staticmethod
    def _walk(node):
        yield node
        for child in node.children:
            yield from TreeBehavior._walk(child)

    def overlay(self, frame):
        leaf = self.root.running_leaf()
        behavior = getattr(leaf, "behavior", None) if leaf else None
        return behavior.overlay(frame) if behavior is not None else frame

    def status(self):
        leaf = self.root.running_leaf()
        return {
            "path": " > ".join(self.root.active_path()) or "-",
            "result": self.result or "running",
            "target": self.blackboard.get("target"),
            "tree": self.root.describe(),
            **({"leaf": leaf.name} if leaf else {}),
        }
