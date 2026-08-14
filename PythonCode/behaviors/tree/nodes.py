"""The node types a tree is built from.

Three families:

* **Composites** run children and combine their answers -- this is where the
  control flow lives. `Sequence` is "and then", `Fallback` is "or else".
* **Decorators** wrap a single child and bend its answer: retry it, give it a
  deadline, invert it.
* **Leaves** do the actual work or check a fact.

The important leaf is `RunBehavior`, which turns any existing `Behavior` into a
tree node. That is the join between the two layers: behaviours stay simple
policies that know nothing about tasks, and the tree arranges them.
"""

import time

from hal import Velocity

from .core import Node, Status, Tick


# --------------------------------------------------------------------------
# Composites
# --------------------------------------------------------------------------


class Sequence(Node):
    """"And then." Runs children in order; any failure fails the whole thing.

    Succeeds only when every child has succeeded. This is how a task with
    stages is written: scan, *then* approach, *then* stop.
    """

    kind = "sequence"

    def __init__(self, children, name="Sequence"):
        super().__init__(name, children)
        self.index = 0

    def reset(self):
        super().reset()
        self.index = 0

    def run(self, ctx):
        while self.index < len(self.children):
            result = self.children[self.index].tick(ctx)

            if result.status is Status.RUNNING:
                return result
            if result.status is Status.FAILURE:
                self.index = 0
                return Tick.failure(result.velocity)

            self.index += 1

        self.index = 0
        return Tick.success()


class Fallback(Node):
    """"Or else." Tries children in order until one does not fail.

    This is what makes a tree *reactive* rather than a rigid script: put the
    preferred option first and a default last, and the robot picks whichever
    currently applies, re-deciding every tick.
    """

    kind = "fallback"

    def __init__(self, children, name="Fallback"):
        super().__init__(name, children)
        self.index = 0

    def reset(self):
        super().reset()
        self.index = 0

    def run(self, ctx):
        # Always start from the top so a higher-priority child that has become
        # available again takes over from a lower-priority one.
        for index, child in enumerate(self.children):
            result = child.tick(ctx)
            if result.status is Status.FAILURE:
                continue
            if index != self.index:
                # Switched branch: abandon whatever the old one was doing.
                for other in self.children[index + 1:]:
                    other.reset()
                self.index = index
            return result

        self.index = 0
        return Tick.failure()


class Parallel(Node):
    """Ticks every child each time; succeeds once `threshold` have succeeded.

    Velocity comes from the first child that wants to move, so put the child
    that drives first and use the others as monitors.
    """

    kind = "parallel"

    def __init__(self, children, threshold=None, name="Parallel"):
        super().__init__(name, children)
        self.threshold = threshold if threshold is not None else len(children)

    def run(self, ctx):
        successes = failures = 0
        velocity = Velocity.STOP

        for child in self.children:
            result = child.tick(ctx)
            if result.status is Status.SUCCESS:
                successes += 1
            elif result.status is Status.FAILURE:
                failures += 1
            if not velocity.moving and result.velocity.moving:
                velocity = result.velocity

        if successes >= self.threshold:
            return Tick.success(velocity)
        if failures > len(self.children) - self.threshold:
            return Tick.failure(velocity)
        return Tick.running(velocity)


# --------------------------------------------------------------------------
# Decorators
# --------------------------------------------------------------------------


class Decorator(Node):
    kind = "decorator"

    def __init__(self, child, name=None):
        super().__init__(name or type(self).__name__, [child])

    @property
    def child(self):
        return self.children[0]


class Inverter(Decorator):
    """Swaps SUCCESS and FAILURE. Lets one condition serve both polarities."""

    def run(self, ctx):
        result = self.child.tick(ctx)
        if result.status is Status.SUCCESS:
            return Tick.failure(result.velocity)
        if result.status is Status.FAILURE:
            return Tick.success(result.velocity)
        return result


class Succeeder(Decorator):
    """Turns failure into success. Use for steps that are allowed to not work."""

    def run(self, ctx):
        result = self.child.tick(ctx)
        if result.status is Status.FAILURE:
            return Tick.success(result.velocity)
        return result


class Repeat(Decorator):
    """Restarts the child when it finishes. `times=None` means forever."""

    def __init__(self, child, times=None, name=None):
        super().__init__(child, name or f"Repeat({times or 'forever'})")
        self.times = times
        self.count = 0

    def reset(self):
        super().reset()
        self.count = 0

    def run(self, ctx):
        result = self.child.tick(ctx)
        if result.status is Status.RUNNING:
            return result

        self.count += 1
        self.child.reset()
        if self.times is not None and self.count >= self.times:
            return Tick.success(result.velocity)
        return Tick.running(result.velocity)


class Retry(Decorator):
    """Re-runs the child after a failure, up to `attempts` times."""

    def __init__(self, child, attempts=3, name=None):
        super().__init__(child, name or f"Retry({attempts})")
        self.attempts = attempts
        self.used = 0

    def reset(self):
        super().reset()
        self.used = 0

    def run(self, ctx):
        result = self.child.tick(ctx)
        if result.status is not Status.FAILURE:
            if result.status is Status.SUCCESS:
                self.used = 0
            return result

        self.used += 1
        self.child.reset()
        if self.used >= self.attempts:
            self.used = 0
            return Tick.failure(result.velocity)
        return Tick.running(result.velocity)


class Timeout(Decorator):
    """Fails the child if it has not finished within `seconds`.

    Without this a search that never finds anything runs until someone notices.
    """

    def __init__(self, child, seconds, name=None):
        super().__init__(child, name or f"Timeout({seconds}s)")
        self.seconds = seconds
        self.started = None

    def reset(self):
        super().reset()
        self.started = None

    def run(self, ctx):
        now = time.monotonic()
        if self.started is None:
            self.started = now

        if now - self.started > self.seconds:
            self.child.reset()
            self.started = None
            return Tick.failure()

        result = self.child.tick(ctx)
        if result.status is not Status.RUNNING:
            self.started = None
        return result

    def status_note(self):
        if self.started is None:
            return ""
        return f"{self.seconds - (time.monotonic() - self.started):.0f}s left"


# --------------------------------------------------------------------------
# Leaves
# --------------------------------------------------------------------------


class Condition(Node):
    """Checks a fact about the world. Never moves the robot."""

    kind = "condition"

    def __init__(self, check, name="Condition"):
        super().__init__(name)
        self.check = check

    def run(self, ctx):
        return Tick.success() if self.check(ctx) else Tick.failure()


class Wait(Node):
    """Holds still for a while, then succeeds."""

    kind = "action"

    def __init__(self, seconds, name=None):
        super().__init__(name or f"Wait({seconds}s)")
        self.seconds = seconds
        self.started = None

    def reset(self):
        super().reset()
        self.started = None

    def run(self, ctx):
        now = time.monotonic()
        if self.started is None:
            self.started = now
        if now - self.started >= self.seconds:
            self.started = None
            return Tick.success()
        return Tick.running()


class Move(Node):
    """Drives at a fixed velocity for a fixed time. Useful as a building block."""

    kind = "action"

    def __init__(self, velocity, seconds=None, name=None):
        super().__init__(name or "Move")
        self.velocity = velocity
        self.seconds = seconds
        self.started = None

    def reset(self):
        super().reset()
        self.started = None

    def run(self, ctx):
        now = time.monotonic()
        if self.started is None:
            self.started = now
        if self.seconds is not None and now - self.started >= self.seconds:
            self.started = None
            return Tick.success()
        return Tick.running(self.velocity)


class RunBehavior(Node):
    """Runs an existing Behavior as a tree leaf.

    `until` is checked each tick and decides when the leaf is done: return a
    Status to finish, or None to keep going. Left as None the behaviour runs
    until something above it -- a Timeout, a Fallback re-deciding -- takes over.

    This adapter is why the two layers stay separate: `FollowBehavior` has no
    idea it is part of a task, and the task has no idea how following works.
    """

    kind = "action"

    def __init__(self, behavior, until=None, name=None):
        super().__init__(name or behavior.label)
        self.behavior = behavior
        self.until = until
        self.started = False

    def reset(self):
        super().reset()
        self.started = False

    def on_exit(self, ctx):
        if self.started:
            self.behavior.stop(ctx.robot)
            self.started = False

    def run(self, ctx):
        if not self.started:
            self.behavior.start(ctx.robot)
            self.started = True

        velocity = self.behavior.update(ctx.robot, ctx.world)

        if self.until is not None:
            verdict = self.until(ctx, self.behavior)
            if verdict is not None:
                return Tick(verdict, Velocity.STOP)

        return Tick.running(velocity)

    def describe(self):
        info = super().describe()
        info["detail"] = self.behavior.status().get("state", "")
        return info


# -- small helpers for writing readable conditions -------------------------


def sees(label_or_key="target"):
    """Condition: the detector can currently see the target.

    Pass a class name, or leave it to read `target` from the blackboard so the
    same tree works for whatever the operator asked for.
    """

    def check(ctx):
        label = ctx.blackboard.get(label_or_key, label_or_key)
        return ctx.world is not None and ctx.world.largest(label) is not None

    return Condition(check, f"sees {label_or_key}")


def closer_than(centimetres):
    def check(ctx):
        distance = ctx.world.range_cm if ctx.world else None
        return distance is not None and 0 < distance < centimetres

    return Condition(check, f"closer than {centimetres}cm")


def fills_frame(fraction=0.4, key="target"):
    """Condition: the target takes up at least `fraction` of the frame height.

    A single camera cannot measure distance, so apparent size is the stand-in
    for "close enough".
    """

    def check(ctx):
        if ctx.world is None or not ctx.world.frame_size[1]:
            return False
        label = ctx.blackboard.get(key, key)
        found = ctx.world.largest(label)
        return found is not None and found.height / ctx.world.frame_size[1] >= fraction

    return Condition(check, f"target fills {int(fraction * 100)}% of frame")
