"""Checks for behaviour tree semantics. No hardware, no robot, no camera."""

import time

from behaviors.tree import (
    Fallback,
    Inverter,
    Repeat,
    Retry,
    Sequence,
    Status,
    Succeeder,
    Timeout,
    Wait,
)
from behaviors.tree.core import Context, Node, Tick
from hal import Velocity

FAILURES = []


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if not ok:
        print(f"        got {got!r}  want {want!r}")
        FAILURES.append(name)


class Canned(Node):
    """A leaf that returns whatever you tell it, and counts its ticks."""

    def __init__(self, script, name="canned"):
        super().__init__(name)
        self.script = list(script)
        self.ticks = 0
        self.resets = 0

    def run(self, ctx):
        self.ticks += 1
        status = self.script[min(self.ticks - 1, len(self.script) - 1)]
        return Tick(status, Velocity(vx=0.5) if status is Status.RUNNING else Velocity.STOP)

    def reset(self):
        super().reset()
        self.resets += 1
        self.ticks = 0


S, F, R = Status.SUCCESS, Status.FAILURE, Status.RUNNING
ctx = Context(robot=None, world=None)


print("Sequence -- 'and then'")
a, b = Canned([S], "a"), Canned([S], "b")
seq = Sequence([a, b])
check("both succeed -> SUCCESS", seq.tick(ctx).status, S)
check("  each child ticked once", (a.ticks, b.ticks), (1, 1))

a, b = Canned([F], "a"), Canned([S], "b")
seq = Sequence([a, b])
check("first fails -> FAILURE", seq.tick(ctx).status, F)
check("  second never ran", b.ticks, 0)

a, b = Canned([R, R, S], "a"), Canned([S], "b")
seq = Sequence([a, b])
check("first still running -> RUNNING", seq.tick(ctx).status, R)
check("  second still not started", b.ticks, 0)
seq.tick(ctx)
check("after first finishes, sequence completes", seq.tick(ctx).status, S)
check("  and the second finally ran", b.ticks, 1)

print("\nSequence passes the running child's velocity through")
a = Canned([R], "a")
check("velocity reaches the top", Sequence([a]).tick(ctx).velocity.vx, 0.5)

print("\nFallback -- 'or else'")
a, b = Canned([F], "a"), Canned([S], "b")
check("first fails, second succeeds -> SUCCESS", Fallback([a, b]).tick(ctx).status, S)

a, b = Canned([S], "a"), Canned([S], "b")
fb = Fallback([a, b])
check("first succeeds -> SUCCESS", fb.tick(ctx).status, S)
check("  second never tried", b.ticks, 0)

a, b = Canned([F, F], "a"), Canned([F, F], "b")
check("all fail -> FAILURE", Fallback([a, b]).tick(ctx).status, F)

print("\nFallback re-decides every tick (this is what makes a tree reactive)")
# 'a' fails at first, so 'b' runs. Then 'a' becomes available and must take
# over, because higher-priority branches are always retried from the top.
a = Canned([F, R, R], "preferred")
b = Canned([R, R, R], "default")
fb = Fallback([a, b])
first = fb.tick(ctx)
check("tick 1 runs the fallback branch", (first.status, b.ticks), (R, 1))
fb.tick(ctx)
check("tick 2 switches back to the preferred branch", a.ticks, 2)
check("  and the abandoned branch was reset", b.resets >= 1, True)

print("\nInverter / Succeeder")
check("Inverter turns SUCCESS into FAILURE", Inverter(Canned([S])).tick(ctx).status, F)
check("Inverter turns FAILURE into SUCCESS", Inverter(Canned([F])).tick(ctx).status, S)
check("Inverter leaves RUNNING alone", Inverter(Canned([R])).tick(ctx).status, R)
check("Succeeder swallows failure", Succeeder(Canned([F])).tick(ctx).status, S)

print("\nRepeat")
child = Canned([S], "once")
rep = Repeat(child, times=3)
statuses = [rep.tick(ctx).status for _ in range(3)]
check("keeps going until the count is reached", statuses, [R, R, S])
check("  child restarted between runs", child.resets >= 2, True)

forever = Repeat(Canned([S]), times=None)
check("times=None never finishes", [forever.tick(ctx).status for _ in range(5)], [R] * 5)

print("\nRetry")
child = Canned([F, F, S], "flaky")
retry = Retry(child, attempts=3)
# Each failure resets the child, so the canned script restarts too -- the
# child fails forever here, which is what exhausting the attempts should look
# like.
statuses = [retry.tick(ctx).status for _ in range(3)]
check("gives up after the attempt budget", statuses, [R, R, F])

print("\nTimeout")
slow = Canned([R] * 100, "slow")
timeout = Timeout(slow, 0.15)
check("still within the deadline", timeout.tick(ctx).status, R)
time.sleep(0.2)
check("past the deadline -> FAILURE", timeout.tick(ctx).status, F)
check("  child was reset", slow.resets >= 1, True)

quick = Canned([S], "quick")
check("finishing in time is untouched", Timeout(quick, 5).tick(ctx).status, S)

print("\nWait")
wait = Wait(0.15)
check("running while waiting", wait.tick(ctx).status, R)
time.sleep(0.2)
check("succeeds once elapsed", wait.tick(ctx).status, S)

print("\nintrospection (this is what the web UI renders)")
inner = Canned([R], "leaf")
tree = Sequence([Canned([S], "done"), Sequence([inner], "inner")], "root")
tree.tick(ctx)
check("active path names the running branch", tree.active_path(), ["root", "inner", "leaf"])
check("running_leaf finds the deepest node", tree.running_leaf().name, "leaf")
described = tree.describe()
check("describe() is nested", described["name"], "root")
check("  with children", len(described["children"]), 2)

print("\nreset clears the whole subtree")
seq = Sequence([Canned([R]), Canned([S])])
seq.tick(ctx)
seq.reset()
check("index rewound", seq.index, 0)
check("status forgotten", seq.last_status, None)

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S): {FAILURES}")
    raise SystemExit(1)
print("all tree checks passed")
