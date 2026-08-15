"""Play back a path that was driven by hand."""

from hal import Velocity

from .base import Behavior


class ReplayBehavior(Behavior):
    name = "replay"
    label = "Replay path"
    description = "Drives back the route you recorded, at the same speeds."
    category = "mission"
    requires = ("motors",)
    # The recorded segments were already governed when they were driven, so
    # scaling them again would replay the route at a fraction of the speed
    # it was recorded at, and slower still on each round trip.
    governed = False

    def __init__(self, recorder):
        self.recorder = recorder
        self.segments = []
        self.index = 0
        self.elapsed = 0.0
        self.loop = False
        self.state = "idle"

    def configure(self, **kwargs):
        if "loop" in kwargs:
            self.loop = bool(kwargs["loop"])
        return self.status()

    def start(self, robot):
        # Take a copy: the recorder can keep collecting while this runs.
        self.segments = self.recorder.snapshot()
        self.index, self.elapsed = 0, 0.0
        self.state = "nothing recorded" if not self.segments else "replaying"

    def stop(self, robot):
        self.state = "idle"

    def update(self, robot, world):
        if self.index >= len(self.segments):
            if self.loop and self.segments:
                self.index, self.elapsed = 0, 0.0
            else:
                self.state = "done" if self.segments else "nothing recorded"
                return Velocity.STOP

        segment = self.segments[self.index]
        self.elapsed += world.dt
        if self.elapsed >= segment.seconds:
            self.index += 1
            self.elapsed = 0.0

        self.state = f"{min(self.index + 1, len(self.segments))}/{len(self.segments)}"
        return segment.velocity

    def status(self):
        return {
            "state": self.state,
            "step": f"{self.index}/{len(self.segments)}",
            "loop": self.loop,
        }
