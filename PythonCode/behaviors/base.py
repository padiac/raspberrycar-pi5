"""What a behaviour is.

A behaviour is a policy, not a program. It reads a `WorldState` and returns the
`Velocity` it would like; it does not own the camera, the motors or the loop, so
behaviours can be swapped mid-drive, and a new one can be written without
touching anything else.

It also does not have to worry about obstacles. The safety layer sits between
every behaviour and the wheels, so a behaviour can ask for full speed ahead and
trust that it will be refused if there is a wall there. That separation is why
this layer stays readable.
"""

from hal import Velocity


class Behavior:
    #: Stable identifier, used by the API.
    name = "base"
    #: What the UI shows.
    label = "Base"
    description = ""
    #: How the UI files this. Not cosmetic -- these are genuinely different
    #: kinds of thing, and listing them together is what made the old UI read
    #: as a pile of demos:
    #:   control  the baseline: stopped, driven by hand. Always available.
    #:   skill    one capability, runs until you stop it. A building block --
    #:            missions are assembled out of these.
    #:   mission  a composed task with a goal, which succeeds or fails.
    category = "skill"
    #: True if this behaviour follows a named object, so the UI knows whether
    #: picking a target on the video means anything here.
    accepts_target = False
    #: Hardware this behaviour cannot work without. The robot refuses to select
    #: a behaviour whose requirements are unmet, rather than failing later.
    #: Valid keys: motors, camera, detector, rangefinder, proximity, floor
    requires = ()
    #: True to have the robot switch object detection on while this runs.
    wants_detection = False

    def start(self, robot):
        """Called once when this behaviour becomes active."""

    def stop(self, robot):
        """Called once when this behaviour is replaced."""

    def update(self, robot, world):
        """Read the world, return the Velocity you want."""
        return Velocity.STOP

    def status(self):
        """Extra fields for the telemetry endpoint."""
        return {}

    def configure(self, **kwargs):
        """Settings from the web UI. Return a status dict."""
        return self.status()

    def overlay(self, frame):
        """Optionally draw debug graphics on the streamed frame."""
        return frame
