"""Everything the web page is told, assembled in one place.

Lifted out of the robot because it is not control, it is reporting, and
because it was the single largest thing in there: ninety lines of dict
literal in the middle of a class that also runs the wheels. Nothing here
decides anything. If a line in this file changes what the car does, it is
in the wrong file.

It takes the robot rather than being a method on it for the same reason:
the dependency should point this way round. Telemetry knows about the
robot; the robot does not need to know how it is described.
"""

import time

from hal.drivetrain import DRIVETRAINS


def describe(robot):
    world = robot.world
    return {
        "behavior": robot.behavior.name,
        "behavior_label": robot.behavior.label,
        "behavior_status": robot.behavior.status(),
        "behavior_error": robot.behavior_error,
        "behavior_category": getattr(robot.behavior, "category", "skill"),
        "accepts_target": getattr(robot.behavior, "accepts_target", False),
        "moving": robot.allowed.moving,
        "requested": robot.requested.as_dict(),
        "allowed": robot.allowed.as_dict(),
        "safety": robot.safety.status(),
        "speed_limit": robot.speed_limit,
        "supply": robot.supply.status() if robot.supply else None,
        "idle": {
            "after_s": robot.idle_camera_after,
            "quiet_s": round(time.monotonic() - robot.last_activity, 1),
            "camera_slept": robot.camera_slept,
            "watchers": robot.stream_clients,
        },
        "peers": robot.peer_list(),
        "path": robot.recorder.status(),
        "drivetrain": (
            {**robot.drivetrain.status(), "kinds": sorted(DRIVETRAINS),
             "kind": getattr(robot.drivetrain, "kind", None)}
            if robot.drivetrain else None
        ),
        "range_cm": None if world.range_cm is None else round(world.range_cm, 1),
        # What the safety layer actually judges on, which is not always
        # what the head happens to be pointing at this instant.
        "forward_cm": (None if world.forward_cm is None
                       else round(world.forward_cm, 1)),
        "forward_age": (None if world.forward_age is None
                        else round(world.forward_age, 2)),
        "pan": robot.pan.status() if robot.pan else None,
        "clip": robot.clips.status() if robot.clips else None,
        "gamepad": robot.gamepad.status() if robot.gamepad else None,
        "proximity": world.proximity.as_dict() if world.proximity else None,
        # What the pins say regardless of whether they are trusted yet,
        # so wiring can be checked by waving a hand at them.
        "imu": (
            {"heading": robot.link.telemetry.heading,
             "bumped": robot.link.telemetry.bumped,
             **robot.turning.status()}
            if robot.link else None
        ),
        "ir": (
            {"fitted": robot.proximity.fitted,
             "reporting": robot.proximity.reporting,
             "pins": robot.link.telemetry.proximity}
            # Asked of the contract, not of one implementation: any
            # proximity sensor says whether it is fitted and whether it
            # is reporting, and the raw pins only exist when a board is
            # sending them.
            if robot.proximity is not None and robot.link is not None
            else None
        ),
        "floor": world.floor.as_dict() if world.floor else None,
        # The room as the car has measured it, one spoke at a time. Not a
        # map -- there is no position to hang one on -- but the nearest
        # thing a car with a single spinning sensor has to a picture.
        "scan": robot.scan.status(time.monotonic()),
        # The serial link itself, which was never reported. Its counters
        # are the only place a fault that comes and goes leaves a mark:
        # how many times the board has restarted by itself, and how many
        # bytes arrived that were not lines. Both were invisible, and an
        # invisible intermittent fault is one nobody can be told about.
        "link": robot.link.status() if robot.link else None,
        # The logbook is deliberately NOT here. This runs twice a second
        # and its status walks the log directory, so putting it in the
        # main poll would buy a stat() of every rotated file for every
        # tick of a page nobody may even be looking at. It has its own
        # endpoint, polled far more slowly by the panel that wants it.
        "loop_hz": None if robot.loop_hz is None else round(robot.loop_hz, 1),
        "stream_fps": round(robot.vision.stream_fps, 1) if robot.vision else 0.0,
        "detect_fps": round(robot.vision.detect_fps, 1) if robot.vision else 0.0,
        "detect_enabled": robot.vision.detect_enabled if robot.vision else False,
        "camera_enabled": bool(robot.camera and robot.camera.enabled),
        # Boxes go out too so the page can work out which object was
        # tapped without asking the robot to hit-test for it.
        "objects": [
            {**d.as_dict(), "box": list(d.box)} for d in world.detections
        ],
        "frame_size": list(world.frame_size),
        "detector": robot.detector.status() if robot.detector else None,
        "available": robot.available,
    }
