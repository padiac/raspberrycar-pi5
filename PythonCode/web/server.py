"""HTTP front end.

This file moves bytes; it holds no logic of its own. Every endpoint is a thin
wrapper over something the Robot already knows how to do, which is what makes
the same surface usable by the browser, by curl, and later by an LLM issuing
tool calls.
"""

import logging
import os
import subprocess

from flask import (
    Flask, Response, jsonify, render_template, request, send_from_directory,
)

from hal import Velocity
from robot import Robot

HTTP_PORT = int(os.environ.get("CAR_PORT", "8000"))

# The page polls telemetry twice a second, so per-request access logs bury
# anything worth reading -- under systemd the journal becomes unusable for
# finding an actual fault. Warnings and errors still come through.
logging.getLogger("werkzeug").setLevel(logging.WARNING)

app = Flask(__name__)
# Jinja caches compiled templates, and with debug off it never re-reads them --
# so edits to index.html silently do nothing until the process restarts, which
# is a confusing way to lose half an hour. The check is cheap at this traffic.
app.config["TEMPLATES_AUTO_RELOAD"] = True
robot = Robot()


@app.before_request
def _note_activity():
    """Anything that changes the robot counts as somebody being here.

    Reads deliberately do not. The page polls telemetry twice a second, so
    counting those would mean a tab left open on a desk keeps the camera
    running all night, which is the case this exists to catch.
    """
    if request.method == "POST":
        robot.touch()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/stream.mjpg")
def stream():
    if robot.vision is None:
        return jsonify(error="no camera"), 503

    frames = robot.vision.frames

    def generate():
        # Counted around the whole generator, so a browser that goes away
        # without saying goodbye is still let go of when its socket dies.
        robot.stream_clients += 1
        try:
            yield from _frames(frames)
        finally:
            robot.stream_clients = max(0, robot.stream_clients - 1)

    def _frames(frames):
        while True:
            got, jpeg = frames.wait_for_next(timeout=2.0)
            # While the camera is off nothing is published, so re-send the last
            # frame (the 'camera off' placeholder) on each timeout -- otherwise
            # a client that connects while it is off would sit on a blank image.
            if not got and robot.camera and robot.camera.enabled:
                continue
            if jpeg is None:
                continue
            yield (
                b"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: "
                + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n"
            )

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=FRAME")


# -- driving --------------------------------------------------------------


@app.route("/api/drive", methods=["POST"])
def api_drive():
    """Manual driving. Accepts the full velocity: vx, vy (strafe), omega."""
    data = request.get_json(silent=True) or {}
    try:
        velocity = Velocity(
            vx=float(data.get("vx", 0.0)),
            vy=float(data.get("vy", 0.0)),
            omega=float(data.get("omega", 0.0)),
        )
    except (TypeError, ValueError):
        return jsonify(error="vx, vy and omega must be numbers"), 400

    ok, message = robot.manual(velocity)
    if not ok:
        return jsonify(error=message), 409
    return jsonify(requested=velocity.as_dict(), behavior=robot.behavior.name)


@app.route("/api/pan", methods=["POST"])
def api_pan():
    """Aim the head the ultrasonic is on, or trim where its forward is."""
    if robot.pan is None:
        return jsonify(error="no pan servo"), 503
    data = request.get_json(silent=True) or {}
    try:
        if "trim" in data:
            robot.set_pan_trim(data["trim"])
        if "angle" in data:
            robot.pan.look(data["angle"])
        if "by" in data:
            # Relative, for anything holding a direction rather than naming an
            # angle -- the controller's d-pad, and arrow keys later.
            robot.pan_by(data["by"])
    except (TypeError, ValueError):
        return jsonify(error="angle and trim must be numbers"), 400
    return jsonify(robot.pan.status())


@app.route("/api/idle", methods=["POST"])
def api_idle():
    """How long the robot waits before switching its own camera off."""
    data = request.get_json(silent=True) or {}
    if "after_s" in data:
        try:
            # Zero disables it. A very short one would fight the person
            # setting it, so the floor is a sensible minute.
            value = float(data["after_s"])
            robot.idle_camera_after = 0.0 if value <= 0 else max(60.0, value)
        except (TypeError, ValueError):
            return jsonify(error="after_s must be a number"), 400
    return jsonify(after_s=robot.idle_camera_after)


@app.route("/api/speed", methods=["POST"])
def api_speed():
    """The one speed control: scales every behaviour, not just the d-pad."""
    data = request.get_json(silent=True) or {}
    if "limit" in data:
        try:
            robot.speed_limit = max(0.05, min(1.0, float(data["limit"])))
        except (TypeError, ValueError):
            return jsonify(error="limit must be a number"), 400
    return jsonify(limit=robot.speed_limit)


@app.route("/api/stop", methods=["POST"])
def api_stop():
    robot.emergency_stop()
    return jsonify(stopped=True, behavior=robot.behavior.name)


# -- behaviours ------------------------------------------------------------


@app.route("/api/controls")
def api_controls():
    """What the controller's buttons do.

    Served from the same table the reader dispatches on, so the help on
    the page cannot describe a binding that is not there, or miss one
    that is.
    """
    from hal.gamepad import describe
    return jsonify(**describe())


@app.route("/api/behaviors")
def api_behaviors():
    return jsonify(behaviors=robot.behavior_list(), active=robot.behavior.name)


@app.route("/api/behavior", methods=["POST"])
def api_behavior():
    data = request.get_json(silent=True) or {}
    name = data.get("name")
    if not name:
        return jsonify(error="name required"), 400

    ok, message = robot.select(name)
    if not ok:
        return jsonify(error=message), 409

    config = data.get("config")
    if isinstance(config, dict):
        robot.behavior.configure(**config)
    return jsonify(
        active=robot.behavior.name, message=message, status=robot.behavior.status()
    )


@app.route("/api/behavior/next", methods=["POST"])
def api_behavior_next():
    """Step to the next usable behaviour, in the order the page lists them."""
    ok, message = robot.cycle_behavior()
    if not ok:
        return jsonify(error=message), 409
    return jsonify(active=robot.behavior.name, message=message)


@app.route("/api/target", methods=["POST"])
def api_target():
    """Aim at an object, typically because someone tapped it in the video."""
    data = request.get_json(silent=True) or {}
    label = data.get("label")
    if not label:
        return jsonify(error="label required"), 400

    ok, message = robot.set_target(label)
    if not ok:
        return jsonify(error=message), 409
    return jsonify(
        behavior=robot.behavior.name, message=message, status=robot.behavior.status()
    )


@app.route("/api/behavior/config", methods=["POST"])
def api_behavior_config():
    data = request.get_json(silent=True) or {}
    name = data.pop("name", None)
    behavior = robot.behaviors.get(name) if name else robot.behavior
    if behavior is None:
        return jsonify(error=f"unknown behaviour '{name}'"), 400
    return jsonify(status=behavior.configure(**data))


# -- safety ----------------------------------------------------------------


@app.route("/api/path", methods=["POST"])
def api_path():
    """Record / stop / clear the driven path, or play it back."""
    data = request.get_json(silent=True) or {}
    action = data.get("action")

    if action == "record":
        robot.recorder.arm()
    elif action == "stop":
        robot.recorder.disarm()
    elif action == "clear":
        robot.recorder.disarm()
        robot.recorder.clear()
    elif action == "run":
        # Through the robot, not around it. This used to select the behaviour
        # itself, which meant the button on the controller and the button on
        # the page were two implementations of one operation -- and only one of
        # them knew that replaying again after it has finished is allowed while
        # replaying again halfway through is not.
        if "loop" in data:
            robot.behaviors["replay"].configure(loop=data["loop"])
        ok, message = robot.play_path()
        if not ok:
            return jsonify(error=message), 409
    elif action is not None:
        return jsonify(error=f"unknown action '{action}'"), 400

    return jsonify(path=robot.recorder.status(), behavior=robot.behavior.name)


@app.route("/api/drivetrain", methods=["POST"])
def api_drivetrain():
    data = request.get_json(silent=True) or {}
    kind = data.get("kind")
    if not kind:
        return jsonify(error="kind required"), 400

    ok, message = robot.set_drivetrain(kind)
    status = robot.drivetrain.status() if robot.drivetrain else None
    if not ok:
        return jsonify(error=message, drivetrain=status), 409
    return jsonify(message=message, drivetrain=status)


@app.route("/api/safety", methods=["POST"])
def api_safety():
    data = request.get_json(silent=True) or {}
    if "enabled" in data:
        robot.safety.enabled = bool(data["enabled"])
    for key in ("stop_cm", "slow_cm"):
        if key in data:
            try:
                setattr(robot.safety, key, float(data[key]))
            except (TypeError, ValueError):
                return jsonify(error=f"{key} must be a number"), 400
    return jsonify(robot.safety.status())


# -- camera and detection --------------------------------------------------


@app.route("/api/camera", methods=["POST"])
def api_camera():
    if robot.vision is None:
        return jsonify(error="no camera"), 503
    data = request.get_json(silent=True) or {}
    if "enabled" in data:
        robot.vision.set_camera(data["enabled"])
    return jsonify(enabled=bool(robot.camera and robot.camera.enabled))


@app.route("/api/detect", methods=["POST"])
def api_detect():
    if robot.detector is None or robot.vision is None:
        return jsonify(error="detector unavailable"), 503

    data = request.get_json(silent=True) or {}
    if "enabled" in data:
        robot.vision.set_detection(data["enabled"])
    if "conf" in data or "classes" in data:
        robot.detector.configure(
            confidence=data.get("conf"), classes=data.get("classes")
        )
    if "target_fps" in data:
        try:
            robot.vision.detect_target_fps = max(0.0, min(60.0, float(data["target_fps"])))
        except (TypeError, ValueError):
            return jsonify(error="target_fps must be a number"), 400

    return jsonify(
        enabled=robot.vision.detect_enabled,
        target_fps=robot.vision.detect_target_fps,
        **robot.detector.status(),
    )


@app.route("/api/classes")
def api_classes():
    return jsonify(classes=robot.detector.class_names if robot.detector else [])


@app.route("/api/shutdown", methods=["POST"])
def api_shutdown():
    """Power the Pi down cleanly.

    Yanking the plug on a running Linux box risks the SD card, and the card is
    the whole robot. There is no console on a car, so the only place this
    button can live is the page that is already open.

    Deliberately awkward to trigger: it wants an explicit token in the body, so
    a stray POST -- a retried request, something crawling the API, a fat finger
    on a browser's replay button -- cannot switch the car off. The token is not
    a password. Anyone who can reach this port can already drive the car; the
    point is only that it takes an intent to say it.
    """
    data = request.get_json(silent=True) or {}
    if data.get("confirm") != "shutdown":
        return jsonify(error='send {"confirm": "shutdown"} to power off'), 400

    # Stop the wheels before the process dies. systemd would get there too, via
    # KillSignal=SIGINT, but not until it starts tearing the unit down, and a
    # car that keeps rolling through its own shutdown is a bad look.
    robot.emergency_stop()
    try:
        subprocess.Popen(["sudo", "-n", "/sbin/shutdown", "-h", "now"])
    except OSError as exc:
        return jsonify(error=f"could not shut down: {exc}"), 500
    return jsonify(shutting_down=True)


@app.route("/api/clip", methods=["POST"])
def api_clip():
    """Start or stop recording the stream to a file."""
    if robot.clips is None:
        return jsonify(error="no camera"), 503
    data = request.get_json(silent=True) or {}
    action = data.get("action")
    if action == "start":
        ok, message = robot.clips.start()
    elif action == "stop":
        ok, message = robot.clips.stop()
    elif action is None:
        ok, message = True, "ok"
    else:
        return jsonify(error=f"unknown action '{action}'"), 400
    status = robot.clips.status()
    if not ok:
        return jsonify(error=message, clip=status), 409
    return jsonify(message=message, clip=status, clips=robot.clips.clips())


@app.route("/api/clips")
def api_clips():
    if robot.clips is None:
        return jsonify(clips=[])
    return jsonify(clips=robot.clips.clips(), clip=robot.clips.status())


@app.route("/api/clips/<name>")
def api_clip_download(name):
    """Hand over a finished clip.

    resolve() vets the name rather than this route cleaning it up: the only
    names that mean anything are ones the recorder made, so anything else is
    simply not a clip, and there is no path here to traverse out of.
    """
    if robot.clips is None or robot.clips.resolve(name) is None:
        return jsonify(error="no such clip"), 404
    return send_from_directory(
        robot.clips.directory, name, as_attachment=True, mimetype="video/mp4"
    )


@app.route("/api/clips/<name>/delete", methods=["POST"])
def api_clip_delete(name):
    if robot.clips is None:
        return jsonify(error="no camera"), 503
    ok, message = robot.clips.delete(name)
    if not ok:
        return jsonify(error=message), 409
    return jsonify(message=message, clips=robot.clips.clips())


@app.route("/api/telemetry")
def api_telemetry():
    return jsonify(robot.telemetry())


def main():
    try:
        app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True)
    finally:
        robot.close()


if __name__ == "__main__":
    main()
