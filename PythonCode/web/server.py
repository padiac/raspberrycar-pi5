"""HTTP front end.

This file moves bytes; it holds no logic of its own. Every endpoint is a thin
wrapper over something the Robot already knows how to do, which is what makes
the same surface usable by the browser, by curl, and later by an LLM issuing
tool calls.
"""

import os

from flask import Flask, Response, jsonify, render_template, request

from hal import Velocity
from robot import Robot

HTTP_PORT = int(os.environ.get("CAR_PORT", "8000"))

app = Flask(__name__)
# Jinja caches compiled templates, and with debug off it never re-reads them --
# so edits to index.html silently do nothing until the process restarts, which
# is a confusing way to lose half an hour. The check is cheap at this traffic.
app.config["TEMPLATES_AUTO_RELOAD"] = True
robot = Robot()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/stream.mjpg")
def stream():
    if robot.vision is None:
        return jsonify(error="no camera"), 503

    frames = robot.vision.frames

    def generate():
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


@app.route("/api/stop", methods=["POST"])
def api_stop():
    robot.emergency_stop()
    return jsonify(stopped=True, behavior=robot.behavior.name)


# -- behaviours ------------------------------------------------------------


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
