#!/usr/bin/env python3
"""HTTP front end for the car.

All the behaviour lives in robot.py and modes/; this file only moves bytes
between the browser and the Robot. Run it on the Pi:

    cd PythonCode
    python3 web_server.py

then open http://<pi-ip>:8000 from a PC or phone on the same network.
"""

import os

from flask import Flask, Response, jsonify, render_template, request

from robot import Robot

HTTP_PORT = int(os.environ.get("CAR_PORT", "8000"))

app = Flask(__name__)
robot = Robot()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/stream.mjpg")
def stream():
    output = robot.vision.output

    def frames():
        while True:
            with output.condition:
                # Time-limited: if the sensor stops delivering (a half-seated
                # ribbon cable will do it) the generator keeps ticking over so
                # the worker thread can notice the client left, instead of
                # parking on the condition forever.
                got = output.condition.wait(timeout=2.0)
                # While the camera is off nothing is published, so re-send the
                # last frame (the 'camera off' placeholder) on each timeout --
                # otherwise a client that connects while it's off, or reloads
                # the page, would sit on a blank image forever.
                frame = output.frame
                if not got and robot.vision.camera_enabled:
                    frame = None
            if frame is None:
                continue
            yield (
                b"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: "
                + str(len(frame)).encode()
                + b"\r\n\r\n"
                + frame
                + b"\r\n"
            )

    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=FRAME")


# -- driving -------------------------------------------------------------


@app.route("/api/drive", methods=["POST"])
def api_drive():
    data = request.get_json(silent=True) or {}
    try:
        throttle = float(data.get("throttle", 0.0))
        steer = float(data.get("steer", 0.0))
    except (TypeError, ValueError):
        return jsonify(error="throttle and steer must be numbers"), 400

    ok, message = robot.manual_command(throttle, steer)
    if not ok:
        return jsonify(error=message), 409
    return jsonify(throttle=robot.throttle, steer=robot.steer, mode=robot.mode.name)


@app.route("/api/stop", methods=["POST"])
def api_stop():
    robot.stop()
    return jsonify(stopped=True, mode=robot.mode.name)


# -- modes ---------------------------------------------------------------


@app.route("/api/modes")
def api_modes():
    return jsonify(modes=robot.mode_list(), active=robot.mode.name)


@app.route("/api/mode", methods=["POST"])
def api_mode():
    data = request.get_json(silent=True) or {}
    name = data.get("name")
    if not name:
        return jsonify(error="name required"), 400

    ok, message = robot.select_mode(name)
    if not ok:
        return jsonify(error=message), 409

    if "config" in data and isinstance(data["config"], dict):
        robot.mode.configure(**data["config"])
    return jsonify(active=robot.mode.name, message=message, status=robot.mode.status())


@app.route("/api/mode/config", methods=["POST"])
def api_mode_config():
    data = request.get_json(silent=True) or {}
    name = data.pop("name", None)
    mode = robot.modes.get(name) if name else robot.mode
    if mode is None:
        return jsonify(error=f"unknown mode '{name}'"), 400
    return jsonify(status=mode.configure(**data))


# -- camera and detection -------------------------------------------------


@app.route("/api/camera", methods=["POST"])
def api_camera():
    data = request.get_json(silent=True) or {}
    if "enabled" in data:
        robot.vision.set_camera(data["enabled"])
    return jsonify(enabled=robot.vision.camera_enabled)


@app.route("/api/detect", methods=["POST"])
def api_detect():
    if robot.detector is None:
        return jsonify(error="detector unavailable"), 503

    data = request.get_json(silent=True) or {}
    if "enabled" in data:
        robot.vision.set_detection(data["enabled"])
    if "conf" in data or "classes" in data:
        robot.detector.configure(conf=data.get("conf"), classes=data.get("classes"))
    if "target_fps" in data:
        try:
            robot.vision.detect_target_fps = max(0.0, min(60.0, float(data["target_fps"])))
        except (TypeError, ValueError):
            return jsonify(error="target_fps must be a number"), 400

    return jsonify(
        enabled=robot.vision.detect_enabled,
        conf=robot.detector.conf,
        classes=sorted(robot.detector.classes),
        target_fps=robot.vision.detect_target_fps,
    )


@app.route("/api/classes")
def api_classes():
    return jsonify(classes=robot.detector.class_names if robot.detector else [])


@app.route("/api/telemetry")
def api_telemetry():
    return jsonify(robot.telemetry())


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True)
    finally:
        robot.close()
