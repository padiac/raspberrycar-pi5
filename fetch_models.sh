#!/bin/bash
# Download the model weights the vision code needs (~52 MB total).
# They're deliberately not in git; run this once after cloning.
#
#   ./fetch_models.sh
#
# YOLO26n is fetched as .pt and then exported to NCNN, which is what the
# detector actually loads: same model, but NCNN is built for ARM CPUs and
# measured ~4x faster on a Pi 5 (58 fps vs 14 fps at 320px).

set -euo pipefail
cd "$(dirname "$0")"
mkdir -p models
cd models

ZOO=https://github.com/opencv/opencv_zoo/raw/main/models

fetch() {
    if [ -f "$2" ]; then
        echo "  $2 already here, skipping"
    else
        echo "  downloading $2"
        curl -sL -o "$2" "$1"
    fi
}

echo "OpenCV model zoo (face detection + recognition):"
fetch "$ZOO/face_detection_yunet/face_detection_yunet_2023mar.onnx" yunet.onnx
fetch "$ZOO/face_recognition_sface/face_recognition_sface_2021dec.onnx" sface.onnx

echo "YOLO26n (general object detection):"
if [ -d yolo26n_ncnn_model ]; then
    echo "  yolo26n_ncnn_model already here, skipping"
else
    python3 - <<'PY'
from ultralytics import YOLO
model = YOLO("yolo26n.pt")          # downloads on first use
model.export(format="ncnn", imgsz=320)
PY
fi

echo
echo "done:"
ls -la
