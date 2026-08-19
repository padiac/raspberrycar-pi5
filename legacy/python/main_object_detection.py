import time
import cv2

import torch
from torchvision.models.detection import ssdlite320_mobilenet_v3_large, SSDLite320_MobileNet_V3_Large_Weights

from move import CarMove
from ultrasound import CarUltrasound
from infrared import CarInfrared
from camera import CarCamera
from detect import CarDetect


SCORE_THRESHOLD = 0.5


class Car(CarMove, CarUltrasound, CarInfrared, CarCamera, CarDetect):  # create class Car, which derives all the modules
    def __init__(self):
        CarMove.__init__(self)
        CarUltrasound.__init__(self)
        CarInfrared.__init__(self)
        CarCamera.__init__(self)
        CarDetect.__init__(self)

    def AllStop(self):
        CarMove.MotorStop(self)
        CarCamera.CameraCleanup(self)


def draw_detections(frame, boxes, labels, scores, categories):  # 在frame上画框（检测结果）
    for box, label, score in zip(boxes, labels, scores):
        if score < SCORE_THRESHOLD:
            continue
        x1, y1, x2, y2 = box.astype(int)
        class_name = categories[label] if label < len(categories) else str(label)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'{class_name}: {score:.2f}', (x1, max(y1 - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return frame


if __name__ == '__main__':
    try:
        car = Car()

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        video_out = cv2.VideoWriter('out.mp4', fourcc, 0.8, (640, 480))

        ##### Prepare for the torchvision object detection model #####
        # replaces the TensorFlow Object Detection API + frozen graph +
        # label map that this script used to load.
        print('Loading...')
        weights = SSDLite320_MobileNet_V3_Large_Weights.COCO_V1
        model = ssdlite320_mobilenet_v3_large(weights=weights)
        model.eval()
        categories = weights.meta['categories']
        preprocess = weights.transforms()
        print('Finish Load Model')
        ##### Finish the Preparation for the object detection model #####

        car.CameraInit()  # Initialize the camera

        with torch.no_grad():
            for frame in car.capture_continuous():
                t_start = time.time()  # 用来计算FPS

                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image_tensor = torch.from_numpy(rgb_frame).permute(2, 0, 1)  # HWC -> CHW
                input_tensor = preprocess(image_tensor)

                print('Running detection..')
                output = model([input_tensor])[0]  # 使用torchvision模型来Detect

                boxes = output['boxes'].numpy()
                labels = output['labels'].numpy()
                scores = output['scores'].numpy()

                print('Done.  Visualizing..')
                frame = draw_detections(frame, boxes, labels, scores, categories)

                car.VideoTransmission(frame)  # 向PC传输视频帧
                video_out.write(frame)

                mfps = 1 / (time.time() - t_start)  # 计算FPS
                print('FPS: ', mfps)

    except KeyboardInterrupt:
        print("Measurement stopped by User")
        car.AllStop()
        video_out.release()
