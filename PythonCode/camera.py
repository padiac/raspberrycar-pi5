import time
import numpy as np
import cv2
import socket
import struct

from picamera2 import Picamera2


class CarCamera(object):
    def __init__(self):
        # initialize the camera and grab a reference to the raw camera capture

        HOST = '192.168.12.90'  # ip of PC
        PORT = 8000  # 随便设置一个，对应起来就行
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.connect((HOST, PORT))

    def CameraInit(self):  # 初始化摄像头 (picamera2)
        self.camera = Picamera2()
        # NOTE: Picamera2's "RGB888" format is actually delivered as BGR byte
        # order in memory, which is exactly what OpenCV (cv2) expects. This
        # replaces the old picamera + PiRGBArray(format="bgr") pattern.
        config = self.camera.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.camera.configure(config)
        self.camera.start()
        time.sleep(2)  # wait for camera starting 相机需要时间预热

        return self.camera

    def CaptureFrame(self):  # capture a single BGR frame as a numpy array
        return self.camera.capture_array()

    def capture_continuous(self):  # generator, replaces picamera's capture_continuous(rawCapture, ...)
        while True:
            yield self.camera.capture_array()

    def VideoTransmission(self, frame):  # transmit video from Pi to PC
        result, imgencode = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 10])  # 编码
        try:
            self.server.sendall(struct.pack('i', imgencode.shape[0]))  # 发送数据长度作为校验
            self.server.sendall(imgencode)
            print("have sent one frame")
        except Exception:
            print("fail to send the frame")

    def CameraCleanup(self):
        self.server.sendall(struct.pack('c', b'1'))  # 发送关闭消息
        self.server.close()
        self.camera.close()


if __name__ == '__main__':
    try:
        car = CarCamera()
        car.CameraInit()

        for frame in car.capture_continuous():
            car.VideoTransmission(frame)  # 向PC传输视频帧

    except KeyboardInterrupt:
        car.CameraCleanup()
