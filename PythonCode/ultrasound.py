#  Ultrasonic ranging module

import time

import gpio_setup  # noqa: F401 - selects the correct gpiochip for this kernel
from gpiozero import DistanceSensor


class CarUltrasound(object):
    def __init__(self):
        # GPIO setting (BCM coding): trigger=20, echo=21
        # max_distance matches the HC-SR04's real range (~4m); gpiozero's
        # own default (1m) would clip valid longer readings.
        # partial=True: return as soon as a reading is available instead of
        # blocking until gpiozero's internal smoothing queue fills up (which
        # otherwise hangs here with no sensor wired in yet).
        self.sensor = DistanceSensor(echo=21, trigger=20, max_distance=4, partial=True)

        self.dist_mov_ave = 0

    def DistMeasure(self):  # distance measuing, in cm
        return self.sensor.distance * 100

    def DistMeasureMovingAverage(self):
        dist_current = self.DistMeasure()
        self.dist_mov_ave = 0.8 * dist_current + 0.2 * self.dist_mov_ave  # using the moving average of distance measured by sensor to reduce the error
        return self.dist_mov_ave


if __name__ == '__main__':
    try:
        car = CarUltrasound()
        while True:
            dist = car.DistMeasure()
            print("Measured Distance = {:.2f} cm".format(dist))
            time.sleep(1)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
