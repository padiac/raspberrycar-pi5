# Infrared obstacle avoidance module

import time

import gpio_setup  # noqa: F401 - selects the correct gpiochip for this kernel
from gpiozero import DigitalInputDevice


class CarInfrared(object):
    def __init__(self):
        # GPIO setting (BCM coding)
        self.infrared_right = DigitalInputDevice(8, pull_up=False)
        self.infrared_left = DigitalInputDevice(7, pull_up=False)

        self.left_tracking = DigitalInputDevice(16, pull_up=False)
        self.right_tracking = DigitalInputDevice(12, pull_up=False)

    def InfraredMeasure(self):
        left_measure = self.infrared_left.value  # if there is an obstacle, this will become 0; else, 1;
        right_measure = self.infrared_right.value

        return [left_measure, right_measure]

    def TrackingMreasure(self):
        left_tracking = self.left_tracking.value
        right_tracking = self.right_tracking.value  # 1黑0白

        return [left_tracking, right_tracking]


if __name__ == '__main__':
    try:
        car = CarInfrared()
        while True:
            [left, right] = car.InfraredMeasure()
            [l, r] = car.TrackingMreasure()
            # print(left, right)
            print(l, r)
            time.sleep(1)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
