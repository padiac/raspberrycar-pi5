# Car's movement control (forward, back, left, right, brake)
# motor control

import gpio_setup  # noqa: F401 - selects the correct gpiochip for this kernel
from gpiozero import Motor


class CarMove(object):
    def __init__(self):
        # GPIO setting (BCM coding)
        # motor_a: motor_1 (forward) / motor_4 (backward)
        # motor_b: motor_5 (forward) / motor_6 (backward)
        self.motor_a = Motor(forward=18, backward=23)
        self.motor_b = Motor(forward=24, backward=25)

    def forward(self, speed):  # speed range: 0~100
        self.motor_a.forward(speed / 100)
        self.motor_b.forward(speed / 100)

    def back(self, speed):
        self.motor_a.backward(speed / 100)
        self.motor_b.backward(speed / 100)

    def left(self, speed):
        self.motor_a.backward(speed / 100)
        self.motor_b.forward(speed / 100)

    def right(self, speed):
        self.motor_a.forward(speed / 100)
        self.motor_b.backward(speed / 100)

    def brake(self):
        self.motor_a.stop()
        self.motor_b.stop()

    def forward_turn(self, speed_left, speed_right):
        self.motor_a.forward(speed_left / 100)
        self.motor_b.forward(speed_right / 100)

    def MotorStop(self):
        self.motor_a.stop()
        self.motor_b.stop()

    def set_motors(self, left, right):  # continuous control, each -1.0 (full back) ~ 1.0 (full forward)
        # Used by the web UI / gamepad, where the existing discrete
        # forward/back/left/right calls are too coarse.
        self.motor_a.value = max(-1.0, min(1.0, float(left)))
        self.motor_b.value = max(-1.0, min(1.0, float(right)))

    def track_move_left(self, speed, diff):
        self.motor_a.forward(0.4)
        self.motor_b.backward(0.4)


if __name__ == '__main__':
    try:
        car = CarMove()
        while True:
            car.track_move_left(40, 10)

    except KeyboardInterrupt:
        print("Measurement stopped by User")
        car.MotorStop()
