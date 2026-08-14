"""A small PID controller.

Same maths as the original project's PID_controller.py, with the accumulator
bounded. Unbounded integral windup is the classic way a controller like this
misbehaves: if the target is unreachable for a while the I term grows without
limit and the output stays saturated long after the error is gone.
"""


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, setpoint=0.0, integral_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral_limit = integral_limit

        self.error = 0.0
        self.last_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.error = self.last_error = self.integral = 0.0

    def update(self, measured, dt=None):
        self.error = self.setpoint - measured

        proportional = self.kp * self.error

        self.integral += self.error * (dt if dt else 1.0)
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        integral = self.ki * self.integral

        derivative_input = (self.error - self.last_error)
        if dt:
            derivative_input /= dt
        derivative = self.kd * derivative_input
        self.last_error = self.error

        return proportional + integral + derivative
