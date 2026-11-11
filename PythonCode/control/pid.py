"""A small PID controller.

Same maths as the original project's PID_controller.py, with the accumulator
bounded. Unbounded integral windup is the classic way a controller like this
misbehaves: if the target is unreachable for a while the I term grows without
limit and the output stays saturated long after the error is gone.

The derivative term can be fed from outside, which is the fix for the second
classic way it misbehaves. See `update()`.
"""


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, setpoint=0.0, integral_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral_limit = integral_limit

        self.error = 0.0
        #: None until there has been one update. Differencing the first
        #: error against a made-up zero turned the whole of it into one
        #: tick's change: the follower's distance loop, 9cm out on its
        #: first tick, asked for 0.9 from the D term alone.
        self.last_error = None
        self.integral = 0.0
        #: True when the last update's derivative came from a rate handed in
        #: rather than from subtracting two measurements. Reported so that a
        #: controller quietly falling back to differencing is visible.
        self.measured_rate = False

    def reset(self):
        self.error = self.integral = 0.0
        self.last_error = None
        self.measured_rate = False

    def update(self, measured, dt=None, rate=None):
        """One step. `rate` is d(measured)/dt, if somebody already knows it.

        Differencing two measurements to get a derivative amplifies whatever
        noise is on them by 1/dt, which at this car's 20Hz tick is a factor
        of twenty before the gain is even applied. Fed raw detector output,
        the steering loop's D term was producing four times more command
        from frame-to-frame jitter than the P term was producing from the
        actual error, and the car hunted left and right at a ball that was
        not moving.

        So when a caller holds an estimator that carries velocity as part of
        its state -- see `control/estimator.py` -- it passes that rate here
        instead. This is the standard "derivative on measurement" form, and
        it fixes a second thing for free: differencing the *error* makes the
        D term spike every time the setpoint moves, because the whole step
        lands in one tick's difference. A rate of the measurement alone
        cannot do that, so changing the target distance no longer kicks.

        Without `rate` the old behaviour is unchanged, which matters because
        not every caller has an estimator and a controller that silently
        needed one would be worse than this.
        """
        self.error = self.setpoint - measured

        proportional = self.kp * self.error

        self.integral += self.error * (dt if dt else 1.0)
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        integral = self.ki * self.integral

        if rate is None:
            self.measured_rate = False
            derivative_input = (0.0 if self.last_error is None
                                else self.error - self.last_error)
            if dt:
                derivative_input /= dt
        else:
            self.measured_rate = True
            # error = setpoint - measured, so with a setpoint that is not
            # moving, d(error)/dt is minus the rate of the measurement.
            derivative_input = -rate
        derivative = self.kd * derivative_input
        self.last_error = self.error

        return proportional + integral + derivative
