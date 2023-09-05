import time

class PID():
    def __init__(self, Kp=0, Ki=0, Kd=0, I_max=1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.I_max = I_max
        self.current_time = 0
        self.previous_time = 0
        self.current_error = 0
        self.previous_error = 0
        self.P = 0
        self.I = 0
        self.D = 0

    def __call__(self, current_value, requested_value):
        self.current_time = time.time()
        delta_time = self.current_time - self.previous_time
        self.current_error = requested_value - current_value
        delta_error = self.current_error - self.previous_error

        self.P = self.Kp * self.current_error
        self.I += self.Ki * self.current_error * delta_time
        self.D = self.Kd * delta_error / delta_time

        # Integrator limit
        if self.I > self.I_max: self.I = self.I_max
        if self.I < -self.I_max: self.I = -self.I_max

        # Integrator Reset
        # if - 0.1 < self.current_error < 0.1: self.I = 0

        self.previous_time = self.current_time
        self.previous_error = self.current_error

        return self.P + self.I + self.D

class Limiter():
    def __init__(self, min_=0, max_=1):
        self.min = min_
        self.max = max_

    def __call__(self, value):
        if value > self.max:
            return self.max
        elif value < self.min:
            return self.min
        else:
            return value

def sgn(x):
    return (x > 0) - (x < 0)

def pwm2float(value):
    return (value - 1500) / 500

def float2pwm(value):
    return int(value * 500 + 1500)