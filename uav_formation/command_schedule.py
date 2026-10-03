"""Optional simulation-time scheduling for persistent Plane heading targets."""
import math


class HeadingSchedule:
    def __init__(self, period):
        if not math.isfinite(period) or period <= 0:
            raise ValueError('heading period must be finite and positive')
        self.period = period
        self.last_time = None
        self.last_target = None
        self.last_clock = None

    def due(self, now, bearing, acceleration):
        if not all(math.isfinite(v) for v in (now, bearing, acceleration)) or acceleration <= 0:
            raise ValueError('invalid heading target or clock')
        target = (bearing % 360., acceleration)
        if self.last_clock is not None and now < self.last_clock:
            raise ValueError('simulation clock moved backwards')
        self.last_clock = now
        if self.last_time is not None:
            delta = (target[0] - self.last_target[0] + 180.) % 360. - 180.
            if abs(delta) < 1e-9 and abs(target[1] - self.last_target[1]) < 1e-9:
                return False
            if now - self.last_time < self.period - 1e-9:
                return False
        self.last_time, self.last_target = now, target
        return True
