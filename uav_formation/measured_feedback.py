"""Timestamped measured planar states; independent of ROS and command integration."""
from collections import deque
from dataclasses import dataclass
import math
import threading


class FeedbackError(ValueError):
    pass


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_yaw(x, y, z, w):
    values = (x, y, z, w)
    if not all(math.isfinite(v) for v in values):
        raise FeedbackError('nonfinite quaternion')
    norm = math.sqrt(sum(v*v for v in values))
    if norm < 1e-6:
        raise FeedbackError('zero quaternion')
    x, y, z, w = (v/norm for v in values)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


@dataclass(frozen=True)
class WorldTransform:
    east: float = 0.
    north: float = 0.
    yaw: float = 0.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.east, self.north, self.yaw)):
            raise ValueError('finite world transform required')

    def position(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.east+c*x-s*y, self.north+s*x+c*y

    def local_position(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y = x-self.east, y-self.north
        return c*x+s*y, -s*x+c*y

    def heading(self, yaw):
        return wrap_angle(yaw+self.yaw)


@dataclass(frozen=True)
class MeasuredState:
    x: float
    y: float
    yaw: float
    stamp: float
    position_stamp: float
    yaw_stamp: float
    position_age: float
    yaw_age: float
    extrapolation: float

    @property
    def state(self):
        return (self.x, self.y, self.yaw)


class MeasuredFeedback:
    """Interpolate streams; short extrapolation uses *measured* sample slopes only.

    Stamp and target use one source clock. Arrival times use monotonic wall time.
    Reject clock jumps, stale streams, long gaps and unsupported extrapolation.
    No command or MPC state enters this buffer.
    """
    def __init__(self, transform=None, max_age=.5, max_extrapolation=.2, max_gap=.25):
        self.transform = transform or WorldTransform()
        self.max_age = max_age
        self.max_extrapolation = max_extrapolation
        self.max_gap = max_gap
        if not 0 < max_extrapolation <= max_age or max_gap <= 0:
            raise ValueError('invalid feedback timing limits')
        self.positions = deque(maxlen=64)
        self.headings = deque(maxlen=64)
        self.fault = None
        self.lock = threading.RLock()

    def invalidate(self, reason):
        with self.lock:
            self.positions.clear()
            self.headings.clear()
            self.fault = str(reason)

    def _append(self, stream, stamp, values, received):
        if not all(math.isfinite(v) for v in (stamp, received, *values)) or stamp <= 0:
            self.invalidate('invalid measurement or timestamp')
            raise FeedbackError(self.fault)
        if stream and stamp < stream[-1][0]-1e-9:
            self.invalidate('source clock moved backwards/out-of-order sample')
            raise FeedbackError(self.fault)
        if stream and stamp == stream[-1][0]:
            return  # duplicates must not refresh stream age
        stream.append((stamp, tuple(values), received))
        self.fault = None

    def push_position(self, stamp, x, y, received):
        with self.lock:
            self._append(self.positions, stamp, self.transform.position(x, y), received)

    def push_yaw(self, stamp, yaw, received):
        if not math.isfinite(yaw):
            self.invalidate('nonfinite measured yaw')
            raise FeedbackError(self.fault)
        with self.lock:
            self._append(self.headings, stamp, (self.transform.heading(yaw),), received)

    def push_pose(self, stamp, x, y, yaw, received):
        with self.lock:
            self.push_position(stamp, x, y, received)
            self.push_yaw(stamp, yaw, received)

    def _sample(self, stream, target, now, arrival_now, angular=False):
        if not stream:
            raise FeedbackError(self.fault or 'missing measurement')
        newest, _, arrival = stream[-1]
        if now-newest > self.max_age or arrival_now-arrival > self.max_age:
            raise FeedbackError('stale measurement')
        if newest > now+.05 or target > now+.05 or arrival > arrival_now+.001:
            raise FeedbackError('future timestamp/clock mismatch')
        if target < stream[0][0]-1e-9:
            raise FeedbackError('no measurement history at requested time')
        for stamp, value, _ in reversed(stream):
            if abs(stamp-target) < 1e-9:
                return value, newest, 0.
        if len(stream) < 2:
            raise FeedbackError('need two measurements for time alignment')
        if target > newest:
            if target-newest > self.max_extrapolation:
                raise FeedbackError('extrapolation limit exceeded')
            first, second = stream[-2], stream[-1]
        else:
            first, second = next((a, b) for a, b in zip(stream, list(stream)[1:])
                                 if a[0] <= target <= b[0])
        gap = second[0]-first[0]
        if gap <= 1e-6 or gap > self.max_gap:
            raise FeedbackError('measurement gap exceeds alignment limit')
        fraction = (target-first[0])/gap
        difference = [b-a for a, b in zip(first[1], second[1])]
        if angular:
            difference = [wrap_angle(difference[0])]
        value = tuple(a+fraction*d for a, d in zip(first[1], difference))
        if angular:
            value = (wrap_angle(value[0]),)
        return value, newest, max(0., target-newest)

    def at(self, target, now, arrival_now):
        if not all(math.isfinite(v) for v in (target, now, arrival_now)):
            raise FeedbackError('nonfinite alignment time')
        if target > now + .05:
            raise FeedbackError('future timestamp/clock mismatch')
        with self.lock:
            # The control loop's source timestamp may lag the telemetry
            # receiver by one or two frames.  Use the newest buffered source
            # clock for freshness/future checks while keeping ``target`` as
            # the requested interpolation time.  Holding the lock across
            # both samples prevents a new frame from racing the check.
            newest_stamps = [stream[-1][0] for stream in (self.positions, self.headings)
                             if stream]
            current = max([now, *newest_stamps])
            xy, ps, pe = self._sample(self.positions, target, current, arrival_now)
            yaw, ys, ye = self._sample(self.headings, target, current, arrival_now, True)
        return MeasuredState(*xy, yaw[0], target, ps, ys, current-ps, current-ys, max(pe, ye))


def command_endpoint(state, speed, yaw_rate, hold):
    """A future setpoint, deliberately separate from the measured feedback state."""
    if not all(math.isfinite(v) for v in (*state, speed, yaw_rate, hold)) or hold < 0:
        raise ValueError('invalid command endpoint')
    x, y, yaw = state
    half = yaw_rate*hold/2
    factor = math.sin(half)/half if abs(half) > 1e-8 else 1.
    return (x+speed*hold*factor*math.cos(yaw+half),
            y+speed*hold*factor*math.sin(yaw+half), wrap_angle(yaw+yaw_rate*hold))
