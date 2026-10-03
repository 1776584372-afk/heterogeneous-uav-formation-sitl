"""No-wind, fixed lateral-slot capability model. SI units, ENU, left normal."""
from dataclasses import dataclass
from enum import IntEnum
import math

G = 9.80665
Q_EPSILON = 1e-6
KAPPA_EPSILON = 1e-12


class Limit(IntEnum):
    SPEED_MIN = 1
    SPEED_MAX = 2
    LATERAL_ACCEL = 3
    HEADING_RATE = 4
    BANK = 5
    TURN_RADIUS = 6
    TANGENTIAL_MIN = 7
    TANGENTIAL_MAX = 8
    BANK_RATE = 9
    GEOMETRY = 10


@dataclass(frozen=True)
class MemberSpec:
    member_id: int
    platform: str
    r_y: float
    v_min: float
    v_max: float
    a_lat_max: float
    yaw_rate_max: float | None = None
    bank_max: float | None = None
    bank_rate_max: float | None = None
    a_t_min: float = -2.0
    a_t_max: float = 2.0
    turn_radius_min: float | None = None

    def __post_init__(self):
        if self.platform not in ('multirotor', 'fixedwing'):
            raise ValueError('unknown platform')
        if not isinstance(self.member_id, int) or self.member_id < 0:
            raise ValueError('member_id must be a nonnegative integer')
        for name in ('r_y', 'v_min', 'v_max', 'a_lat_max', 'a_t_min', 'a_t_max'):
            if not math.isfinite(getattr(self, name)):
                raise ValueError('%s must be finite' % name)
        if not 0 <= self.v_min < self.v_max or self.a_lat_max <= 0:
            raise ValueError('invalid speed or acceleration bounds')
        if not self.a_t_min < 0 < self.a_t_max:
            raise ValueError('tangential limits must straddle zero')
        if self.platform == 'fixedwing' and (self.v_min <= 0 or self.bank_max is None):
            raise ValueError('fixedwing needs positive v_min and bank_max')
        for name in ('yaw_rate_max', 'bank_max', 'bank_rate_max', 'turn_radius_min'):
            value = getattr(self, name)
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError('%s must be positive and finite or null' % name)
        if self.bank_max is not None and self.bank_max >= math.pi / 2:
            raise ValueError('bank_max must be below pi/2 radians')


def slot_scale(r_y, kappa):
    q = 1.0 - float(r_y) * float(kappa)
    if not math.isfinite(q) or q <= Q_EPSILON:
        raise ValueError('GEOMETRY: q <= q_epsilon')
    return q


def static_progress_interval(member, kappa):
    q = slot_scale(member.r_y, kappa)
    lower = member.v_min / q
    bounds = [(member.v_max / q, Limit.SPEED_MAX)]
    ak = abs(kappa)
    if ak >= KAPPA_EPSILON:
        bounds.append((math.sqrt(member.a_lat_max / (q * ak)), Limit.LATERAL_ACCEL))
        if member.yaw_rate_max is not None:
            bounds.append((member.yaw_rate_max / ak, Limit.HEADING_RATE))
        if member.bank_max is not None:
            bounds.append((math.sqrt(G * math.tan(member.bank_max) / (q * ak)), Limit.BANK))
        if member.turn_radius_min is not None and q / ak < member.turn_radius_min:
            return lower, -1.0, (Limit.SPEED_MIN, Limit.TURN_RADIUS)
    upper, active = min(bounds, key=lambda x: (x[0], int(x[1])))
    return lower, upper, (Limit.SPEED_MIN, active)


def team_interval(members, kappa):
    if not members or len({m.member_id for m in members}) != len(members):
        raise ValueError('members must be nonempty with unique IDs')
    intervals = [(m, static_progress_interval(m, kappa)) for m in members]
    low_m, low = max(intervals, key=lambda x: (x[1][0], -x[0].member_id))
    high_m, high = min(intervals, key=lambda x: (x[1][1], x[0].member_id))
    return low[0], high[1], (low_m.member_id, low[2][0]), (high_m.member_id, high[2][1])


def member_demands(member, kappa, nu):
    if not math.isfinite(nu) or nu < 0:
        raise ValueError('nu must be finite and nonnegative')
    q = slot_scale(member.r_y, kappa)
    return q * nu, kappa / q, nu * kappa, math.atan(q * kappa * nu * nu / G)


def dynamic_margin(member, kappa, kappa_prime, z, dz_ds):
    if not all(math.isfinite(v) for v in (kappa, kappa_prime, z, dz_ds)) or z < 0:
        raise ValueError('invalid dynamic state')
    q = slot_scale(member.r_y, kappa)
    nu = math.sqrt(z)
    v, ki, omega, phi = member_demands(member, kappa, nu)
    at = .5 * q * dz_ds - member.r_y * kappa_prime * z
    b = q * kappa * z / G
    db = ((1 - 2 * member.r_y * kappa) * kappa_prime * z + q * kappa * dz_ds) / G
    phi_dot = nu * db / (1 + b * b)
    margins = {
        Limit.SPEED_MIN: v - member.v_min,
        Limit.SPEED_MAX: member.v_max - v,
        Limit.LATERAL_ACCEL: member.a_lat_max - abs(q * kappa * z),
        Limit.TANGENTIAL_MIN: at - member.a_t_min,
        Limit.TANGENTIAL_MAX: member.a_t_max - at,
        Limit.GEOMETRY: q - Q_EPSILON,
    }
    if member.yaw_rate_max is not None:
        margins[Limit.HEADING_RATE] = member.yaw_rate_max - abs(omega)
    if member.bank_max is not None:
        margins[Limit.BANK] = member.bank_max - abs(phi)
    if member.bank_rate_max is not None:
        margins[Limit.BANK_RATE] = member.bank_rate_max - abs(phi_dot)
    if member.turn_radius_min is not None:
        margins[Limit.TURN_RADIUS] = 1 / member.turn_radius_min - abs(ki)
    return margins


def normalized_min_margin(member, margins):
    """Dimensionless scalar for ROS diagnostics; exports retain each SI margin."""
    scales={Limit.SPEED_MIN:member.v_max, Limit.SPEED_MAX:member.v_max,
            Limit.LATERAL_ACCEL:member.a_lat_max, Limit.TANGENTIAL_MIN:-member.a_t_min,
            Limit.TANGENTIAL_MAX:member.a_t_max, Limit.GEOMETRY:1.,
            Limit.HEADING_RATE:member.yaw_rate_max, Limit.BANK:member.bank_max,
            Limit.BANK_RATE:member.bank_rate_max,
            Limit.TURN_RADIUS:1/member.turn_radius_min if member.turn_radius_min else None}
    return min(value/scales[limit] for limit,value in margins.items())
