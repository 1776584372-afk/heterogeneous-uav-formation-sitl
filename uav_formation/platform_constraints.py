#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Platform capability checks for shared formation references.

This module is deliberately independent of ROS and flight-controller APIs. It
turns the common ICC route into per-slot kinematic references and checks those
references against a vehicle capability profile. The same functions can be
used by the current multirotor node and by a future fixed-wing adapter.
"""
from dataclasses import dataclass
import math


GRAVITY = 9.80665


@dataclass(frozen=True)
class PlatformCapability:
    """Bounds used by the reference feasibility layer.

    ``min_speed`` and ``max_speed`` are ground-speed bounds for multirotors.
    For fixed-wing profiles they represent the no-wind airspeed bounds until a
    wind-aware airspeed conversion is added to the guidance adapter.
    """

    name: str
    min_speed: float = 0.0
    max_speed: float = math.inf
    max_lateral_accel: float = math.inf
    max_bank_rad: float = math.inf
    max_bank_rate_rad_s: float = math.inf
    max_climb_rate: float = math.inf
    max_descent_rate: float = math.inf
    speed_kind: str = "ground"

    def __post_init__(self):
        if self.min_speed < 0.0:
            raise ValueError("min_speed must be >= 0")
        if self.max_speed <= 0.0 or self.max_speed < self.min_speed:
            raise ValueError("max_speed must be >= min_speed and > 0")
        if self.max_lateral_accel <= 0.0:
            raise ValueError("max_lateral_accel must be > 0")
        if self.speed_kind not in ("ground", "airspeed"):
            raise ValueError("speed_kind must be 'ground' or 'airspeed'")


def multirotor_capability(max_speed=30.0, max_lateral_accel=5.0):
    """Return the default capability profile for the existing NMPC chain."""
    return PlatformCapability(
        name="multirotor",
        min_speed=0.0,
        max_speed=float(max_speed),
        max_lateral_accel=float(max_lateral_accel),
    )


def fixedwing_capability(min_airspeed, max_airspeed,
                         max_bank_deg=35.0,
                         max_bank_rate_deg_s=25.0,
                         max_climb_rate=math.inf,
                         max_descent_rate=math.inf):
    """Build a no-wind fixed-wing capability profile."""
    return PlatformCapability(
        name="fixedwing",
        min_speed=float(min_airspeed),
        max_speed=float(max_airspeed),
        max_lateral_accel=math.inf,
        max_bank_rad=math.radians(float(max_bank_deg)),
        max_bank_rate_rad_s=math.radians(float(max_bank_rate_deg_s)),
        max_climb_rate=float(max_climb_rate),
        max_descent_rate=float(max_descent_rate),
        speed_kind="airspeed",
    )


@dataclass(frozen=True)
class SlotReference:
    """One member's geometric and kinematic reference at a center progress."""

    s: float
    x: float
    y: float
    course: float
    curvature: float
    speed: float
    lateral_accel: float
    bank_required: float
    rx: float
    ry: float
    stretch: float = 1.0


@dataclass(frozen=True)
class ConstraintViolation:
    vehicle_index: int
    platform: str
    s: float
    constraint: str
    value: float
    limit: float
    message: str


@dataclass(frozen=True)
class FeasibilityReport:
    feasible: bool
    samples: int
    violations: tuple
    min_speed: float
    max_speed: float
    max_lateral_accel: float
    max_bank_deg: float

    @property
    def summary(self):
        if self.feasible:
            return "feasible"
        if not self.violations:
            return "infeasible"
        first = self.violations[0]
        return "%s: %s" % (first.platform, first.message)


def _wrap_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _slot_kinematics(route, s, rx, ry):
    """Evaluate a slot reference using the center arc-length parameter."""
    x, y, psi, kappa, center_speed = route.eval_center_state(s)
    if abs(rx) < 1e-12 and 1.0 - ry*kappa <= 1e-6:
        raise ValueError('lateral slot requires positive q')
    c = math.cos(psi)
    sn = math.sin(psi)
    tx, ty = c, sn
    nx, ny = -sn, c

    # d(center + rx*t + ry*n)/ds for a planar Frenet frame.
    dx_ds = (1.0 - ry * kappa) * tx + rx * kappa * nx
    dy_ds = (1.0 - ry * kappa) * ty + rx * kappa * ny
    stretch = math.hypot(dx_ds, dy_ds)
    if stretch <= 1e-9:
        raise ValueError("slot mapping collapses at s=%.3f" % float(s))

    px = x + rx * tx + ry * nx
    py = y + rx * ty + ry * ny
    course = math.atan2(dy_ds, dx_ds)
    speed = max(0.0, float(center_speed) * stretch)

    # Differentiate the member course with respect to center arc length. A
    # finite difference is robust across the table's straight/transition/arc
    # segment boundaries and is only used by the slow preflight checker.
    h = min(0.05, max(1e-3, float(getattr(route, "step", 0.05))))
    s0 = max(0.0, float(s) - h)
    s1 = min(route.total_length(), float(s) + h)
    if s1 - s0 <= 1e-9:
        member_curvature = 0.0
    else:
        def member_course(q):
            _x, _y, qpsi, qk, _v = route.eval_center_state(q)
            qc, qsn = math.cos(qpsi), math.sin(qpsi)
            qtx, qty = qc, qsn
            qnx, qny = -qsn, qc
            qdx = (1.0 - ry * qk) * qtx + rx * qk * qnx
            qdy = (1.0 - ry * qk) * qty + rx * qk * qny
            return math.atan2(qdy, qdx)

        dcourse_ds = _wrap_angle(member_course(s1) - member_course(s0)) / (s1 - s0)
        member_curvature = dcourse_ds / stretch

    lateral_accel = speed * speed * abs(member_curvature)
    bank_required = math.atan2(speed * speed * member_curvature, GRAVITY)
    return SlotReference(
        s=float(s), x=px, y=py, course=course,
        curvature=member_curvature, speed=speed,
        lateral_accel=lateral_accel, bank_required=bank_required,
        rx=float(rx), ry=float(ry), stretch=stretch,
    )


def slot_reference(route, s, slot):
    """Public slot mapper: ``slot`` is ``(rx, ry, rz)``; z is ignored here."""
    return _slot_kinematics(route, s, float(slot[0]), float(slot[1]))


@dataclass(frozen=True)
class ProgressInterval:
    """One member's admissible public progress interval at center arc length."""

    vehicle_index: int
    platform: str
    s: float
    lower: float
    upper: float
    member_speed_scale: float
    member_curvature: float


@dataclass(frozen=True)
class TeamProgressSlice:
    """Intersection of all member intervals at one center arc length."""

    s: float
    lower: float
    upper: float
    intervals: tuple

    @property
    def feasible(self):
        return self.lower <= self.upper + 1e-9


@dataclass(frozen=True)
class ProgressFeasibilityReport:
    feasible: bool
    slices: tuple
    first_infeasible_s: float = None

    @property
    def min_progress(self):
        values = [x.lower for x in self.slices if x.feasible]
        return min(values) if values else math.inf

    @property
    def max_progress(self):
        values = [x.upper for x in self.slices if x.feasible]
        return max(values) if values else -math.inf


def progress_interval_at(route, s, vehicle_index, slot, capability):
    """Map one member's platform limits into a public progress interval.

    For the lateral-slot model, member speed is
    ``v_i = dot{s} * |1 - r_y*kappa|`` and member curvature is
    ``kappa_i = kappa / (1 - r_y*kappa)``. The acceleration and bank limits
    therefore become upper bounds on the shared progress rate ``dot{s}``.
    """
    ref = slot_reference(route, s, slot)
    scale = ref.stretch
    if scale <= 1e-9:
        raise ValueError("member path scale is zero at s=%.3f" % float(s))
    lower = float(capability.min_speed) / scale
    upper = float(capability.max_speed) / scale
    abs_kappa = abs(ref.curvature)
    if abs_kappa > 1e-12:
        if math.isfinite(capability.max_lateral_accel):
            upper = min(upper, math.sqrt(
                capability.max_lateral_accel / (scale * scale * abs_kappa)))
        if math.isfinite(capability.max_bank_rad):
            if capability.max_bank_rad >= math.pi / 2.0:
                pass
            else:
                bank_accel = GRAVITY * math.tan(capability.max_bank_rad)
                upper = min(upper, math.sqrt(bank_accel /
                                             (scale * scale * abs_kappa)))
    return ProgressInterval(
        vehicle_index=int(vehicle_index), platform=capability.name,
        s=float(s), lower=max(0.0, lower), upper=max(0.0, upper),
        member_speed_scale=scale, member_curvature=ref.curvature,
    )


def check_public_progress_feasibility(route, slots, capabilities,
                                      sample_step=0.5):
    """Compute the all-member public-progress intersection along the route."""
    if len(slots) != len(capabilities):
        raise ValueError("slots and capabilities must have the same length")
    total = route.total_length()
    count = max(1, int(math.ceil(total / max(1e-3, float(sample_step)))))
    slices = []
    first_bad = None
    for k in range(count + 1):
        s = total * k / count
        intervals = []
        for i, (slot, capability) in enumerate(zip(slots, capabilities)):
            try:
                intervals.append(progress_interval_at(
                    route, s, i, slot, capability))
            except ValueError:
                intervals.append(ProgressInterval(
                    i, capability.name, s, 1.0, 0.0, 0.0, 0.0))
        lower = max((x.lower for x in intervals), default=0.0)
        upper = min((x.upper for x in intervals), default=0.0)
        current = TeamProgressSlice(s, lower, upper, tuple(intervals))
        slices.append(current)
        if not current.feasible and first_bad is None:
            first_bad = s
    return ProgressFeasibilityReport(
        feasible=first_bad is None,
        slices=tuple(slices), first_infeasible_s=first_bad)


def _check_reference(vehicle_index, ref, capability):
    violations = []
    if ref.speed < capability.min_speed - 1e-6:
        violations.append(ConstraintViolation(
            vehicle_index, capability.name, ref.s, "min_speed", ref.speed,
            capability.min_speed,
            "speed %.2f below minimum %.2f" % (ref.speed, capability.min_speed)))
    if ref.speed > capability.max_speed + 1e-6:
        violations.append(ConstraintViolation(
            vehicle_index, capability.name, ref.s, "max_speed", ref.speed,
            capability.max_speed,
            "speed %.2f above maximum %.2f" % (ref.speed, capability.max_speed)))
    if ref.lateral_accel > capability.max_lateral_accel + 1e-6:
        violations.append(ConstraintViolation(
            vehicle_index, capability.name, ref.s, "max_lateral_accel",
            ref.lateral_accel, capability.max_lateral_accel,
            "lateral acceleration %.2f above maximum %.2f"
            % (ref.lateral_accel, capability.max_lateral_accel)))
    if abs(ref.bank_required) > capability.max_bank_rad + 1e-6:
        violations.append(ConstraintViolation(
            vehicle_index, capability.name, ref.s, "max_bank",
            abs(ref.bank_required), capability.max_bank_rad,
            "required bank %.1f deg above maximum %.1f deg"
            % (math.degrees(abs(ref.bank_required)),
               math.degrees(capability.max_bank_rad))))
    return violations


def check_formation_feasibility(route, slots, capabilities, sample_step=0.5):
    """Check every slot against its platform capability over the full route."""
    if len(slots) != len(capabilities):
        raise ValueError("slots and capabilities must have the same length")
    total = route.total_length()
    count = max(1, int(math.ceil(total / max(1e-3, float(sample_step)))))
    s_values = [total * i / count for i in range(count + 1)]
    violations = []
    speeds = []
    lateral_accels = []
    banks = []
    previous = [None] * len(slots)

    for s in s_values:
        for i, (slot, capability) in enumerate(zip(slots, capabilities)):
            try:
                ref = slot_reference(route, s, slot)
            except ValueError as exc:
                violations.append(ConstraintViolation(
                    i, capability.name, s, "slot_mapping", 0.0, 1.0,
                    str(exc)))
                continue
            speeds.append(ref.speed)
            lateral_accels.append(ref.lateral_accel)
            banks.append(abs(ref.bank_required))
            violations.extend(_check_reference(i, ref, capability))
            if previous[i] is not None:
                ds = max(1e-6, s - previous[i].s)
                nu0 = previous[i].speed / previous[i].stretch
                nu1 = ref.speed / ref.stretch
                dt = 2 * ds / max(nu0 + nu1, 1e-3)
                bank_rate = abs(_wrap_angle(ref.bank_required - previous[i].bank_required)) / dt
                if bank_rate > capability.max_bank_rate_rad_s + 1e-6:
                    violations.append(ConstraintViolation(
                        i, capability.name, s, "max_bank_rate", bank_rate,
                        capability.max_bank_rate_rad_s,
                        "bank rate %.1f deg/s above maximum %.1f deg/s"
                        % (math.degrees(bank_rate),
                           math.degrees(capability.max_bank_rate_rad_s))))
            previous[i] = ref

    return FeasibilityReport(
        feasible=not violations,
        samples=len(s_values),
        violations=tuple(violations),
        min_speed=min(speeds) if speeds else 0.0,
        max_speed=max(speeds) if speeds else 0.0,
        max_lateral_accel=max(lateral_accels) if lateral_accels else 0.0,
        max_bank_deg=math.degrees(max(banks)) if banks else 0.0,
    )
