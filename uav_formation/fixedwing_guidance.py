#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure fixed-wing guidance adapter for a shared slot reference.

The module produces an abstract guidance command. A later ArduPlane bridge can
map that command to the MAVLink interface supported by the deployed firmware.
Keeping this layer ROS-free makes the lateral guidance testable in isolation.
"""
from dataclasses import dataclass
import math

from uav_formation.platform_constraints import GRAVITY, PlatformCapability


def _wrap_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class FixedWingState:
    x: float
    y: float
    altitude: float
    course: float
    airspeed: float


@dataclass(frozen=True)
class FixedWingCommand:
    target_course: float
    target_airspeed: float
    target_bank: float
    target_altitude: float
    lateral_accel: float
    cross_track_error: float
    course_error: float


class FixedWingGuidance:
    """Convert a common slot reference into fixed-wing guidance targets.

    The feedback law is a small L1-like lateral layer: feed forward the
    reference curvature, then correct cross-track and course errors. It does
    not command servos or MAVLink directly.
    """

    def __init__(self, capability: PlatformCapability,
                 cross_track_gain=0.8, course_gain=1.2):
        if capability.name != "fixedwing":
            raise ValueError("FixedWingGuidance requires a fixedwing capability")
        self.capability = capability
        self.cross_track_gain = float(cross_track_gain)
        self.course_gain = float(course_gain)
        if self.cross_track_gain <= 0.0 or self.course_gain <= 0.0:
            raise ValueError("guidance gains must be > 0")

    def update(self, reference, state: FixedWingState, target_altitude=None):
        """Return a command for a ``SlotReference`` and measured state."""
        nx = -math.sin(reference.course)
        ny = math.cos(reference.course)
        cross_track = nx * (state.x - reference.x) + ny * (state.y - reference.y)
        course_error = _wrap_angle(reference.course - state.course)

        speed = max(state.airspeed, 1e-3)
        feed_forward = speed * speed * reference.curvature
        correction = (-self.cross_track_gain * cross_track
                      + self.course_gain * speed * course_error)
        lateral_accel = feed_forward + correction
        bank = math.atan2(lateral_accel, GRAVITY)
        bank = max(-self.capability.max_bank_rad,
                   min(self.capability.max_bank_rad, bank))
        airspeed = max(self.capability.min_speed,
                       min(self.capability.max_speed, reference.speed))
        altitude = state.altitude if target_altitude is None else float(target_altitude)
        return FixedWingCommand(
            target_course=float(reference.course),
            target_airspeed=float(airspeed),
            target_bank=float(bank),
            target_altitude=altitude,
            lateral_accel=float(lateral_accel),
            cross_track_error=float(cross_track),
            course_error=float(course_error),
        )
