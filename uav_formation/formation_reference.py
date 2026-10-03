#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Platform-neutral ICC route planning with capability adaptation."""
from dataclasses import dataclass
import math

from uav_formation.icc_traj import IccRoute
from uav_formation.platform_constraints import (
    check_formation_feasibility,
    check_public_progress_feasibility,
)


@dataclass(frozen=True)
class RoutePlan:
    route: IccRoute
    center_speed: float
    center_radius: float
    report: object
    attempts: int
    selected_progress_min: float = 0.0
    selected_progress_max: float = 0.0
    progress_report: object = None


def _default_radius(slots, turn_margin):
    half_width = max((abs(float(slot[1])) for slot in slots), default=0.0)
    return half_width + max(0.0, float(turn_margin))


def build_feasible_route(waypoints, slots, capabilities, center_speed,
                         center_radius=0.0, turn_margin=5.0,
                         trans_len=None, sample_step=0.5,
                         speed_floor=None, max_radius=500.0,
                         max_attempts=80):
    """Build an ICC route accepted by every platform profile.

    The planner first computes a public-progress interval for every member and
    intersects those intervals at each center arc length. It selects the
    requested progress when it is inside the intersection; otherwise it picks
    the nearest feasible value. If an intersection is empty, the geometry is
    reconstructed with a larger center radius. Slot spacing is left to the
    caller because changing it changes the mission's formation geometry.
    """
    if len(slots) != len(capabilities):
        raise ValueError("slots and capabilities must have the same length")
    speed = float(center_speed)
    if speed <= 0.0:
        raise ValueError("center_speed must be > 0")
    radius = (float(center_radius) if center_radius and center_radius > 0.0
              else _default_radius(slots, turn_margin))
    if radius <= 0.0:
        raise ValueError("center_radius must be > 0")
    # A member's local speed can be smaller than the center speed on the
    # inside of a turn. Keep a small default margin above the platform minimum
    # so the planner does not converge to a mathematically marginal solution.
    last_route = None
    last_report = None
    attempts = 0
    for _ in range(max_attempts):
        attempts += 1
        # The candidate geometry is independent of the desired progress rate.
        # A unit center speed makes the route's member scale equal to the
        # derivative with respect to the public progress variable.
        route = IccRoute(waypoints, radius, 1.0, trans_len=trans_len)
        report = check_public_progress_feasibility(
            route, slots, capabilities, sample_step=sample_step)
        last_route, last_report = route, report
        if report.feasible:
            def selected_progress(s):
                slices = report.slices
                if not slices:
                    return speed
                if s <= slices[0].s:
                    current = slices[0]
                elif s >= slices[-1].s:
                    current = slices[-1]
                else:
                    idx = min(range(len(slices)),
                              key=lambda j: abs(slices[j].s - s))
                    current = slices[idx]
                return min(current.upper, max(current.lower, speed))

            route.set_speed_profile(selected_progress)
            # Verify the selected profile in physical units as a final guard.
            physical = check_formation_feasibility(
                route, slots, capabilities, sample_step=sample_step)
            if not physical.feasible:
                next_radius = radius * 1.25
                if next_radius <= radius + 1e-6 or next_radius > max_radius:
                    break
                radius = next_radius
                continue
            selected = [selected_progress(x.s) for x in report.slices]
            return RoutePlan(route, speed, radius, physical, attempts,
                             min(selected), max(selected), report)

        next_radius = radius * 1.25
        if next_radius <= radius + 1e-6 or next_radius > max_radius:
            break
        radius = next_radius

    if last_route is None or last_report is None:
        raise ValueError("could not build an ICC route")
    reason = ("empty public-progress intersection at s=%.3f"
              % last_report.first_infeasible_s
              if last_report.first_infeasible_s is not None
              else "unknown")
    raise ValueError(
        "no feasible route after %d attempts (speed=%.2f, radius=%.2f): %s"
        % (attempts, speed, radius, reason))
