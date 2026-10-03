"""Bounded candidate search; never equate solver rejection with impossibility."""
from dataclasses import dataclass, field
import math
import time
import numpy as np

from .capability_model import slot_scale
from .icc_traj import IccRoute
from .progress_profile import PlanningError, plan_progress


@dataclass
class CandidateRecord:
    radius: float
    status: str
    detail: object = None
    duration: float | None = None
    margins: dict = field(default_factory=dict)
    compute_seconds: float = 0.


@dataclass
class CoordinationResult:
    route: object = None
    profile: object = None
    selected_radius: float | None = None
    candidates: list = field(default_factory=list)

    @property
    def feasible(self):
        return self.route is not None and self.profile is not None


class ReferenceCoordinator:
    def __init__(self, members, radii, nominal_radius, nominal_speed, ds=.5,
                 trans_len=None, bounds=None, start_speed=None, end_speed=None,
                 lambda_time=0., smooth_weight=.05, objective_method='quadratic'):
        self.members = tuple(members)
        if not self.members or len({m.member_id for m in members}) != len(members):
            raise ValueError('nonempty unique members required')
        if not radii or any(not math.isfinite(r) or r <= 0 for r in radii):
            raise ValueError('explicit positive finite radius candidates required')
        if not math.isfinite(nominal_radius) or nominal_radius <= 0 or lambda_time < 0:
            raise ValueError('invalid nominal radius or time weight')
        self.radii = sorted(set(float(r) for r in radii), key=lambda r: (abs(r-nominal_radius), r))
        self.nominal_radius, self.nominal_speed = nominal_radius, nominal_speed
        self.ds, self.trans_len, self.bounds = ds, trans_len, bounds
        if bounds is not None and (len(bounds) != 4 or not all(math.isfinite(v) for v in bounds)
                                   or bounds[0] >= bounds[2] or bounds[1] >= bounds[3]):
            raise ValueError('bounds must be [xmin,ymin,xmax,ymax]')
        self.start_speed, self.end_speed = start_speed, end_speed
        self.lambda_time, self.smooth_weight = lambda_time, smooth_weight
        self.objective_method = objective_method

    def plan(self, waypoints):
        result = CoordinationResult()
        best = math.inf
        for radius in self.radii:
            began=time.monotonic()
            try:
                route = IccRoute(waypoints, radius, 1., trans_len=self.trans_len, strict_geometry=True)
                for k in route._K:
                    for member in self.members:
                        slot_scale(member.r_y, k)
                if self.bounds is not None:
                    xmin, ymin, xmax, ymax = self.bounds
                    for s in route._S:
                        x, y, psi, _ = route.eval_center(s)
                        for offset in [0.] + [m.r_y for m in self.members]:
                            px, py = x-offset*math.sin(psi), y+offset*math.cos(psi)
                            if not (xmin <= px <= xmax and ymin <= py <= ymax):
                                raise PlanningError('SPACE_BOUNDARY', dict(s=s, x=px, y=py))
                profile = plan_progress(route, self.members, self.nominal_speed, self.ds,
                                        self.start_speed, self.end_speed, self.smooth_weight,
                                        objective_method=self.objective_method)
                duration = float(profile.time[-1])
                result.candidates.append(CandidateRecord(radius, 'FEASIBLE', duration=duration, margins=profile.margins))
                score = abs(radius-self.nominal_radius) + self.lambda_time*duration
                if score < best:
                    best = score
                    result.route, result.profile, result.selected_radius = route, profile, radius
            except PlanningError as exc:
                result.candidates.append(CandidateRecord(radius, exc.code, exc.detail))
            except ValueError as exc:
                result.candidates.append(CandidateRecord(radius, 'GEOMETRY_REJECTED', str(exc)))
            result.candidates[-1].compute_seconds=time.monotonic()-began
        return result
