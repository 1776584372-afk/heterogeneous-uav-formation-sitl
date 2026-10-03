"""Whole-route planning in z=nu**2 with sparse linear constraints and a QP.

z is piecewise linear; nu and time are continuous, acceleration may jump at
nodes. Constraints use subinterval collocation and an independent denser audit.
This is a discretized feasibility test, not a continuous-time proof.
"""
from dataclasses import dataclass
import math
import numpy as np

from .capability_model import G, Limit, dynamic_margin, slot_scale, team_interval


class PlanningError(ValueError):
    def __init__(self, code, detail):
        self.code, self.detail = code, detail
        super().__init__('%s: %s' % (code, detail))


@dataclass
class ProgressProfile:
    s: np.ndarray
    z: np.ndarray
    team_lower: np.ndarray
    team_upper: np.ndarray
    active_lower: np.ndarray
    active_upper: np.ndarray
    margins: dict

    def __post_init__(self):
        self.nu = np.sqrt(np.maximum(self.z, 0))
        sums = self.nu[:-1] + self.nu[1:]
        if np.any(sums <= 0):
            raise PlanningError('STALLED_PROFILE', 'zero progress on a finite interval')
        self.time = np.r_[0., np.cumsum(2 * np.diff(self.s) / sums)]

    def nu_at(self, s):
        return math.sqrt(max(0., float(np.interp(s, self.s, self.z))))

    def slope_at(self, s):
        i = min(max(0, int(np.searchsorted(self.s, s, side='right') - 1)), len(self.s) - 2)
        return float((self.z[i+1] - self.z[i]) / (self.s[i+1] - self.s[i]))

    def time_at(self, s):
        s = float(np.clip(s, self.s[0], self.s[-1]))
        i = min(max(0, int(np.searchsorted(self.s, s, side='right') - 1)), len(self.s)-2)
        distance = s - self.s[i]
        if distance <= 0:
            return float(self.time[i])
        return float(self.time[i] + 2 * distance / (self.nu[i] + self.nu_at(s)))

    def advance(self, s, dt):
        """Exact time integration of piecewise linear z, including zero-speed start."""
        if dt < 0 or not math.isfinite(dt):
            raise ValueError('invalid dt')
        t = min(self.time[-1], self.time_at(s) + dt)
        i = min(max(0, int(np.searchsorted(self.time, t, side='right') - 1)), len(self.s)-2)
        elapsed = t - self.time[i]
        slope = (self.z[i+1] - self.z[i]) / (self.s[i+1] - self.s[i])
        return float(np.clip(self.s[i] + self.nu[i]*elapsed + .25*slope*elapsed**2,
                             self.s[i], self.s[i+1]))


def validate_profile(route, members, profile, subdivisions=12, tolerance=2e-6):
    """Audit both one-sided slopes at every node and between planning nodes."""
    minima = {m.member_id: {} for m in members}
    for j, h in enumerate(np.diff(profile.s)):
        slope = (profile.z[j+1] - profile.z[j]) / h
        for fraction in np.linspace(0, 1, subdivisions + 1):
            s = profile.s[j] + h * fraction
            z = max(0., profile.z[j]*(1-fraction) + profile.z[j+1]*fraction)
            k, kp = route.curvature_at(s), route.curvature_derivative_at(s)
            for m in members:
                for limit, margin in dynamic_margin(m, k, kp, z, slope).items():
                    minima[m.member_id][int(limit)] = min(minima[m.member_id].get(int(limit), math.inf), margin)
                    if margin < -tolerance:
                        raise PlanningError('AUDIT_REJECTED', dict(s=float(s), member=m.member_id,
                                            limit=int(limit), margin=float(margin)))
    return minima


def plan_progress(route, members, nominal_speed, ds=.5, start_speed=None,
                  end_speed=None, smooth_weight=.05, max_nodes=12000, objective_method='quadratic'):
    if not math.isfinite(nominal_speed) or nominal_speed <= 0 or smooth_weight < 0:
        raise ValueError('invalid nominal speed or smoothness weight')
    # Fail loudly on the VM's incompatible user-site NumPy/SciPy combination.
    from scipy.optimize import linprog
    from scipy.sparse import coo_matrix
    if objective_method not in ('quadratic', 'l1_legacy'):
        raise ValueError('unknown objective_method')

    data = route.sample(ds)
    ss = data['s']
    n = len(ss)
    if n > max_nodes:
        raise PlanningError('RESOURCE_LIMIT', '%d nodes > %d' % (n, max_nodes))
    intervals = [team_interval(members, k) for k in data['kappa']]
    lo = np.array([v[0] for v in intervals])
    hi = np.array([v[1] for v in intervals])
    for j in np.flatnonzero(lo > hi):
        raise PlanningError('STATIC_CONFLICT', dict(s=float(ss[j]), lower=float(lo[j]), upper=float(hi[j]),
                            lower_owner=intervals[j][2], upper_owner=intervals[j][3]))
    lowz, highz = lo**2, hi**2
    # Prevent finite zero-speed plateaux, but permit rest at either endpoint.
    lowz[1:-1] = np.maximum(lowz[1:-1], 1e-8)
    for index, value in ((0, start_speed), (-1, end_speed)):
        if value is not None:
            if not math.isfinite(value) or value < lo[index]-1e-10 or value > hi[index]+1e-10:
                raise PlanningError('ENDPOINT_CONFLICT', dict(index=index, speed=value))
            lowz[index] = highz[index] = value**2
    rows, cols, vals, rhs = [], [], [], []

    def inequality(coeffs, upper):
        row = len(rhs)
        for index, coeff in coeffs.items():
            rows.append(row); cols.append(index); vals.append(coeff)
        rhs.append(upper)

    # The sparse LP supplies a feasible warm start (and the legacy L1 objective).
    for j in range(n):
        inequality({j: 1, n+j: -1}, nominal_speed**2)
        inequality({j: -1, n+j: -1}, -nominal_speed**2)
    for j, h in enumerate(np.diff(ss)):
        inequality({j: -1, j+1: 1, 2*n+j: -1}, 0)
        inequality({j: 1, j+1: -1, 2*n+j: -1}, 0)
        for f in np.linspace(0, 1, 5):
            s = ss[j] + f*h
            k, kp = route.curvature_at(s), route.curvature_derivative_at(s)
            lower, upper, low_owner, high_owner = team_interval(members, k)
            if lower > upper:
                raise PlanningError('STATIC_CONFLICT', dict(s=float(s), lower_owner=low_owner, upper_owner=high_owner))
            inequality({j: 1-f, j+1: f}, upper**2)
            inequality({j: -(1-f), j+1: -f}, -lower**2)
            for m in members:
                q = slot_scale(m.r_y, k)
                a = -.5*q/h - m.r_y*kp*(1-f)
                b = .5*q/h - m.r_y*kp*f
                inequality({j: a, j+1: b}, m.a_t_max * .999)
                inequality({j: -a, j+1: -b}, -m.a_t_min * .999)
                if m.bank_rate_max is not None:
                    # |phi_dot| <= sqrt(z_cap)*|d(q*k*z/g)/ds|.
                    # Dropping 1+b^2 is conservative, keeping constraints linear.
                    cap = max(hi[j], hi[j+1], upper)
                    a = ((1-2*m.r_y*k)*kp*(1-f) - q*k/h)/G
                    b = ((1-2*m.r_y*k)*kp*f + q*k/h)/G
                    limit = m.bank_rate_max * .999 / cap
                    inequality({j: a, j+1: b}, limit)
                    inequality({j: -a, j+1: -b}, limit)
    matrix = coo_matrix((vals, (rows, cols)), shape=(len(rhs), 3*n-1)).tocsr()
    weights = np.r_[np.diff(ss), ss[-1]-ss[-2]]
    objective = np.r_[np.zeros(n), weights, np.full(n-1, smooth_weight)]
    result = linprog(objective, A_ub=matrix, b_ub=rhs,
                     bounds=list(zip(lowz, highz)) + [(0, None)]*(2*n-1),
                     method='highs', options={'time_limit': 60.})
    if not result.success:
        code = 'DYNAMIC_DISCRETE_INFEASIBLE' if result.status == 2 else 'SOLVER_UNRESOLVED'
        raise PlanningError(code, dict(solver='HiGHS', status=int(result.status), message=result.message))
    z = result.x[:n]
    if objective_method == 'quadratic':
        import clarabel
        from scipy.sparse import diags, eye, vstack, triu
        # Paper Eq. (17): squared nominal error plus squared neighboring changes.
        # Remove LP epigraph rows, retaining the same physical linear constraints.
        physical = np.asarray(matrix[:, n:].getnnz(axis=1)).ravel() == 0
        a = vstack([matrix[physical, :n], eye(n)], format='csc')
        lower = np.r_[np.full(int(physical.sum()), -np.inf), lowz]
        upper = np.r_[np.asarray(rhs)[physical], highz]
        difference = diags([-np.ones(n-1), np.ones(n-1)], [0, 1], shape=(n-1, n), format='csc')
        p = (2*eye(n) + 2*smooth_weight*(difference.T@difference)).tocsc()
        # Optimize x=z/z_scale. Normalize each constraint separately so tight
        # acceleration bounds and squared-speed bounds have comparable residuals.
        z_scale = max(1., nominal_speed**2)
        a = a*z_scale
        norms = np.sqrt(np.asarray(a.multiply(a).sum(axis=1)).ravel())
        row_scale = 1./np.where(norms > 1e-12, norms, 1.)
        a = (diags(row_scale)@a).tocsc()
        lower = lower*row_scale
        upper = upper*row_scale
        equality = np.isfinite(lower) & (lower == upper)
        has_upper = np.isfinite(upper) & ~equality
        has_lower = np.isfinite(lower) & ~equality
        ca = vstack([a[equality], a[has_upper], -a[has_lower]], format='csc')
        cb = np.r_[upper[equality], upper[has_upper], -lower[has_lower]]
        cones = [clarabel.ZeroConeT(int(equality.sum())),
                 clarabel.NonnegativeConeT(int(has_upper.sum()+has_lower.sum()))]
        settings = clarabel.DefaultSettings()
        settings.verbose = False
        settings.max_iter = 200
        settings.time_limit = 60.
        settings.tol_gap_abs = settings.tol_gap_rel = settings.tol_feas = 1e-9
        solver = clarabel.DefaultSolver(triu(p).tocsc(), np.full(n, -2*nominal_speed**2/z_scale),
                                        ca, cb, cones, settings)
        quadratic = solver.solve()
        if str(quadratic.status) != 'Solved':
            raise PlanningError('SOLVER_UNRESOLVED', dict(solver='Clarabel', message=str(quadratic.status),
                                primal_residual=quadratic.r_prim, dual_residual=quadratic.r_dual))
        z = np.clip(np.asarray(quadratic.x)*z_scale, lowz, highz)
    profile = ProgressProfile(ss, np.maximum(z, 0), lo, hi,
                              np.array([v[2] for v in intervals], int),
                              np.array([v[3] for v in intervals], int), {})
    profile.margins = validate_profile(route, members, profile)
    return profile
