#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""icc_traj.py —— 瞬时曲率中心(ICC)同心圆弧参考轨迹生成器(纯函数, 无 ROS)。

背景(操作说明书 V1.0 第一阶段: 防内侧倒飞):
  刚体编队 NMPC 大角度转弯时, 内侧机"以编队中心为基准平移复制路径点"得到的
  偏移曲线半径 Rcenter + di 变负(di 为负的内侧偏移) → 内侧参考点沿自身路径
  倒退, 期望速度矢量为负 = "倒飞"。根治办法:

  1. 以编队中心为虚拟领航者(P0, 航向 ψ, 转弯角速度 ω), 设中心转弯半径 Rcenter。
  2. 每架机期望半径 Ri = Rcenter + di(di 侧向偏移, 内侧为负、外侧为正)。
  3. 参数化轨迹: 直线段 + 圆弧段, 用【五次多项式】过渡连接, 保证位置/速度/
     加速度(C2)连续。
  4. 解析速度 Vdes_i = ω·Ri = V·(1 - ry·κ), 严禁离散点差分求速度。

符号约定(与 geometry.make_slots 一致):
  槽位 (rx, ry): rx=前向偏移, ry=左法向偏移(ry>0 在机身左侧)。
  中心轨迹弧长 s 处: 切向 t=(cosψ,sinψ), 左法向 n=(-sinψ,cosψ), 有向曲率 κ。
  无人机 i 参考位置 = 中心(s) + rx·t + ry·n。
  无人机 i 期望速度 = V·(1 - ry·κ)  (= ω·(Rcenter - ry), 即 ω·Ri)。

  五次多项式过渡: 过渡段内曲率 κ 按 quintic smoothstep 从 0→κ_arc(入弧)或
  κ_arc→0(出弧), 两端曲率 0/1/2 阶导均为 0 → 位置(C0)、速度/切向(C1)、
  加速度/曲率(C2)全部连续; 过渡段航向扫过 Δψ = L_trans·κ_arc/2。

用法(单测见 tests/test_icc.py):
  from uav_formation.icc_traj import IccRoute
  r = IccRoute([(0,0),(35,0),(35,35)], r_center=15.0, v_cruise=1.5)
  x, y, psi, kappa = r.eval_center(10.0)
  px, py, ppsi, vdes = r.eval_drone(10.0, rx=0.0, ry=-10.0)
"""
import bisect
import math


def quintic_smoothstep(u):
    """五次多项式光滑阶跃 0→1: 两端 0/1/2 阶导均为 0(曲率 C2 过渡)。"""
    u = max(0.0, min(1.0, float(u)))
    return 6.0 * u ** 5 - 15.0 * u ** 4 + 10.0 * u ** 3


def adaptive_turn_radius(half_width, v_cruise, a_n_max=5.0, turn_margin=5.0,
                         manual=0.0, dv_max=3.5):
    """中心转弯半径自适应(纯函数, 无 ROS 依赖, 便于单测)。

    ★ 2026-09-14 设计变更: 半径不再随速度放大(禁止"加大半径过弯") ——
      半径只由几何决定, 物理/差速约束全部转为弯道降速(见 egompc 的 f_corner):
        R_geo = half_width + turn_margin    # 几何下限: 内侧机不倒飞(R > 半宽)
        v_corner = v_cruise·f_corner        # f = min(√(a_n·R)/v, dv_max·R/(hw·v))
    a_n_max/dv_max/v_cruise 参数保留(向后兼容)但不再推大半径。
    返回 max(R_geo, manual)
    """
    hw = max(0.0, float(half_width))     # 半宽恒≥0(槽位 |ry| 绝对值); 非法间距兜底
    R_geo = hw + max(0.0, float(turn_margin))
    R_man = float(manual) if (manual and float(manual) > 0) else 0.0
    return max(R_geo, R_man)


def corner_factor_by_spread(r_center, half_width, v_cruise, dv_max=3.5):
    """§8.23 弯道速度系数(队形无关, 物理式, 替代固定系数表):
    弯道角速度 ω = v_corner/r; 队形内外速度差 = hw×ω ≤ dv_max
    → v_corner ≤ dv_max×r/hw → 系数 = min(1, dv_max×r/(hw×v))。
    hw=0(纵队等) → 1.0(零差速, 不需减速)。"""
    hw = max(0.0, float(half_width))
    v = float(v_cruise)
    r = float(r_center)
    if hw <= 0 or v <= 0 or r <= 0:
        return 1.0
    return min(1.0, float(dv_max) * r / (hw * v))


# 转弯角(航向变化量, 度) → 巡航速度保留系数 的锚点(方向已修正: 转角越大越慢)
#   0°(直线) → 1.0; 45° → 0.85; 90° → 0.7; 135° → 0.5(封底)
CORNER_SPEED_PTS = ((0.0, 1.0), (45.0, 0.85), (90.0, 0.7), (135.0, 0.5))


def corner_speed_factor(theta_deg, pts=CORNER_SPEED_PTS):
    """转弯角 → 速度保留系数(线性插值, 纯函数)。

    目的: 转弯段降速, 使外侧机向心需求 v_outer²/ρ ≤ a_max(高速转弯追不上的根因)。
    方向修正: 航向变化越大(弯越急)降速越多 —— 45°→0.7 / 90°→0.5 / 135°→0.3;
    直线(≤0°)→1.0 全速; ≥135° 封底 0.3。0~45 之间也随角度渐缓(如 20°→≈0.87)。
    """
    th = float(theta_deg)
    pts = sorted(pts)
    if not pts or th <= pts[0][0]:
        return pts[0][1] if pts else 1.0
    if th >= pts[-1][0]:
        return pts[-1][1]
    for i in range(len(pts) - 1):
        a0, f0 = pts[i]
        a1, f1 = pts[i + 1]
        if a0 <= th <= a1 and a1 > a0:
            return f0 + (f1 - f0) * (th - a0) / (a1 - a0)
    return 1.0


class IccRoute:
    """ICC 同心圆弧参考轨迹: 直线 + 圆弧 + 五次多项式过渡, 解析速度。

    构造时按弧长 s 预积分出稠密查表 (s, x, y, ψ, κ);
    eval_center / eval_drone 用线性插值取用, O(log n)。
    """

    def __init__(self, waypoints, r_center, v_cruise, trans_len=None, step=0.05,
                 corner_slow=None, strict_geometry=False):
        """corner_slow: None=全程恒定巡航速度; 可调用对象 f(θ_deg)→速度保留系数
        (如 corner_speed_factor) —— 直线段全速, 圆弧/过渡段按转角降速(速度剖面)。"""
        raw = [(float(p[0]), float(p[1])) for p in waypoints]
        if not all(math.isfinite(v) for p in raw for v in p):
            raise ValueError('nonfinite waypoint')
        self.strict_geometry = bool(strict_geometry)
        # ★ 相邻重合点去重(实测: FC 任务 seq0=home 常与首个上传点重合,
        #   如 [(4.4,6.1),(4.4,6.1),...], 不去重会触发"相邻航点重合"异常)
        pts = []
        for p in raw:
            if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-6:
                pts.append(p)
        if len(pts) < 2:
            raise ValueError("icc_traj: 去重后不足 2 个航点")
        self.pts = pts
        self.r_center = float(r_center)
        if self.r_center <= 0:
            raise ValueError("icc_traj: r_center 必须 > 0")
        self.v = float(v_cruise)
        if self.v <= 0:
            raise ValueError("icc_traj: v_cruise 必须 > 0")
        self.trans_len = float(trans_len) if trans_len is not None else 0.25 * self.r_center
        if self.trans_len < 0:
            raise ValueError("icc_traj: trans_len 必须 >= 0")
        self.step = float(step)
        if not all(math.isfinite(v) for v in (self.r_center, self.v, self.trans_len, self.step)) or self.step <= 0:
            raise ValueError('invalid route parameters')
        self.corner_slow = corner_slow
        self._build()

    # ---------------- 构造 ----------------
    def _build(self):
        pts = self.pts
        m = len(pts)
        leg_u, leg_len, leg_psi = [], [], []
        for i in range(m - 1):
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
            L = math.hypot(dx, dy)
            if L < 1e-9:
                raise ValueError("icc_traj: 相邻航点重合")
            leg_u.append((dx / L, dy / L))
            leg_len.append(L)
            leg_psi.append(math.atan2(dy, dx))

        # 每个内部顶点的拐角参数
        corner = [None] * m
        for c in range(1, m - 1):
            u0x, u0y = leg_u[c - 1]
            u1x, u1y = leg_u[c]
            cross = u0x * u1y - u0y * u1x
            dot = u0x * u1x + u0y * u1y
            theta = math.atan2(cross, max(-1.0, min(1.0, dot)))
            if self.strict_geometry and abs(theta) > math.pi - 1e-5:
                raise ValueError('GEOMETRY: reversal needs an explicit intermediate waypoint')
            if abs(theta) < (1e-10 if self.strict_geometry else math.radians(1.0)):
                continue                       # 近直线, 不设拐角
            side = 1.0 if theta > 0 else -1.0
            kappa_arc = side / self.r_center
            # 过渡段弧长: 保证圆弧段转角 θ_arc = θ - 2·Δψ 仍为正
            trans = min(self.trans_len,
                        (abs(theta) - math.radians(2.0)) * self.r_center)
            trans = max(0.0, trans)
            if self.strict_geometry:
                trans = self.trans_len
                if abs(theta) * self.r_center - trans < -1e-9:
                    raise ValueError('GEOMETRY: negative circular core length')
                if trans <= 0:
                    raise ValueError('GEOMETRY: dynamic mode needs positive curvature transition')
            theta_arc = theta - side * (trans / self.r_center)
            L_arc = abs(theta_arc) * self.r_center
            d = self._corner_pullback(theta, trans, kappa_arc)
            # ★ 转弯段降速: 该拐角的当地巡航速度 = v·f(转角)(corner_slow 未开则恒 v)
            v_corner = (self.v * self.corner_slow(math.degrees(abs(theta)))
                        if self.corner_slow is not None else self.v)
            corner[c] = dict(theta=theta, side=side, kappa_arc=kappa_arc,
                             trans=trans, theta_arc=theta_arc, L_arc=L_arc,
                             pullback=d, v_corner=v_corner)

        # 各直腿可用长度(减去两端拐角的切向缩进量)
        pull = [0.0] * m
        for c in range(1, m - 1):
            if corner[c] is not None:
                pull[c] = corner[c]['pullback']
        straight = []
        for i in range(m - 1):
            sl = leg_len[i] - pull[i] - pull[i + 1]
            if self.strict_geometry and sl < -1e-7:
                raise ValueError('GEOMETRY: turn pullbacks overlap on leg %d' % i)
            straight.append(max(0.0, sl))

        # 组装段表: (s0, s1, k0, k1, v0, v1) —— κ(s) 与 v(s) 均按
        # quintic smoothstep 在段内过渡(入弧: 全速→弯速; 出弧: 弯速→全速)
        segs = []
        s = 0.0

        def add(L, k0, k1, v0, v1):
            nonlocal s
            if L > 1e-9:
                segs.append((s, s + L, k0, k1, v0, v1))
                s += L

        vv = self.v
        add(straight[0], 0.0, 0.0, vv, vv)
        for c in range(1, m - 1):
            if corner[c] is None:
                add(straight[c], 0.0, 0.0, vv, vv)
                continue
            ck = corner[c]
            vc = ck['v_corner']
            add(ck['trans'], 0.0, ck['kappa_arc'], vv, vc)       # 入弧过渡: 减速
            add(ck['L_arc'], ck['kappa_arc'], ck['kappa_arc'], vc, vc)  # 圆弧: 恒弯速
            add(ck['trans'], ck['kappa_arc'], 0.0, vc, vv)       # 出弧过渡: 加速
            add(straight[c], 0.0, 0.0, vv, vv)

        self._segs = segs
        self._seg_ends = [seg[1] for seg in segs]
        self._integrate(leg_psi[0])

    def _trace_end(self, segs, x0, y0, psi0):
        """数值积分一段段表, 返回终点 (x, y, ψ)(中点法)。"""
        x, y, psi = x0, y0, psi0
        for _s0, _s1, k0, k1 in segs:
            L = _s1 - _s0
            n = max(1, int(round(L / self.step)))
            h = L / n
            for j in range(n):
                k_mid = k0 + (k1 - k0) * quintic_smoothstep((j + 0.5) / n)
                psi_half = psi + k_mid * h / 2.0
                x += math.cos(psi_half) * h
                y += math.sin(psi_half) * h
                psi += k_mid * h
        return x, y, psi

    def _corner_pullback(self, theta, trans, kappa_arc):
        """数值求拐角切向缩进量 d: 入腿切点 → 顶点 的距离(出腿对称相等)。

        规范拐角: 入腿切点 T_in=(0,0), 航向 0, 转 theta;
        顶点 B = 入腿切线(y=0) 与 出腿切线 的交点, d = |B - T_in|。
        """
        side = 1.0 if theta > 0 else -1.0
        L_arc = abs(theta - side * (trans / self.r_center)) * self.r_center
        segs = [(0.0, trans, 0.0, kappa_arc),
                (trans, trans + L_arc, kappa_arc, kappa_arc),
                (trans + L_arc, trans + L_arc + trans, kappa_arc, 0.0)]
        ex, ey, _ = self._trace_end(segs, 0.0, 0.0, 0.0)
        sn = math.sin(theta)
        if abs(sn) < 1e-9:
            return max(0.0, abs(ex))
        bx = ex - ey / math.tan(theta)
        return max(0.0, bx)

    def _integrate(self, psi0):
        """沿段表按步长积分, 生成稠密查表 (s, x, y, ψ, κ, v)(κ/v 按解析公式)。"""
        x, y, psi = self.pts[0][0], self.pts[0][1], psi0
        s = 0.0
        S = [0.0]; X = [x]; Y = [y]; P = [psi]
        K = [self._segs[0][2] if self._segs else 0.0]
        V = [self._segs[0][4] if self._segs else self.v]
        for _s0, _s1, k0, k1, v0, v1 in self._segs:
            L = _s1 - _s0
            n = max(1, int(round(L / self.step)))
            h = L / n
            for j in range(n):
                k_mid = k0 + (k1 - k0) * quintic_smoothstep((j + 0.5) / n)
                psi_half = psi + k_mid * h / 2.0
                x += math.cos(psi_half) * h
                y += math.sin(psi_half) * h
                psi += k_mid * h
                s += h
                S.append(s); X.append(x); Y.append(y); P.append(psi)
                K.append(k0 + (k1 - k0) * quintic_smoothstep((j + 1) / n))
                V.append(v0 + (v1 - v0) * quintic_smoothstep((j + 1) / n))
        self._S = S; self._X = X; self._Y = Y; self._P = P; self._K = K
        self._V = V
        self._s_end = s

    # ---------------- 查询 ----------------
    def total_length(self):
        return self._s_end

    def initial_heading(self):
        """起点航向(用于编队成形布局朝向第一段)。"""
        return self._P[0]

    def curvature_at(self, s):
        seg = self._segment_at(s)
        a, b, k0, k1 = seg[:4]
        return k0 + (k1 - k0) * quintic_smoothstep((s - a) / (b - a))

    def _segment_at(self, s):
        if not math.isfinite(s):
            raise ValueError('nonfinite arc length')
        i = min(bisect.bisect_right(self._seg_ends, s), len(self._segs) - 1)
        return self._segs[i]

    def curvature_derivative_at(self, s):
        a, b, k0, k1 = self._segment_at(s)[:4]
        u = min(1.0, max(0.0, (s - a) / (b - a)))
        return (k1 - k0) * 30 * u * u * (1 - u) ** 2 / (b - a)

    def sample(self, ds):
        """Regular grid plus segment boundaries; never skip a short transition."""
        import numpy as np
        if not math.isfinite(ds) or ds <= 0:
            raise ValueError('ds must be positive and finite')
        ss = np.unique(np.concatenate((np.arange(0, self.total_length(), ds),
                                       [0, self.total_length()], self._seg_ends)))
        ss = ss[np.r_[True, np.diff(ss) > 1e-8]]
        ss[-1] = self.total_length()
        states = np.array([self.eval_center(s) for s in ss])
        return dict(s=ss, x=states[:, 0], y=states[:, 1], psi=states[:, 2],
                    kappa=np.array([self.curvature_at(s) for s in ss]),
                    kappa_prime=np.array([self.curvature_derivative_at(s) for s in ss]))

    def _interp(self, s):
        s = min(max(0.0, float(s)), self._s_end)
        i = bisect.bisect_right(self._S, s) - 1
        i = max(0, min(i, len(self._S) - 2))
        s0, s1 = self._S[i], self._S[i + 1]
        t = (s - s0) / (s1 - s0) if s1 > s0 else 0.0
        x = self._X[i] + t * (self._X[i + 1] - self._X[i])
        y = self._Y[i] + t * (self._Y[i + 1] - self._Y[i])
        psi = self._P[i] + t * (self._P[i + 1] - self._P[i])
        kappa = self._K[i] + t * (self._K[i + 1] - self._K[i])
        if self.strict_geometry:
            kappa = self.curvature_at(s)
        v = self._V[i] + t * (self._V[i + 1] - self._V[i])
        return x, y, psi, kappa, v

    def eval_center(self, s):
        """中心轨迹弧长 s 处 → (x, y, ψ, κ)。"""
        return self._interp(s)[:4]

    def eval_center_state(self, s):
        """中心轨迹弧长 s 处的完整参考状态。

        返回 (x, y, psi, kappa, speed)。保留 eval_center 的四元组接口，
        供平台约束层读取中心局部速度而不依赖内部查表实现。
        """
        return self._interp(s)

    def speed_at(self, s):
        """弧长 s 处当地巡航速度 m/s(直线全速; 圆弧段按转角降速; 供弧长推进/时域)。
        corner_slow=None 时恒等于 self.v(与旧版一致)。"""
        return self._interp(s)[4]

    def set_speed_profile(self, speed_fn):
        """Replace the center progress-speed profile without changing geometry.

        ``speed_fn`` receives center arc length and returns the selected public
        progress rate. Geometry and the ICC slot mapping remain unchanged.
        """
        self._V = [max(0.0, float(speed_fn(s))) for s in self._S]

    def eval_drone(self, s, rx, ry):
        """无人机 i 弧长 s 处 → (x, y, ψ, v_des)。

        rx=前向槽位偏移, ry=左法向槽位偏移(geometry.make_slots 的 (x,y))。
        位置 = 中心 + rx·t + ry·n; 期望速度 v_des = v(s)·(1 - ry·κ) = ω·Ri,
        v(s) 为当地巡航速度(转弯段已按转角降速)。
        """
        x, y, psi, kappa, v = self._interp(s)
        c, sn = math.cos(psi), math.sin(psi)
        px = x + rx * c - ry * sn
        py = y + rx * sn + ry * c
        v_des = v * (1.0 - ry * kappa)
        if v_des < 0.0:
            v_des = 0.0
        return px, py, psi, v_des


def parse_route(s):
    """解析 'x,y x,y ...'(或 'x,y,z') 航线字符串 → [(x,y), ...]。"""
    pts = []
    for tok in (s or '').split():
        parts = tok.split(',')
        if len(parts) < 2:
            continue
        try:
            pts.append((float(parts[0]), float(parts[1])))
        except ValueError:
            continue
    return pts


def build_route(pts, r_center, v_cruise, trans_len=None, corner_slow=None):
    """便捷构造: pts 可为 [(x,y,z)|(x,y)], z 忽略(ICC 为平面轨迹)。"""
    xy = [(float(p[0]), float(p[1])) for p in pts]
    return IccRoute(xy, r_center, v_cruise, trans_len, corner_slow=corner_slow)
