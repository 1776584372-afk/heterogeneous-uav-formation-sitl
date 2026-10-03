#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""egompc_formation.py —— EGO+NMPC 编队(无领机版, v9.8hou §8.2 + ICC 转弯平滑化)

架构(无领机/虚拟中心):
  编队中心轨迹由【解析 ICC 参考轨迹】(uav_formation.icc_traj.IccRoute)给出:
  直线段 + 圆弧段 + 五次多项式过渡(位置/速度/加速度 C2 连续)。本节点让
  【全部 N 架】各自跑 unicycle NMPC 追踪【每架自己的同心圆弧偏移曲线】+ 解析
  速度 Vdes_i = ω·Ri = V·(1 - ry·κ) —— 内侧机减速、外侧机加速, 大角度转弯
  不再出现内侧机期望速度矢量为负("倒飞")。

  参考轨迹来源(两种):
    · preset 模式: --route "x,y x,y ..." 命令行直传(scripts/run_ego.sh)。
    · mission 模式: 订阅 /formation/route(PoseArray, ego_env_bridge 发布)。
  成形/起飞门控: /formation/start(std_msgs/Bool) 由 ego_env_bridge 发布 ——
    preset 模式授权即发; mission 模式成形稳定后发。收到 start 才推进弧长。

  反馈失效: 冻结公共进度，健康成员保持实测位置；失效成员停发设定点，
  poscmd_bridge 请求 LOITER 并锁存故障，恢复反馈后不能自动恢复飞行。

  输出 /drone_{0..N-1}/position_cmd(quadrotor_msgs/PositionCommand),
  由 poscmd_bridge.py 转成 /uav{i}/mavros/setpoint_position/local。

用法:
  source /opt/ros/humble/setup.bash && source ~/ros2_mavros_ws/install/setup.bash
  source ~/bamboo_square_ws/install/setup.bash      # quadrotor_msgs
  python3 uav_formation/egompc_formation.py --n-drones 3 --formation line --spacing 10 \
      --route "5,0 35,0 45,30 25,50 5,30" --turn-radius 15 --speed 1.5
"""
import argparse
from dataclasses import asdict
import json
import math
import time
from dataclasses import replace

import numpy as np
import rclpy

try:
    from scipy.optimize import minimize as scipy_minimize
except Exception:
    scipy_minimize = None

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32MultiArray, Int32, String
from geometry_msgs.msg import PoseArray

from uav_formation.geometry import make_slots
from uav_formation.icc_traj import (IccRoute, parse_route, adaptive_turn_radius,
                                   corner_factor_by_spread)
from uav_formation.experiment_config import load_config, coordinator_from_config
from uav_formation.capability_model import member_demands, dynamic_margin, normalized_min_margin, team_interval
from uav_formation.measured_feedback import (MeasuredFeedback, WorldTransform, FeedbackError,
                                            quaternion_yaw, wrap_angle, command_endpoint)

# ---------- NMPC 参数(与 bamboo nmpc_node.py 原版一致, scipy 重写) ----------
DT = 0.05
N = 12
V_DES = 1.5
V_MAX, W_MAX = 30.0, 1.2   # ★ 单机速度硬上界设计值 30(可 --v-max 覆盖); 角速度上限不变
V_MIN = 0.0            # ★ 飞行阶段 v 下界(防倒飞硬兜底); 成形阶段用 -v_max(可后退收敛)
W_LAT, W_HEAD = 10.0, 7.0
W_V = 2.0
W_U = 0.02
RATE = 1.0 / 3.0        # 3Hz 控制周期(原版)
PSI_RATE = math.radians(60.0)   # ★ 无 route 成形期航向旋转限速 60°/s(§8.5)
FLY_Z = 2.0             # 默认飞行高度 m(2026-09-04 对齐 v13: 原 10 易卡门控; 低空起飞快,
                        #   真正常飞高度由航点 z/--fly-z 覆盖)
TAU_Z = 0.8


class DroneMPC:
    """单架无人机的 unicycle NMPC(scipy SLSQP 版)。状态 [x,y,ψ], 控制 [v,w]。

    v 下界按阶段传入: 飞行阶段 v_min=0(防倒飞硬兜底, ICC 参考速度恒≥0);
    成形阶段 v_min=-v_max(允许后退 —— 目标在初始航向后方时单向 v 会把
    SLSQP 卡在 v=0 边界, 设定点冻结、成形永不完成, 实测 uav2 卡 17m)。"""

    def __init__(self, idx, v_max=V_MAX, analytic_gradient=False, held_control=False):
        self.idx = idx
        self.analytic_gradient = bool(analytic_gradient)
        self.held_control = bool(held_control)
        self.v_max = float(v_max)    # ★ 单机速度硬上界(默认 30, --v-max 覆盖)
        if not math.isfinite(self.v_max) or self.v_max <= 0:
            raise ValueError('v_max must be finite and positive')
        self.state = None            # [x, y, psi]
        self.z = FLY_Z
        self.u_prev = None           # ★ 上一帧控制解(暖启动种子, 摆动根治②)
        self.fail_count = 0          # ★ 求解失败计数(诊断用)
        self.last_status = 'NOT_SOLVED'
        self.last_detail = ''
        self.last_feedback = None
        self.last_solve_seconds = 0.

    def solve_measured(self, ref, v_des, measured, dt_hold=RATE, v_min=0.):
        """Flight entry: replace X0 with this solve's aligned measurement."""
        self.state = np.asarray(measured.state, dtype=float)
        ref = np.asarray(ref, dtype=float)
        if ref.shape == (3, N+1) and np.all(np.isfinite(ref)) and np.all(np.isfinite(self.state)):
            self.state[2] = float(ref[2, 0])+wrap_angle(self.state[2]-float(ref[2, 0]))
        self.last_feedback = measured
        began = time.monotonic()
        try:
            return self.solve(ref, v_des, dt_hold, v_min)
        finally:
            self.last_solve_seconds = time.monotonic()-began

    def solve(self, ref, v_des, dt_hold=DT, v_min=None):
        """ref: (3, N+1) 参考轨迹 [x,y,psi]。返回 (v0, w0)。
        dt_hold: 本次解的控制保持时长(=tick 重解周期)。
        v_des: 本机解析期望速度 ω·Ri(转弯时内外差速, 非全局标量)。
        v_min: 本拍 v 下界(0=飞行阶段禁倒车; None=成形阶段默认 -self.v_max 可后退收敛)。"""
        if v_min is None:
            v_min = -self.v_max
        ref = np.asarray(ref, dtype=float)
        if (ref.shape != (3, N+1) or not np.all(np.isfinite(ref))
                or self.state is None or np.shape(self.state) != (3,)
                or not np.all(np.isfinite(self.state))):
            self.fail_count += 1
            self.last_status, self.last_detail = 'INVALID_INPUT_STOP', 'invalid reference/state'
            self.u_prev = None
            return 0., 0.
        if not (math.isfinite(v_min) and -self.v_max <= v_min <= 0
                and math.isfinite(dt_hold) and dt_hold > 0):
            raise ValueError('invalid control hold or speed lower bound')
        speeds = np.asarray(v_des, dtype=float)
        if speeds.ndim == 0:
            speeds = np.full(N, float(speeds))
        if speeds.shape != (N,) or not np.all(np.isfinite(speeds)):
            raise ValueError('v_des must be a finite scalar or length-N sequence')
        if scipy_minimize is None:
            self.fail_count += 1
            self.last_status, self.last_detail = 'FALLBACK', 'SciPy unavailable'
            return self._fallback(ref, v_min)
        X0 = np.array(self.state, float)
        n = N
        dt = DT

        def cost(u):
            u = np.asarray(u, float)
            vv, ww = u[:n], u[n:]
            X = np.zeros((3, n + 1))
            X[:, 0] = X0
            for k in range(n):
                X[0, k + 1] = X[0, k] + dt * vv[k] * math.cos(X[2, k])
                X[1, k + 1] = X[1, k] + dt * vv[k] * math.sin(X[2, k])
                X[2, k + 1] = X[2, k] + dt * ww[k]
            c = 0.0
            for k in range(n + 1):
                c += W_LAT * ((X[0, k] - ref[0, k]) ** 2
                              + (X[1, k] - ref[1, k]) ** 2)
                # ★ 朝向项用"包裹角二次"(sin² 在 ψ=0 处梯度为 0, SLSQP 会迷失方向;
                #   包裹角梯度正确 → 向左/右最短转向)
                wrapped = (X[2, k] - ref[2, k] + math.pi) % (2 * math.pi) - math.pi
                c += W_HEAD * wrapped * wrapped
            for k in range(n):
                c += W_U * (vv[k] ** 2 + ww[k] ** 2)
                c += W_V * (vv[k] - speeds[k]) ** 2
            return c

        # ★ 暖启动(摆动根治②): 用上一帧解做种子
        u0 = (self.u_prev if self.u_prev is not None
              else np.concatenate([np.maximum(0., speeds), np.zeros(n)]))
        # 阶段切换时旧种子可能越出新边界(成形 v<0 → 飞行 v≥0), 先裁剪,
        # 避免 SLSQP 行搜索反复越界裁剪(实测 RuntimeWarning + 收敛慢)
        u0 = np.asarray(u0, float).copy()
        u0[:n] = np.clip(u0[:n], v_min, self.v_max)
        u0[n:] = np.clip(u0[n:], -W_MAX, W_MAX)
        # ★ 防倒飞硬兜底: v 下界按阶段传入(0=飞行禁倒车; -v_max=成形可后退),
        #   上界 self.v_max; w ∈ [-W_MAX, W_MAX]
        bounds = [(v_min, self.v_max)] * n + [(-W_MAX, W_MAX)] * n
        try:
            if self.held_control:
                from .planar_objective import held_cost_gradient, expand_held_controls
                block=max(1,min(n,int(round(dt_hold/dt))))
                reduced=np.r_[np.mean(u0[:block]),u0[block:n],np.mean(u0[n:n+block]),u0[n+block:]]
                objective=lambda u: held_cost_gradient(u,X0,ref,speeds,dt,(W_LAT,W_HEAD,W_V,W_U),block)
                reduced_bounds=[(v_min,self.v_max)]*(n-block+1)+[(-W_MAX,W_MAX)]*(n-block+1)
                res=scipy_minimize(objective,reduced,jac=True,method='SLSQP',bounds=reduced_bounds,
                                   options={'maxiter':120,'ftol':1e-4})
                res.x=expand_held_controls(res.x,n,block)
            elif self.analytic_gradient:
                from .planar_objective import cost_gradient
                objective=lambda u: cost_gradient(u,X0,ref,speeds,dt,(W_LAT,W_HEAD,W_V,W_U))
                res = scipy_minimize(objective, u0, jac=True, method='SLSQP', bounds=bounds,
                                     options={'maxiter': 120, 'ftol': 1e-4})
            else:
                res = scipy_minimize(cost, u0, method='SLSQP', bounds=bounds,
                                     options={'maxiter': 120, 'ftol': 1e-4})
            if (not res.success or np.shape(res.x) != (2*n,)
                    or not np.all(np.isfinite(res.x))):
                self.fail_count += 1
                self.last_status, self.last_detail = 'FALLBACK', str(res.message)
                self.u_prev = None
                return self._fallback(ref, v_min)
            res.x[:n] = np.clip(res.x[:n], v_min, self.v_max)
            res.x[n:] = np.clip(res.x[n:], -W_MAX, W_MAX)
            self.u_prev = res.x.copy()
            self.last_status, self.last_detail = 'SOLVED', str(res.message)
            # ★ 摆动根治④: 只应用解的前 k=dt_hold/DT 拍均值(零阶保持)
            k = max(1, min(n, int(round(dt_hold / DT))))
            return (float(np.mean(res.x[:k])), float(np.mean(res.x[n:n + k])))
        except Exception as exc:
            self.fail_count += 1
            self.last_status, self.last_detail = 'FALLBACK', repr(exc)
            self.u_prev = None
            return self._fallback(ref, v_min)

    def _fallback(self, ref, v_min):
        """解析追踪兜底: SLSQP 失败时朝参考点转向 + 按距离比例速度。

        ★ 绝不再返回 (v_des,0) 冻结状态 —— 成形期 v_des=0 时旧回退会输出
        (0,0), 设定点永久冻结(实测: 起飞后动 5m 就停, 成形永不完成)。
        """
        dx = ref[0, 0] - self.state[0]
        dy = ref[1, 0] - self.state[1]
        if not (math.isfinite(dx) and math.isfinite(dy)):
            return 0.0, 0.0
        dist = math.hypot(dx, dy)
        if dist < 0.05:
            return 0.0, 0.0
        target_psi = math.atan2(dy, dx)
        wrap = (target_psi - self.state[2] + math.pi) % (2 * math.pi) - math.pi
        v = dist * 1.5
        v = min(self.v_max, max(v_min, v))
        w = max(-W_MAX, min(W_MAX, wrap * 3.0))
        return v, w


class EgoMpcFormation(Node):
    """解析 ICC 中心轨迹 + unicycle NMPC 编队(无领机: N 架全跟随)。"""

    def __init__(self, n_drones, formation='line', spacing=10.0, fly_z=FLY_Z,
                 route_str='', turn_radius=0.0, speed=V_DES, auto_start=False,
                 v_max=V_MAX, a_n_max=5.0, turn_margin=5.0,
                 trans_len=None, uniform_speed_ref=False,
                 capability_coordination=False, experiment_config=None,
                 home_offsets=None, home_spacing=10., feedback_topic='odom'):
        super().__init__('egompc_formation')
        from quadrotor_msgs.msg import PositionCommand
        self._PositionCommand = PositionCommand
        self.nd = int(n_drones)
        offsets = home_offsets if home_offsets is not None else [(home_spacing*i, 0.) for i in range(self.nd)]
        if len(offsets) != self.nd:
            raise ValueError('one local-to-world transform per member required')
        self.feedback = [MeasuredFeedback(WorldTransform(*offset)) for offset in offsets]
        self.feedback_valid = [False]*self.nd
        self.feedback_events = [self.create_publisher(String, '/drone_%d/feedback_status'%i, 10)
                                for i in range(self.nd)]
        self.last_setpoints = {}
        self.last_feedback_warning = {}
        self.feedback_topic = feedback_topic
        if feedback_topic not in ('odom', 'pose'):
            raise ValueError('feedback topic must be odom or pose')
        from geometry_msgs.msg import PoseStamped
        for i in range(self.nd):
            self.create_subscription(Odometry if feedback_topic=='odom' else PoseStamped,
                '/uav%d/mavros/local_position/%s'%(i, feedback_topic),
                lambda msg, i=i: self._feedback_cb(i, msg), qos_profile_sensor_data)
        self.slots_base = make_slots(formation, self.nd, spacing)
        self.slots = list(self.slots_base)   # 生效槽位(§8.14 指派后重排)
        self.slot_perm = None                # 当前指派(None=未收到, 恒等)
        self.capability_coordination = bool(capability_coordination)
        self.progress_profile = None
        self.coordination_result = None
        self.planning_fault = False
        self.selected_radius = 0.
        if self.capability_coordination:
            if experiment_config is None:
                raise ValueError('capability coordination requires --experiment-config')
            if scipy_minimize is None:
                raise RuntimeError('SciPy unavailable: use scripts/experiment_env.sh')
            self.experiment_cfg, self.members = load_config(experiment_config)
            if len(self.members) != self.nd or any(m.platform != 'multirotor' for m in self.members):
                raise ValueError('EgoMpcFormation is the multirotor execution branch only')
            if self.experiment_cfg['endpoint'] != 'hover':
                raise ValueError('multirotor executor currently requires hover endpoint')
            self.slots_base = [(0., m.r_y, 0.) for m in self.members]
            self.slots = list(self.slots_base)
        # ★ §8.14 成形防对穿: 订阅桥的最近槽位指派 —— 起飞排列与本班航向
        #   不匹配时, 按机号固定槽位会让两机对穿换位(实测)。指派全飞行固定。
        self.create_subscription(
            Int32MultiArray, '/formation/slot_assign', self._slot_assign_cb, 10)
        self.fly_z = float(fly_z)
        self.v_cruise = float(speed)
        # 转弯半径: 0=自动(半宽+5); >0 手动指定(越小拐角内切越小, 但内侧机
        # 半径 R-半宽 也越小, 过小会逼近倒飞边界, 建议 ≥ 半宽+2)
        self.turn_radius = float(turn_radius)
        # ★ 速度上界 + 转弯半径自适应参数(自适应中心转弯半径, 随速度/间距联动)
        self.v_max = float(v_max)
        self.a_n_max = float(a_n_max)
        self.turn_margin = float(turn_margin)
        # ★ 实验消融开关(2026-09-23; 默认值 = 完全不改原行为):
        #   trans_len         五次曲率过渡段长度 m; None=默认 0.25*R_c, 0=取消过渡(B1)
        #   uniform_speed_ref True=成员速度代价参考用统一中心速度(B2), False=槽位差速(P)
        self.trans_len = (None if trans_len is None else float(trans_len))
        self.uniform_speed_ref = bool(uniform_speed_ref)
        # 全部 N 架的 NMPC 输出(drone_0..N-1)
        self.pubs = [self.create_publisher(
            self._PositionCommand, '/drone_%d/position_cmd' % i, 10)
            for i in range(self.nd)]
        # ★ 实验审计话题(2026-09-23, 只发布不参与控制): 记录参考层真值, 供论文算
        #   e_ICC = 实测 vs ICC 原始槽位参考。绝不能用 position_cmd 代替 —— 那是
        #   NMPC 生成的执行设定点，不是原始 ICC 参考。
        self.speed_ref_pubs = [self.create_publisher(
            Float32, '/drone_%d/speed_reference' % i, 10) for i in range(self.nd)]
        self.icc_ref_pubs = [self.create_publisher(
            Float32MultiArray, '/drone_%d/icc_reference' % i, 10)
            for i in range(self.nd)]
        self.progress_pub = self.create_publisher(Float32, '/formation/progress_s', 10)
        self.kappa_pub = self.create_publisher(Float32, '/formation/curvature', 10)
        self.center_speed_pub = self.create_publisher(
            Float32, '/formation/center_speed_reference', 10)
        self.reference_diag = {name: self.create_publisher(Float32, '/formation/'+name, 10)
            for name in ('reference_progress', 'reference_progress_rate', 'reference_progress_accel',
                         'reference_curvature', 'reference_curvature_derivative',
                         'team_feasible_lower', 'team_feasible_upper', 'selected_radius')}
        self.active_diag = {name: self.create_publisher(Int32, '/formation/'+name, 10)
            for name in ('active_lower_member', 'active_upper_member', 'active_lower_limit',
                         'active_upper_limit', 'candidate_status')}
        self.member_diag = [{name: self.create_publisher(Float32, '/drone_%d/%s' % (i,name), 10)
            for name in ('curvature_reference', 'heading_rate_reference', 'feasibility_margin')}
            for i in range(self.nd)]
        self.candidate_report_pub = self.create_publisher(String, '/formation/candidate_report',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # ★ 解析 ICC 参考轨迹(preset: --route; mission: /formation/route)
        self.route = None
        self.route_pts = None           # 最近一次建轨用的航点(速度变化时据此重建轨迹)
        if route_str:
            pts = parse_route(route_str)
            if len(pts) >= 2:
                self.route, _ = self._build_route(pts)
        self.route_active = bool(auto_start) and self.route is not None
        # 成形阶段: 无 route/未激活时虚拟中心 = 真实中心(/formation_center/odom)
        self.center_live = None         # (x, y) 真实中心
        self.create_subscription(
            Odometry, '/formation_center/odom', self.center_odom_cb, 10)
        # 队形航向(无 route 成形期用, 根源修复 z 轴摆动 §8.6)
        self.psi_leg = None
        self.psi_rot = None
        self.create_subscription(
            Float32, '/formation/heading', self.heading_cb, 10)
        # mission 模式: 桥发布整条航点 → 重建 ICC route
        self.create_subscription(
            PoseArray, '/formation/route', self.route_cb, 10)
        # ★ 地面站 DO_CHANGE_SPEED 速度(桥 PullMission 解析后经 /formation/speed 下发)
        self.create_subscription(
            Float32, '/formation/speed', self.speed_cb, 10)
        # 起飞/成形门控: 桥发布 start 才推进弧长
        self.create_subscription(
            Bool, '/formation/start', self.start_cb, 10)
        self.mpc = [DroneMPC(i, self.v_max) for i in range(self.nd)]
        if self.capability_coordination:
            for mpc, member in zip(self.mpc, self.members):
                mpc.v_max = min(self.v_max, member.v_max)
        self.s = 0.0
        self._last_tick = None
        self._last_feedback_time = None
        self.timer = self.create_timer(RATE, self.tick)
        self.get_logger().info(
            'egompc_formation(ICC+NMPC): %d 架全 NMPC, %s 槽位=%s, z=%.1f, '
            'route=%s'
            % (self.nd, formation,
               [(round(s[0], 1), round(s[1], 1)) for s in self.slots], self.fly_z,
               '有(%d点)' % len(self.route.pts) if self.route else '无'))
        # ★ 每个 bag 都要能审计这次到底跑的是哪一组
        self.get_logger().info(
            '★ 实验开关: trans_len=%s, uniform_speed_ref=%s'
            % ('默认(0.25*R_c)' if self.trans_len is None else '%.3fm' % self.trans_len,
               self.uniform_speed_ref))
        self.get_logger().info('measured feedback: topic=%s transforms=%s; max_age=0.5s extrapolation<=0.2s'
                               % (feedback_topic, offsets))

    def _feedback_cb(self, i, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        pose = msg.pose.pose if self.feedback_topic=='odom' else msg.pose
        try:
            q = pose.orientation
            self.feedback[i].push_pose(stamp, pose.position.x, pose.position.y,
                quaternion_yaw(q.x,q.y,q.z,q.w), time.monotonic())
        except FeedbackError as exc:
            self.feedback[i].invalidate(str(exc))

    def _feedback_report(self, i, valid, reason, measured=None, control=None):
        self.feedback_valid[i] = valid
        record = dict(member=i, valid=valid, reason=reason, stamp=self.get_clock().now().nanoseconds*1e-9,
                      measured=asdict(measured) if measured is not None else None,
                      control=control, solver=self.mpc[i].last_status,
                      solver_detail=self.mpc[i].last_detail, failures=self.mpc[i].fail_count,
                      x0=self.mpc[i].state.tolist() if reason=='MEASURED' else None,
                      solve_wall_s=self.mpc[i].last_solve_seconds if reason=='MEASURED' else None)
        msg = String(); msg.data = json.dumps(record, allow_nan=False)
        self.feedback_events[i].publish(msg)
        if not valid or self.mpc[i].last_status in ('FALLBACK', 'INVALID_INPUT_STOP'):
            now = time.monotonic()
            if now-self.last_feedback_warning.get(i, -100) > 2:
                self.get_logger().warning('NMPC feedback '+msg.data)
                self.last_feedback_warning[i] = now

    def heading_cb(self, msg):
        self.psi_leg = float(msg.data)

    def _slot_assign_cb(self, msg):
        perm = list(msg.data)
        if len(perm) != len(self.slots_base) or sorted(perm) != list(range(len(perm))):
            self.get_logger().warn('槽位指派非法: %s, 忽略' % (perm,))
            return
        if perm == self.slot_perm:
            return
        if self.capability_coordination and self.route_active:
            self.get_logger().warn('Research mode rejects slot reassignment during execution')
            return
        self.slot_perm = perm
        self.slots = [self.slots_base[p] for p in perm]
        if self.capability_coordination:
            self.members = tuple(replace(m, r_y=self.slots[i][1]) for i,m in enumerate(self.members))
            if self.route_pts is not None:
                try:
                    self.route, _ = self._build_route(self.route_pts)
                except ValueError as exc:
                    self.get_logger().error(str(exc))
        self.get_logger().info(
            '★ 槽位指派已应用(防对穿): %s -> 槽位 %s'
            % (perm, [(round(s[0], 1), round(s[1], 1)) for s in self.slots]))

    def center_odom_cb(self, msg):
        self.center_live = (float(msg.pose.pose.position.x),
                            float(msg.pose.pose.position.y))

    def _build_route(self, pts):
        """构造 ICC 候选轨迹，求成员公共进度可行域并选择公共进度。

        返回 (route, r_center)。IccRoute 可能抛 ValueError(相邻点重合等)，
        由调用方兜住。同时缓存 self.route_pts 供速度变化时重建轨迹。
        """
        self.route_pts = [tuple(float(v) for v in p) for p in pts]
        if self.capability_coordination:
            self.route = None
            self.progress_profile = None
            self.route_active = False
            self.planning_fault = True
            coordinator = coordinator_from_config(self.experiment_cfg, self.members, self.v_cruise)
            result = coordinator.plan(pts)
            self.coordination_result = result
            self._publish_candidate_report()
            if not result.feasible:
                raise ValueError('No verified candidate; old reference invalidated')
            self.progress_profile = result.profile
            self.selected_radius = result.selected_radius
            self.planning_fault = False
            return result.route, result.selected_radius
        # Compatibility reproduces the pre-coordination radius and slowdown law.
        half_width = max(abs(slot[1]) for slot in self.slots)
        radius = adaptive_turn_radius(half_width, self.v_cruise, self.a_n_max,
                                      self.turn_margin, self.turn_radius, 3.5)
        phys = min(1., math.sqrt(max(0., self.a_n_max)*radius)/self.v_cruise) if self.a_n_max > 0 else 1.
        spread = corner_factor_by_spread(radius, half_width, self.v_cruise, 3.5)
        self.selected_radius = radius
        return IccRoute(pts, radius, self.v_cruise, trans_len=self.trans_len,
                        corner_slow=lambda th: min(phys, spread)), radius

    def _publish_candidate_report(self):
        import json
        from dataclasses import asdict
        if self.coordination_result is not None:
            msg = String()
            msg.data = json.dumps([asdict(c) for c in self.coordination_result.candidates])
            self.candidate_report_pub.publish(msg)

    def speed_cb(self, msg):
        """DO_CHANGE_SPEED 速度(桥经 /formation/speed 下发): 更新巡航速度并重建
        ICC 轨迹(转弯半径随速度联动)。起飞前设定, 重建时弧长归零无跳变。"""
        s = float(msg.data)
        if not math.isfinite(s) or s <= 0 or s == self.v_cruise:
            return
        if self.capability_coordination and self.route_active:
            self.get_logger().warn('Research mode freezes speed during execution')
            return
        self.v_cruise = s
        if self.route_pts is None:
            self.get_logger().info('★ 速度更新 %.2f m/s(航线到达后生效)' % s)
            return
        try:
            self.route, r_center = self._build_route(self.route_pts)
        except ValueError:
            self.get_logger().warn('Speed rebuild failed; coordination mode invalidates the reference')
            return
        self.s = 0.0
        self.get_logger().info(
            '★ 速度更新 %.2f m/s → 重建 ICC 轨迹(转弯半径 %.1fm)' % (s, r_center))

    def route_cb(self, msg):
        """mission 模式: 桥发布的航点 → 重建 ICC route。

        ★ 高度随地面站航点 z(对齐 v13): 航线带有效 z(z>0.1)时, 巡航高度 fly_z
        改为航点最高高度 —— EGO+ICC 链不再只认 --fly-z, 中心线改高度即可生效。"""
        if self.capability_coordination and self.route_active:
            self.get_logger().warn('Research mode freezes route during execution')
            return
        pts = [(float(p.position.x), float(p.position.y))
               for p in msg.poses]
        if len(pts) < 2:
            return
        # 地面站航点高度 → 巡航高度(起飞/爬升由位置设定点带到该高度)
        zs = [float(p.position.z) for p in msg.poses
              if math.isfinite(float(p.position.z)) and float(p.position.z) > 0.1]
        if zs:
            new_z = max(zs)
            if abs(new_z - self.fly_z) > 0.05:
                old = self.fly_z
                self.fly_z = new_z
                self.get_logger().info(
                    '★ 航点高度 %.1f → %.1f m (地面站设定)' % (old, new_z))
        try:
            self.route, r_center = self._build_route(pts)
        except ValueError as e:
            # 相邻点重合等异常: 保留旧 route, 不崩节点(实测曾因此整进程退出)
            self.get_logger().warn('Route rebuild failed: %s' % e)
            return
        self.s = 0.0
        self.route_active = False
        self.get_logger().info(
            '★ 收到 /formation/route(%d 点), 重建 ICC 轨迹(转弯半径 %.1fm)'
            % (len(self.route.pts), r_center))

    def start_cb(self, msg):
        """桥发布起飞/成形完成 → 开始沿 ICC 轨迹推进。"""
        if not bool(msg.data):
            self.route_active = False
        elif self.route is not None and not self.planning_fault:
            self.route_active = True
            self.get_logger().info('★ /formation/start → 开始沿 ICC 轨迹飞行')

    def _make_ref(self, i):
        """drone i 参考: 每机自己的 ICC 偏移曲线(rx·t + ry·n) + 解析 v_des_i。

        返回 (ref, v_des_i)。ref 形状 (3, N+1); v_des_i = V·(1 - ry·κ)。
        """
        rx, ry, _ = self.slots[i]
        route = self.route
        ref = np.zeros((3, N + 1))
        # 飞行阶段: route 激活 → 每机自己的 ICC 偏移曲线 + 解析速度
        if route is not None and self.route_active:
            if self.capability_coordination:
                speeds = np.zeros(N)
                s = self.s
                for k in range(N+1):
                    px, py, psi, _ = route.eval_drone(s, rx, ry)
                    ref[:,k] = (px,py,psi)
                    if k < N:
                        nu = self.progress_profile.nu_at(s)
                        speeds[k] = nu if self.uniform_speed_ref else nu*(1-ry*route.curvature_at(s))
                    s = self.progress_profile.advance(s, DT)
                return ref, speeds
            s0 = self.s
            if s0 >= route.total_length() - 1e-6:
                v_des = 0.0          # 终点悬停(摆动根治①)
            elif self.uniform_speed_ref:
                # ★ B2 组: 取消槽位相关速度参考, 成员统一用中心速度作为 NMPC
                #   速度【代价参考】。位置参考仍各自不同 → 优化器输出速度仍可能
                #   不同, 因此论文表述必须是"统一速度代价参考", 不是"实际同速"。
                v_des = route.speed_at(s0)
            else:
                v_des = route.eval_drone(s0, rx, ry)[3]
            # 时域采样沿弧长按【当地速度】推进(转弯段慢 → 采样间距小, 与实飞一致)
            s = s0
            for k in range(N + 1):
                px, py, ppsi, _ = route.eval_drone(s, rx, ry)
                ref[0, k] = px
                ref[1, k] = py
                ref[2, k] = ppsi
                s += route.speed_at(s) * DT
            return ref, v_des
        # 成形/悬停阶段(无 route 或未激活): 钉在 live center + 队形航向
        # ★ 必须与 ego_env_bridge 的成形判定用同一锚点(form_anchor)与航向
        #   (/formation/heading), 否则成形永远判不过、/formation/start 不发
        cx, cy = self.center_live if self.center_live is not None else (0.0, 0.0)
        psi = self.psi_rot if self.psi_rot is not None else 0.0
        c, sn = math.cos(psi), math.sin(psi)
        for k in range(N + 1):
            ref[0, k] = cx + rx * c - ry * sn
            ref[1, k] = cy + rx * sn + ry * c
            ref[2, k] = psi
        return ref, 0.0

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        wall_dt = (now - self._last_tick) if self._last_tick is not None else RATE
        self._last_tick = now
        # MAVROS timestamps can bunch on reception. Use the latest common
        # measured time, not sample-slope extrapolation to the wall-clock tick.
        stamps = [stream[-1][0] for buffer in self.feedback
                  for stream in (buffer.positions, buffer.headings) if stream]
        target_time = min(stamps, default=now)
        measurements = []
        for i, buffer in enumerate(self.feedback):
            try:
                measurements.append(buffer.at(target_time, now, time.monotonic()))
            except FeedbackError as exc:
                measurements.append(None)
                self._feedback_report(i, False, str(exc))
                self.mpc[i].u_prev = None
        # Keep the common reference stationary when any required feedback is lost.
        # Missing members receive no new moving setpoint; the bridge requests LOITER.
        if any(value is None for value in measurements):
            self._last_feedback_time = None
            for i, measured in enumerate(measurements):
                if measured is not None and i in self.last_setpoints:
                    self._publish(i, measured.state, 0., 0.)
                    self._feedback_report(i, True, 'TEAM_FEEDBACK_HOLD', measured, (0.,0.))
            return
        dt = (target_time-self._last_feedback_time) if self._last_feedback_time is not None else RATE
        if (not math.isfinite(wall_dt) or wall_dt <= 0 or wall_dt > 2.0
                or not math.isfinite(dt) or dt < 0 or dt > 2.0):
            self.planning_fault = True
            self.route_active = False
        if self.planning_fault:
            for i, measured in enumerate(measurements):
                self.mpc[i].u_prev = None
                self._feedback_report(i, False, 'PLANNING_OR_CLOCK_FAULT', measured)
            self._publish_center_diag()
            return
        if dt == 0:
            return  # A duplicate source sample cannot advance progress or refresh X0.
        self._last_feedback_time = target_time

        # 队形航向限速(仅无 route 成形期): 桥发布的航段方位角 → psi_rot
        if self.psi_leg is not None:
            if self.psi_rot is None:
                self.psi_rot = self.psi_leg
            else:
                diff = (self.psi_leg - self.psi_rot + math.pi) % (2 * math.pi) - math.pi
                step = max(-PSI_RATE * dt, min(PSI_RATE * dt, diff))
                self.psi_rot += step

        # 弧长推进(仅 route 激活后, 以当地巡航速度推进 —— 转弯段自动减速)
        if self.route is not None and self.route_active:
            if self.capability_coordination:
                self.s = self.progress_profile.advance(self.s, dt)
            else:
                self.s = min(self.s + self.route.speed_at(self.s) * dt,
                             self.route.total_length())

        # Measured states replace X0 at every solve; command states stay separate.
        for m in self.mpc:
            if m.state is None:
                if self.center_live is None:
                    continue
                m.state = np.asarray(measurements[m.idx].state)
                m.z = self.fly_z

        # ★ v 下界按阶段: 飞行阶段 0(防倒飞硬兜底, ICC 参考速度恒≥0);
        #   成形阶段 -v_max(允许后退 —— 目标在初始航向后方时, 单向 v 会让
        #   SLSQP 卡在 v=0 边界反复裁剪, 设定点冻结 → 成形永不完成,
        #   实测 uav2 卡 17.1m、/formation/start 永不发)。
        v_min = V_MIN if (self.route is not None and self.route_active) else -self.v_max
        for m in self.mpc:
            if m.state is None:
                continue                      # §8.28: 未锚定不发设定点
            ref, v_des = self._make_ref(m.idx)
            self._publish_diag(m.idx, ref, v_des)
            measured = measurements[m.idx]
            member_v_min = 0. if (self.route is not None and self.route_active) else -m.v_max
            v, w = m.solve_measured(ref, v_des, measured, RATE, member_v_min)
            target = command_endpoint(measured.state, v, w, RATE)
            self.last_setpoints[m.idx] = target
            m.z = m.z + (self.fly_z + self.slots[m.idx][2] - m.z) * min(1.0, dt / TAU_Z)
            self._feedback_report(m.idx, True, 'MEASURED', measured, (v,w))
            self._publish(m.idx, target, v, w)

        self._publish_center_diag()

        # ★ 1Hz 诊断: 每机 状态/参考/误差/失败计数, 定位"起飞后停住"用
        if now - getattr(self, '_diag_t', 0.0) >= 1.0:
            self._diag_t = now
            parts = []
            for m in self.mpc:
                if m.state is None:
                    continue                  # §8.28 同款保护: 未锚定跳过诊断, 防 NoneType 崩溃
                ref, _ = self._make_ref(m.idx)
                err = math.hypot(m.state[0] - ref[0, 0], m.state[1] - ref[1, 0])
                parts.append(
                    'i%d st=(%.1f,%.1f,%.0f°) ref=(%.1f,%.1f) err=%.1f fail=%d'
                    % (m.idx, m.state[0], m.state[1], math.degrees(m.state[2]),
                       ref[0, 0], ref[1, 0], err, m.fail_count))
            self.get_logger().info(
                '[diag] active=%s s=%.1f vmin=%.1f c_live=%s psi_rot=%.0f° | %s'
                % (self.route_active, self.s, v_min,
                   '(%.1f,%.1f)' % self.center_live if self.center_live else 'None',
                   math.degrees(self.psi_rot) if self.psi_rot is not None else 0.0,
                   ' | '.join(parts)))

    def _publish_diag(self, i, ref, v_des):
        """实验审计(不改控制): 每机速度代价参考 + ICC 原始槽位参考。

        icc_reference = [x_ref, y_ref, yaw_ref, speed_ref, s] —— 取 NMPC 时域
        第一拍, 即本拍该机在 ICC 上的成员参考位置/航向/速度参考。
        """
        sr = Float32()
        speed = float(np.asarray(v_des).flat[0])
        sr.data = speed
        self.speed_ref_pubs[i].publish(sr)
        m = Float32MultiArray()
        m.data = [float(ref[0, 0]), float(ref[1, 0]), float(ref[2, 0]),
                  speed, float(self.s)]
        self.icc_ref_pubs[i].publish(m)
        if self.capability_coordination and self.route is not None and self.route_active:
            kappa = self.route.curvature_at(self.s)
            kp = self.route.curvature_derivative_at(self.s)
            nu = self.progress_profile.nu_at(self.s)
            _, ki, omega, _ = member_demands(self.members[i], kappa, nu)
            margins = dynamic_margin(self.members[i], kappa, kp, nu**2, self.progress_profile.slope_at(self.s))
            for name, value in (('curvature_reference',ki), ('heading_rate_reference',omega),
                                ('feasibility_margin', normalized_min_margin(self.members[i],margins))):
                message = Float32(); message.data = float(value)
                self.member_diag[i][name].publish(message)

    def _publish_center_diag(self):
        """实验审计(不改控制): 中心弧长进度 / 中心曲率 / 中心参考速度。"""
        if self.route is not None:
            _x, _y, _psi, kappa = self.route.eval_center(self.s)
            v = self.route.speed_at(self.s)
        else:
            kappa, v = 0.0, 0.0
        if self.capability_coordination:
            p = self.progress_profile
            v = p.nu_at(self.s) if p is not None and self.route_active else 0.
            lower,upper,low_owner,high_owner = team_interval(self.members,kappa) if p is not None else (0.,0.,(-1,-1),(-1,-1))
            values = dict(reference_progress=self.s, reference_progress_rate=v,
                          reference_progress_accel=.5*p.slope_at(self.s) if p is not None and self.route_active else 0.,
                          reference_curvature=kappa,
                          reference_curvature_derivative=self.route.curvature_derivative_at(self.s) if self.route else 0.,
                          team_feasible_lower=lower,
                          team_feasible_upper=upper,
                          selected_radius=self.selected_radius)
            for name,value in values.items():
                message=Float32(); message.data=float(value); self.reference_diag[name].publish(message)
            owners = dict(active_lower_member=int(low_owner[0]),
                          active_upper_member=int(high_owner[0]),
                          active_lower_limit=int(low_owner[1]),
                          active_upper_limit=int(high_owner[1]),
                          candidate_status=-1 if self.planning_fault else (1 if p else 0))
            for name,value in owners.items():
                message=Int32(); message.data=value; self.active_diag[name].publish(message)
        for pub, val in ((self.progress_pub, self.s), (self.kappa_pub, kappa),
                         (self.center_speed_pub, v)):
            msg = Float32()
            msg.data = float(val)
            pub.publish(msg)

    def _publish(self, i, state, v, w):
        m = self._PositionCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.position.x = float(state[0])
        m.position.y = float(state[1])
        m.position.z = float(self.mpc[i].z)
        m.velocity.x = float(v * math.cos(state[2]))
        m.velocity.y = float(v * math.sin(state[2]))
        m.velocity.z = 0.0
        m.yaw = float(state[2])
        m.yaw_dot = float(w)
        m.trajectory_flag = self._PositionCommand.TRAJECTORY_STATUS_READY
        self.pubs[i].publish(m)


def main():
    ap = argparse.ArgumentParser(description='EGO+ICC+NMPC 编队(无领机: N 架全跟随)')
    ap.add_argument('--n-drones', type=int, default=3, help='编队总机数(默认 3)')
    ap.add_argument('--formation', default='line',
                    choices=['line', 'column'], help='队形(默认 line 横队)')
    ap.add_argument('--spacing', type=float, default=10.0, help='槽位间距 m(默认 10)')
    ap.add_argument('--fly-z', type=float, default=FLY_Z,
                    help='飞行高度 m(默认 %.0f; 航点带 z 时以航点高度为准)' % FLY_Z)
    ap.add_argument('--route', default='',
                    help='preset 航线 "x,y x,y ..."(空格分隔, 默认空=等 /formation/route)')
    ap.add_argument('--turn-radius', type=float, default=0.0,
                    help='中心转弯半径 m(默认 0=自动取 半宽+5)')
    ap.add_argument('--speed', type=float, default=V_DES, help='巡航速度 m/s(默认 1.5)')
    ap.add_argument('--v-max', type=float, default=V_MAX,
                    help='单机速度硬上界 m/s(默认 30, 设计上限)')
    ap.add_argument('--a-n-max', type=float, default=5.0,
                    help='最大法向加速度 m/s²(转弯不失速权衡, 默认 5.0, 与飞控 WP_ACC 对齐)')
    ap.add_argument('--turn-margin', type=float, default=5.0,
                    help='转弯半径几何裕量 m(默认 5.0, 沿用"半宽+5")')
    # ★ 实验消融开关(2026-09-23)
    ap.add_argument('--trans-len', type=float, default=None,
                    help='五次曲率过渡段长度 m; 不指定=默认 0.25*R_c, 0=取消过渡(B1 组)')
    ap.add_argument('--uniform-speed-ref', action='store_true',
                    help='成员速度代价参考改用统一中心速度(B2 组; 默认=False 即槽位差速)')
    ap.add_argument('--auto-start', action='store_true',
                    help='启动即开始(测试用; 默认等 /formation/start)')
    ap.add_argument('--capability-coordination', action='store_true')
    ap.add_argument('--experiment-config')
    ap.add_argument('--home-spacing', type=float, default=10.)
    ap.add_argument('--home-offsets', help='JSON list of [east,north] or [east,north,yaw_rad] per member')
    ap.add_argument('--feedback-topic', choices=['odom','pose'], default='odom')
    args = ap.parse_args()
    rclpy.init()
    node = EgoMpcFormation(args.n_drones, args.formation, args.spacing, args.fly_z,
                           args.route, args.turn_radius, args.speed, args.auto_start,
                           args.v_max, args.a_n_max, args.turn_margin,
                           args.trans_len, args.uniform_speed_ref,
                           args.capability_coordination, args.experiment_config,
                           json.loads(args.home_offsets) if args.home_offsets else None,
                           args.home_spacing, args.feedback_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
