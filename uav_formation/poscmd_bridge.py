#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""poscmd_bridge.py —— PositionCommand(/drone_{i}) → MAVROS 位置设定点(/uav{i})

把 EGO+MPC 链路的输出(drone_0..N 的 PositionCommand, world 系 ENU, 原点=FC1 home)
转成 ArduPilot 各机 local 系位置设定点, 发给 /uav{i}/mavros/setpoint_position/local。

坐标换算: 各飞控 local 系原点=各自 home; drone{i} 的 home 相对 FC1 沿东偏移
HOME_OFFSET[i]=(10m*i, 0)。设定点 = PositionCommand.position - HOME_OFFSET[i]。
朝向: PositionCommand.yaw → 四元数(仅绕 z)。

用法:
  source /opt/ros/humble/setup.bash && source ~/ros2_mavros_ws/install/setup.bash
  source ~/bamboo_square_ws/install/setup.bash
  python3 uav_formation/poscmd_bridge.py --n-drones 3
"""
import argparse
import json
import math
import time
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from std_msgs.msg import String
from quadrotor_msgs.msg import PositionCommand
from uav_formation.measured_feedback import WorldTransform, wrap_angle

Q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               durability=DurabilityPolicy.VOLATILE,
               history=HistoryPolicy.KEEP_LAST, depth=10)


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class PosCmdBridge(Node):
    def __init__(self, n_drones, home_offsets, gate_alt=9.0):
        super().__init__('poscmd_bridge')
        self.home_offsets = home_offsets      # [(ex, ey), ...] 每机 home 相对 FC1
        self.transforms = [WorldTransform(*offset) for offset in home_offsets]
        self.last_pose = {}; self.last_state = {}; self.last_command = {}
        self.last_feedback = {}; self.feedback_ok = {}; self.faulted = set()
        self.mode_clients = [self.create_client(SetMode, '/uav%d/mavros/set_mode'%i)
                             for i in range(n_drones)]
        self.mode_requests = {}
        # ★ 起飞门控高度: 0.9×巡航高度 —— NAV_TAKEOFF 先完成全部爬升,
        #   离地即发设定点会取消剩余爬升(实测卡在 1.4m), 复刻 drone_node
        #   "TAKEOFF 到 0.9×高度后才发设定点" 语义。
        self.gate_alt = float(gate_alt)
        self.pubs = [self.create_publisher(
            PoseStamped, '/uav%d/mavros/setpoint_position/local' % i, 10)
            for i in range(n_drones)]
        # ★ 起飞门控: 飞机离地(>0.5m)前不转发设定点 —— ArduPilot GUIDED 下
        #   NAV_TAKEOFF 与远端位置目标流并存时, 目标流会抑制起飞(实测: 飞机带
        #   (35,-10,10) 目标坐在地上不动)。NAV_TAKEOFF(由 ego_env_bridge 在
        #   授权瞬间发出)先爬升到巡航高度, 离地后设定点接管水平/高度引导。
        self.alts = {}                        # {i: 当前高度 m}
        self._susp_since = {}                 # §8.28: {i: 设定点挂起起始时刻}
        self.airborne = set()                 # 已离地的机(告警只打一次)
        self._warn_t = {}                     # {i: 上次"等待爬升"告警时刻(节流 5s)}
        self._diag_t = {}                     # {i: 上次设定点诊断时刻(2s 节流)}
        # ★ 模式门控状态(§8.7): FC 非 armed+GUIDED → 停发该机设定点
        self.states = {}                      # {i: (armed, mode)}
        self._hold_logged = set()             # 已打过"停发"日志的机
        BQ = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        SQ = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        for i in range(n_drones):
            self.create_subscription(
                State, '/uav%d/mavros/state' % i,
                self._mk_state_cb(i), SQ)   # ★ 补: 模式门控的状态订阅(上次漏建)
            self.create_subscription(
                PoseStamped, '/uav%d/mavros/local_position/pose' % i,
                self._mk_alt_cb(i), BQ)
            self.create_subscription(
                PositionCommand, '/drone_%d/position_cmd' % i,
                self._mk_cb(i), 10)
            self.create_subscription(String, '/drone_%d/feedback_status'%i,
                                     lambda msg, i=i: self._feedback_cb(i, msg), 10)
        self.create_timer(.1, self._watchdog)
        self.get_logger().info(
            'poscmd_bridge: %d drones, offsets=%s (起飞门控: z>0.5m 才发设定点)'
            % (n_drones, home_offsets))
        self.get_logger().info('OUTPUT_CONTRACT: PoseStamped position + yaw only; velocity, acceleration and yaw_dot are NOT forwarded')

    def _feedback_cb(self, i, msg):
        try:
            record = json.loads(msg.data)
            age = self.get_clock().now().nanoseconds*1e-9-float(record['stamp'])
            self.feedback_ok[i] = record['valid'] is True and 0 <= age <= .5
            self.last_feedback[i] = time.monotonic()
        except (ValueError, KeyError, TypeError):
            self.feedback_ok[i] = False

    def _fault(self, i, reason):
        if i not in self.faulted:
            self.faulted.add(i)
            self.get_logger().error('drone_%d feedback/setpoint fault: %s; suppress commands, request LOITER'%(i,reason))
        # Latch until restart; never resume an old route automatically after a loss.
        pending = self.mode_requests.get(i)
        if pending is None and self.mode_clients[i].service_is_ready():
            req = SetMode.Request(); req.custom_mode = 'LOITER'
            self.mode_requests[i] = self.mode_clients[i].call_async(req)
        elif pending is not None and pending.done():
            try:
                self.get_logger().warning('drone_%d LOITER transport result: %s'%(i,pending.result()))
            except Exception as exc:
                self.get_logger().error('LOITER transport: '+repr(exc))
            self.mode_requests.pop(i, None)

    def _watchdog(self):
        now = time.monotonic()
        for i in self.airborne:
            if self.states.get(i) != (True, 'GUIDED'):
                continue
            age = max(now-self.last_pose.get(i,0), now-self.last_command.get(i,0),
                      now-self.last_feedback.get(i,0))
            if age > 1. or not self.feedback_ok.get(i, False) or now-self.last_state.get(i,0)>3.:
                self._fault(i, 'stale/missing feedback or command')

    def _mk_state_cb(self, i):
        def cb(m):
            prev = self.states.get(i)
            cur = (bool(m.armed), m.mode)
            self.states[i] = cur
            self.last_state[i] = time.monotonic()
            ok = cur[0] and cur[1] == 'GUIDED'
            if prev is not None and (prev[0] and prev[1] == 'GUIDED') and not ok:
                self.get_logger().info(
                    '★ drone_%d 非 GUIDED/未上锁(%s %s) → 停发设定点, 控制权交还'
                    % (i, cur[1], 'armed' if cur[0] else 'disarmed'))
                self._hold_logged.discard(i)
            elif ok and i in self._hold_logged:
                self.get_logger().info('★ drone_%d 恢复 GUIDED → 设定点恢复' % i)
                self._hold_logged.discard(i)
        return cb

    def _mk_alt_cb(self, i):
        def cb(m):
            # mavros local_position/pose z 为 ENU 向上正(飞行 10m 时 z=+10, 实测)
            self.alts[i] = float(m.pose.position.z)
            self.last_pose[i] = time.monotonic()
        return cb

    def _mk_cb(self, i):
        def cb(msg):
            if i in self.faulted:
                return
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if (not all(math.isfinite(v) for v in (msg.position.x,msg.position.y,msg.position.z,msg.yaw))
                    or not 0 <= age <= .5):
                self._fault(i, 'invalid/stale setpoint')
                return
            if not self.feedback_ok.get(i, False):
                return
            receipt = time.monotonic()
            if (receipt-self.last_feedback.get(i,0) > 1.
                    or receipt-self.last_pose.get(i,0) > 1.
                    or receipt-self.last_state.get(i,0) > 3.):
                if i in self.airborne:
                    self._fault(i, 'stale feedback/pose/state at command reception')
                return
            self.last_command[i] = time.monotonic()
            # ★ 模式门控: 非 armed+GUIDED 停发(地面站收回/降落时让路)
            st = self.states.get(i)
            if st is None or not (st[0] and st[1] == 'GUIDED'):
                self._susp_since.pop(i, None)   # §8.28: 离开挂起态清计时
                if i not in self._hold_logged:
                    self._hold_logged.add(i)
                    self.get_logger().info(
                        'drone_%d 设定点挂起(非 GUIDED/未上锁)' % i)
                return
            # 起飞门控: 未到 0.9×巡航高度不转发(NAV_TAKEOFF 负责全部爬升)
            if self.alts.get(i, -100.0) < self.gate_alt:
                # ★ 2026-09-01: 日志节流 —— 原实现每帧打一行(≈30Hz×N 机),
                #   实测 50 分钟刷 4 万+ 行/5MB+, 排障反而看不清; 每机 5s 一条。
                # ★ §8.28(BUG-3 残余修复): 挂起超 30s → 升级为 WARN 并点名
                #   "起飞指令可能早于 GUIDED 被拒/高度源异常", 不再无限静默。
                now = time.time()
                if i not in self._susp_since:
                    self._susp_since[i] = now
                susp = now - self._susp_since[i]
                if now - self._warn_t.get(i, 0.0) >= 5.0:
                    self._warn_t[i] = now
                    if susp > 30.0:
                        self.get_logger().warn(
                            'drone_%d 已挂起 %.0fs 未爬升 —— 起飞指令可能早于 '
                            'GUIDED 被拒或高度源异常, 建议人工介入'
                            % (i, susp))
                    else:
                        self.get_logger().info(
                            'drone_%d 等待爬升(高度 %.2fm < 门控 %.2fm), 设定点挂起 '
                            '(NAV_TAKEOFF 由桥在授权瞬间发送, 先完成全部爬升)'
                            % (i, self.alts.get(i, -100.0), self.gate_alt))
                return
            self._susp_since.pop(i, None)       # §8.28: 已离地清计时
            if i not in self.airborne:
                self.airborne.add(i)
                self.get_logger().info('★ drone_%d 已离地, 设定点接管' % i)
            transform = self.transforms[i]
            sp = PoseStamped()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.header.frame_id = 'map'
            sp.pose.position.x, sp.pose.position.y = transform.local_position(msg.position.x, msg.position.y)
            sp.pose.position.z = float(msg.position.z)
            qx, qy, qz, qw = yaw_to_quat(wrap_angle(float(msg.yaw)-transform.yaw))
            sp.pose.orientation.x, sp.pose.orientation.y = qx, qy
            sp.pose.orientation.z, sp.pose.orientation.w = qz, qw
            self.pubs[i].publish(sp)
            # ★ 2s 节流诊断: 转发设定点值(定位"起飞后停住"用)
            t_now = time.time()
            if t_now - self._diag_t.get(i, 0.0) >= 2.0:
                self._diag_t[i] = t_now
                self.get_logger().info(
                    '[diag] drone_%d 转发 setpoint world=(%.1f,%.1f,%.1f) '
                    'local=(%.1f,%.1f)'
                    % (i, msg.position.x, msg.position.y, msg.position.z,
                       sp.pose.position.x, sp.pose.position.y))
        return cb


def main():
    ap = argparse.ArgumentParser(description='PositionCommand → MAVROS setpoint 桥')
    ap.add_argument('--n-drones', type=int, default=3)
    ap.add_argument('--home-spacing', type=float, default=10.0,
                    help='相邻机 home 东向间距 m(与 start_all.sh HOME_SPACING 一致)')
    ap.add_argument('--gate-alt', type=float, default=9.0,
                    help='起飞门控高度 m(默认 9=0.9×巡航 10m)')
    ap.add_argument('--home-offsets', help='JSON list of [east,north] or [east,north,yaw_rad], same as NMPC')
    args = ap.parse_args()
    offsets = (json.loads(args.home_offsets) if args.home_offsets else
               [(args.home_spacing * i, 0.0) for i in range(args.n_drones)])
    if len(offsets) != args.n_drones:
        ap.error('one home transform per member required')
    rclpy.init()
    node = PosCmdBridge(args.n_drones, offsets, gate_alt=args.gate_alt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
