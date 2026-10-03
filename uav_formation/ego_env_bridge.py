#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ego_env_bridge.py —— 真飞控(MAVROS) → EGO-planner 环境桥(里程碑B)

bamboo_square_ws 的 EGO-planner(plan_manage) 为 drone_0(领机) 规划时需要:
  1. /drone_0/visual_slam/odom  (nav_msgs/Odometry, world 系) —— 仿真里由
     poscmd_2_odom 闭环提供; 真机接入改为 MAVROS /uav0 local_position 拼装。
  2. /drone_0/pcl_render_node/cloud  (sensor_msgs/PointCloud2) —— 仿真里由
     pcl_render_node 从森林地图渲染; ArduPilot SITL 无障碍地图, 发布空点云
     (grid_map 缓冲默认 0=free, EGO 按无障碍平滑轨迹规划)。

★ 里程计发布按 [armed && GUIDED] 门控: EGO FSM 收到首帧 odom 即自动规划
  (have_trigger_=true, realworld_experiment=false), 门控保证"轨迹起点=授权
  起飞时刻的真实位置", 避免在地面提前规划导致轨迹时间轴作废。

★ --mission-goals(地面站上传航线模式, §8.1): 配合规划器 fsm.flight_type=1
  (MANUAL_TARGET, 订阅绝对话题 /goal)。后台线程经 PullMission 从 FC1 拉取
  地面站上传的航线(未授权时每 5s 拉 + 授权时补拉一次), 授权后逐点发 /goal
  —— 领机水平距当前目标 < --goal-radius 自动切下一点。地面站工作流保持
  "画线→上传→auth" 不变; 航点格式与编队模式一致 [(x,y,z) 公共系 ENU 米]。

用法:
  source /opt/ros/humble/setup.bash && source ~/ros2_mavros_ws/install/setup.bash
  source ~/bamboo_square_ws/install/setup.bash     # quadrotor_msgs 由本节点不依赖
  python3 uav_formation/ego_env_bridge.py --uav-index 0
  python3 uav_formation/ego_env_bridge.py --uav-index 0 --mission-goals
"""
import argparse
import math
import os
import sys
import threading
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2, PointField
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from std_msgs.msg import Bool, Float32, Int32MultiArray

from uav_formation.gcs_channel import MavlinkChannel
from uav_formation.geometry import make_slots, rotate_slot, assign_slots

POSE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=10)
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=10)

REF = (30.893000, 120.087000)     # 公共系 ENU 基线(= world 原点=FC1 home)
POSE_STALE_S = 3.0                # 位置样本过期秒数(掉机的位置不再计入均值中心)


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class EgoEnvBridge(Node):
    def __init__(self, uav_index, n_drones=3, formation='line', spacing=10.0,
                 mission_goals=False, goal_radius=3.0,
                 default_z=10.0, pull_interval=5.0, home_spacing=10.0):
        super().__init__('ego_env_bridge')
        self.uav_index = int(uav_index)
        self.ns = '/uav%d' % self.uav_index
        self.gated = False                 # 当前是否处于"已授权发布"状态
        self.state = None
        self.state_lock = threading.Lock()   # armed_modes 并发保护
        self.armed_modes = {}                # {i: (armed, mode)} 全部 N 架(门控诊断)

        # ---------- 均值中心反馈(无领机 §8.2) ----------
        # 中心 = mean(各在线机 位置ᵢ − 槽位ᵢ); 槽位全对称(geometry.make_slots);
        # 掉机留洞: 过期(>POSE_STALE_S)的机不计入均值, 全队照飞。
        self.n_drones = int(n_drones)
        self.slots = [s[:2] for s in make_slots(formation, self.n_drones, spacing)]
        # ★ 差速车拐角圆弧半径(§8.4): ≥ 编队半宽 + 5m 余量, 保证内外偏移
        #   曲线为同心圆弧不重叠(内侧半径 = R − 半宽 ≥ 5m)
        self.round_radius = max(abs(sv) for s_ in self.slots for sv in s_) + 5.0
        # ★ 各机 home 相对 FC1 的偏移(东向 spacing·i, 与 start_all.sh 一致):
        #   local_position 是各机本地系(原点=自家 home), 均值中心必须先转
        #   世界系(world_i = local_i + home_offset_i) —— 漏掉此转换中心会被
        #   低估 spacing·(i)/N, 到位判定永远不触发(实测卡死在第一目标)。
        self.home_offsets = [(home_spacing * i, 0.0) for i in range(self.n_drones)]
        self.poses = {}                    # {i: (position(x,y), wall_time)}

        # EGO 规划器(drone_0 命名空间)的 odom/cloud 输入
        self.odom_pub = self.create_publisher(
            Odometry, '/formation_center/odom', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/drone_%d/pcl_render_node/cloud' % uav_index, 10)
        # ★ ICC 转弯平滑化(第一阶段): 向 egompc_formation 下发整条航点
        #   (/formation/route) 与 起飞/成形完成信号 (/formation/start)。
        self.route_pub = self.create_publisher(PoseArray, '/formation/route', 10)
        self.start_pub = self.create_publisher(Bool, '/formation/start', 10)
        # ★ DO_CHANGE_SPEED 速度下发(补充文档机制 A): PullMission 解析后经此话题
        #   转发给 egompc_formation 更新 route.v / 转弯半径。
        self.speed_pub = self.create_publisher(Float32, '/formation/speed', 10)
        self._last_speed = None
        self.create_subscription(
            State, '%s/mavros/state' % self.ns,
            self._mk_state_cb(self.uav_index), STATE_QOS)
        # ★ 2026-09-01: 额外订阅其余 N-1 架 state —— 周期诊断输出每架 armed/mode。
        # ★ 2026-09-23 修复: 本机 state 也改走 _mk_state_cb(原直连 _on_state),
        #   否则 armed_modes 缺本机项 → "全部 N 架"聚合门控永远无法满足。
        for i in range(self.n_drones):
            if i == self.uav_index:
                continue
            self.create_subscription(
                State, '/uav%d/mavros/state' % i, self._mk_state_cb(i), STATE_QOS)
        self.create_timer(5.0, self._auth_diag)
        for i in range(self.n_drones):
            self.create_subscription(
                PoseStamped, '/uav%d/mavros/local_position/pose' % i,
                self._mk_pose_cb(i), POSE_QOS)
        # velocity_local 仅领机 MAVROS 有就够(中心速度=各机均值近似, 平滑用)
        self.create_subscription(TwistStamped, '%s/mavros/local_position/velocity_local' % self.ns,
                                 self._on_twist, POSE_QOS)
        self._last_twist = None
        self.cloud_timer = self.create_timer(0.5, self._pub_empty_cloud)
        self.get_logger().info(
            'ego_env_bridge(无领机): %d 架位置 → 均值中心 /formation_center/odom, '
            '槽位=%s' % (self.n_drones,
                         [(round(s[0], 1), round(s[1], 1)) for s in self.slots]))

        # ---------- 地面站上传航线 → EGO 逐点 goal(§8.1) ----------
        self.mission_mode = bool(mission_goals)
        self.goal_radius = float(goal_radius)
        self.default_z = float(default_z)
        self.pull_interval = float(pull_interval)
        self.goals = []                    # [(x, y, z) ENU world]
        self.goal_idx = 0
        self._goals_sent = False           # 当前 goal 是否已发出
        # ★ 成形阶段(§8.5): 第一个 goal 前要求全员 在线+巡航高度+槽位误差
        #   <1.5m 且持续 2s(复刻 drone_node FORMING+HOLD, 消除初始收敛 4.79m 压缩)
        self.form_hold_s = 2.0
        self.form_started = None
        # ★ 成形锚点(§8.10): 门控开启瞬间的均值中心, 冻结为成形阶段的中心
        #   odom —— 均值中心自引用回路存在自由漂移模态(整体匀速漂移自洽),
        #   锚定后飞机收敛到 锚点+槽位, 漂移被物理修正。
        self._slot_perm = None             # §8.14 槽位指派(防对穿): None=未指派
        self._takeoff_sent = False         # §8.28 起飞指令已发标志(航线采纳时才发)
        self.slot_assign_pub = self.create_publisher(
            Int32MultiArray, '/formation/slot_assign', 10)
        self.form_anchor = None
        self._mission_started = False    # 第一个 goal 已发出(此后 odom 切回实时均值)
        self._auth_once = False            # 授权后已补拉过一次
        self._goals_from_upload = False    # ★ goals 是否来自上传点直收(优先于拉取)
        self.goals_lock = threading.Lock() # goals 并发保护(上传回调/拉取线程/定时器)
        self._stop = False
        # ★ 每架 FC 各一条 MAVROS 透传通道(/uas1→FC1, ...) —— 两种模式都需要:
        #   mission 模式用于拉航线; 全模式用于授权门控开启时发 NAV_TAKEOFF
        #   (ArduPilot GUIDED 收到位置目标【不会】自动起飞, 必须显式起飞指令,
        #   否则飞机带目标坐在地上不动 —— 实测踩坑)。
        self.mav_chs = [
            MavlinkChannel(self, '/uas%d' % (i + 1), REF[0], REF[1],
                           passive=True)
            for i in range(self.n_drones)]
        if self.mission_mode:
            self.goal_pub = self.create_publisher(
                PoseStamped, '/goal', 10)
            self.trigger_pub = self.create_publisher(
                PoseStamped, '/traj_start_trigger', 10)
            # ★ 队形航向 = 当前航段方位角(根源修复 z 轴摆动, §8.6):
            #   航向由航线本身决定(goal[k-1]→goal[k] 的方位角, 常量零噪声),
            #   不再由 EGO 轨迹切线的 atan2 反推(低速/重规划跳变源)。
            self.heading_pub = self.create_publisher(
                Float32, '/formation/heading', 10)
            self._leg_heading = None
            # 中心轨迹流: 流新鲜 = 规划器正在执行目标(避免"目标早于 odom
            #  被规划器丢弃"的死锁 —— 流断档超 2s 视为未接受, 周期重发当前目标)
            self._last_cmd_t = 0.0
            self._last_goal_send = 0.0
            self.create_subscription(
                PositionCommand, '/formation_center/position_cmd',
                self._on_center_cmd, 10)
            threading.Thread(target=self._pull_loop, daemon=True).start()
            self.create_timer(0.5, self._goal_tick)
            self.get_logger().info(
                '★ 地面站上传航线模式: PullMission(全 FC 一致性校验) → 逐点 /goal, '
                '到位半径 %.1fm' % self.goal_radius)

        # ★ 2026-09-23 修复: 起飞重发 + preset 成形门控看门狗(preset/mission 通用)
        self.create_timer(0.5, self._watch_tick)

        # ---------- 位置流请求(★ 关键, §8.2 实测坑) ----------
        # 新版 mavros 不自动请求流; SR1_POSITION 默认在 SITL 不生效(实测 serial1
        # 只有心跳)。不请求 LOCAL_POSITION_NED → 无 pose → 无均值中心 odom →
        # EGO 规划器永远 INIT。drone_node 的 request_streams 同款。
        self.create_client(MessageInterval, self.ns + '/mavros/set_message_interval')
        threading.Thread(target=self._request_streams_loop, daemon=True).start()

    def _on_center_cmd(self, msg):
        self._last_cmd_t = time.time()

    @staticmethod
    def round_corners(pts, radius):
        """差速车拐角圆弧化(§8.4): 把尖角替换成半径 radius 的圆弧过渡
        (每角 3 点: 弧起点 A / 弧中点 M / 弧终点 B, EGO minjerk 平滑串联)。

        ★ 根因: 参考轨迹 = 中心路径的 ±槽位偏移曲线; 中心路径拐角半径
          (EGO minjerk ~0.4m)小于编队半宽(槽位 10m)时, 内侧偏移曲线半径
          变负数 → 自交/倒退 → 轨迹重叠相撞。圆弧化后中心路径曲率半径
          ≥ 编队半宽, 内外偏移曲线 = 同心圆弧 —— 内轮减速、外轮加速,
          永不相交(差速车转弯)。
        圆弧不精确过角点(向内侧切割, 90°/R=15 时偏离角点约 6m) ——
        这是物理上必要的(编队有宽度, 无法绕零半径角点)。
        纯函数便于单测。pts: [(x,y,z)], 返回新列表。"""
        if len(pts) < 3 or radius <= 0:
            return list(pts)
        out = [(float(pts[0][0]), float(pts[0][1]), float(pts[0][2]))]
        for i in range(1, len(pts) - 1):
            p_prev, p, p_next = pts[i - 1], pts[i], pts[i + 1]
            d1 = (p[0] - p_prev[0], p[1] - p_prev[1])
            d2 = (p_next[0] - p[0], p_next[1] - p[1])
            l1 = math.hypot(*d1)
            l2 = math.hypot(*d2)
            if l1 < 1e-6 or l2 < 1e-6:
                out.append((float(p[0]), float(p[1]), float(p[2])))
                continue
            u1 = (d1[0] / l1, d1[1] / l1)
            u2 = (d2[0] / l2, d2[1] / l2)
            dot = max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))
            theta = math.acos(dot)                 # 转角(0~π)
            if theta < math.radians(15):           # 近直线: 保持原点
                out.append((float(p[0]), float(p[1]), float(p[2])))
                continue
            # 切线缩进 L = R·tan(θ/2), 不超过每段一半(留 4m 余量)
            L = min(radius * math.tan(theta / 2.0),
                    (l1 - 4.0) / 2.0, (l2 - 4.0) / 2.0)
            L = max(L, 1.0)
            r_eff = L / math.tan(theta / 2.0)      # 实际可达圆弧半径
            A = (p[0] - u1[0] * L, p[1] - u1[1] * L)
            B = (p[0] + u2[0] * L, p[1] + u2[1] * L)
            cross = u1[0] * u2[1] - u1[1] * u2[0]  # >0 左转 <0 右转
            side = 1.0 if cross > 0 else -1.0
            n1 = (-u1[1] * side, u1[0] * side)
            O = (A[0] + n1[0] * r_eff, A[1] + n1[1] * r_eff)
            a0 = math.atan2(A[1] - O[1], A[0] - O[0])
            a1 = math.atan2(B[1] - O[1], B[0] - O[0])
            if side > 0:
                while a1 < a0:
                    a1 += 2 * math.pi
            else:
                while a1 > a0:
                    a1 -= 2 * math.pi
            am = (a0 + a1) / 2.0
            M = (O[0] + r_eff * math.cos(am), O[1] + r_eff * math.sin(am))
            for q in (A, M, B):
                out.append((q[0], q[1], float(p[2])))
        out.append((float(pts[-1][0]), float(pts[-1][1]), float(pts[-1][2])))
        return out

    def on_gcs_speed(self, speed):
        """DO_CHANGE_SPEED(178) 上报(MavlinkChannel._pull_once → PullMission.on_speed):
        转发给 egompc(/formation/speed)。全 FC 拉取会重复上报同一速度, 去重后仅变化时发布。"""
        try:
            s = float(speed)
        except (TypeError, ValueError):
            return
        if s <= 0:
            return
        if s == self._last_speed:
            return
        self._last_speed = s
        self.speed_pub.publish(Float32(data=s))
        self.get_logger().info(
            '★ DO_CHANGE_SPEED 速度 %.2f m/s → /formation/speed' % s)

    def on_gcs_waypoints(self, wps):
        """MavlinkChannel 宿主回调 —— 地面站经透传通道上传的航点。

        ★ 2026-09-01 修复(实测坑): mission 模式下把【广播上传】(target=0,
        会被飞控回显到 serial1 → 桥可见)直接作为 goals —— 不再依赖 FC 任务
        存储拉取: 单点上传(count=1)会被 ArduPilot home 顶替(seq0 槽恒为
        home, 上传 N 点 → 真航点 N-1), 拉取永远"空任务", 整链卡死。
        逐机直发(target=1..N)桥收不到, 仍由 _pull_loop 拉取兜底。
        """
        if not self.mission_mode:
            return (1, 0)
        if not wps:
            return (1, 0)
        if len(wps) == 1:
            self.get_logger().warn(
                '单点上传 %s 将被 ArduPilot home 顶替(拉取侧恒空任务); '
                '已直接采用该点作目标, 建议一次上传 ≥2 点航线(首点为起始点)'
                % (str(tuple(round(v, 1) for v in wps[0])),))
        pts = [(float(w[0]), float(w[1]),
                float(w[2]) if len(w) > 2 and float(w[2]) > 0.1 else self.default_z)
               for w in wps]
        # 差速车原地转向(§8.5)不做切角圆弧 —— 航线精准过角点;
        # round_corners(§8.4)保留备用(未来高速连续通过时可启用)
        # pts = self.round_corners(pts, self.round_radius)
        with self.goals_lock:
            if pts == self.goals:
                return (1, 0)              # 多通道重复回显(实测 ×2/×3), 幂等
            self.goals = pts
            self.goal_idx = 0
            self._goals_sent = False
            self._goals_from_upload = True
        self._adopt_route_z(pts)          # ★ 航点高度 → NAV_TAKEOFF 高度(对齐 v13)
        self._publish_route()              # ★ ICC: 下发整条航点供 egompc 重建轨迹
        # ★ §8.28: 已授权且起飞未发 → 此刻航线刚采纳(高度已对齐), 才发起飞
        if self.gated and not getattr(self, '_takeoff_sent', False):
            self._send_takeoff_all()
        # ★ 航向先行(§8.6): goals 一设置就发布第一段方位角 —— egompc 成形阶段
        #   按此航向展开队形, 否则成形布局与第一段航向不匹配(实测 14.2m 死锁)
        c0 = self._current_center()
        if c0 is not None:
            self._set_leg_heading(0, c0)
        self.get_logger().info(
            '★ 上传点直接采用(%d 点): %s → 授权后逐点 /goal'
            % (len(pts), [(round(x, 1), round(y, 1)) for x, y, _ in pts]))
        return (1, 0)

    # ---------- MAVROS 回调 ----------
    def _mk_pose_cb(self, i):
        ox, oy = self.home_offsets[i]

        def cb(m):
            p = m.pose.position
            self.poses[i] = ((float(p.x) + ox, float(p.y) + oy), time.time())
            if not hasattr(self, '_alts'):
                self._alts = {}
            self._alts[i] = float(p.z)
        return cb

    def _mk_state_cb(self, i):
        """第 i 架 state 回调: 记录 armed/mode; 门控按【全部 N 架】聚合判定。

        ★ 2026-09-23 修复: 原实现只让 uav_index(1 号机)驱动门控。集中式(一块板
        管 N 架)下, 1 号机一进 GUIDED 门控就开, 此时其余机可能还没切模式, 广播
        的 NAV_TAKEOFF 被 ArduPilot 拒收 → 该机永久趴地
        (见 docs/分布式时序与职责.md §4 "集中版死锁教训")。
        """
        def cb(m):
            with self.state_lock:
                self.armed_modes[i] = (bool(m.armed), str(m.mode))
                all_states = dict(self.armed_modes)
            self._on_state(m, all_states)
        return cb

    def _auth_diag(self):
        """★ 2026-09-01: 未授权时每 5s 输出每架 armed/mode, 定位"门控为什么
        不开"(实测: 全程未 armed/GUIDED 却无任何提示, 只能靠猜)。授权后停。"""
        if self.gated:
            return
        with self.state_lock:
            states = dict(self.armed_modes)
        if not states:
            self.get_logger().warn(
                '门控未开: 尚未收到任何飞控 state(检查 MAVROS 连接/state 话题)')
            return
        desc = ' '.join(
            'uav%d:%s' % (i, 'armed+GUIDED' if a and md == 'GUIDED'
                          else ('armed/' + md if a else '未armed/' + md))
            for i, (a, md) in sorted(states.items()))
        self.get_logger().warn(
            '门控未开(需全部 %d 架 armed && GUIDED, 当前: %s) —— 地面站须先授权 '
            '(arm+GUIDED), 桥才会发 NAV_TAKEOFF/中心 odom//goal' % (self.n_drones, desc))

    def _on_state(self, m, all_states=None):
        # ★ 2026-09-23: 门控 = 全部 N 架 armed && GUIDED(原先只看本机 state)
        st = all_states if all_states is not None else dict(self.armed_modes)
        authed = (len(st) >= self.n_drones
                  and all(a and md == 'GUIDED' for a, md in st.values()))
        if authed != self.gated:
            self.gated = authed
            self.get_logger().info(
                '★ 里程计门控 %s (armed=%s mode=%s)'
                % ('开启, EGO 将自动规划' if authed else '关闭',
                   m.armed, m.mode))
            if authed:
                self._slot_perm = None      # §8.14: 门控开启重置, 成形检查首拍重新指派
                self._form_t0 = time.time()   # §8.26: 成形计时起点(卡住升级告警用)
                self._takeoff_sent = False    # §8.28: 每次门控开启重计(航线采纳时发)
                # ★ §8.24 阶段式启动(2026-09-09): 槽位指派提前到【起飞命令之前】——
                #   航线已到时此刻按"地面静止位置+第一段航向"定死槽位编号,
                #   五机先对准已定槽位再动, 消除指派晚于运动造成的槽位突然变换;
                #   航线未到时维持原策略(成形检查首拍指派)。
                c0 = self._current_center()
                if self.goals and c0 is not None:
                    gx0, gy0 = self.goals[0][0], self.goals[0][1]
                    hd0 = math.atan2(gy0 - c0[1], gx0 - c0[0])
                    fresh = [i for i in range(self.n_drones)
                             if i in self.poses
                             and time.time() - self.poses[i][1] <= 3.0]
                    if len(fresh) == self.n_drones:
                        rot_slots = []
                        for sl in self.slots:
                            sx, sy = rotate_slot((sl[0], sl[1], 0.0), hd0)
                            rot_slots.append((c0[0] + sx, c0[1] + sy))
                        poss = [self.poses[i][0] for i in range(self.n_drones)]
                        self._slot_perm = assign_slots(rot_slots, poss)
                        msg = Int32MultiArray()
                        msg.data = [int(x) for x in self._slot_perm]
                        self.slot_assign_pub.publish(msg)
                        self.get_logger().info(
                            '★ 槽位指派(防对穿, 起飞前定死): %s' % (self._slot_perm,))
                self._send_takeoff_all()
                # ★ ICC: preset 模式授权即发 start(mission 模式等成形稳定后再发)
                # ★ §8.28(BUG-1 残余修复): mission 模式航线未采纳不发起飞 ——
                #   旧逻辑门控一开就广播 NAV_TAKEOFF(default_z=2 默认高度),
                #   航线到达后又按 10m 补发, 高度指挥忽上忽下(实测)。
                #   改为: 起飞在航线采纳时按正确高度单发(见 _pull_loop/on_gcs_wps);
                #   preset 模式 goals 预设, 维持门控开启即发。
                if not self.mission_mode or self.goals:
                    self._send_takeoff_all()
                else:
                    self.get_logger().info(
                        '★ 航线未采纳, 起飞指令将在航线采纳后按航点高度自动发出')
                # ★ 2026-09-23: preset 模式的 /formation/start 不再"授权即发",
                #   改由 _watch_tick 用与 mission 相同的成形判据(全员在线+高度+
                #   槽位误差稳定 2s)决定; 原行为会让 ICC 参考在飞机还在地面时
                #   就开始推进(实测 s 跑满 153.5m 而编队中心几乎没动)。
                self.form_anchor = self._current_center()
                fa = self.form_anchor
                self.get_logger().info(
                    '★ 成形锚点 = (%.2f, %.2f)' % (fa[0], fa[1])
                    if fa else '★ 成形锚点 = None')
        self.state = m

    def _send_takeoff_all(self):
        """授权门控开启瞬间向全部 FC 发 NAV_TAKEOFF(高度=fly_z)。
        ★ ArduPilot GUIDED 收到位置目标不会自动起飞(实测: 带目标坐在地上
        不动), 必须显式 NAV_TAKEOFF —— drone_node 模式的 TAKEOFF 阶段同职责。"""
        for i, ch in enumerate(self.mav_chs):
            try:
                ch._send(self.mav_chs[0].mav.command_long_encode(
                    i + 1, 1, 22, 0, 0, 0, 0, 0, 0, 0,
                    float(self.default_z)))
                self.get_logger().info('★ FC%d NAV_TAKEOFF(%.1fm) 已发' % (i + 1, self.default_z))
            except Exception as e:
                self.get_logger().error('FC%d 起飞指令异常: %s' % (i + 1, e))
        self._takeoff_sent = True

    def _on_twist(self, m):
        self._last_twist = m

    # ---------- 发布 ----------
    def _pub_empty_cloud(self):
        """空点云: header 对齐 world。grid_map 缓冲默认 0=free → 无障碍规划。"""
        c = PointCloud2()
        c.header.stamp = self.get_clock().now().to_msg()
        c.header.frame_id = 'world'
        c.height = 1
        c.width = 0
        c.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        c.is_bigendian = False
        c.point_step = 16
        c.row_step = 0
        c.is_dense = True
        self.cloud_pub.publish(c)

    @staticmethod
    def compute_center(samples):
        """均值中心: samples = [(x, y, slot_x, slot_y), ...](在线机)。
        center = mean(pos_i − slot_i)。纯函数, 便于单测。"""
        if not samples:
            return None
        cx = sum(x - sx for x, y, sx, sy in samples) / len(samples)
        cy = sum(y - sy for x, y, sx, sy in samples) / len(samples)
        return (cx, cy)

    def tick_odom(self):
        """由定时器调用: 门控开 + 有在线机位置 → 均值中心 Odometry 发布。"""
        if not (self.gated and self.poses):
            return
        now = time.time()
        samples = [(x, y, self.slots[i][0], self.slots[i][1])
                   for i, ((x, y), t) in sorted(self.poses.items())
                   if now - t <= POSE_STALE_S]
        if not samples:
            return
        if len(samples) < self.n_drones:
            # 掉机留洞: 只用在线机均值(节流打印, 3s 一次)
            if not hasattr(self, '_warn_t') or now - self._warn_t > 3.0:
                self._warn_t = now
                self.get_logger().warn(
                    '中心反馈仅 %d/%d 架在线(掉机留洞模式)' % (len(samples), self.n_drones))
        live = self.compute_center(samples)
        if live is None:
            return
        # ★ 成形阶段(首个 goal 未发): odom = 冻结锚点(原地成形, 抗漂移);
        #   首个 goal 发出后: odom = 实时均值(EGO 重规划跟随机队实际位置)
        if self.mission_mode and not self._mission_started \
                and self.form_anchor is not None:
            center = self.form_anchor
        else:
            center = live
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'world'
        od.child_frame_id = 'formation_center'
        od.pose.pose.position.x = float(center[0])
        od.pose.pose.position.y = float(center[1])
        if self._last_twist is not None:
            od.twist.twist = self._last_twist.twist
        self.odom_pub.publish(od)

    def _pub_odom_tick(self):
        self.tick_odom()

    # ---------- 位置流请求 ----------
    def _request_streams_loop(self):
        """向全部 N 架请求 LOCAL_POSITION_NED(20Hz)。
        ★ 周期性巡检重请求(2026-09-01): 实测位置流会在 FC 运行 20~35 分钟后
          静默停止(1 倍速也复现, 原因未明), 一次性请求不够 —— 每 60s 检查
          本机 pose 新鲜度, 断流即重发请求, 保证中心 odom 永不饿死。"""
        time.sleep(3.0)                # 等 MAVROS 服务就绪
        while not self._stop:
            for i in range(self.n_drones):
                pose = self.poses.get(i)
                fresh = pose is not None and time.time() - pose[1] < 5.0
                if fresh:
                    continue
                cli = self.create_client(
                    MessageInterval, '/uav%d/mavros/set_message_interval' % i)
                if not cli.service_is_ready():
                    continue
                fut = cli.call_async(MessageInterval.Request(
                    message_id=32, message_rate=20.0))
                t0 = time.time()
                while not fut.done() and time.time() - t0 < 5.0:
                    time.sleep(0.1)    # 主线程 spin 负责完成 future
                if fut.done() and fut.result() is not None \
                        and getattr(fut.result(), 'success', False):
                    self.get_logger().info(
                        '★ uav%d 位置流%s(LOCAL_POSITION_NED@20Hz)'
                        % (i, '巡检正常' if pose is None else '已重请求'))
            time.sleep(60.0)           # 巡检周期

    # ---------- 地面站上传航线 → 逐点 /goal(§8.1) ----------
    def _pull_fc(self, fc):
        """经第 fc 架自己的 MAVROS 通道拉取该机的任务存储(失败 None/空列表=无航线)。"""
        return self.mav_chs[fc - 1].pull_mission(fc, timeout=8.0)

    def _pull_all_fc(self):
        """拉取全部 FC 航线 + 【多数表决】一致性关卡(§8.25, 2026-09-09):

        场景: 地面站逐架上传时手滑把 1 号机的航线传给了 3 号机 —— 旧逻辑
        取"第一架拉取成功"当参照, 错航线可能碰巧被采纳且照飞。

        新逻辑(阶段确认第一关):
          1. 逐架拉取, 按航线内容分组;
          2. 取【出现最多】的航线为参照(多数派), 少数派指名告警;
          3. 无多数派(全部互不相同)→ 不采纳任何航线, 返回 None,
             门控不开 —— 航线一致性确认失败, 人工重传后再试。
        返回参照航线 [(x,y,z)] 或 None(全部失败/全空/无多数)。"""
        n = self.n_drones
        groups = {}                       # key=航线内容, value=[(fc, wps)]
        for fc in range(1, n + 1):
            wps = self._pull_fc(fc)
            if wps is None:
                print(f"[ego_bridge] FC{fc} 拉取失败(超时)")
                continue
            if not wps:
                print(f"[ego_bridge] FC{fc} 无航线(空任务)")
                continue
            key = tuple(tuple(round(float(c), 2) for c in w) for w in wps)
            groups.setdefault(key, []).append((fc, wps))
        if not groups:
            return None
        best_key = max(groups, key=lambda k: len(groups[k]))
        best = groups[best_key]
        sizes = sorted((len(v) for v in groups.values()), reverse=True)
        if len(sizes) >= 2 and sizes[0] == sizes[1]:
            tied = [sorted(fc for fc, _ in v) for v in groups.values()
                    if len(v) == sizes[0]]
            print(f"[ego_bridge] [!!] 航线一致性校验失败: 无多数派(并列 {sizes}, "
                  f"分组 FC{tied}) —— 不采纳任何航线, 门控不开; "
                  f"请在地面站重新逐架上传同一条航线!")
            return None
        ref_fc, ref = best[0]
        majority_fcs = [fc for fc, _ in best]
        minority = [fc for k in groups if k != best_key
                    for fc, _ in groups[k]]
        if minority:
            print(f"[ego_bridge] [!] 航线不一致: 多数派 FC{majority_fcs}"
                  f"({len(ref)} 点), 少数派 FC{minority} —— 采用多数派, "
                  f"少数派请在地面站重传!")
        else:
            print(f"[ego_bridge] 航线一致性校验 ✓ (全 {len(best)} 架一致, "
                  f"{len(ref)} 点)")
        return ref

    @staticmethod
    def route_mismatch(wps, ref):
        """对比两条航线(参照 ref), 返回不一致描述字符串或 None(一致)。纯函数便于单测。"""
        if not wps:
            return "空任务"
        if [list(w) for w in wps] != [list(w) for w in ref]:
            if len(wps) != len(ref):
                return "%d点≠%d点" % (len(wps), len(ref))
            return "内容不同"
        return None

    def _pull_loop(self):
        """后台线程: 未授权每 pull_interval 拉一轮(全 FC 拉取+一致性校验)
        + 授权时补拉一次(与 drone_node._pull_loop 的"按需拉取"同一防打断
        设计, §7.4/§7.8; 飞行中不改航线 = 授权后不再轮询)。"""
        time.sleep(2.0)                # 等 MAVROS/通道就绪
        while not self._stop:
            need = (not self.gated) or (not self.goals) or not self._auth_once
            if need and self.mav_chs is not None:
                try:
                    wps = self._pull_all_fc()
                except Exception as e:
                    print(f"[ego_bridge] 拉取航线异常: {e}")
                    wps = None
                if wps:
                    # 差速车原地转向(§8.5): 不做切角圆弧, 航线精准过角点
                    pts = [(float(w[0]), float(w[1]),
                            float(w[2]) if len(w) > 2 and w[2] > 0.1
                            else self.default_z) for w in wps]
                    changed = False
                    with self.goals_lock:
                        # ★ 2026-09-01: 上传点直收优先 —— 地面站上传点(尤其
                        #   广播单点)已被直接采用时, 拉取结果(空/被 home 顶替
                        #   的残点)不得覆盖。
                        if self._goals_from_upload:
                            pass
                        elif pts != self.goals:
                            self.goals = pts
                            self.goal_idx = 0
                            self._goals_sent = False
                            changed = True
                            c0 = self._current_center()
                            if c0 is not None:
                                self._set_leg_heading(0, c0)
                            self.get_logger().info(
                                '★ 航线已到达(%d 点): %s → 授权后逐点 /goal'
                                % (len(pts),
                                   [(round(x, 1), round(y, 1)) for x, y, _ in pts]))
                    if changed:
                        # ★ _publish_route 内部会再取 goals_lock, 必须放到锁外
                        #   调用, 否则 threading.Lock 不可重入会自死锁(卡死 pull
                        #   线程持锁 → 主线程 _goal_tick 阻塞 → /uav0 state 回调
                        #   _on_state 得不到执行 → 不发 NAV_TAKEOFF → 不起飞)。
                        self._adopt_route_z(pts)   # ★ 航点高度 → NAV_TAKEOFF 高度
                        self._publish_route()   # ★ ICC: 下发整条航点
                        # ★ §8.28: 已授权且起飞未发 → 航线刚采纳, 此时才发起飞
                        if self.gated and not getattr(self, '_takeoff_sent', False):
                            self._send_takeoff_all()
                if self.gated and not self._auth_once:
                    self._auth_once = True   # 授权后只补拉一次
            time.sleep(self.pull_interval)

    def _current_center(self):
        """当前均值中心 (x, y)；无在线机样本返回 None。"""
        if not self.poses:
            return None
        now = time.time()
        samples = [(x, y, self.slots[i][0], self.slots[i][1])
                   for i, ((x, y), t) in sorted(self.poses.items())
                   if now - t <= POSE_STALE_S]
        return self.compute_center(samples)

    def _watch_tick(self):
        """0.5s 看门狗(两种模式通用):

        1) 起飞重发: 门控已开但航线未开始时, 凡高度 < 0.9*default_z 的机每 3s
           重发一次 NAV_TAKEOFF —— 授权是逐架完成的, 单发起飞必被后授权的机
           拒收(docs/分布式时序与职责.md §4)。mission 模式下 _goal_tick 也会重发,
           两处重复无害。
        2) preset 模式成形门控: 与 mission 模式同一判据(全员在线 + 高度达标 +
           槽位误差 < 1.5m 持续 form_hold_s), 满足后才发 /formation/start。
           preset 的槽位按基准朝向(0°)摆放, 与 egompc 无 /formation/heading 时的
           槽位一致。
        """
        if self._mission_started or not self.gated:
            return
        now = time.time()
        alts = getattr(self, '_alts', {})

        # (1) 起飞重发
        if now - getattr(self, '_tk_watch_t', 0.0) >= 3.0:
            self._tk_watch_t = now
            for i, ch in enumerate(self.mav_chs):
                if alts.get(i, -100.0) < 0.9 * self.default_z:
                    try:
                        ch._send(self.mav_chs[0].mav.command_long_encode(
                            i + 1, 1, 22, 0, 0, 0, 0, 0, 0, 0,
                            float(self.default_z)))
                        self.get_logger().warn(
                            '★ FC%d 高度 %.1fm < 门控 %.1fm, 重发 NAV_TAKEOFF'
                            % (i + 1, alts.get(i, -100.0), 0.9 * self.default_z))
                    except Exception as e:
                        self.get_logger().error('FC%d 起飞重发异常: %s' % (i + 1, e))

        if self.mission_mode:
            return                      # mission 模式成形判据在 _goal_tick

        # (2) preset 模式成形门控
        center = (self.form_anchor if self.form_anchor is not None
                  else self._current_center())
        if center is None:
            return
        fresh = [i for i, (xy, t) in self.poses.items() if now - t <= 3.0]
        alts_ok = all(alts.get(i, -100.0) >= 0.9 * self.default_z
                      for i in range(self.n_drones))
        slot_errs = []
        for i in range(self.n_drones):
            xy = self.poses.get(i, (None, None))[0]
            if xy is None:
                slot_errs.append(None)
                continue
            sl = self.slots[i]
            slot_errs.append(math.hypot(xy[0] - (center[0] + sl[0]),
                                        xy[1] - (center[1] + sl[1])))
        formed = (len(fresh) == self.n_drones and alts_ok
                  and all(e is not None and e < 1.5 for e in slot_errs))
        if formed:
            if self.form_started is None:
                self.form_started = now
            elif now - self.form_started >= self.form_hold_s:
                self.get_logger().info(
                    '★ 编队成形稳定(preset, 槽位误差 %s), 开始沿 ICC 轨迹飞行'
                    % [round(e, 2) for e in slot_errs])
                self.form_started = None
                self._mission_started = True
                self.form_anchor = None
                self._publish_start()
        else:
            self.form_started = None
            if now - getattr(self, '_pf_diag_t', 0.0) > 10.0:
                self._pf_diag_t = now
                self.get_logger().info(
                    'preset 成形中: 在线 %d/%d, 高度 %s, 槽位误差 %s '
                    '(需全员 >=%.0fm 且误差 <1.5m 持续 %.0fs)'
                    % (len(fresh), self.n_drones,
                       [round(alts.get(i, -100.0), 1) for i in range(self.n_drones)],
                       [('NA' if e is None else round(e, 1)) for e in slot_errs],
                       0.9 * self.default_z, self.form_hold_s))

    def _goal_tick(self):
        """授权后逐点发 /goal; 中心到位(水平 < goal_radius)自动切下一点。"""
        with self.goals_lock:
            if not (self.mission_mode and self.gated and self.goals):
                return
            if self.goal_idx >= len(self.goals):
                return                        # 全部目标已发完(飞完由地面站 revoke)
            now = time.time()
            # ★ §8.26(BUG-1 修复): 起飞指令周期重发 —— 授权瞬间 FC2~N 可能
            #   尚未完成 GUIDED 切换(逐架解锁离散 0~10s), 单发 NAV_TAKEOFF
            #   会被 ArduPilot 拒绝且无人再问 → 五机死锁(1号在槽位, 其余趴地,
            #   2026-09-10 实测)。凡高度未到门控者, 每 3s 重发一次。
            if now - getattr(self, '_tk_resend_t', 0.0) >= 3.0:
                self._tk_resend_t = now
                alts_r = getattr(self, '_alts', {})
                for i, ch in enumerate(self.mav_chs):
                    try:
                        if alts_r.get(i, -100.0) < 0.9 * self.default_z:
                            ch._send(self.mav_chs[0].mav.command_long_encode(
                                i + 1, 1, 22, 0, 0, 0, 0, 0, 0, 0,
                                float(self.default_z)))
                            self.get_logger().warn(
                                '★ FC%d 高度 %.1fm < 门控 %.1fm, 重发 NAV_TAKEOFF'
                                % (i + 1, alts_r.get(i, -100.0),
                                   0.9 * self.default_z))
                    except Exception as e:
                        self.get_logger().error('FC%d 起飞重发异常: %s' % (i + 1, e))
            if not self._goals_sent and self.goal_idx == 0:
                # ★ 成形检查(★ 仅第一个目标): 全员在线+巡航高度+槽位误差<1.5m
                #   持续 2s。后续目标不检查 —— 拐角处编队在旧朝向停稳, 新朝向的
                #   槽位误差恒 >1.5m, 若检查会与"收到目标才转向"互相等死(实测)。
                center = self.form_anchor if (
                    self.mission_mode and self.form_anchor is not None
                ) else self._current_center()
                now_t = time.time()
                fresh = [i for i, (xy, t) in self.poses.items() if now_t - t <= 3.0]
                alts = getattr(self, '_alts', {})
                alts_ok = all(alts.get(i, -100.0) >= 0.9 * self.default_z
                              for i in range(self.n_drones))
                slot_errs = []
                # ★ 槽位按"指向第一目标"的航向旋转后再算误差(原始槽位表是
                #   东向飞行布局, 实际第一段航向不同 → 误差恒 10m 死锁)
                gx0, gy0, _ = self.goals[0]
                hd0 = math.atan2(gy0 - center[1], gx0 - center[0])
                # ★ §8.14 成形防对穿: 首拍(3/3 在线)按"最近槽位"指派并下发 ——
                #   起飞排列与本班航向不匹配时, 按机号固定槽位会让两机对穿换位
                #   (实测 1/3 号机对穿, 合拢 ~12 m/s)。指派全飞行固定, 桥与
                #   egompc 按同一确定性算法各自算出相同排列。
                if self._slot_perm is None and len(fresh) == self.n_drones:
                    rot_slots = []
                    for sl in self.slots:
                        sx, sy = rotate_slot((sl[0], sl[1], 0.0), hd0)
                        rot_slots.append((center[0] + sx, center[1] + sy))
                    poss = [self.poses[i][0] for i in range(self.n_drones)]
                    self._slot_perm = assign_slots(rot_slots, poss)
                    msg = Int32MultiArray()
                    msg.data = [int(x) for x in self._slot_perm]
                    self.slot_assign_pub.publish(msg)
                    self.get_logger().info(
                        '★ 槽位指派(防对穿): %s' % (self._slot_perm,))
                perm = self._slot_perm or list(range(self.n_drones))
                for i in range(self.n_drones):
                    xy = self.poses.get(i, (None, None))[0]
                    if xy is None:
                        slot_errs.append(None)
                        continue
                    sl = self.slots[perm[i]]
                    sx, sy = rotate_slot((sl[0], sl[1], 0.0), hd0)
                    slot_errs.append(math.hypot(xy[0] - (center[0] + sx),
                                                xy[1] - (center[1] + sy)))
                formed = (len(fresh) == self.n_drones and alts_ok
                          and all(e is not None and e < 1.5 for e in slot_errs))
                if formed:
                    if self.form_started is None:
                        self.form_started = now_t
                    elif now_t - self.form_started >= self.form_hold_s:
                        self.get_logger().info(
                            '★ 编队成形稳定(槽位误差 %s), 开始航线'
                            % [round(e, 2) for e in slot_errs])
                        self._set_leg_heading(self.goal_idx, center)
                        self._publish_goal(self.goals[self.goal_idx])
                        self._goals_sent = True
                        self._last_goal_send = now_t
                        self.form_started = None
                        self._mission_started = True
                        self.form_anchor = None   # 锚点使命完成, odom 切实时均值
                        self._publish_start()     # ★ ICC: 成形稳定 → 开始沿轨迹飞行
                    return
                self.form_started = None
                if now_t - getattr(self, '_form_diag_t', 0) > 10.0:
                    self._form_diag_t = now_t
                    self.get_logger().info(
                        '成形中: 在线 %d/%d, 高度 %s, 槽位误差 %s '
                        '(需全员 >=%.0fm 且误差 <1.5m 稳定 2s)'
                        % (len(fresh), self.n_drones,
                           [round(alts.get(i, -100.0), 1)
                            for i in range(self.n_drones)],
                           [('NA' if e is None else round(e, 1)) for e in slot_errs],
                           0.9 * self.default_z))
                    # §8.26(BUG-2/3): 成形超 60s 未成 → 每 30s 升级告警,
                    #   点名"谁趴在地面没起飞"(起飞指令可能早于 GUIDED 被拒)
                    ft = now_t - getattr(self, '_form_t0', now_t)
                    ground = ['FC%d' % (i + 1) for i in range(self.n_drones)
                              if alts.get(i, -100.0) < 0.9 * self.default_z]
                    if ft > 60 and ground and \
                            now_t - getattr(self, '_form_esc_t', 0) > 30:
                        self._form_esc_t = now_t
                        self.get_logger().warn(
                            '★ 成形卡住 %.0fs: %s 仍在地面未起飞 —— 起飞指令'
                            '可能早于 GUIDED 被拒(正周期重发中); 若再等 2 分钟'
                            '仍不起, 请人工检查该机 PreArm/模式' % (ft, ground))
                return
            # ★ 到位判定(差速车语义, §8.5): 中心到位【且】中心轨迹流已断档
            #   (= 已在角点减速停稳) 才推进下一段 —— 停稳后新轨迹的 minjerk 慢
            #   启动让编队原地旋转到新航向再平移(四旋翼可原地转, 绕中心各扫
            #   10m 弧, 机间距恒定 20m 无重叠); 移动中推进会让机头突变与平移
            #   叠加 → 参考轨迹互相扫过 → 重叠相撞(实测转向撕裂根源)。
            center = self._current_center()
            stream_stale = (now - self._last_cmd_t) > 1.5
            within = False
            if center is not None:
                gx, gy, _ = self.goals[self.goal_idx]
                within = math.hypot(center[0] - gx, center[1] - gy) < self.goal_radius
                if within and stream_stale:
                    self.goal_idx += 1
                    self._goals_sent = False
                    if self.goal_idx < len(self.goals):
                        self._set_leg_heading(self.goal_idx, center)
                        self.get_logger().info(
                            '★ 中心停稳于目标 %d/%d → 原地转向后前往 %s'
                            % (self.goal_idx, len(self.goals),
                               tuple(round(v, 1) for v in self.goals[self.goal_idx])))
                    else:
                        self.get_logger().info('★ 全部目标已送达, 等地面站指令(revoke/land)')
                    return
            # ★ 目标早于 odom 会被规划器静默丢弃(waypointCallback 无 odom 直接
            #   return) → 未到位且流断档 = 目标未被接受, 每 5s 重发(+trigger)。
            #   (到位但未停稳的情形在上面已 return 等待停稳)
            if not within and now - self._last_cmd_t > 2.0:
                if now - self._last_goal_send >= 5.0:
                    self._publish_goal(self.goals[self.goal_idx])
                    self._last_goal_send = now

    def _set_leg_heading(self, idx, center):
        """当前航段方位角 = goal[idx-1]→goal[idx](第一段 = 中心→goal0), 发布
        /formation/heading; egompc 以此做槽位旋转(根源替代轨迹切线反推)。"""
        gx, gy, _ = self.goals[idx]
        prev = self.goals[idx - 1][:2] if idx >= 1 else center[:2]
        hd = math.atan2(gy - prev[1], gx - prev[0])
        self._leg_heading = hd
        self.heading_pub.publish(Float32(data=hd))

    def _publish_goal(self, g):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.pose.position.x = float(g[0])
        m.pose.position.y = float(g[1])
        m.pose.position.z = float(g[2])
        m.pose.orientation.w = 1.0
        self.goal_pub.publish(m)
        # ★ 联发 /traj_start_trigger: FSM 每段轨迹完成后 have_trigger_=false,
        #   WAIT_TARGET 需要 trigger 才会进入下一段规划(triggerCallback 置位)
        self.trigger_pub.publish(m)
        self.get_logger().info('★ /goal + trigger → (%.1f, %.1f, %.1f)'
                               % (g[0], g[1], g[2]))

    def _adopt_route_z(self, pts):
        """★ 地面站航点高度 → 桥 default_z(对齐 v13: NAV_TAKEOFF/成形高度检查
        对齐到航点高度, 中心线改高度才真正起飞到新高度)。

        仅授权前生效: 起飞指令在门控开启瞬间发出, 之后改航点不重发起飞;
        起飞后新高度由 egompc 的设定点(fly_z 随航线 z)负责带到。"""
        if self.gated:
            return
        zs = [float(w[2]) for w in pts if len(w) > 2
              and math.isfinite(float(w[2])) and float(w[2]) > 0.1]
        if not zs:
            return
        new_z = max(zs)
        if abs(new_z - self.default_z) > 0.05:
            old = self.default_z
            self.default_z = new_z
            self.get_logger().info(
                '★ 航点高度 %.1f → %.1f m (NAV_TAKEOFF/成形门控对齐)'
                % (old, new_z))

    def _publish_route(self):
        """向 egompc 下发整条航点(/formation/route), 供其重建解析 ICC 轨迹。

        ★ 阶段式启动(§8.24, 2026-09-09): 先把当前编队中心 prepend 为 ICC 起点
        —— 否则 ICC 从 route[0](画线首点)起轨, start 瞬间五机参考点整体跳变
        一个航点距(实测"槽位突然变换"/起步紊乱)。prepend 后 ICC(0)=当前位置:
        授权 → 起飞 → 对准槽位(确认) → 起飞沿航线, 参考全程连续零跳变。"""
        with self.goals_lock:
            pts = list(self.goals)
        if len(pts) < 2:
            return
        c0 = self._current_center()
        if c0 is not None and math.hypot(c0[0] - pts[0][0],
                                         c0[1] - pts[0][1]) > 0.5:
            pts = [(c0[0], c0[1], self.default_z)] + pts
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'world'
        for g in pts:
            p = Pose()
            p.position.x = float(g[0])
            p.position.y = float(g[1])
            p.position.z = float(g[2])
            pa.poses.append(p)
        self.route_pub.publish(pa)

    def _publish_start(self):
        """起飞/成形完成信号(/formation/start), egompc 收到才沿 ICC 轨迹推进。"""
        self.start_pub.publish(Bool(data=True))


def main():
    ap = argparse.ArgumentParser(
        description='MAVROS → EGO-planner 环境桥(均值中心 odom + 空点云 + 上传航线逐点 goal)')
    ap.add_argument('--uav-index', type=int, default=0,
                    help='EGO 规划器命名空间下标(默认 0, 仅用于 cloud 话题)')
    ap.add_argument('--n-drones', type=int, default=3, help='编队总机数(默认 3)')
    ap.add_argument('--formation', default='line', choices=['line', 'column'],
                    help='队形(默认 line 横队)')
    ap.add_argument('--spacing', type=float, default=10.0, help='槽位间距 m(默认 10)')
    ap.add_argument('--odom-hz', type=float, default=30.0, help='odom 发布频率(默认 30Hz)')
    ap.add_argument('--mission-goals', action='store_true',
                    help='地面站上传航线模式: PullMission(FC1) → 逐点 /goal(规划器须 flight_type=1)')
    ap.add_argument('--goal-radius', type=float, default=3.0, help='目标到位半径 m(默认 3)')
    ap.add_argument('--z', type=float, default=10.0, help='航点缺省高度 m(默认 10)')
    ap.add_argument('--pull-interval', type=float, default=5.0, help='航线拉取周期 s(默认 5)')
    ap.add_argument('--home-spacing', type=float, default=10.0,
                    help='相邻机 home 东向间距 m(与 start_all.sh HOME_SPACING 一致)')
    args = ap.parse_args()
    rclpy.init()
    node = EgoEnvBridge(args.uav_index, n_drones=args.n_drones,
                        formation=args.formation, spacing=args.spacing,
                        mission_goals=args.mission_goals,
                        goal_radius=args.goal_radius, default_z=args.z,
                        pull_interval=args.pull_interval,
                        home_spacing=args.home_spacing)
    node.create_timer(1.0 / args.odom_hz, node._pub_odom_tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._stop = True
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

