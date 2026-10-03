#!/usr/bin/env python3
"""uav_formation 编队节点 —— 订阅 /uav{i}/mavros 的集中式编队控制(含地面站指令通道)。

特点:
  * 直接订阅现有 MAVROS 命名空间 /uav{i}/mavros (i = 0 .. n-1), 不依赖其它节点;
  * 固定队形(无自动变形): line/column/wedge/grid/circle, 但**地面站可随时切换**;
  * 机数可配: --n-drones 3 .. 10, 槽位按机数自动生成;
  * 两种飞行模式:
      - 默认模式(v1): 起飞 → 虚拟中心+槽位 保持/沿 +x 巡航 → 降落;
      - --mission 任务模式: 地面站(MAVLink 官方消息)发布航点与编队方法,
        虚拟中心依次飞向航点; **飞行途中可追加/修改/删除航点、切换队形**;
  * 不含避障(纯设定点控制)。

坐标系:
  * --mode plain (默认): 公共系 = local + home 偏移(相对基准机 uav0 的 home);
  * --mode gazebo:       local 直接是公共系(所有机同一 home / Gazebo JSON 模式)。

用法(需 N 套 /uav{i}/mavros 已在跑):
  source /opt/ros/humble/setup.bash
  python3 uav_formation/formation_node.py --check                  # 只订阅打印
  python3 uav_formation/formation_node.py --shape line --hold-s 30 # v1 保持
  python3 uav_formation/formation_node.py --mission --auto-start   # 任务模式(自动起飞)
  python3 uav_formation/formation_node.py --mission                # 任务模式(等地面站 start)
"""
import argparse
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition, State
from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from uav_formation.control import FormationPhase, WaypointAdvancer
from uav_formation.geometry import formation_rms, make_slots, rotate_slot
from uav_formation.gcs_channel import GcsChannel, MavlinkChannel

M_PER_DEG_LAT = 110540.0
SP_HZ = 20.0
MAX_N = 10

# ★ QoS 必须与 mavros 发布端完全匹配(实测坑):
#   /uav{i}/mavros/state             → RELIABLE + TRANSIENT_LOCAL
#   /uav{i}/mavros/local_position/pose → BEST_EFFORT + VOLATILE
#   /uav{i}/mavros/home_position/home → RELIABLE + TRANSIENT_LOCAL
# 用 VOLATILE 订阅 state 会收不到(ros2 topic echo 默认 QoS 就是这个坑, 一直空转)。
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
HOME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def enu_offset(lat0, lon0, lat1, lon1):
    dy = (lat1 - lat0) * M_PER_DEG_LAT
    dx = (lon1 - lon0) * M_PER_DEG_LAT * math.cos(math.radians(lat0))
    return dx, dy


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class UavLink:
    """一架机的 MAVROS 接口 + 公共系坐标对齐。"""

    def __init__(self, node, idx, prefix, mode):
        self.idx = idx
        self.ns = f"/{prefix}{idx}/mavros"
        self.mode = mode
        self.node = node
        self.state = None
        self.local = None
        self.home = None

        node.create_subscription(State, f"{self.ns}/state", self._on_state, STATE_QOS)
        node.create_subscription(PoseStamped, f"{self.ns}/local_position/pose",
                                 self._on_local, SENSOR_QOS)
        node.create_subscription(HomePosition, f"{self.ns}/home_position/home",
                                 self._on_home, HOME_QOS)
        self.sp_pub = node.create_publisher(
            PoseStamped, f"{self.ns}/setpoint_position/local", 10)
        self.cli_mode = node.create_client(SetMode, f"{self.ns}/set_mode")
        self.cli_arm = node.create_client(CommandBool, f"{self.ns}/cmd/arming")
        self.cli_takeoff = node.create_client(CommandTOL, f"{self.ns}/cmd/takeoff")
        self.cli_land = node.create_client(CommandTOL, f"{self.ns}/cmd/land")

    # ---- 回调 ----
    def _on_state(self, m):
        self.state = m

    def _on_local(self, m):
        self.local = m.pose.position

    def _on_home(self, m):
        self.home = m.geo

    # ---- 坐标 ----
    def aligned_xy(self, ref_home):
        """公共系坐标 (x, y)。plain = local + home 偏移; gazebo = local 直接用。"""
        if self.local is None:
            return None
        if self.mode == "gazebo":
            return (self.local.x, self.local.y)
        if self.home is None or ref_home is None:
            return None
        ox, oy = enu_offset(ref_home.latitude, ref_home.longitude,
                            self.home.latitude, self.home.longitude)
        return (self.local.x + ox, self.local.y + oy)

    def to_local(self, tx, ty, ref_home):
        """公共系目标 → 本机 local 系(下发 setpoint 用)。
        真 SITL 多机 home 沿东错开时(如 start_all.sh HOME_SPACING=10m), 飞控的
        local 原点=自己 home, 必须把公共系目标减掉本机 home 偏移, 否则各机
        飞偏一个 home 间距(实测 2026-08: uav1/uav2 偏 10/20m)。"""
        if self.mode == "gazebo" or self.home is None or ref_home is None:
            return tx, ty
        ox, oy = enu_offset(ref_home.latitude, ref_home.longitude,
                            self.home.latitude, self.home.longitude)
        return tx - ox, ty - oy

    def common_z(self):
        return self.local.z if self.local is not None else None

    # ---- 服务 ----
    def call(self, client, req, name, retries=3, timeout=30.0):
        for attempt in range(retries):
            if client.wait_for_service(timeout_sec=10.0):
                fut = client.call_async(req)
                rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
                res = fut.result()
                if res is not None:
                    ok = getattr(res, "success", True)
                    print(f"  [uav{self.idx}] {name} -> {'OK' if ok else res}")
                    return res
                print(f"  [uav{self.idx}] {name} 超时, 重试 {attempt + 1}")
            else:
                print(f"  [uav{self.idx}] {name} 服务不在, 重试 {attempt + 1}")
        return None

    def set_guided(self):
        for _ in range(5):
            self.call(self.cli_mode,
                      SetMode.Request(base_mode=0, custom_mode="GUIDED"), "set_mode")
            if self.state is not None and self.state.mode == "GUIDED":
                return True
            time.sleep(2.0)
        return self.state is not None and self.state.mode == "GUIDED"

    def arm(self):
        for attempt in range(10):
            self.call(self.cli_arm, CommandBool.Request(value=True), "cmd/arming")
            if self.state is not None and self.state.armed:
                return True
            time.sleep(2.0)      # 等状态位同步; EKF 未收敛时会被 PreArm 顶回
        return self.state is not None and self.state.armed

    def takeoff_to(self, z):
        for attempt in range(3):
            self.call(self.cli_takeoff, CommandTOL.Request(altitude=float(z)), "cmd/takeoff")
            t0 = time.time()
            while time.time() - t0 < 60:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                if self.local is not None and self.local.z >= z * 0.9:
                    return True
                if self.state is not None and not self.state.armed:
                    break          # 自动上锁(DISARM_DELAY)→ 重新解锁重试
            if self.state is not None and not self.state.armed:
                print(f"  [uav{self.idx}] 起飞期间自动上锁, 重新解锁")
                self.arm()
        return self.local is not None and self.local.z >= z * 0.9

    def land(self):
        self.call(self.cli_land, CommandTOL.Request(), "cmd/land")
        t0 = time.time()
        while time.time() - t0 < 120:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if self.state is not None and not self.state.armed:
                return True
        return False

    def pub_setpoint(self, x, y, z, yaw):
        sp = PoseStamped()
        sp.header.stamp = self.node.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = float(x)
        sp.pose.position.y = float(y)
        sp.pose.position.z = float(z)
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        sp.pose.orientation.x, sp.pose.orientation.y = qx, qy
        sp.pose.orientation.z, sp.pose.orientation.w = qz, qw
        self.sp_pub.publish(sp)


class UavFormationNode(Node):
    def __init__(self, args):
        super().__init__("uav_formation_node")
        self.args = args
        self.n = args.n_drones
        self.prefix = args.prefix
        self.mode = args.mode
        self.fly_z = float(args.fly_z)
        self.v_des = float(args.v_des)
        self.hold_s = float(args.hold_s)
        self.cruise_m = float(args.cruise_m)
        self.yaw = math.radians(float(args.yaw_deg))
        # ★ EGO/MPC 风格动态朝向(集中式航点模式):
        #   yaw_mode=heading → 中心朝向=运动方向, 槽位随朝向旋转(刚体转弯), 机头朝前
        #   yaw_mode=fixed   → 旧行为(固定朝向平移)
        self.yaw_mode = args.yaw_mode
        self.yaw_rate_deg = float(args.yaw_rate_deg)   # 转向限速 °/s
        self.yaw_dyn = self.yaw
        self.ref_home = None
        # ---- 起飞后阶段机 / 航点推进（docs/28 §6.2/§6.3, docs/27 §3.4）----
        self.phase = FormationPhase(
            judge_dist=float(args.formation_judge_dist),
            judge_time=float(args.formation_judge_time),
            timeout=float(args.formation_timeout),
            hold_s=float(args.hold_s), sp_hz=SP_HZ,
            n_expected=self.n)   # 集中式: 全部机误差齐全才判成型(规范#12)
        self.arrival_radius = float(args.arrival_radius)
        self.advancer = WaypointAdvancer([], self.arrival_radius, self.v_des,
                                         1.0 / SP_HZ, True)
        self.mav_ch = None                # ★ 经飞控转发的编队指令通道（主链路）
        self.authorized = False           # 全部机 GUIDED = 授权（docs/28 §5）

        # 队形(可被地面站 COMMAND_LONG 40010 动态切换)
        self.lock = threading.Lock()
        self.shape = args.shape
        self.spacing = float(args.spacing)
        self.radius = float(args.radius)
        self.slots = make_slots(self.shape, self.n, self.spacing, self.radius)

        # 任务模式状态(地面站线程写, 控制主循环读)
        self.waypoints = []          # [[x, y], ...] 公共系 ENU
        self.gcs_started = False     # NAV_TAKEOFF 或 --auto-start
        self.land_requested = False  # NAV_LAND
        self.paused = False          # MAV_CMD 40001/40002
        self.gcs = None
        self.gcs_connected = False
        self.leader_link = None          # 主链路命令源(地面站→主无人机链路)
        self.leader_link_active = False
        self.leader_mode = getattr(self.args, "leader_mode", "relay")
        # leader_mode: relay=节点统一指挥全部(主无人机只作中继); follow=主无人机由地面站飞, 编队跟随

        self.drones = [UavLink(self, i, self.prefix, self.mode)
                       for i in range(self.n)]
        print(f"[cfg] {self.n} 架 ({self.prefix}0..{self.prefix}{self.n - 1}) "
              f"队形={self.shape} 间距={self.spacing}m 高={self.fly_z}m "
              f"巡航={self.cruise_m}m 保持={self.hold_s}s mode={self.mode}")
        print(f"[cfg] 槽位: {[(round(s[0],2), round(s[1],2)) for s in self.slots]}")

    # ================= 地面站回调(GCS 线程调用) =================
    def on_gcs_waypoints(self, wps):
        """标准航点协议收齐后回调: 整体替换航点表。返回 (accepted, reason)。
        航点格式 [(x, y[, z])] —— z 保留(2026-08-28, 与 drone_node 一致)。"""
        pts = []
        for wp in wps:
            x, y = float(wp[0]), float(wp[1])
            z = float(wp[2]) if len(wp) > 2 else 0.0
            if abs(x) > self.gcs_bounds() or abs(y) > self.gcs_bounds():
                return (0, 2)
            pts.append([x, y, z])
        with self.lock:
            self.waypoints = pts
            summary = [(round(w[0], 1), round(w[1], 1)) for w in self.waypoints]
        self.advancer.set_waypoints(pts)   # ★ 更新航点推进器(docs/28 §6.3)
        if self._leader_follow():
            # ★ 跟随模式: 航点应交给【主无人机飞控】执行(飞控自己飞), 编队节点
            #   不按这些航点驱动中心, 只把主无人机实时位置当刚体中心, 从机围绕跟随。
            print(f"[gcs] 航点已记录(共 {len(self.waypoints)} 个 {summary}), "
                  "跟随模式: 由主无人机飞控执行, 编队围绕主无人机")
        else:
            # ★ relay/中继模式: 该航点经主链路(14559)由地面站发给主无人机飞控
            #   (sysid=1) 转发而来, 语义是【编队几何中心】, 不是某架机的本机航点;
            #   各机目标 = 中心 + 队形槽位, 由节点经 MAVROS setpoint 逐机下发。
            #   (飞控在 GUIDED 模式只存航点不执行, 不会把中心航点当本机任务飞)
            print(f"[gcs] ★ 中心航点(编队几何中心, 非单机航点): "
                  f"{len(self.waypoints)} 个 {summary}, 各机目标=中心+槽位")
        return (1, 0)

    def on_gcs_command(self, action):
        """COMMAND_LONG: 22=start 21=land 40001=hold 40002=resume"""
        if action == 0:
            self.gcs_started = True
            print("[gcs] ★ 收到 start, 开始任务")
        elif action == 1:
            self.land_requested = True
            print("[gcs] ★ 收到 land, 准备降落")
        elif action == 2:
            self.paused = True
            print("[gcs] hold: 暂停航点推进")
        elif action == 3:
            self.paused = False
            print("[gcs] resume: 继续航点推进")

    def on_gcs_shape(self, shape, spacing, radius):
        """COMMAND_LONG 40010: 切换编队方法(飞行途中即时生效)。"""
        if shape is None:
            print("[gcs] 未知队形, 忽略")
            return
        with self.lock:
            self.shape = shape
            if spacing > 0:
                self.spacing = float(spacing)
            if radius > 0:
                self.radius = float(radius)
            self.slots = make_slots(self.shape, self.n,
                                    self.spacing, self.radius)
        print(f"[gcs] 编队方法已切换 -> {shape} 间距={self.spacing}m "
              f"槽位: {[(round(s[0],2), round(s[1],2)) for s in self.slots]}")

    def gcs_bounds(self):
        return 1000.0

    # ================= 阶段 =================
    def _leader_follow(self):
        """True=跟随模式: 主链路 + 主无人机由地面站直接飞, 编队围绕它跟随。"""
        return self.leader_link_active and self.leader_mode == "follow"

    def start_gcs(self, port):
        if port <= 0:
            print("[gcs] --gcs-port 0: 直连通道已关闭, 全部命令经主链路(14559)监听主1")
            return
        self.gcs = GcsChannel(self, port, self.args.ref_lat, self.args.ref_lon,
                              node_sysid=self.args.node_sysid)
        self.gcs.start()

    def start_mavlink_channel(self):
        """★ 主链路: 经飞控转发的编队指令通道(docs/28 §4.3)。

        本节点为集中式单节点, 监听主无人机(uav0)的 MAVROS 透传口:
          地面站 target=99 → MAVProxy → serial0 → 飞控转发 → serial1
          → /uas1/mavlink_source → 本节点(MISSION/40010)。
        ★★ 话题基址 = /uas{tgt_system}（uav0 的 tgt_system=1 → /uas1）,
          本版 MAVROS 原始透传话题是全局绝对路径(联调实测, 见 gcs_channel.py docstring)。
        节点以 sysid=99 心跳注册路由(规范#4), 回执经 mavlink_sink 反向。
        """
        self.mav_ch = MavlinkChannel(self, "/uas1", self.args.ref_lat,
                                     self.args.ref_lon,
                                     node_sysid=self.args.node_sysid)
        print(f"[mavlink] 集中式主链路通道: /uas1 (编队指令经飞控转发, target=99)")

    def start_leader_link(self):
        """主链路命令源(路径A, 旧): 监听 MAVProxy 转发口, 被动读取地面站发给主无人机
        (sysid=--leader-sysid) 的 COMMAND_LONG, 触发编队动作。

        主无人机自身的起飞/降落由飞控直接执行地面站指令, 编队逻辑不再重复指挥它。
        实机: 该端口指向主无人机机载电脑收到的飞控 MAVLink 流(与仿真 MAVProxy 转发同构)。
        """
        port = int(self.args.leader_link_port)
        if port <= 0:
            return
        self.leader_link = GcsChannel(
            self, port, self.args.ref_lat, self.args.ref_lon,
            node_sysid=self.args.node_sysid, passive=True,
            accept_extra=(int(self.args.leader_sysid),))
        self.leader_link.start()
        self.leader_link_active = True
        print(f"[leader-link] ★ 主链路命令源: 监听 udp:{port}, "
              f"接受地面站→主无人机(sysid={self.args.leader_sysid}) 指令; "
              f"主无人机只作中继, 编队节点统一指挥全部机器(含主无人机)")

    def wait_connected(self, timeout=90.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if all(d.state is not None and d.state.connected for d in self.drones):
                print(f"[ok] {self.n} 架 MAVROS 全部 connected")
                return True
            rclpy.spin_once(self, timeout_sec=0.5)
        miss = [i for i, d in enumerate(self.drones)
                if d.state is None or not d.state.connected]
        print(f"[!!] 等 connected 超时, 未连上: uav{miss}")
        return False

    def request_streams(self):
        """请求飞控下发 LOCAL_POSITION_NED(20Hz)。

        ★ 新版 mavros(2026, 源码编译版)不自动请求流 —— local_position/pose 永远没数据,
          编队节点等 EKF 位置会一直超时(实测坑)。必须主动调 set_message_interval。
          模拟器(experiment_sim.py)没有该服务, 会自动跳过。
        """
        for d in self.drones:
            cli = self.create_client(MessageInterval, f"{d.ns}/set_message_interval")
            if cli.wait_for_service(timeout_sec=2.0):
                d.call(cli, MessageInterval.Request(message_id=32, message_rate=20.0),
                       "set_interval LOCAL_POSITION_NED", retries=1, timeout=10.0)
            else:
                print(f"  [uav{d.idx}] set_message_interval 服务不在(跳过)")

    def takeoff_all(self, wait_guided_timeout=180.0):
        """等地面站授权(每机 mode==GUIDED 且 armed, docs/28 §6.1 八步)后起飞。

        ★ 本节点【不调用 set_mode / 不解锁】(docs/28 §5.3 硬保护: 只有地面站发
          SET_MODE; 解锁也直发飞控)。只检测授权状态, 授权后自行 takeoff_to。"""
        if self.args.skip_takeoff:
            print("[..] --skip-takeoff: 假设已在空中, 跳过起飞")
            return True
        for d in self.drones:
            if self._leader_follow() and d.idx == 0:
                print(f"[..] uav{d.idx}: 主无人机(follow模式)由地面站经飞控直接起飞")
                continue
            # 冷启动: EKF 位置估计
            if d.local is None:
                print(f"[..] uav{d.idx}: 等 EKF 位置估计(冷启动 30~60s)...")
                t0 = time.time()
                while time.time() - t0 < 180 and d.local is None:
                    if self.land_requested:
                        print(f"[..] uav{d.idx}: 收到 land, 放弃起飞")
                        return False
                    rclpy.spin_once(self, timeout_sec=1.0)
                if d.local is None:
                    print(f"[!!] uav{d.idx} EKF 位置一直没就绪(180s)")
                    return False
            # ★ 等授权: GUIDED + armed（地面站步骤 6 解锁 / 步骤 7 授权起飞）
            wt = time.time()
            lt = 0.0
            while True:
                st = d.state
                if st is not None and st.mode == "GUIDED" and st.armed:
                    break
                if st is not None and time.time() - lt > 5.0:
                    lt = time.time()
                    if st.mode == "GUIDED" and not st.armed:
                        print(f"[..] uav{d.idx}: 已 GUIDED 但未解锁, 等地面站解锁(arm)...")
                    else:
                        print(f"[..] uav{d.idx}: 等地面站授权 GUIDED(当前 {st.mode})...")
                if self.land_requested or time.time() - wt > wait_guided_timeout:
                    print(f"[!!] uav{d.idx} 等授权超时")
                    return False
                rclpy.spin_once(self, timeout_sec=0.5)
            print(f"[..] uav{d.idx}: 已授权(GUIDED+armed), 起飞到 {self.fly_z}m")
            if not d.takeoff_to(self.fly_z):
                print(f"[!!] uav{d.idx} 起飞未达高度")
                return False
            print(f"[ok] uav{d.idx} 已到 {d.common_z():.1f}m")
        return True

    def land_all(self):
        print("[..] 逐架降落")
        for d in self.drones:
            if self._leader_follow() and d.idx == 0:
                print(f"[..] uav{d.idx}: 主无人机(follow模式)由地面站经飞控直接降落")
                continue
            if d.state is None or not d.state.armed:
                continue
            if d.land():
                print(f"[ok] uav{d.idx} 已落地上锁")
            else:
                print(f"[!!] uav{d.idx} 降落超时")

    def emergency_land(self, reason):
        print(f"[!] 异常({reason}), 兜底降落")
        for d in self.drones:
            if d.state is not None and d.state.armed:
                try:
                    d.land()
                except Exception as e:
                    print(f"[!!] uav{d.idx} 兜底降落失败: {e}")

    def _center_from_leader(self):
        for _ in range(50):
            if self.ref_home is None:          # 兜底: home 晚到时补取
                for d in self.drones:
                    if d.home is not None:
                        self.ref_home = d.home
                        break
            p = self.drones[0].aligned_xy(self.ref_home)
            if p is not None:
                return [float(p[0]), float(p[1])]
            rclpy.spin_once(self, timeout_sec=0.2)
        return None

    def _follower_slots(self):
        """跟随模式从机槽位: 主无人机占据队形'中心/顶点'位, 从机用其余槽位。

        不能直接 make_slots(n-1) 或 slots[i-1] —— 会让从机占用中心槽位,
        与主无人机重叠(实测 bug); 也不能整队平移, 否则线队形左右不对称。
        规则: 生成 n 架队形, 去掉距原点最近的槽(主无人机位), 其余按序给从机。
        """
        with self.lock:
            shape, spacing, radius = self.shape, self.spacing, self.radius
        full = make_slots(shape, self.n, spacing, radius)
        dist = [x * x + y * y for (x, y, _) in full]
        k = dist.index(min(dist))          # 主无人机位: 最靠中心/顶点的槽
        return [s for j, s in enumerate(full) if j != k]

    def _publish_targets(self, center, log=False, yaw=None):
        """按当前队形槽位给每架发设定点; 返回 (actuals, targets)。

        yaw=None → 用固定 self.yaw(旧行为);
        yaw=动态朝向(EGO/MPC 风格) → 槽位随朝向旋转(刚体转弯) + 每架机头朝运动方向。
        """
        if yaw is None:
            yaw = self.yaw
        with self.lock:
            slots = list(self.slots)
        follower_slots = self._follower_slots() if self._leader_follow() else None
        fi = 0
        actuals, targets = [], []
        for i, d in enumerate(self.drones):
            if self._leader_follow() and i == 0:
                # 跟随模式: 主无人机在中心(由地面站直接飞), 只读其位置, 不下发设定点
                p = d.aligned_xy(self.ref_home)
                if p is not None:
                    actuals.append(p)
                    targets.append(p)
                continue
            if follower_slots is not None:
                slot = follower_slots[fi]
                fi += 1
            else:
                slot = slots[i]
            tx, ty = rotate_slot(slot, yaw)
            if self._leader_follow() and tx * tx + ty * ty < 0.01:
                # 兜底: 从机槽位不得落在主无人机中心(如 line 奇数从机), 向后平移半格
                tx, ty = -self.spacing * 0.5, 0.0
            target = (center[0] + tx, center[1] + ty)
            targets.append(target)
            # ★ 公共系目标 → 本机 local 系(真 SITL 多机 home 错开时必需, 见 to_local)
            lx, ly = d.to_local(target[0], target[1], self.ref_home)
            d.pub_setpoint(lx, ly, self.fly_z, yaw)
            p = d.aligned_xy(self.ref_home)
            if p is not None:
                actuals.append(p)
        return actuals, targets

    def _update_heading(self, tx, ty, dt, center):
        """EGO/MPC 风格朝向更新: 中心朝向 = 运动方向(atan2), 限速平滑过渡。

        - 槽位随中心朝向旋转 → 编队像刚体转弯(参考 bamboo_square_ws nmpc_ego_formation)
        - 每架机头(setpoint yaw) = 运动方向, 不再固定
        - 限速: --yaw-rate-deg °/s, 避免直角急转
        """
        if self.yaw_mode != "heading" or tx is None:
            return
        target = math.atan2(ty - center[1], tx - center[0])
        max_step = math.radians(self.yaw_rate_deg) * dt
        d = (target - self.yaw_dyn + math.pi) % (2.0 * math.pi) - math.pi
        self.yaw_dyn += max(-max_step, min(max_step, d))

    # ================= v1 模式: 保持/巡航 =================
    def run_formation_v1(self):
        if self._leader_follow():
            return self.run_follow_leader()      # ★ 跟随模式: 编队围绕主无人机

        center = self._center_from_leader()
        if center is None:
            print("[!!] 没有 uav0 位置, 无法确定虚拟中心")
            return None
        print(f"[..] 虚拟中心=({center[0]:.2f},{center[1]:.2f}) "
              f"队形 yaw={math.degrees(self.yaw):.0f}°")

        metrics = dict(rms_last=0.0, max_rms=0.0, rms_final=0.0,
                       min_spacing=1e9, gather_t=None, t=0.0)
        t_sim, dt = 0.0, 1.0 / SP_HZ
        cruise_total = 0.0
        t_real0 = time.time()
        log_t = 0.0
        print(f"[..] 编队保持 {self.hold_s}s"
              + (f" + 直线巡航 {self.cruise_m}m(v={self.v_des}m/s)"
                 if self.cruise_m > 0 else ""))

        while True:
            t_sim += dt
            if self.mav_ch is not None:
                self.mav_ch.tick(time.time())   # ★ 心跳注册
            if self.land_requested:
                print("[..] 收到 land, 结束编队")
                break
            if self.cruise_m > 0 and cruise_total < self.cruise_m:
                step = self.v_des * dt
                center[0] += step
                cruise_total += step
            actuals, targets = self._publish_targets(center)
            self._update_metrics(metrics, actuals, targets, t_sim)
            now = time.time()
            if now - log_t >= 2.0:
                log_t = now
                pos = " ".join(f"({p[0]:6.2f},{p[1]:6.2f})" for p in actuals)
                print(f"     t={t_sim:5.1f}s rms={metrics['rms_last']:5.2f}m "
                      f"中心=({center[0]:6.2f},{center[1]:6.2f}) 实际: {pos}")
            done = t_sim >= self.hold_s
            if self.cruise_m > 0:
                done = done and cruise_total >= self.cruise_m
            if done:
                metrics["rms_final"] = metrics["rms_last"]
                metrics["t"] = t_sim
                break
            if time.time() - t_real0 > self.hold_s + 600:
                print("[!!] 编队超时")
                break
            sleep = t_real0 + t_sim - time.time()
            if sleep > 0:
                time.sleep(sleep)
            for _ in range(10):          # 每 tick 排空最多 10 条回调, 避免 state/local 滞后堆积
                rclpy.spin_once(self, timeout_sec=0.0)
        return metrics

    # ================= 任务模式: 航点跟随 =================
    def run_mission(self):
        """等地面站授权(全部机 GUIDED) → 起飞 → 队形成型 → hold → 沿中心航线飞。

        ★ 控制权模型(docs/28 §5): 地面站是唯一 SET_MODE 权威。节点检测全部机
          mode==GUIDED 视为授权(docs/28 §6.1 步骤 7); 任一架离开 GUIDED →
          立即停发全部 setpoint, 等重新授权(规范#11)。
        ★ 起飞后自动流程(docs/28 §6.2): 升到设定高度 → 队形成型(0.3m 持续 1s,
          30s 超时) → 等 hold_s 秒 → 中心沿中心航线推进, 队形整体移动。
        """
        print("[mission] 等地面站授权: 全部机解锁 + setmode GUIDED (docs/28 §6.1 八步)...")
        t0 = time.time()
        while not self.args.auto_start:
            if self.mav_ch is not None:
                self.mav_ch.tick(time.time())   # ★ 心跳注册须在授权前持续(docs/28 §4.3)
            if self.land_requested:
                print("[mission] 未起飞就收到 land, 直接结束")
                return "landed"
            if all(d.state is not None and d.state.mode == "GUIDED"
                   for d in self.drones):
                print("[mission] ★ 全部机已授权(GUIDED)")
                break
            if time.time() - t0 > 3600:
                print("[!!] 等授权超时(1h)")
                return "timeout"
            rclpy.spin_once(self, timeout_sec=0.2)

        if not self.takeoff_all():
            print("[!!] 起飞失败")
            self.emergency_land("起飞失败")
            return "takeoff_fail"

        if self._leader_follow():
            return self.run_follow_leader()      # ★ 跟随模式: 编队围绕主无人机

        center = self._center_from_leader()
        if center is None:
            print("[!!] 没有 uav0 位置")
            self.land_all()            # ★ 起飞后拿不到中心, 必须降落, 不能让机队悬停
            return "no_pos"

        with self.lock:
            wps = [list(w) for w in self.waypoints]
        self.advancer.set_waypoints(wps)
        center = [float(center[0]), float(center[1])]
        print(f"[mission] ★ 开始编队: 中心=({center[0]:.2f},{center[1]:.2f}) "
              f"航点: {self._wp_summary()} 成型判据="
              f"{self.phase.judge_dist}m/{self.phase.judge_time}s, "
              f"超时={self.phase.timeout}s, hold={self.phase.hold_s}s")

        self.authorized = True
        self.phase.on_authorized(0.0)      # 已授权 → TAKEOFF
        self.phase.on_takeoff_done(0.0)    # takeoff_all 已完成 → 直接成型
        t_sim, dt = 0.0, 1.0 / SP_HZ
        t_real0 = time.time()
        log_t = 0.0
        metrics = dict(rms_last=0.0, max_rms=0.0, rms_final=0.0,
                       min_spacing=1e9, gather_t=None, t=0.0)
        self.yaw_dyn = self.yaw      # EGO/MPC 风格动态朝向: 初始=固定朝向, 之后跟随运动方向
        last_phase = None

        while not self.land_requested:
            t_sim += dt
            now = time.time()
            if self.mav_ch is not None:
                self.mav_ch.tick(now)          # ★ 心跳注册 + Mission 超时

            # ★ GUIDED 授权检查(规范#11): 任一架非 GUIDED → 收回控制
            guided_all = all(d.state is not None and d.state.mode == "GUIDED"
                             for d in self.drones)
            if not guided_all and self.authorized:
                self.authorized = False
                self.phase.on_deauthorized()
                print("[mission] ★ 控制收回(有机制非 GUIDED), 停止 setpoint, 等重新授权")
            elif guided_all and not self.authorized:
                self.authorized = True
                self.phase.on_authorized(t_sim)
                self.phase.on_takeoff_done(t_sim)
                print("[mission] ★ 重新授权(GUIDED), 恢复编队")

            phase = self.phase.phase
            if phase != last_phase:
                print(f"[mission] 阶段 -> {self.phase.name()}")
                last_phase = phase

            # ---- 各阶段执行 ----
            if phase in (FormationPhase.FORMING, FormationPhase.HOLD):
                # 成型/保持: 中心不动, 各机就位(判据见 docs/28 §6.2)
                errors = self._formation_errors(center)
                self.phase.update(errors, t_sim)
                actuals, targets = self._publish_targets(center, yaw=self.yaw_dyn)
            elif phase == FormationPhase.FLY:
                with self.lock:
                    wps = [list(w) for w in self.waypoints]
                    paused = self.paused
                # 中心沿中心航线推进(仅 FLY; docs/28 §6.3)
                if wps and not paused:
                    tgt_wp = self.advancer.current_target()
                    if tgt_wp is not None:
                        self._update_heading(tgt_wp[0], tgt_wp[1], dt, center)
                    _, event = self.advancer.step(center)
                    if isinstance(event, tuple) and event[0] == self.advancer.WP_ARRIVED:
                        print(f"[mission] 到达中心航点 {event[1]}, 前往 {event[2]}")
                    elif event == self.advancer.ALL_DONE:
                        print(f"[mission] ★ 全部中心航点已到达, 自动降落")
                        self.phase.on_all_done()
                        self.land_requested = True
                        break
                actuals, targets = self._publish_targets(center, yaw=self.yaw_dyn)
            elif phase == FormationPhase.HOVER_WAIT:
                # 成型超时/无航点: 悬停保持, 等地面站介入(LOITER/LAND 直发飞控)
                actuals, targets = self._publish_targets(center, yaw=self.yaw_dyn)
            elif phase == FormationPhase.DONE:
                break
            else:   # WAIT_AUTH: 不发 setpoint
                actuals, targets = [], []
            self._update_metrics(metrics, actuals, targets, t_sim)

            if not self.waypoints and phase in (FormationPhase.FORMING,
                                                FormationPhase.HOLD):
                self.phase.on_no_waypoints()

            now = time.time()
            if now - log_t >= 2.0:
                log_t = now
                pos = " ".join(f"({p[0]:5.1f},{p[1]:5.1f})" for p in actuals) if actuals else "(无)"
                tgt = (f"航点{min(self.advancer.wp_idx, max(len(self.waypoints)-1,0))}"
                       if self.waypoints else "无航点-悬停")
                print(f"     t={t_sim:5.1f}s [{self.phase.name():9s}] "
                      f"rms={metrics['rms_last']:5.2f}m 目标={tgt} "
                      f"中心=({center[0]:6.2f},{center[1]:6.2f}) 位置: {pos}")

            if time.time() - t_real0 > 3600:
                print("[!!] 任务超时(1h)")
                break
            sleep = t_real0 + t_sim - time.time()
            if sleep > 0:
                time.sleep(sleep)
            for _ in range(10):          # 每 tick 排空最多 10 条回调, 避免 state/local 滞后堆积
                rclpy.spin_once(self, timeout_sec=0.0)

        metrics["rms_final"] = metrics["rms_last"]
        metrics["t"] = t_sim
        self.land_all()
        self._print_metrics(metrics)
        return "done"

    def _formation_errors(self, center):
        """每机距自己目标槽位(公共系)的水平距离 {idx: err}（成型判据, docs/28 §6.2）。

        与 _publish_targets 相同的槽位规则(含动态朝向/follow 模式), 但不发设定点。
        """
        errors = {}
        with self.lock:
            slots = list(self.slots)
        follower_slots = self._follower_slots() if self._leader_follow() else None
        fi = 0
        yaw = self.yaw_dyn if self.yaw_mode == "heading" else self.yaw
        for i, d in enumerate(self.drones):
            if self._leader_follow() and i == 0:
                continue
            slot = follower_slots[fi] if follower_slots is not None else slots[i]
            if follower_slots is not None:
                fi += 1
            tx, ty = rotate_slot(slot, yaw)
            target = (center[0] + tx, center[1] + ty)
            p = d.aligned_xy(self.ref_home)
            if p is not None:
                errors[i] = math.hypot(p[0] - target[0], p[1] - target[1])
        return errors

    def run_follow_leader(self):
        """跟随模式(follow): 中心 = 主无人机实时位置, 从机保持队形。

        主无人机由地面站指令经其飞控直接飞行(编队不指挥它); 编队保证从机
        围绕主无人机按队形排布, 直到收到 land。
        """
        print("[leader-link] ★ 跟随模式: 编队围绕主无人机保持队形, 等地面站 land")
        t_real0 = time.time()
        log_t = 0.0
        metrics = dict(rms_last=0.0, max_rms=0.0, rms_final=0.0,
                       min_spacing=1e9, gather_t=None, t=0.0)
        dt = 1.0 / SP_HZ
        t_sim = 0.0
        while not self.land_requested:
            t_sim += dt
            if self.mav_ch is not None:
                self.mav_ch.tick(time.time())   # ★ 心跳注册
            p = self.drones[0].aligned_xy(self.ref_home)
            if p is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue
            center = [float(p[0]), float(p[1])]
            actuals, targets = self._publish_targets(center)
            self._update_metrics(metrics, actuals, targets, t_sim)
            now = time.time()
            if now - log_t >= 2.0:
                log_t = now
                pos = " ".join(f"({q[0]:5.1f},{q[1]:5.1f})" for q in actuals)
                print(f"     t={t_sim:5.1f}s rms={metrics['rms_last']:5.2f}m "
                      f"主无人机=({center[0]:6.2f},{center[1]:6.2f}) 位置: {pos}")
            if time.time() - t_real0 > 3600:
                print("[!!] 跟随超时(1h)")
                break
            sleep = t_real0 + t_sim - time.time()
            if sleep > 0:
                time.sleep(sleep)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.0)
        metrics["rms_final"] = metrics["rms_last"]
        metrics["t"] = t_sim
        self.land_all()
        self._print_metrics(metrics)
        return "done"

    def _wp_summary(self):
        with self.lock:
            return [(round(w[0], 1), round(w[1], 1)) for w in self.waypoints]

    def _update_metrics(self, metrics, actuals, targets, t_sim):
        if len(actuals) != self.n:
            return
        rms = formation_rms(actuals, targets)
        metrics["rms_last"] = rms      # 只保留最近值; 不用 list, 避免随任务时长无限堆积
        metrics["max_rms"] = max(metrics["max_rms"], rms)
        if metrics["gather_t"] is None and rms < 0.5:
            metrics["gather_t"] = t_sim
        for a in range(self.n):
            for b in range(a + 1, self.n):
                dg = math.hypot(actuals[a][0] - actuals[b][0],
                                actuals[a][1] - actuals[b][1])
                metrics["min_spacing"] = min(metrics["min_spacing"], dg)

    def _print_metrics(self, m):
        print("\n===== 编队指标 =====")
        print(f"  队形RMS 最大  : {m['max_rms']:.3f} m (要求 < 1.2)")
        print(f"  队形RMS 最终  : {m['rms_final']:.3f} m (要求 < 0.5)")
        print(f"  最小机间距    : {m['min_spacing']:.3f} m (要求 >= 0.7)")
        print(f"  聚集时间      : {m['gather_t'] if m['gather_t'] else float('nan'):.1f} s")
        print("===== 结束 =====")

    def persist(self):
        """任务结束后保持监听, 等地面站新指令; Ctrl-C 退出。"""
        print("\n[persist] 节点保持监听(地面站可再发航点/队形/命令), Ctrl-C 退出")
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass

    # ---------- 只检查模式 ----------
    def check(self, seconds=10.0):
        print(f"[check] 订阅 {self.n} 架 /uav{{i}}/mavros, 观察 {seconds:.0f}s...")
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.5)
            now = time.time()
            for d in self.drones:
                if d.state is not None and now - getattr(d, "_chk_t", 0) > 2.0:
                    d._chk_t = now
                    pos = (f"local=({d.local.x:6.2f},{d.local.y:6.2f},{d.local.z:5.2f})"
                           if d.local is not None else "local=None")
                    home = (f"home=({d.home.latitude},{d.home.longitude})"
                            if d.home is not None else "home=None")
                    print(f"  uav{d.idx}: connected={d.state.connected} "
                          f"mode={d.state.mode} armed={d.state.armed} {pos} {home}")
        n_ok = sum(1 for d in self.drones
                   if d.state is not None and d.state.connected)
        print(f"[check] 结束: {n_ok}/{self.n} 架 connected")
        return n_ok == self.n


def load_yaml_defaults(path):
    """读取 config/formation.yaml, 作为命令行参数的默认值(命令行可覆盖)。"""
    import yaml
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        print(f"[!] 未找到配置文件 {path}, 使用内置默认值")
        return {}


def main():
    cfg_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "config", "formation.yaml")
    cfg = load_yaml_defaults(cfg_path)

    ap = argparse.ArgumentParser(description="uav_formation 编队节点")
    ap.add_argument("--n-drones", type=int, default=cfg.get("n_drones", 3),
                    help=f"机数 2..{MAX_N}(默认取 config/formation.yaml)")
    ap.add_argument("--prefix", default=cfg.get("prefix", "uav"),
                    help="MAVROS 命名空间前缀(默认 uav → /uav0/mavros ...)")
    ap.add_argument("--mode", choices=["plain", "gazebo"],
                    default=cfg.get("mode", "plain"))
    ap.add_argument("--shape", choices=["line", "column", "wedge", "grid", "circle"],
                    default=cfg.get("shape", "line"),
                    help="初始固定队形(地面站 COMMAND_LONG 40010 可随时切换)")
    ap.add_argument("--spacing", type=float, default=cfg.get("spacing", 1.5),
                    help="队形间距 m")
    ap.add_argument("--radius", type=float, default=cfg.get("radius", 5.0),
                    help="circle 队形的半径 m")
    ap.add_argument("--fly-z", type=float, default=cfg.get("fly_z", 2.0),
                    help="巡航高度 m")
    ap.add_argument("--v-des", type=float, default=cfg.get("v_des", 1.0),
                    help="巡航速度 m/s")
    ap.add_argument("--hold-s", type=float, default=cfg.get("hold_s", 5.0),
                    help="成型后稳定等待秒数(默认 5s, docs/28 §8)")
    ap.add_argument("--arrival-radius", type=float,
                    default=cfg.get("arrival_radius", 1.0),
                    help="航点到达判定距离 m(docs/28 §6.3)")
    ap.add_argument("--formation-judge-dist", type=float,
                    default=cfg.get("formation_judge_dist", 0.3),
                    help="成型判据: 距目标槽位 < 该值 m(docs/28 §6.2)")
    ap.add_argument("--formation-judge-time", type=float,
                    default=cfg.get("formation_judge_time", 1.0),
                    help="成型判据: 持续秒数")
    ap.add_argument("--formation-timeout", type=float,
                    default=cfg.get("formation_timeout", 30.0),
                    help="成型超时秒数(超时悬停等地面站)")
    ap.add_argument("--cruise-m", type=float, default=cfg.get("cruise_m", 0.0),
                    help="v1 模式沿 +x 直线巡航距离 m(0=只悬停保持)")
    ap.add_argument("--yaw-deg", type=float, default=cfg.get("yaw_deg", 0.0),
                    help="队形朝向(度); yaw-mode=heading 时作为初始朝向")
    ap.add_argument("--yaw-mode", choices=["heading", "fixed"],
                    default=cfg.get("yaw_mode", "heading"),
                    help="heading=EGO/MPC风格动态朝向(中心朝运动方向, 槽位/机头随转, "
                         "默认); fixed=旧行为固定朝向平移")
    ap.add_argument("--yaw-rate-deg", type=float, default=cfg.get("yaw_rate_deg", 30.0),
                    help="动态朝向转向限速(°/s), 避免直角急转(默认 30)")
    ap.add_argument("--skip-takeoff", action="store_true",
                    help="跳过起飞(假设已在空中)")
    ap.add_argument("--check", action="store_true",
                    help="只订阅打印状态/位置, 不控制不飞行")
    ap.add_argument("--mission", action="store_true",
                    help="任务模式: 地面站官方 MAVLink 发航点/队形, 中心依次飞航点")
    ap.add_argument("--gcs-port", type=int, default=cfg.get("gcs_port", 14580),
                    help="地面站指令 UDP 监听端口(MAVProxy: output add udp:127.0.0.1:<port>)")
    ap.add_argument("--ref-lat", type=float, default=cfg.get("ref_lat", 30.893000),
                    help="地面站经纬→ENU 固定基线纬度")
    ap.add_argument("--ref-lon", type=float, default=cfg.get("ref_lon", 120.087000),
                    help="地面站经纬→ENU 固定基线经度")
    ap.add_argument("--node-sysid", type=int, default=cfg.get("node_sysid", 99),
                    help="编队节点在 MAVLink 里的 system id")
    ap.add_argument("--leader-link-port", type=int,
                    default=cfg.get("leader_link_port", 14559),
                    help="主链路命令源端口: 被动读地面站→主无人机的指令(0=关闭; 默认 14559)")
    ap.add_argument("--leader-sysid", type=int, default=cfg.get("leader_sysid", 1),
                    help="主无人机(指挥机)的 sysid(默认 1 = uav0)")
    ap.add_argument("--leader-mode", choices=["relay", "follow"],
                    default=cfg.get("leader_mode", "relay"),
                    help="relay=节点统一指挥全部(主无人机只作中继, 默认); "
                         "follow=主无人机由地面站直接飞, 编队围绕它跟随")
    ap.add_argument("--auto-start", action="store_true",
                    help="任务模式: 不等地地面站 start, 自动起飞开始(仍需航点)")
    args = ap.parse_args()

    if not (2 <= args.n_drones <= MAX_N):
        print(f"[!!] n_drones 需在 2..{MAX_N}")
        return 1

    rclpy.init()
    n = UavFormationNode(args)
    ok = False
    try:
        if args.check:
            ok = n.check()
            return 0 if ok else 1
        if not n.wait_connected():
            return 1
        n.request_streams()          # ★ 请求 LOCAL_POSITION_NED 流(新版 mavros 不自动请求)
        # 等 home(连接后 ~10s 由 home_position 插件轮询 GET_HOME_POSITION 拿到;
        # 太早取会一直是 None → plain 模式 aligned_xy 恒为 None → "没有 uav0 位置")
        t0 = time.time()
        while n.ref_home is None and time.time() - t0 < 60:
            for d in n.drones:
                if d.home is not None:
                    n.ref_home = d.home
                    break
            if n.ref_home is None:
                rclpy.spin_once(n, timeout_sec=0.5)
        if n.ref_home is not None:
            print(f"[ok] 基准 home: ({n.ref_home.latitude:.6f},{n.ref_home.longitude:.6f})")
        else:
            print("[!!] home 一直没到(60s), 公共系对齐可能不准")
        n.start_gcs(args.gcs_port)      # 地面站指令通道(14580, 可选; --gcs-port 0 关闭)
        n.start_leader_link()           # ★ 主链路命令源(地面站→主无人机链路, 路径A, 旧)
        n.start_mavlink_channel()       # ★ 主链路: 经飞控转发的编队指令通道(target=99)
        if args.mission:
            n.run_mission()             # 内部负责起飞/航点跟随/降落
            n.persist()                 # 降落后保持监听
        else:
            if not n.takeoff_all():
                print("[!!] 起飞失败, 降落已起飞的机")
                n.emergency_land("起飞失败")
                return 1
            m = n.run_formation_v1()
            n.land_all()
            if m:
                n._print_metrics(m)
                ok = m["rms_final"] < 0.5 and m["min_spacing"] >= 0.7
    except KeyboardInterrupt:
        print("\n[!] Ctrl-C")
        n.emergency_land("用户中断")
    except Exception as e:
        import traceback
        print(f"\n[!!] 异常: {e}")
        traceback.print_exc()
        n.emergency_land(str(e))
    finally:
        if n.gcs is not None:
            n.gcs.stop_channel()
        if n.mav_ch is not None:
            n.mav_ch.stop_channel()
        n.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
