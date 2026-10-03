#!/usr/bin/env python3
"""单机编队节点（分布式控制方法, docs/28 修订版）—— 每架机跑一个实例, 只管理自己。

架构（docs/28 §2/§4, docs/24 §4.0; ★ 2026-08-28 调整: 删除 sysid=99 通道）:
  * 主链路 = MavlinkChannel（/uas{1+idx} mavlink_source|sink）与【自家飞控】对话:
      发请求/收遥测; 中心航线由节点【主动从飞控拉取】(PullMission, 地面站经 MAVSDK
      逐架直发飞控 target=1..N 后, 每架节点拉自己的)。
  * 可选保留 UDP 直连测试通道（--gcs-port-base, 0=关, docs/28 §3 14560+idx 可选）。

控制（docs/28 §6.2/§6.3, docs/27 §3.4）:
  * 授权模型: 地面站是唯一 SET_MODE 权威（docs/28 §5）。本节点【不调用 set_mode】，
    只检测 /uav{i}/mavros/state.mode == "GUIDED" 视为授权；非 GUIDED 立即停发
    setpoint 并回到 WAIT_AUTH（规范#11, 硬保护 docs/28 §5.3）。
  * 起飞后自动流程: 升到 fly_z → 队形成型(0.3m 持续 1s, 30s 超时) → 等 hold_s 秒
    → 中心沿【中心航线】推进(航点=编队几何中心, 每机目标=中心+槽位)。
  * 分布式控制方法机间 DDS 避碰: /formation/positions 全机快照(公共系, 1Hz) + 死区/低通/
    限幅软避碰（docs/28 §7.2, docs/26）。
  * 中心航线/队形一致性: 各机从自家飞控拉到相同中心线(地面站逐架直发), 必要时
    经 /formation/mission、/formation/shape 广播同步（DDS 兜底）。

用法:
  source /opt/ros/humble/setup.bash && source ~/ros2_mavros_ws/install/setup.bash
  python3 uav_formation/drone_node.py --idx 0 --prefix uav --n-drones 3
  python3 uav_formation/drone_node.py --idx 1 --prefix uav --n-drones 3
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
from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from uav_formation.control import (AvoidanceOffset, FormationPhase, MissionSharer,
                                   PoseSharer, WaypointAdvancer)
from uav_formation.gcs_channel import (MavlinkChannel, GcsChannel, SHAPE_IDS,
                                       SHAPE_NAMES, MAV_CMD_FORMATION_SHAPE)
from uav_formation.geometry import make_slots, rotate_slot

SP_HZ = 20.0

# ★ QoS 必须与 mavros 发布端完全匹配(实测坑)
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=10)
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
HOME_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=10)

M_PER_DEG_LAT = 111319.49


def enu_offset(lat0, lon0, lat1, lon1):
    dy = (lat1 - lat0) * M_PER_DEG_LAT
    dx = (lon1 - lon0) * M_PER_DEG_LAT * math.cos(math.radians(lat0))
    return dx, dy


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def load_yaml_defaults(path):
    """读取 config/formation.yaml, 作为命令行参数默认值(命令行可覆盖, 规范#24)。"""
    import yaml
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        print(f"[!] 未找到配置文件 {path}, 使用内置默认值")
        return {}


class DroneNode(Node):
    def __init__(self, args):
        super().__init__(f"drone_node_{args.idx}")
        self.args = args
        self.idx = int(args.idx)
        self.ns = f"/{args.prefix}{self.idx}/mavros"
        self.mode = args.mode
        self.ref_home = None
        self.state = None
        self.local = None
        self.home = None

        # ---- 订阅自己飞控 ----
        self.create_subscription(State, f"{self.ns}/state", self._on_state, STATE_QOS)
        self.create_subscription(PoseStamped, f"{self.ns}/local_position/pose",
                                 self._on_local, SENSOR_QOS)
        self.create_subscription(HomePosition, f"{self.ns}/home_position/home",
                                 self._on_home, HOME_QOS)

        # ---- 服务客户端（★ 无 set_mode: 地面站是唯一 SET_MODE 权威, 规范#11/docs/28 §5.3）----
        self.cli_arm = self.create_client(CommandBool, f"{self.ns}/cmd/arming")
        self.cli_takeoff = self.create_client(CommandTOL, f"{self.ns}/cmd/takeoff")
        self.cli_land = self.create_client(CommandTOL, f"{self.ns}/cmd/land")

        # ---- 发布设定点(自己) ----
        self.sp_pub = self.create_publisher(
            PoseStamped, f"{self.ns}/setpoint_position/local", 10)

        # ---- 队形/飞行参数（yaml 默认 + CLI 覆盖 + 地面站 40010 即时切换）----
        self.shape = args.shape
        self.spacing = float(args.spacing)
        self.radius = float(args.radius)
        self.yaw = math.radians(float(args.yaw_deg))
        self.yaw_cur = self.yaw           # 当前平滑朝向(FLY 跟随中心线方向, 2026-08-28)
        self.n_drones = int(args.n_drones)
        self.fly_z = float(args.fly_z)
        # ★ 2026-08-28 修复: 实际飞行高度 —— 默认 fly_z, 收到带 z 的航点后改用航点高度
        self.flight_z = self.fly_z
        self.v_des = float(args.v_des)
        self.arrival_radius = float(args.arrival_radius)
        self.skip_first_wp = bool(args.skip_first_wp)

        # ---- 中心航线（2026-08-28: PullMission 从自家飞控拉取 / DDS 共享兜底）----
        self.waypoints = []
        self._pull_pending = False     # 授权时置位 → 拉取线程补拉(覆盖新上传/残留)
        self.advancer = WaypointAdvancer([], self.arrival_radius, self.v_des,
                                         1.0 / SP_HZ, self.skip_first_wp)
        self.center = None
        # ★ 成型阶段中心位置(评审注记#7): takeoff=起飞点(默认, docs/28 §6.2 ②);
        #   first_wp=航点列表第 0 个(需与起飞点一致或显式配置)
        self.center_init = args.center_init
        self._fly_entered = False

        # ---- 阶段机 / 避碰 ----
        self.phase = FormationPhase(
            judge_dist=float(args.formation_judge_dist),
            judge_time=float(args.formation_judge_time),
            timeout=float(args.formation_timeout),
            hold_s=float(args.hold_s), sp_hz=SP_HZ,
            n_expected=1)      # 分布式: 每机只判自己(规范#12)
        self.avoid = AvoidanceOffset(
            safe_dist=float(args.safe_dist), deadband=float(args.deadband),
            max_offset=float(args.max_offset), tau=float(args.offset_filter_tau),
            dt=1.0 / SP_HZ)
        self.avoid_enabled = bool(args.avoid)
        self.dds_enabled = bool(args.dds)

        # ---- 通道 / 共享（run() 里启动）----
        self.mav_ch = None
        self.udp_ch = None
        self.sharer = None
        self.mission_sharer = None
        self._applied_mission_ver = -1
        self._applied_shape = None
        self.land_requested = False

    # ================= 地面站回调(通道线程调用) =================
    def on_gcs_waypoints(self, wps):
        """上传中心航点(公共系 ENU 米, 编队几何中心轨迹, docs/28 §6.3)。
        航点格式 [(x, y[, z])] —— 2026-08-28 起地面站逐架上传带高度 z,
        节点飞行高度 = 航点 z(首个非零), 不再恒用 fly_z(否则实际高度≠地面站设置)。
        返回 (accepted, reason)。"""
        pts = []
        # ★ 2026-08-28: 去掉 1000m 安全边界(用户要求) —— 中心线不限距参考点距离
        for wp in wps:
            x, y = float(wp[0]), float(wp[1])
            z = float(wp[2]) if len(wp) > 2 else 0.0
            pts.append([x, y, z])
        changed = pts != self.waypoints
        self.waypoints = pts
        # ★ 2026-08-28 修复: 飞行高度 = 航点 z(地面站设置, 如 50m); 全 0 则回退 fly_z
        wz = [w[2] for w in pts if w[2] > 0.1]
        if wz:
            self.flight_z = float(max(wz))     # 用最高航点高度(编队统一)
            print(f"[uav{self.idx}] ★ 航点高度 z={self.flight_z:.1f}m "
                  f"(地面站上传), 起飞/飞行按此高度")
        self.advancer.set_waypoints(pts)
        print(f"[uav{self.idx}] ★ 中心航点: {len(pts)} 个 -> "
              f"{[(round(w[0],1), round(w[1],1)) for w in pts]}")
        # ★ DDS 兜底分发(评审注记#1): 只有内容变化才广播, 防止双通道(飞控转发+DDS)
        #   重复广播造成各机版本号漂移; MissionSharer 内部再按版本戳去重。
        if changed and self.dds_enabled and self.mission_sharer is not None:
            self.mission_sharer.broadcast_mission(pts)
        return (1, 0)

    def on_gcs_speed(self, speed):
        """DO_CHANGE_SPEED(178) 上报: 覆盖编队巡航速度 v_des(formation.yaml/--v-des)。

        param1=0(空速), param2=速度 m/s。地面站只在"勾选了改变速度且速度>0"时
        插入此命令, 故 param2<=0 一律忽略。新速度同时同步到 advancer(中心推进速度)。"""
        try:
            s = float(speed)
        except (TypeError, ValueError):
            return
        if s <= 0:
            return
        self.v_des = s
        self.advancer.v_des = s
        print(f"[uav{self.idx}] ★ 速度更新: v_des={self.v_des:.2f} m/s "
              f"(地面站 DO_CHANGE_SPEED, 编队推进/插值速度)")

    def on_gcs_shape(self, shape, spacing, radius):
        """COMMAND_LONG 40010: 切换队形(飞行途中即时生效, docs/28 §4.2)。"""
        if shape is None:
            print(f"[uav{self.idx}] 未知队形, 忽略")
            return
        self._apply_shape(shape, spacing, radius)
        if self.dds_enabled and self.mission_sharer is not None:
            sid = SHAPE_NAMES.get(shape, 0)
            self.mission_sharer.broadcast_shape(sid, self.spacing, self.radius)
        print(f"[uav{self.idx}] ★ 队形已切换 -> {shape} 间距={self.spacing}m "
              f"槽位={self.my_slot()}")

    def _apply_shape(self, shape, spacing, radius):
        self.shape = shape
        if spacing > 0:
            self.spacing = float(spacing)
        if radius > 0:
            self.radius = float(radius)

    # ---------- 回调 ----------
    def _on_state(self, m):
        self.state = m

    def _on_local(self, m):
        self.local = m.pose.position

    def _on_home(self, m):
        self.home = m.geo

    # ---------- 坐标 ----------
    def aligned_xy(self):
        if self.local is None or self.home is None or self.ref_home is None:
            return None
        if self.mode == "gazebo":
            return (self.local.x, self.local.y)
        ox, oy = enu_offset(self.ref_home.latitude, self.ref_home.longitude,
                            self.home.latitude, self.home.longitude)
        return (self.local.x + ox, self.local.y + oy)

    def to_local(self, tx, ty):
        """公共系目标 → 本机 local 系(下发 setpoint 用)。真 SITL 多机 home 错开时必须。"""
        if self.mode == "gazebo" or self.home is None or self.ref_home is None:
            return tx, ty
        ox, oy = enu_offset(self.ref_home.latitude, self.ref_home.longitude,
                            self.home.latitude, self.home.longitude)
        return tx - ox, ty - oy

    def common_z(self):
        return self.local.z if self.local is not None else None

    # ---------- 服务 ----------
    def request_streams(self):
        """请求 LOCAL_POSITION_NED(20Hz)。新版 mavros 不自动请求流, 必须主动调。"""
        cli = self.create_client(MessageInterval, f"{self.ns}/set_message_interval")
        if self._service_ready(cli, timeout=5.0):
            self.call(cli, MessageInterval.Request(message_id=32, message_rate=20.0),
                      "set_interval LOCAL_POSITION_NED", retries=1, timeout=10.0)
        else:
            print(f"  [uav{self.idx}] set_message_interval 服务不在(跳过, 模拟器)")

    def call(self, client, req, name, retries=3, timeout=30.0):
        """异步调用服务: 自旋等待结果(不用 spin_until_future_complete, 避免与
        run() 主循环的 spin_once 冲突导致 context invalid)。"""
        for attempt in range(retries):
            if not self._service_ready(client, timeout=10.0):
                print(f"  [uav{self.idx}] {name} 服务不在, 重试 {attempt+1}")
                continue
            fut = client.call_async(req)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < timeout:
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.01)
            if fut.done():
                res = fut.result()
                if res is not None:
                    ok = getattr(res, "success", True)
                    print(f"  [uav{self.idx}] {name} -> {'OK' if ok else res}")
                    return res
                print(f"  [uav{self.idx}] {name} 返回空, 重试 {attempt+1}")
            else:
                print(f"  [uav{self.idx}] {name} 超时, 重试 {attempt+1}")
        return None

    def arm(self):
        for _ in range(10):
            self.call(self.cli_arm, CommandBool.Request(value=True), "cmd/arming")
            if self.state is not None and self.state.armed:
                return True
            time.sleep(2.0)
        return self.state is not None and self.state.armed

    def takeoff_to(self, z):
        """起飞并【等待达到高度】。★ 若已在空中(z≥90%)直接返回。"""
        if self.local is not None and self.local.z >= z * 0.9:
            return True
        for attempt in range(3):
            self.call(self.cli_takeoff, CommandTOL.Request(altitude=float(z)),
                      "cmd/takeoff")
            t0 = time.time()
            while time.time() - t0 < 60:
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.local is not None and self.local.z >= z * 0.9:
                    return True
                if self.state is not None and not self.state.armed:
                    break          # 自动上锁 → 重新解锁重试
            time.sleep(1.0)
        return self.local is not None and self.local.z >= z * 0.9

    def land(self):
        return self.call(self.cli_land, CommandTOL.Request(altitude=0.0), "cmd/land")

    def _service_ready(self, client, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if client.service_is_ready():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        return client.service_is_ready()

    # ---------- 我的槽位 ----------
    def my_slot(self):
        """按编号取自己槽位(含旋转)。越界防护: 编号非法时报警并取槽位0。"""
        slots = make_slots(self.shape, self.n_drones, self.spacing, self.radius)
        if self.idx >= len(slots):
            print(f"[!!] uav{self.idx} 编号越界(机数={self.n_drones}), 取槽位0; "
                  f"请检查 --idx/--n-drones")
            i = 0
        else:
            i = self.idx
        return rotate_slot(slots[i], self.yaw_cur)

    # ---------- 设定点 ----------
    def _update_yaw(self):
        """★ 朝向纳入算法(2026-08-28): FLY 阶段队形机头【跟随中心线行进方向】
        (朝向下一个中心航点), 平滑旋转(限 yaw_rate); 非 FLY 回到固定 --yaw-deg。
        队形槽位随 yaw_cur 旋转 → 转弯时整个队形像刚体一样转向(差速式转弯)。"""
        target = self.yaw
        if self.phase.phase == FormationPhase.FLY and self.waypoints \
                and self.center is not None:
            tgt = self.advancer.current_target()
            if tgt is not None:
                dx, dy = tgt[0] - self.center[0], tgt[1] - self.center[1]
                if math.hypot(dx, dy) > 0.5:       # 距离太近不转向(已到目标点)
                    target = math.atan2(dy, dx)
        diff = (target - self.yaw_cur + math.pi) % (2 * math.pi) - math.pi  # 最短角
        step = math.radians(self.args.yaw_rate) / SP_HZ
        if abs(diff) <= step:
            self.yaw_cur = target
        else:
            self.yaw_cur += step if diff > 0 else -step

    def pub_setpoint(self, x, y, z, yaw=0.0):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = float(x)
        sp.pose.position.y = float(y)
        sp.pose.position.z = float(z)
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        sp.pose.orientation.x, sp.pose.orientation.y = qx, qy
        sp.pose.orientation.z, sp.pose.orientation.w = qz, qw
        self.sp_pub.publish(sp)

    # ---------- 共享状态应用(DDS, 保证各机一致) ----------
    def _apply_shared(self):
        if self.mission_sharer is None:
            return
        ver = self.mission_sharer.mission_version()
        if ver > self._applied_mission_ver:
            wps = self.mission_sharer.mission()
            if wps:
                # ★ 2026-08-28 修复: 内容相同不重置 —— 各机各自从飞控拉到的航线
                #   首点=自家 home(逐架直发), 三份内容仅首点不同且首点默认跳过;
                #   若不加去重, 飞行中 DDS 重复广播会反复 set_waypoints 复位
                #   wp_idx(推进回退/重飞)。只有内容真正变化才替换航线。
                if [list(w) for w in wps] != self.waypoints:
                    self.waypoints = [list(w) for w in wps]
                    self.advancer.set_waypoints(self.waypoints)
                    # ★ 2026-08-28: DDS 航点也带 z → 同步飞行高度(与 on_gcs_waypoints 一致)
                    wz = [w[2] for w in self.waypoints if len(w) > 2 and w[2] > 0.1]
                    if wz:
                        self.flight_z = float(max(wz))
                    print(f"[uav{self.idx}] ★ DDS 共享中心航线(v={ver}): "
                          f"{len(wps)} 个 -> "
                          f"{[(round(w[0],1), round(w[1],1)) for w in wps]}")
                self._applied_mission_ver = ver
            else:
                self._applied_mission_ver = ver
        shp = self.mission_sharer.shape()
        if shp is not None and shp != self._applied_shape:
            shape_name = SHAPE_IDS.get(shp[0], self.shape)
            self._apply_shape(shape_name, shp[1], shp[2])
            print(f"[uav{self.idx}] ★ DDS 共享队形 -> {shape_name} "
                  f"(spacing={shp[1]} radius={shp[2]})")
            self._applied_shape = shp

    # ---------- 主循环 ----------
    def _do_pull(self):
        """从自家飞控拉取一次航线; 结果应用:
        - 非空 → on_gcs_waypoints(去重后进状态机/DDS)
        - 空列表(飞控无航线) → 也应用, 清除 DDS 残留的旧航点(2026-08-28 修复)
        - None(超时) → 跳过"""
        try:
            wps = self.mav_ch.pull_mission(1 + self.idx, timeout=8.0)
            if wps is not None:
                self.on_gcs_waypoints(wps)
        except Exception as e:
            print(f"[uav{self.idx}] 拉取航线异常: {e}")

    def _pull_loop(self):
        """后台线程: 拉取只在【需要时】发生(2026-08-28 修复):
          - 启动后拉一次(飞控可能已有地面站上传的航线; 空则清除 DDS 残留);
          - 授权时补拉(_pull_pending, 覆盖授权前上传的新中心线);
          - 已授权但无航线时周期补拉。
          地面站上传发生在授权前, 授权后补拉时上传已完成, 不打断 serial0 上传
          (ArduPilot mission 协议跨串口互相打断, docs/29 §7.4)。"""
        time.sleep(2.0)          # 等通道/MAVROS 就绪
        self._do_pull()
        while True:
            time.sleep(self.args.pull_interval)
            if self._pull_pending:
                self._pull_pending = False
                self._do_pull()
            elif self.phase.is_authorized() and not self.waypoints:
                self._do_pull()

    def run(self):
        from types import SimpleNamespace
        print(f"[uav{self.idx}] 单机节点就绪(分布式控制方法, docs/28), 等本机 home...")
        # ★ 公共系基准 = 地面站固定基线(ref_lat/ref_lon), 与 gcs_channel 航点转换一致。
        #   ⚠️ 不能用"自己的 home"当基准 —— 真 SITL 多机 home 沿东错开时队形会错位。

        # ★ 通道先行创建(不依赖 home): 主链路 mavlink 透传 + 可选 UDP + DDS 共享
        #   (2026-08-28: 已去掉中心线安全边界保护, 不限制距参考点距离)
        self.mav_ch = MavlinkChannel(self, f"/uas{1 + self.idx}",
                                     self.args.ref_lat, self.args.ref_lon,
                                     node_sysid=self.args.node_sysid)
        if self.args.gcs_port_base > 0:
            port = self.args.gcs_port_base + self.idx
            self.udp_ch = GcsChannel(self, port, self.args.ref_lat,
                                     self.args.ref_lon,
                                     node_sysid=self.args.node_sysid)
            self.udp_ch.start()
            print(f"[uav{self.idx}] ★ 旧 UDP 直连通道: udp:0.0.0.0:{port} (可选兼容)")
        if self.dds_enabled:
            self.sharer = PoseSharer(self, self.idx, self.n_drones)
            self.mission_sharer = MissionSharer(self)
            print(f"[uav{self.idx}] DDS 共享: /formation/positions + /formation/mission|shape")

        # ★ home 等待带恢复(联调实测修复): MAVROS home_position 偶发只请求一次且拿不到
        #   home(启动竞态)。超时后【不退出】, 每 10s 经 mavlink 通道向本机飞控重发
        #   MAV_CMD_GET_HOME_POSITION(410) —— 飞控回 HOME_POSITION → MAVROS 插件发布
        #   → 本节点订阅生效。避免节点因 home 缺失崩溃。
        t0 = time.time()
        last_req = 0.0
        while self.home is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            now = time.time()
            if now - t0 > 60 and now - last_req > 10.0:
                last_req = now
                print(f"[uav{self.idx}] [!!] home 仍未到({(now - t0):.0f}s), "
                      f"重发 GET_HOME_POSITION(410) -> 飞控 sysid={1 + self.idx}")
                try:
                    self.mav_ch._send(self.mav_ch.mav.command_long_encode(
                        1 + self.idx, 1, 410, 0, 0, 0, 0, 0, 0, 0, 0))
                except Exception as e:
                    print(f"[uav{self.idx}] 重发 home 请求异常: {e}")
            if now - t0 > 300:
                print(f"[uav{self.idx}] [!!] home 等待超时(300s), 继续运行但坐标对齐可能不准")
                break
        self.ref_home = SimpleNamespace(latitude=self.args.ref_lat,
                                        longitude=self.args.ref_lon)
        if self.home is not None:
            print(f"[ok] 公共系基准: ({self.ref_home.latitude:.6f},{self.ref_home.longitude:.6f}) "
                  f"本机 home: ({self.home.latitude:.6f},{self.home.longitude:.6f})")
        else:
            print(f"[!!] home 缺失, aligned_xy 将不可用(公共系对齐失效)")

        self.request_streams()      # ★ 请求 LOCAL_POSITION_NED(否则 local 永远没数据)

        # ★ 2026-08-28: 删除 99 后, 中心航线由节点从【自家飞控】周期拉取
        #   (地面站经 MAVSDK 逐架直发飞控 target=1..N)。拉到即进 on_gcs_waypoints
        #   (自动 DDS 广播 + 状态机)。--pull-interval=0 关闭。
        if self.args.pull_interval > 0:
            threading.Thread(target=self._pull_loop, daemon=True).start()
            print(f"[uav{self.idx}] ★ 航线拉取: 每 {self.args.pull_interval}s 从飞控 "
                  f"sysid={1 + self.idx} 下载中心线(PullMission)")

        print(f"[uav{self.idx}] ★ 等待地面站授权(解锁 + setmode GUIDED): "
              f"未授权不发 setpoint (规范#11)...")

        # ---------- 主循环 ----------
        t_sim, dt = 0.0, 1.0 / SP_HZ
        t_real0 = time.time()
        log_t, share_t = 0.0, 0.0
        log_target = None
        last_phase = None
        while not self.land_requested:
            t_sim += dt
            now = time.time()
            self.mav_ch.tick(now)
            self._update_yaw()             # ★ 朝向跟随中心线方向(2026-08-28)
            self._apply_shared()
            # ★ 2026-08-28 修复"真实地面站只有一架飞/其余阻塞": 真实地面站时序 =
            #   授权(arm+GUIDED)先于/紧邻航线上传完成 → 起飞成型时无航线 → HOVER_WAIT;
            #   航线随后经 PullMission/DDS 到达, 若不恢复将永久悬停。航线到达即恢复成型。
            if self.waypoints and self.phase.on_mission_arrived(t_sim):
                print(f"[uav{self.idx}] ★ 航线已到达, 从无航点悬停恢复成型流程"
                      f"(先授权后上传时序), {len(self.waypoints)} 个航点")

            guided = self.state is not None and self.state.mode == "GUIDED"
            armed = self.state is not None and self.state.armed

            # ---- 授权状态机（规范#11, docs/28 §5）----
            if guided and not self.phase.is_authorized():
                self.phase.on_authorized(t_sim)
                self.center = None
                print(f"[uav{self.idx}] ★ 已授权(GUIDED), 进入起飞/成型流程")
                self._pull_pending = True     # 授权时触发一次拉取(覆盖授权前上传的中心线)
            elif not guided and self.phase.is_authorized():
                self.phase.on_deauthorized()
                self.avoid.reset()
                print(f"[uav{self.idx}] ★ 控制收回(mode={self.state.mode if self.state else '?'}), "
                      f"停止 setpoint, 等地面站重新授权")

            phase = self.phase.phase
            if phase != last_phase:
                print(f"[uav{self.idx}] 阶段 -> {self.phase.name()}"
                      + (f" (mode={self.state.mode})" if self.state else ""))
                last_phase = phase

            # ---- TAKEOFF: 升到设定高度（未解锁则等待, 不发 setpoint）----
            if phase == FormationPhase.TAKEOFF:
                if armed and not self.takeoff_to(self.flight_z):
                    print(f"[uav{self.idx}] [!!] 起飞未达高度, 保持等待")
                if self.common_z() is not None and self.common_z() >= self.flight_z * 0.9:
                    p = self.aligned_xy()
                    self.center = [p[0], p[1]] if p else [0.0, 0.0]
                    self.phase.on_takeoff_done(t_sim)
                    print(f"[uav{self.idx}] 已到 {self.common_z():.1f}m, 队形成型中: "
                          f"中心=({self.center[0]:.2f},{self.center[1]:.2f}) "
                          f"槽位={self.my_slot()}")

            # ---- FORMING/HOLD/FLY/HOVER_WAIT: 发设定点 ----
            elif self.phase.should_send_setpoint():
                if self.center is None:
                    p = self.aligned_xy()
                    self.center = [p[0], p[1]] if p else [0.0, 0.0]
                # 中心沿中心航线推进（仅 FLY 阶段; 成型/保持期中心不动）
                if phase == FormationPhase.FLY:
                    if not self._fly_entered:
                        # ★ 评审注记#7: 明确成型中心 = 起飞点; 进入 FLY 时才可选跳到
                        #   航点0(center_init=first_wp), 默认保持起飞点再按 skip_first 推进
                        if self.center_init == "first_wp" and self.waypoints:
                            old = list(self.center)
                            self.center = [self.waypoints[0][0], self.waypoints[0][1]]
                            print(f"[uav{self.idx}] ★ 中心初始化=航点0: "
                                  f"({old[0]:.2f},{old[1]:.2f}) -> "
                                  f"({self.center[0]:.2f},{self.center[1]:.2f})")
                        self._fly_entered = True
                    # ★ 2026-08-28 修复: 到达判定必须基于【真实位置】, 不能用纯时间推进的
                    #   虚拟中心 —— 否则飞控若卡住(位置控制失效/未响应), 中心仍按 v_des·dt
                    #   空推 → 误判"到达航点/全部到达" → 空推后自动降落(实测 2/3 号机卡住
                    #   仍被判定飞完)。每拍用【实际位置 − 槽位】校正中心, 再交 advancer
                    #   推进与判到达: 飞控不动 → 中心不动 → d 不变 → 不误判。
                    p_act = self.aligned_xy()
                    if p_act is not None:
                        sx, sy = self.my_slot()
                        self.center = [p_act[0] - sx, p_act[1] - sy]
                    _, event = self.advancer.step(self.center)
                    if isinstance(event, tuple) and event[0] == self.advancer.WP_ARRIVED:
                        print(f"[uav{self.idx}] 到达中心航点 {event[1]}, "
                              f"前往 {event[2][0]:.1f},{event[2][1]:.1f}")
                    elif event == self.advancer.ALL_DONE:
                        print(f"[uav{self.idx}] ★ 全部中心航点已到达, 自动降落")
                        self.phase.on_all_done()
                        self.land_requested = True
                        break
                elif phase != FormationPhase.FLY:
                    self._fly_entered = False

                # 本机目标 = 中心 + 自己槽位（docs/28 §6.3）
                sx, sy = self.my_slot()
                target = (self.center[0] + sx, self.center[1] + sy)
                log_target = target
                p = self.aligned_xy()

                # 队形成型判据: 用【未加避碰偏移】的槽位目标（docs/28 §6.2）
                if p is not None:
                    err = math.hypot(p[0] - target[0], p[1] - target[1])
                    self.phase.update({self.idx: err}, t_sim)
                else:
                    self.phase.update({}, t_sim)

                # 分布式控制方法机间软避碰（docs/28 §7.2）
                if self.avoid_enabled and p is not None and self.sharer is not None:
                    others = self.sharer.others()
                    if others:
                        off = self.avoid.update(p, others)
                        target = (target[0] + off[0], target[1] + off[1])
                elif p is not None:
                    self.avoid.update(p, [])     # 保持低通衰减

                lx, ly = self.to_local(target[0], target[1])
                self.pub_setpoint(lx, ly, self.flight_z, self.yaw_cur)

            # ---- 无航点但已成型: 悬停等地面站 ----
            if not self.waypoints and phase in (FormationPhase.FORMING,
                                                FormationPhase.HOLD):
                self.phase.on_no_waypoints()

            # ---- DDS 位置发布 1Hz（docs/28 §7.1）----
            if self.dds_enabled and self.sharer is not None \
                    and now - share_t >= 1.0:
                share_t = now
                self.sharer.publish(self.aligned_xy())

            # ---- 周期日志 ----
            if now - log_t >= 2.0:
                log_t = now
                p_now = self.aligned_xy()
                pos = f"({p_now[0]:5.1f},{p_now[1]:5.1f})" if p_now else "(无)"
                tgt_s = (f"({log_target[0]:6.2f},{log_target[1]:6.2f})"
                         if log_target else "—")
                # ★ 联调实测修复: WAIT_AUTH 阶段 center 尚未初始化(None), 必须守卫
                center_s = (f"({self.center[0]:6.2f},{self.center[1]:6.2f})"
                            if self.center else "—")
                auth = "已授权" if self.phase.is_authorized() else "未授权"
                print(f"     t={t_sim:5.1f}s [{auth}] 阶段={self.phase.name():9s} "
                      f"目标={tgt_s} 中心={center_s} "
                      f"我: {pos} 航点={len(self.waypoints)}")

            if time.time() - t_real0 > 3600:
                print("[!!] 任务超时(1h)")
                break
            sleep = t_real0 + t_sim - time.time()
            if sleep > 0:
                time.sleep(sleep)
            # ★ 2026-08-28 修复: 每拍 spin 10 → 100 次 —— SIM_SPEEDUP=10 使 /uas{i}
            #   mavlink_source 洪泛(实测 213 msg/s, 20Hz×10), 原 10 次/拍(≈100/s)跟不上,
            #   队列积压 ~4.7s → PullMission 的 COUNT/ITEM 被延迟/丢弃(拉取第一轮成功
            #   后全部超时)。100 次/拍 ≈1000/s 足以清空队列, mission 消息即时处理。
            for _ in range(100):
                rclpy.spin_once(self, timeout_sec=0.0)

        # ---------- 结束: 自动降落（评审注记#5: 降落后/被收回时交还地面站）----------
        if self.land_requested and self.state is not None:
            if self.state.armed and self.state.mode == "GUIDED":
                print(f"[uav{self.idx}] 自动降落(仍受权 GUIDED)...")
                self.land()
            else:
                # 已非 GUIDED/已上锁: 不再发降落指令, 控制交还地面站(LAND/RTL 直发飞控)
                print(f"[uav{self.idx}] 已非 GUIDED(mode={self.state.mode}, "
                      f"armed={self.state.armed}): 降落交还地面站")
        else:
            print(f"[uav{self.idx}] 结束")
        print(f"[uav{self.idx}] 结束")


def main():
    cfg_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "config", "formation.yaml")
    cfg = load_yaml_defaults(cfg_path)

    ap = argparse.ArgumentParser(
        description="单机编队节点(分布式控制方法, docs/28 修订版: 经飞控转发收编队指令)")
    ap.add_argument("--idx", type=int, default=0, help="本机编号(0..n-1)")
    ap.add_argument("--prefix", default=cfg.get("prefix", "uav"),
                    help="MAVROS 命名空间前缀")
    ap.add_argument("--mode", default=cfg.get("mode", "plain"), help="plain|gazebo")
    ap.add_argument("--n-drones", type=int, default=cfg.get("n_drones", 3),
                    help="编队总机数(槽位表长度)")
    # ---- 队形（地面站 40010 可随时切换, docs/28 §4.2）----
    ap.add_argument("--shape", default=cfg.get("shape", "line"),
                    choices=sorted(SHAPE_IDS.values()),
                    help="初始队形")
    ap.add_argument("--spacing", type=float, default=cfg.get("spacing", 1.5),
                    help="队形间距 m")
    ap.add_argument("--radius", type=float, default=cfg.get("radius", 5.0),
                    help="circle 半径 m")
    ap.add_argument("--yaw-deg", type=float, default=cfg.get("yaw_deg", 0.0),
                    help="队形朝向(度, 非 FLY 时固定值)")
    ap.add_argument("--yaw-rate", type=float, default=cfg.get("yaw_rate", 30.0),
                    help="队形转向最大角速度 deg/s(FLY 跟随中心线方向, 默认 30)")
    # ---- 控制参数（docs/28 §8）----
    ap.add_argument("--fly-z", type=float, default=cfg.get("fly_z", 2.0),
                    help="巡航高度 m")
    ap.add_argument("--v-des", type=float, default=cfg.get("v_des", 1.0),
                    help="巡航速度 m/s")
    ap.add_argument("--hold-s", type=float, default=cfg.get("hold_s", 5.0),
                    help="队形成型后稳定等待秒数(默认 5s, docs/28 §8)")
    ap.add_argument("--arrival-radius", type=float,
                    default=cfg.get("arrival_radius", 1.0),
                    help="航点到达判定距离 m(docs/28 §6.3)")
    ap.add_argument("--skip-first-wp", type=int,
                    default=1 if cfg.get("skip_first_wp", False) else 0,
                    help="跳过首个中心航点(★ 2026-08-28 §7.14 默认 0: PullMission 已"
                         "丢弃飞控 seq=0 home 项, 再跳首点=双重跳过损失真航点; 旧"
                         "直传节点链路可显式 --skip-first-wp 1)")
    ap.add_argument("--center-init", default=cfg.get("center_init", "takeoff"),
                    choices=["takeoff", "first_wp"],
                    help="成型阶段中心位置: takeoff=起飞点(默认, docs/28 §6.2); "
                         "first_wp=进入 FLY 时跳到航点0")
    # ---- 队形成型判据/超时（docs/28 §6.2, 规范#12）----
    ap.add_argument("--formation-judge-dist", type=float,
                    default=cfg.get("formation_judge_dist", 0.3),
                    help="成型判据: 距目标槽位 < 该值")
    ap.add_argument("--formation-judge-time", type=float,
                    default=cfg.get("formation_judge_time", 1.0),
                    help="成型判据: 持续秒数")
    ap.add_argument("--formation-timeout", type=float,
                    default=cfg.get("formation_timeout", 30.0),
                    help="成型超时秒数(超时悬停等地面站)")
    # ---- DDS 避碰（docs/28 §7.2, 规范#15）----
    ap.add_argument("--safe-dist", type=float, default=cfg.get("safe_dist", 1.5),
                    help="避碰安全距离 m")
    ap.add_argument("--deadband", type=float, default=cfg.get("deadband", 0.3),
                    help="避碰死区 m")
    ap.add_argument("--max-offset", type=float, default=cfg.get("max_offset", 2.0),
                    help="避碰最大偏移 m")
    ap.add_argument("--offset-filter-tau", type=float,
                    default=cfg.get("offset_filter_tau", 1.0),
                    help="避碰偏移低通时间常数 s")
    ap.add_argument("--avoid", type=int, default=1 if cfg.get("avoid", True) else 0,
                    help="启用 DDS 机间避碰(默认 1)")
    ap.add_argument("--dds", type=int, default=1 if cfg.get("dds", True) else 0,
                    help="启用 DDS 位置/航线/队形共享(默认 1)")
    # ---- 通道 ----
    ap.add_argument("--gcs-port-base", type=int, default=cfg.get("gcs_port_base", 0),
                    help="旧 UDP 直连测试通道端口基数(0=关, 默认关; 主链路=经 MAVROS)")
    ap.add_argument("--node-sysid", type=int, default=cfg.get("node_sysid", 255),
                    help="本节点在 MAVLink 里的 system id(拉取/应答身份, 默认 255)")
    ap.add_argument("--pull-interval", type=float, default=cfg.get("pull_interval", 10.0),
                    help="从自家飞控拉取航线的周期 s(0=关闭拉取)")
    ap.add_argument("--ref-lat", type=float, default=cfg.get("ref_lat", 30.893000),
                    help="地面站经纬→ENU 固定基线纬度")
    ap.add_argument("--ref-lon", type=float, default=cfg.get("ref_lon", 120.087000),
                    help="地面站经纬→ENU 固定基线经度")
    args = ap.parse_args()

    rclpy.init()
    n = DroneNode(args)
    try:
        n.run()
    except KeyboardInterrupt:
        print(f"\n[uav{n.idx}] Ctrl-C")
    finally:
        if n.mav_ch is not None:
            n.mav_ch.stop_channel()
        if n.udp_ch is not None:
            n.udp_ch.stop_channel()
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
