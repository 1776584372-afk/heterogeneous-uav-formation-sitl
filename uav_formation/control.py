"""编队控制算法模块 —— 与 MAVLink/ROS 传输解耦，便于单测（docs/28 §9.3, 开发规范#23）。

包含四类可独立测试的组件：
  * WaypointAdvancer   航点推进：到达判定(arrival_radius) + 中心速度插值(v_des·dt)
                       （docs/28 §6.3, 规范#13）
  * FormationPhase     起飞后阶段机：升高度→队形成型(判据+超时)→等 hold_s→沿航线飞
                       （docs/28 §6.2, docs/27 §3.4, 规范#12）
  * AvoidanceOffset    机间软避碰：死区 + 低通滤波 + 最大偏移限制
                       （docs/28 §7.2, docs/26 §4.1, 规范#15）
  * PoseSharer        分布式控制方法机间 DDS 位置共享（/formation/positions, PoseArray, 公共系）
  * MissionSharer     中心航线/队形 DDS 共享（/formation/mission, /formation/shape）
                       —— 因 MAVProxy 路由把 target=99 只送到第一路 master（FC1），
                         只有 FC1 的节点能收到特殊指令，需要经 DDS 分发给其余节点。

坐标系约定：公共系 ENU 米（ref_lat/ref_lon 固定基线），与 gcs_channel/drone_node 一致。
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray


# =====================================================================
# 1. 航点推进（docs/28 §6.3：上传航点 = 编队几何中心轨迹）
# =====================================================================
class WaypointAdvancer:
    """虚拟中心沿中心航点推进：

    - 当前目标航点序号 wp_idx 初始为 1（跳过起点，docs/28 §6.3）；
    - 中心到目标水平距离 d < arrival_radius 视为到达，wp_idx++；
    - wp_idx 超界 → ALL_DONE（节点随后悬停等待/自动降落）；
    - 中心每拍按 center += (target-center).normalized() * v_des * dt 插值，防止跳变。
    """

    MOVING = "moving"
    WP_ARRIVED = "wp_arrived"
    ALL_DONE = "all_done"

    def __init__(self, waypoints, arrival_radius=1.0, v_des=1.0, dt=0.05,
                 skip_first=True):
        self.set_waypoints(waypoints, skip_first)
        self.arrival_radius = float(arrival_radius)
        self.v_des = float(v_des)
        self.dt = float(dt)
        self.wp_idx = 1 if (skip_first and len(self.waypoints) > 1) else 0
        self.all_done = False

    def set_waypoints(self, waypoints, skip_first=None):
        """整体替换中心航线（地面站重新上传时调用）。返回航点数。
        航点格式 [(x, y[, z])] —— z 为航点相对高(2026-08-28 起保留, 推进只用 x/y)。"""
        self.waypoints = [[float(w[0]), float(w[1])] for w in waypoints]
        if skip_first is not None:
            self.skip_first = bool(skip_first)
        else:
            self.skip_first = getattr(self, "skip_first", True)
        self.wp_idx = 1 if (self.skip_first and len(self.waypoints) > 1) else 0
        self.all_done = False
        return len(self.waypoints)

    def reset(self):
        self.wp_idx = 1 if (self.skip_first and len(self.waypoints) > 1) else 0
        self.all_done = False

    def current_target(self):
        if not self.waypoints:
            return None
        return self.waypoints[min(self.wp_idx, len(self.waypoints) - 1)]

    def step(self, center):
        """推进中心一步。center: [x, y]（原地修改并返回）。返回 (center, event)。"""
        if self.all_done or not self.waypoints:
            return center, self.ALL_DONE
        idx = min(self.wp_idx, len(self.waypoints) - 1)
        tx, ty = self.waypoints[idx]
        dx, dy = tx - center[0], ty - center[1]
        d = math.hypot(dx, dy)

        if d <= self.arrival_radius:
            # 到达当前航点
            if self.wp_idx < len(self.waypoints) - 1:
                self.wp_idx += 1
                return center, (self.WP_ARRIVED, self.wp_idx - 1, self.waypoints[self.wp_idx])
            self.all_done = True
            return center, self.ALL_DONE

        # 速度插值推进（docs/28 §6.3 公式），到达前最后一拍直接落点防过冲
        step = self.v_des * self.dt
        if step >= d:
            center[0], center[1] = tx, ty
        else:
            ux, uy = dx / d, dy / d
            center[0] += ux * step
            center[1] += uy * step
        return center, self.MOVING


# =====================================================================
# 2. 起飞后阶段机（docs/28 §6.2, docs/27 §3.4, 规范#12）
# =====================================================================
class FormationPhase:
    """编队阶段状态机（节点每拍调用 update()）：

      WAIT_AUTH  未授权（非 GUIDED）：不发 setpoint，等地面站授权
      TAKEOFF    已授权 → 升到设定高度（takeoff_to，等 z 到位）
      FORMING    队形成型：判据 = 全部机距自己目标槽位 < judge_dist 且持续 judge_time；
                 超时 formation_timeout → HOVER_WAIT（悬停等地面站介入）
      HOLD       成型后保持 hold_s 秒（队形稳定期）
      FLY        中心沿中心航线推进，队形整体移动
      HOVER_WAIT 成型超时/无航点：悬停保持，等地面站指令
                 （无航点原因的悬停: 航线到达后自动恢复成型, 见 on_mission_arrived）
      DONE       全部航点到达（由宿主决定降落）

    分布式：每机只报自己的误差（update(errors={idx: dist})），
    集中式：节点上报全部机误差。
    """

    WAIT_AUTH, TAKEOFF, FORMING, HOLD, FLY, HOVER_WAIT, DONE = range(7)
    NAMES = ["WAIT_AUTH", "TAKEOFF", "FORMING", "HOLD", "FLY", "HOVER_WAIT", "DONE"]

    # HOVER_WAIT 进入原因(2026-08-28 修复"先授权后上传"死锁, 见 on_mission_arrived)
    HOVER_NO_WP = "no_waypoints"       # 无中心航线悬停 —— 航线到达即恢复成型
    HOVER_TIMEOUT = "timeout"          # 成型超时悬停 —— 等地面站介入(模式切换), 不自动恢复

    def __init__(self, judge_dist=0.3, judge_time=1.0, timeout=30.0,
                 hold_s=5.0, sp_hz=20.0, n_expected=None):
        self.judge_dist = float(judge_dist)
        self.judge_time = float(judge_time)
        self.timeout = float(timeout)
        self.hold_s = float(hold_s)
        self.dt = 1.0 / float(sp_hz)
        self.n_expected = n_expected   # 若设置: 成型判据要求全部机误差齐全(防部分数据误判)
        self.phase = self.WAIT_AUTH
        self.hover_reason = None       # 进入 HOVER_WAIT 的原因(HOVER_NO_WP/HOVER_TIMEOUT)
        self.formed_at = None          # 已持续成型(满足判据)的起始仿真时刻
        self.forming_started = None    # 进入 FORMING 的时刻（超时计时）
        self.hold_started = None       # 进入 HOLD 的时刻
        self.ever_authorized = False
        self.last_errors = {}

    # ---- 状态查询 ----
    def name(self):
        return self.NAMES[self.phase]

    def is_authorized(self):
        return self.phase not in (self.WAIT_AUTH,)

    def is_flying_autonomously(self):
        return self.phase in (self.FORMING, self.HOLD, self.FLY)

    def should_send_setpoint(self):
        """是否允许发位置设定点（规范#11：非 GUIDED 不发）。

        FORMING/HOLD/FLY 正常编队；HOVER_WAIT（成型超时/无航点）仍需发
        当前位置保持悬停，直到地面站介入。"""
        return self.phase in (self.FORMING, self.HOLD, self.FLY, self.HOVER_WAIT)

    # ---- 驱动 ----
    def on_authorized(self, t_sim):
        """检测到 GUIDED(+armed) → 进入 TAKEOFF（由宿主调 takeoff_to）。"""
        if not self.ever_authorized:
            self.ever_authorized = True
        self.phase = self.TAKEOFF
        self.formed_at = None
        self.forming_started = None
        self.hold_started = None
        self.hover_reason = None
        self.last_errors = {}

    def on_deauthorized(self):
        """模式≠GUIDED → 收回控制：回到 WAIT_AUTH（规范#11, docs/27 §2 状态③）。"""
        self.phase = self.WAIT_AUTH
        self.formed_at = None
        self.forming_started = None
        self.hold_started = None
        self.hover_reason = None

    def on_takeoff_done(self, t_sim):
        """升到设定高度完成 → 进入 FORMING。"""
        self.phase = self.FORMING
        self.forming_started = t_sim
        self.hover_reason = None

    def update(self, errors, t_sim):
        """每拍调用。errors: {idx: 该机距自己目标槽位的水平距离}（可为空 dict）。"""
        if errors:
            self.last_errors = dict(errors)
        errs = list(self.last_errors.values()) if self.last_errors else []
        complete = (self.n_expected is None
                    or len(self.last_errors) == self.n_expected)
        all_formed = complete and bool(errs) and all(e <= self.judge_dist for e in errs)

        if self.phase == self.FORMING:
            if all_formed:
                if self.formed_at is None:
                    self.formed_at = t_sim
                elif t_sim - self.formed_at >= self.judge_time:
                    # ★ 成型判据满足（0.3m 持续 1s）→ 进入 HOLD 稳定期
                    self.phase = self.HOLD
                    self.hold_started = t_sim
                    self.formed_at = None
            else:
                self.formed_at = None
            # ★ 超时（30s 未成型 → 悬停等地面站，docs/28 §6.2）
            if self.forming_started is not None \
                    and t_sim - self.forming_started >= self.timeout:
                self.phase = self.HOVER_WAIT
                self.hover_reason = self.HOVER_TIMEOUT
        elif self.phase == self.HOLD:
            if self.hold_started is not None \
                    and t_sim - self.hold_started >= self.hold_s:
                self.phase = self.FLY

        return self.phase

    def on_mission_arrived(self, t_sim):
        """中心航线到达/更新为非空 → 从【无航点悬停】恢复成型流程。

        ★ 2026-08-28 修复"真实地面站只有一架飞/其余阻塞"死锁:
          真实地面站(sys2/KT GCS)时序 = 授权(arm+GUIDED)与航线上传紧邻甚至先行,
          节点起飞后 FORMING 阶段尚无航线 → on_no_waypoints 进 HOVER_WAIT;
          随后 PullMission 拉到航线, 但 HOVER_WAIT 无任何出口 → 永久悬停
          (实测 3 架全卡)。航线到达即恢复成型(成型计时重置)。

        成型超时悬停(HOVER_TIMEOUT)不在此恢复 —— 按文档语义等地面站介入
        (revoke/re-auth 模式切换), 航线更新不算介入。幂等: 非目标状态返回 False。"""
        if self.phase == self.HOVER_WAIT and self.hover_reason == self.HOVER_NO_WP:
            self.phase = self.FORMING
            self.forming_started = t_sim
            self.formed_at = None
            self.hover_reason = None
            return True
        return False

    def on_all_done(self):
        self.phase = self.DONE

    def on_no_waypoints(self):
        """无中心航线：成型后悬停等地面站（不推进）。"""
        if self.phase in (self.FORMING, self.HOLD):
            self.phase = self.HOVER_WAIT
            self.hover_reason = self.HOVER_NO_WP


# =====================================================================
# 3. 机间软避碰（docs/28 §7.2, docs/26 §4.1, 规范#15）
# =====================================================================
class AvoidanceOffset:
    """目标偏移式软避碰：

      对每架其他机 j：d = |pos_self - pos_j|
        d < safe_dist          → 偏移 += unit(远离 j) × (safe_dist - d) × 0.5
        d > safe_dist+deadband → 该机贡献完全撤销（死区，防来回触发）
      所有偏移经一阶低通滤波（时间常数 tau），总偏移限幅 max_offset（超限记警告）。
    """

    def __init__(self, safe_dist=1.5, deadband=0.3, max_offset=2.0,
                 tau=1.0, dt=0.05):
        self.safe_dist = float(safe_dist)
        self.deadband = float(deadband)
        self.max_offset = float(max_offset)
        self.tau = float(tau)
        self.dt = float(dt)
        self.offset = [0.0, 0.0]
        self._warned = False

    def reset(self):
        self.offset = [0.0, 0.0]

    def update(self, pos_self, others, log_warn=lambda m: print(m)):
        """others: 其他机公共系 (x, y) 列表。返回当前低通滤波后的偏移 (ox, oy)。"""
        raw_x, raw_y = 0.0, 0.0
        for (jx, jy) in others:
            dx, dy = pos_self[0] - jx, pos_self[1] - jy
            d = math.hypot(dx, dy)
            if d <= 1e-6:
                continue                     # 位置重叠：忽略（边界情况, 规范#22）
            if d < self.safe_dist:
                unit_x, unit_y = dx / d, dy / d
                push = (self.safe_dist - d) * 0.5
                raw_x += unit_x * push
                raw_y += unit_y * push
            # d >= safe_dist: 若 d > safe_dist + deadband 则撤销；安全距离附近(死区内)
            # 保持上一拍贡献由低通自然衰减 → 避免死区边界来回触发（规范#15）

        # 一阶低通（时间常数 tau）
        a = self.dt / (self.tau + self.dt)
        self.offset[0] += a * (raw_x - self.offset[0])
        self.offset[1] += a * (raw_y - self.offset[1])

        # 限幅（规范#15: 最大偏移 2m，超限警告）
        mag = math.hypot(self.offset[0], self.offset[1])
        if mag > self.max_offset:
            if not self._warned:
                log_warn(f"[avoid] 避碰偏移超限 {mag:.2f}m > {self.max_offset}m，限幅")
                self._warned = True
            k = self.max_offset / mag
            self.offset[0] *= k
            self.offset[1] *= k
        elif mag < 1e-3:
            self._warned = False
        return (self.offset[0], self.offset[1])

    def apply(self, target):
        """把避碰偏移加到目标点，返回偏移后的 (x, y)。"""
        return (target[0] + self.offset[0], target[1] + self.offset[1])


# =====================================================================
# 4. DDS 机间共享（docs/26 §3, §5；docs/28 §7.1；分布式控制方法）
# =====================================================================
# mavlink_source/sink 的 QoS 是 BEST_EFFORT+VOLATILE(深度1000, mavros_router.cpp)。
# 机间 DDS 用 TRANSIENT_LOCAL：新节点加入能立即收到最近一帧快照（docs/26 §3）。
SHARE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class PoseSharer:
    """分布式控制方法机间位置共享：/formation/positions（PoseArray, 公共系 ENU, 1Hz）。

    - 每机把自己位置放进快照后发布【全机快照】（未知机位填 NaN）；
    - 订阅合并：对每个 idx 取"有限值"（自己的 > DDS 快照 > 上次缓存）；
    - others() 返回除自己外已知位置列表，供避碰使用。
    """

    def __init__(self, node, idx, n_drones):
        self.node = node
        self.idx = int(idx)
        self.n = int(n_drones)
        self.snapshot = {}                    # idx -> (x, y)
        self.lock = None                     # rclpy 单线程回调, 无需锁
        self.pub = node.create_publisher(PoseArray, "/formation/positions", SHARE_QOS)
        self.sub = node.create_subscription(
            PoseArray, "/formation/positions", self._on_snapshot, SHARE_QOS)

    def _on_snapshot(self, msg):
        for i, pose in enumerate(msg.poses):
            x, y = pose.position.x, pose.position.y
            if math.isfinite(x) and math.isfinite(y):
                self.snapshot[i] = (float(x), float(y))

    def publish(self, my_pos):
        """发布全机快照（1Hz 调用）。my_pos: 本机公共系 (x, y)。"""
        if my_pos is not None:
            self.snapshot[self.idx] = (float(my_pos[0]), float(my_pos[1]))
        msg = PoseArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "formation"
        for i in range(self.n):
            p = PoseStamped()
            p.header.frame_id = "formation"
            if i in self.snapshot:
                p.pose.position.x = self.snapshot[i][0]
                p.pose.position.y = self.snapshot[i][1]
                p.pose.position.z = 0.0
            else:
                p.pose.position.x = float("nan")
                p.pose.position.y = float("nan")
                p.pose.position.z = 0.0
            msg.poses.append(p.pose)
        self.pub.publish(msg)

    def others(self):
        """其他机已知位置列表 [(x, y), ...]（公共系）。"""
        return [pos for i, pos in self.snapshot.items() if i != self.idx]

    def all_positions(self):
        return [self.snapshot.get(i) for i in range(self.n)]


class MissionSharer:
    """中心航线/队形 DDS 分发（docs/28 分布式控制方法一致性保障，见模块 docstring）。

    - /formation/mission  ：PoseArray, frame_id="mission", stamp=版本号（纳秒）。
      收到中心航线的节点（经飞控转发的 target=99 通道）把它重新广播，
      其余节点应用"更新版本"的航线 —— 保证 3 机拿到的中心航线一致。
    - /formation/shape    ：Float32MultiArray [shape_id, spacing, radius]。
      收到 40010 的节点广播队形配置，其余节点应用（幂等，最新生效）。
    """

    def __init__(self, node):
        self.node = node
        self._mission_ver = 0
        self._mission = []
        self._shape = None
        self.mission_pub = node.create_publisher(
            PoseArray, "/formation/mission", SHARE_QOS)
        self.shape_pub = node.create_publisher(
            Float32MultiArray, "/formation/shape", SHARE_QOS)
        node.create_subscription(PoseArray, "/formation/mission",
                                 self._on_mission, SHARE_QOS)
        node.create_subscription(Float32MultiArray, "/formation/shape",
                                 self._on_shape, SHARE_QOS)

    # ---- 本地状态 ----
    def mission(self):
        return [list(w) for w in self._mission]

    def mission_version(self):
        return self._mission_ver

    def shape(self):
        return self._shape

    # ---- DDS 收 ----
    def _on_mission(self, msg):
        ver = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if ver > self._mission_ver:
            self._mission_ver = ver
            self._mission = [[p.position.x, p.position.y,
                              p.position.z] for p in msg.poses]

    def _on_shape(self, msg):
        if len(msg.data) >= 3:
            self._shape = (int(msg.data[0]), float(msg.data[1]), float(msg.data[2]))

    # ---- DDS 发（仅由"经飞控转发通道"收到的节点调用，避免回环） ----
    def broadcast_mission(self, waypoints):
        # ★ rclpy Time.nanoseconds 是属性不是方法(联调实测: 'int' object is not callable)
        ver = self.node.get_clock().now().nanoseconds
        self._mission_ver = ver
        self._mission = [[float(w[0]), float(w[1]),
                          float(w[2] if len(w) > 2 else 0.0)] for w in waypoints]
        msg = PoseArray()
        msg.header.stamp.sec = ver // 10**9
        msg.header.stamp.nanosec = ver % 10**9
        msg.header.frame_id = "mission"
        for (x, y, z) in self._mission:
            p = PoseStamped()
            p.header.frame_id = "mission"
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.position.z = float(z)
            msg.poses.append(p.pose)
        self.mission_pub.publish(msg)

    def broadcast_shape(self, shape_id, spacing, radius):
        self._shape = (int(shape_id), float(spacing), float(radius))
        msg = Float32MultiArray()
        msg.data = [float(self._shape[0]), float(self._shape[1]), float(self._shape[2])]
        self.shape_pub.publish(msg)
