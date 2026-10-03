"""地面站指令通道 —— MAVLink 2 官方消息（v20 方言）。

两类传输：
  * GcsChannel（UDP）     地面站/测试工具直连节点 UDP 口（docs/28 §3: 14560+idx, 可选测试通道）
  * MavlinkChannel（ROS） ★ 经 MAVROS mavlink_source/mavlink_sink 的原始透传通道（与自家飞控对话:
                           发 setpoint/请求, 收飞控遥测; 2026-08-28 起节点【主动从飞控拉取航线】）

协议（docs/25 §2, 规范#1/#2/#3）：
  * ★ 2026-08-28 架构调整: 删除 sysid=99 编队指令通道（地面站实际经 MAVSDK 逐架直发飞控,
    target=1..N; 节点不再以 99 注册路由, 也不再接收 target=99 指令）。
  * 中心航线: 地面站逐架直发飞控(MAVSDK mission 上传) → 节点经 MavlinkChannel
    MISSION_REQUEST_LIST 从【自家飞控】拉取（PullMission, 方向与上传相反）。
  * 队形: 由启动参数 --shape 决定（40010 空中切换通道随 99 一并删除）。
  * MISSION 接收必须严格走状态机：COUNT → REQUEST(seq=0) → ITEM(seq) → ... → ACK，
    每次收到 ITEM 都要回 REQUEST 或 ACK，不可遗漏（规范#2）。

Mavlink 原始消息 ↔ mavros_msgs/Mavlink 的转换见 MavlinkCodec。
"""
import math
import os
import socket
import threading
import time

from pymavlink.dialects.v20 import common as mav   # ★ MAVLink 2 协议(v20 方言)

M_PER_DEG_LAT = 110540.0

SHAPE_IDS = {0: "line", 1: "column", 2: "wedge", 3: "grid", 4: "circle"}
SHAPE_NAMES = {v: k for k, v in SHAPE_IDS.items()}

# 队形切换为厂商保留 MAV_CMD(40000~40999, MAVLink 官方合法扩展区)；其余 40001/40002
# （旧自定义 hold/resume）按 docs/25 §6 不再由节点处理，直发飞控。
MAV_CMD_FORMATION_HOLD = 40001    # 旧自定义, 保留常量兼容
MAV_CMD_FORMATION_RESUME = 40002  # 旧自定义, 保留常量兼容
MAV_CMD_FORMATION_SHAPE = 40010   # param1=队形id(0..4) param2=间距 param3=半径

# mavros_router.cpp 中 mavlink_source/sink 的 QoS: QoS(1000).best_effort().durability_volatile()
MAVLINK_QOS = None  # 延迟到有 rclpy 时构造（模块级导入避免硬依赖）


def _qos_profile():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=1000)


def enu_offset(lat0, lon0, lat1, lon1):
    dy = (lat1 - lat0) * M_PER_DEG_LAT
    dx = (lon1 - lon0) * M_PER_DEG_LAT * math.cos(math.radians(lat0))
    return dx, dy


def math_cos(lat):
    return math.cos(math.radians(lat))


# =====================================================================
# MavlinkCodec —— pymavlink 消息 ↔ mavros_msgs/Mavlink（原始透传编解码）
# =====================================================================
class MavlinkCodec:
    """mavros_msgs/Mavlink ↔ pymavlink MAVLink2 帧 双向转换（docs/24 §4.0）。"""

    @staticmethod
    def to_ros(mav, pm_msg):
        """pymavlink 消息 → mavros_msgs/Mavlink（发 mavlink_sink 用）。"""
        from mavros_msgs.msg import Mavlink
        raw = pm_msg.pack(mav)               # MAVLink2 帧: fd,len,incompat,compat,seq,sysid,compid,msgid×3,payload,crc×2
        ln = raw[1]
        payload = raw[10:10 + ln]
        n = (ln + 7) // 8
        padded = payload + b"\x00" * (n * 8 - ln)
        m = Mavlink()
        m.magic = raw[0]
        m.len = ln
        m.incompat_flags = raw[2]
        m.compat_flags = raw[3]
        m.seq = raw[4]
        m.sysid = raw[5]
        m.compid = raw[6]
        m.msgid = raw[7] | (raw[8] << 8) | (raw[9] << 16)
        m.checksum = raw[10 + ln] | (raw[11 + ln] << 8)
        m.payload64 = [int.from_bytes(padded[i * 8:(i + 1) * 8], "little")
                       for i in range(n)]
        return m

    @staticmethod
    def from_ros(mav, ros_msg):
        """mavros_msgs/Mavlink → pymavlink 消息（收 mavlink_source 用）。

        ★ 防护(联调实测): 偶发坏帧(CRC 错/截断)必须返回 None, 不能让 MAVError
          kill 掉 rclpy 回调线程(否则节点崩溃)。"""
        try:
            ln = ros_msg.len
            payload = b"".join(int(v).to_bytes(8, "little")
                               for v in ros_msg.payload64)[:ln]
            header = bytes([0xfd, ln, ros_msg.incompat_flags, ros_msg.compat_flags,
                            ros_msg.seq, ros_msg.sysid, ros_msg.compid,
                            ros_msg.msgid & 0xFF, (ros_msg.msgid >> 8) & 0xFF,
                            (ros_msg.msgid >> 16) & 0xFF])
            crc = bytes([ros_msg.checksum & 0xFF, (ros_msg.checksum >> 8) & 0xFF])
            frame = header + payload + crc
            return mav.parse_char(frame)
        except Exception as e:
            print(f"[mavlink] 透传帧解码异常(忽略): {e}")
            return None


# =====================================================================
# MissionReceiver —— MISSION 接收状态机（传输无关, 规范#2, 可单测）
# =====================================================================
class MissionReceiver:
    """标准航点接收状态机：

        MISSION_COUNT → 回 MISSION_REQUEST_INT(seq=0)
        MISSION_ITEM_INT(seq) → 回 MISSION_REQUEST_INT(seq+1) 或收齐回 MISSION_ACK
        MISSION_CLEAR_ALL → 清空并回 MISSION_ACK
        （seq 乱序忽略；15s 无进展超时中止，防死等）

    send_fn(msg)：发送已编码的 pymavlink 消息（UDP 通道绑地址 / ROS 通道发 sink）。
    on_waypoints(wps)：收齐后回调宿主，返回 (accepted, reason)。
    """

    def __init__(self, send_fn, on_waypoints, ref,
                 timeout=15.0, passive=False):
        self.send_fn = send_fn
        self.on_waypoints = on_waypoints
        self.ref = ref
        self.timeout = float(timeout)
        self.passive = bool(passive)     # passive: 不回 REQUEST/ACK（主链路旁听模式）
        self.mav = mav.MAVLink(None)
        self.mav.srcSystem = 255          # 节点应答身份(2026-08-28 删除 99, 用标准 GCS 身份)
        self.mav.srcComponent = 1
        self.target = 255               # ★ 回执目标 = 指令来源 sysid(start 时更新)
        self.reset()

    def reset(self):
        self.active = False
        self.count = 0
        self.items = []                  # [(x, y, z), ...] 公共系 ENU (z=航点相对高)
        self.expected = 0
        self._last_t = 0.0

    # ---- 收发 ----
    def _req(self, seq):
        """回 MISSION_REQUEST_INT(seq)。★ target 必须是地面站来源 sysid(或 0 广播):
        若 target=99(自己)会被 MAVProxy/飞控再路由回 serial1 打转(联调实测坑)。"""
        if not self.passive:
            self.send_fn(self.mav.mission_request_int_encode(
                self.target, 1, seq, 0))

    def _ack(self, ok=True):
        if not self.passive:
            self.send_fn(self.mav.mission_ack_encode(
                self.target, 1,
                mav.MAV_MISSION_ACCEPTED if ok else mav.MAV_MISSION_INVALID, 0))

    # ---- 事件 ----
    def start(self, count, now, src=255):
        """MISSION_COUNT：开始新任务，回 REQUEST(seq=0)（规范#2）。

        src: 指令来源 sysid(地面站) —— 回执必须回给它, 不能回自己 99(会打转)。"""
        self.reset()
        self.active = True
        self.count = int(count)
        self.target = int(src)
        self._last_t = now
        self._req(0)
        return self.count

    def item(self, msg, now):
        """MISSION_ITEM_INT / MISSION_ITEM：校验 seq 并回 REQUEST(seq+1) 或最终 ACK。"""
        if not self.active:
            return None
        if msg.seq != self.expected:
            print(f"[gcs] 航点 seq 乱序(期望 {self.expected}, 收到 {msg.seq}), 忽略")
            return None
        self._last_t = now
        # ★ GLOBAL/GLOBAL_RELATIVE_ALT frame: ITEM_INT.x=纬度(degE7) y=经度(degE7)；
        #   ITEM(非INT) x/y 是 float 度（规范#2 兼容两种）
        if msg.get_type() == "MISSION_ITEM_INT":
            lat, lon = msg.x / 1e7, msg.y / 1e7
        else:
            lat, lon = float(msg.x), float(msg.y)
        dx, dy = enu_offset(self.ref[0], self.ref[1], lat, lon)
        # ★ 2026-08-28: 去掉安全边界保护(用户要求) —— 中心线不限距参考点距离
        # ★ 2026-08-28 修复: 保留航点高度 z(地面站上传 50m/100m 等), 节点按此高度飞,
        #   不再丢弃(否则实际飞行高度恒为 fly_z=2m, 与地面站设置不符)。
        #   航点格式 [(x, y, z) 公共系 ENU 米, z 相对高]。
        z = float(getattr(msg, "z", 0.0) or 0.0)
        self.items.append((dx, dy, z))
        self.expected += 1
        if self.expected < self.count:
            self._req(self.expected)         # 每次收 ITEM 必回（规范#2）
        else:
            accepted, reason = self.on_waypoints(self.items)
            self._ack(accepted)
            print(f"[gcs] 航点接收完成: {len(self.items)} 个 -> "
                  f"{[(round(w[0],1), round(w[1],1)) for w in self.items]} -> "
                  f"{'接受' if accepted else '拒绝(reason=' + str(reason) + ')'}")
            self.active = False
        return self.items

    def clear(self, now):
        """MISSION_CLEAR_ALL：清空航点并回 ACK。"""
        self.reset()
        ok = True
        try:
            accepted, reason = self.on_waypoints([])
            ok = bool(accepted)
        except Exception as e:
            print(f"[gcs] on_waypoints([]) 异常: {e}")
        self._ack(ok)
        print("[gcs] 航点已清空")

    def tick(self, now):
        """超时检测（规范#2 防死等）：15s 无进展 → 中止并回 ACK(拒绝)。"""
        if self.active and now - self._last_t > self.timeout:
            print(f"[gcs] 航点接收超时({self.timeout}s), 中止")
            self._ack(False)
            self.reset()


# =====================================================================
# PullMission —— 节点主动从飞控【下载】航线（2026-08-28 新增, 替代 99 转发）
# =====================================================================
class PullMission:
    """方向与 MissionReceiver(接收上传)相反:
    MISSION_REQUEST_LIST(target=飞控) → 飞控回 MISSION_COUNT → 逐点 MISSION_REQUEST_INT
    → 飞控回 MISSION_ITEM_INT → 收齐后 MISSION_ACK(ACCEPTED)。

    用途: 地面站经 MAVSDK 把中心线逐架直发飞控(target=1..N)后, 节点从【自家飞控】
    拉取航线。走 /uas{i} mavlink 通道(serial1), 与地面站 serial0 上传互不干扰
    (ArduPilot 每个串口独立 mission 协议状态机)。

    ★ seq=0 是 home 项, 不是航线航点(2026-08-28 修复"虚假单点航线"):
      ArduPilot AP_Mission 第 0 项固定存 home(AP_MISSION_FIRST_REAL_COMMAND=1),
      且解锁/EKF 定 home 时 write_home_to_storage() 会把 home 写回空任务 ——
      清空后只要解锁, 任务必然"复活"成 1 个航点(=本机 home, z=home 高)。
      旧版把 count=1 的 home 项当真航线 → 目标=自身位置 → 立即"全部到达"
      → 虚假自动降落(实测)。现丢弃 seq=0, count≤1 视为空航线。

    航点格式与 MissionReceiver 一致: [(x, y, z) 公共系 ENU 米, z=航点相对高]。
    由调用线程 wait() 阻塞等待; 消息经 MavlinkChannel._handle 喂给 feed()。
    """

    def __init__(self, send_fn, mav, fc_sysid, ref, timeout=8.0, on_speed=None):
        self.send_fn = send_fn
        self.mav = mav                     # 复用通道 MAVLink 对象(srcSystem=节点sysid)
        self.fc = int(fc_sysid)
        self.ref = ref
        self.timeout = float(timeout)
        self.on_speed = on_speed           # ★ DO_CHANGE_SPEED(178) 回调: on_speed(speed_m_s)
        self._ev = threading.Event()
        self._started = 0.0
        self.result = None                 # [(x,y,z) ENU] 或 None
        self._items = []
        self._total = None
        self._next_seq = 0                 # 下一个期望的飞控 seq(含 home 项 0)

    def start(self):
        self._items = []
        self._total = None
        self._next_seq = 0
        self.result = None
        self._ev.clear()
        self._started = time.time()
        self.send_fn(self.mav.mission_request_list_encode(self.fc, 1, 0))

    def wait(self, timeout=None):
        self._ev.wait(timeout if timeout is not None else self.timeout)
        return self.result

    def _req(self, seq):
        if os.environ.get("PULL_DEBUG"):
            print(f"[dbg] pull._req({seq})")
        self.send_fn(self.mav.mission_request_int_encode(self.fc, 1, seq, 0))

    def _finish(self, result):
        if os.environ.get("PULL_DEBUG"):
            print(f"[dbg] pull._finish result={'OK'+str(len(result)) if result is not None else 'None'}")
        self.result = result
        self._ev.set()

    def feed(self, msg, now):
        t = msg.get_type()
        if os.environ.get("PULL_DEBUG"):
            print(f"[dbg] pull.feed {t} seq={getattr(msg,'seq','-')} "
                  f"total={self._total} items={len(self._items)}")
        if t == "MISSION_COUNT":
            self._total = int(msg.count)
            self._items = []
            self._next_seq = 0
            if self._total == 0:
                # ★ count=0(飞控无航线): 直接空结果结束, 不要 _req(0)
                #   (否则 REQUEST_INT(0) 会引来残留 ITEM 造出"1 点=自己home"的假航点)
                self._finish([])
                return
            self._req(0)
        elif t in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            if self._total is None:
                return
            seq = int(msg.seq)
            if seq != self._next_seq:
                print(f"[pull] 航点 seq 乱序(期望 {self._next_seq}, 收到 {seq}), 忽略")
                return
            self._next_seq += 1
            # ★ 丢弃 seq=0(home 项, ArduPilot 任务存储约定, 见类 docstring):
            #   count=1(纯 home)→ 空航线; count=1+k → 只保留 k 个真航点。
            if seq >= 1:
                # ★ 2026-08-28: DO_CHANGE_SPEED(178) 是速度指令, 不是航点 ——
                #   x/y/z=0, 不当航点入列(否则在参考点附近造假航点);
                #   把 param2(速度 m/s) 经 on_speed 回调上报(地面站 mission 首条)。
                cmd = int(getattr(msg, "command", 0) or 0)
                if cmd == mav.MAV_CMD_DO_CHANGE_SPEED:
                    if self.on_speed is not None:
                        try:
                            spd = float(getattr(msg, "param2", 0.0) or 0.0)
                            if spd > 0:
                                self.on_speed(spd)
                        except (TypeError, ValueError):
                            pass
                    # 协议照常推进: 下方统一回 MISSION_REQUEST_INT(seq+1)/ACK
                else:
                    if t == "MISSION_ITEM_INT":
                        lat, lon = msg.x / 1e7, msg.y / 1e7
                    else:
                        lat, lon = float(msg.x), float(msg.y)
                    dx, dy = enu_offset(self.ref[0], self.ref[1], lat, lon)
                    # ★ 2026-08-28: 去掉安全边界保护(用户要求)
                    # ★ 2026-08-28 修复: 保留航点高度 z(地面站上传 50m/100m 等)
                    z = float(getattr(msg, "z", 0.0) or 0.0)
                    self._items.append((dx, dy, z))
            if self._next_seq < self._total:
                self._req(self._next_seq)
            else:
                self.send_fn(self.mav.mission_ack_encode(
                    self.fc, 1, mav.MAV_MISSION_ACCEPTED, 0))
                self._finish(self._items)
        # 其余消息忽略

    def tick(self, now):
        """超时: start 后 timeout 内未完成 → 结果置 None。"""
        if self._started and not self._ev.is_set() and now - self._started > self.timeout:
            print(f"[pull] 拉取航线超时({self.timeout}s), 放弃")
            self._finish(None)


# =====================================================================
# GcsChannel —— UDP 直连通道（测试通道, 可选保留）
# =====================================================================
class GcsChannel:
    """UDP 监听线程: 收地面站官方 MAVLink → 调宿主回调；回心跳/回执。

    passive=True: 只收不发（主链路旁听）。accept_extra: 额外接受的 target_system。
    """

    def __init__(self, node, port, ref_lat, ref_lon,
                 node_sysid=255, node_compid=1,
                 passive=False, accept_extra=()):
        self.node = node
        self.port = int(port)
        self.ref = (float(ref_lat), float(ref_lon))
        self.sysid = int(node_sysid)
        self.compid = int(node_compid)
        self.passive = bool(passive)
        self.accept_extra = tuple(accept_extra)
        self.stop = False
        self.connected = False
        self.clients = {}
        self._hb_t = 0.0
        self._last_addr = None          # ★ 最近一条消息的客户端地址(回执用, 联调实测修复)
        self.mav = mav.MAVLink(None)
        self.mav.srcSystem = self.sysid
        self.mav.srcComponent = self.compid
        # ★ MissionReceiver 回执必须发回指令来源地址(addr=None 会 sendto 异常, 实测坑)
        self.mission = MissionReceiver(
            lambda msg: self._send(msg, self._last_addr),
            self.node.on_gcs_waypoints, self.ref,
            passive=self.passive)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(0.2)
        self.thread = threading.Thread(target=self._run, daemon=True)
        print(f"[gcs] 地面站通道(UDP): udp:0.0.0.0:{self.port} "
              f"(sysid={self.sysid}, 官方 MAVLink 消息) 等指令...")

    # ---------- 收发 ----------
    def _send(self, msg, addr=None):
        try:
            self.sock.sendto(msg.pack(self.mav), addr)
        except OSError:
            pass

    def _heartbeat(self, now):
        if self.passive:
            return
        stale = [a for a, t in self.clients.items() if now - t > 60.0]
        for a in stale:
            del self.clients[a]
        for addr in list(self.clients):
            self._send(self.mav.heartbeat_encode(2, 8, 0, 0, 4), addr)

    def _run(self):
        while not self.stop:
            now = time.time()
            if now - self._hb_t > 1.0:
                self._hb_t = now
                self._heartbeat(now)
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                continue
            if not data:
                continue
            self.clients[addr] = time.time()
            self._last_addr = addr     # ★ 回执目标地址(修复 sendto(None) 实测坑)
            # ★ 畸形包防护(联调实测: 偶发坏 CRC 包会 kill 通道线程, parse 必须兜住)
            try:
                msg = self.mav.parse_char(data)
            except Exception as e:
                print(f"[gcs] 解析异常(忽略坏包): {e}")
                continue
            if msg is None:
                continue
            try:
                self._handle(msg, addr, now)
            except Exception as e:
                print(f"[gcs] 处理消息异常: {e}")
                import traceback
                traceback.print_exc()

    # ---------- 消息分发 ----------
    def _accepts(self, target):
        return target in (0, self.sysid) or target in self.accept_extra

    def _handle(self, msg, addr, now=None):
        now = now or time.time()
        t = msg.get_type()
        if t == "HEARTBEAT":
            if not self.connected:
                self.connected = True
                print(f"[gcs] ★ 地面站已连接 (system={msg.get_srcSystem()})")
        elif t == "MISSION_COUNT":
            if not self._accepts(msg.target_system):
                return
            self.mission.start(msg.count, now, msg.get_srcSystem())
        elif t in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            if not self._accepts(msg.target_system):
                return
            self.mission.item(msg, now)
        elif t == "MISSION_CLEAR_ALL":
            if not self._accepts(msg.target_system):
                return
            self.mission.clear(now)
        elif t == "COMMAND_LONG":
            if not self._accepts(msg.target_system):
                return
            self._handle_command(msg)

    # ---------- COMMAND_LONG 分发（docs/25 §2：只有 40010 进 ROS） ----------
    def _handle_command(self, msg):
        """COMMAND_LONG 分发（docs/25 §2：只有 40010 进 ROS）。

        ★ ACK 构造（联调实测修复）: 本版 pymavlink 签名
          command_ack_encode(command, result, progress, result_param2,
                             target_system, target_component)
          target 必须显式回给指令来源(地面站 sysid), 不能默认 0 或误传 command。"""
        cmd = msg.command
        src = msg.get_srcSystem()
        if cmd == MAV_CMD_FORMATION_SHAPE:                 # 40010 编队切换 ★
            shape = SHAPE_IDS.get(int(msg.param1))
            if shape is None:
                print(f"[gcs] 未知队形 id={int(msg.param1)}, 忽略")
            else:
                print(f"[gcs] 编队切换 -> {shape} (spacing={msg.param2} radius={msg.param3})")
                self.node.on_gcs_shape(shape, msg.param2, msg.param3)
            self._send(self.mav.command_ack_encode(
                cmd, mav.MAV_RESULT_ACCEPTED, 0, 0, src, 1))
            return
        # docs/25 §6: 起飞/降落/暂停/继续 已改直发飞控(target=1..N)，节点不再处理
        if cmd in (mav.MAV_CMD_NAV_TAKEOFF, mav.MAV_CMD_NAV_LAND,
                   mav.MAV_CMD_DO_PAUSE_CONTINUE,
                   MAV_CMD_FORMATION_HOLD, MAV_CMD_FORMATION_RESUME):
            print(f"[gcs] 命令 {cmd}: 已改直发飞控(target=1..N), 节点不处理")
            self._send(self.mav.command_ack_encode(
                cmd, mav.MAV_RESULT_UNSUPPORTED, 0, 0, src, 1))
            return
        # 常规询问命令(REQUEST_MESSAGE 511/512 等)静默回 UNSUPPORTED
        if cmd in (510, 511, 512, 519, 520, 521):
            self._send(self.mav.command_ack_encode(
                cmd, mav.MAV_RESULT_UNSUPPORTED, 0, 0, src, 1))
            return
        print(f"[gcs] 未识别命令 {cmd}, 忽略")
        self._send(self.mav.command_ack_encode(
            cmd, mav.MAV_RESULT_UNSUPPORTED, 0, 0, src, 1))

    def start(self):
        self.thread.start()

    def stop_channel(self):
        self.stop = True
        try:
            self.sock.close()
        except OSError:
            pass


# =====================================================================
# MavlinkChannel —— ★ 经 MAVROS mavlink_source/sink 的原始透传通道（主链路）
# =====================================================================
class MavlinkChannel:
    """节点与【自家飞控】对话的透传通道（docs/24 §4.0, docs/28 §4.3 修订）:

        节点 ──mavlink_sink──► MAVROS ──serial1──► 飞控（发请求/拉取/setpoint 指令）
        飞控 ──serial1──► MAVROS ──mavlink_source──► 本通道（收遥测/航点回包）

    ★ 2026-08-28 架构调整: 删除 sysid=99 心跳注册与 target=99 指令通道。
      节点与自家飞控经本通道对话(发请求/收遥测); 中心航线改为节点【主动拉取】
      (PullMission: MISSION_REQUEST_LIST → COUNT → ITEM → ACK, 见类定义)。

    ★★ 话题路径（2026-08-27 实机联调实测发现）：
      本版 MAVROS 的原始透传话题是 **全局 /uas{tgt_system}/mavlink_source|sink**
      （mavros_node.cpp: uas_url = "/uas{tgt_system}"，绝对路径不受 __ns 影响），
      **不是** docs/24 图里的 /uav{i}/mavros/mavlink_source（该路径无人发布，空）。
      调用方必须传 topic_base = f"/uas{1+idx}"（idx=0 → /uas1）。

    QoS 必须与 mavros_router.cpp 一致: BEST_EFFORT + VOLATILE（深度 1000）。
    """

    def __init__(self, node, topic_base, ref_lat, ref_lon,
                 node_sysid=255, node_compid=1,
                 passive=False, accept_extra=()):
        from mavros_msgs.msg import Mavlink
        self.node = node
        self.topic_base = topic_base
        self.ref = (float(ref_lat), float(ref_lon))
        self.sysid = int(node_sysid)
        self.compid = int(node_compid)
        self.passive = bool(passive)
        self.accept_extra = tuple(accept_extra)
        self.connected = False
        self._miss_t = 0.0
        self.pull = None                 # 进行中的航线拉取(PullMission, 一次一个)
        self.mav = mav.MAVLink(None)
        self.mav.srcSystem = self.sysid
        self.mav.srcComponent = self.compid
        self.sink_pub = node.create_publisher(Mavlink, f"{topic_base}/mavlink_sink",
                                              _qos_profile())
        node.create_subscription(Mavlink, f"{topic_base}/mavlink_source",
                                 self._on_source, _qos_profile())
        self.mission = MissionReceiver(self._send, self.node.on_gcs_waypoints,
                                       self.ref, passive=self.passive)
        print(f"[mavlink] 透传通道: {topic_base}/mavlink_source|sink (sysid={self.sysid})")

    # ---------- 收发 ----------
    def _send(self, msg):
        """pymavlink 消息 → mavlink_sink（原始透传回飞控）。"""
        try:
            self.sink_pub.publish(MavlinkCodec.to_ros(self.mav, msg))
        except Exception as e:
            print(f"[mavlink] 发送异常: {e}")

    def pull_mission(self, fc_sysid, timeout=8.0):
        """★ 主动从自家飞控【下载】航线(阻塞, 需在非 rclpy spin 线程调用)。

        地面站把中心线逐架直发飞控(MAVSDK, target=1..N)后, 节点拉取自己的飞控上的航点;
        与地面站 serial0 上传互不干扰(飞控每串口独立 mission 协议状态机)。
        返回 [(x, y, z) 公共系ENU米, z=航点相对高] 或 None(超时/失败)。

        ★ 防部分上传竞态(2026-08-28 实测): 节点已授权时周期拉取可能与地面站
          正在进行中的上传撞车 —— 飞控任务存储边传边写, 下载读到残缺航线
          (实测 3 点航线拉到 2 点, 按残缺航线飞+提前降落)。措施: 结果复核 ——
          1.5s 后重拉一次, 两次一致才返回; 不一致以新结果为准再复核(最多 3 轮)。
          上传完成后两次读取必一致; 空列表同理稳定。"""
        if os.environ.get("PULL_DEBUG"):
            print(f"[dbg] pull_mission start fc={fc_sysid}")
        wps = self._pull_once(fc_sysid, timeout)
        for _ in range(2):
            if wps is None:
                break
            time.sleep(1.5)
            again = self._pull_once(fc_sysid, timeout)
            if again is None:
                break                     # 复核失败: 保守采用上次结果
            if again == wps:
                break                     # 两次一致 → 稳定可信
            wps = again                   # 残缺→完整: 以新结果为准继续复核
        if os.environ.get("PULL_DEBUG"):
            print(f"[dbg] pull_mission done result="
                  f"{'OK'+str(len(wps)) if wps is not None else 'None'}")
        return wps

    def _pull_once(self, fc_sysid, timeout):
        """单次完整拉取(REQUEST_LIST → COUNT → ITEM×n → ACK)。"""
        if self.pull is not None:
            self.pull._ev.set()          # 上一轮尽快结束(结果丢弃)
        p = PullMission(self._send, self.mav, int(fc_sysid), self.ref,
                        timeout=float(timeout),
                        on_speed=getattr(self.node, "on_gcs_speed", None))
        self.pull = p
        p.start()
        wps = p.wait(timeout + 2.0)
        if self.pull is p:
            self.pull = None
        return wps

    def tick(self, now=None):
        """宿主主循环每拍调用：拉取超时 + Mission 超时。"""
        now = now if now is not None else time.time()
        if self.pull is not None:
            self.pull.tick(now)
        self.mission.tick(now)

    # ---------- 接收 ----------
    def _on_source(self, ros_msg):
        # ★ 2026-08-28 修复: 解析必须用【新 MAVLink 对象】—— 共享 self.mav 的 parse_char
        #   缓冲会被前一帧残留污染, 导致拉取第一次成功、后续 COUNT/ITEM 解析失败(静默丢弃)。
        #   self.mav 只用于编码(pack), 不用于解析。
        pm = MavlinkCodec.from_ros(mav.MAVLink(None), ros_msg)
        if pm is None:
            return
        if os.environ.get("PULL_DEBUG"):
            t = pm.get_type()
            if t in ("MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_REQUEST_LIST", "MISSION_ACK"):
                print(f"[dbg] _on_source {t} target={getattr(pm,'target_system','-')} "
                      f"src={pm.get_srcSystem()} seq={getattr(pm,'seq','-')} "
                      f"count={getattr(pm,'count','-')} pull={'Y' if self.pull is not None else 'N'}")
        if self._accepts(getattr(pm, "target_system", -1)):
            self._handle(pm, time.time())

    def _accepts(self, target):
        return target in (0, self.sysid) or target in self.accept_extra

    def _handle(self, msg, now):
        t = msg.get_type()
        # ★ 拉取进行中: COUNT/ITEM 归 PullMission(与 MissionReceiver 方向相反, 先拦截)
        if self.pull is not None and t in ("MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_ITEM"):
            if os.environ.get("PULL_DEBUG"):
                print(f"[dbg] _handle→pull.feed {t}")
            self.pull.feed(msg, now)
            return
        if t == "HEARTBEAT":
            if not self.connected:
                self.connected = True
                print(f"[mavlink] ★ 地面站经飞控转发链路已连通 (system={msg.get_srcSystem()})")
        elif t == "MISSION_COUNT":
            self.mission.start(msg.count, now, msg.get_srcSystem())
        elif t in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            self.mission.item(msg, now)
        elif t == "MISSION_CLEAR_ALL":
            self.mission.clear(now)
        elif t == "COMMAND_LONG":
            self._handle_command(msg)

    def _handle_command(self, msg):
        """COMMAND_LONG: 只有 40010 进 ROS；回执 target 显式回给指令来源(联调实测修复)。

        本版 pymavlink 签名: command_ack_encode(command, result, progress,
        result_param2, target_system, target_component)。"""
        cmd = msg.command
        src = msg.get_srcSystem()
        if cmd == MAV_CMD_FORMATION_SHAPE:                 # 40010 ★
            shape = SHAPE_IDS.get(int(msg.param1))
            if shape is None:
                print(f"[mavlink] 未知队形 id={int(msg.param1)}, 忽略")
            else:
                print(f"[mavlink] 编队切换 -> {shape} "
                      f"(spacing={msg.param2} radius={msg.param3})")
                self.node.on_gcs_shape(shape, msg.param2, msg.param3)
            self._send(self.mav.command_ack_encode(
                cmd, mav.MAV_RESULT_ACCEPTED, 0, 0, src, 1))
            return
        # 直发飞控的指令 / 常规询问：回 UNSUPPORTED（节点不处理）
        self._send(self.mav.command_ack_encode(
            cmd, mav.MAV_RESULT_UNSUPPORTED, 0, 0, src, 1))

    def stop_channel(self):
        pass
