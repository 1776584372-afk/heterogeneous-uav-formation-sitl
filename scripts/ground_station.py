#!/usr/bin/env python3
"""Python 地面站工具 —— 完全复刻 C++ UAVMapSystem(MavsdkCommandApi) 的消息格式。

消息格式提取自: ~/Desktop/02_代码项目/sys/src/link/mavsdk/mavsdkcommandapi.cpp
(C++ MAVSDK 地面站的发送侧), 用 pymavlink 等价实现。

已复刻的消息(与 C++ 一致):
  * COMMAND_LONG (msg 76)          — 起飞/降落/悬停/继续/返航等
  * SET_MODE (msg 11)              — 切飞行模式
  * SET_POSITION_TARGET_LOCAL_NED (msg 84) — GUIDED 位置/速度控制
  * SET_ATTITUDE_TARGET (msg 150)  — 姿态四元数控制
  * PARAM_SET (msg 23)             — 写参数
  * REQUEST_DATA_STREAM (msg 66)   — 请求数据流(旧)
  * SET_MESSAGE_INTERVAL (cmd 511) — 请求指定消息频率(新)
  * REQUEST_MESSAGE (cmd 512)      — 请求指定消息
  * MISSION_ITEM_INT (msg 73)      — 航点(上传/下载)
  * RC_CHANNELS_OVERRIDE (msg 70)  — 遥控通道覆盖

用法:
  python3 ground_station.py --host 127.0.0.1 --port 14550 \
      --target 1 --all 3 start
  python3 ground_station.py --port 14550 --all 3 params --shape line --spacing 1.5
"""
import argparse
import math
import socket
import struct
import sys
import threading
import time

from pymavlink.dialects.v20 import common as mav   # ★ MAVLink 2 协议(v20 方言)

# ---- 常量(与 C++ 地面站一致) ----
SHAPE_IDS = {"line": 0, "column": 1, "wedge": 2, "grid": 3, "circle": 4}
MAV_CMD_FORMATION_SHAPE = 40010     # 编队切换(特殊指令, target=99, docs/28 §4.2)
REF_LAT, REF_LON = 30.893000, 120.087000          # 浙江湖州基线
M_PER_DEG_LAT = 111319.49

# ★ 规范#16: GUIDED 下地面站不得发位置设定点(位置只归 ROS)。这些指令被过滤,
#   除非显式 --force。允许: SET_MODE / COMMAND_LONG(起降返航暂停) / MISSION_* / PARAM_SET。
BLOCKED_POSITION_CMDS = {"goto", "setpos", "setposition", "attitude", "setatt",
                         "rc", "rcoverride"}

# ---- 接收队列(单线程锁) ----
_rx_lock = threading.Lock()
_rx_queue = []


class GroundStation:
    """Python 版地面站: 发 MAVLink 官方消息, 与 C++ UAVMapSystem 消息格式一致。"""

    def __init__(self, host, port, listen, sysid=255, target=1, force=False):
        self.dst = (host, int(port))
        self.sysid = int(sysid)          # 地面站自身 sysid(C++ 默认 255)
        self.target = int(target)        # 目标飞控 sysid
        self.force = bool(force)         # 绕过位置设定点过滤(规范#16, 仅调试)
        self.compid = 1                  # 地面站组件(与 C++ addr.component_id 一致)
        self.mav = mav.MAVLink(None)     # MAVLink 2
        self.mav.srcSystem = self.sysid
        self.mav.srcComponent = self.compid
        self.waypoints = []              # 本地航点缓存 (x, y) ENU
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", int(listen)))
        self.sock.settimeout(0.2)
        threading.Thread(target=self._recv_loop, daemon=True).start()
        # ★ 心跳注册: MAVProxy 需要先收到客户端 HEARTBEAT 才把它加入转发列表,
        #   否则地面站发的命令不被转发(实测: 先心跳后命令才通)
        for _ in range(3):
            self._send(self.mav.heartbeat_encode(2, 8, 0, 0, 0))
            time.sleep(0.2)
        print(f"[gs] 目标 udp:{host}:{port} (sysid={sysid}) target={target} "
              f"本端监听 :{listen} (已心跳注册)")

    # ---------------- 收发 ----------------
    def _send(self, msg):
        self.sock.sendto(msg.pack(self.mav), self.dst)

    def _recv_loop(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            msg = self.mav.parse_char(data)
            if msg is None:
                continue
            with _rx_lock:
                _rx_queue.append(msg)

    def _wait_type(self, mtype, timeout=5.0):
        """等一条指定类型消息(轮询队列)。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with _rx_lock:
                for i, m in enumerate(_rx_queue):
                    if m.get_type() == mtype:
                        return _rx_queue.pop(i)
            time.sleep(0.05)
        return None

    def _wait_ack(self, command, timeout=5.0):
        """等一条 COMMAND_ACK 且 command 字段匹配(联调实测修复: 过滤 REQUEST_MESSAGE
        等常规询问的 UNSUPPORTED 回执噪声, 只认目标命令的回执)。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with _rx_lock:
                for i, m in enumerate(_rx_queue):
                    if m.get_type() == "COMMAND_ACK" and m.command == command:
                        return _rx_queue.pop(i)
            time.sleep(0.05)
        return None

    # ---------------- COMMAND_LONG (msg 76) — 与 C++ sendCommandLong 一致 ----------------
    def command_long(self, command, params, target=None):
        """COMMAND_LONG: target_system, target_component, command, confirmation=0, param1..7

        target=None → 用 self.target（基本指令 target=1..N 直发飞控）;
        特殊指令(上传航点/编队切换)传 target=99 → 飞控转发进 ROS（docs/25 §2）。
        """
        tgt = self.target if target is None else int(target)
        p = list(params) + [0.0] * (7 - len(params))
        # ★ 按 command 字段过滤回执(联调实测修复: 512/511 常规询问的 UNSUPPORTED
        #   回执会混入, 只认本命令的 COMMAND_ACK)
        # ★★ 等待放宽到 12s(联调实测: target=99 经飞控转发回程被限速, 回执延迟
        #   2~6s 才到; 3s 超时必然错过)
        ack = None
        for _ in range(2):
            self._send(self.mav.command_long_encode(
                tgt, 1, command, 0, *p))
            ack = self._wait_ack(command, timeout=12.0)
            if ack is not None:
                break
        if ack is None:
            print(f"[gs] 命令 {command}(target={tgt}): 无回执")
        elif ack.result != mav.MAV_RESULT_ACCEPTED:
            print(f"[gs] 命令 {command}(target={tgt}): 拒绝 result={ack.result}")
        else:
            print(f"[gs] 命令 {command}(target={tgt}): 接受")
        return ack

    # ---------------- SET_MODE (msg 11) — 与 C++ sendSetMode 一致 ----------------
    # ArduPilot custom_mode 枚举(与 C++ 地面站一致): STABILIZE=0 ACRO=1 ALT_HOLD=2
    # AUTO=3 GUIDED=4 LOITER=5 RTL=6 CIRCLE=7 LAND=9 ...
    MODE_IDS = {"STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
                "LOITER": 5, "RTL": 6, "CIRCLE": 7, "LAND": 9, "BRAKE": 17,
                "THROW": 18, "AVOID_ADSB": 19, "GUIDED_NOGPS": 20}

    def set_mode(self, custom_mode, base_mode=1):
        # ★ base_mode 必须 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED(1)!
        #   ArduPilot 只认 base_mode=1 时用 custom_mode; base_mode=0 会被忽略
        #   (2026-08 实测: 用 0 发的 SET_MODE 飞控完全不处理, mavutil/mavros 都用 1)
        """SET_MODE: target_system, base_mode, custom_mode(支持名称或数值)"""
        if isinstance(custom_mode, str):
            cm = self.MODE_IDS.get(custom_mode.upper())
            if cm is None:
                print(f"[!] 未知模式 {custom_mode}, 可选: {sorted(self.MODE_IDS)}")
                return
            custom_mode = cm
        self._send(self.mav.set_mode_encode(self.target, base_mode, custom_mode))
        print(f"[gs] SET_MODE target={self.target} custom_mode={custom_mode}")

    # ---------------- 位置/姿态控制 — 与 C++ 一致 ----------------
    def set_position_target_local_ned(self, x, y, z, vx=0, vy=0, vz=0,
                                      yaw=0.0, frame=8, type_mask=0b0000111111000111):
        """SET_POSITION_TARGET_LOCAL_NED (msg 84): GUIDED 位置控制。
        frame=8=MAV_FRAME_LOCAL_NED, type_mask 默认忽略速度/加速度。"""
        self._send(self.mav.set_position_target_local_ned_encode(
            0, self.target, 1, frame, type_mask,
            x, y, z, vx, vy, vz, 0, 0, 0, yaw, 0.0))
        print(f"[gs] SET_POSITION_TARGET_LOCAL_NED ({x},{y},{z})")

    def set_attitude_target(self, q, roll_rate=0, pitch_rate=0, yaw_rate=0, thrust=0.5):
        """SET_ATTITUDE_TARGET (msg 150): 姿态四元数控制。"""
        self._send(self.mav.set_attitude_target_encode(
            0, self.target, 1, 0b00000011, q, roll_rate, pitch_rate, yaw_rate, thrust))
        print(f"[gs] SET_ATTITUDE_TARGET q={q}")

    def rc_channels_override(self, ch1=0, ch2=0, ch3=0, ch4=0,
                             ch5=0, ch6=0, ch7=0, ch8=0):
        """RC_CHANNELS_OVERRIDE (msg 70): 遥控通道覆盖(0=释放)。"""
        self._send(self.mav.rc_channels_override_encode(
            self.target, 1, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8))
        print(f"[gs] RC_OVERRIDE ch1..8 = {ch1},{ch2},{ch3},{ch4},{ch5},{ch6},{ch7},{ch8}")

    # ---------------- PARAM_SET (msg 23) — 与 C++ sendParamSet 一致 ----------------
    def param_set(self, param_id, value):
        """PARAM_SET: target_system, target_component, param_id, param_value(float), type"""
        self._send(self.mav.param_set_encode(
            self.target, 1, param_id.encode(), float(value),
            mav.MAV_PARAM_TYPE_REAL32))
        print(f"[gs] PARAM_SET {param_id}={value} -> sysid={self.target}")

    def params(self, shape=None, spacing=None, radius=None, yaw=None,
               n=None, fly_z=None, v_des=None, drone_idx=None):
        """写队形/编号参数(每架飞控=参数仓库)。"""
        if isinstance(shape, str):
            if shape not in SHAPE_IDS:
                print(f"[!] 未知队形 {shape}, 可选: {sorted(SHAPE_IDS)}")
                return
            shape = SHAPE_IDS[shape]
        kwargs = {"FORM_SHAPE": shape, "FORM_SPAC": spacing, "FORM_RADI": radius,
                  "FORM_YAW": yaw, "N_DRONES": n, "FLY_Z": fly_z,
                  "V_DES": v_des, "DRONE_IDX": drone_idx}
        for k, v in kwargs.items():
            if v is not None:
                self.param_set(k, v)

    # ---------------- 数据流请求 — 与 C++ 一致 ----------------
    def request_data_stream(self, stream_id, rate_hz, start=True):
        """REQUEST_DATA_STREAM (msg 66, 旧): stream_id, rate, start=1/0"""
        self._send(self.mav.request_data_stream_encode(
            self.target, 1, stream_id, rate_hz, 1 if start else 0))
        print(f"[gs] REQUEST_DATA_STREAM id={stream_id} rate={rate_hz}Hz")

    def set_message_interval(self, msg_id, hz):
        """SET_MESSAGE_INTERVAL (cmd 511, 新): param1=msg_id, param2=interval_us"""
        interval_us = int(1e6 / hz) if hz > 0 else -1
        self.command_long(511, [float(msg_id), float(interval_us), 0, 0, 0, 0, 0])

    def request_message(self, msg_id):
        """REQUEST_MESSAGE (cmd 512): param1=msg_id(如 148=AUTOPILOT_VERSION)"""
        self.command_long(512, [float(msg_id), 0, 0, 0, 0, 0, 0])

    # ---------------- 航点 (MISSION_ITEM_INT msg 73) — 与 C++ uploadMission 一致 ----------------
    def enu_to_latlon(self, x, y):
        lon = REF_LON + x / (M_PER_DEG_LAT * math.cos(math.radians(REF_LAT)))
        lat = REF_LAT + y / M_PER_DEG_LAT
        return int(lat * 1e7), int(lon * 1e7)

    def mission_send(self, wps, target=None):
        """MISSION_COUNT → 等 REQUEST_INT(51) → MISSION_ITEM_INT → 等 ACK。
        wps: [(x, y), ...] 全部作为 NAV_WAYPOINT(16) 上传。
        target=None → self.target（下载航点 target=1..N 直发飞控）;
        上传【编队几何中心航线】用 target=99（docs/28 §6.3, 特殊指令进 ROS）。"""
        self._mission_send_items([(16, x, y) for x, y in wps], target=target)

    def upload_center(self, wps):
        """★ 上传编队几何中心航线（2026-08-28 改版: 逐架直发飞控 target=1..N,
        节点经 PullMission 从自家飞控拉取; 原 target=99 转发通道已删除）。
        航点是虚拟中心轨迹, 每机目标 = 中心 + 自己槽位。"""
        ok = self.mission_send(wps, target=None)
        print(f"[gs] 中心航线已上传: {len(wps)} 个航点 -> sysid={self.target} (直发飞控, 节点拉取)")
        return ok

    def mission_send_with_takeoff(self, wps, takeoff_z=20.0):
        """上传航线: 第一个点是 NAV_TAKEOFF(22)(起飞点), 其余是 NAV_WAYPOINT(16)。
        对应方案B: 起飞点标记为起飞指令, drone_node 会跳过 22(中心线不含起飞点),
        飞控 AUTO 模式会先执行起飞。"""
        items = [(22, 0.0, 0.0)] + [(16, x, y) for x, y in wps]
        self._mission_send_items(items, takeoff_z)

    def _mission_send_items(self, items, takeoff_z=20.0, target=None):
        """通用航点上传: items=[(command, x, y), ...]。
        target=None → self.target；特殊指令(中心航线)传 99。"""
        tgt = self.target if target is None else int(target)
        self._send(self.mav.mission_count_encode(
            tgt, 1, len(items), 0))
        print(f"[gs] 航点任务: 共 {len(items)} 个 -> sysid={tgt}")
        for i, (command, x, y) in enumerate(items):
            # ★★ 等待放宽(联调实测: target=99 经飞控转发回程被限速, 每拍回执延迟
            #   3~7s; 旧 3s 超时必然中断握手)
            req = self._wait_type("MISSION_REQUEST_INT", timeout=12.0)
            if req is None:
                # 兼容旧飞控可能发 MISSION_REQUEST(40)
                req = self._wait_type("MISSION_REQUEST", timeout=5.0)
            if req is None:
                print(f"[!] 等飞控请求 seq={i} 超时, 中断")
                return False
            lat, lon = self.enu_to_latlon(x, y)
            z = takeoff_z if command == 22 else 2.0
            # ★ MAVLink GLOBAL frame: x=纬度(degE7), y=经度(degE7)
            self._send(self.mav.mission_item_int_encode(
                tgt, 1, i,
                mav.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command,          # 22=NAV_TAKEOFF, 16=NAV_WAYPOINT
                0, 1, 0, 0, 0, 0, lat, lon, z, 0))
        ack = self._wait_type("MISSION_ACK", timeout=12.0)
        if ack is None:
            print("[!] 等 MISSION_ACK 超时")
            return False
        print(f"[gs] 航点回执(target={tgt}): "
              f"{'接受' if ack.type == mav.MAV_MISSION_ACCEPTED else '拒绝 type=' + str(ack.type)}")
        return True

    def mission_clear(self):
        self._send(self.mav.mission_clear_all_encode(self.target, 1))
        ack = self._wait_type("MISSION_ACK", timeout=3.0)
        print(f"[gs] 清空航点: {'接受' if ack and ack.type == mav.MAV_MISSION_ACCEPTED else '无回执'}"
              if ack else "[gs] 清空航点: 无回执")
        return bool(ack)

    # ---------------- 命令封装(与 C++ Action 插件语义一致) ----------------
    def start(self, takeoff_z=20.0):
        """起飞(完整三步, 与 MAVSDK Action::takeoff 语义一致):
        ① 解锁 MAV_CMD_COMPONENT_ARM_DISARM(400, param1=1)
        ② 切 GUIDED (SET_MODE custom_mode=4)
        ③ 起飞 MAV_CMD_NAV_TAKEOFF(22, param7=高度)
        只发 TAKEOFF 不解锁/不切 GUIDED 会被 PreArm 拒绝。"""
        print("[gs] 命令: start(解锁+GUIDED+起飞)")
        self.command_long(mav.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0, 0, 0, 0, 0, 0])
        time.sleep(0.3)
        self.set_mode("GUIDED")
        time.sleep(0.3)
        print(f"[gs] 命令: takeoff -> MAV_CMD_NAV_TAKEOFF(22) 高={takeoff_z}m")
        self.command_long(mav.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, takeoff_z])

    def land(self):
        print("[gs] 命令: land(降落) -> MAV_CMD_NAV_LAND(21)")
        self.command_long(mav.MAV_CMD_NAV_LAND, [0] * 7)

    def rtl(self):
        print("[gs] 命令: rtl(返航) -> MAV_CMD_NAV_RETURN_TO_LAUNCH(20)")
        self.command_long(mav.MAV_CMD_NAV_RETURN_TO_LAUNCH, [0] * 7)

    def hold(self):
        print("[gs] 命令: hold(悬停) -> MAV_CMD_DO_PAUSE_CONTINUE(193,param1=0)")
        self.command_long(mav.MAV_CMD_DO_PAUSE_CONTINUE, [0, 0, 0, 0, 0, 0, 0])

    def resume(self):
        print("[gs] 命令: resume(继续) -> MAV_CMD_DO_PAUSE_CONTINUE(193,param1=1)")
        self.command_long(mav.MAV_CMD_DO_PAUSE_CONTINUE, [1, 0, 0, 0, 0, 0, 0])

    # ---------------- 控制权（docs/28 §5, docs/27 §3.1 准备动作, 规范#17）----------------
    def auth(self):
        """授权控制: 解锁(400,p1=1) + 切 GUIDED（起飞前八步第 6/7 步, docs/28 §6.1）。

        地面站是唯一 SET_MODE 权威; 切 GUIDED = 把方向盘交给 ROS(规范#11)。"""
        print(f"[gs] 授权控制 target={self.target}: 解锁 + setmode GUIDED")
        self.command_long(mav.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0, 0, 0, 0, 0, 0])
        time.sleep(0.3)
        self.set_mode("GUIDED")

    def revoke(self, mode="LOITER"):
        """收回控制: 切离 GUIDED（LOITER 暂停 / LAND 降落 / RTL 返航）。

        ★ 地面站优先级最高: 切走模式 = 立即剥夺 ROS setpoint 有效性(docs/28 §5.3)。"""
        mode = mode.upper()
        if mode not in self.MODE_IDS:
            print(f"[!] revoke 模式 {mode} 无效, 可选: LOITER/LAND/RTL")
            return
        print(f"[gs] 收回控制 target={self.target}: setmode {mode}")
        self.set_mode(mode)

    def formation(self, shape, spacing=None, radius=None):
        """★ 编队切换 —— 【已停用】(2026-08-28 删除 sysid=99 通道后, 40010 无转发路径;
        队形由节点启动参数 --shape 决定)。"""
        print("[!] formation: 已停用 —— 40010 空中切换通道随 sysid=99 一并删除, "
              "队形由节点 --shape 启动参数决定")
        return None

    def route_check(self):
        """路由确认探针(docs/28 §6.1 步骤3): 发 40010(target=99) 看是否收到回执。

        ★ ArduPilot 无路由表查询命令(评审注记#4) —— 用探针间接验证:
        收到 COMMAND_ACK 且 srcSystem==99、command==40010、result==ACCEPTED
        = 飞控路由表已学 sysid=99 → serial1, target=99 消息被飞控转发到 ROS 节点。
        无回执 = 路由未注册/链路不通。"""
        print("[gs] 路由探针: COMMAND_LONG 40010 target=99 (等飞控转发回执)...")
        ack = self.command_long(MAV_CMD_FORMATION_SHAPE,
                                [0.0, 1.5, 5.0, 0, 0, 0, 0], target=99)
        if ack is None:
            print("[gs] ✗ 路由未通: 无回执。检查节点心跳注册(sysid=99)/链路")
        elif ack.get_srcSystem() != 99 or ack.command != MAV_CMD_FORMATION_SHAPE \
                or ack.result != mav.MAV_RESULT_ACCEPTED:
            print(f"[gs] ✗ 路由未生效: src={ack.get_srcSystem()} "
                  f"cmd={ack.command} result={ack.result} "
                  f"(期望 src=99 cmd=40010 result=0)")
        else:
            print(f"[gs] ✓ 路由生效: 节点(sysid=99)回执 ACCEPTED")
        return ack

    # ---------------- 接收侧: 遥测/状态/航线下载 ----------------
    def download_mission(self):
        """下载飞控航线: REQUEST_LIST(43) → COUNT(44) → REQUEST_INT(51) → ITEM_INT(73)。
        对应 C++ downloadMission 流程, 用 MISSION_REQUEST_INT(51)。"""
        self._send(self.mav.mission_request_list_encode(self.target, 1, 0))
        cnt = self._wait_type("MISSION_COUNT", timeout=3.0)
        if cnt is None:
            print("[gs] 下载: 无 MISSION_COUNT(飞控无航线或未响应)")
            return None
        n = cnt.count
        print(f"[gs] 下载: 飞控有 {n} 个航点")
        items = []
        for seq in range(n):
            self._send(self.mav.mission_request_int_encode(self.target, 1, seq, 0))
            item = self._wait_type("MISSION_ITEM_INT", timeout=3.0)
            if item is None:
                print(f"[gs] 下载: 等 seq={seq} 超时")
                break
            lat, lon = item.x / 1e7, item.y / 1e7
            dx = (lon - REF_LON) * M_PER_DEG_LAT * math.cos(math.radians(REF_LAT))
            dy = (lat - REF_LAT) * M_PER_DEG_LAT
            items.append([round(dx, 1), round(dy, 1)])
        self._send(self.mav.mission_ack_encode(self.target, 1,
                                               mav.MAV_MISSION_ACCEPTED, 0))
        print(f"[gs] 下载完成: {len(items)} 个 -> {items}")
        self.waypoints = items
        return items

    def monitor(self, seconds=15.0):
        """实时监听: 显示飞控心跳/模式/位置(HEARTBEAT/ATTITUDE/GPS)。
        对应 C++ 接收侧(HEARTBEAT case 0 + Telemetry)。"""
        print(f"[gs] 监听 {seconds:.0f}s (Ctrl-C 停止)...")
        hb_t, att_t = {}, {}
        t0 = time.time()
        while time.time() - t0 < seconds:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            msg = self.mav.parse_char(data)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT":
                hb_t[msg.get_srcSystem()] = (msg.type, msg.system_status,
                                             msg.base_mode, time.time())
            elif t == "ATTITUDE":
                att_t[msg.get_srcSystem()] = (msg.roll, msg.pitch, msg.yaw, time.time())
            elif t == "LOCAL_POSITION_NED":
                pass  # 位置也可用 LOCAL_POSITION_NED
            now = time.time()
            if now - t0 > 5.0 and int(now) % 2 == 0:
                self._print_monitor(hb_t, att_t)
        self._print_monitor(hb_t, att_t, final=True)

    def _print_monitor(self, hb_t, att_t, final=False):
        import os
        os.system("clear" if os.name == "posix" else "cls")
        now = time.time()
        print("===== 编队遥测 (地面站) =====")
        if not hb_t and not att_t:
            print("  未收到任何飞控数据(检查连接/端口)")
            return
        for sysid in sorted(set(list(hb_t) + list(att_t))):
            hb = hb_t.get(sysid)
            att = att_t.get(sysid)
            age_hb = f"{now - hb[3]:.0f}s" if hb else "—"
            mode = {0: "STAB", 1: "ACRO", 2: "ALT", 3: "AUTO", 4: "GUIDED",
                    5: "LOIT", 6: "RTL", 7: "CIRC", 9: "LAND"}.get(
                (hb[2] >> 0) & 0xFF, "?") if hb else "—"
            armed = "ARMED" if (hb[2] & 0x80) else "disarm" if hb else "—"
            att_s = (f"r={math.degrees(att[0]):6.1f}° p={math.degrees(att[1]):6.1f}° "
                     f"y={math.degrees(att[2]):7.1f}°") if att else "无姿态"
            print(f"  sysid={sysid}: mode={mode:6s} {armed:7s} 心跳age={age_hb} "
                  f"| {att_s}")
        print(f"  (刷新中, Ctrl-C 退出{' -- 监听结束' if final else ''})")

    def status(self):
        """查目标飞控状态: 请求 AUTOPILOT_VERSION + 显示连接。"""
        self.request_message(148)
        hb = self._wait_type("HEARTBEAT", timeout=2.0)
        if hb is None:
            print(f"[gs] sysid={self.target}: 无心跳(未连接或链路不通)")
        else:
            print(f"[gs] sysid={self.target}: 类型={hb.mavtype} "
                  f"status={hb.system_status} mode={hb.base_mode}")

    # ---------------- 交互 ----------------
    def repl(self):
        print("交互命令: auth | revoke [LOITER/LAND/RTL] | start | land | rtl | hold |"
              " resume | setmode <名> | upload <x> <y> ... | formation <队形> "
              "[--spacing X --radius X] | route-check |"
              " params --shape X | add <x> <y> | monitor [秒] | download | status | list | q")
        while True:
            try:
                line = input("gs> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            if line == "q":
                break
            try:
                self.dispatch(line)
            except Exception as e:
                print(f"[!] 指令错误: {e}")

    def dispatch(self, line):
        parts = line.split()
        cmd = parts[0]
        # ★ 规范#16: GUIDED 下地面站不发位置设定点(位置只归 ROS), 除非 --force
        if cmd in BLOCKED_POSITION_CMDS and not self.force:
            print(f"[!] {cmd}: GUIDED 下位置设定点被过滤(位置只归 ROS, 规范#16), "
                  f"确需调试请用 --force")
            return
        if cmd == "start":
            self.start()
        elif cmd == "auth":
            self.auth()
        elif cmd == "disarm":
            print(f"[gs] 加锁 target={self.target}: COMMAND_LONG 400 p1=0")
            self.command_long(mav.MAV_CMD_COMPONENT_ARM_DISARM, [0, 0, 0, 0, 0, 0, 0])
        elif cmd == "revoke":
            self.revoke(parts[1] if len(parts) > 1 else "LOITER")
        elif cmd == "land":
            self.land()
        elif cmd == "rtl":
            self.rtl()
        elif cmd == "hold":
            self.hold()
        elif cmd == "resume":
            self.resume()
        elif cmd == "setmode":
            self.set_mode(parts[1])
        elif cmd == "upload":
            # ★ 上传编队几何中心航线(特殊指令, target=99, docs/28 §6.3)
            vals = [float(v) for v in parts[1:]]
            if len(vals) == 0 or len(vals) % 2 != 0:
                print("[!] upload 用法: upload <x> <y> [<x> <y> ...] (公共系 ENU 米)")
                return
            wps = [[vals[k], vals[k + 1]] for k in range(0, len(vals), 2)]
            self.waypoints = wps
            self.upload_center(wps)
        elif cmd == "formation":
            g = lambda key: next((parts[i + 1] for i, p in enumerate(parts)
                                  if p == key), None)
            def f(key):
                v = g(key)
                return float(v) if v is not None else None
            if len(parts) < 2:
                print(f"[!] formation 用法: formation <队形> [--spacing X] [--radius X], "
                      f"可选: {sorted(SHAPE_IDS)}")
                return
            self.formation(parts[1], spacing=f("--spacing"), radius=f("--radius"))
        elif cmd in ("route-check", "route_check", "routecheck"):
            self.route_check()
        elif cmd == "goto":
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            self.set_position_target_local_ned(x, y, z)
        elif cmd == "add":
            vals = [float(v) for v in parts[1:]]
            if len(vals) == 0 or len(vals) % 2 != 0:
                print("[!] add 用法: add <x> <y> [<x> <y> ...]")
                return
            for k in range(0, len(vals), 2):
                self.waypoints.append([vals[k], vals[k + 1]])
            self.mission_send(self.waypoints)
        elif cmd == "takeoff-wp":
            # 方案B: 起飞点(NAV_TAKEOFF 22) + 任务航点(NAV_WAYPOINT 16)
            # 用法: takeoff-wp [--z 20] x y [x y ...]
            z = next((float(parts[i + 1]) for i, p in enumerate(parts)
                      if p == "--z"), 20.0)
            vals = [float(v) for v in parts if v not in ("--z",) and not v.startswith("--")]
            pts = [float(v) for v in parts if not v.startswith("--")]
            # 过滤掉 --z 及其值
            pts2, skip = [], False
            for v in parts[1:]:
                if v == "--z":
                    skip = True
                    continue
                if skip:
                    skip = False
                    continue
                pts2.append(float(v))
            if len(pts2) == 0 or len(pts2) % 2 != 0:
                print("[!] takeoff-wp 用法: takeoff-wp [--z 20] x y [x y ...]")
                return
            wps = [[pts2[k], pts2[k + 1]] for k in range(0, len(pts2), 2)]
            self.waypoints = wps
            self.mission_send_with_takeoff(wps, z)
            print(f"[gs] 方案B: 起飞点(22)@(0,0) + {len(wps)} 个航点(16)")
        elif cmd == "clear":
            self.waypoints = []
            self.mission_clear()
        elif cmd == "params":
            g = lambda key: next((parts[i + 1] for i, p in enumerate(parts)
                                  if p == key), None)
            def f(key):
                v = g(key)
                return float(v) if v is not None else None
            self.params(shape=g("--shape"), spacing=f("--spacing"),
                        radius=f("--radius"), yaw=f("--yaw"),
                        n=int(f("--n")) if g("--n") else None,
                        fly_z=f("--fly-z"), v_des=f("--v-des"),
                        drone_idx=int(f("--drone-idx")) if g("--drone-idx") else None)
        elif cmd == "monitor":
            secs = float(parts[1]) if len(parts) > 1 else 15.0
            self.monitor(secs)
        elif cmd == "download":
            self.download_mission()
        elif cmd == "status":
            self.status()
        elif cmd == "list":
            print(f"本地航点({len(self.waypoints)}): "
                  f"{[(round(w[0],1), round(w[1],1)) for w in self.waypoints]}")
        else:
            print(f"未知命令: {cmd}")


def main():
    ap = argparse.ArgumentParser(description="Python 地面站(复刻 C++ UAVMapSystem 消息格式)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--listen", type=int, default=14581)
    ap.add_argument("--sysid", type=int, default=255)
    ap.add_argument("--target", type=int, default=1)
    ap.add_argument("--all", type=int, default=0, metavar="N",
                    help="对 sysid=1..N 每架各执行一次命令")
    ap.add_argument("--force", action="store_true",
                    help="绕过位置设定点过滤(规范#16, 仅调试用)")
    ap.add_argument("command", nargs="*")
    args, _ = ap.parse_known_args()     # ★ 命令内选项(如 --shape)不被 argparse 拦截

    if args.all > 0:
        for tgt in range(1, args.all + 1):
            s = GroundStation(args.host, args.port, args.listen,
                              args.sysid, tgt, args.force)
            if args.command:
                print(f"── 发送到 sysid={tgt} ──")
                s.dispatch(" ".join(args.command))
                time.sleep(0.5)
        time.sleep(1.0)
        return

    s = GroundStation(args.host, args.port, args.listen, args.sysid,
                      args.target, args.force)
    if args.command:
        s.dispatch(" ".join(args.command))
        time.sleep(1.0)
    else:
        s.repl()


if __name__ == "__main__":
    main()
