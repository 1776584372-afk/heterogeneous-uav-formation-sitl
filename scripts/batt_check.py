#!/usr/bin/env python3
"""只读检查当前 SITL 电池状态与参数（连 MAVProxy 14550 命令入口，sysid 过滤）。"""
import sys, time
from pymavlink import mavutil

def main():
    tgt = sys.argv[1] if len(sys.argv) > 1 else "udpout:127.0.0.1:14550"
    sysid = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    print(f"[batt_check] connect {tgt}, filter sysid={sysid}")
    m = mavutil.mavlink_connection(tgt, source_system=255)
    # ★ 联调实测修复:
    #   ① 必须用 "udpout:" —— 本版 pymavlink 的 "udp:host:port" 会把本地端绑定到
    #      目标端口 14550(与 MAVProxy udpin 共绑冲突), 心跳被自己吃掉, 永不转发。
    #   ② MAVProxy 只向"发过心跳的客户端"转发遥测 —— 必须先心跳注册。
    for _ in range(3):
        m.mav.heartbeat_send(2, 8, 0, 0, 0)
        time.sleep(0.2)
    m.wait_heartbeat(timeout=5)
    print(f"[batt_check] heartbeat from sysid={m.target_system} compid={m.target_component}")

    # 请求参数
    for pname in ("SIM_BATT_DRAIN", "SIM_BATT_CAP_AH", "SIM_BATT_VOLTAGE", "BATT_CAPACITY", "BATT_LOW_VOLT", "BATT_CRT_VOLT", "BATT_FS_LOW_ACT"):
        m.mav.param_request_read_send(m.target_system, m.target_component, pname.encode(), -1)
    t0 = time.time()
    got = {}
    while time.time() - t0 < 4:
        msg = m.recv_match(type=["PARAM_VALUE", "SYS_STATUS", "BATTERY_STATUS"], blocking=True, timeout=0.5)
        if not msg:
            continue
        if msg.get_type() == "PARAM_VALUE":
            if msg.get_srcSystem() == sysid or msg.get_srcSystem() == 0:
                got[msg.param_id.strip("\x00")] = msg.param_value
        elif msg.get_type() == "SYS_STATUS" and msg.get_srcSystem() == sysid:
            print(f"[batt_check] SYS_STATUS sysid={msg.get_srcSystem()} battery_remaining={msg.battery_remaining}% "
                  f"battery_voltage={msg.voltage_battery/1000:.2f}V current={msg.current_battery/100 if msg.current_battery < 0 else msg.current_battery:.2f}A")
        elif msg.get_type() == "BATTERY_STATUS" and msg.get_srcSystem() == sysid:
            print(f"[batt_check] BATTERY_STATUS sysid={msg.get_srcSystem()} battery_remaining={msg.battery_remaining}% "
                  f"voltages={[v/1000 for v in msg.voltages[:4]]}")
    for k, v in sorted(got.items()):
        print(f"[batt_check] PARAM {k} = {v}")

if __name__ == "__main__":
    main()
