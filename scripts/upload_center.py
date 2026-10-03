#!/usr/bin/env python3
"""受控上传多弯道中心线到各飞控（经桥 udp:14550 → 各飞控 serial0）。

默认 5 点多弯道中心线（公共系 ENU 米, docs/29 §7.10 验证航线）:
    (5,0) → (35,0) → (45,30) → (25,50) → (5,30)

★ 首点 = 起始点（ArduPilot 会用 home 顶替任务第 0 槽, docs/29 §7.14.1）;
  节点拉取时已丢弃 home 项, 实际飞行 = 第 2 点起的真航点。

用法（需堆栈已启动、桥已监听 14550; 上传前建议先 mission_reset）:
  python3 scripts/upload_center.py                    # 默认 5 点, z=20m, 3 架
  python3 scripts/upload_center.py --z 50             # 改高度
  python3 scripts/upload_center.py --n 5              # 换机数
  python3 scripts/upload_center.py --route 5,0 35,0 45,30
"""
import argparse
import math
import socket
import sys
import time

from pymavlink.dialects.v20 import common as mav

REF = (30.893000, 120.087000)          # 公共系 ENU 基线(与节点 --ref-lat/lon 一致)
MPD = 111319.49
DEFAULT_ROUTE = [(5, 0), (35, 0), (45, 30), (25, 50), (5, 30)]


class Uploader:
    def __init__(self, host, port, n, z, route):
        self.dst = (host, port)
        self.n = n
        self.z = z
        self.route = route
        self.m = mav.MAVLink(None)
        self.m.srcSystem = 255          # 地面站身份
        self.m.srcComponent = 1
        self.pm = mav.MAVLink(None)     # 解析用独立对象(共享 parse 缓冲会污染)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.3)

    def _send(self, msg):
        self.sock.sendto(msg.pack(self.m), self.dst)

    def hb(self):
        """抢占桥的回执通道(其它发送者可能劫持 gcs_addr, 实测坑)。"""
        for _ in range(3):
            self._send(self.m.heartbeat_encode(2, 8, 0, 0, 0))
            time.sleep(0.2)

    def _recv(self):
        try:
            data, _ = self.sock.recvfrom(65535)
        except socket.timeout:
            return None
        try:
            return self.pm.parse_char(data)
        except Exception:
            return None

    def upload_fc(self, tgt):
        """给飞控 tgt 上传整条航线: COUNT → 逐点(等 REQUEST_INT) → ACK(type=0)。"""
        self._send(self.m.mission_count_encode(tgt, 1, len(self.route), 0))
        seq = 0
        ack = None
        t0 = time.time()
        while time.time() - t0 < 20 and ack is None:
            msg = self._recv()
            if msg is None:
                continue
            t = msg.get_type()
            src = msg.get_srcSystem()
            if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST") \
                    and src == tgt and getattr(msg, "seq", -1) == seq \
                    and seq < len(self.route):
                x, y = self.route[seq]
                lon = REF[1] + x / (MPD * math.cos(math.radians(REF[0])))
                lat = REF[0] + y / MPD
                self._send(self.m.mission_item_int_encode(
                    tgt, 1, seq, mav.MAV_FRAME_GLOBAL_RELATIVE_ALT, 16,
                    0, 1, 0, 0, 0, 0, int(lat * 1e7), int(lon * 1e7),
                    self.z, 0))
                seq += 1
            elif t == "MISSION_ACK" and src == tgt:
                ack = msg
        ok = ack is not None and ack.type == mav.MAV_MISSION_ACCEPTED
        print(f"  飞控{tgt}: 上传 {len(self.route)} 点(z={self.z}m), 收到 {seq} 点请求 -> "
              f"{'ACCEPTED ✓' if ok else '✗ ' + (str(ack.type) if ack else '无回执(超时)')}")
        return ok

    def run(self):
        self.hb()
        all_ok = True
        for tgt in range(1, self.n + 1):
            all_ok = self.upload_fc(tgt) and all_ok
            time.sleep(0.3)             # 逐架顺序上传(与真实 GCS 行为一致)
        print("== " + ("全部飞控上传成功 ✓" if all_ok else "有飞控失败 ✗ (可重跑本脚本或先 mission_reset)") + " ==")
        return 0 if all_ok else 1


def main():
    ap = argparse.ArgumentParser(description="受控上传多弯道中心线到各飞控")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--n", type=int, default=3, help="机数(默认 3)")
    ap.add_argument("--z", type=float, default=20.0, help="航线高度 m(默认 20)")
    ap.add_argument("--route", nargs="*", default=None,
                    help="航点 x,y(公共系 ENU 米, 默认 5 点多弯道)")
    args = ap.parse_args()
    route = DEFAULT_ROUTE
    if args.route:
        route = [tuple(float(v) for v in p.split(",")) for p in args.route]
    return Uploader(args.host, args.port, args.n, args.z, route).run()


if __name__ == "__main__":
    sys.exit(main())
