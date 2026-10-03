#!/usr/bin/env python3
"""飞控 mission 状态清理工具 —— 地面站上传中心线/航点前跑一次。

背景（2026-08-27 实机联调）:
  地面站(sys2 GCS)上传中心线给飞控时, 若飞控的 mission 协议状态机残留
  (早前中断/超时的上传未清), 会回 MISSION_ACK type=13(INVALID_SEQUENCE) 拒绝。
  实测: MISSION_CLEAR_ALL + 等协议超时(默认 8s) 后, 3 架飞控全部恢复 ACCEPTED。

用法:
  python3 scripts/mission_reset.py                # 清空 sysid=1..3 并等待
  python3 scripts/mission_reset.py --n 5          # 机数 5
  python3 scripts/mission_reset.py --host 10.200.7.122 --port 14550
  python3 scripts/mission_reset.py --test         # 清理后做一次 2 航点上传承接验证
"""
import argparse
import math
import socket
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0] if "/" in __file__ else ".")

from pymavlink.dialects.v20 import common as mav

REF = (30.893000, 120.087000)
M_PER_DEG_LAT = 111319.49


class MissionReset:
    def __init__(self, host, port, n, wait, test):
        self.dst = (host, int(port))
        self.n = int(n)
        self.wait = float(wait)
        self.test = bool(test)
        self.mav = mav.MAVLink(None)
        self.mav.srcSystem = 255
        self.mav.srcComponent = 1
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.3)

    def _send(self, msg):
        self.sock.sendto(msg.pack(self.mav), self.dst)

    def _hb(self):
        # ★ MAVProxy 只向"发过心跳的客户端"转发遥测(联调实测坑)
        for _ in range(3):
            self._send(self.mav.heartbeat_encode(2, 8, 0, 0, 0))
            time.sleep(0.2)

    def clear_all(self):
        print(f"[reset] 清空 sysid=1..{self.n} 飞控航点(MISSION_CLEAR_ALL)...")
        for tgt in range(1, self.n + 1):
            self._send(self.mav.mission_clear_all_encode(tgt, 1))
        print(f"[reset] 已发送, 等待 {self.wait:.0f}s 让协议状态机完全清除(超时重置)...")
        time.sleep(self.wait)
        # ★ 2026-08-28 修复: 清空后【排空 socket 里的残留 CLEAR ACK】—— 桥会把各飞控的
        #   MISSION_ACK/遥测推回本 socket, 若不排空, 后续上传会把"残留 CLEAR ACK"误当
        #   "上传 ACK"而假阳性 ACCEPTED(实测: seq=0 就 ACK=✓, 实际一个 ITEM 都没传)。
        self.drain_acks(quiet=0.8)

    def drain_acks(self, quiet=0.8, timeout=6.0):
        """丢弃 socket 中残留的 MISSION_ACK(含遥测), 直到 quiet 秒内无新 ACK 或超时。"""
        t_end = time.time() + timeout
        last_ack = time.time()
        pmav = mav.MAVLink(None)          # ★ 解析用独立对象(共享 self.mav 的 parse 缓冲会污染)
        while time.time() < t_end:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                if time.time() - last_ack > quiet:
                    break
                continue
            try:
                msg = pmav.parse_char(data)
            except Exception:
                continue
            if msg is not None and msg.get_type() == "MISSION_ACK":
                last_ack = time.time()
        print("[reset] 已排空残留 ACK")

    def heartbeat_check(self, timeout=6.0):
        """确认飞控在线(compid=1 真飞控心跳)。"""
        seen = set()
        t0 = time.time()
        pmav = mav.MAVLink(None)          # ★ 解析用独立对象(共享 self.mav 的 parse 缓冲会污染)
        while time.time() - t0 < timeout and len(seen) < self.n:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            msg = pmav.parse_char(data)
            if msg is None:
                continue
            if msg.get_type() == "HEARTBEAT" and msg.get_srcComponent() == 1 \
                    and 1 <= msg.get_srcSystem() <= self.n:
                seen.add(msg.get_srcSystem())
        ok = len(seen) == self.n
        print(f"[reset] 飞控心跳: {sorted(seen) if seen else '✗ 无'} "
              f"({'全部在线' if ok else '不完整!'})")
        return ok

    def test_upload(self):
        """清理后做一次 2 航点上传承接, 确认 ACCEPTED(验证修复)。"""
        wps = [(5.0, 0.0), (35.0, 0.0)]
        all_ok = True
        for tgt in range(1, self.n + 1):
            self._send(self.mav.mission_count_encode(tgt, 1, len(wps), 0))
            t0 = time.time()
            seq = 0
            ack = None
            pmav = mav.MAVLink(None)      # ★ 解析用独立对象(共享 self.mav 的 parse 缓冲会污染)
            while time.time() - t0 < 15 and ack is None:
                try:
                    data, _ = self.sock.recvfrom(65535)
                except socket.timeout:
                    continue
                msg = pmav.parse_char(data)
                if msg is None:
                    continue
                t = msg.get_type()
                src = msg.get_srcSystem()
                if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST") \
                        and src == tgt \
                        and getattr(msg, "seq", -1) == seq and seq < len(wps):
                    x, y = wps[seq]
                    lon = REF[1] + x / (M_PER_DEG_LAT * math.cos(math.radians(REF[0])))
                    lat = REF[0] + y / M_PER_DEG_LAT
                    self._send(self.mav.mission_item_int_encode(
                        tgt, 1, seq, 3, 16, 0, 1, 0, 0, 0, 0,
                        int(lat * 1e7), int(lon * 1e7), 2.0, 0))
                    seq += 1
                elif t == "MISSION_ACK" and src == tgt:
                    ack = msg
            ok = ack is not None and ack.type == 0
            all_ok = all_ok and ok
            print(f"[reset] sysid={tgt} 上传: "
                  f"{'ACCEPTED(type=0) ✓' if ok else ('拒绝 type=' + str(ack.type) if ack else '✗ 无')}")
        return all_ok

    def run(self):
        self._hb()
        ok_hb = self.heartbeat_check()
        self.clear_all()
        # ★ 2026-08-28: 清空(8s+排空)期间, 其它发送者(如远端 GCS 收推流后回包)可能
        #   劫持桥的回执通道(gcs_addr) → 重新发心跳抢占, 否则第二次心跳检查收不到。
        self._hb()
        # 再收一次心跳确认清理后飞控仍在线
        ok_hb2 = self.heartbeat_check()
        if not (ok_hb and ok_hb2):
            print("[reset] ✗ 飞控不在线, 检查堆栈/链路")
            return 1
        if self.test:
            self._hb()                     # 上传前同样抢占回执通道
            if not self.test_upload():
                print("[reset] ✗ 上传验证失败")
                return 1
        print("[reset] ✓ 完成: 飞控 mission 状态已清理, 可重新上传中心线")
        return 0


def main():
    ap = argparse.ArgumentParser(description="飞控 mission 状态清理(地面站上传前跑)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--n", type=int, default=3, help="机数(默认 3)")
    ap.add_argument("--wait", type=float, default=8.0,
                    help="清理后等待秒数(默认 8s, 让协议超时完全清除)")
    ap.add_argument("--test", action="store_true",
                    help="清理后做一次 2 航点上传承接验证(ACCEPTED)")
    args = ap.parse_args()
    return MissionReset(args.host, args.port, args.n, args.wait, args.test).run()


if __name__ == "__main__":
    sys.exit(main())
