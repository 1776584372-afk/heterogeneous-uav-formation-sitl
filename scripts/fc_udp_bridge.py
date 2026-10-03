#!/usr/bin/env python3
"""fc_udp_bridge.py —— 地面站 UDP ⇄ 各飞控 serial0 TCP 路由桥（对齐官方 sim_vehicle.py 直连架构）

背景（2026-08-27 实机联调）:
  官方 sim_vehicle.py 多机下"航线上传没问题"，是因为地面站**直连飞控**（TCP serial0），
  mission 流量不经 MAVProxy；而我们让 mission 挤过单 MAVProxy 多 master → 并发握手竞态
  （speedup=10 把飞控 mission 超时压到 0.8 真实秒, 慢的那架被取消）。
  本桥把地面站的 UDP 消息按 target_system 严格路由到对应飞控的 serial0 TCP,
  响应原路返回 —— 等价于官方直连, 绕开 MAVProxy 的 mission 干扰。

拓扑:
  sys2 地面站 ──UDP──► 本桥 ──TCP(5760/5770/5780)──► 飞控1/2/3
                        ▲ 响应(REQ/ACK/心跳/遥测) 原路返回 ▼

用法:
  # 监听 UDP 14550(地面站原有命令入口, GCS 零修改)+14600(新口), 按 sysid 路由到 3 架飞控 serial0
  # --push: 遥测除"回执原路返回"外, 额外推送到固定地面站(如 10.200.7.103:14550)
  python3 scripts/fc_udp_bridge.py --udp-ports 14550,14600 --push 10.200.7.103:14550 \
      --fcs 1:127.0.0.1:5760,2:127.0.0.1:5770,3:127.0.0.1:5780

  # 地面站不用改任何代码/配置 —— 它本来就发往 10.200.7.122:14550 (原 MAVProxy 命令入口)
"""
import argparse
import socket
import struct
import sys
import threading
import time

from pymavlink.dialects.v20 import common as mav

# ★ 本机地址集合: 判断"自回声"必须【源IP + 源端口】双条件。
#   2026-08-28 实机发现: 真实地面站(MAVSDK)本地绑定 14550, 发包源端口就是 14550;
#   旧版只查端口 → 把地面站的命令/遥测当"自回声"全丢弃 → 地面站永远连不上。
_LOCAL_IPS = {"127.0.0.1", "::1", "127.0.1.1"}
try:
    import subprocess
    _out = subprocess.run(["ip", "-4", "-o", "addr", "show"],
                          capture_output=True, text=True, timeout=3).stdout
    for _line in _out.splitlines():
        for _tok in _line.split():
            if "/" in _tok and ":" not in _tok:
                _LOCAL_IPS.add(_tok.split("/")[0])
except Exception:
    pass
try:
    _tmp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        _tmp.connect(("8.8.8.8", 80))          # 兜底: 拿本机默认出口 IP(不实际发包)
        _LOCAL_IPS.add(_tmp.getsockname()[0])
    finally:
        _tmp.close()
except Exception:
    pass
try:
    for _info in socket.getaddrinfo(socket.gethostname(), None):
        _LOCAL_IPS.add(_info[4][0])
except Exception:
    pass


def split_mav_frames(buf):
    """把 TCP 字节流按 MAVLink 帧边界切成完整帧。

    TCP recv 的字节块边界与 MAVLink 帧边界不对齐, 若把每个 recv 块直接
    作为一个 UDP 数据报转发, 地面站会收到被拆散的半帧(CRC 报错)。
    这里按帧头/长度解析, 保证"一条 UDP 数据报 == 一条完整 MAVLink 帧"。
    返回 (frames, 剩余未满一帧的字节)。
    """
    frames = []
    while True:
        if len(buf) < 3:
            break
        stx = buf[0]
        if stx == 0xFD:  # MAVLink2
            plen = buf[1]
            total = 10 + plen + 2 + (13 if (buf[2] & 0x01) else 0)
        elif stx == 0xFE:  # MAVLink1
            plen = buf[1]
            total = 6 + plen + 2
        else:
            buf = buf[1:]  # 失去同步, 前移 1 字节重找帧头
            continue
        if len(buf) < total:
            break
        frames.append(buf[:total])
        buf = buf[total:]
    return frames, buf


class FcUdpBridge:
    """UDP 收地面站 MAVLink → 按 target_system 路由到对应飞控 TCP; 响应回传。"""

    def __init__(self, udp_ports, fcs_map, push, verbose=False):
        self.udp_ports = [int(p) for p in udp_ports.split(",")]
        # fcs_map: {sysid: "host:port"}
        self.fcs = {int(s): (h, int(p)) for s, h, p in
                    (tuple(x.split(":")) for x in fcs_map.split(","))}
        # 固定推流目标(地面站): 遥测除回执原路返回外, 额外推送
        self.push = []
        for x in (push or "").split(","):
            if x.strip():
                h, p = x.strip().split(":")
                self.push.append((h, int(p)))
        self.verbose = verbose
        self.mav = mav.MAVLink(None)
        # 主套接字: 绑第一个端口, 兼作回包发送口(地面站 UDP 不校验源端口)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp.bind(("0.0.0.0", self.udp_ports[0]))
        self.udp.settimeout(0.3)
        # 其余监听口(14550 是原 MAVProxy 命令入口, 保留它 = 地面站零修改)
        self.listeners = []
        for p in self.udp_ports[1:]:
            s2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s2.bind(("0.0.0.0", p))
            s2.settimeout(0.3)
            self.listeners.append((p, s2))
        self.gcs_addr = None            # 最近发过包的地面站地址(响应回给)
        self.lock = threading.Lock()
        self.tcp = {}                   # sysid -> socket
        self.buf = {}                   # sysid -> TCP 字节流残帧缓冲(帧对齐转发用)
        self.stop = False

    # ---------- TCP 链路管理 ----------
    def _tcp(self, sysid):
        """取(必要时建立)该飞控的 TCP 连接。"""
        with self.lock:
            sock = self.tcp.get(sysid)
            if sock is not None:
                return sock
        host, port = self.fcs.get(sysid)
        if host is None:
            return None
        try:
            sock = socket.create_connection((host, port), timeout=5.0)
            sock.settimeout(0.5)
            with self.lock:
                self.tcp[sysid] = sock
            print(f"[bridge] 已连 飞控{sysid} {host}:{port}")
            return sock
        except OSError as e:
            print(f"[bridge] 连 飞控{sysid} 失败: {e}")
            return None

    def _relay_to_fc(self, data, sysid):
        """把地面站的数据帧转发给对应飞控(字节级透明)。"""
        sock = self._tcp(sysid)
        if sock is None:
            return
        try:
            sock.sendall(data)
        except OSError:
            with self.lock:
                self.tcp.pop(sysid, None)
            try:
                sock.close()
            except OSError:
                pass

    # ---------- 地面站 UDP 收包 ----------
    def _udp_loop(self, port, sock):
        print(f"[bridge] UDP 监听 :{port} (地面站命令入口), "
              f"路由表: {self.fcs}")
        while not self.stop:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            # ★ 自回声防护(2026-08-28 修正): 本桥自己转发的包 = 【源IP是本机 + 源端口是本桥端口】。
            #   旧版只查端口, 会把"本地绑定 14550 的地面站"(MAVSDK 常见, 源端口=14550)
            #   的包误当自回声丢弃 → 地面站永远连不上。现在远程地面站(源IP≠本机)永不误杀。
            if addr[0] in _LOCAL_IPS and addr[1] in self.udp_ports:
                if self.verbose:
                    print(f"[bridge] 丢弃自回声({addr[0]}:{addr[1]}), 环路已掐断")
                continue
            with self.lock:
                self.gcs_addr = addr
            msg = None
            try:
                msg = self.mav.parse_char(data)
            except Exception:
                pass
            if msg is None:
                continue
            sysid = getattr(msg, "target_system", 0)
            if self.verbose:
                print(f"[bridge] GCS→ {msg.get_type()} target={sysid}")
            # ★ 目标=0 广播 → 全部飞控; 未知目标 → 回退 FC1
            #   (对齐原 MAVProxy master() 行为: 未知 sysid 回退第一路 master),
            #   由 FC1 的 MAVLink_routing 继续按路由表转发。
            #   (2026-08-28 已删除 sysid=99 编队指令通道, 未知目标仅剩容错兜底)
            if sysid == 0:
                targets = list(self.fcs)
            elif sysid in self.fcs:
                targets = [sysid]
            else:
                print(f"[bridge] 未知目标 sysid={sysid} → 回退 FC1")
                targets = [1]
            for t in targets:
                if t in self.fcs:
                    self._relay_to_fc(data, t)

    # ---------- 飞控 TCP 收包 → 回地面站 ----------
    def _fc_loop(self, sysid):
        host, port = self.fcs[sysid]
        while not self.stop:
            sock = self._tcp(sysid)
            if sock is None:
                time.sleep(1.0)
                continue
            try:
                data = sock.recv(65535)
            except socket.timeout:
                continue
            except OSError:
                with self.lock:
                    self.tcp.pop(sysid, None)
                try:
                    sock.close()
                except OSError:
                    pass
                time.sleep(1.0)
                continue
            if not data:
                time.sleep(0.1)
                continue
            self.buf[sysid] = self.buf.get(sysid, b"") + data
            frames, self.buf[sysid] = split_mav_frames(self.buf[sysid])
            with self.lock:
                gcs = self.gcs_addr
            # 接收方 = 最近发包地面站(回执原路返回) + 配置的固定推流目标(--push)
            # 自回声防护(2026-08-28 修正): 只屏蔽【本机IP+本桥端口】的自身回环, 不再按端口一刀切
            #   —— 地面站 10.200.7.103:14550 源端口=14550 必须收(旧版把它当自回声排除了)。
            targets = []
            if gcs is not None and not (gcs[0] in _LOCAL_IPS and gcs[1] in self.udp_ports):
                targets.append(gcs)
            for p in self.push:
                if not (p[0] in _LOCAL_IPS and p[1] in self.udp_ports) and p not in targets:
                    targets.append(p)
            if not targets:
                continue
            for fr in frames:
                for tgt in targets:
                    try:
                        self.udp.sendto(fr, tgt)
                    except OSError:
                        pass
                if self.verbose:
                    try:
                        m = self.mav.parse_char(fr)
                        if m is not None:
                            print(f"[bridge] 飞控{sysid}→ {m.get_type()}")
                    except Exception:
                        pass

    # ---------- 启动 ----------
    def run(self):
        threading.Thread(target=self._udp_loop,
                         args=(self.udp_ports[0], self.udp), daemon=True).start()
        for p, s2 in self.listeners:
            threading.Thread(target=self._udp_loop, args=(p, s2),
                             daemon=True).start()
        for sysid in self.fcs:
            threading.Thread(target=self._fc_loop, args=(sysid,), daemon=True).start()
        if self.push:
            print(f"[bridge] 固定推流目标: {self.push} (遥测额外推送)")
        print("[bridge] 运行中 (Ctrl-C 停止)")
        try:
            while not self.stop:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        self.stop = True
        print("[bridge] 已停止")


def main():
    ap = argparse.ArgumentParser(description="地面站 UDP ⇄ 飞控 serial0 TCP 路由桥")
    ap.add_argument("--udp-ports", default="14550,14600",
                    help="地面站 UDP 命令入口端口(逗号分隔; 14550=原 MAVProxy 入口, 地面站零修改)")
    ap.add_argument("--push", default="",
                    help="固定推流目标 host:port(逗号分隔, 如 10.200.7.103:14550); 遥测除回执原路返回外额外推送")
    ap.add_argument("--fcs", default="1:127.0.0.1:5760,2:127.0.0.1:5770,3:127.0.0.1:5780",
                    help="sysid:host:port 逗号分隔(各飞控 serial0)")
    ap.add_argument("--verbose", action="store_true", help="逐帧打印")
    args = ap.parse_args()
    FcUdpBridge(args.udp_ports, args.fcs, args.push, args.verbose).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
