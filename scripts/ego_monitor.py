#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ego_monitor.py —— EGO 链路实时可视化监控(matplotlib)

订阅 ROS 话题, 在 DISPLAY=:0 弹实时窗口并定时保存 PNG 到工作区:
  * /uav{i}/mavros/local_position/pose   3 机位置(ENU, 加 home 偏移转世界系)
  * /formation_center/odom               均值中心
  * /goal                                当前目标
  * /formation_center/position_cmd       中心轨迹流(轨迹执行中标记)
  * /uav0/mavros/state                   armed/mode 状态

用法(需 source ROS 环境):
  MPLCONFIGDIR=<可写目录> python3 scripts/ego_monitor.py [--out gui/ego_monitor.png]

窗口: 左 2D 平面(队形+中心+航线+目标), 右 高度曲线 + 状态文本。
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

Q_BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE,
                  history=HistoryPolicy.KEEP_LAST, depth=5)


class Monitor(Node):
    def __init__(self, n, out):
        super().__init__('ego_monitor')
        self.n = n
        self.out = out
        self.poses = {}        # {i: (x, y, z)} world(加 home 偏移)
        self.alt = {}          # {i: z}
        self.center = None
        self.goal = None
        self.cmd_t = 0.0
        self.state = (False, "?")
        self.t0 = None
        self.alt_history = {i: [] for i in range(n)}
        self.t_history = []
        self.center_traj = []  # [(x, y)]
        for i in range(n):
            self.create_subscription(
                PoseStamped, '/uav%d/mavros/local_position/pose' % i,
                self._mk_pose(i), Q_BE)
        self.create_subscription(Odometry, '/formation_center/odom',
                                 self._on_odom, Q_BE)
        self.create_subscription(PoseStamped, '/goal', self._on_goal, Q_BE)
        self.create_subscription(PoseStamped, '/formation_center/position_cmd',
                                 self._on_cmd, Q_BE)
        self.create_subscription(State, '/uav0/mavros/state',
                                 self._on_state, Q_BE)

    def _mk_pose(self, i):
        off = 10.0 * i
        def cb(m):
            p = m.pose.position
            self.poses[i] = (float(p.x) + off, float(p.y), float(p.z))
            now = self.get_clock().now().nanoseconds / 1e9
            if self.t0 is None:
                self.t0 = now
            t = now - self.t0
            self.t_history.append(t)
            for j in range(self.n):
                if j == i:
                    self.alt_history[j].append(float(p.z))
                elif len(self.alt_history[j]) < len(self.t_history):
                    self.alt_history[j].append(self.alt_history[j][-1] if self.alt_history[j] else 0.0)
        return cb

    def _on_odom(self, m):
        p = m.pose.pose.position
        self.center = (float(p.x), float(p.y))
        if self.center_traj and math.hypot(self.center_traj[-1][0] - p.x,
                                           self.center_traj[-1][1] - p.y) > 0.2:
            self.center_traj.append(self.center)
        elif not self.center_traj:
            self.center_traj.append(self.center)

    def _on_goal(self, m):
        self.goal = (float(m.pose.position.x), float(m.pose.position.y),
                     float(m.pose.position.z))

    def _on_cmd(self, m):
        self.cmd_t = self.get_clock().now().nanoseconds / 1e9

    def _on_state(self, m):
        self.state = (bool(m.armed), str(m.mode))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--out', default='gui/ego_monitor.png')
    ap.add_argument('--save-every', type=float, default=2.0)
    args = ap.parse_args()

    rclpy.init()
    mon = Monitor(args.n, args.out)
    os.makedirs(os.path.dirname(args.out), exist_ok=True) if os.path.dirname(args.out) else None

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))
    fig.canvas.manager.set_window_title('EGO Formation Monitor')

    def update(_):
        rclpy.spin_once(mon, timeout_sec=0.01)   # 驱动订阅回调(主线程动画)
        ax1.clear(); ax2.clear()
        # 左: 平面
        xs, ys, zs = [], [], []
        for i in range(mon.n):
            if i in mon.poses:
                x, y, z = mon.poses[i]
                xs.append(x); ys.append(y); zs.append(z)
                ax1.plot(x, y, 'o', ms=10, label='uav%d (z=%.1f)' % (i, z))
        if mon.center:
            ax1.plot(mon.center[0], mon.center[1], 'p', color='black', ms=14,
                     label='center')
        if mon.center_traj:
            tx = [p[0] for p in mon.center_traj]
            ty = [p[1] for p in mon.center_traj]
            ax1.plot(tx, ty, '-', color='gray', alpha=0.6, lw=1)
        if mon.goal:
            ax1.plot(mon.goal[0], mon.goal[1], 'x', color='red', ms=14, mew=3,
                     label='goal (%.1f,%.1f)' % (mon.goal[0], mon.goal[1]))
        # 期望槽位(line): 中心 + (0,-10)/(0,0)/(0,10)
        if mon.center:
            for sx, sy in [(0, -10), (0, 0), (0, 10)]:
                ax1.plot(mon.center[0] + sx, mon.center[1] + sy, 's', mfc='none',
                         mec='blue', ms=9)
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_title('Formation XY (world ENU)  %s' %
                      ('[traj active]' if mon.get_clock().now().nanoseconds / 1e9 - mon.cmd_t < 2.0 else ''))
        ax1.set_xlabel('X east (m)'); ax1.set_ylabel('Y north (m)')
        ax1.legend(loc='upper left', fontsize=8)
        # 右: 高度 + 状态
        if mon.t_history:
            for i in range(mon.n):
                ax2.plot(mon.t_history, mon.alt_history[i], label='uav%d' % i)
        ax2.set_title('Altitude  armed=%s mode=%s' % mon.state)
        ax2.set_xlabel('t (s)'); ax2.set_ylabel('z (m)')
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper left', fontsize=8)
        fig.tight_layout()
        try:
            fig.savefig(args.out, dpi=100)
        except OSError as e:
            print('[monitor] save fail:', e)

    ani = animation.FuncAnimation(fig, update, interval=500, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
