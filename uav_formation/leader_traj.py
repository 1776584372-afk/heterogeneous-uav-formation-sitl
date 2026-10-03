#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""leader_traj.py —— EGO leader 解析 ICC 参考轨迹(部署里程碑A, EGO-planner 接入前)

发布 /formation_center/position_cmd(quadrotor_msgs/PositionCommand, world 系 ENU,
原点=FC1 home): 一条 ICC 平滑转弯航线 —— 直线段 + 圆弧段 + 五次多项式过渡
(位置/速度/加速度 C2 连续)。朝向=解析切线, yaw_dot=解析角速度 ω=V·κ。
之后可被 EGO-planner(plan_manage) 替代。

★ 弃用旧"直线+直角"生成逻辑(平移直线 + 90° 直角瞬时转向): 改为瞬时曲率中心
  (ICC)同心圆弧算法, 中心转弯半径 Rcenter, 转弯角速度 ω=V/Rcenter, 解析速度
  V=ω·Rcenter 直接写入参考输入, 严禁离散点差分求速度。

用法:
  source /opt/ros/humble/setup.bash && source ~/ros2_mavros_ws/install/setup.bash
  source ~/bamboo_square_ws/install/setup.bash
  python3 uav_formation/leader_traj.py --route "5,0 35,0 45,30 25,50 5,30" \
      --turn-radius 15 --speed 1.5 --hz 10
"""
import argparse
import math
import rclpy

from rclpy.node import Node
from quadrotor_msgs.msg import PositionCommand

from uav_formation.icc_traj import IccRoute, parse_route


class LeaderTraj(Node):
    def __init__(self, route_pts, turn_radius, speed, hz, topic):
        super().__init__('leader_traj')
        self.pub = self.create_publisher(PositionCommand, topic, 10)
        self.route = IccRoute(route_pts, turn_radius, speed)
        self.s = 0.0
        self._hz = float(hz)
        self.timer = self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(
            'leader_traj: ICC 平滑转弯航线 %d 点, R=%.1fm, V=%.1fm/s, %dHz → %s'
            % (len(route_pts), turn_radius, speed, hz, topic))

    def tick(self):
        s = min(self.s, self.route.total_length())
        x, y, psi, kappa = self.route.eval_center(s)
        # 解析速度 = V·t(s); 解析角速度 ω = V·κ(s)(严禁差分)
        vx = self.route.v * math.cos(psi)
        vy = self.route.v * math.sin(psi)
        if s >= self.route.total_length() - 1e-6:
            vx = vy = 0.0
            kappa = 0.0
        m = PositionCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.position.x = float(x)
        m.position.y = float(y)
        m.position.z = 2.0
        m.velocity.x = float(vx)
        m.velocity.y = float(vy)
        m.velocity.z = 0.0
        m.yaw = float(psi)
        m.yaw_dot = float(self.route.v * kappa)
        m.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.pub.publish(m)
        self.s += self.route.v / self._hz  # 中心沿弧长以 V 推进


def main():
    ap = argparse.ArgumentParser(description='EGO leader 解析 ICC 参考轨迹(里程碑A)')
    ap.add_argument('--route', default='5,0 35,0 45,30 25,50 5,30',
                    help='航线 "x,y x,y ..."(空格分隔)')
    ap.add_argument('--turn-radius', type=float, default=15.0, help='中心转弯半径 m')
    ap.add_argument('--speed', type=float, default=1.5, help='巡航速度 m/s')
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--topic', default='/formation_center/position_cmd',
                    help='发布话题(默认 /formation_center/position_cmd)')
    args = ap.parse_args()
    pts = parse_route(args.route)
    if len(pts) < 2:
        raise SystemExit('航线至少 2 个点')
    rclpy.init()
    node = LeaderTraj(pts, args.turn_radius, args.speed, args.hz, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
