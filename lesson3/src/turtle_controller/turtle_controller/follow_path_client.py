#!/usr/bin/env python3
"""
ROS2 路径跟随客户端

支持通过命令行参数传入任意数量的路径点，自动计算每个点的朝向角度。

用法:
  ros2 run turtle_controller follow_path_client --ros-args -p points:="2,2;8,2;8,8;2,8"
  
格式: "x1,y1;x2,y2;x3,y3;..."  用分号分隔每个点，逗号分隔x和y
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from turtle_interfaces.srv import FollowPath


class FollowPathClient(Node):
    """路径跟随客户端"""

    def __init__(self):
        super().__init__('follow_path_client')

        # 声明参数，使用分号分隔点，逗号分隔坐标
        self.declare_parameter('points', '')

        # 获取参数
        points_str = self.get_parameter('points').get_parameter_value().string_value

        if not points_str:
            self.get_logger().error('Please provide points parameter')
            self.get_logger().error('Usage: --ros-args -p points:="2,2;8,2;8,8;2,8"')
            self._valid = False
            return

        # 解析路径点
        try:
            points = self.parse_points(points_str)
            if len(points) < 1:
                self.get_logger().error('At least 1 point required')
                self._valid = False
                return
        except ValueError as e:
            self.get_logger().error(f'Failed to parse points: {e}')
            self.get_logger().error('Format: "x1,y1;x2,y2;..." e.g. "2,2;8,2;8,8"')
            self._valid = False
            return

        self._valid = True
        self.get_logger().info(f'Parsed {len(points)} points: {points}')

        # 创建服务客户端
        self.cli = self.create_client(FollowPath, 'follow_path')

        self.get_logger().info('Waiting for /follow_path service...')
        self.cli.wait_for_service()
        self.get_logger().info('/follow_path available. Sending request...')

        # 发送请求
        self.send_request(points)

    def parse_points(self, points_str):
        """
        解析点字符串
        格式: "x1,y1;x2,y2;x3,y3"
        """
        points = []
        for point_str in points_str.split(';'):
            point_str = point_str.strip()
            if not point_str:
                continue
            parts = point_str.split(',')
            if len(parts) != 2:
                raise ValueError(f'Invalid point format: {point_str}')
            x = float(parts[0].strip())
            y = float(parts[1].strip())
            points.append((x, y))
        return points

    def calculate_angles(self, points):
        """
        计算每个点的朝向角度
        每个点的角度 = 指向下一个点的方向
        最后一个点保持与前一个点相同的角度
        """
        poses = []
        n = len(points)

        for i in range(n):
            x, y = points[i]

            if i < n - 1:
                # 计算指向下一个点的角度
                next_x, next_y = points[i + 1]
                theta = math.atan2(next_y - y, next_x - x)
            else:
                # 最后一个点：保持前一个点的角度
                if n > 1:
                    prev_x, prev_y = points[i - 1]
                    theta = math.atan2(y - prev_y, x - prev_x)
                else:
                    theta = 0.0

            poses.append(Pose2D(x=float(x), y=float(y), theta=theta))
            self.get_logger().info(
                f'Point {i}: ({x}, {y}) -> theta={math.degrees(theta):.1f}deg'
            )

        return poses

    def send_request(self, points):
        """构造并发送路径跟随请求"""
        req = FollowPath.Request()
        req.poses = self.calculate_angles(points)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            resp = future.result()
            self.get_logger().info(
                f'Service result: success={resp.success}, message="{resp.message}"'
            )
        else:
            self.get_logger().error('Service call failed.')


def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClient()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
