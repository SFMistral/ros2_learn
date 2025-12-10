#!/usr/bin/env python3
"""
FollowPathService - 路径跟随服务节点

使用速度控制让乌龟平滑移动到目标点，而不是瞬移。
通过订阅乌龟位置、发布速度命令来实现。
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_interfaces.srv import FollowPath


class FollowPathService(Node):
    """
    路径跟随服务节点 - 使用速度控制平滑移动
    """

    def __init__(self):
        super().__init__('follow_path_service')

        # 使用 ReentrantCallbackGroup 允许回调并发执行
        self.callback_group = ReentrantCallbackGroup()

        # 当前乌龟位置
        self.current_pose = None
        self.pose_lock = threading.Lock()

        # 订阅乌龟位置
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )

        # 发布速度命令
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 创建服务
        self.srv = self.create_service(
            FollowPath,
            'follow_path',
            self.handle_follow_path,
            callback_group=self.callback_group
        )

        # 控制参数
        self.linear_speed = 1.5      # 线速度
        self.angular_speed = 3.0     # 角速度
        self.distance_tolerance = 0.1  # 到达目标的距离容差
        self.angle_tolerance = 0.05    # 角度容差

        self.get_logger().info('FollowPathService ready. Service name: /follow_path')

    def pose_callback(self, msg):
        """更新当前位置"""
        with self.pose_lock:
            self.current_pose = msg

    def get_current_pose(self):
        """线程安全地获取当前位置"""
        with self.pose_lock:
            return self.current_pose

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def move_to_goal(self, goal_x, goal_y, goal_theta):
        """
        移动到目标点
        返回 True 表示成功，False 表示失败
        """
        rate_hz = 20
        sleep_time = 1.0 / rate_hz
        timeout = 30.0  # 超时时间
        start_time = time.time()

        # 等待获取初始位置
        while self.get_current_pose() is None:
            if time.time() - start_time > 5.0:
                self.get_logger().error('Timeout waiting for pose')
                return False
            time.sleep(0.1)

        # 第一阶段：移动到目标位置
        while True:
            if time.time() - start_time > timeout:
                self.get_logger().error('Timeout moving to goal')
                return False

            pose = self.get_current_pose()
            if pose is None:
                time.sleep(sleep_time)
                continue

            # 计算到目标的距离和角度
            dx = goal_x - pose.x
            dy = goal_y - pose.y
            distance = math.sqrt(dx * dx + dy * dy)

            # 检查是否到达目标位置
            if distance < self.distance_tolerance:
                break

            # 计算目标方向
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - pose.theta)

            cmd = Twist()

            # 如果角度偏差大，先转向
            if abs(angle_diff) > 0.3:
                cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                cmd.linear.x = 0.0
            else:
                # 边走边调整方向
                cmd.linear.x = min(self.linear_speed, distance)
                cmd.angular.z = 2.0 * angle_diff

            self.vel_pub.publish(cmd)
            time.sleep(sleep_time)

        # 第二阶段：调整到目标朝向
        while True:
            if time.time() - start_time > timeout:
                self.get_logger().error('Timeout adjusting orientation')
                return False

            pose = self.get_current_pose()
            if pose is None:
                time.sleep(sleep_time)
                continue

            angle_diff = self.normalize_angle(goal_theta - pose.theta)

            if abs(angle_diff) < self.angle_tolerance:
                break

            cmd = Twist()
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            # 减慢接近目标时的速度
            if abs(angle_diff) < 0.5:
                cmd.angular.z *= 0.5
            self.vel_pub.publish(cmd)
            time.sleep(sleep_time)

        # 停止
        self.vel_pub.publish(Twist())
        return True

    def handle_follow_path(self, request, response):
        """处理路径跟随请求"""
        poses = request.poses

        if not poses:
            response.success = False
            response.message = 'No poses provided.'
            return response

        self.get_logger().info(f'Received path with {len(poses)} poses.')

        for i, pose in enumerate(poses):
            self.get_logger().info(
                f'Moving to #{i}: x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f}'
            )

            success = self.move_to_goal(pose.x, pose.y, pose.theta)

            if not success:
                response.success = False
                response.message = f'Failed at index {i}'
                return response

            self.get_logger().info(f'Reached point #{i}')

        response.success = True
        response.message = 'Path completed successfully.'
        self.get_logger().info('Path execution finished.')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FollowPathService()

    # 使用多线程执行器，允许服务回调和订阅回调并发执行
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
