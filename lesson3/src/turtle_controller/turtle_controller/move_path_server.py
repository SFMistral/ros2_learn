#!/usr/bin/env python3
"""
ROS2 Action Server 示例：实现海龟路径移动服务

这个文件演示了如何创建一个 Action Server 来处理路径移动请求。
与简单的服务不同，Action Server 适合长时间运行的任务，提供：
- 目标验证和接受/拒绝机制
- 实时反馈（当前进度）
- 任务取消支持
- 最终结果返回

这是一个完整的路径规划和执行系统的服务端实现。
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# 导入消息类型
from geometry_msgs.msg import Twist, Pose2D  # 速度控制和2D位姿
from turtlesim.msg import Pose               # turtlesim的位姿消息

# 导入我们自定义的 MovePath 动作接口
from turtle_interfaces.action import MovePath


class MovePathActionServer(Node):
    """
    路径移动动作服务器类
    
    继承自 rclpy.node.Node，实现一个 Action Server 来处理路径移动请求。
    这个类展示了完整的 Action Server 实现，包括：
    - 目标验证逻辑
    - 路径执行算法
    - 实时反馈机制
    - 取消处理
    """
    def __init__(self):
        """
        初始化路径移动动作服务器
        
        设置发布者、订阅者和 Action Server
        """
        # 调用父类构造函数，设置节点名称
        super().__init__('move_path_action_server')

        # 日志节流：记录上次日志输出时间
        self._last_log_time = 0.0
        self._log_interval = 0.5  # 日志输出间隔（秒）

        # 创建可重入回调组，允许回调并行执行
        self._callback_group = ReentrantCallbackGroup()

        # 创建速度控制发布者
        # 用于发送 Twist 消息控制海龟移动
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 订阅海龟当前位姿
        # 需要实时获取海龟位置来计算导航控制
        self.current_pose = None  # 存储当前位姿
        self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10,
            callback_group=self._callback_group
        )

        # 创建 Action Server
        # 参数说明：
        # - self: 当前节点实例
        # - MovePath: 动作类型
        # - 'move_path': 动作服务名称
        # - execute_callback: 执行目标的回调函数
        # - goal_callback: 验证目标的回调函数
        # - cancel_callback: 处理取消请求的回调函数
        self._action_server = ActionServer(
            self,
            MovePath,
            'move_path',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('MovePath 动作服务器已启动，监听 /move_path ✔')

    def pose_callback(self, msg: Pose):
        """
        位姿订阅回调函数
        
        实时更新海龟的当前位置和朝向，用于路径导航计算。
        
        Args:
            msg (Pose): 海龟当前位姿信息，包含 x, y, theta 等
        """
        self.current_pose = msg

    def goal_callback(self, goal_request: MovePath.Goal):
        """
        目标验证回调函数
        
        当收到新的路径目标时，验证其有效性。
        只有通过验证的目标才会被执行。
        
        Args:
            goal_request (MovePath.Goal): 客户端发送的目标请求
            
        Returns:
            GoalResponse: ACCEPT 或 REJECT
        """
        self.get_logger().info(
            f'收到路径目标：{len(goal_request.poses)} 个路径点，速度 {goal_request.speed:.2f} m/s'
        )
        
        # 验证路径不能为空
        if len(goal_request.poses) == 0:
            self.get_logger().warn('拒绝目标：路径为空 ❌')
            return GoalResponse.REJECT

        # 验证速度必须为正数
        if goal_request.speed <= 0.0:
            self.get_logger().warn('拒绝目标：速度必须大于 0 ❌')
            return GoalResponse.REJECT

        # 可以添加更多验证逻辑，如：
        # - 检查路径点是否在有效范围内
        # - 验证路径点之间的距离是否合理
        # - 检查速度是否在安全范围内

        self.get_logger().info('目标验证通过，接受执行 ✔')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        取消请求回调函数
        
        当客户端请求取消当前任务时被调用。
        可以根据当前状态决定是否允许取消。
        
        Args:
            goal_handle: 目标句柄
            
        Returns:
            CancelResponse: ACCEPT 或 REJECT
        """
        self.get_logger().info('收到取消请求，接受取消 ✔')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        核心执行逻辑（在单独线程中运行）
        
        这是 Action Server 的核心函数，负责执行整个路径移动任务。
        该函数在独立线程中运行，不会阻塞其他 ROS2 操作。
        
        Args:
            goal_handle: 目标句柄，包含目标数据和控制方法
            
        Returns:
            MovePath.Result: 执行结果
        """
        self.get_logger().info('开始执行路径移动任务...')

        # 提取目标参数
        poses = goal_handle.request.poses  # 路径点列表
        speed = goal_handle.request.speed  # 移动速度

        # 创建反馈消息对象
        feedback_msg = MovePath.Feedback()

        # 等待位姿数据可用
        # 必须先获取海龟当前位置才能开始导航
        while rclpy.ok() and self.current_pose is None:
            self.get_logger().info('等待海龟位姿数据...')
            time.sleep(0.1)

        # 遍历每个路径点，依次导航
        for idx, target in enumerate(poses):
            # 检查是否收到取消请求
            if goal_handle.is_cancel_requested:
                self.stop_turtle()
                goal_handle.canceled()
                result = MovePath.Result()
                result.success = False
                result.message = f'任务在第 {idx + 1} 个路径点被取消'
                self.get_logger().info(result.message)
                return result

            self.get_logger().info(
                f'导航到第 {idx + 1} 个路径点：x={target.x:.2f}, y={target.y:.2f}'
            )

            # 导航到当前目标点的控制循环
            while rclpy.ok():
                # 再次检查取消请求
                if goal_handle.is_cancel_requested:
                    self.stop_turtle()
                    goal_handle.canceled()
                    result = MovePath.Result()
                    result.success = False
                    result.message = f'任务在第 {idx + 1} 个路径点被取消'
                    self.get_logger().info(result.message)
                    return result

                # 确保位姿数据可用
                if self.current_pose is None:
                    time.sleep(0.05)
                    continue

                # 计算到目标点的距离和方向
                dx = target.x - self.current_pose.x
                dy = target.y - self.current_pose.y
                dist = math.hypot(dx, dy)  # 欧几里得距离

                # 检查是否到达目标点（距离阈值）
                if dist < 0.1:
                    self.stop_turtle()
                    self.get_logger().info(f'到达第 {idx + 1} 个路径点 ✔')
                    break

                # 计算导航控制
                # 1. 计算目标方向角
                angle_to_goal = math.atan2(dy, dx)
                
                # 2. 计算角度误差
                angle_diff = angle_to_goal - self.current_pose.theta
                
                # 3. 将角度误差归一化到 [-π, π] 范围
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

                # 4. 生成控制命令
                twist = Twist()

                # 角度误差较大时：主要进行旋转调整
                if abs(angle_diff) > 0.3:  # 约17度
                    twist.angular.z = 2.0 * angle_diff  # 角速度控制
                    twist.linear.x = 0.0                # 停止前进
                else:
                    # 角度误差较小时：边旋转边前进
                    twist.angular.z = 2.0 * angle_diff           # 微调角度
                    twist.linear.x = min(speed, dist)            # 前进速度（距离越近速度越慢）

                # 发布控制命令
                self.cmd_pub.publish(twist)

                # 发布实时反馈给客户端（节流：0.5秒一次）
                current_time = time.time()
                if current_time - self._last_log_time >= self._log_interval:
                    self._last_log_time = current_time
                    feedback_msg.current_index = idx
                    feedback_msg.current_target = Pose2D(
                        x=target.x, y=target.y, theta=target.theta
                    )
                    goal_handle.publish_feedback(feedback_msg)

                # 控制循环频率（20Hz）
                time.sleep(0.05)

        # 所有路径点都已到达
        self.stop_turtle()
        result = MovePath.Result()
        result.success = True
        result.message = f'路径执行完成！共访问了 {len(poses)} 个路径点。'
        self.get_logger().info(result.message)
        goal_handle.succeed()  # 标记目标成功完成
        return result

    def stop_turtle(self):
        """
        停止海龟移动
        
        发送零速度命令，让海龟立即停止移动和旋转。
        在到达路径点、任务取消或完成时调用。
        """
        twist = Twist()
        twist.linear.x = 0.0   # 停止前进
        twist.angular.z = 0.0  # 停止旋转
        self.cmd_pub.publish(twist)


def main(args=None):
    """
    主函数：程序入口点
    
    Args:
        args: 命令行参数（可选）
    """
    # 初始化 ROS2 Python 客户端库
    rclpy.init(args=args)
    
    # 创建路径移动动作服务器节点
    node = MovePathActionServer()
    
    # 使用多线程执行器，允许回调并行执行
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # 开始事件循环，处理客户端请求
        # 服务器会一直运行，等待和处理路径移动请求
        executor.spin()
    except KeyboardInterrupt:
        # 捕获 Ctrl+C 中断信号，优雅退出
        pass
    
    # 清理节点资源
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """
    当脚本直接运行时执行 main 函数
    
    使用方法：
    1. 启动 turtlesim：ros2 run turtlesim turtlesim_node
    2. 启动服务器：ros2 run turtle_controller move_path_server
    3. 启动客户端：ros2 run turtle_controller move_path_client
    
    服务器功能：
    - 接收路径移动请求并验证有效性
    - 实现智能导航算法（角度调整 + 直线移动）
    - 提供实时反馈和进度更新
    - 支持任务取消和错误处理
    
    导航算法特点：
    - 先转向目标方向，再前进移动
    - 距离目标越近，移动速度越慢
    - 到达阈值：0.1 单位距离
    """
    main()
