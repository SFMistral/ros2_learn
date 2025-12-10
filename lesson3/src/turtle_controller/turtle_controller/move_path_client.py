#!/usr/bin/env python3
"""
ROS2 Action Client 示例：控制海龟原地旋转

这个文件演示了如何创建一个 Action Client 来调用自定义的 MovePath 动作服务。
与 RotateAbsolute 不同，这是我们自己定义的动作接口，展示了：
- 自定义 Action 接口的使用
- 原地旋转的客户端实现
- 日志输出频率控制
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# 导入几何消息类型，用于表示 2D 位姿
from geometry_msgs.msg import Pose2D
# 导入我们自定义的 MovePath 动作接口
from turtle_interfaces.action import MovePath


class MovePathActionClient(Node):
    """
    原地旋转客户端类
    
    继承自 rclpy.node.Node，实现一个 Action Client 来控制海龟原地旋转。
    这个类展示了如何使用自定义 Action 接口进行旋转任务。
    """
    def __init__(self):
        """
        初始化原地旋转客户端
        
        设置 Action Client 并发送旋转目标
        """
        # 调用父类构造函数，设置节点名称
        super().__init__('move_path_action_client')

        # 日志节流：记录上次日志输出时间
        self._last_log_time = 0.0
        self._log_interval = 0.5  # 日志输出间隔（秒）

        # 创建 Action Client
        # 参数说明：
        # - self: 当前节点实例
        # - MovePath: 自定义动作类型（来自 turtle_interfaces.action）
        # - 'move_path': 动作服务器的话题名称
        self._client = ActionClient(self, MovePath, 'move_path')
        
        # 等待动作服务器上线
        self.get_logger().info('等待 MovePath 动作服务器上线...')
        self._client.wait_for_server()  # 阻塞等待，直到服务器可用
        self.get_logger().info('MovePath 动作服务器已就绪 ✔')

        # 发送原地旋转目标
        self.send_goal()

    def send_goal(self):
        """
        发送路径移动目标到动作服务器
        
        构造一个矩形路径，让海龟按顺序访问四个角点
        """
        # 创建目标消息
        goal_msg = MovePath.Goal()
        goal_msg.speed = 1.0  # 设置移动速度（单位：m/s）

        # 构造一个矩形路径的四个顶点
        # Pose2D 包含 x, y 坐标和朝向角 theta
        p1 = Pose2D(x=2.0, y=2.0, theta=0.0)  # 左下角
        p2 = Pose2D(x=8.0, y=2.0, theta=0.0)  # 右下角  
        p3 = Pose2D(x=8.0, y=8.0, theta=0.0)  # 右上角
        p4 = Pose2D(x=2.0, y=8.0, theta=0.0)  # 左上角

        # 将路径点添加到目标消息中
        # 海龟将按照列表顺序依次访问这些点
        goal_msg.poses = [p1, p2, p3, p4]

        self.get_logger().info(f'发送路径目标：{len(goal_msg.poses)} 个路径点，速度 {goal_msg.speed} m/s')
        
        # 打印路径详情
        for i, pose in enumerate(goal_msg.poses):
            self.get_logger().info(f'  点 {i+1}: ({pose.x:.1f}, {pose.y:.1f})')

        # 异步发送目标
        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback  # 设置反馈回调函数
        )
        
        # 当目标发送完成时，调用响应回调
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        处理目标响应的回调函数
        
        当服务器响应我们的路径目标请求时被调用。
        服务器会检查路径的有效性（如路径点数量、速度等）。
        
        Args:
            future: 包含服务器响应的 Future 对象
        """
        # 获取目标句柄
        goal_handle = future.result()
        
        # 检查目标是否被接受
        if not goal_handle.accepted:
            self.get_logger().error('路径目标被拒绝 ❌ (可能是路径为空或速度无效)')
            rclpy.shutdown()
            return

        self.get_logger().info('路径目标已接受 ✔ 开始执行路径...')
        
        # 异步获取执行结果
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """
        处理路径执行过程中的反馈信息
        
        在路径执行期间，服务器会定期发送反馈信息，
        告诉客户端当前正在前往哪个路径点。
        日志输出频率限制为 0.5 秒一次。
        
        Args:
            feedback_msg: 包含反馈数据的消息
        """
        # 日志节流：每 0.5 秒输出一次
        current_time = time.time()
        if current_time - self._last_log_time < self._log_interval:
            return  # 跳过本次日志输出
        self._last_log_time = current_time

        feedback = feedback_msg.feedback
        idx = feedback.current_index        # 当前目标点的索引
        target = feedback.current_target    # 当前目标点的坐标
        
        self.get_logger().info(
            f'执行反馈：正在前往第 {idx + 1} 个路径点 '
            f'({target.x:.2f}, {target.y:.2f})'
        )

    def result_callback(self, future):
        """
        处理路径执行完成后的结果
        
        当整个路径执行完成（成功或失败）时被调用。
        
        Args:
            future: 包含最终结果的 Future 对象
        """
        result = future.result().result
        
        # 根据执行结果显示不同的消息
        if result.success:
            self.get_logger().info(f'路径执行成功 ✔ {result.message}')
        else:
            self.get_logger().error(f'路径执行失败 ❌ {result.message}')
        
        # 任务完成后关闭 ROS2 系统
        rclpy.shutdown()


def main(args=None):
    """
    主函数：程序入口点
    
    Args:
        args: 命令行参数（可选）
    """
    # 初始化 ROS2 Python 客户端库
    rclpy.init(args=args)
    
    # 创建路径移动客户端节点
    node = MovePathActionClient()
    
    try:
        # 开始事件循环，处理回调函数
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获 Ctrl+C 中断信号，优雅退出
        pass
    finally:
        # 清理节点资源
        node.destroy_node()
        # 只在上下文仍然有效时才调用 shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    """
    当脚本直接运行时执行 main 函数
    
    使用方法：
    1. 启动 turtlesim：ros2 run turtlesim turtlesim_node
    2. 启动路径服务器：ros2 run turtle_controller move_path_server  
    3. 运行客户端：ros2 run turtle_controller move_path_client
    
    预期行为：
    - 海龟将沿矩形路径移动，依次访问四个角点
    - 客户端会显示当前正在前往的路径点
    - 完成后显示执行结果
    """
    main()
