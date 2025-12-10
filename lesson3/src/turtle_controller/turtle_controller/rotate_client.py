#!/usr/bin/env python3
"""
ROS2 Action Client 示例：控制海龟旋转到指定角度

这个文件演示了如何创建一个 Action Client 来调用 turtlesim 的旋转动作服务。
Action 是 ROS2 中用于长时间运行任务的通信机制，支持：
- 目标设定 (Goal)
- 实时反馈 (Feedback) 
- 最终结果 (Result)
- 任务取消 (Cancel)
"""

# 导入 ROS2 Python 客户端库
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# 导入 turtlesim 包中定义的 RotateAbsolute 动作接口
from turtlesim.action import RotateAbsolute


class RotateClient(Node):
    """
    旋转客户端类
    
    继承自 rclpy.node.Node，实现一个 Action Client 来控制海龟旋转。
    这个类展示了 Action Client 的完整工作流程。
    """
    def __init__(self):
        """
        初始化旋转客户端
        
        设置 Action Client 并发送旋转目标
        """
        # 调用父类构造函数，设置节点名称为 'rotate_client'
        super().__init__('rotate_client')

        # 创建 Action Client
        # 参数说明：
        # - self: 当前节点实例
        # - RotateAbsolute: 动作类型（来自 turtlesim.action）
        # - '/turtle1/rotate_absolute': 动作服务器的话题名称
        self._client = ActionClient(self, RotateAbsolute, '/turtle1/rotate_absolute')
        
        # 等待动作服务器上线
        self.get_logger().info('等待 RotateAbsolute 动作服务器上线...')
        self._client.wait_for_server()  # 阻塞等待，直到服务器可用
        self.get_logger().info('动作服务器已就绪 ✔')

        # 声明并获取参数：目标角度（弧度制）
        # 默认值为 3.14 弧度（180度）
        self.declare_parameter('theta', 3.14)
        theta = self.get_parameter('theta').value
        
        # 记录上次输出反馈日志的时间，用于控制日志输出频率
        self._last_feedback_time = 0.0
        
        # 发送旋转目标
        self.send_goal(theta)

    def send_goal(self, theta: float):
        """
        发送旋转目标到动作服务器
        
        Args:
            theta (float): 目标角度（弧度制）
                          0 = 向右，π/2 = 向上，π = 向左，3π/2 = 向下
        """
        # 创建目标消息
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = float(theta)  # 设置目标角度

        self.get_logger().info(f'发送目标：旋转到 {theta:.2f} 弧度 ({theta*180/3.14159:.1f}°)')

        # 异步发送目标
        # send_goal_async 返回一个 Future 对象
        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback  # 设置反馈回调函数
        )
        
        # 当目标发送完成时，调用 goal_response_callback
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        处理目标响应的回调函数
        
        当服务器响应我们的目标请求时被调用。
        服务器可能接受或拒绝我们的目标。
        
        Args:
            future: 包含服务器响应的 Future 对象
        """
        # 获取目标句柄（Goal Handle）
        # 目标句柄用于跟踪特定目标的执行状态
        goal_handle = future.result()

        # 检查目标是否被接受
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝 ❌')
            rclpy.shutdown()  # 关闭 ROS2 系统
            return

        self.get_logger().info('目标已接受 ✔ 开始执行...')

        # 异步获取执行结果
        # get_result_async 返回一个 Future，当动作完成时会包含结果
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """
        处理动作执行过程中的反馈信息
        
        在动作执行期间，服务器会定期发送反馈信息，
        让客户端了解当前的执行进度。
        
        Args:
            feedback_msg: 包含反馈数据的消息
        """
        feedback = feedback_msg.feedback
        # remaining 表示还需要旋转的角度（弧度）
        remaining_degrees = feedback.remaining * 180 / 3.14159
        
        # 限制日志输出频率：每 0.5 秒输出一次，避免日志刷屏
        current_time = time.time()
        if current_time - self._last_feedback_time >= 0.5:
            self._last_feedback_time = current_time
            self.get_logger().info(f'执行反馈：还需旋转 {feedback.remaining:.2f} 弧度 ({remaining_degrees:.1f}°)')

    def result_callback(self, future):
        """
        处理动作执行完成后的结果
        
        当动作执行完成（成功或失败）时被调用。
        
        Args:
            future: 包含最终结果的 Future 对象
        
        注意：RotateAbsolute.Result 是空的，没有返回字段
        可以通过 `ros2 interface show turtlesim/action/RotateAbsolute` 查看接口定义
        """
        # 获取结果状态
        result_status = future.result().status
        
        # 检查执行状态（4 = SUCCEEDED）
        if result_status == 4:
            self.get_logger().info('旋转完成 ✔')
        else:
            self.get_logger().warn(f'动作结束，状态码：{result_status}')
        
        # 动作完成后关闭 ROS2 系统
        # 这会导致程序退出
        rclpy.shutdown()


def main(args=None):
    """
    主函数：程序入口点
    
    Args:
        args: 命令行参数（可选）
    """
    # 初始化 ROS2 Python 客户端库
    rclpy.init(args=args)
    
    # 创建旋转客户端节点
    node = RotateClient()
    
    try:
        # 开始事件循环，处理回调函数
        # spin() 会一直运行直到 rclpy.shutdown() 被调用
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获 Ctrl+C 中断信号，优雅退出
        pass
    
    # 清理节点资源
    node.destroy_node()


if __name__ == '__main__':
    """
    当脚本直接运行时执行 main 函数
    
    使用方法：
    1. 基本运行：python3 rotate_client.py
    2. 指定角度：ros2 run turtle_controller rotate_client --ros-args -p theta:=1.57
    
    角度参考：
    - 0 弧度 = 0° (向右)
    - π/2 弧度 ≈ 1.57 弧度 = 90° (向上)  
    - π 弧度 ≈ 3.14 弧度 = 180° (向左)
    - 3π/2 弧度 ≈ 4.71 弧度 = 270° (向下)
    """
    main()
