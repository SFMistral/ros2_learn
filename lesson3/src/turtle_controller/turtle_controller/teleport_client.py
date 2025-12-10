#!/usr/bin/env python3
"""
ROS2 服务客户端示例 - 海龟传送服务客户端

这个文件演示了如何创建一个ROS2服务客户端来调用turtlesim的传送服务。
主要学习内容：
1. 如何创建带参数的服务客户端
2. 如何声明和使用ROS2参数
3. 如何构造服务请求消息
4. 如何处理带返回值的服务调用
"""

# 导入必要的ROS2 Python库
import rclpy                        # ROS2 Python客户端库的核心模块
from rclpy.node import Node         # ROS2节点基类
from turtlesim.srv import TeleportAbsolute  # turtlesim包提供的绝对传送服务类型


class TeleportClient(Node):
    """
    海龟传送服务客户端类
    
    这个类继承自ROS2的Node基类，实现了一个服务客户端，
    用于调用turtlesim的传送服务，将海龟瞬间移动到指定位置和角度。
    """
    
    def __init__(self):
        """
        构造函数：初始化节点、服务客户端和参数
        """
        # 调用父类构造函数，创建名为'teleport_client'的节点
        super().__init__('teleport_client')
        
        # 创建服务客户端
        # 参数1: TeleportAbsolute - 服务类型（需要x, y, theta三个参数）
        # 参数2: '/turtle1/teleport_absolute' - 服务名称（针对turtle1的传送服务）
        self.cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        
        # 输出日志信息，告知用户正在等待服务
        self.get_logger().info('Waiting for /turtle1/teleport_absolute service...')
        
        # 等待服务变为可用状态
        # 这确保了turtlesim节点已经启动并提供服务
        self.cli.wait_for_service()
        
        # 服务可用后输出确认信息
        self.get_logger().info('Service available.')

        # 声明ROS2参数，支持外部传入坐标和角度
        # 这些参数可以通过命令行或launch文件设置
        self.declare_parameter('x', 5.0)      # X坐标，默认值5.0
        self.declare_parameter('y', 5.0)      # Y坐标，默认值5.0  
        self.declare_parameter('theta', 0.0)  # 角度（弧度），默认值0.0

        # 立即发送传送请求
        self.send_request()

    def send_request(self):
        """
        发送传送服务请求的方法
        
        从参数中获取目标位置和角度，构造请求消息并发送给服务端。
        """
        # 从ROS2参数中获取传送目标坐标和角度
        # get_parameter()返回参数对象，需要进一步获取具体数值
        x = self.get_parameter('x').get_parameter_value().double_value
        y = self.get_parameter('y').get_parameter_value().double_value
        theta = self.get_parameter('theta').get_parameter_value().double_value

        # 创建传送服务请求对象
        req = TeleportAbsolute.Request()
        
        # 设置请求参数
        # 将参数值转换为float类型并赋值给请求对象
        req.x = float(x)        # 目标X坐标
        req.y = float(y)        # 目标Y坐标
        req.theta = float(theta) # 目标角度（弧度制）

        # 输出即将执行的传送操作信息
        self.get_logger().info(f'Calling teleport to ({x:.2f}, {y:.2f}, {theta:.2f})')
        
        # 异步调用传送服务
        future = self.cli.call_async(req)
        
        # 等待服务调用完成
        # 这会阻塞执行直到服务端处理完请求并返回结果
        rclpy.spin_until_future_complete(self, future)

        # 检查服务调用结果
        if future.result() is not None:
            # 传送成功
            self.get_logger().info('Teleport succeeded.')
        else:
            # 传送失败（可能是服务端错误或网络问题）
            self.get_logger().error('Teleport failed.')


def main(args=None):
    """
    主函数：程序入口点
    
    Args:
        args: 命令行参数（可选）
        
    使用示例：
        # 使用默认参数传送到(5.0, 5.0, 0.0)
        ros2 run turtle_controller teleport_client
        
        # 传送到指定位置
        ros2 run turtle_controller teleport_client --ros-args -p x:=2.0 -p y:=3.0 -p theta:=1.57
    """
    # 初始化ROS2 Python客户端库
    rclpy.init(args=args)
    
    # 创建并运行传送客户端节点
    # 节点创建后会立即执行传送操作
    node = TeleportClient()
    
    # 销毁节点，释放资源
    node.destroy_node()
    
    # 关闭ROS2 Python客户端库
    rclpy.shutdown()


# Python脚本入口点
# 当直接运行此文件时（而非作为模块导入），执行main函数
if __name__ == '__main__':
    main()
