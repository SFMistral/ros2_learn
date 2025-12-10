#!/usr/bin/env python3
"""
ROS2 服务客户端示例 - 清屏服务客户端

这个文件演示了如何创建一个ROS2服务客户端来调用turtlesim的清屏服务。
主要学习内容：
1. 如何创建服务客户端
2. 如何等待服务可用
3. 如何发送服务请求
4. 如何处理服务响应
"""

# 导入必要的ROS2 Python库
import rclpy                    # ROS2 Python客户端库的核心模块
from rclpy.node import Node     # ROS2节点基类
from std_srvs.srv import Empty  # 标准服务类型：空服务（无参数输入输出）


class ClearClient(Node):
    """
    清屏服务客户端类
    
    这个类继承自ROS2的Node基类，实现了一个服务客户端，
    用于调用turtlesim的/clear服务来清除绘图轨迹。
    """
    
    def __init__(self):
        """
        构造函数：初始化节点和服务客户端
        """
        # 调用父类构造函数，创建名为'clear_client'的节点
        super().__init__('clear_client')
        
        # 创建服务客户端
        # 参数1: Empty - 服务类型（无输入输出参数的空服务）
        # 参数2: '/clear' - 服务名称（turtlesim提供的清屏服务）
        self.cli = self.create_client(Empty, '/clear')
        
        # 输出日志信息，告知用户正在等待服务
        self.get_logger().info('Waiting for /clear service...')
        
        # 等待服务变为可用状态
        # 这是一个阻塞调用，会一直等待直到服务端启动
        self.cli.wait_for_service()
        
        # 服务可用后输出确认信息
        self.get_logger().info('/clear service available, calling it now.')
        
        # 立即发送服务请求
        self.send_request()

    def send_request(self):
        """
        发送服务请求的方法
        
        创建请求对象并异步调用服务，然后等待响应结果。
        """
        # 创建空服务请求对象
        # Empty服务不需要任何输入参数
        req = Empty.Request()
        
        # 异步调用服务
        # call_async()返回一个Future对象，用于获取异步结果
        future = self.cli.call_async(req)

        # 等待异步调用完成
        # spin_until_future_complete()会阻塞执行直到服务调用完成
        rclpy.spin_until_future_complete(self, future)
        
        # 检查服务调用结果
        if future.result() is not None:
            # 服务调用成功
            self.get_logger().info('Clear service call succeeded.')
        else:
            # 服务调用失败
            self.get_logger().error('Clear service call failed.')


def main(args=None):
    """
    主函数：程序入口点
    
    Args:
        args: 命令行参数（可选）
    """
    # 初始化ROS2 Python客户端库
    # 这是使用ROS2功能的必要步骤
    rclpy.init(args=args)
    
    # 创建并运行清屏客户端节点
    # 注意：由于在__init__中就完成了服务调用，节点创建后立即执行清屏操作
    node = ClearClient()
    
    # 销毁节点，释放资源
    node.destroy_node()
    
    # 关闭ROS2 Python客户端库
    rclpy.shutdown()


# Python脚本入口点
# 当直接运行此文件时（而非作为模块导入），执行main函数
if __name__ == '__main__':
    main()
