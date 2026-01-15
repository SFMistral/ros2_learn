#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from tf2_ros import TransformBroadcaster


class TurtleTFBroadcaster(Node):
    def __init__(self):
        super().__init__('turtle_tf_broadcaster')

        # 创建 TF broadcaster
        self.br = TransformBroadcaster(self)

        # 订阅小乌龟的位姿
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('turtle_tf_broadcaster started. Listening /turtle1/pose')

    def pose_callback(self, msg: Pose):
        # 创建一个 TransformStamped 消息
        t = TransformStamped()

        # 时间戳和坐标系名字
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'     # 父坐标
        t.child_frame_id = 'turtle1'    # 子坐标

        # 平移：直接用 turtlesim 的 x, y
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # 旋转：turtlesim 的 theta 是平面上的 yaw
        # 转成四元数 (绕 z 轴旋转)
        yaw = msg.theta
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # 发送 TF
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
