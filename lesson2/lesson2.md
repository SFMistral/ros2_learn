# Lesson 2: ROS2 发布者节点与海龟控制

## 学习目标

在本课程中，你将学习：
- 如何创建ROS2发布者节点
- 理解Twist消息类型
- 控制turtlesim海龟的运动
- 掌握ROS2包的基本结构
- 学习定时器的使用

## 前置条件

- 完成Lesson 1的学习
- 已安装ROS2 Humble
- 熟悉基本的Python编程

## 项目概述

本课程将创建一个名为`turtle_controller`的ROS2包，其中包含一个节点来控制turtlesim中的海龟做圆形运动。

## 项目结构

```
lesson2/
├── src/
│   └── turtle_controller/
│       ├── turtle_controller/
│       │   ├── __init__.py
│       │   └── turtle_circle.py      # 主要的控制节点
│       ├── resource/
│       ├── test/
│       ├── package.xml               # 包描述文件
│       ├── setup.cfg                 # 配置文件
│       └── setup.py                  # 安装脚本
├── build/                            # 编译输出目录
├── install/                          # 安装目录
└── log/                             # 日志目录
```

## 核心概念

### 1. ROS2发布者（Publisher）

发布者是ROS2中用于发送消息的组件：
- 发布者向特定话题发送消息
- 可以有多个订阅者接收同一个话题的消息
- 消息类型必须匹配

### 2. Twist消息类型

`geometry_msgs/msg/Twist`是ROS2中用于表示速度的标准消息类型：

```python
# Twist消息结构
geometry_msgs/Vector3 linear   # 线性速度 (x, y, z)
geometry_msgs/Vector3 angular  # 角速度 (x, y, z)
```

对于2D移动（如turtlesim）：
- `linear.x`: 前进/后退速度
- `angular.z`: 旋转速度

### 3. 定时器（Timer）

定时器用于周期性执行任务：
- 设置执行频率
- 绑定回调函数
- 自动管理执行周期

## 代码详解

### turtle_circle.py 分析

让我们逐步分析主要代码：

#### 1. 导入必要的模块

```python
import rclpy                    # ROS2 Python客户端库
from rclpy.node import Node     # 节点基类
from geometry_msgs.msg import Twist  # Twist消息类型
```

#### 2. 创建节点类

```python
class TurtleCircle(Node):
    def __init__(self):
        super().__init__('turtle_circle')  # 设置节点名称
```

#### 3. 创建发布者

```python
self.publisher_ = self.create_publisher(
    Twist,              # 消息类型
    '/turtle1/cmd_vel', # 话题名称
    10                  # 队列大小
)
```

#### 4. 设置定时器

```python
self.timer = self.create_timer(
    0.1,                    # 时间间隔（秒）
    self.timer_callback     # 回调函数
)
```

#### 5. 发布速度命令

```python
def timer_callback(self):
    msg = Twist()
    msg.linear.x = 1.0   # 前进速度
    msg.angular.z = 1.0  # 旋转速度
    self.publisher_.publish(msg)
```

## 运行步骤

### 1. 编译包

在lesson2目录下执行：

```bash
# 编译包
colcon build

# 设置环境变量
source install/setup.bash
```

### 2. 启动turtlesim

在新终端中：

```bash
# 启动turtlesim节点
ros2 run turtlesim turtlesim_node
```

### 3. 运行控制节点

在另一个终端中：

```bash
# 设置环境变量（如果需要）
source install/setup.bash

# 运行海龟控制节点
ros2 run turtle_controller turtle_circle
```

## 预期结果

运行成功后，你应该看到：
1. turtlesim窗口中的海龟开始做圆形运动
2. 海龟以恒定的线性和角速度移动
3. 形成一个圆形轨迹

## 实验与探索

### 实验1：修改运动参数

尝试修改`turtle_circle.py`中的速度值：

```python
# 实验不同的速度组合
msg.linear.x = 2.0   # 增加线性速度
msg.angular.z = 0.5  # 减少角速度
```

观察海龟运动轨迹的变化。

### 实验2：改变发布频率

修改定时器频率：

```python
# 改变定时器频率
self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
```

### 实验3：添加运动模式

扩展代码，实现不同的运动模式：
- 直线运动
- 方形轨迹
- 8字形轨迹

## 故障排除

### 常见问题

1. **编译错误**
   - 检查package.xml和setup.py配置
   - 确保所有依赖项已安装

2. **节点无法启动**
   - 检查环境变量设置
   - 确保已正确编译包

3. **海龟不动**
   - 检查话题名称是否正确
   - 使用`ros2 topic list`查看可用话题

### 调试命令

```bash
# 查看活动节点
ros2 node list

# 查看话题信息
ros2 topic info /turtle1/cmd_vel

# 监听话题消息
ros2 topic echo /turtle1/cmd_vel

# 查看节点图
rqt_graph
```

## 扩展学习

### 下一步学习方向

1. **订阅者节点**: 学习如何接收和处理消息
2. **服务**: 了解请求-响应通信模式
3. **参数**: 学习动态配置节点行为
4. **启动文件**: 自动化多节点启动

### 相关资源

- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [geometry_msgs文档](https://docs.ros2.org/latest/api/geometry_msgs/)
- [rclpy API参考](https://docs.ros2.org/latest/api/rclpy/)

## 总结

本课程学习了：
- ✅ ROS2发布者的创建和使用
- ✅ Twist消息类型的结构和应用
- ✅ 定时器的使用方法
- ✅ 海龟运动控制的基本原理
- ✅ ROS2包的基本结构

通过这个简单的例子，你已经掌握了ROS2中最重要的概念之一：发布者-订阅者模式。这是构建更复杂机器人应用的基础。

## 作业

1. 修改代码，让海龟画出一个正方形
2. 创建一个新节点，让海龟在到达边界时自动转向
3. 实现一个可以通过参数控制速度的节点

---

**下一课预告**: Lesson 3将学习订阅者节点，实现海龟的键盘控制功能。