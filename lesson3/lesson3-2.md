# Lesson 3.2: ROS2 服务与路径跟随系统

## 学习目标

在本课程中，你将学习：
- 如何创建ROS2服务节点（服务器和客户端）
- 理解请求-响应通信模式
- 实现平滑的路径跟随控制
- 掌握速度控制和位置反馈
- 学习多线程执行器的使用
- 命令行参数解析和处理

## 前置条件

- 完成Lesson 3.1的学习
- 已安装ROS2 Humble
- 熟悉Python编程和基本的数学（三角函数）
- 理解ROS2发布-订阅和服务通信模式

## 项目概述

本课程将实现一个完整的路径跟随系统，包括：
- **服务端**：接收路径请求，控制海龟平滑移动
- **客户端**：通过命令行参数发送路径点，自动计算朝向角度
- **核心功能**：速度控制、位置反馈、多点路径规划

## 项目结构

```
lesson3/
├── src/
│   ├── turtle_controller/
│   │   ├── turtle_controller/
│   │   │   ├── __init__.py
│   │   │   ├── turtle_circle.py           # 圆形运动（Lesson 2）
│   │   │   ├── turtle_pose_sub.py         # 位置监控（Lesson 3.1）
│   │   │   ├── follow_path_service.py     # 路径跟随服务端（本课程）
│   │   │   └── follow_path_client.py      # 路径跟随客户端（本课程）
│   │   ├── setup.py
│   │   └── package.xml
│   └── turtle_interfaces/
│       ├── srv/
│       │   └── FollowPath.srv             # 自定义服务接口
│       └── package.xml
├── build/
├── install/
└── log/
```

## 核心概念

### 1. ROS2服务（Service）

服务是一种请求-响应通信模式：

```
客户端                          服务端
  |                              |
  |-------- 发送请求 -------->   |
  |                              |
  |                         处理请求
  |                              |
  |  <------ 返回响应 --------   |
  |                              |
```

**特点**：
- 同步通信：客户端等待响应
- 一对多：一个服务可以有多个客户端
- 请求-响应结构化数据

### 2. 自定义服务接口（FollowPath.srv）

```
# 请求部分
geometry_msgs/Pose2D[] poses

---

# 响应部分
bool success
string message
```

### 3. Pose2D消息类型

```python
float64 x      # x坐标
float64 y      # y坐标
float64 theta  # 朝向角度（弧度）
```

### 4. Twist消息类型（速度控制）

```python
geometry_msgs/Vector3 linear   # 线速度 (x, y, z)
geometry_msgs/Vector3 angular  # 角速度 (x, y, z)
```

对于turtlesim：
- `linear.x`：前进速度
- `angular.z`：旋转速度

### 5. 多线程执行器

在服务回调中调用异步操作时，需要使用多线程执行器避免死锁：

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
```

## 代码详解

### follow_path_service.py 分析

#### 1. 初始化与订阅

```python
def __init__(self):
    super().__init__('follow_path_service')
    
    # 使用可重入回调组
    self.callback_group = ReentrantCallbackGroup()
    
    # 订阅海龟位置
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
```

#### 2. 位置反馈机制

```python
def pose_callback(self, msg):
    """线程安全地更新位置"""
    with self.pose_lock:
        self.current_pose = msg

def get_current_pose(self):
    """线程安全地获取位置"""
    with self.pose_lock:
        return self.current_pose
```

#### 3. 角度归一化

```python
def normalize_angle(self, angle):
    """将角度归一化到 [-π, π]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
```

#### 4. 平滑移动算法

```python
def move_to_goal(self, goal_x, goal_y, goal_theta):
    """
    两阶段移动：
    1. 移动到目标位置
    2. 调整到目标朝向
    """
    # 第一阶段：移动到目标位置
    while distance > self.distance_tolerance:
        # 计算目标方向
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - pose.theta)
        
        # 如果角度偏差大，先转向
        if abs(angle_diff) > 0.3:
            cmd.angular.z = self.angular_speed
            cmd.linear.x = 0.0
        else:
            # 边走边调整方向
            cmd.linear.x = min(self.linear_speed, distance)
            cmd.angular.z = 2.0 * angle_diff
        
        self.vel_pub.publish(cmd)
    
    # 第二阶段：调整朝向
    while abs(angle_diff) > self.angle_tolerance:
        cmd.angular.z = self.angular_speed
        self.vel_pub.publish(cmd)
```

### follow_path_client.py 分析

#### 1. 参数解析

```python
def parse_points(self, points_str):
    """
    解析点字符串
    格式: "x1,y1;x2,y2;x3,y3"
    """
    points = []
    for point_str in points_str.split(';'):
        parts = point_str.split(',')
        x = float(parts[0].strip())
        y = float(parts[1].strip())
        points.append((x, y))
    return points
```

#### 2. 自动角度计算

```python
def calculate_angles(self, points):
    """
    计算每个点的朝向角度
    - 每个点的角度 = 指向下一个点的方向
    - 最后一个点保持前进方向的角度
    """
    for i in range(n):
        if i < n - 1:
            # 指向下一个点
            next_x, next_y = points[i + 1]
            theta = math.atan2(next_y - y, next_x - x)
        else:
            # 最后一个点保持前进方向
            prev_x, prev_y = points[i - 1]
            theta = math.atan2(y - prev_y, x - prev_x)
        
        poses.append(Pose2D(x=x, y=y, theta=theta))
```

#### 3. 服务调用

```python
def send_request(self, points):
    """构造并发送路径跟随请求"""
    req = FollowPath.Request()
    req.poses = self.calculate_angles(points)
    
    # 异步调用服务
    future = self.cli.call_async(req)
    
    # 等待响应
    rclpy.spin_until_future_complete(self, future)
    
    # 处理响应
    if future.result() is not None:
        resp = future.result()
        self.get_logger().info(f'Success: {resp.success}, Message: {resp.message}')
```

## 运行步骤

### 1. 编译包

```bash
cd lesson3
colcon build --packages-select turtle_interfaces turtle_controller
source install/setup.bash
```

### 2. 启动turtlesim

```bash
ros2 run turtlesim turtlesim_node
```

### 3. 启动服务端

在新终端中：

```bash
source install/setup.bash
ros2 run turtle_controller follow_path_service
```

### 4. 发送路径请求

在新终端中：

```bash
source install/setup.bash
ros2 run turtle_controller follow_path_client --ros-args -p points:="2,2;8,2;8,8;2,8"
```

## 使用示例

### 示例1：简单矩形路径

```bash
ros2 run turtle_controller follow_path_client --ros-args -p points:="2,2;8,2;8,8;2,8"
```

海龟将按顺序访问四个点，形成矩形轨迹。

### 示例2：复杂路径

```bash
ros2 run turtle_controller follow_path_client --ros-args -p points:="2,2;8,2;8,8;2,8;5,5;5,2;2,8"
```

海龟将按顺序访问七个点。

### 示例3：单点移动

```bash
ros2 run turtle_controller follow_path_client --ros-args -p points:="5,5"
```

海龟移动到中心点。

## 参数说明

### 服务端参数

在`follow_path_service.py`中可调整：

```python
self.linear_speed = 1.5        # 线速度（m/s）
self.angular_speed = 3.0       # 角速度（rad/s）
self.distance_tolerance = 0.1  # 到达目标的距离容差（m）
self.angle_tolerance = 0.05    # 角度容差（rad）
```

### 客户端参数

```bash
# 参数格式
--ros-args -p points:="x1,y1;x2,y2;..."

# 坐标范围：0-11（turtlesim窗口大小）
# 分隔符：分号(;)分隔点，逗号(,)分隔坐标
```

## 实验与探索

### 实验1：调整运动速度

修改服务端的速度参数：

```python
# 快速移动
self.linear_speed = 3.0
self.angular_speed = 6.0

# 缓慢移动
self.linear_speed = 0.5
self.angular_speed = 1.0
```

### 实验2：改变容差值

```python
# 精确到达
self.distance_tolerance = 0.01
self.angle_tolerance = 0.01

# 粗略到达
self.distance_tolerance = 0.5
self.angle_tolerance = 0.2
```

### 实验3：自定义路径

创建有趣的路径形状：

```bash
# 三角形
ros2 run turtle_controller follow_path_client --ros-args -p points:="2,2;8,2;5,8"

# 五边形
ros2 run turtle_controller follow_path_client --ros-args -p points:="5.5,2;8.5,4;7.5,7.5;3.5,7.5;2.5,4"

# 螺旋形（多个点）
ros2 run turtle_controller follow_path_client --ros-args -p points:="5.5,5.5;6,5;6.5,5.5;6.5,6;6,6.5;5.5,6.5;5,6;5,5;5,4;6,4;7,4;8,5;8,6;8,7;7,8;6,8;5,8"
```

### 实验4：监控执行过程

在另一个终端运行位置监控：

```bash
source install/setup.bash
ros2 run turtle_controller turtle_pose_sub
```

观察海龟的实时位置变化。

## 故障排除

### 常见问题

1. **服务调用超时**
   - 检查服务端是否正在运行
   - 查看日志输出是否有错误
   - 增加超时时间

2. **海龟不移动**
   - 确认turtlesim节点正在运行
   - 检查速度参数是否为0
   - 验证坐标是否在有效范围内（0-11）

3. **路径不平滑**
   - 减小`distance_tolerance`和`angle_tolerance`
   - 增加`linear_speed`和`angular_speed`
   - 检查消息发布频率

4. **死锁问题**
   - 确保使用了`MultiThreadedExecutor`
   - 检查是否在回调中使用了`spin_until_future_complete()`

### 调试命令

```bash
# 查看服务列表
ros2 service list

# 查看服务信息
ros2 service info /follow_path

# 查看话题列表
ros2 topic list

# 监听速度命令
ros2 topic echo /turtle1/cmd_vel

# 监听位置信息
ros2 topic echo /turtle1/pose

# 查看节点图
rqt_graph
```

## 性能优化

### 1. 控制循环频率

```python
rate_hz = 20
sleep_time = 1.0 / rate_hz
```

增加频率可以提高控制精度，但会增加CPU使用率。

### 2. 动态速度调整

```python
# 接近目标时减速
if distance < 1.0:
    cmd.linear.x *= 0.5

# 角度偏差大时减速
if abs(angle_diff) > 1.0:
    cmd.angular.z *= 0.5
```

### 3. 预测性控制

```python
# 提前减速以避免超调
deceleration_distance = (self.linear_speed ** 2) / (2 * 0.5)
if distance < deceleration_distance:
    cmd.linear.x = math.sqrt(2 * 0.5 * distance)
```

## 扩展功能

### 功能1：路径可视化

添加路径点标记：

```python
# 在服务端添加标记发布
self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)

def publish_path_markers(self, poses):
    markers = MarkerArray()
    for i, pose in enumerate(poses):
        marker = Marker()
        marker.id = i
        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        markers.markers.append(marker)
    self.marker_pub.publish(markers)
```

### 功能2：路径优化

实现最短路径算法：

```python
def optimize_path(self, points):
    """使用贪心算法优化路径"""
    # 实现TSP（旅行商问题）的近似解
    pass
```

### 功能3：障碍物避免

添加简单的障碍物检测：

```python
def check_collision(self, target_x, target_y):
    """检查是否会碰撞"""
    # 实现碰撞检测逻辑
    pass
```

## 相关资源

- [ROS2服务教程](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Twist消息文档](https://docs.ros2.org/latest/api/geometry_msgs/)
- [turtlesim文档](https://docs.ros2.org/latest/api/turtlesim/)
- [多线程执行器](https://docs.ros2.org/en/humble/Concepts/About-Executors.html)

## 总结

本课程学习了：
- ✅ ROS2服务的创建和使用
- ✅ 请求-响应通信模式
- ✅ 速度控制和位置反馈
- ✅ 平滑路径跟随算法
- ✅ 多线程执行器的应用
- ✅ 命令行参数解析
- ✅ 自动角度计算

通过本课程，你已经掌握了ROS2的三种主要通信方式（发布-订阅、服务、动作的基础）。结合这些知识，你可以构建复杂的机器人控制系统。

## 下一步

- 学习ROS2动作系统（Action）
- 实现更复杂的路径规划算法
- 添加实时可视化功能
- 集成传感器数据进行自主导航
