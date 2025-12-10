# Lesson 3-3: ROS2 自定义 Action 接口与路径导航

## 学习目标

- 理解 Action 接口的三部分结构（Goal / Feedback / Result）
- 学会定义自定义 Action 接口
- 实现复杂的 Action Server（路径导航服务器）
- 创建 Action Client 调用自定义服务
- 掌握路径规划和导航控制算法

## Action 机制回顾

Action 是 ROS2 中用于长时间运行任务的通信机制，特别适合：
- 需要时间执行的任务（如移动、旋转）
- 需要实时反馈的操作（如进度更新）
- 可能需要取消的任务

### Action 的三部分结构

```
Goal（目标）    → 客户端发送给服务器的任务请求
Feedback（反馈） → 服务器向客户端报告执行进度  
Result（结果）   → 任务完成后的最终结果
```

## 1. 定义自定义 Action 接口

### 1.1 创建 MovePath.action 接口

在 `turtle_interfaces/action/MovePath.action` 中定义路径移动接口：

```action
# Goal：一串目标点 + 速度
geometry_msgs/Pose2D[] poses
float32 speed
---
# Result：是否成功
bool success
string message
---
# Feedback：当前正在走到第几个点
int32 current_index
geometry_msgs/Pose2D current_target
```

**接口说明：**
- **Goal**: 包含路径点数组和移动速度
- **Result**: 返回执行成功状态和描述信息
- **Feedback**: 实时报告当前正在前往的路径点

### 1.2 配置接口包

确保 `turtle_interfaces` 包正确配置了 Action 接口生成：

**CMakeLists.txt**:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FollowPath.srv"
  "action/MovePath.action"
  DEPENDENCIES geometry_msgs
)
```

**package.xml**:
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## 2. 实现 Action Server（路径导航服务器）

### 2.1 服务器核心功能

`move_path_server.py` 实现了完整的路径导航服务器：

```python
class MovePathActionServer(Node):
    def __init__(self):
        super().__init__('move_path_action_server')
        
        # 速度控制发布者
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 位姿订阅者
        self.current_pose = None
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # Action Server
        self._action_server = ActionServer(
            self, MovePath, 'move_path',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
```

### 2.2 目标验证机制

```python
def goal_callback(self, goal_request: MovePath.Goal):
    """验证目标的有效性"""
    # 检查路径不能为空
    if len(goal_request.poses) == 0:
        return GoalResponse.REJECT
    
    # 检查速度必须为正数
    if goal_request.speed <= 0.0:
        return GoalResponse.REJECT
        
    return GoalResponse.ACCEPT
```

### 2.3 导航控制算法

核心导航算法采用"先转向，后前进"的策略：

```python
# 计算到目标的距离和方向
dx = target.x - self.current_pose.x
dy = target.y - self.current_pose.y
dist = math.hypot(dx, dy)

# 计算目标方向角和角度误差
angle_to_goal = math.atan2(dy, dx)
angle_diff = angle_to_goal - self.current_pose.theta
angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

# 生成控制命令
twist = Twist()
if abs(angle_diff) > 0.3:  # 角度误差大时主要旋转
    twist.angular.z = 2.0 * angle_diff
    twist.linear.x = 0.0
else:  # 角度误差小时边转边进
    twist.angular.z = 2.0 * angle_diff
    twist.linear.x = min(speed, dist)
```

### 2.4 实时反馈机制

```python
# 发布执行进度反馈
feedback_msg.current_index = idx
feedback_msg.current_target = Pose2D(x=target.x, y=target.y, theta=target.theta)
goal_handle.publish_feedback(feedback_msg)
```

## 3. 实现 Action Client（路径请求客户端）

### 3.1 客户端基本结构

`move_path_client.py` 实现了路径移动请求客户端：

```python
class MovePathActionClient(Node):
    def __init__(self):
        super().__init__('move_path_action_client')
        
        # 创建 Action Client
        self._client = ActionClient(self, MovePath, 'move_path')
        self._client.wait_for_server()
        
        # 发送路径目标
        self.send_goal()
```

### 3.2 构造路径目标

```python
def send_goal(self):
    goal_msg = MovePath.Goal()
    goal_msg.speed = 1.0
    
    # 构造矩形路径
    p1 = Pose2D(x=2.0, y=2.0, theta=0.0)  # 左下角
    p2 = Pose2D(x=8.0, y=2.0, theta=0.0)  # 右下角  
    p3 = Pose2D(x=8.0, y=8.0, theta=0.0)  # 右上角
    p4 = Pose2D(x=2.0, y=8.0, theta=0.0)  # 左上角
    
    goal_msg.poses = [p1, p2, p3, p4]
```

### 3.3 处理反馈和结果

```python
def feedback_callback(self, feedback_msg):
    """处理实时反馈"""
    feedback = feedback_msg.feedback
    idx = feedback.current_index
    target = feedback.current_target
    self.get_logger().info(f'正在前往第 {idx + 1} 个路径点 ({target.x:.2f}, {target.y:.2f})')

def result_callback(self, future):
    """处理最终结果"""
    result = future.result().result
    if result.success:
        self.get_logger().info(f'路径执行成功 ✔ {result.message}')
    else:
        self.get_logger().error(f'路径执行失败 ❌ {result.message}')
```

## 4. 编译和运行

### 4.1 编译接口和代码

```bash
# 编译接口包
colcon build --packages-select turtle_interfaces

# 编译控制器包  
colcon build --packages-select turtle_controller

# 加载环境
source install/setup.bash
```

### 4.2 运行演示

**终端1 - 启动 turtlesim**:
```bash
ros2 run turtlesim turtlesim_node
```

**终端2 - 启动路径服务器**:
```bash
ros2 run turtle_controller move_path_server
```

**终端3 - 启动路径客户端**:
```bash
ros2 run turtle_controller move_path_client
```

### 4.3 预期行为

- 海龟将沿矩形路径移动，依次访问四个角点
- 服务器输出导航进度和到达信息
- 客户端显示当前正在前往的路径点
- 完成后显示执行结果

## 5. 命令行工具测试

### 5.1 查看 Action 接口

```bash
# 列出可用的 Action
ros2 action list

# 查看 MovePath Action 接口定义
ros2 interface show turtle_interfaces/action/MovePath

# 查看 Action 服务器信息
ros2 action info /move_path
```

### 5.2 命令行调用 Action

```bash
# 发送单点路径
ros2 action send_goal /move_path turtle_interfaces/action/MovePath "{poses: [{x: 5.0, y: 5.0, theta: 0.0}], speed: 2.0}"

# 发送多点路径
ros2 action send_goal /move_path turtle_interfaces/action/MovePath "{poses: [{x: 2.0, y: 2.0, theta: 0.0}, {x: 8.0, y: 8.0, theta: 0.0}], speed: 1.5}"
```

## 6. 关键技术点总结

### 6.1 Action vs Service 对比

| 特性 | Service | Action |
|------|---------|--------|
| 执行时间 | 短时间 | 长时间 |
| 反馈机制 | 无 | 有实时反馈 |
| 取消支持 | 无 | 支持取消 |
| 适用场景 | 查询、简单操作 | 导航、复杂任务 |

### 6.2 导航控制要点

1. **角度控制优先**: 先调整朝向，再进行移动
2. **距离自适应**: 距离越近，速度越慢
3. **到达判断**: 使用距离阈值判断是否到达
4. **角度归一化**: 将角度差限制在 [-π, π] 范围

### 6.3 多线程考虑

- Action Server 的 `execute_callback` 在独立线程中运行
- 不会阻塞其他 ROS2 回调函数
- 需要注意线程安全和资源共享

## 7. 扩展练习

1. **修改路径形状**: 尝试创建圆形、三角形等不同路径
2. **添加路径优化**: 实现最短路径规划算法
3. **增强导航算法**: 添加障碍物避让功能
4. **参数化配置**: 通过 ROS2 参数动态配置路径和速度
5. **可视化支持**: 在 RViz 中显示规划路径和执行轨迹

## 总结

本节学习了 ROS2 自定义 Action 接口的完整开发流程，从接口定义到服务器实现，再到客户端调用。通过路径导航这个实际案例，深入理解了 Action 机制在复杂任务中的应用，掌握了基本的导航控制算法和实时反馈机制。