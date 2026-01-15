# Lesson 4: TurtleBot3 Humble 仿真环境安装

本教程将指导你在 ROS 2 Humble 环境下安装 TurtleBot3 仿真环境。

## 前置条件

- Ubuntu 22.04
- ROS 2 Humble 已安装
- Gazebo 已安装（ROS 2 Humble 默认使用 Gazebo Classic 11）

## 1. 安装依赖包

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

## 2. 安装 TurtleBot3 相关包

```bash
# 安装所有 TurtleBot3 包（包含核心包和仿真包）
sudo apt install -y ros-humble-turtlebot3*
```

> 注：使用通配符 `*` 会一次性安装所有 TurtleBot3 相关包，包括 `turtlebot3-gazebo` 仿真包，无需单独安装。

## 3. 设置 TurtleBot3 模型环境变量

TurtleBot3 有三种模型可选：`burger`、`waffle`、`waffle_pi`

将以下内容添加到 `~/.bashrc`：

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

或者每次启动前手动设置：

```bash
export TURTLEBOT3_MODEL=burger
```

## 4. 启动仿真环境

### 4.1 启动 Gazebo 仿真世界

打开一个终端：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 4.2 可用的仿真世界

TurtleBot3 提供多个预设仿真世界：

```bash
# 空白世界
ros2 launch turtlebot3_gazebo empty_world.launch.py

# TurtleBot3 世界（带障碍物）
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# TurtleBot3 房屋环境
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

## 5. 控制 TurtleBot3

### 5.1 键盘遥控

打开新终端：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

控制按键：
- `w` - 前进
- `x` - 后退
- `a` - 左转
- `d` - 右转
- `s` - 停止

### 5.2 使用 RViz2 可视化

打开新终端：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup rviz2.launch.py
```

## 6. SLAM 建图（可选）

### 6.1 启动 SLAM

确保 Gazebo 仿真已启动，然后打开新终端：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 6.2 保存地图

建图完成后，保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## 7. 导航（可选）

使用已保存的地图进行导航：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

## 常见问题

### Q1: Gazebo 启动后黑屏或卡住

尝试：
```bash
# 清理 Gazebo 缓存
rm -rf ~/.gazebo/models
```

### Q2: 找不到 TurtleBot3 模型

确保设置了环境变量：
```bash
echo $TURTLEBOT3_MODEL
# 应该输出: burger, waffle, 或 waffle_pi
```

### Q3: 仿真运行缓慢

- 降低 Gazebo 的实时因子
- 关闭不必要的可视化
- 使用更简单的仿真世界（如 empty_world）

## 参考链接

- [TurtleBot3 官方文档](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [Gazebo 文档](https://gazebosim.org/docs)
