# 机器人建模与控制课程项目
## 一个五轴机械臂执行下棋仿真

## Quick Start
在项目源目录下编译、进入工作空间并启动仿真：

1. 在项目源目录编译
```
# 进入工作区根目录（包含 `src` 的目录）
cd /path/to/your/workspace
# 若为 catkin 工具链（ROS1）
catkin_make
# 或 若为 colcon（ROS2 / colcon 工作区）
colcon build
```

2. 进入工作空间（source 环境）
```
# 在 Bash（Linux / WSL）中：
source devel/setup.bash
```

3. 启动仿真
```
roslaunch robot_arm display.launch
```

备注：
- 请根据你使用的 ROS 版本（ROS1/ROS2）选择输入指令。