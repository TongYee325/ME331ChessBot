# 机器人建模与控制课程项目
## 一个五轴机械臂执行下棋仿真（ROS Noetic 工作流）

### Quick Start（ROS Noetic + UR5 + Gazebo）

以下流程适用于本仓库下的 `ros_noetic_ur5` 工作空间（请根据你的实际路径调整）。

1. **进入 Noetic 工作空间并编译**
```bash
cd <path-to-repo>/ros_noetic_ur5
catkin_make
source devel/setup.bash
```

2. **打开 3 个终端，并在每个终端都 source 环境**
> 每个新终端都需要重新 `source` 一次，否则可能找不到 ROS package / node。
```bash
cd <path-to-repo>/ros_noetic_ur5
source devel/setup.bash
```

3. **终端 1：启动 Gazebo + MoveIt Demo**
```bash
roslaunch ur5_moveit_config demo_gazebo.launch
```
等待 Gazebo 中棋子全部生成完成后再继续（通常需要数秒）。

4. **终端 2：发布棋子位姿信息（注意脚本路径）**
```bash
python3 <path-to-repo>/ros_noetic_ur5/src/board_new/src/simple_manager.py
```
该脚本用于持续发布棋子位姿信息（供运动规划/抓取节点读取）。

> 若已在工作空间根目录（`ros_noetic_ur5`）下，也可使用相对路径：
```bash
python3 ./src/board_new/src/simple_manager.py
```

5. **终端 3：启动抓取并输入要移动的棋子名称**
```bash
rosrun motion_planner move_given_pieces
```
等待出现提示：
```text
Enter chess piece name to PICK (e.g., pion1):
```
输入想要移动的棋子名称（例如：`benteng4`），观察 Gazebo 中机械臂执行**夹取并放置**动作。

---

### 调参建议（夹取失败 / 末端与棋子“黏连”）

若出现以下现象：
- 夹爪闭合但抓不住棋子（频繁掉落/穿模）
- 末端与棋子“黏连”，释放不干净
- 抓取后移动时棋子抖动明显

可尝试调整 `grasp-fix` 插件参数（Gazebo 抓取稳定性相关）：

文件路径：
```
ME331ChessBot/ros_noetic_ur5/src/ur5_robot-master/ur5_moveit_config/config/gazebo_ur5.xacro
```
建议关注 **第 899–918 行**附近的 grasp-fix 插件配置（show/hide 行号以便定位）。

常见参数含义（不同版本命名可能略有差异，以文件内实际字段为准）：

| 参数（示例名） | 作用 | 调整建议 |
|---|---|---|
| `update_rate` | 插件更新频率（Hz），影响“抓稳/释放”的响应速度 | 抓取不稳定可适当提高；过高可能增大计算负担 |
| `grip_count_threshold` / `max_grip_count` | 判定“已经抓稳”的连续接触计数阈值 | 棋子容易掉：适当增大；黏连严重：适当减小或配合 release 参数 |
| `release_tolerance` | 判定“允许释放”的阈值/容差（通常与接触/相对速度等相关） | 释放不干净：适当增大；过大可能导致提前释放 |
| `disable_collisions_on_attach` / `disable_collisions` | 抓取后是否临时禁用夹爪与物体的碰撞（减少抖动/卡住） | 黏连/抖动可尝试开启；但可能降低真实碰撞效果 |
| `contact_links` / `gripper_link` / `palm_link` | 指定用于接触判定/附着的末端链接集合 | 确保配置与 URDF/MoveIt 中末端 link 命名一致，否则会“抓不住” |

---

### 注意事项（放置逻辑目前为简化版）

当前棋子**放置位置逻辑未做完整设计**，仅做了简单处理（例如将夹取点在 **y 方向 +0.2** 后放置）。  
后续可在以下文件中修改/优化放置策略（例如：根据棋盘格坐标、避障、落子规则等计算目标位姿）：
- `move_given_pieces.cpp`

---

### Demo 视频

- 视频位置：`ME331ChessBot/videos/夹取棋子.webm`
- 在 README 中可直接引用（相对路径）：
    ![DemoVideo](videos/demo.mp4)