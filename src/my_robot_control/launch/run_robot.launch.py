from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 加载 MoveIt 配置 (自动寻找你的 _moveit_config 包)
    # 请把 "my_robot_moveit_config" 替换为你生成的配置包的名字
    moveit_config = MoveItConfigsBuilder("my_urdf",package_name="my_robot_moveit_config").to_moveit_configs()
# 2. 定义你的节点
    run_robot_node = Node(
        package="my_robot_control",
        executable="run_robot",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        run_robot_node
    ])
