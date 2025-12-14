from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    declare_model_arg = DeclareLaunchArgument(
        name="model",
        default_value=PathJoinSubstitution([FindPackageShare("my_urdf"), "urdf", "urdf.urdf"])
    )
    declare_gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="False",
        description="Enable joint_state_publisher_gui"
    )

    # 修复后的关节状态发布节点
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable=PythonExpression([
    #         "'joint_state_publisher_gui' if '", LaunchConfiguration("gui"), "' == 'True' else 'joint_state_publisher'"
    #     ]),
    #     name="joint_state_publisher"
    # )


    # 将joint_state_publisher_node改为：
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",  # 直接启动GUI版本
        name="joint_state_publisher_gui"
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("my_urdf"), "urdf.rviz"])]
    )

    return LaunchDescription([declare_model_arg, declare_gui_arg, joint_state_publisher_node, robot_state_publisher_node, rviz_node])
