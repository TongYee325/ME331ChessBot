import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # 获取包路径
    pkg_share = FindPackageShare('board_new')
    gazebo_ros_share = FindPackageShare('gazebo_ros')
    
    # URDF 文件路径
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'board_new.urdf'])

    # 设置环境变量 (如果你在虚拟机，保留这个；如果是物理机有显卡，建议注释掉)
    # env = os.environ.copy()
    # env['LIBGL_ALWAYS_SOFTWARE'] = '1' 
    # 注意：如果必须使用 env，请在 ExecuteProcess 中传入 env=env
    
    # 1. 启动 Gazebo 服务器
    # 使用 IncludeLaunchDescription 是更标准的做法，因为它会自动加载必要的 Gazebo 路径
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={'verbose': 'true', 'world': PathJoinSubstitution([gazebo_ros_share, 'worlds', 'empty.world'])}.items()
    )
    
    # 2. 启动 Gazebo 客户端
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzclient.launch.py'])
        )
    )
    
    # 3. Robot State Publisher (关键修复)
    # 这将读取 URDF 并发布 /robot_description 和 TF 树
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file]
    )
    
    # 4. 静态坐标变换 (修复参数格式)
    # 格式: x y z qx qy qz qw parent child
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'base_footprint'],
        output='screen'
    )
    
    # 5. 生成模型
    # 建议使用 -topic robot_description，这样可以确保 RSP 已经处理完 URDF
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-entity', 'board_new',
            '-topic', 'robot_description', # 从 robot_state_publisher 获取模型描述
            '-x', '0', '-y', '0', '-z', '1.5',
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )
    
    # 使用 Timer 稍微延迟 spawn，确保 Gazebo 服务已就绪
    spawn_entity_delayed = TimerAction(
        period=3.0,
        actions=[spawn_entity]
    )
    
    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher, # 必须添加
        static_transform,
        spawn_entity_delayed,
    ])