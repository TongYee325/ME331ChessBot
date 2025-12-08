import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 1. 获取包路径和 SDF 绝对路径（转换后的 SDF 文件）
    my_urdf_dir = get_package_share_directory('my_urdf')
    sdf_filename = "robot.sdf"  # 转换后的 SDF 文件名
    sdf_abs_path = os.path.abspath(os.path.join(my_urdf_dir, 'urdf', sdf_filename))

    # 2. 验证 SDF 文件存在
    if not os.path.exists(sdf_abs_path):
        raise FileNotFoundError(
            f"\nSDF 文件不存在！请先执行 URDF 转 SDF：\n"
            f"cd {os.path.join(my_urdf_dir, 'urdf')}\n"
            f"gz sdf -p urdf.urdf > robot.sdf\n"
        )
    print(f"\n✅ 找到 SDF 文件（绝对路径）：\n{sdf_abs_path}\n")

    # 3. 启动 gz sim（直接加载 SDF 机器人，不加载 URDF）
    start_gazebo = ExecuteProcess(
        # 核心命令：加载内置空世界 + SDF 机器人
        cmd=[
            f'gz sim empty.sdf {sdf_abs_path} --gui --verbose'
        ],
        output='screen',
        shell=True,
        log_cmd=True
    )

    # 4. 关节状态发布（带 GUI 控制面板，和 SDF 关节名称匹配）
    joint_state_pub = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[
            {'use_sim_time': True},  # 同步 Gazebo 仿真时间
            {'joints': [  # 明确指定 SDF 中的可动关节（从你的 URDF 提取）
                'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 
                'joint6_left', 'joint7_left', 'joint8_left',
                'joint9_right', 'joint10_right', 'joint11_right'
            ]}
        ]
    )

    # 5. 机器人状态发布（解析 SDF 生成 TF 变换）
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': open(sdf_abs_path, 'r').read()}  # 直接读取 SDF 内容
        ]
    )

    return LaunchDescription([
        start_gazebo,
        joint_state_pub,
        robot_state_pub
    ])
