# cmake_minimum_required(VERSION 2.8.3)

# project(my_urdf)

# find_package(catkin REQUIRED)

# catkin_package()

# find_package(roslaunch)

# find_package(ament_cmake REQUIRED)

# install(DIRECTORY launch urdf meshes
#   DESTINATION share/${PROJECT_NAME}) 

# ament_package()

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)




# cmake_minimum_required(VERSION 3.8)
# project(robot)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()




# # 最低 CMake 版本（ROS 2 Humble 要求 ≥3.16，其他版本 ≥3.8 即可）
# cmake_minimum_required(VERSION 3.16)
# project(my_urdf)

# # 编译选项（默认，保留）
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # -------------------------- 关键：查找依赖包 --------------------------
# # 必须包含 launch 和 launch_ros（两者都需要，顺序无关）
# find_package(ament_cmake REQUIRED)
#     # 依赖 launch 基础包
# find_package(launch_ros REQUIRED)   # launch_ros 包（用于 ROS 2 节点启动）
# # 其他你需要的依赖（根据你的包补充，如 urdf、gazebo_ros 等）
# find_package(urdf REQUIRED)
# find_package(gazebo_ros REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(std_msgs REQUIRED)

# # -------------------------- 安装 launch/urdf 文件 --------------------------
# # 安装 launch 目录（确保 launch 文件被正确安装到 install 目录）
# install(DIRECTORY
#   launch  # 你的 launch 文件所在目录（若没有则创建：mkdir -p src/my_urdf/launch）
#   urdf    # 你的 URDF 文件所在目录
#   DESTINATION share/${PROJECT_NAME}/  # 安装路径：install/share/my_urdf/
# )

# # -------------------------- 测试配置（默认，保留） --------------------------
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# # -------------------------- 必须在最后：声明包类型 --------------------------
# ament_package()




# 最低 CMake 版本（ROS 2 Humble 要求 ≥3.16，Iron/Jazzy 建议 ≥3.20，兼容所有版本）
cmake_minimum_required(VERSION 3.16)
project(my_urdf)

# 编译选项（开启警告，提升代码质量，保留）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------- 查找依赖包（精简+必要） --------------------------
# ament_cmake：ROS 2 基础构建工具（必须）
# launch_ros：ROS 2 launch 功能核心依赖（解决之前的报错）
# urdf：解析 URDF 文件的依赖（必须）
# 其他依赖根据实际使用情况保留/删除（不需要则注释掉，减少编译依赖）
find_package(ament_cmake REQUIRED)

find_package(urdf REQUIRED)

# 可选依赖（根据你的实际需求决定是否保留）
# find_package(gazebo_ros REQUIRED)  # 若用 Gazebo 仿真则保留
# find_package(tf2_ros REQUIRED)     # 若用 TF2 坐标变换则保留
# find_package(std_msgs REQUIRED)     # 若用到标准消息则保留

# -------------------------- 安装文件（简洁+覆盖所有资源目录） --------------------------
# 安装 launch/urdf/meshes 等目录（若目录不存在，编译时会警告但不报错，建议提前创建）
install(DIRECTORY
  launch   # 存放 .launch.py 文件
  urdf     # 存放 .urdf/.xacro 文件
  meshes   # 存放模型网格文件（如 .stl/.dae，若无此目录可删除该行）
  config   # 存放配置文件（如 RViz 配置，若无此目录可删除该行）
  DESTINATION share/${PROJECT_NAME}/  # 安装路径：install/share/my_urdf/
)

# -------------------------- 测试配置（默认保留，可选关闭） --------------------------
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # 跳过版权检查（若添加了版权声明，可删除这行）
#   set(ament_cmake_copyright_FOUND TRUE)
#   # 跳过 cpplint 检查（若需要代码风格检查，可删除这行）
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# -------------------------- 声明 ROS 2 包（必须在最后） --------------------------
ament_package()
