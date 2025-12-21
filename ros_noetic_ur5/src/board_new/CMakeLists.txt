cmake_minimum_required(VERSION 3.0.2)
project(board_new)

add_compile_options(-std=c++14)

# 寻找 catkin 构建系统
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  gazebo_msgs
  geometry_msgs
  tf2
  tf2_ros
  # 添加以下包
  moveit_ros_planning_interface
  moveit_msgs
  geometric_shapes
)

# 声明 catkin 包
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(spawn_chess_pieces src/spawn_chess_pieces.cpp)
target_link_libraries(spawn_chess_pieces ${catkin_LIBRARIES})

# 添加 pieces_manager
add_executable(pieces_manager src/pieces_manager.cpp)
target_link_libraries(pieces_manager ${catkin_LIBRARIES})

# ---------------------------------------------------------
# 安装资源文件
# ---------------------------------------------------------
install(DIRECTORY
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

