cmake_minimum_required(VERSION 3.0.2)
project(motion_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  tf2_ros            # 新增
  tf2_geometry_msgs  # 新增
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_planner
#  CATKIN_DEPENDS roscpp moveit_ros_planning_interface geometry_msgs tf2_ros tf2_geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motion_planner_node src/motion_planner_node.cpp)
target_link_libraries(motion_planner_node ${catkin_LIBRARIES})

add_executable(custom_position_node src/custom_position_node.cpp)
target_link_libraries(custom_position_node ${catkin_LIBRARIES})

add_executable(move_given_pieces src/move_given_pieces.cpp)
target_link_libraries(move_given_pieces ${catkin_LIBRARIES})