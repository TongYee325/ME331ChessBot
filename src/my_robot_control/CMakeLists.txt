cmake_minimum_required(VERSION 3.8)
project(my_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
# ... 现有的 find_package ...
find_package(moveit_ros_planning_interface REQUIRED)

# 添加可执行文件
add_executable(run_robot src/run_robot.cpp)

# 链接库
ament_target_dependencies(run_robot
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# 安装规则 (让 ros2 run 能找到它)
install(TARGETS
  run_robot
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
 DESTINATION share/${PROJECT_NAME}
 )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
