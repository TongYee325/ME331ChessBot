cmake_minimum_required(VERSION 3.0.2)
project(board_new)

# 寻找 catkin 构建系统
find_package(catkin REQUIRED)

# 声明 catkin 包
catkin_package()

# ---------------------------------------------------------
# 安装资源文件
# ---------------------------------------------------------
install(DIRECTORY
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)
