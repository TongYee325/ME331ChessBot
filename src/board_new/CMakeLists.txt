cmake_minimum_required(VERSION 3.8)
project(board_new)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找构建系统
find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------
# 关键步骤：安装资源文件
# 这会将 launch, meshes, urdf 文件夹复制到 install/share/board 下
# 这样 package://board/meshes/... 路径才能生效
# ---------------------------------------------------------
install(DIRECTORY
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
