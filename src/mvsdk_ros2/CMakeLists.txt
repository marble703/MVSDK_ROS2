cmake_minimum_required(VERSION 3.16)

project(mvsdk_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

if(CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Debug")
endif()

message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Project path : ${PROJECT_SOURCE_DIR}")
message(STATUS "Binary path : ${PROJECT_BINARY_DIR}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 构建选项, 可选择不构建录制节点
# 后续改为构建工具包内节点的开关
option(BUILD_RECORDER_NODE "Build the image recorder node" ON)

ament_auto_find_build_dependencies()

# 创建主节点
ament_auto_add_executable(${PROJECT_NAME}_node
  src/mvsdk_ros2_node.cpp
)

# 查找 MindVision 驱动
# 理论上删掉这段也可以编译
find_path(MVSDK_INCLUDE_DIR
  NAMES CameraApi.h
  HINTS /usr/include/mindvision /usr/local/include/mindvision /usr/local/include /usr/include
)
message(STATUS "Found MVSDK include dir: ${MVSDK_INCLUDE_DIR}")

# 设置主节点头文件包含目录
target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${MVSDK_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/../mvsdk_cpp/include
)

# 链接主节点库文件
target_link_libraries(${PROJECT_NAME}_node
  ${CMAKE_BINARY_DIR}/../mvsdk_cpp/libmv.a
  MVSDK
  ${OpenCV_LIBS}
)

# 设置主节点的 ROS2 依赖
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# --- 录制节点 ---

# 创建图像录制节点可执行文件
ament_auto_add_executable(${PROJECT_NAME}_recorder_node
  src/image_recorder_node.cpp
)

# 设置录制节点头文件包含目录
target_include_directories(${PROJECT_NAME}_recorder_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${MVSDK_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/../mvsdk_cpp/include
)

# 链接录制节点库文件
target_link_libraries(${PROJECT_NAME}_recorder_node
  ${OpenCV_LIBS}
)

# 设置录制节点的 ROS2 依赖
ament_target_dependencies(${PROJECT_NAME}_recorder_node
  rclcpp
  sensor_msgs
  std_srvs
  cv_bridge
)

# 安装配置文件和启动文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 自动打包
ament_auto_package()