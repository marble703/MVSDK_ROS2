cmake_minimum_required(VERSION 3.16)
project(mv)

set(CMAKE_CXX_STANDARD 20)
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

option(BUILD_RECORDER_NODE "Build the image recorder node" ON)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  src/mv/src/${PROJECT_NAME}_node.cpp
  src/mv/src/camera.cpp
)

# 找 MV 驱动
# 理论上删掉这段也可以编译
# 但是我重装了之后会编译错误，所以保留
find_path(MVSDK_INCLUDE_DIR
  NAMES CameraApi.h
  HINTS /usr/include/mindvision /usr/local/include/mindvision /usr/local/include /usr/include
)
message(STATUS "Found MVSDK include dir: ${MVSDK_INCLUDE_DIR}")

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${MVSDK_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/src/mv/include
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
  MVSDK
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)


# 添加图像录制节点
ament_auto_add_executable(${PROJECT_NAME}_recorder_node
  src/mv/src/image_recorder_node.cpp
)

target_include_directories(${PROJECT_NAME}_recorder_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${MVSDK_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/src/mv/include
)

target_link_libraries(${PROJECT_NAME}_recorder_node
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}_recorder_node
  rclcpp
  sensor_msgs
  std_srvs
  cv_bridge
)


# 安装配置和启动文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()