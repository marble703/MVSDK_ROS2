cmake_minimum_required(VERSION 3.16)
project(mvsdk_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 查找相机 SDK 头和库
find_path(MVSDK_INCLUDE_DIR
  NAMES CameraApi.h
  HINTS /usr/include/mindvision /usr/local/include/mindvision /usr/local/include /usr/include
)
find_library(MVSDK_LIBRARY
  NAMES MVSDK
  HINTS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

message(STATUS "mvsdk_cpp: Found MVSDK include dir: ${MVSDK_INCLUDE_DIR}")
message(STATUS "mvsdk_cpp: Found MVSDK lib: ${MVSDK_LIBRARY}")

# 库目标：mv
file(GLOB MV_SRC
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
add_library(mv STATIC ${MV_SRC})

target_include_directories(mv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${MVSDK_INCLUDE_DIR}
)

target_link_libraries(mv PUBLIC
  ${OpenCV_LIBS}
  ${MVSDK_LIBRARY}
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mv ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)

# 如果存在 example 目录，自动编译
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/example/CMakeLists.txt")
  add_subdirectory(example)
endif()