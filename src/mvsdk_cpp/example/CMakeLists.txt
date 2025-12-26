cmake_minimum_required(VERSION 3.16)
project(mvsdk_examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

# 如果是从主工程通过 add_subdirectory 进来的，这些变量已经存在，直接用即可。
# 如果是独立编译，要再 find 一遍。
if(NOT TARGET mv)
  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)

  find_path(MVSDK_INCLUDE_DIR CameraApi.h
            HINTS /usr/include/mindvision /usr/local/include/mindvision)
  find_library(MVSDK_LIBRARY NAMES MVSDK
               HINTS /usr/lib /usr/local/lib)

  # 引入主工程生成的静态库
  find_library(MV_LIB mv
               PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../build"
               NO_DEFAULT_PATH)
  if(NOT MV_LIB)
    message(FATAL_ERROR "libmv.a not found. "
            "Please build the main project first or set MV_LIB.")
  endif()

  set(MV_LINK_LIBS ${MV_LIB} ${OpenCV_LIBS} ${MVSDK_LIBRARY})
  set(MV_INCLUDE_DIRS
      "${CMAKE_CURRENT_SOURCE_DIR}/../include"
      ${OpenCV_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${MVSDK_INCLUDE_DIR})
else()
  set(MV_LINK_LIBS mv)
  set(MV_INCLUDE_DIRS
      $<TARGET_PROPERTY:mv,INTERFACE_INCLUDE_DIRECTORIES>)
endif()

# 生成示例可执行文件
function(add_example name src)
  add_executable(${name} ${src})
  target_include_directories(${name} PRIVATE ${MV_INCLUDE_DIRS})
  target_link_libraries(${name} PRIVATE ${MV_LINK_LIBS})
  set_target_properties(${name} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
endfunction()

# 单相机采集测试
add_example(one_camera src/one_camera.cpp)
