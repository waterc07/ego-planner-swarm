cmake_minimum_required(VERSION 3.8)
project(plan_env)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

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

include_directories( 
    SYSTEM 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

ament_export_include_directories(include)

add_library( plan_env SHARED
    src/grid_map.cpp 
    src/raycast.cpp
    src/obj_predictor.cpp 
    )

ament_target_dependencies(plan_env std_msgs visualization_msgs geometry_msgs nav_msgs rclcpp Eigen3 PCL cv_bridge message_filters pcl_conversions)

target_link_libraries( plan_env 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)  

add_executable(obj_generator
    src/obj_generator.cpp 
)
ament_target_dependencies(obj_generator std_msgs visualization_msgs geometry_msgs nav_msgs rclcpp Eigen3 PCL cv_bridge message_filters pcl_conversions)

install(TARGETS plan_env
        DESTINATION lib
)

install(TARGETS obj_generator
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(plan_env)

# [BB-TEST] Boom_Birds 离线地图行为测试（task-2 D）。
# 默认关闭，不影响正常构建；打开方式：
#   colcon build --cmake-args -DBB_BUILD_GRIDMAP_TESTS=ON
# 补丁前对照二进制（同一份测试源码 + 基线 SHA 的原始源文件）：
#   -DBB_PREPATCH_SRC=/tmp/bb/ego/prepatch_src/src/planner/plan_env
#   -DBB_COMPAT_INCLUDE_DIR=/tmp/bb/ego/compat_include   # Jazzy cv_bridge 头文件名兼容 shim
option(BB_BUILD_GRIDMAP_TESTS "Build Boom_Birds grid_map behavior tests" OFF)
if(BB_BUILD_GRIDMAP_TESTS)
  add_executable(bb_grid_map_test test/bb_grid_map_test.cpp)
  ament_target_dependencies(bb_grid_map_test std_msgs visualization_msgs geometry_msgs
                            nav_msgs rclcpp Eigen3 PCL cv_bridge message_filters pcl_conversions)
  target_link_libraries(bb_grid_map_test plan_env ${PCL_LIBRARIES} ${OpenCV_LIBS})

  if(BB_PREPATCH_SRC)
    add_executable(bb_grid_map_test_prepatch test/bb_grid_map_test.cpp
        ${BB_PREPATCH_SRC}/src/grid_map.cpp
        ${BB_PREPATCH_SRC}/src/raycast.cpp)
    # 基线源码的 #include <cv_bridge/cv_bridge.h> 在 Jazzy 上需要 shim（仅编译脚手架，不改基线逻辑）
    target_include_directories(bb_grid_map_test_prepatch BEFORE PRIVATE
        ${BB_PREPATCH_SRC}/include
        ${BB_COMPAT_INCLUDE_DIR})
    target_compile_definitions(bb_grid_map_test_prepatch PRIVATE BB_PREPATCH=1)
    ament_target_dependencies(bb_grid_map_test_prepatch std_msgs visualization_msgs geometry_msgs
                              nav_msgs rclcpp Eigen3 PCL cv_bridge message_filters pcl_conversions)
    target_link_libraries(bb_grid_map_test_prepatch ${PCL_LIBRARIES} ${OpenCV_LIBS})
  endif()

  install(TARGETS bb_grid_map_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
