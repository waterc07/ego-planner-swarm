cmake_minimum_required(VERSION 3.8)
project(ament_target_dependencies_compat)

find_package(ament_cmake REQUIRED)

ament_package(
  CONFIG_EXTRAS
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ament_target_dependencies_compat-extras.cmake
)
