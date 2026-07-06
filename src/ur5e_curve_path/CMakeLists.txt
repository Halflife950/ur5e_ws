cmake_minimum_required(VERSION 3.8)
project(ur5e_curve_path)

if(CMAKE_COMPILER_IS_GNUCXX OR
   CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(trajectory_msgs REQUIRED)

add_executable(curve_path_node
  src/curve_path_main.cpp
  src/config_utils.cpp
  src/curve_path_node.cpp
  src/ik_utils.cpp
  src/orientation_utils.cpp
  src/path_utils.cpp
  src/trajectory_utils.cpp
)

target_include_directories(curve_path_node
  PRIVATE
    include
)

ament_target_dependencies(curve_path_node
  ament_index_cpp
  rclcpp

  geometry_msgs
  visualization_msgs

  moveit_core
  moveit_msgs
  moveit_ros_planning_interface

  tf2
  tf2_geometry_msgs

  trajectory_msgs
)

install(
  TARGETS curve_path_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
    paths
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
