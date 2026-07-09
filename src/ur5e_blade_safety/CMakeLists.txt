cmake_minimum_required(VERSION 3.8)
project(ur5e_blade_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(blade_contour_safety src/blade_contour_safety.cpp)
ament_target_dependencies(
  blade_contour_safety
  ament_index_cpp
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_executable(opening_alignment_planner src/opening_alignment_planner.cpp)
ament_target_dependencies(
  opening_alignment_planner
  ament_index_cpp
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  visualization_msgs
)

add_executable(opening_insert_executor src/opening_insert_executor.cpp)
ament_target_dependencies(
  opening_insert_executor
  ament_index_cpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

add_executable(blade_motion_safety_interlock src/blade_motion_safety_interlock.cpp)
ament_target_dependencies(
  blade_motion_safety_interlock
  control_msgs
  rclcpp
  rclcpp_action
  std_msgs
)

add_executable(cartesian_forward_probe src/cartesian_forward_probe.cpp)
ament_target_dependencies(
  cartesian_forward_probe
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  tf2
  tf2_ros
)

install(TARGETS
  blade_contour_safety
  blade_motion_safety_interlock
  cartesian_forward_probe
  opening_alignment_planner
  opening_insert_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch data rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
