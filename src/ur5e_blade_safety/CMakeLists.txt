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
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(contour_model STATIC src/contour_model.cpp)
target_include_directories(
  contour_model
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(contour_model ament_index_cpp)

add_executable(blade_contour_safety src/blade_contour_safety.cpp)
target_link_libraries(blade_contour_safety contour_model)
ament_target_dependencies(
  blade_contour_safety
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_executable(opening_alignment_planner src/opening_alignment_planner.cpp)
target_link_libraries(opening_alignment_planner contour_model)
ament_target_dependencies(
  opening_alignment_planner
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  visualization_msgs
)

add_executable(opening_alignment_executor src/opening_alignment_executor.cpp)
target_link_libraries(
  opening_alignment_executor
  control_msgs::control_msgs__rosidl_typesupport_cpp
)
target_include_directories(
  opening_alignment_executor
  PRIVATE
    /opt/ros/$ENV{ROS_DISTRO}/include/control_msgs
    /opt/ros/$ENV{ROS_DISTRO}/include/sensor_msgs
    /opt/ros/$ENV{ROS_DISTRO}/include/trajectory_msgs
)
ament_target_dependencies(
  opening_alignment_executor
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_ros
  trajectory_msgs
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
  contour_model
  opening_alignment_executor
  opening_alignment_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch data rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
