cmake_minimum_required(VERSION 3.8)
project(ur5e_motion_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(move_joints src/move_joints.cpp)

ament_target_dependencies(move_joints
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)

install(TARGETS
  move_joints
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()