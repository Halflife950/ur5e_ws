#!/usr/bin/env bash
set -e

cd /home/tinavi/ur5e_ws
source install/setup.bash

export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1

exec ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py \
  description_package:=ur5e_custom_tool_description \
  description_file:=ur_with_screwdriver.urdf.xacro
