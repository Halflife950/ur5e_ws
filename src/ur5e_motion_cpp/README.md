# ur5e_motion_cpp

这个包保存一个最小 ROS 2 C++ 关节轨迹 action 示例，用来验证 UR5e Gazebo 仿真中的轨迹控制器是否可以接收并执行关节目标。

它不使用 MoveIt 做运动规划，而是直接向控制器 action 发送 `control_msgs/action/FollowJointTrajectory` 目标。

## 内容

```text
ur5e_motion_cpp
├── src
│   └── move_joints.cpp
├── CMakeLists.txt
└── package.xml
```

## 功能

`move_joints` 会创建 action client，并连接：

```text
/joint_trajectory_controller/follow_joint_trajectory
```

随后发送 6 个关节的目标角度：

```text
shoulder_pan_joint:   0.0
shoulder_lift_joint: -1.57
elbow_joint:          1.57
wrist_1_joint:       -1.57
wrist_2_joint:       -1.57
wrist_3_joint:        0.0
```

轨迹时间为 `4.0s`。

## 编译

在 workspace 根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ur5e_motion_cpp --executor sequential --parallel-workers 1
source install/setup.bash
```

## 运行

先启动 UR5e Gazebo 与 MoveIt 联动仿真：

```bash
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true
```

在另一个终端运行：

```bash
source ~/ur5e_ws/install/setup.bash
ros2 run ur5e_motion_cpp move_joints
```

## 运行前检查

确认控制器 active：

```bash
ros2 control list_controllers
```

确认 action server 存在：

```bash
ros2 action list | grep follow_joint_trajectory
```

确认可执行文件已安装：

```bash
ros2 pkg executables ur5e_motion_cpp
```

期望看到：

```text
ur5e_motion_cpp move_joints
```

## 注意

- 这个包直接发送关节轨迹，不会做碰撞检测、逆运动学或路径可达性检查。
- 它适合验证 `ros2_control` action 接口，不适合作为复杂任务规划示例。
- 当前说明只面向 Gazebo 仿真环境。
