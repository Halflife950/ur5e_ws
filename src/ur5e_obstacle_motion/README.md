# ur5e_obstacle_motion

这个包保存 UR5e 的 MoveIt 避障运动示例，用来验证 Planning Scene 中的碰撞物体是否能被正确加入，并让机械臂在规划时绕开障碍物到达目标位姿。

## 内容

```text
ur5e_obstacle_motion
├── launch
│   └── obstacle_motion.launch.py
├── src
│   └── obstacle_motion.cpp
├── CMakeLists.txt
└── package.xml
```

## 功能

`obstacle_motion` 会创建 `MoveGroupInterface` 和 `PlanningSceneInterface`，使用规划组 `ur_manipulator`，先向 MoveIt Planning Scene 中加入一个盒子障碍物，然后规划并执行到一个固定目标位姿。

### 障碍物

碰撞物体 ID：

```text
box_obstacle
```

障碍物类型为 box，尺寸为：

- `x=0.10`
- `y=0.10`
- `z=0.10`

障碍物位姿为：

- 参考坐标系：`move_group.getPlanningFrame()`
- 位置：`x=0.85, y=0.0, z=0.1`
- 姿态：`orientation.w=1.0`

程序会调用 `planning_scene_interface.applyCollisionObject(box)` 将障碍物加入规划场景，并等待 `2s` 后再开始规划。

### 目标位姿

目标位置：

- `x=0.55`
- `y=0.4`
- `z=0.33`

目标姿态由以下欧拉角转换为四元数：

- `roll=0.0`
- `pitch=0.0`
- `yaw=0.0`

### 规划参数

- 规划组：`ur_manipulator`
- 规划时间：`10.0s`
- 规划尝试次数：`10`
- 速度缩放：`0.3`
- 加速度缩放：`0.3`

如果规划成功，程序会执行轨迹；如果规划失败，会在终端输出 `Planning failed`。

## 编译

在 workspace 根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ur5e_obstacle_motion --executor sequential --parallel-workers 1
source install/setup.bash
```

资源紧张时，建议只编当前包，避免一次性编译整个 workspace。

## 运行

使用 launch 启动：

```bash
source install/setup.bash
ros2 launch ur5e_obstacle_motion obstacle_motion.launch.py
```

也可以直接运行节点：

```bash
source install/setup.bash
ros2 run ur5e_obstacle_motion obstacle_motion
```

当前 launch 使用：

- `package="ur5e_obstacle_motion"`
- `executable="obstacle_motion"`
- `use_sim_time=True`
- `/opt/ros/humble/share/ur_moveit_config/config/kinematics.yaml`

## 运行前检查

确认 ROS 能找到包和可执行文件：

```bash
source install/setup.bash
ros2 pkg executables ur5e_obstacle_motion
```

期望看到：

```text
ur5e_obstacle_motion obstacle_motion
```

查看 launch 参数：

```bash
ros2 launch ur5e_obstacle_motion obstacle_motion.launch.py --show-args
```

## 注意

- 运行前需要已有可用的 UR/MoveIt 环境，例如 `ur_moveit_config`。
- 这个包主要验证 Planning Scene 避障规划；基础固定位姿运动在 `ur5e_basic_motion`，笛卡尔路径运动在 `ur5e_cartesian_motion`。
- 当前 launch 使用 `use_sim_time=True`，更适合仿真环境。
- 如果运行真实机械臂，必须先确认障碍物坐标、目标位姿、控制器状态、MoveIt 当前状态和工作空间安全范围。
- 障碍物位置和目标位姿都写在 `src/obstacle_motion.cpp` 中，需要换场景时直接修改对应数值。
