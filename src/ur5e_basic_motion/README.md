# ur5e_basic_motion

这个包只保留 UR5e 的基础 MoveIt 运动示例，用来验证 MoveIt、机器人模型、规划组和基础位姿规划是否正常。

## 内容

```text
ur5e_basic_motion
├── launch
│   └── basic_motion.launch.py
├── src
│   └── basic_motion.cpp
├── CMakeLists.txt
└── package.xml
```

## 功能

`basic_motion` 会创建 `MoveGroupInterface`，使用规划组 `ur_manipulator`，然后规划并执行到一个固定目标位姿：

- 位置：`x=0.35, y=0.4, z=0.6`
- 姿态：由 `roll=1.57, pitch=0.0, yaw=0.0` 转为四元数
- 速度缩放：`0.2`
- 加速度缩放：`0.2`
- 规划时间：`10.0s`

这个包适合做最小验证：如果它能正常规划和执行，说明基础 MoveIt 链路大体是通的。

## 编译

在 workspace 根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ur5e_basic_motion --executor sequential --parallel-workers 1
source install/setup.bash
```

如果机器资源紧张，可以只编这个包，不要一次性编整个 workspace。

## 运行

```bash
source install/setup.bash
ros2 launch ur5e_basic_motion basic_motion.launch.py
```

当前 launch 使用：

- `package="ur5e_basic_motion"`
- `executable="basic_motion"`
- `use_sim_time=True`
- `/opt/ros/humble/share/ur_moveit_config/config/kinematics.yaml`

## 注意

- 运行前需要已有可用的 UR/MoveIt 环境，例如 `ur_moveit_config`。
- 这个包不再包含 cartesian 相关代码；cartesian 轨迹已经拆到 `ur5e_cartesian_motion`。
- 如果运行真实机械臂，先确认控制器、MoveIt 配置和安全空间，别直接用仿真参数去带真机。
