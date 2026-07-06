# ur5e_cartesian_motion

这个包保存 UR5e 的笛卡尔空间运动示例，已经从 `ur5e_basic_motion` 中拆分出来。它主要用于测试直线、方形、圆形等笛卡尔路径规划和循环绘制。

## 内容

```text
ur5e_cartesian_motion
├── launch
│   ├── cartesian_draw.launch.py
│   ├── cartesian_draw_loop.launch.py
│   └── cartesian_draw_loop_real.launch.py
├── src
│   ├── cartesian_draw.cpp
│   └── cartesian_draw_loop.cpp
├── CMakeLists.txt
└── package.xml
```

## 可执行文件

### `cartesian_draw`

执行一次笛卡尔路径绘制。

启动文件：

```bash
ros2 launch ur5e_cartesian_motion cartesian_draw.launch.py
```

### `cartesian_draw_loop`

循环执行笛卡尔路径绘制。

仿真启动文件：

```bash
ros2 launch ur5e_cartesian_motion cartesian_draw_loop.launch.py
```

真实机器人相关启动文件：

```bash
ros2 launch ur5e_cartesian_motion cartesian_draw_loop_real.launch.py
```

`cartesian_draw_loop_real.launch.py` 会构造 `robot_description`、`robot_description_semantic` 和 kinematics 参数，默认：

- `ur_type=ur5e`
- `prefix=""`
- `use_sim_time=False`

里面的 `robot_ip:=xxx.yyy.zzz.www` 仍是占位值，真机使用前需要改成实际机器人 IP。

## 运动模式

这些 launch 都支持 `mode` 参数：

- `line`
- `square`
- `circle`

示例：

```bash
ros2 launch ur5e_cartesian_motion cartesian_draw_loop.launch.py mode:=circle
```

## 编译

在 workspace 根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ur5e_cartesian_motion --executor sequential --parallel-workers 1
source install/setup.bash
```

资源紧张时，建议只编当前包或只编需要调试的包，避免全 workspace 编译。

## 运行前检查

确认 ROS 能找到包和可执行文件：

```bash
source install/setup.bash
ros2 pkg executables ur5e_cartesian_motion
```

期望至少看到：

```text
ur5e_cartesian_motion cartesian_draw
ur5e_cartesian_motion cartesian_draw_loop
```

查看 launch 参数：

```bash
ros2 launch ur5e_cartesian_motion cartesian_draw_loop.launch.py --show-args
```

## 注意

- 仿真 launch 使用 `use_sim_time=True`。
- 真机 launch 使用 `use_sim_time=False`。
- 真机运行前必须确认机器人 IP、控制器状态、MoveIt 当前状态和工作空间安全范围。
- 这个包只放 cartesian 相关内容；基础 MoveIt 固定位姿运动在 `ur5e_basic_motion`。
