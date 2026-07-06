# ur5e_curve_path

`ur5e_curve_path` 是一个用于 UR5e 的笛卡尔曲线路径跟踪示例包。节点会读取 CSV 路径点，将路径贴到当前 `tool0` 起点附近，做路径连续性检查、末端朝下姿态检查、释放 yaw 的连续 IK 检查，然后生成带时间戳的关节轨迹并循环执行。

当前默认路径是 `paths/heart_path.csv`，用于绘制一个闭合的心形曲线。

## 功能概览

- 从 CSV 读取毫米单位的路径点。
- 检查相邻路径点和首尾闭合距离。
- 将 CSV 坐标轴映射到机器人 `base_link` 坐标系。
- 移动到预设起始关节位姿。
- 使用 MoveIt 对每个路径点做释放 yaw 的连续 IK。
- 生成三角形或梯形笛卡尔速度曲线。
- 自动拉长轨迹时间以满足关节速度和关节加速度上限。
- 发布 RViz marker：默认 topic 为 `/heart_path_marker`。
- 循环执行生成的轨迹。

## 文件结构

```text
ur5e_curve_path/
  config/
    kinematics.yaml              # MoveIt IK 配置
  include/ur5e_curve_path/
    config_utils.hpp            # 节点参数配置读取和校验
    curve_path_node.hpp          # 节点类声明
    ik_utils.hpp                 # 释放 yaw 的 IK 搜索工具
    orientation_utils.hpp        # 姿态计算工具
    path_utils.hpp               # CSV 和路径检查工具
    trajectory_utils.hpp         # 时间和轨迹数学工具
  launch/
    curve_path_sim.launch.py     # 仿真参数版本
    curve_path_real.launch.py    # 真机参数版本
  paths/
    heart_path.csv               # 默认心形路径
  src/
    config_utils.cpp            # 参数读取、类型兼容、范围校验
    curve_path_main.cpp          # main 入口
    curve_path_node.cpp          # ROS2/MoveIt 节点主体
    ik_utils.cpp                 # yaw 采样、IK 候选选择
    orientation_utils.cpp
    path_utils.cpp
    trajectory_utils.cpp
```

## 代码职责划分

当前代码已经按职责拆分，后续改功能时建议优先找对应模块：

| 文件 | 主要职责 |
|---|---|
| `curve_path_node.cpp` | ROS2 参数、MoveIt 初始化、起始位姿运动、主流程编排、日志和轨迹执行 |
| `config_utils.cpp` | 读取 ROS2 参数、处理 YAML 字符串/布尔歧义、校验参数范围 |
| `path_utils.cpp` | CSV 读取、数值解析、路径连续性检查、CSV 轴映射、路径尺寸和点距计算 |
| `ik_utils.cpp` | 围绕工具 Z 轴采样 yaw，调用 MoveIt IK，并选择最平滑的关节解 |
| `orientation_utils.cpp` | 检查末端工具 Z 轴是否足够朝下 |
| `trajectory_utils.cpp` | 时间戳转换、笛卡尔速度曲线、关节轨迹生成、关节速度/加速度统计、轨迹时间缩放 |

整体主流程在 `CurvePathNode::initialize()` 中串起来：

```text
读取并检查 CSV
  -> 初始化 MoveGroup
  -> 移动到预设起始关节位姿
  -> 记录当前 tool0 位姿作为路径原点
  -> 验证末端 Z 轴朝下
  -> 对路径点做释放 yaw 的连续 IK
  -> 生成带时间戳的关节轨迹
  -> 必要时自动拉长轨迹时间
  -> 最终速度/加速度校验
  -> 循环执行轨迹
```

## 编译

在工作空间根目录执行：

```bash
cd ~/ur5e_ws
colcon build --packages-select ur5e_curve_path
source install/setup.bash
```

如果电脑性能有限，建议保守编译：

```bash
cd ~/ur5e_ws
env CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build --packages-select ur5e_curve_path \
  --parallel-workers 1 --executor sequential
source install/setup.bash
```

## 仿真运行

先启动 UR5e Gazebo/MoveIt 仿真环境，例如：

```bash
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py \
  ur_type:=ur5e launch_rviz:=true
```

确认 RViz、MoveIt 和控制器正常后，在另一个终端运行：

```bash
cd ~/ur5e_ws
source install/setup.bash
ros2 launch ur5e_curve_path curve_path_sim.launch.py
```

`curve_path_sim.launch.py` 会设置 `use_sim_time:=True`，并加载本包的 `config/kinematics.yaml`。

## 真机运行

真机运行前请先完成 UR 官方驱动、控制器和 MoveIt 环境启动，并确认机器人处于可安全运动状态。

然后运行：

```bash
cd ~/ur5e_ws
source install/setup.bash
ros2 launch ur5e_curve_path curve_path_real.launch.py
```

`curve_path_real.launch.py` 会设置 `use_sim_time:=False`。

真机执行前建议先在仿真里完整跑通，并确认路径范围、起始姿态和速度限制适合当前工作空间。

## CSV 路径格式

默认文件：

```text
paths/heart_path.csv
```

格式如下：

```csv
x_mm,y_mm,z_mm
0.0,0.0,0.0
-1.181875,0.790969,0.0
```

要求：

- 第一行是表头。
- 每行必须包含 `x_mm,y_mm,z_mm` 三列。
- 单位是毫米。
- 路径至少包含 2 个点。
- 当前节点要求路径连续且首尾闭合，默认最大允许间距为 `10.0 mm`。
- CSV 中不能包含 `nan`、`inf`、多余列或非法数字。

CSV 的数值只代表路径形状，不直接代表机器人绝对坐标。第一个 CSV 点会被当作路径原点，对应机器人到达预设起始关节位姿后的当前 `tool0` 位置。后续点只使用相对第一个点的偏移。

## 坐标映射

CSV 文件始终按表头规范读成 `x_mm,y_mm,z_mm`。读入之后，再通过参数把 CSV 轴映射到机器人 `base_link` 轴。默认映射采用直觉上的同名轴对应：

```text
CSV x -> base_link X
CSV y -> base_link Y
base_link Z 固定在起点高度
```

CSV 的第一个点会对应机器人到达预设起始关节位姿后的当前末端位置。执行路径时，末端 Z 轴保持朝下，yaw 会围绕工具 Z 轴采样，并选择与上一个 IK 解最接近的关节解，让机械臂可以更自然地绕 Z 轴转动。

如果以后 CSV 来自 CAD 或草图坐标系，需要交换轴，不需要改源码，只需要改映射参数。例如使用 `CSV y -> base_link X`、`CSV x -> base_link Y`：

```bash
ros2 launch ur5e_curve_path curve_path_sim.launch.py \
  base_x_from_csv_axis:=y \
  base_y_from_csv_axis:=x \
  base_z_from_csv_axis:=fixed
```

运行时日志会同时打印原始 CSV 尺寸和映射后的路径尺寸：

```text
Raw path dimensions: X ... mm, Y ... mm, Z ... mm
Mapped path dimensions: base_link X ... mm, Y ... mm, Z ... mm
```

如果映射后的某个方向尺寸意外为 `0.000 mm`，通常说明 CSV 轴映射和文件内容不匹配。例如 CSV 的 `z_mm` 全是 0，但把 `base_x_from_csv_axis` 设成了 `z`，路径就会在 `base_link X` 方向被压扁。

## 末端姿态和 yaw 释放

节点不是固定完整的 6D 姿态，而是保持工具 Z 轴朝下，同时释放绕工具 Z 轴的 yaw。这样可以避免机械臂为了保持一个不必要的 yaw 角而做别扭的腕部运动。

实现方式在 `ik_utils.cpp`：

1. 以起点处验证过的 `tool0` 姿态作为基准姿态。
2. 对每个路径点，在工具自身 Z 轴周围采样多个 yaw 候选。
3. 对每个候选姿态调用 MoveIt IK。
4. 丢弃 IK 失败或越过关节边界的候选。
5. 在剩余候选中选择和上一个关节解距离最小的解。

`yaw_sample_count` 控制每个点尝试多少个 yaw 候选。默认 `37`，相当于大约每 10 度一个候选。数值越大，IK 有更多机会找到平滑解，但计算会更慢。

## 速度、加速度和自动时间缩放

路径执行速度分两层：

| 参数 | 作用范围 |
|---|---|
| `velocity_scaling` / `acceleration_scaling` | MoveIt 规划到预设起始关节位姿时使用 |
| `cartesian_speed_mps` / `cartesian_acceleration_mps2` | CSV 曲线路径的笛卡尔速度曲线 |
| `joint_velocity_limit_radps` / `joint_acceleration_limit_radps2` | 对最终关节轨迹做安全校验和自动时间缩放 |

CSV 路径的时间戳先按笛卡尔速度生成三角形或梯形速度曲线。之后程序会统计整条关节轨迹的最大关节速度和最大关节加速度。如果超过关节限制，节点会把整条轨迹时间按比例拉长，再重新校验。

这意味着 `cartesian_speed_mps` 很低时，仍然可能出现局部关节加速度偏高，因为 IK 解在关节空间可能有比较急的变化。此时程序会打印类似日志：

```text
Stretching trajectory time by 1.234x to satisfy joint limits.
```

这不是错误，而是节点在保护关节速度/加速度限制。最终如果缩放后仍然超限，才会报错并停止执行。

## 常用参数

这些参数可以通过 launch 文件或命令行覆盖。默认值在 `curve_path_node.hpp` 的 `Config` 结构体中定义。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `move_group_name` | `ur_manipulator` | MoveIt planning group |
| `end_effector_link` | `tool0` | 末端 link |
| `path_csv` | `paths/heart_path.csv` | CSV 路径。相对路径会从包 share 目录解析，绝对路径也可用 |
| `base_x_from_csv_axis` | `x` | 映射到 `base_link X` 的 CSV 轴，可选 `x/y/z` |
| `base_y_from_csv_axis` | `y` | 映射到 `base_link Y` 的 CSV 轴，可选 `x/y/z` |
| `base_z_from_csv_axis` | `fixed` | 映射到 `base_link Z` 的 CSV 轴，可选 `fixed/x/y/z` |
| `marker_topic` | `/heart_path_marker` | RViz marker topic |
| `marker_frame` | `base_link` | marker 坐标系 |
| `planning_time_s` | `10.0` | MoveIt 规划时间 |
| `planning_attempts` | `10` | 规划尝试次数 |
| `velocity_scaling` | `0.15` | MoveIt 起始位姿运动速度比例 |
| `acceleration_scaling` | `0.15` | MoveIt 起始位姿运动加速度比例 |
| `max_gap_mm` | `10.0` | 路径相邻点和首尾闭合最大间距 |
| `max_tilt_error_deg` | `2.0` | 末端 Z 轴朝下允许误差 |
| `ik_timeout_s` | `0.05` | 单个路径点 IK 超时 |
| `yaw_sample_count` | `37` | 每个路径点尝试的 yaw 采样数量。默认约 10 度一个候选 |
| `max_joint_step_rad` | `0.35` | 相邻 IK 解最大关节跳变 |
| `cartesian_speed_mps` | `0.08` | 轨迹笛卡尔速度上限 |
| `cartesian_acceleration_mps2` | `0.16` | 轨迹笛卡尔加速度上限 |
| `joint_velocity_limit_radps` | `2.0` | 轨迹关节速度校验上限 |
| `joint_acceleration_limit_radps2` | `10.0` | 轨迹关节加速度校验和自动时间缩放上限 |
| `loop_pause_ms` | `200` | 每次循环执行后的暂停时间 |

`path_csv` 和坐标映射参数已经声明为 launch argument，可以直接在命令行覆盖：

```bash
ros2 launch ur5e_curve_path curve_path_sim.launch.py \
  path_csv:=paths/my_path.csv
```

也可以传绝对路径：

```bash
ros2 launch ur5e_curve_path curve_path_sim.launch.py \
  path_csv:=/home/tinavi/my_paths/demo.csv
```

轴映射参数使用 `x/y/z/fixed` 这种短字符串。由于 ROS2 launch 参数会经过 YAML 解析，命令行手动传 `y` 时，最稳妥的写法是加一层引号：

```bash
ros2 launch ur5e_curve_path curve_path_sim.launch.py \
  base_y_from_csv_axis:="'y'"
```

代码里也做了兼容处理，即使裸 `y` 被 YAML 当成布尔值，节点也会尝试恢复为轴名。

## RViz 查看路径

节点会发布 `visualization_msgs/msg/Marker`：

```text
/heart_path_marker
```

在 RViz 中添加 `Marker` display，并将 topic 设为 `/heart_path_marker`。默认 frame 是 `base_link`。

## 安全提示

- 真机运行前先仿真验证。
- 保持急停可用，并确认工作空间内没有障碍物。
- 修改 CSV 或速度参数后，先观察日志中的路径尺寸、IK 检查和轨迹校验结果。
- 如果出现 IK 失败、关节跳变、速度或加速度超限，节点会停止并报错，不会继续执行轨迹。

## 常见问题

### 找不到 CSV 文件

确认已经重新编译并 source：

```bash
colcon build --packages-select ur5e_curve_path
source install/setup.bash
```

默认 CSV 会被安装到：

```text
install/ur5e_curve_path/share/ur5e_curve_path/paths/heart_path.csv
```

### IK 在某个路径点失败

可能原因：

- 起始关节位姿让路径落在不可达区域。
- 路径尺寸太大。
- 末端姿态约束过紧。
- IK 插件或 timeout 设置不适合当前场景。

可以先降低路径尺寸，或适当增大 `ik_timeout_s`。

### 轨迹速度或加速度超限

程序会先尝试自动拉长轨迹时间。如果仍然超限，可以降低：

```text
cartesian_speed_mps
cartesian_acceleration_mps2
```

也可以检查：

- CSV 点间距是否过大。
- 路径是否有尖锐跳变。
- `yaw_sample_count` 是否过小，导致 IK 解不够平滑。
- `max_joint_step_rad` 是否过紧或过松。过紧会较早报关节跳变，过松可能允许不够平滑的解进入轨迹。

如果看到 `Stretching trajectory time by ...x`，说明节点已经自动放慢轨迹来满足关节限制。这通常是正常现象。

### 参数类型报 expected string got bool

如果看到类似：

```text
expected [string] got [bool]
```

通常是 launch/YAML 把轴名 `y` 当成了布尔值。当前代码已经对这个情况做了兼容，launch 文件也会显式把轴映射参数作为字符串传递。若命令行覆盖参数仍然遇到类似问题，可以使用：

```bash
base_y_from_csv_axis:="'y'"
```
