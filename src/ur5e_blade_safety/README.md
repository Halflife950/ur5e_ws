# UR5e Blade Safety 当前整合记录

主仓库：

```text
https://github.com/Halflife950/ur5e_ws
```

本文记录当前 `ur5e_blade_safety` 工作流。旧的 `step*.md` 保留为历史记录；实际运行、调试和后续开发以本文为准。

## 1. 当前目标

当前系统分成两条职责清晰的链路：

```text
初始化链路：
  自动识别开口
  生成 pre-insert 目标
  MoveIt 将机器人运动到 pre-insert

安全联锁链路：
  持续检测 blade_center_link 扇形刀刃截面到轮廓边界的最小距离
  发布 safe / unsafe
  独立节点在 unsafe 时 cancel 当前 FollowJointTrajectory action
```

这样做的目的：

```text
opening_insert_executor 只负责初始化到 pre-insert
后续插入或其他运动可以由任意程序执行
blade_motion_safety_interlock 独立监督并中断控制器轨迹
```

## 2. 当前核心节点

```text
opening_alignment_planner
blade_contour_safety
blade_motion_safety_interlock
opening_insert_executor
cartesian_forward_probe
```

职责：

```text
opening_alignment_planner:
  读取 data/cavity_contour.csv
  识别唯一开口
  发布 /blade_preinsert_pose 和 /blade_insert_pose
  发布 /opening_alignment_markers

blade_contour_safety:
  根据当前 base_link -> blade_center_link TF 动态检测刀刃是否接近轮廓边界
  发布 /blade_contour_safe
  发布 /blade_contour_min_distance
  发布检测耗时和当前检测周期

blade_motion_safety_interlock:
  订阅 /blade_contour_safe 和 /blade_contour_min_distance
  unsafe 时 cancel 配置的 FollowJointTrajectory action 的所有 active goals
  与具体运动程序解耦

opening_insert_executor:
  等待 pre-insert / insert target pose
  将 blade_center_link 目标反算为 tool0 目标
  执行 approach -> pre-insert 初始化
  默认不执行 insert 阶段

cartesian_forward_probe:
  测试节点
  沿当前刀头方向做较长 Cartesian 直线运动
  用于验证 interlock 是否能在 unsafe 时中断 MoveIt/controller 轨迹
```

## 3. 关键文件

```text
src/ur5e_blade_safety/src/opening_alignment_planner.cpp
src/ur5e_blade_safety/src/blade_contour_safety.cpp
src/ur5e_blade_safety/src/blade_motion_safety_interlock.cpp
src/ur5e_blade_safety/src/opening_insert_executor.cpp
src/ur5e_blade_safety/src/cartesian_forward_probe.cpp

src/ur5e_blade_safety/config/blade_task_tuning.yaml

src/ur5e_blade_safety/launch/opening_monitor.launch.py
src/ur5e_blade_safety/launch/opening_insert_executor.launch.py
src/ur5e_blade_safety/launch/cartesian_forward_probe.launch.py

src/ur5e_blade_safety/rviz/blade_monitor.rviz

scripts/launch_ur5e_gazebo_gui.sh
```

主调参文件：

```text
src/ur5e_blade_safety/config/blade_task_tuning.yaml
```

当前 launch 默认都加载这个集中调参文件。

## 4. 启动顺序

终端 1：启动 Gazebo、MoveIt 和 RViz。

```bash
cd ~/ur5e_ws
./scripts/launch_ur5e_gazebo_gui.sh
```

如果遇到 Gazebo 报：

```text
Unable to start server[bind: Address already in use]
```

说明默认 Gazebo master 端口 `11345` 被旧 `gzserver` 占用。可以杀掉旧进程，或临时使用新端口：

```bash
GAZEBO_MASTER_URI=http://127.0.0.1:11346 ./scripts/launch_ur5e_gazebo_gui.sh
```

终端 2：启动目标生成、动态检测、安全联锁。

```bash
cd ~/ur5e_ws
source install/setup.bash
ros2 launch ur5e_blade_safety opening_monitor.launch.py
```

该 launch 启动：

```text
opening_alignment_planner
blade_contour_safety
blade_motion_safety_interlock
```

终端 3：初始化到 pre-insert。

```bash
cd ~/ur5e_ws
source install/setup.bash
ros2 launch ur5e_blade_safety opening_insert_executor.launch.py
```

默认行为：

```text
执行 approach
执行 approach -> pre-insert
到达 pre-insert 后停止
不执行 insert
```

终端 4：可选，执行向前直线测试。

```bash
cd ~/ur5e_ws
source install/setup.bash
ros2 launch ur5e_blade_safety cartesian_forward_probe.launch.py
```

预期：

```text
cartesian_forward_probe 发起 MoveIt Cartesian 直线运动
blade_contour_safety 检测到 unsafe
blade_motion_safety_interlock cancel joint_trajectory_controller 当前轨迹
forward probe 终端显示运动未完整成功，这属于预期
```

## 5. 当前 Topic

目标和可视化：

```text
/blade_preinsert_pose
/blade_insert_pose
/opening_alignment_markers
```

安全检测：

```text
/blade_contour_safe
/blade_contour_min_distance
/blade_contour_compute_time_ms
/blade_contour_detection_period_ms
```

控制器 action：

```text
/joint_trajectory_controller/follow_joint_trajectory
```

`blade_motion_safety_interlock` 当前通过这个 action 的 `async_cancel_all_goals()` 中断运动。

当前 workspace 的 Gazebo 仿真默认启动 `joint_trajectory_controller`，所以这个 action 名字对当前仿真流程是合理的。如果后续切到真实 UR5e，或 launch 时使用 `initial_joint_controller:=scaled_joint_trajectory_controller`，则需要改为：

```text
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

现场确认以实际 action 列表为准：

```bash
ros2 action list | grep follow_joint_trajectory
```

## 6. RViz 当前显示

RViz 配置：

```text
src/ur5e_blade_safety/rviz/blade_monitor.rviz
```

当前只配置一个 MarkerArray 显示：

```text
Topic: /opening_alignment_markers
Namespace: opening_alignment
```

显示内容：

```text
半透明蓝色壳体：开口轮廓沿 z 方向拉伸出的开口壳
青色线框：壳体边线
蓝色轮廓：非开口实体边界
白色线：识别出的开口
黄色端点：开口两端
橙色点：开口中心
绿色点：pre-insert 目标
```

已移除显示：

```text
外侧方向箭头
插入方向箭头
insert / inside 红色目标点
OPENING / PRE-INSERT / INSERT 字幕
当前刀刃采样点 marker
最近危险点 marker
```

刀头上的小红点来自 URDF 中的 `blade_center_link` visual，不属于 MarkerArray。

## 7. opening_alignment_planner

输入：

```text
data/cavity_contour.csv
```

输出：

```text
/blade_preinsert_pose
/blade_insert_pose
/opening_alignment_markers
```

核心逻辑：

```text
1. 读取 CSV 轮廓
2. 找相邻点最大 gap 作为唯一开口
3. 开口段不作为实体边界
4. 计算开口中心、外侧方向、插入方向
5. 生成 blade_center_link 的 pre-insert 和 insert pose
6. 发布简化后的 RViz marker
```

常用参数：

```yaml
pre_offset: 0.06
insert_depth: 0.02
contour_z: 0.20
target_z: 0.20
blade_shaft_axis: minus_y
shell_bottom_z: 0.0
shell_height: 0.40
```

虽然 RViz 不再显示 insert 点，`/blade_insert_pose` 仍然保留，供后续外部运动程序使用。

## 8. blade_contour_safety

输出：

```text
/blade_contour_safe
/blade_contour_min_distance
/blade_contour_compute_time_ms
/blade_contour_detection_period_ms
```

当前检测逻辑：

```text
CSV 原始点
识别最大 gap 作为开口
非开口点两两连线作为实体边界
根据 base_link -> blade_center_link TF 生成刀刃扇形采样点
计算采样点到实体边界线段的最小距离
min_distance >= safety_margin 则 safe=true
否则 safe=false
```

当前不做 B 样条、不做 SDF、不做轮廓平滑。理由是：

```text
polyline 版本简单、可验证、速度足够
当前最重要的是验证 unsafe 能否真正中断运动
```

常用参数：

```yaml
safety_margin: 0.003
danger_distance: 0.006
blade_radius: 0.005
blade_angle_deg: 47.2
blade_sample_spacing: 0.0005
base_timer_period_ms: 2.0
idle_publish_period_ms: 2.0
moving_publish_period_ms: 2.0
insert_publish_period_ms: 2.0
danger_publish_period_ms: 2.0
```

说明：

```text
当前不再使用明显不同的动态检测周期
idle / moving / insert / danger 都配置为 2 ms
实际频率仍受 ROS timer、TF 查询和系统调度影响
```

刀刃距离计算基于整个扇形采样截面，而不是只取扇形中心：

```text
blade_center_link 局部截面内生成扇形采样点
扇形半径由 blade_radius 控制，当前为 5 mm
扇形角度由 blade_angle_deg 控制，当前配置为 47.2 deg
对每个采样点计算到非开口轮廓线段的 XY 最小距离
所有采样点中的最小值作为 /blade_contour_min_distance
```

debug marker publisher 仍在代码中保留，但当前默认关闭：

```yaml
publish_markers: false
```

## 9. blade_motion_safety_interlock

文件：

```text
src/ur5e_blade_safety/src/blade_motion_safety_interlock.cpp
```

职责：

```text
监听 /blade_contour_safe
监听 /blade_contour_min_distance
当 safe 从 true 变为 false 时打印一次 unsafe 日志并发送 cancel
调用配置的 trajectory_action 的 cancel all goals
持续 unsafe 时不重复高频 cancel
safe 恢复后允许下一次 unsafe 再次 cancel
```

关键参数：

```yaml
trajectory_action: /joint_trajectory_controller/follow_joint_trajectory
cancel_trajectory_on_unsafe: true
log_transition_only: true
action_server_wait_s: 0.0
```

当前没有 latch 机制：

```text
没有 latched unsafe 状态
没有 /manual_reset_safety_interlock
unsafe 后不需要人工 reset
safe 重新变 true 后，下一次 unsafe 仍可再次触发 cancel
```

限制：

```text
它能中断走当前 trajectory_action 的运动
如果后续使用别的底层控制方式，需要为对应控制器补一层 stop / cancel 接口
真实 UR5e 的 dashboard stop / protective stop 目前还没有接入
```

当前仿真默认 action：

```text
/joint_trajectory_controller/follow_joint_trajectory
```

真实 UR5e 或 scaled controller 常见 action：

```text
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

## 10. opening_insert_executor

文件：

```text
src/ur5e_blade_safety/src/opening_insert_executor.cpp
```

当前默认用途：

```text
将机器人初始化到 pre-insert
不执行后续 insert
```

流程：

```text
1. 等待 /blade_preinsert_pose 和 /blade_insert_pose
2. 等待 /blade_contour_safe
3. 查询 tool0 -> blade_center_link 固定 TF
4. 将 blade_center_link 的 approach / pre-insert 目标反算为 tool0 目标
5. 用 preferred IK seed 规划 approach
6. 静态验证 approach plan 中 blade_center_link 不进入轮廓柱体
7. 执行 approach
8. Cartesian 执行 approach -> pre-insert
9. execute_insert_stage=false 时停止
```

关键参数：

```yaml
execute_motion: true
execute_insert_stage: false
require_safety_status_before_motion: true
require_safe_before_approach: true
require_safe_before_preinsert: true
approach_extra_offset: 0.08
velocity_scaling: 0.05
acceleration_scaling: 0.05
max_cartesian_speed: 0.02
cartesian_speed_link: blade_center_link
```

说明：

```text
executor 内仍保留一些旧的安全检查和 insert 执行代码路径
但当前 tuning 文件只暴露初始化到 pre-insert 所需参数
运行时任意外部运动的中断交给 blade_motion_safety_interlock
```

`max_cartesian_speed` 会对 executor 内部的 Cartesian 轨迹做重定时，当前主要限制 `approach -> pre-insert` 段的刀头线速度。默认用 `blade_center_link` 作为限速参考点，避免接近 pre-insert 时运动过快。

## 11. cartesian_forward_probe

文件：

```text
src/ur5e_blade_safety/src/cartesian_forward_probe.cpp
```

用途：

```text
验证安全联锁是否能中断 MoveIt 发起的控制器轨迹
```

流程：

```text
1. 查询 base_link -> blade_center_link
2. 取 blade_center_link 局部 minus_y 作为刀头前进方向
3. 转换为 base_link 下方向
4. 保持 tool0 当前姿态
5. 沿该方向规划较长 Cartesian 直线
6. 执行运动，等待 interlock 在 unsafe 时 cancel
```

关键参数：

```yaml
blade_forward_axis: minus_y
cartesian_distance: 0.35
velocity_scaling: 0.03
acceleration_scaling: 0.03
max_cartesian_speed: 0.02
cartesian_speed_link: blade_center_link
cartesian_eef_step: 0.002
```

`max_cartesian_speed` 会在规划后重定时轨迹，限制 `cartesian_speed_link` 在相邻轨迹点之间的 Cartesian 线速度。当前用于避免 probe 运动过快导致检测和 cancel 没有足够响应距离。

如果距离不够触发 unsafe，可以继续增大：

```yaml
cartesian_distance: 0.50
```

## 12. 当前验证结论

已验证：

```text
Gazebo + MoveIt 可正常启动
自定义 blade_center_link / screwdriver_tool_link 已进入 robot_state_publisher
joint_state_broadcaster active
joint_trajectory_controller active
/joint_states 正常发布
opening_monitor.launch.py 启动目标生成、动态检测、安全联锁
opening_insert_executor.launch.py 默认初始化到 pre-insert
cartesian_forward_probe 可触发长距离 Cartesian 测试
unsafe 后 interlock 能 cancel 控制器轨迹
```

已知注意点：

```text
如果 Gazebo 默认端口 11345 被旧进程占用，新 gzserver 会启动失败
如果在受限沙箱中运行 ROS/Gazebo，可能需要 ROS_LOG_DIR=/tmp/ros_log 或宿主权限
MoveIt 关于 tool link visual 无 collision geometry 的 warning 暂不影响当前测试
Octomap sensor plugin warning 暂不影响当前流程
```

## 13. Step Notes 中仍然有效的细节

旧的 step note 里有不少是历史实现，当前源码已经清理掉了，例如 `blade_center_monitor`、`blade_contour_visualizer`、旧的单节点 yaml、旧脚本等。下面只保留当前系统仍然需要记住的细节。

### 13.1 自定义刀头模型

自定义工具 description 包：

```text
src/ur5e_custom_tool_description/
```

关键文件：

```text
src/ur5e_custom_tool_description/urdf/screwdriver_tool.xacro
src/ur5e_custom_tool_description/urdf/ur_with_screwdriver.urdf.xacro
src/ur5e_custom_tool_description/meshes/Screwdriver.stl
src/ur5e_custom_tool_description/config/ur5e/
```

当前结构：

```text
wrist_3_link
  -> screwdriver_tool_link
       -> blade_center_link
```

重要原因：

```text
单独的 screwdriver_tool.xacro 只是工具片段，不能作为完整机器人启动
ur_with_screwdriver.urdf.xacro 负责 include 官方 UR robot macro 和自定义刀头
ur_moveit_config 会从 description_package/config/ur5e/ 读取 UR 官方配置
Gazebo Classic 可能折叠 tool0 等 fixed frame，所以工具挂在 wrist_3_link 下更稳定
```

当前关键装配参数：

```text
mesh path: file://$(find ur5e_custom_tool_description)/meshes/Screwdriver.stl
mesh scale: 0.001 0.001 0.001
visual rpy: 0 pi 0
blade_center_link xyz: 0 -0.115 0.075
```

`blade_center_link` 表示扇形刀刃圆心/检测截面参考点，不是刀尖。刀头上的红色小球来自该 link 的 visual。

MoveIt 侧不再使用完整 STL collision mesh：

```text
完整 STL collision 曾导致 wrist_3_link 与 screwdriver_tool_link 起始自碰撞
当前保留 visual + inertial，自定义轮廓安全由 blade_contour_safety 负责
如后续希望 MoveIt 也感知工具碰撞，应使用简化 collision 或 SRDF disabled collision
```

### 13.2 CSV 轮廓与开口识别

当前轮廓文件：

```text
src/ur5e_blade_safety/data/cavity_contour.csv
```

CSV 表头是：

```text
x_mm,y_mm,z_mm
```

所以必须使用：

```yaml
csv_scale: 0.001
```

开口识别必须检查最后一个点到第一个点：

```text
points.back() -> points.front()
```

当前数据中最大 gap 是：

```text
376 -> 0
```

如果漏掉首尾 gap，会把开口错误闭合成实体边界。

### 13.3 开口目标生成

`opening_alignment_planner` 的几何规则来自 Step 4：

```text
A / B: 最大 gap 两端点
M = (A + B) / 2
t = normalize(B - A)
n1 / n2: 开口线的两个法向
outward_dir: 更朝向 base_link 原点的一侧
insert_dir = -outward_dir
pre_center = M + outward_dir * pre_offset
inside_center = M - outward_dir * insert_depth
```

姿态规则：

```text
tool_z_axis_down=true 时，目标局部 Z 轴对齐 base_link 的 -Z
blade_shaft_axis=minus_y 时，目标局部 -Y 轴对齐 insert_dir
```

如果 RViz 中确认刀杆方向反了，优先检查：

```yaml
blade_shaft_axis: minus_y
```

必要时改成 `plus_y`。

### 13.4 MoveIt 执行目标反算

MoveIt 当前规划组：

```text
group: ur_manipulator
chain: base_link -> tool0
```

`blade_center_link` 不直接作为 MoveIt tip，因此 executor 使用固定 TF 反算：

```text
T_base_tool0_target = T_base_blade_target * inverse(T_tool0_blade)
```

这样 MoveIt 控制 `tool0`，但最终让 `blade_center_link` 到达 pre-insert 目标。

当前 preferred IK seed 来自早期 curve path demo 的稳定姿态，用于减少 IK 多解导致的上下翻转和绕路：

```yaml
preferred_shoulder_pan_joint: 0.0
preferred_shoulder_lift_joint: -1.2217305
preferred_elbow_joint: 1.8849556
preferred_wrist_1_joint: -2.2165682
preferred_wrist_2_joint: -1.5533430
preferred_wrist_3_joint: 0.0
```

如果 `tool0 -> blade_center_link` 查不到，优先检查：

```bash
ros2 run tf2_ros tf2_echo tool0 blade_center_link
```

### 13.5 Approach 与禁入区验证

Step 6 后，初始化不再直接规划到 pre-insert，而是：

```text
1. 根据 pre-insert 和 insert 连线生成更外侧的 approach 点
2. 对 approach 使用 IK + MoveIt joint plan
3. 对 plan 逐点正解 blade_center_link
4. 检查 blade_center_link 是否进入由轮廓 XY 拉伸出的柱状禁入区
5. 若进入禁入区，则拒绝该 plan 并重新规划
6. approach -> pre-insert 使用 Cartesian 直线
7. execute_insert_stage=false 时在 pre-insert 停止
```

关键参数：

```yaml
reject_preinsert_plan_inside_contour: true
valid_preinsert_plan_attempts: 5
approach_extra_offset: 0.08
execute_insert_stage: false
```

### 13.6 RViz 与 Gazebo 注意事项

RViz 建议：

```text
Fixed Frame = base_link
MarkerArray topic = /opening_alignment_markers
```

当前 RViz 只保留一个 MarkerArray，避免旧调试 marker 重复显示。历史上的箭头、insert 点、文字、采样点和最近危险点都已移除或默认关闭。

Gazebo 注意事项：

```text
旧 gzserver 残留可能导致 Entity [ur] already exists
默认 Gazebo master 端口 11345 被占用会导致 bind: Address already in use
当前脚本避免使用 LIBGL_ALWAYS_SOFTWARE=1，因为它曾导致 Gazebo GUI 卡顿
```

可用排查命令：

```bash
ps -ef | rg 'gzserver|gzclient|ros2 launch|rviz2|robot_state_publisher|controller_manager'
ros2 action list | grep follow_joint_trajectory
ros2 topic echo /blade_contour_safe
ros2 topic echo /blade_contour_min_distance
```

## 14. 后续建议

优先级建议：

```text
1. 若后续运动仍走 joint_trajectory_controller，继续使用当前 interlock
2. 若换成其他控制方式，为对应控制器补独立 stop/cancel 后端
3. 如果需要真实 UR5e 安全停止，再接 dashboard stop / speed scaling / protective stop 策略
4. 如果 polyline 检测误停，再考虑轮廓平滑、B 样条或 SDF
5. 后续如果接入非 MoveIt 控制程序，应先确认它对应的 stop/cancel 接口
```
