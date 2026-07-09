# ur5e_custom_tool_description

Main repository:

```text
https://github.com/Halflife950/ur5e_ws
```

This package provides the UR5e robot description wrapper used by the blade
safety workflow. It keeps the standard Universal Robots UR5e model and adds the
custom screwdriver/blade visual tool plus a `blade_center_link` reference frame.

## Purpose

The package is used as the `description_package` for the current Gazebo, RViz,
MoveIt and safety-monitor workflow.

It provides:

```text
UR5e base robot from ur_description
custom Screwdriver.stl visual mesh
screwdriver_tool_link fixed to the wrist
blade_center_link fixed to the custom tool
UR official config files required by ur_moveit_config
```

Current link structure:

```text
wrist_3_link
  -> screwdriver_tool_link
       -> blade_center_link
```

`blade_center_link` is the reference frame used by `ur5e_blade_safety` for
opening alignment and contour safety detection. It represents the blade sector
center/reference frame, not the physical blade tip.

## Key Files

```text
urdf/ur_with_screwdriver.urdf.xacro
urdf/screwdriver_tool.xacro
meshes/Screwdriver.stl
config/ur5e/joint_limits.yaml
config/ur5e/default_kinematics.yaml
config/ur5e/physical_parameters.yaml
config/ur5e/visual_parameters.yaml
```

`ur_with_screwdriver.urdf.xacro` includes the standard UR robot macro and then
includes `screwdriver_tool.xacro`.

`screwdriver_tool.xacro` defines:

```text
screwdriver_tool_link
tool0_to_screwdriver_tool fixed joint
blade_center_link
screwdriver_tool_to_blade_center fixed joint
Gazebo preserveFixedJoint tags
```

## Important Geometry

Current tool mesh settings:

```text
mesh path: file://$(find ur5e_custom_tool_description)/meshes/Screwdriver.stl
mesh scale: 0.001 0.001 0.001
visual rpy: 0 pi 0
```

Current blade center offset:

```text
parent: screwdriver_tool_link
child: blade_center_link
xyz: 0 -0.115 0.075
rpy: 0 0 0
```

The red sphere visible on the tool comes from `blade_center_link` visual
geometry. It is useful for RViz/Gazebo inspection, but the actual safety
decision is computed in `ur5e_blade_safety`.

## Why The Tool Is Attached To wrist_3_link

The first attempt was semantically closer to:

```text
tool0 -> screwdriver_tool_link
```

RViz can display that, but Gazebo Classic may fold or drop fixed frames around
`tool0` when generating the simulated model. Attaching the tool to
`wrist_3_link` keeps the custom link stable in Gazebo while the rest of the
system still treats it as an end-effector tool.

The joint name remains:

```text
tool0_to_screwdriver_tool
```

but its parent link is currently:

```text
wrist_3_link
```

## Collision Model

The custom STL is intentionally not used as a full MoveIt collision mesh.

Earlier tests with the full STL collision geometry caused start-state
self-collision between `wrist_3_link` and `screwdriver_tool_link`. The current
workflow keeps:

```text
visual mesh
small inertial model
blade_center_link visual marker
```

Blade-to-contour safety is handled by:

```text
ur5e_blade_safety/src/blade_contour_safety.cpp
```

If MoveIt needs to reason about the tool collision later, prefer a simplified
collision primitive or an SRDF disabled-collision setup instead of reusing the
full visual STL directly.

## Gazebo Notes

Gazebo-specific details in `screwdriver_tool.xacro`:

```text
preserveFixedJoint for tool0_to_screwdriver_tool
preserveFixedJoint for screwdriver_tool_to_blade_center
Gazebo/DarkGrey material for screwdriver_tool_link
Gazebo/Red material for blade_center_link
```

The mesh path uses `file://$(find ...)` because it has been more reliable in
Gazebo Classic than `package://` for this setup.

If the tool disappears in Gazebo, first check:

```bash
ros2 run tf2_ros tf2_echo wrist_3_link screwdriver_tool_link
ros2 run tf2_ros tf2_echo tool0 blade_center_link
```

If Gazebo still shows an old model, stop stale `gzserver` / `gzclient` processes
and relaunch the simulation.

## Relationship To ur5e_blade_safety

The safety package depends on these frames:

```text
tool0
screwdriver_tool_link
blade_center_link
```

`opening_insert_executor` computes a desired `tool0` pose from a desired
`blade_center_link` pose:

```text
T_base_tool0_target = T_base_blade_target * inverse(T_tool0_blade)
```

`blade_contour_safety` queries:

```text
base_link -> blade_center_link
```

and evaluates the configured blade fan geometry around that frame.

For the current full workflow, see:

```text
../ur5e_blade_safety/README.md
```
