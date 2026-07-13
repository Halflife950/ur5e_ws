#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/tinavi/ur5e_ws}"
GAZEBO_READY_TIMEOUT_S="${GAZEBO_READY_TIMEOUT_S:-180}"
MONITOR_READY_TIMEOUT_S="${MONITOR_READY_TIMEOUT_S:-60}"

cd "$WORKSPACE_DIR"

source_workspace() {
  set +u
  source install/setup.bash
  set -u
}

source_workspace

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "[blade-stack] gnome-terminal was not found. Please install it or run launches manually." >&2
  exit 1
fi

open_terminal() {
  local title="$1"
  local command="$2"

  gnome-terminal \
    --title="$title" \
    -- bash -lc "set +u; cd '$WORKSPACE_DIR'; source install/setup.bash; set -u; $command; echo; echo '[blade-stack] terminal finished. Press Ctrl-D or close this window.'; exec bash"
}

wait_for_node() {
  local node_name="$1"
  local timeout_s="$2"
  local start_s
  start_s="$(date +%s)"

  echo "[blade-stack] waiting for node: ${node_name}"
  while true; do
    if ros2 node list 2>/dev/null | grep -Fxq "$node_name"; then
      echo "[blade-stack] node ready: ${node_name}"
      return 0
    fi

    if (( "$(date +%s)" - start_s >= timeout_s )); then
      echo "[blade-stack] timeout waiting for node: ${node_name}" >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_topic() {
  local topic_name="$1"
  local timeout_s="$2"
  local start_s
  start_s="$(date +%s)"

  echo "[blade-stack] waiting for topic: ${topic_name}"
  while true; do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic_name"; then
      echo "[blade-stack] topic ready: ${topic_name}"
      return 0
    fi

    if (( "$(date +%s)" - start_s >= timeout_s )); then
      echo "[blade-stack] timeout waiting for topic: ${topic_name}" >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_action() {
  local action_name="$1"
  local timeout_s="$2"
  local start_s
  start_s="$(date +%s)"

  echo "[blade-stack] waiting for action: ${action_name}"
  while true; do
    if ros2 action list 2>/dev/null | grep -Fxq "$action_name"; then
      echo "[blade-stack] action ready: ${action_name}"
      return 0
    fi

    if (( "$(date +%s)" - start_s >= timeout_s )); then
      echo "[blade-stack] timeout waiting for action: ${action_name}" >&2
      return 1
    fi
    sleep 1
  done
}

echo "[blade-stack] opening Gazebo + MoveIt + RViz terminal..."
open_terminal "blade gazebo moveit rviz" "./scripts/launch_ur5e_gazebo_gui.sh"

wait_for_node "/move_group" "$GAZEBO_READY_TIMEOUT_S"
wait_for_topic "/joint_states" "$GAZEBO_READY_TIMEOUT_S"
wait_for_action "/joint_trajectory_controller/follow_joint_trajectory" "$GAZEBO_READY_TIMEOUT_S"

echo "[blade-stack] opening opening-monitor terminal..."
open_terminal "blade opening monitor" "ros2 launch ur5e_blade_safety opening_monitor.launch.py"

wait_for_node "/opening_alignment_planner" "$MONITOR_READY_TIMEOUT_S"
wait_for_node "/blade_contour_safety" "$MONITOR_READY_TIMEOUT_S"
wait_for_node "/blade_motion_safety_interlock" "$MONITOR_READY_TIMEOUT_S"
wait_for_topic "/blade_preinsert_pose" "$MONITOR_READY_TIMEOUT_S"
wait_for_topic "/blade_contour_safe" "$MONITOR_READY_TIMEOUT_S"

echo "[blade-stack] opening insert-executor terminal to initialize robot to pre-insert..."
open_terminal "blade insert executor" "ros2 launch ur5e_blade_safety opening_insert_executor.launch.py"

echo
echo "[blade-stack] started Gazebo/MoveIt, opening monitor, and insert executor."
echo "[blade-stack] Watch the insert-executor terminal until it reaches pre-insert."
echo "[blade-stack] Then run the final motion in another terminal, for example:"
echo "  cd $WORKSPACE_DIR"
echo "  source install/setup.bash"
echo "  ros2 launch ur5e_blade_safety cartesian_forward_probe.launch.py"
