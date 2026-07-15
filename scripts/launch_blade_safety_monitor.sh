#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/tinavi/ur5e_ws}"
MONITOR_READY_TIMEOUT_S="${MONITOR_READY_TIMEOUT_S:-60}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
ENABLE_INTERLOCK="${ENABLE_INTERLOCK:-true}"
TUNING_FILE="${TUNING_FILE:-}"

cd "$WORKSPACE_DIR"

source_workspace() {
  set +u
  source install/setup.bash
  set -u
}

source_workspace

launch_args=(
  "enable_alignment:=false"
  "enable_interlock:=${ENABLE_INTERLOCK}"
  "enable_safety_markers:=true"
  "use_sim_time:=${USE_SIM_TIME}"
)

if [[ -n "$TUNING_FILE" ]]; then
  launch_args+=("tuning_file:=${TUNING_FILE}")
fi

wait_for_node() {
  local node_name="$1"
  local timeout_s="$2"
  local start_s
  start_s="$(date +%s)"

  echo "[blade-safety-monitor] waiting for node: ${node_name}"
  while true; do
    if ros2 node list 2>/dev/null | grep -Fxq "$node_name"; then
      echo "[blade-safety-monitor] node ready: ${node_name}"
      return 0
    fi

    if (( "$(date +%s)" - start_s >= timeout_s )); then
      echo "[blade-safety-monitor] timeout waiting for node: ${node_name}" >&2
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

  echo "[blade-safety-monitor] waiting for topic: ${topic_name}"
  while true; do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic_name"; then
      echo "[blade-safety-monitor] topic ready: ${topic_name}"
      return 0
    fi

    if (( "$(date +%s)" - start_s >= timeout_s )); then
      echo "[blade-safety-monitor] timeout waiting for topic: ${topic_name}" >&2
      return 1
    fi
    sleep 1
  done
}

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "[blade-safety-monitor] gnome-terminal was not found. Running monitor in this terminal."
  ros2 launch ur5e_blade_safety opening_monitor.launch.py "${launch_args[@]}"
  exit 0
fi

echo "[blade-safety-monitor] opening contour safety + interlock terminal..."
gnome-terminal \
  --title="blade safety monitor" \
  -- bash -lc "set +u; cd '$WORKSPACE_DIR'; source install/setup.bash; set -u; ros2 launch ur5e_blade_safety opening_monitor.launch.py ${launch_args[*]}; echo; echo '[blade-safety-monitor] terminal finished. Press Ctrl-D or close this window.'; exec bash"

wait_for_node "/blade_contour_safety" "$MONITOR_READY_TIMEOUT_S"
if [[ "$ENABLE_INTERLOCK" == "true" ]]; then
  wait_for_node "/blade_motion_safety_interlock" "$MONITOR_READY_TIMEOUT_S"
fi
wait_for_topic "/blade_contour_safe" "$MONITOR_READY_TIMEOUT_S"
wait_for_topic "/blade_contour_min_distance" "$MONITOR_READY_TIMEOUT_S"

echo
echo "[blade-safety-monitor] safety monitor is running without opening_alignment_planner."
echo "[blade-safety-monitor] USE_SIM_TIME=${USE_SIM_TIME}, ENABLE_INTERLOCK=${ENABLE_INTERLOCK}"
