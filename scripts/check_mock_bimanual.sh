#!/usr/bin/env bash
set -uo pipefail

readonly project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
readonly workspace_setup="${project_root}/ros2_ws/install/setup.bash"
failures=0

set +u
source "${ros_setup}"
source "${workspace_setup}"
set -u

pass() {
  echo "PASS: $1"
}

fail() {
  echo "FAIL: $1" >&2
  failures=$((failures + 1))
}

node_is_visible() {
  local expected="$1"
  local attempt
  for attempt in 1 2 3 4 5; do
    if ros2 node list 2>/dev/null | grep -Fxq "${expected}"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

tf_is_available() {
  local target="$1"
  local attempt
  local output
  for attempt in 1 2 3; do
    output="$(timeout 4s ros2 run tf2_ros tf2_echo world "${target}" 2>&1 || true)"
    if grep -q -- "- Translation:" <<<"${output}"; then
      return 0
    fi
  done
  return 1
}

for node in /controller_manager /move_group /robot_state_publisher /rviz2; do
  if node_is_visible "${node}"; then
    pass "node ${node}"
  else
    fail "node ${node} is missing"
  fi
done

controllers="$(ros2 control list_controllers 2>/dev/null | sed -E $'s/\033\\[[0-9;]*[mK]//g')"
for controller in \
  joint_state_broadcaster \
  left_joint_trajectory_controller \
  right_joint_trajectory_controller \
  left_gripper_controller \
  right_gripper_controller; do
  if grep -E "^${controller}[[:space:]].*[[:space:]]active$" <<<"${controllers}" >/dev/null; then
    pass "controller ${controller} is active"
  else
    fail "controller ${controller} is not active"
  fi
done

hardware="$(ros2 control list_hardware_components 2>/dev/null | sed -E $'s/\033\\[[0-9;]*[mK]//g')"
for component in openarm_left_hardware_interface openarm_right_hardware_interface; do
  if awk -v component="${component}" '
      $1 == "name:" { selected = ($2 == component) }
      selected && $1 == "state:" && /id=3 label=active/ { found = 1 }
      END { exit !found }
    ' <<<"${hardware}"; then
    pass "mock hardware ${component} is active"
  else
    fail "mock hardware ${component} is not active"
  fi
done

services="$(ros2 service list 2>/dev/null)"
if grep -Fxq "/compute_ik" <<<"${services}"; then
  pass "service /compute_ik"
else
  fail "service /compute_ik is missing"
fi

actions="$(ros2 action list 2>/dev/null)"
if grep -Fxq "/move_action" <<<"${actions}"; then
  pass "action /move_action"
else
  fail "action /move_action is missing"
fi

for arm in left right; do
  link="openarm_${arm}_ee_base_link"
  if tf_is_available "${link}"; then
    pass "TF world -> ${link}"
  else
    fail "TF world -> ${link} is unavailable"
  fi
done

if ((failures > 0)); then
  echo "VERDICT: FAIL (${failures} check(s) failed)" >&2
  exit 1
fi

echo "VERDICT: PASS — default_bimanual mock stack is healthy"
