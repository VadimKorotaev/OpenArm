#!/usr/bin/env bash
set -euo pipefail

readonly project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
readonly workspace_setup="${project_root}/ros2_ws/install/setup.bash"

if [[ ! -r "${ros_setup}" ]]; then
  echo "ERROR: ROS 2 Humble setup was not found: ${ros_setup}" >&2
  exit 1
fi

if [[ ! -r "${workspace_setup}" ]]; then
  echo "ERROR: workspace is not built: ${workspace_setup}" >&2
  exit 1
fi

set +u
source "${ros_setup}"
source "${workspace_setup}"
set -u

exec ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  arm_type:=v2.0 \
  use_fake_hardware:=true \
  robot_controller:=joint_trajectory_controller
