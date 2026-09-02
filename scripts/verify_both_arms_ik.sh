#!/usr/bin/env bash
set -euo pipefail

readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${script_dir}/../ros2_ws/install/setup.bash"
set -u

readonly verifier="${script_dir}/verify_ik_motion.py"

python3 "${verifier}" --arm left
python3 "${verifier}" --arm right

echo "VERDICT: PASS — pose-based IK motion verified for both arms"
