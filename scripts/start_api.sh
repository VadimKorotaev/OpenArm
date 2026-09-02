#!/usr/bin/env bash
set -euo pipefail

readonly project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly python="${project_root}/.venv/bin/python"
readonly ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [[ ! -x "${python}" ]]; then
  echo "ERROR: virtual environment is missing: ${python}" >&2
  exit 1
fi

set +u
source "${ros_setup}"
source "${project_root}/ros2_ws/install/setup.bash"
set -u

export PYTHONPATH="${project_root}/backend${PYTHONPATH:+:${PYTHONPATH}}"
export OPENARM_SKILLS_DB="${OPENARM_SKILLS_DB:-${project_root}/data/openarm_skills.db}"
export OPENARM_ROS_ENABLED=1

exec "${python}" -m uvicorn openarm_skills.app:app \
  --host 127.0.0.1 \
  --port 8000
