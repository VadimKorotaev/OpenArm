#!/usr/bin/env bash
# Apply this project's patches to the vendored openarm_bimanual_moveit_config
# package. Both patches are idempotent and should be re-applied after the
# upstream packages are (re-)cloned.
#
# 1. RViz config: adds a TF display for the ArUco marker and the two end
#    effectors, and frames the camera on the whole workspace. demo.launch.py
#    hardcodes its RViz config path and exposes no argument to override it, so
#    replacing the vendored file is the only way to avoid a second RViz window.
#
# 2. demo.launch.py: stops handing the RViz node the joint-limit and kinematics
#    parameters. RViz's MotionPlanning display re-declares those names as
#    strings, which conflicts with the doubles from the YAML and aborts
#    loadRobotModel -- leaving the 3D view with no robot and an empty planning
#    group. move_group keeps every parameter, so planning and IK are unchanged.
set -euo pipefail

readonly project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly package_root="${project_root}/ros2_ws/src/openarm_ros2/openarm_bimanual_moveit_config"
readonly source_config="${project_root}/config/openarm_skills.rviz"
readonly target_config="${package_root}/config/openarm_v2.0/moveit.rviz"
readonly target_launch="${package_root}/launch/demo.launch.py"

for path in "${source_config}" "${target_config}" "${target_launch}"; do
  if [[ ! -e "${path}" ]]; then
    echo "ERROR: missing ${path}" >&2
    echo "Follow the README section 'Исходники ROS и сборка' first." >&2
    exit 1
  fi
done

if cmp -s "${source_config}" "${target_config}"; then
  echo "rviz config: already applied"
else
  cp "${source_config}" "${target_config}"
  echo "rviz config: applied"
fi

python3 - "${target_launch}" <<'PY'
import sys
from pathlib import Path

launch = Path(sys.argv[1])
text = launch.read_text()

if "rviz_params" in text:
    print("demo.launch.py: already patched")
    raise SystemExit(0)

anchor = """    rviz_cfg = os.path.join(moveit_pkg_path, "config",
                            config_dir, "moveit.rviz")
"""
node = """        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_cfg],
            parameters=[moveit_params],
        ),"""

for fragment, label in ((anchor, "rviz_cfg assignment"), (node, "rviz2 Node")):
    if text.count(fragment) != 1:
        raise SystemExit(
            f"ERROR: {label} not found exactly once in {launch}; "
            "the upstream launch file has changed and this patch needs review."
        )

text = text.replace(
    anchor,
    anchor
    + """
    # RViz only displays the robot; move_group does the planning. Handing RViz
    # the joint-limit and kinematics parameters makes its MotionPlanning
    # display re-declare them with a different type, which aborts
    # loadRobotModel and leaves the 3D view without a robot.
    rviz_excluded = ("robot_description_planning",
                     "robot_description_kinematics")
    rviz_params = {k: v for k, v in moveit_params.items()
                   if k not in rviz_excluded}
""",
)
text = text.replace(node, node.replace("parameters=[moveit_params],", "parameters=[rviz_params],"))
launch.write_text(text)
print("demo.launch.py: applied")
PY

echo "Restart scripts/start_mock_bimanual.sh for the changes to take effect."
