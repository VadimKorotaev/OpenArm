#!/usr/bin/env python3
"""MoveIt smoke test for either OpenArm v2.0 arm on mock hardware.

The script first leaves the all-zero singular configuration using the upstream
``hands_up`` joint state, then asks /compute_ik for a nearby end-effector pose
and sends that same pose as a MoveGroup goal for planning and execution.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.msg import OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


WORLD_FRAME = "world"
HANDS_UP = [0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]


@dataclass(frozen=True)
class CartesianOffset:
    x: float
    y: float
    z: float


class IkMotionVerifier(Node):
    def __init__(self, arm: str) -> None:
        super().__init__(f"openarm_{arm}_ik_motion_verifier")
        self.arm = arm
        self.group = f"{arm}_arm"
        self.ee_link = f"openarm_{arm}_ee_base_link"
        self.joint_names = [f"openarm_{arm}_joint{index}" for index in range(1, 8)]
        self.move_group = ActionClient(self, MoveGroup, "/move_action")
        self.compute_ik = self.create_client(GetPositionIK, "/compute_ik")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def wait_for_future(self, future, timeout: float):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError(f"ROS request did not finish in {timeout:.1f} s")
        return future.result()

    def current_pose(self, timeout: float = 8.0) -> Pose:
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    WORLD_FRAME, self.ee_link, rclpy.time.Time()
                )
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                return pose
            except TransformException as exc:
                last_error = exc
        raise TimeoutError(
            f"TF {WORLD_FRAME} -> {self.ee_link} unavailable: {last_error}"
        )

    @staticmethod
    def format_pose(pose: Pose) -> str:
        p = pose.position
        q = pose.orientation
        return (
            f"position=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) m, "
            f"quaternion=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})"
        )

    @staticmethod
    def pose_with_offset(origin: Pose, offset: CartesianOffset) -> PoseStamped:
        target = PoseStamped()
        target.header.frame_id = WORLD_FRAME
        target.pose.position.x = origin.position.x + offset.x
        target.pose.position.y = origin.position.y + offset.y
        target.pose.position.z = origin.position.z + offset.z
        target.pose.orientation = origin.orientation
        return target

    def make_base_goal(self) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.group
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.10
        request.max_acceleration_scaling_factor = 0.10
        request.start_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def send_move_group_goal(self, goal: MoveGroup.Goal, label: str) -> None:
        if not self.move_group.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveGroup action /move_action is unavailable")
        goal_handle = self.wait_for_future(
            self.move_group.send_goal_async(goal), timeout=10.0
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"MoveGroup rejected {label}")
        wrapped_result = self.wait_for_future(
            goal_handle.get_result_async(), timeout=45.0
        )
        error_code = wrapped_result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{label} failed with MoveIt error {error_code}")
        self.get_logger().info(f"{label}: MoveIt SUCCESS ({error_code})")

    def move_to_hands_up(self) -> None:
        constraints = Constraints(name="hands_up")
        for name, position in zip(self.joint_names, HANDS_UP):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = 0.001
            joint.tolerance_below = 0.001
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal = self.make_base_goal()
        goal.request.goal_constraints = [constraints]
        self.send_move_group_goal(
            goal, f"{self.arm} hands_up joint pre-position"
        )

    def solve_ik(self, target: PoseStamped) -> bool:
        if not self.compute_ik.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MoveIt service /compute_ik is unavailable")
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = self.ee_link
        request.ik_request.pose_stamped = target
        request.ik_request.timeout = Duration(seconds=1.0).to_msg()
        response = self.wait_for_future(
            self.compute_ik.call_async(request), timeout=5.0
        )
        return response.error_code.val == MoveItErrorCodes.SUCCESS

    def move_to_pose(self, target: PoseStamped) -> None:
        volume = SolidPrimitive()
        volume.type = SolidPrimitive.SPHERE
        volume.dimensions = [0.002]

        position = PositionConstraint()
        position.header.frame_id = WORLD_FRAME
        position.link_name = self.ee_link
        position.constraint_region.primitives = [volume]
        position.constraint_region.primitive_poses = [target.pose]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = WORLD_FRAME
        orientation.link_name = self.ee_link
        orientation.orientation = target.pose.orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0

        constraints = Constraints(name="nearby_pose_via_ik")
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]

        goal = self.make_base_goal()
        goal.request.goal_constraints = [constraints]
        self.send_move_group_goal(goal, f"{self.arm} pose goal via IK")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify one OpenArm arm with a 30 mm pose-based IK motion."
    )
    parser.add_argument(
        "--arm",
        choices=("left", "right"),
        default="left",
        help="arm to move (default: left)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = IkMotionVerifier(args.arm)
    try:
        initial = node.current_pose()
        node.get_logger().info(f"initial EE: {node.format_pose(initial)}")
        node.move_to_hands_up()
        pre_ik = node.current_pose()
        node.get_logger().info(f"pre-IK EE: {node.format_pose(pre_ik)}")

        offsets = (
            CartesianOffset(0.03, 0.00, 0.00),
            CartesianOffset(0.00, 0.00, 0.03),
            CartesianOffset(0.00, 0.03, 0.00),
            CartesianOffset(-0.03, 0.00, 0.00),
        )
        target = None
        selected = None
        for offset in offsets:
            candidate = node.pose_with_offset(pre_ik, offset)
            if node.solve_ik(candidate):
                target = candidate
                selected = offset
                break
        if target is None or selected is None:
            raise RuntimeError("KDL IK rejected all nearby 30 mm pose targets")

        node.get_logger().info(
            "IK preflight SUCCESS for offset "
            f"({selected.x:.3f}, {selected.y:.3f}, {selected.z:.3f}) m"
        )
        node.get_logger().info(f"target EE: {node.format_pose(target.pose)}")
        node.move_to_pose(target)
        final = node.current_pose()
        node.get_logger().info(f"final EE: {node.format_pose(final)}")

        error = math.sqrt(
            (final.position.x - target.pose.position.x) ** 2
            + (final.position.y - target.pose.position.y) ** 2
            + (final.position.z - target.pose.position.z) ** 2
        )
        node.get_logger().info(f"Cartesian position error: {error * 1000.0:.2f} mm")
        if error > 0.01:
            raise RuntimeError(f"final Cartesian error is too large: {error:.4f} m")
        print(
            f"VERDICT: PASS — {args.arm} pose-based IK motion "
            "executed and verified"
        )
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        print(
            f"VERDICT: FAIL — {args.arm} pose-based IK motion was not verified",
            file=sys.stderr,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
