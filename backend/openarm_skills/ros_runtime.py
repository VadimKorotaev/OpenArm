"""ROS 2 / MoveIt adapter used by the Skill execution manager."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose as PoseMessage
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, MoveItErrorCodes
from moveit_msgs.msg import OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .execution import CancellationToken, ExecutionCancelledError
from .gripper import opening_to_joint_position
from .models import (
    CartesianPose,
    GripperAction,
    MarkerState,
    MoveAction,
    PoseReference,
    Quaternion,
    Vector3,
)
from .trajectory_sync import TrajectoryPoint, duration, sample_at, sample_times
from .transforms import compose, inverse


WORLD_FRAME = "world"


@dataclass(frozen=True)
class ActiveMotion:
    """A trajectory currently running on one arm, in monotonic wall time."""

    started_at: float
    joint_names: list[str]
    points: list[TrajectoryPoint]


def trajectory_points(trajectory) -> list[TrajectoryPoint]:
    return [
        (
            point.time_from_start.sec + point.time_from_start.nanosec * 1e-9,
            tuple(point.positions),
        )
        for point in trajectory.points
    ]


class RosMotionRuntime:
    def __init__(self) -> None:
        self.context: Context | None = None
        self.node: Node | None = None
        self.executor: MultiThreadedExecutor | None = None
        self.spin_thread: threading.Thread | None = None
        self.move_group: ActionClient | None = None
        self.compute_ik = None
        self.state_validity = None
        self.arm_controllers: dict[str, ActionClient] = {}
        self.grippers: dict[str, ActionClient] = {}
        # Held while a plan is validated against the arms already moving and
        # handed to its controller, so two branches cannot start blind to each
        # other in the window between the check and the goal.
        self.dispatch_lock = threading.Lock()
        self.active_motions: dict[str, ActiveMotion] = {}
        self.tf_buffer: Buffer | None = None
        self.tf_listener: TransformListener | None = None
        self.marker_broadcaster: TransformBroadcaster | None = None
        self.marker_timer = None
        self.marker_lock = threading.Lock()
        self.markers: dict[str, CartesianPose] = {}
        self.ready = False

    def start(self) -> None:
        if self.ready:
            return
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("openarm_skill_runtime", context=self.context)
        self.executor = MultiThreadedExecutor(num_threads=6, context=self.context)
        self.executor.add_node(self.node)

        # Both arms query move_group from their own branch threads, so these
        # clients must be re-entrant; a mutually exclusive group would put the
        # planning and validity round trips of the two arms back in a queue.
        planning_group = ReentrantCallbackGroup()
        self.move_group = ActionClient(
            self.node, MoveGroup, "/move_action", callback_group=planning_group
        )
        self.compute_ik = self.node.create_client(
            GetPositionIK, "/compute_ik", callback_group=planning_group
        )
        self.state_validity = self.node.create_client(
            GetStateValidity, "/check_state_validity", callback_group=planning_group
        )
        # One group per arm: a busy left arm never delays right-arm callbacks,
        # while the goals of a single arm stay strictly ordered.
        arm_groups = {
            arm: MutuallyExclusiveCallbackGroup() for arm in ("left", "right")
        }
        self.arm_controllers = {
            arm: ActionClient(
                self.node,
                FollowJointTrajectory,
                f"/{arm}_joint_trajectory_controller/follow_joint_trajectory",
                callback_group=arm_groups[arm],
            )
            for arm in ("left", "right")
        }
        self.grippers = {
            arm: ActionClient(
                self.node,
                FollowJointTrajectory,
                f"/{arm}_gripper_controller/follow_joint_trajectory",
                callback_group=arm_groups[arm],
            )
            for arm in ("left", "right")
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self.node, spin_thread=False
        )
        self.marker_broadcaster = TransformBroadcaster(self.node)
        self.marker_timer = self.node.create_timer(0.05, self._publish_markers)

        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            name="openarm-ros-executor",
            daemon=True,
        )
        self.spin_thread.start()

        if not self.move_group.wait_for_server(timeout_sec=10.0):
            self.stop()
            raise RuntimeError("MoveIt action /move_action is unavailable")
        if not self.compute_ik.wait_for_service(timeout_sec=10.0):
            self.stop()
            raise RuntimeError("MoveIt service /compute_ik is unavailable")
        if not self.state_validity.wait_for_service(timeout_sec=10.0):
            self.stop()
            raise RuntimeError("MoveIt service /check_state_validity is unavailable")
        for arm, client in self.arm_controllers.items():
            if not client.wait_for_server(timeout_sec=10.0):
                self.stop()
                raise RuntimeError(f"{arm} arm trajectory action is unavailable")
        for arm, client in self.grippers.items():
            if not client.wait_for_server(timeout_sec=10.0):
                self.stop()
                raise RuntimeError(f"{arm} gripper action is unavailable")
        self.lookup_world_pose("openarm_left_ee_base_link", timeout=10.0)
        self.lookup_world_pose("openarm_right_ee_base_link", timeout=10.0)
        self.ready = True
        self.node.get_logger().info("OpenArm Skill ROS runtime is ready")

    def stop(self) -> None:
        self.ready = False
        with self.dispatch_lock:
            self.active_motions.clear()
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.0)
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None and self.context.ok():
            self.context.shutdown()
        if self.spin_thread is not None:
            self.spin_thread.join(timeout=2.0)
        self.spin_thread = None
        self.executor = None
        self.node = None
        self.context = None

    def _publish_markers(self) -> None:
        if self.node is None or self.marker_broadcaster is None:
            return
        with self.marker_lock:
            marker_items = [
                (frame_id, pose.model_copy(deep=True))
                for frame_id, pose in self.markers.items()
            ]
        if not marker_items:
            return
        timestamp = self.node.get_clock().now().to_msg()
        messages: list[TransformStamped] = []
        for frame_id, pose in marker_items:
            message = TransformStamped()
            message.header.stamp = timestamp
            message.header.frame_id = WORLD_FRAME
            message.child_frame_id = frame_id
            message.transform.translation.x = pose.position.x
            message.transform.translation.y = pose.position.y
            message.transform.translation.z = pose.position.z
            message.transform.rotation.x = pose.orientation.x
            message.transform.rotation.y = pose.orientation.y
            message.transform.rotation.z = pose.orientation.z
            message.transform.rotation.w = pose.orientation.w
            messages.append(message)
        self.marker_broadcaster.sendTransform(messages)

    def set_marker(self, frame_id: str, pose: CartesianPose) -> MarkerState:
        with self.marker_lock:
            self.markers[frame_id] = pose.model_copy(deep=True)
        return MarkerState(frame_id=frame_id, pose=pose)

    def list_markers(self) -> list[MarkerState]:
        with self.marker_lock:
            return [
                MarkerState(frame_id=frame_id, pose=pose.model_copy(deep=True))
                for frame_id, pose in sorted(self.markers.items())
            ]

    def delete_marker(self, frame_id: str) -> None:
        with self.marker_lock:
            if self.markers.pop(frame_id, None) is None:
                raise KeyError(frame_id)

    @staticmethod
    def _pose_message(value: CartesianPose) -> PoseMessage:
        result = PoseMessage()
        result.position.x = value.position.x
        result.position.y = value.position.y
        result.position.z = value.position.z
        result.orientation.x = value.orientation.x
        result.orientation.y = value.orientation.y
        result.orientation.z = value.orientation.z
        result.orientation.w = value.orientation.w
        return result

    @staticmethod
    def _transform_pose(transform) -> CartesianPose:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return CartesianPose(
            position=Vector3(
                x=translation.x,
                y=translation.y,
                z=translation.z,
            ),
            orientation=Quaternion(
                x=rotation.x,
                y=rotation.y,
                z=rotation.z,
                w=rotation.w,
            ),
        )

    def lookup_world_pose(self, frame_id: str, timeout: float = 5.0) -> CartesianPose:
        if self.tf_buffer is None:
            raise RuntimeError("ROS runtime is not started")
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    WORLD_FRAME, frame_id, rclpy.time.Time()
                )
                return self._transform_pose(transform)
            except TransformException as exc:
                last_error = exc
                time.sleep(0.05)
        raise RuntimeError(f"TF {WORLD_FRAME} -> {frame_id} unavailable: {last_error}")

    def current_pose(self, arm: str, reference: PoseReference) -> CartesianPose:
        world_ee = self.lookup_world_pose(f"openarm_{arm}_ee_base_link")
        if reference.kind == "world":
            return world_ee
        world_reference = self.lookup_world_pose(reference.frame_id)
        # T_marker_ee = inverse(T_world_marker) * T_world_ee
        return compose(inverse(world_reference), world_ee)

    def _resolve_target(self, action: MoveAction) -> CartesianPose:
        if action.target.reference.kind == "world":
            return action.target.pose
        world_marker = self.lookup_world_pose(action.target.reference.frame_id)
        # T_world_target = T_world_marker_current * T_marker_ee_saved
        return compose(world_marker, action.target.pose)

    @staticmethod
    def _wait_future(
        future,
        timeout: float,
        cancellation: CancellationToken | None = None,
        poll: float = 0.02,
    ):
        deadline = time.monotonic() + timeout
        while not future.done():
            if cancellation is not None:
                cancellation.raise_if_cancelled()
            if time.monotonic() >= deadline:
                raise TimeoutError(f"ROS request timed out after {timeout:.1f} s")
            time.sleep(poll)
        result = future.result()
        if result is None:
            raise RuntimeError("ROS request completed without a result")
        return result

    def _preflight_ik(
        self,
        action: MoveAction,
        target: CartesianPose,
        cancellation: CancellationToken,
    ) -> None:
        request = GetPositionIK.Request()
        request.ik_request.group_name = f"{action.arm}_arm"
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = f"openarm_{action.arm}_ee_base_link"
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = WORLD_FRAME
        request.ik_request.pose_stamped.pose = self._pose_message(target)
        request.ik_request.timeout = Duration(seconds=1.0).to_msg()
        response = self._wait_future(
            self.compute_ik.call_async(request),
            timeout=5.0,
            cancellation=cancellation,
        )
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"IK failed for {action.arm} arm with MoveIt error "
                f"{response.error_code.val}"
            )

    def move(self, action: MoveAction, cancellation: CancellationToken) -> None:
        if not self.ready or self.move_group is None:
            raise RuntimeError("ROS runtime is not ready")
        cancellation.raise_if_cancelled()
        target = self._resolve_target(action)
        self._preflight_ik(action, target, cancellation)
        trajectory = self._plan(action, target, cancellation)
        self._run_trajectory(action, trajectory, cancellation)

    def _plan(
        self,
        action: MoveAction,
        target: CartesianPose,
        cancellation: CancellationToken,
    ):
        """Ask move_group for a trajectory without letting it execute one.

        move_group serves /move_action from a single callback group and owns a
        single TrajectoryExecutionManager, so a goal it also executes blocks
        every other goal for the whole motion -- which turns the branches of a
        `parallel` action into a queue. Planning alone is short, and the
        trajectory is then handed to the arm's own controller.
        """

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [action.position_tolerance]

        position = PositionConstraint()
        position.header.frame_id = WORLD_FRAME
        position.link_name = f"openarm_{action.arm}_ee_base_link"
        position.constraint_region.primitives = [primitive]
        position.constraint_region.primitive_poses = [self._pose_message(target)]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = WORLD_FRAME
        orientation.link_name = position.link_name
        orientation.orientation = self._pose_message(target).orientation
        orientation.absolute_x_axis_tolerance = action.orientation_tolerance
        orientation.absolute_y_axis_tolerance = action.orientation_tolerance
        orientation.absolute_z_axis_tolerance = action.orientation_tolerance
        orientation.weight = 1.0

        constraints = Constraints(name=f"skill_{action.action_id}")
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]

        goal = MoveGroup.Goal()
        goal.request.group_name = f"{action.arm}_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = action.planning_time
        goal.request.max_velocity_scaling_factor = action.velocity_scale
        goal.request.max_acceleration_scaling_factor = action.acceleration_scale
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        # Wait for acceptance even if cancellation arrives, so an accepted goal
        # can always be explicitly cancelled instead of becoming orphaned.
        goal_handle = self._wait_future(
            self.move_group.send_goal_async(goal), timeout=10.0
        )
        if not goal_handle.accepted:
            raise RuntimeError(f"MoveIt rejected {action.arm} move action")
        if cancellation.is_set():
            self._cancel_goal(goal_handle)
            raise ExecutionCancelledError("move cancelled before planning finished")
        try:
            wrapped_result = self._wait_future(
                goal_handle.get_result_async(),
                timeout=action.planning_time + 30.0,
                cancellation=cancellation,
            )
        except ExecutionCancelledError:
            self._cancel_goal(goal_handle)
            raise
        error_code = wrapped_result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"MoveIt planning failed for {action.arm} arm with error {error_code}"
            )
        trajectory = wrapped_result.result.planned_trajectory.joint_trajectory
        if not trajectory.points:
            raise RuntimeError(
                f"MoveIt returned an empty trajectory for the {action.arm} arm"
            )
        return trajectory

    def _run_trajectory(
        self,
        action: MoveAction,
        trajectory,
        cancellation: CancellationToken,
    ) -> None:
        points = trajectory_points(trajectory)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        client = self.arm_controllers[action.arm]

        with self.dispatch_lock:
            cancellation.raise_if_cancelled()
            self._reject_arm_collisions(action.arm, trajectory, points, cancellation)
            goal_handle = self._wait_future(client.send_goal_async(goal), timeout=5.0)
            if not goal_handle.accepted:
                raise RuntimeError(
                    f"{action.arm} arm trajectory controller rejected the goal"
                )
            self.active_motions[action.arm] = ActiveMotion(
                started_at=time.monotonic(),
                joint_names=list(trajectory.joint_names),
                points=points,
            )
        try:
            if cancellation.is_set():
                self._cancel_goal(goal_handle)
                raise ExecutionCancelledError("move cancelled before execution")
            try:
                wrapped_result = self._wait_future(
                    goal_handle.get_result_async(),
                    timeout=duration(points) + 30.0,
                    cancellation=cancellation,
                )
            except ExecutionCancelledError:
                self._cancel_goal(goal_handle)
                raise
        finally:
            with self.dispatch_lock:
                self.active_motions.pop(action.arm, None)
        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"{action.arm} arm trajectory failed with error "
                f"{result.error_code}: {result.error_string}"
            )

    def _reject_arm_collisions(
        self,
        arm: str,
        trajectory,
        points: list[TrajectoryPoint],
        cancellation: CancellationToken,
    ) -> None:
        """Refuse a plan that would hit an arm already in motion.

        Each plan was made against a scene holding the other arm still at its
        start pose, so nothing in MoveIt compares the two once they run at the
        same time. The pair is sampled on a shared clock instead and every
        combined state is put to /check_state_validity.
        """

        now = time.monotonic()
        for other_arm, motion in self.active_motions.items():
            if other_arm == arm:
                continue
            offset = now - motion.started_at
            joint_names = list(trajectory.joint_names) + motion.joint_names
            for when in sample_times(duration(points)):
                cancellation.raise_if_cancelled()
                positions = list(sample_at(points, when))
                positions.extend(sample_at(motion.points, offset + when))
                self._assert_state_valid(
                    joint_names, positions, arm, other_arm, when
                )

    def _assert_state_valid(
        self,
        joint_names: list[str],
        positions: list[float],
        arm: str,
        other_arm: str,
        when: float,
    ) -> None:
        request = GetStateValidity.Request()
        # An empty group name checks the whole robot, which is the only way to
        # see a left-against-right contact; a per-group check only reports the
        # group colliding with the scene as it stood when planning started.
        request.group_name = ""
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = joint_names
        request.robot_state.joint_state.position = positions
        # A sweep is dozens of round trips and the second arm cannot start
        # until it ends, so poll far tighter than the default here.
        response = self._wait_future(
            self.state_validity.call_async(request), timeout=5.0, poll=0.005
        )
        if response.valid:
            return
        contact = ""
        if response.contacts:
            first = response.contacts[0]
            contact = f" ({first.contact_body_1} against {first.contact_body_2})"
        raise RuntimeError(
            f"{arm} arm trajectory collides with the moving {other_arm} arm "
            f"{when:.2f} s after its start{contact}"
        )

    def _cancel_goal(self, goal_handle) -> None:
        try:
            self._wait_future(goal_handle.cancel_goal_async(), timeout=5.0)
        except Exception as exc:
            if self.node is not None:
                self.node.get_logger().error(f"failed to cancel ROS goal: {exc}")

    def gripper(
        self, action: GripperAction, cancellation: CancellationToken
    ) -> None:
        if not self.ready:
            raise RuntimeError("ROS runtime is not ready")
        if action.max_effort > 0.0:
            raise RuntimeError(
                "max_effort is not supported by the current gripper "
                "JointTrajectoryController"
            )
        client = self.grippers[action.arm]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f"openarm_{action.arm}_finger_joint1"]
        point = JointTrajectoryPoint()
        point.positions = [opening_to_joint_position(action.arm, action.opening)]
        point.time_from_start = Duration(seconds=min(1.0, action.timeout)).to_msg()
        goal.trajectory.points = [point]
        goal_handle = self._wait_future(client.send_goal_async(goal), timeout=5.0)
        if not goal_handle.accepted:
            raise RuntimeError(f"{action.arm} gripper rejected the goal")
        if cancellation.is_set():
            self._cancel_goal(goal_handle)
            raise ExecutionCancelledError("gripper action cancelled")
        try:
            wrapped_result = self._wait_future(
                goal_handle.get_result_async(),
                timeout=action.timeout,
                cancellation=cancellation,
            )
        except ExecutionCancelledError:
            self._cancel_goal(goal_handle)
            raise
        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"{action.arm} gripper trajectory failed with error "
                f"{result.error_code}: {result.error_string}"
            )
