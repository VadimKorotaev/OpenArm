"""OpenArm gripper joint conventions."""

from typing import Literal


GRIPPER_OPEN_ANGLE = 0.7854


def opening_to_joint_position(
    arm: Literal["left", "right"], opening: float
) -> float:
    """Map normalized opening to the mirrored driver-joint position."""

    direction = 1.0 if arm == "left" else -1.0
    return direction * opening * GRIPPER_OPEN_ANGLE
