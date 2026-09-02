import math

from openarm_skills.gripper import GRIPPER_OPEN_ANGLE, opening_to_joint_position


def test_normalized_opening_uses_mirrored_joint_directions() -> None:
    assert opening_to_joint_position("left", 0.0) == 0.0
    assert opening_to_joint_position("right", 0.0) == 0.0
    assert math.isclose(opening_to_joint_position("left", 1.0), GRIPPER_OPEN_ANGLE)
    assert math.isclose(opening_to_joint_position("right", 1.0), -GRIPPER_OPEN_ANGLE)
    assert math.isclose(
        opening_to_joint_position("right", 0.5), -GRIPPER_OPEN_ANGLE / 2
    )
