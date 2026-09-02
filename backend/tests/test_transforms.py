import math

import pytest

from openarm_skills.models import CartesianPose, Quaternion, Vector3
from openarm_skills.transforms import compose, inverse, quaternion_from_rpy


def pose(x=0.0, y=0.0, z=0.0, q=None) -> CartesianPose:
    return CartesianPose(
        position=Vector3(x=x, y=y, z=z),
        orientation=q or Quaternion(),
    )


def test_marker_save_and_run_equations_round_trip() -> None:
    world_marker_saved = pose(1.0, 2.0, 0.0)
    world_ee_saved = pose(1.2, 2.1, 0.3)

    marker_ee = compose(inverse(world_marker_saved), world_ee_saved)
    assert marker_ee.position.x == pytest.approx(0.2)
    assert marker_ee.position.y == pytest.approx(0.1)

    world_marker_current = pose(2.0, -1.0, 0.5)
    world_target = compose(world_marker_current, marker_ee)
    assert world_target.position.x == pytest.approx(2.2)
    assert world_target.position.y == pytest.approx(-0.9)
    assert world_target.position.z == pytest.approx(0.8)


def test_composition_rotates_child_translation() -> None:
    half_angle = math.pi / 4.0
    quarter_turn_z = Quaternion(z=math.sin(half_angle), w=math.cos(half_angle))
    world_marker = pose(1.0, 0.0, 0.0, quarter_turn_z)
    marker_ee = pose(1.0, 0.0, 0.0)

    world_ee = compose(world_marker, marker_ee)
    assert world_ee.position.x == pytest.approx(1.0, abs=1e-9)
    assert world_ee.position.y == pytest.approx(1.0, abs=1e-9)


def test_transform_times_inverse_is_identity() -> None:
    half_angle = math.pi / 6.0
    value = pose(
        0.4,
        -0.2,
        1.1,
        Quaternion(y=math.sin(half_angle), w=math.cos(half_angle)),
    )
    identity = compose(value, inverse(value))

    assert identity.position.x == pytest.approx(0.0, abs=1e-9)
    assert identity.position.y == pytest.approx(0.0, abs=1e-9)
    assert identity.position.z == pytest.approx(0.0, abs=1e-9)
    assert identity.orientation.w == pytest.approx(1.0, abs=1e-9)


def test_quaternion_from_rpy_is_normalized() -> None:
    quaternion = quaternion_from_rpy(0.1, -0.2, 0.3)
    norm = (
        quaternion.x**2
        + quaternion.y**2
        + quaternion.z**2
        + quaternion.w**2
    ) ** 0.5
    assert norm == pytest.approx(1.0)
