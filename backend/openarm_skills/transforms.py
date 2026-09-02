"""Small dependency-free rigid-transform helpers for TF marker targets."""

from __future__ import annotations

import math

from .models import CartesianPose, Quaternion, Vector3


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    return Quaternion(
        x=left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        y=left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        z=left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
        w=left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
    )


def quaternion_conjugate(value: Quaternion) -> Quaternion:
    return Quaternion(x=-value.x, y=-value.y, z=-value.z, w=value.w)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Build a normalized quaternion from intrinsic roll, pitch, yaw angles."""

    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def rotate_vector(rotation: Quaternion, vector: Vector3) -> Vector3:
    # Expanded q * v * conjugate(q), avoiding construction of a non-unit
    # quaternion for the intermediate pure-vector value.
    qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
    tx = 2.0 * (qy * vector.z - qz * vector.y)
    ty = 2.0 * (qz * vector.x - qx * vector.z)
    tz = 2.0 * (qx * vector.y - qy * vector.x)
    return Vector3(
        x=vector.x + qw * tx + (qy * tz - qz * ty),
        y=vector.y + qw * ty + (qz * tx - qx * tz),
        z=vector.z + qw * tz + (qx * ty - qy * tx),
    )


def compose(left: CartesianPose, right: CartesianPose) -> CartesianPose:
    """Return ``T_a_c = T_a_b * T_b_c``."""

    translated = rotate_vector(left.orientation, right.position)
    return CartesianPose(
        position=Vector3(
            x=left.position.x + translated.x,
            y=left.position.y + translated.y,
            z=left.position.z + translated.z,
        ),
        orientation=quaternion_multiply(left.orientation, right.orientation),
    )


def inverse(value: CartesianPose) -> CartesianPose:
    """Return the inverse rigid transform."""

    inverse_rotation = quaternion_conjugate(value.orientation)
    inverse_translation = rotate_vector(
        inverse_rotation,
        Vector3(
            x=-value.position.x,
            y=-value.position.y,
            z=-value.position.z,
        ),
    )
    return CartesianPose(
        position=inverse_translation,
        orientation=inverse_rotation,
    )
