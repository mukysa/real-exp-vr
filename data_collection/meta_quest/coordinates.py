"""Explicit coordinate transforms for Meta Quest target previews.

The controller APK reports Unity tracking-frame poses.  Unity uses a
left-handed basis (x right, y up, z forward), while the Franka base convention
used by this project is right-handed (x forward, y left, z up).  Rotations are
changed between bases with ``B R B^T``; quaternion component shuffling is not a
valid substitute for that operation.
"""

from __future__ import annotations

import math
from typing import Sequence

from .calibration import RigidPose, compose


Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]

# p_robot = (p_unity.z, -p_unity.x, p_unity.y)
UNITY_TO_ROBOT_BASIS: Matrix3 = (
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)
IDENTITY_POSE = RigidPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def _matrix3(values: Sequence[Sequence[float]]) -> Matrix3:
    matrix = tuple(tuple(float(value) for value in row) for row in values)
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("basis must be a 3x3 matrix")
    if not all(math.isfinite(value) for row in matrix for value in row):
        raise ValueError("basis must contain finite values")
    result: Matrix3 = matrix  # type: ignore[assignment]
    product = _matrix_multiply(result, _transpose(result))
    for row in range(3):
        for column in range(3):
            expected = 1.0 if row == column else 0.0
            if not math.isclose(product[row][column], expected, abs_tol=1e-9):
                raise ValueError("basis must be orthonormal")
    return result


def _transpose(matrix: Matrix3) -> Matrix3:
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))  # type: ignore[return-value]


def _matrix_multiply(first: Matrix3, second: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(first[row][index] * second[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def transform_vector(vector: Sequence[float], basis: Matrix3 = UNITY_TO_ROBOT_BASIS) -> tuple[float, float, float]:
    """Express a three-vector in ``basis`` coordinates."""

    converted_basis = _matrix3(basis)
    converted_vector = tuple(float(value) for value in vector)
    if len(converted_vector) != 3 or not all(math.isfinite(value) for value in converted_vector):
        raise ValueError("vector must contain three finite values")
    return tuple(
        sum(converted_basis[row][column] * converted_vector[column] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def quaternion_to_matrix(quaternion_xyzw: Sequence[float]) -> Matrix3:
    """Convert an xyzw quaternion to a proper rotation matrix."""

    values = tuple(float(value) for value in quaternion_xyzw)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-8:
        raise ValueError("quaternion has zero length")
    x, y, z, w = (value / norm for value in values)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def matrix_to_quaternion(matrix: Sequence[Sequence[float]]) -> tuple[float, float, float, float]:
    """Convert a proper rotation matrix to a normalized xyzw quaternion."""

    rotation = _matrix3(matrix)
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        x = (rotation[2][1] - rotation[1][2]) / scale
        y = (rotation[0][2] - rotation[2][0]) / scale
        z = (rotation[1][0] - rotation[0][1]) / scale
        w = 0.25 * scale
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
        x = 0.25 * scale
        y = (rotation[0][1] + rotation[1][0]) / scale
        z = (rotation[0][2] + rotation[2][0]) / scale
        w = (rotation[2][1] - rotation[1][2]) / scale
    elif rotation[1][1] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
        x = (rotation[0][1] + rotation[1][0]) / scale
        y = 0.25 * scale
        z = (rotation[1][2] + rotation[2][1]) / scale
        w = (rotation[0][2] - rotation[2][0]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
        x = (rotation[0][2] + rotation[2][0]) / scale
        y = (rotation[1][2] + rotation[2][1]) / scale
        z = 0.25 * scale
        w = (rotation[1][0] - rotation[0][1]) / scale
    result = RigidPose((0.0, 0.0, 0.0), (x, y, z, w)).quaternion_xyzw
    # q and -q are equivalent.  A stable sign makes deterministic replay files
    # byte-for-byte comparable.
    return tuple(-value for value in result) if result[3] < 0.0 else result  # type: ignore[return-value]


def change_rotation_basis(
    quaternion_xyzw: Sequence[float],
    basis: Matrix3 = UNITY_TO_ROBOT_BASIS,
) -> tuple[float, float, float, float]:
    """Change a rotation's coordinate basis, including handedness changes."""

    converted_basis = _matrix3(basis)
    source_rotation = quaternion_to_matrix(quaternion_xyzw)
    converted = _matrix_multiply(
        _matrix_multiply(converted_basis, source_rotation),
        _transpose(converted_basis),
    )
    return matrix_to_quaternion(converted)


def unity_to_robot_pose(
    pose: RigidPose,
    *,
    position_scale: float = 1.0,
    controller_to_tool: RigidPose = IDENTITY_POSE,
) -> RigidPose:
    """Convert one Unity pose and optionally apply a fixed tool transform."""

    if not math.isfinite(position_scale) or position_scale <= 0.0:
        raise ValueError("position_scale must be positive and finite")
    position = transform_vector(pose.position)
    converted = RigidPose(
        tuple(position_scale * value for value in position),
        change_rotation_basis(pose.quaternion_xyzw),
    )
    return compose(converted, controller_to_tool)
