import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from data_collection.fr3_ik.kinematics import Kinematics
from data_collection.move_to_target_ee import (
    IK_ORIENTATION_TOLERANCE_RAD,
    IK_POSITION_TOLERANCE_M,
    solve_fr3_ik,
)


@pytest.mark.parametrize('side,q', [
    ('left', (-0.0807, 0.2881, -0.8085, -2.1946, 0.8213, 1.5589, -1.7530)),
    ('right', (-0.1696, 0.6397, 0.9103, -2.1266, -1.2363, 1.5695, 0.6753)),
])
def test_live_optimizer_converges_with_unchanged_pose_verification(side, q):
    kin = Kinematics(side)
    seed = np.asarray(q)
    target = kin.fk(seed)
    target[:3, 3] += [0.02, 0.01, 0.015]
    target[:3, :3] = Rotation.from_rotvec([0.03, -0.04, 0.02]).as_matrix() @ target[:3, :3]
    result = solve_fr3_ik(
        seed, target, kin.tool, kinematics=kin,
        try_alternative_seeds=False, max_function_evaluations=120,
        optimizer_tolerance=1e-6,
    )
    assert result.function_evaluations < 20
    assert result.position_error_m <= IK_POSITION_TOLERANCE_M
    assert result.orientation_error_rad <= IK_ORIENTATION_TOLERANCE_RAD
