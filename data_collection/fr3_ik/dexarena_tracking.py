"""Offline port of DexArena NullspaceArmIK's default kinematic step.

Source: /home/landau/yys/DexArena/data_gen/teleop/arm_ik.py.
Uses the local FR3 TCP model instead of SAPIEN's Panda model. There is no
simulated PD plant or measured feedback: commanded state is the virtual state.
The plant leash is therefore inert. This is not a hardware command generator.
"""
import numpy as np
from scipy.spatial.transform import Rotation


DEXARENA_VELOCITY_LIMITS = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])


def differential_step(kinematics, q, target, dt, lower, upper, *, null_gain=2.0,
                      limit_margin=.02, limit_velocity=True, damping_mode='fixed',
                      lam=.08, lam_max=.20, sigma_eps=.04):
    """DLS + exact null projection + uniform velocity scale + limit margin.

    Matches default lam=.08, k_null=2, rot_weight=1, max_joint_vel=3,
    null_mode=center, limit_margin=.02 and error clamps (.20 m, .80 rad).
    Feedforward is absent, matching the default runner call to step().
    """
    q = np.asarray(q, dtype=float)
    actual = kinematics.fk(q)
    error_v = np.asarray(target[:3, 3]) - actual[:3, 3]
    error_w = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
    for vector, cap in ((error_v, .20), (error_w, .80)):
        magnitude = np.linalg.norm(vector)
        if magnitude > cap:
            vector *= cap / magnitude
    jac = kinematics.tcp_jacobian(q)
    error = np.r_[error_v, error_w]
    u, singular, vt = np.linalg.svd(jac, full_matrices=False)
    if damping_mode == 'adaptive':
        lam2 = np.full_like(singular, lam ** 2)
        near = singular < sigma_eps
        lam2[near] += lam_max ** 2 * (1.0 - (singular[near] / sigma_eps) ** 2)
        task = vt.T @ ((singular / (singular ** 2 + lam2)) * (u.T @ error))
    elif damping_mode == 'fixed':
        task = jac.T @ np.linalg.solve(jac @ jac.T + lam**2 * np.eye(6), error)
    else:
        raise ValueError('damping_mode must be fixed or adaptive')
    # Use the exact truncated pseudoinverse for the null projector; damping
    # this projector would leak the centering motion into task space.
    inverse = np.zeros_like(singular)
    np.divide(1.0, singular, out=inverse,
              where=singular > max(singular[0] * 1e-3, 1e-6))
    projector = np.eye(7) - vt.T @ np.diag(inverse) @ u.T @ jac
    rest = .5 * (np.asarray(lower) + np.asarray(upper))
    null = projector @ (null_gain * dt * (rest - q))
    delta = task + null
    caps = np.minimum(3.0, DEXARENA_VELOCITY_LIMITS) * dt
    ratio = float(np.max(np.abs(delta) / caps))
    if ratio > 1 and limit_velocity:
        delta /= ratio
    proposed = q + delta
    next_q = np.clip(proposed, np.asarray(lower) + limit_margin, np.asarray(upper) - limit_margin)
    return next_q, dict(rate_limited=ratio > 1 and limit_velocity,
                        velocity_scale=1 / max(1, ratio) if limit_velocity else 1.0,
                        joint_margin_clipped=bool(np.any(next_q != proposed)),
                        sigma_min=float(singular[-1]), null_step_rad=float(np.linalg.norm(null)),
                        damping_mode=damping_mode)
