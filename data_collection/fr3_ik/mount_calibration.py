"""Read the documented historical tabletop calibration for offline previews."""
import ast
import hashlib
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation


def load_tabletop_calibration(path):
    path = Path(path)
    raw = path.read_bytes()
    document = raw.decode('utf-8')

    def matrix(name):
        match = re.search(r'^' + re.escape(name) + r' =\s*\n(\[\[.*?\]\])',
                          document, re.MULTILINE | re.DOTALL)
        if match is None:
            raise ValueError(f'Missing calibration matrix {name} in {path}')
        value = np.asarray(ast.literal_eval(match.group(1)), dtype=float)
        if (value.shape != (4, 4) or not np.isfinite(value).all()
                or not np.allclose(value[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
                or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-6, rtol=0)
                or not np.isclose(np.linalg.det(value[:3, :3]), 1, atol=1e-6, rtol=0)):
            raise ValueError(f'Invalid rigid calibration matrix {name}')
        # The document rounds matrix entries. Remove only that small SO(3) error.
        value[:3, :3] = Rotation.from_matrix(value[:3, :3]).as_matrix()
        return value

    world_from_camera = matrix('W_T_C')
    world_from_bases = {side: world_from_camera @ matrix('C_T_B_' + suffix)
                        for side, suffix in (('left', 'L'), ('right', 'R'))}
    provenance = dict(
        source_path=str(path.resolve()), source_sha256=hashlib.sha256(raw).hexdigest(),
        source_copy='mount_calibration.md', calibration_date='2026-08-28',
        frame_convention='A_T_B maps B coordinates into A; tabletop +X forward, +Y left, +Z up',
        assumption='Historical mounting calibration reused; no new physical calibration performed',
        rounding_handling='Documented rotation matrices projected onto SO(3) before composition',
        world_from_camera=world_from_camera.tolist(),
        world_from_bases={side: value.tolist() for side, value in world_from_bases.items()},
        tilt_from_vertical_deg={side: float(np.rad2deg(np.arccos(np.clip(value[2, 2], -1, 1))))
                                for side, value in world_from_bases.items()})
    return world_from_bases, provenance, raw
