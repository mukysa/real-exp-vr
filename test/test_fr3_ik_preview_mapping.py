import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from data_collection.fr3_ik.preview_mapping import base_relative_target, world_relative_target
from data_collection.fr3_ik.mount_calibration import load_tabletop_calibration
from pathlib import Path
from data_collection.fr3_ik.a3_replay import ArmReplay, matrix_pose, pose_matrix
from data_collection.meta_quest.replay import load_recording
import hashlib

class MappingTests(unittest.TestCase):
    def test_replay_spatial_mapping_keeps_common_axes_for_both_grips(self):
        root=Path(__file__).resolve().parents[1]
        source=root/'recordings/quest_raw/stage11-openvr-guided-20261020.jsonl'
        recording=load_recording(source)
        snapshot=(root/'recordings/robot_state/initial-20261021-103516/snapshot.json').read_bytes()
        for side in ('left','right'):
            arm=ArmReplay(recording,snapshot,hashlib.sha256(source.read_bytes()).hexdigest(),side,
                root/'calibration/matrix.md',1.0,tracking_mode='dexarena_velocity',motion_mapping='tabletop-relative')
            # Actual state-machine conversion and anchor path, with deliberately
            # different wrist orientations and nontrivial robot orientations.
            source_anchor=np.eye(4)
            source_anchor[:3,:3]=Rotation.from_euler('xyz',[.7,-.5,1.2] if side=='left' else [-.6,.8,-.9]).as_matrix()
            source_anchor[:3,3]=[.2,-.3,.8]
            target=arm.world_from_base@arm.kin.fk(arm.q0)
            received=arm.poses[3]
            arm.machine._last_safe_target=matrix_pose(target)
            arm.machine._start_following(received,matrix_pose(source_anchor))
            calibration=arm.machine._calibration
            np.testing.assert_allclose(pose_matrix(calibration.apply(matrix_pose(source_anchor))),target,atol=1e-14)
            for axis in range(3):
                for sign in (-1,1):
                    moved=source_anchor.copy();moved[axis,3]+=sign*.1
                    mapped=pose_matrix(calibration.apply(matrix_pose(moved)))
                    delta=np.zeros(3);delta[axis]=sign*.1
                    np.testing.assert_allclose(mapped[:3,3]-target[:3,3],delta,atol=1e-14)
                    np.testing.assert_allclose(mapped[:3,:3],target[:3,:3],atol=1e-14)
                    rotation=Rotation.from_rotvec(np.eye(3)[axis]*sign*.2).as_matrix()
                    moved=source_anchor.copy();moved[:3,:3]=rotation@source_anchor[:3,:3]
                    mapped=pose_matrix(calibration.apply(matrix_pose(moved)))
                    np.testing.assert_allclose(mapped[:3,:3],rotation@target[:3,:3],atol=1e-14)
                    np.testing.assert_allclose(mapped[:3,3],target[:3,3],atol=1e-14)
            np.testing.assert_allclose(pose_matrix(arm.machine.source_to_target_frame),np.eye(4),atol=1e-14)

    def test_translation_independent_of_initial_grip(self):
        source=np.eye(4);source[:3,:3]=Rotation.from_euler('xyz',[.8,-.3,.7]).as_matrix()
        robot=np.eye(4);robot[:3,:3]=Rotation.from_euler('xyz',[3,.1,.2]).as_matrix()
        robot[:3,3]=[.4,0,.6]
        np.testing.assert_allclose(base_relative_target(source,source,robot),robot,atol=1e-14)
        for delta in ([.1,0,0],[0,.1,0],[0,0,-.1]):
            moved=source.copy();moved[:3,3]+=delta
            out=base_relative_target(moved,source,robot)
            np.testing.assert_allclose(out[:3,3]-robot[:3,3],delta,atol=1e-14)
    def test_spatial_rotation(self):
        source=np.eye(4);source[:3,:3]=Rotation.from_euler('xyz',[.3,.4,.5]).as_matrix()
        robot=np.eye(4);robot[:3,:3]=Rotation.from_euler('xyz',[2,.1,.2]).as_matrix()
        delta=Rotation.from_euler('z',.2).as_matrix();moved=source.copy();moved[:3,:3]=delta@source[:3,:3]
        out=base_relative_target(moved,source,robot)
        np.testing.assert_allclose(out[:3,:3],delta@robot[:3,:3],atol=1e-14)

    def test_tabletop_up_and_rotation_with_tilted_mount(self):
        mount=np.eye(4);mount[:3,:3]=Rotation.from_euler('xyz',[.8,.06,-.04]).as_matrix()
        mount[:3,3]=[-.64,-.16,.12]
        source=np.eye(4);source[:3,:3]=Rotation.from_euler('xyz',[.3,.4,.5]).as_matrix()
        anchor=np.eye(4);anchor[:3,:3]=Rotation.from_euler('xyz',[2,.1,.2]).as_matrix()
        anchor[:3,3]=[.44,.22,.14]
        np.testing.assert_allclose(world_relative_target(source,source,anchor,mount),anchor,atol=1e-14)
        moved=source.copy();moved[2,3]+=.1
        delta=Rotation.from_euler('z',.2).as_matrix();moved[:3,:3]=delta@source[:3,:3]
        target=world_relative_target(moved,source,anchor,mount)
        world_start=mount@anchor;world_end=mount@target
        np.testing.assert_allclose(world_end[:3,3]-world_start[:3,3],[0,0,.1],atol=1e-14)
        np.testing.assert_allclose(world_end[:3,:3],delta@world_start[:3,:3],atol=1e-14)
        # A vertical tabletop move requires coupled base Y/Z, rather than base Z alone.
        self.assertGreater(abs(target[1,3]-anchor[1,3]),.05)

    def test_documented_mounting_has_opposite_tilts(self):
        path=Path(__file__).resolve().parents[1]/'calibration/matrix.md'
        mounts,provenance,_=load_tabletop_calibration(path)
        for side in ('left','right'):
            self.assertGreater(provenance['tilt_from_vertical_deg'][side],35)
            self.assertLess(provenance['tilt_from_vertical_deg'][side],55)
            np.testing.assert_allclose(mounts[side][:3,:3].T@mounts[side][:3,:3],np.eye(3),atol=1e-14)
        self.assertGreater(mounts['left'][1,2],.5)
        self.assertLess(mounts['right'][1,2],-.5)

if __name__=='__main__':unittest.main()
