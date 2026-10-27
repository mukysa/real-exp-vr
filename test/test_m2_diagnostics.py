import copy

import numpy as np
import pytest

from data_collection.diagnostics.samples import SampleAudit
from data_collection.lerobot_collection import bridge_packet_readiness
from utils.fr3_kinematics import TARGET_EE_SOURCE_PAIRED_JOINT_FK


def packet(stamp=100.):
    pose = [0., 0., 0.5, 1., 0., 0., 0., 1., 0.]
    return dict(
        teleop_source='quest', arm_mode='left', include_right_arm=False,
        include_gripper=False, include_hand=False, state_action_mode='joint',
        arm_action_representation='delta_joint_position',
        target_ee_source=TARGET_EE_SOURCE_PAIRED_JOINT_FK,
        state=[0.] * 7, action=[0.] * 7, joint_state=[0.] * 7, target_joint=[0.] * 7,
        robot_state_dim=7, action_dim=7, joint_state_dim=7, target_joint_dim=7,
        ee_pose=pose, target_ee_pose=pose, delta_ee_pose=[0.] * 6,
        camera_names=['cam_left'],
        cameras={name: dict(rgb=np.zeros((2, 3, 3), dtype=np.uint8), shape=[2, 3, 3], stamp_s=stamp)
                 for name in ('cam_left',)},
        bridge_publish_s=stamp + 0.01,
        action_freshness={'left': {'stamp_s': stamp}},
        robot_state_freshness={'left_arm': {'stamp_s': stamp}},
    )


def test_static_target_with_new_stamps_is_not_stale():
    audit = SampleAudit(bridge_packet_readiness)
    for i in range(10):
        audit.add(packet(100 + i / 15), i / 15, 100.02 + i / 15)
    result = audit.report()
    assert result['valid'] == 10
    assert result['action_stamp_updates_observed'] == 10
    assert result['target_value_changes_observed'] == 0
    assert result['observed_rate_hz'] == pytest.approx(15)


def test_quest_left_gripper_sample_is_accepted():
    value = packet()
    value['include_gripper'] = True
    for key in ('state', 'action', 'joint_state', 'target_joint'):
        value[key].append(1.0)
    value['robot_state_dim'] = 8
    value['action_dim'] = 8
    value['joint_state_dim'] = 8
    value['target_joint_dim'] = 8
    value['gripper_action_representation'] = 'absolute_width'
    audit = SampleAudit(bridge_packet_readiness)
    audit.add(value, 0, 100.02)
    result = audit.report()
    assert result['valid'] == 1, result['errors']


@pytest.mark.parametrize('failure', ['stale', 'future', 'missing_camera', 'nan', 'wrong_source', 'backlog', 'extra_camera'])
def test_bad_samples_are_reported(failure):
    value = copy.deepcopy(packet())
    if failure == 'stale':
        value['action_freshness']['left']['stamp_s'] -= 2
    elif failure == 'future':
        value['action_freshness']['left']['stamp_s'] += 2
    elif failure == 'missing_camera':
        del value['cameras']['cam_left']
    elif failure == 'nan':
        value['state'][0] = float('nan')
    elif failure == 'wrong_source':
        value['teleop_source'] = 'gello'
    elif failure == 'extra_camera':
        value['camera_names'].append('cam_right')
        value['cameras']['cam_right'] = copy.deepcopy(value['cameras']['cam_left'])
    audit = SampleAudit(bridge_packet_readiness)
    audit.add(value, 1, 102 if failure == 'backlog' else 100.02)
    assert audit.report()['invalid'] == 1
    assert audit.report()['errors']


def test_reused_stamp_count_and_regression():
    audit = SampleAudit(bridge_packet_readiness)
    audit.add(packet(), 0, 100.02)
    audit.add(packet(), 0.1, 100.12)
    audit.add(packet(99.9), 0.2, 100.22)
    assert audit.report()['action_stamp_updates_observed'] == 1
    assert audit.report()['invalid'] == 1
