import json
from pathlib import Path

from data_collection.fr3_ik.a3_replay import PoseReferenceShaper, replay, replay_bilateral
import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]


def test_pose_reference_shaper_smooths_position_and_orientation():
    shaper = PoseReferenceShaper(25.0)
    shaper.reset(np.zeros(3), Rotation.identity())
    target = np.eye(4)
    target[:3, 3] = (0.1, 0.0, 0.0)
    target[:3, :3] = Rotation.from_euler('z', 0.5).as_matrix()

    shaped = shaper.update(target, 1 / 30)

    assert 0.0 < shaped[0, 3] < target[0, 3]
    assert 0.0 < Rotation.from_matrix(shaped[:3, :3]).magnitude() < 0.5


def test_test_recording_strict_replay_holds_without_clutch(tmp_path):
    recording = ROOT/'recordings/quest_raw/test-20261020.jsonl'
    snapshot = ROOT/'recordings/robot_state/initial-20261021-103516/snapshot.json'
    result = replay(recording, snapshot, tmp_path/'first')
    repeated = replay(recording, snapshot, tmp_path/'repeat')

    assert result['acceptance'] == 'PASS'
    assert result['selected_pose_count'] == 729
    assert result['ignored_pose_count'] == 729
    assert result['clutch_true_frames'] == 0
    assert result['state_counts'] == {'HOLD': 729}
    assert result['command_count'] == 0
    assert result['trajectory_sha256'] == repeated['trajectory_sha256']
    rows = [json.loads(line) for line in (tmp_path/'first/samples.jsonl').read_text().splitlines()]
    assert all(not row['command_emitted'] and row['new_command_rad'] is None for row in rows)
    assert all(row['q_safe_rad'] == result['initial_joints_rad'] for row in rows)


def test_stage7_dynamic_limit_fault_latches_without_further_commands(tmp_path):
    recording = ROOT/'recordings/quest_raw/stage7-openvr-guided-20261020.jsonl'
    snapshot = ROOT/'recordings/robot_state/initial-20261021-103516/snapshot.json'
    result = replay(recording, snapshot, tmp_path/'stage7')

    assert result['acceptance'] == 'FAIL'
    assert result['clutch_true_frames'] == 16
    assert result['command_count'] > 0
    assert result['reason_counts']['acceleration_limit'] > 0
    rows = [json.loads(line) for line in (tmp_path/'stage7/samples.jsonl').read_text().splitlines()]
    fault_rows = [row for row in rows if row['ik_state'] == 'FAULT']
    assert fault_rows
    assert all(not row['command_emitted'] for row in fault_rows)


def test_test_recording_bilateral_replay_holds_both_arms(tmp_path):
    recording = ROOT/'recordings/quest_raw/test-20261020.jsonl'
    snapshot = ROOT/'recordings/robot_state/initial-20261021-103516/snapshot.json'
    result = replay_bilateral(recording, snapshot, tmp_path/'bilateral')

    assert result['acceptance'] == 'PASS'
    assert result['selected_pose_count'] == {'left': 729, 'right': 729}
    assert result['clutch_true_frames'] == {'left': 0, 'right': 0}
    assert result['timeline_events'] == 1458
    assert all(result['arms'][side]['command_count'] == 0 for side in ('left', 'right'))
    report = (tmp_path/'bilateral/report.html').read_text()
    assert '左臂' in report and '右臂' in report and '共同时间轴状态' in report
