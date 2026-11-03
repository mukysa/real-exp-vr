"""Exercise the real recorder loop with fake camera/data and dataset storage."""
from argparse import Namespace
from queue import Queue
import time

import pytest
import zmq

from data_collection import lerobot_collection as recorder
from data_collection.meta_quest.record_control import RecorderControl


class DataSet:
    root = 'test-dataset'
    def __init__(self):
        self.buffer = []
        self.saved = []
        self.fail_save = False
    def has_pending_frames(self): return bool(self.buffer)
    def add_frame(self, frame): self.buffer.append(frame)
    def clear_episode_buffer(self): self.buffer.clear()
    def save_episode(self):
        if self.fail_save:
            raise OSError('disk write failed')
        self.saved.append(list(self.buffer))
        self.buffer.clear()


class ControlSocket:
    def __init__(self): self.sent = []
    def setsockopt(self, *args): pass
    def connect(self, *args): pass
    def send_json(self, payload, **kwargs): self.sent.append(dict(payload))
    def close(self, *args): pass


class ControlContext:
    def socket(self, *args): return ControlSocket()


def run_loop(monkeypatch, tmp_path, steps, *, enabled=True):
    dataset = DataSet()
    commands = Queue()
    remote = []
    ctl = RecorderControl(ControlContext(), 'inproc://test-main')
    def drain():
        events = list(remote)
        remote.clear()
        return events
    ctl.drain = drain
    ctl.source = {'hand_arm': True, 'record_session': 'session', 'state': 'FOLLOWING',
                  'right_control_fresh': True, 'generated_unix_s': time.time()}
    ctl.source_at = time.monotonic()
    request_count = 0
    def request(operation='toggle', **changes):
        nonlocal request_count
        request_count += 1
        return {'session': 'session', 'sequence': request_count, 'operation': operation,
                'expires_at': time.time() + .9, 'recorder_id': ctl.recorder_id,
                'revision': ctl.revision, **changes}
    packet = {'teleop_source': 'quest', 'arm_mode': 'left', 'include_gripper': False,
              'include_hand': False, 'task': 'test', 'robot_state_dim': 7, 'action_dim': 7}
    iterator = iter(steps)
    class DataSocket:
        def connect(self, *args): pass
        def setsockopt(self, *args): pass
        def setsockopt_string(self, *args): pass
        def close(self, *args): pass
        def recv_pyobj(self, flags=0):
            if flags:
                raise zmq.Again()
            try:
                frame, actions = next(iterator)
            except StopIteration:
                raise AssertionError('test script ended before q')
            for action in actions:
                if action in ('s', 'e', 'd', 'q'):
                    commands.put(action)
                elif action in ('toggle', 'discard'):
                    remote.append(request(action))
                elif action == 'fault':
                    ctl.source['state'] = 'FAULT'
                elif action == 'follow':
                    ctl.source['state'] = 'FOLLOWING'
                elif action == 'fail':
                    dataset.fail_save = True
                elif action == 'retry':
                    dataset.fail_save = False
                elif callable(action):
                    action(ctl, remote, request)
                else:
                    raise AssertionError(action)
            if frame:
                return dict(packet)
            raise zmq.Again()
    class DataContext:
        def socket(self, *args): return DataSocket()
        def term(self): pass
    monkeypatch.setattr(recorder, 'parse_args', lambda: Namespace(
        local_dir=str(tmp_path / 'dataset'), host='localhost', port=5555, task=None,
        hand_arm=enabled, record_control_port=5559, repo_id='local/test', fps=15,
        bridge_ready_timeout=2, video_codec='h264'))
    monkeypatch.setattr(recorder, 'normalize_episode_metadata', lambda *args: False)
    monkeypatch.setattr(recorder, 'start_command_listener', lambda: commands)
    monkeypatch.setattr(recorder.zmq, 'Context', DataContext)
    monkeypatch.setattr(recorder, 'RecorderControl', lambda *args: ctl)
    monkeypatch.setattr(recorder, 'bridge_packet_readiness', lambda packet: (True, ''))
    monkeypatch.setattr(recorder, 'make_dataset', lambda **kwargs: (dataset, [], False))
    monkeypatch.setattr(recorder, 'action_config_from_packet', lambda packet: {
        'arm_action_representation': 'delta_joint_position', 'arm_action_definition': 'test',
        'gripper_action_representation': 'none'})
    monkeypatch.setattr(recorder, 'trajectory_config_from_packet', lambda packet: {'end_effector': 'arm', 'arm_mode': 'left'})
    monkeypatch.setattr(recorder, 'packet_pair_to_frame', lambda *args: {'frame': True})
    monkeypatch.setattr(recorder, 'finalize_dataset', lambda *args: None)
    recorder.main()
    return dataset, ctl.socket.sent


def test_a_save_trigger_discard_and_keyboard_q_share_real_loop(monkeypatch, tmp_path):
    dataset, states = run_loop(monkeypatch, tmp_path, [
        (True, ['toggle']), (True, []), (True, []), (False, ['toggle']),
        (True, ['toggle']), (True, []), (False, ['discard']),
        (True, ['s']), (True, []), (False, ['q']),
    ])
    assert [len(episode) for episode in dataset.saved] == [2, 1]
    assert any(state['detail'] == 'Current episode discarded.' for state in states)
    assert any(state['state'] == 'saving' for state in states)
    assert states[-1]['state'] == 'offline'


def test_a_start_rejected_then_start_and_save_after_fault_without_camera_frames(monkeypatch, tmp_path):
    dataset, states = run_loop(monkeypatch, tmp_path, [
        (False, ['toggle']), (True, ['toggle']), (True, []),
        (False, ['fault', 'toggle']), (False, ['q']),
    ])
    assert len(dataset.saved) == 1
    assert any(state['result'] == 'rejected' for state in states)


def test_no_following_blocks_start_but_fault_does_not_block_discard(monkeypatch, tmp_path):
    dataset, states = run_loop(monkeypatch, tmp_path, [
        (True, ['fault', 'toggle']), (True, ['follow', 'toggle']), (True, []),
        (False, ['fault', 'discard']), (False, ['q']),
    ])
    assert not dataset.saved and not dataset.buffer
    assert any('not FOLLOWING' in state['detail'] for state in states)


def test_repeated_a_cannot_save_twice_or_reopen_and_empty_episode_is_not_saved(monkeypatch, tmp_path):
    dataset, states = run_loop(monkeypatch, tmp_path, [
        (True, ['toggle']), (True, []), (False, ['toggle', 'toggle']),
        (False, ['toggle']), (False, ['toggle']), (False, ['q']),
    ])
    assert len(dataset.saved) == 1
    assert any(state['result'] == 'empty' for state in states)
    assert any('stale recorder' in state['detail'] for state in states)


def test_save_error_retains_buffer_until_explicit_keyboard_discard(monkeypatch, tmp_path):
    dataset, states = run_loop(monkeypatch, tmp_path, [
        (True, ['toggle']), (True, []), (False, ['fail', 'toggle']),
        (False, ['s']), (False, ['d']), (False, ['q']),
    ])
    assert not dataset.saved and not dataset.buffer
    assert any(state['state'] == 'error' and state['result'] == 'failed' for state in states)
    assert any('retry e' in state['detail'] for state in states)


def test_legacy_keyboard_still_records_saves_discards_and_q_saves(monkeypatch, tmp_path):
    dataset, states = run_loop(monkeypatch, tmp_path, [
        (True, ['s']), (True, []), (False, ['e']),
        (True, ['s']), (True, []), (False, ['d']),
        (True, ['s']), (True, []), (False, ['q']),
    ], enabled=False)
    assert [len(episode) for episode in dataset.saved] == [1, 1]
    assert not states
