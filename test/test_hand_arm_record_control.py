import time

import pytest
import zmq

from data_collection.meta_quest.record_control import RecorderControl


class Socket:
    def __init__(self):
        self.incoming = []
        self.sent = []
    def setsockopt(self, *args): pass
    def connect(self, *args): pass
    def send_json(self, payload, **kwargs): self.sent.append(payload)
    def recv_json(self, **kwargs):
        if not self.incoming:
            raise zmq.Again()
        return self.incoming.pop(0)
    def close(self, *args): pass


class Context:
    def __init__(self): self.transport = Socket()
    def socket(self, *args): return self.transport


def control():
    obj = RecorderControl(Context(), endpoint="inproc://test")
    obj.source = {'hand_arm': True, 'record_session': 'session', 'state': 'FOLLOWING',
                  'right_control_fresh': True, 'generated_unix_s': time.time()}
    obj.source_at = time.monotonic()
    return obj


def event(obj, **kwargs):
    return {'session': 'session', 'sequence': 1, 'operation': 'toggle',
            'expires_at': time.time() + 0.8, 'recorder_id': obj.recorder_id,
            'revision': obj.revision, **kwargs}


def test_start_failure_does_not_turn_next_a_into_save_and_keyboard_invalidates_old_events():
    obj = control()
    first = event(obj)
    assert obj.validate(first) is None
    obj.begin('s', first)
    obj.finish('idle', 'rejected', 'no fresh sample')
    assert obj.state == 'idle'
    assert obj.validate(first) is not None
    assert obj.validate(event(obj, sequence=2)) is None
    queued = event(obj, sequence=3)
    obj.begin('d')
    obj.finish('idle', 'completed', 'keyboard discard')
    assert obj.validate(queued) is not None


def test_save_completion_cannot_replay_queued_a_or_discard():
    obj = control()
    obj.state = 'recording'
    queued_a = event(obj)
    queued_d = event(obj, sequence=2, operation='discard')
    obj.begin('e')
    assert obj.socket.sent[-1]['state'] == 'saving'
    assert obj.validate(queued_a) is not None
    obj.finish('idle', 'completed', 'saved')
    assert obj.validate(queued_d) is not None
    assert obj.validate(event(obj, sequence=3)) is None


@pytest.mark.parametrize('change', [
    {'expires_at': 0}, {'expires_at': float('nan')}, {'expires_at': time.time() + 100},
    {'session': 'old'}, {'recorder_id': 'old'}, {'sequence': 0}, {'sequence': True},
    {'operation': 'q'}, {'revision': -1},
])
def test_old_or_malformed_events_are_rejected(change):
    obj = control()
    assert obj.validate(event(obj, **change))


def test_lost_left_tracking_does_not_block_save_or_discard_but_prevents_start():
    obj = control()
    obj.source['state'] = 'FAULT'
    assert not obj.following()
    assert obj.validate(event(obj, operation='discard')) is None
    obj.source['generated_unix_s'] = time.time() - 10
    assert obj.validate(event(obj, sequence=2))
    assert not obj.following()


def test_duplicate_messages_do_not_apply_twice_and_state_messages_do_not_become_events():
    obj = control()
    request = event(obj)
    assert obj.validate(request) is None
    assert obj.validate(request) == 'duplicate event'
    obj.socket.incoming = [[], {'kind': 'source', 'status': obj.source},
                           {'kind': 'event', 'event': request}]
    assert obj.drain() == [request]
    obj.close()
    assert obj.socket.sent[-1]['state'] == 'offline'


def test_rejected_request_reports_its_own_identity_without_changing_recording_state():
    obj = control()
    obj.state = 'recording'
    stale = event(obj, sequence=42, revision=-1)
    obj.reject(obj.validate(stale), stale)
    status = obj.socket.sent[-1]
    assert status['event_id'] == ['session', 42]
    assert status['state'] == 'recording' and status['result'] == 'rejected'
    assert obj.revision == 0


def test_only_one_local_recorder_can_own_the_port(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    first = RecorderControl(Context())
    try:
        with pytest.raises(RuntimeError, match='already owns'):
            RecorderControl(Context())
    finally:
        first.close()
    replacement = RecorderControl(Context())
    replacement.close()
