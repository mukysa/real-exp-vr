"""Slow saves keep control on the socket owner and cannot replay buttons."""
from threading import Event, get_ident
from types import SimpleNamespace
import time

import pytest
import zmq

from data_collection import lerobot_collection as recorder
from data_collection.meta_quest.record_control import RecorderControl


def control():
    class Socket:
        def __init__(self): self.sent = []
        def setsockopt(self, *args): pass
        def connect(self, *args): pass
        def close(self, *args): pass
        def send_json(self, payload, **kwargs): self.sent.append(dict(payload))
        def recv_json(self, **kwargs): raise zmq.Again()
    return RecorderControl(SimpleNamespace(socket=lambda *args: Socket()), 'inproc://slow-save')


def event(ctl):
    return {'session': 'test', 'sequence': 1, 'operation': 'toggle',
            'recorder_id': ctl.recorder_id, 'revision': ctl.revision,
            'expires_at': time.time() + .9}


def test_slow_save_keeps_heartbeat_and_rejects_buttons_on_owner_thread():
    ctl = control()
    ctl.state = 'recording'
    queued = event(ctl)
    ctl.begin('e')
    owner = get_ident()
    started = Event()
    calls, samples, workers, pulses = [], [], [], []
    original_publish = ctl.publish

    def publish(**kwargs):
        calls.append(get_ident())
        before = len(ctl.socket.sent)
        original_publish(**kwargs)
        if len(ctl.socket.sent) > before:
            pulses.append(time.monotonic())

    def drain():
        calls.append(get_ident())
        if queued:
            item = dict(queued)
            queued.clear()
            return [item]
        return []

    def save():
        workers.append(get_ident())
        started.set()
        time.sleep(1.2)  # Longer than the relay's 1 second timeout.

    def receive(**kwargs):
        samples.append(get_ident())
        assert kwargs == {'flags': zmq.NOBLOCK}
        raise zmq.Again()

    ctl.publish, ctl.drain = publish, drain
    start = time.monotonic()
    recorder.save_episode_responsive(SimpleNamespace(save_episode=save), ctl,
                                     SimpleNamespace(recv_pyobj=receive))
    assert started.is_set() and workers[0] != owner
    assert set(calls + samples) == {owner}
    assert len(pulses) >= 5
    assert max(b - a for a, b in zip([start] + pulses, pulses + [time.monotonic()])) < .8
    assert all(s['state'] == 'saving' for s in ctl.socket.sent)
    assert any(s['result'] == 'rejected' for s in ctl.socket.sent)
    assert not queued
    ctl.close()


def test_failed_save_is_propagated_without_claiming_completion():
    ctl = control()
    ctl.begin('e')
    def fail():
        raise OSError('disk full')
    with pytest.raises(OSError, match='disk full'):
        recorder.save_episode_responsive(SimpleNamespace(save_episode=fail), ctl)
    assert ctl.state == 'saving'
    ctl.close()


def test_keyboard_interrupt_waits_for_save_before_dataset_can_be_finalized():
    ctl = control()
    ctl.begin('e')
    finished = Event()
    def save():
        time.sleep(.08)
        finished.set()
    def interrupt():
        ctl.drain = lambda: []
        raise KeyboardInterrupt
    ctl.drain = interrupt
    with pytest.raises(KeyboardInterrupt):
        recorder.save_episode_responsive(SimpleNamespace(save_episode=save), ctl)
    assert finished.is_set()
    ctl.close()


def test_legacy_save_stays_on_calling_thread():
    calls = []
    recorder.save_episode_responsive(SimpleNamespace(save_episode=lambda: calls.append(get_ident())), None)
    assert calls == [get_ident()]
