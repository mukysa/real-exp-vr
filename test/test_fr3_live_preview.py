"""No devices: exercise takeover, independent watchdog and reference bounds."""
import json
from pathlib import Path
import numpy as np
import pytest
from data_collection.fr3_ik.live.engine import PreviewEngine,SIDES
from data_collection.meta_quest.protocol import PoseSample
from data_collection.meta_quest.receiver import ReceivedPose
ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def rig():
    e=PreviewEngine(ROOT/'calibration/matrix.md')
    qs=json.loads((ROOT/'recordings/robot_state/initial-20261021-103516/snapshot.json').read_text())['arms']
    seq=0
    def feed(t,b=False,offset=0,connection=1,tracked=True):
        nonlocal seq
        for side in SIDES:
            e.update_feedback(side,qs[side]['q_rad'],t,'MOCK')
            seq+=1
            e.pose(ReceivedPose(PoseSample(side,'controller',(0,1,offset),(0,0,0,1),tracked,clutch=b if side=='right' else False),t,seq,connection),t)
    return e,feed,qs

def test_takeover_first_frame_and_stationary(rig):
    e,feed,qs=rig;engage(e,feed)
    assert e.active
    for s in SIDES:np.testing.assert_array_equal(e.reference[s].position,qs[s]['q_rad'])
    for i in range(1,30):feed(1.02+i*.02);e.tick(1.02+i*.02)
    for s in SIDES:np.testing.assert_allclose(e.reference[s].position,qs[s]['q_rad'],atol=1e-10)

def engage(e,feed):
    feed(1);e.tick(1)
    feed(1.02,True);e.tick(1.02)
    assert e.active

def test_move_stop_brake_reanchor(rig):
    e,feed,qs=rig;engage(e,feed)
    for i in range(1,31):
        t=1.02+i*.02;feed(t,offset=i*.003)
        q={s:e.reference[s].position.copy() for s in SIDES};v={s:e.reference[s].velocity.copy() for s in SIDES};e.tick(t)
        for s in SIDES:
            r=e.reference[s];assert np.all(abs(r.velocity)<=np.asarray(e.limits.velocity)+1e-8)
            assert np.all(abs(r.velocity-v[s])<=np.asarray(e.limits.acceleration)*.02+1e-8)
            assert np.all(abs(r.position-q[s])<=e.limits.max_step_rad+1e-8)
    assert any(np.linalg.norm(e.reference[s].position-qs[s]['q_rad'])>.001 for s in SIDES)
    feed(1.64,True,offset=.09);e.tick(1.64);assert not e.active
    # Any attempt before braking finishes must be rejected by ready().
    if any(not r.stopped for r in e.reference.values()):assert not e.ready(1.64)
    for i in range(1,201):feed(1.64+i*.02,offset=.09);e.tick(1.64+i*.02)
    qs['left']['q_rad'][0]+=.01
    feed(5.66,True,offset=.09);e.tick(5.66);assert e.active
    np.testing.assert_array_equal(e.reference['left'].position,qs['left']['q_rad'])

@pytest.mark.parametrize('kind',['all','quest','feedback'])
def test_watchdog_without_input_events(rig,kind):
    e,feed,qs=rig;engage(e,feed)
    for i in range(1,20):
        t=1.02+i*.02
        if kind=='quest':
            for s in SIDES:e.update_feedback(s,qs[s]['q_rad'],t,'MOCK')
        elif kind=='feedback':
            for j,s in enumerate(SIDES):e.pose(ReceivedPose(PoseSample(s,'controller',(0,1,0),(0,0,0,1),True,clutch=False),t,100+i*2+j,1),t)
        e.tick(t)
    assert not e.active

@pytest.mark.parametrize('failure',['jump','reconnect','tracking'])
def test_fault_frame_cannot_reengage(rig,failure):
    e,feed,qs=rig;engage(e,feed)
    feed(1.04);e.tick(1.04)
    feed(1.22,True,offset=.3 if failure=='jump' else 0,connection=2 if failure=='reconnect' else 1,tracked=failure!='tracking')
    assert not e.active
    feed(1.24,True,offset=.3 if failure=='jump' else 0,connection=2 if failure=='reconnect' else 1)
    assert not e.active

def test_held_button_at_start_and_left_button_do_not_engage(rig):
    e,feed,qs=rig
    feed(1,True);e.tick(1);assert not e.active
    feed(1.02,True);e.tick(1.02);assert not e.active
    feed(1.04);e.tick(1.04)
    e.pose(ReceivedPose(PoseSample('left','controller',(0,1,0),(0,0,0,1),True,clutch=True),1.06,99,1),1.06)
    assert not e.active
    feed(1.08,True)  # left sequence is intentionally old, causing a stop
    assert not e.active

def test_invalid_feedback_is_rejected(rig):
    e,feed,qs=rig
    for q in ([float('nan')]*7,[0]*6,[100]*7):
        with pytest.raises(ValueError):e.update_feedback('left',q,1,'MOCK')

def test_receiver_arrival_during_queue_drain_stays_engaged(rig):
    """A receiver can timestamp a sample after loop start, before queue drain."""
    e,feed,qs=rig;engage(e,feed)
    for i in range(1,101):
        loop_start=1.02+i*.02
        arrival=loop_start+.002
        # Production must sample its processing/tick time after dequeue,
        # rather than passing loop_start as the evaluation time.
        feed(arrival)
        e.tick(arrival+.001)
        assert e.active

def test_smooth_return_locks_b_and_obeys_sampled_bounds(rig):
    e,feed,qs=rig;engage(e,feed)
    for i in range(1,101):
        t=1.02+i*.02;feed(t,offset=min(i*.002,.15));e.tick(t)
    home={s:e.home[s].copy() for s in SIDES}
    feed(3.04,True,offset=.15);e.tick(3.04)
    assert not e.active and e.return_phase is not None
    saw_return=False;complete=False
    for i in range(1,501):
        t=3.04+i*.02
        previous={s:(r.position.copy(),r.velocity.copy()) for s,r in e.reference.items()}
        # Attempt repeated engagement throughout braking and return.
        feed(t,b=(i//10)%2==1,offset=.15)
        if not complete:assert not e.active
        e.tick(t)
        for s,r in e.reference.items():
            q,v=previous[s]
            np.testing.assert_allclose((r.position-q)/.02,r.velocity,atol=1e-8)
            assert np.all(abs(r.velocity)<=np.asarray(e.limits.velocity)+1e-8)
            assert np.all(abs(r.velocity-v)<=np.asarray(e.limits.acceleration)*.02+1e-8)
            assert np.all(r.position>=e.limits.lower) and np.all(r.position<=e.limits.upper)
        saw_return|=e.status=='RETURNING'
        if e.return_phase is None:
            complete=True
            for s in SIDES:np.testing.assert_allclose(e.reference[s].position,home[s],atol=1e-12)
            break
    assert saw_return and complete
    # Completion does not turn a held B into an engagement.
    feed(t+.02,True,offset=.15);e.tick(t+.02);assert not e.active
    feed(t+.04,False,offset=.15);e.tick(t+.04)
    for j in range(3,12):feed(t+j*.02,False,offset=.15);e.tick(t+j*.02)
    feed(t+.24,True,offset=.15);e.tick(t+.24)
    assert e.active
    for s in SIDES:np.testing.assert_array_equal(e.reference[s].position,qs[s]['q_rad'])
    assert any(ev['action']=='home_complete' for ev in e.events)


def test_single_left_uses_x_half_scale_and_returns_after_stop():
    e=PreviewEngine(ROOT/'calibration/matrix.md',single_left=True,position_scale=.5)
    snapshot=json.loads((ROOT/'recordings/robot_state/initial-20261021-103516/snapshot.json').read_text())
    q=snapshot['arms']['left']['q_rad']
    def send(t,offset=0,button=False,seq=0):
        e.update_feedback('left',q,t,'MOCK')
        sample=PoseSample('left','controller',(0,1,offset),(0,0,0,1),True,clutch=button)
        e.pose(ReceivedPose(sample,t,seq or int(t*1000)+1,1),t)
    send(1.0,seq=1);e.tick(1.0)
    send(1.02,button=True,seq=2);e.tick(1.02)
    assert e.active and e.sides==('left',)
    np.testing.assert_array_equal(e.reference['left'].position,q)
    anchor=e.robot_anchor['left'][:3,3].copy()
    send(1.04,offset=.01,seq=3);e.tick(1.04)
    assert np.linalg.norm(e.desired['left'][:3,3]-anchor) <= .005+1e-9
    assert 'right' not in e.feedback and 'right' not in e.kin
    send(1.12,offset=.01,button=False,seq=4);e.tick(1.12)
    send(1.22,offset=.01,button=True,seq=5);e.tick(1.22)
    assert not e.active and e.return_phase in {'BRAKING','RETURNING'}


def test_single_left_x_smoothly_returns_locks_button_and_reengages(single_rig):
    e,feed,q=single_rig;engage(e,feed)
    for i in range(1,101):
        t=1.02+i*.02;feed(t,offset=min(i*.002,.15));e.tick(t)
    home=e.home['left'].copy()
    feed(3.04,True,offset=.15);e.tick(3.04)
    assert not e.active and e.return_phase in {'BRAKING','RETURNING'}
    saw_return=False;complete=False
    for i in range(1,501):
        t=3.04+i*.02
        previous=e.reference['left'].position.copy();velocity=e.reference['left'].velocity.copy()
        feed(t,True,offset=.15);e.tick(t)
        assert not e.active
        ref=e.reference['left']
        np.testing.assert_allclose((ref.position-previous)/.02,ref.velocity,atol=1e-8)
        assert np.all(abs(ref.velocity)<=np.asarray(e.limits.velocity)+1e-8)
        assert np.all(abs(ref.velocity-velocity)<=np.asarray(e.limits.acceleration)*.02+1e-8)
        assert np.all(ref.position>=e.limits.lower) and np.all(ref.position<=e.limits.upper)
        saw_return|=e.status=='RETURNING'
        if e.return_phase is None:
            complete=True
            np.testing.assert_allclose(ref.position,home,atol=1e-12)
            break
    assert saw_return and complete and e.reason=='virtual return complete; release Y then press to engage'
    feed(t+.02,True,offset=.15);e.tick(t+.02);assert not e.active
    feed(t+.04,False,offset=.15);e.tick(t+.04)
    for j in range(3,12):feed(t+j*.02,False,offset=.15);e.tick(t+j*.02)
    feed(t+.24,True,offset=.15);e.tick(t+.24)
    assert e.active
    np.testing.assert_array_equal(e.reference['left'].position,q)


@pytest.fixture
def single_rig():
    e=PreviewEngine(ROOT/'calibration/matrix.md',single_left=True)
    q=json.loads((ROOT/'recordings/robot_state/initial-20261021-103516/snapshot.json').read_text())['arms']['left']['q_rad']
    seq=0
    def feed(t,b=False,offset=0,connection=1,tracked=True,rotation=(0,0,0,1)):
        nonlocal seq
        seq+=1
        e.update_feedback('left',q,t,'MOCK')
        e.pose(ReceivedPose(PoseSample('left','controller',(0,1,offset),rotation,tracked,clutch=b),t,seq,connection),t)
    return e,feed,q


def test_single_left_target_converges_without_accumulation_or_rotation(single_rig):
    e,feed,q=single_rig;engage(e,feed)
    anchor=e.robot_anchor['left'].copy()
    for i in range(1,101):
        t=1.02+i*.02
        feed(t,offset=.01,rotation=(0,0,.1,np.sqrt(.99)));e.tick(t)
        target=e.state(t)['arms']['left']['targets']
        assert np.linalg.norm(target['scaled_translation_delta_m'])==pytest.approx(.005)
        assert target['accepted_q'] is None
        np.testing.assert_allclose(e.desired['left'][:3,:3],anchor[:3,:3],atol=1e-12)
    assert np.linalg.norm(e.desired['left'][:3,3]-anchor[:3,3])==pytest.approx(.005,abs=1e-8)
    assert set(e.state(t)['arms'])=={'left'}


@pytest.mark.parametrize('failure',['stale','jump','reconnect','tracking'])
def test_single_left_fault_holds_until_fresh_y_edge(single_rig,failure):
    e,feed,q=single_rig;engage(e,feed)
    feed(1.04);e.tick(1.04)
    offset=.3 if failure=='jump' else 0
    connection=2 if failure=='reconnect' else 1
    if failure=='stale':e.tick(1.4)
    else:
        feed(1.22,True,offset,connection,tracked=failure!='tracking');e.tick(1.22)
    assert not e.active and e.return_phase is None
    for i in range(21):
        t=1.42+i*.02;feed(t,True,offset,connection);e.tick(t)
        assert not e.active and e.return_phase is None
    feed(1.86,False,offset,connection);e.tick(1.86)
    for i in range(1,10):
        t=1.86+i*.02;feed(t,False,offset,connection);e.tick(t)
    feed(2.06,True,offset,connection);e.tick(2.06)
    assert e.active
    np.testing.assert_array_equal(e.reference['left'].position,q)


def test_single_left_ignores_right_input(single_rig):
    e,feed,q=single_rig
    feed(1);e.tick(1)
    e.pose(ReceivedPose(PoseSample('right','controller',(0,1,0),(0,0,0,1),True,clutch=True),1.02,99,1),1.02)
    assert not e.active and 'right' not in e.poses


def test_ready_reason_updates_when_left_input_arrives(single_rig):
    e,feed,q=single_rig
    e.update_feedback('left',q,.98,'MOCK');e.tick(.98)
    assert e.status=='WAITING'
    feed(1);e.tick(1)
    assert e.status=='READY' and e.reason=='left input and feedback ready; press Y to engage'
    for i in range(1,17):e.tick(1+i*.02)
    assert e.status=='WAITING' and e.reason=='waiting for left input and feedback'


def test_openvr_left_y_through_wire_protocol_engages_and_stops(single_rig):
    from types import SimpleNamespace
    from data_collection.meta_quest.openvr_bridge import _controller_controls
    from data_collection.meta_quest.steamvr_bridge import compact_controller_line
    from data_collection.meta_quest.calibration import RigidPose
    from data_collection.meta_quest.protocol import parse_pose_line
    e,_,q=single_rig
    def send(t,mask):
        state=SimpleNamespace(ulButtonPressed=mask,rAxis=[])
        trigger,clutch,record=_controller_controls(SimpleNamespace(getControllerState=lambda _: (True,state)),1,single_left=True)
        line=compact_controller_line(side='left',pose=RigidPose((0,1,0),(0,0,0,1)),
            grasp=trigger,clutch=clutch,record=record,frame_id=int(t*1000),timestamp_ns=int(t*1e9))
        sample=parse_pose_line(line)
        e.update_feedback('left',q,t,'MOCK')
        e.pose(ReceivedPose(sample,t,int(t*1000),1),t);e.tick(t)
    send(1,0)
    send(1.02,1 << 7)  # X is not the configured single-left clutch.
    assert not e.active
    send(1.04,0)
    send(1.06,1 << 1)  # Touch left Y shares the B button bit.
    assert e.active and e.state(1.06)['arms']['left']['clutch_pressed']
    np.testing.assert_array_equal(e.reference['left'].position,q)
    for i in range(1,11):
        send(1.06+i*.02,0)
        assert e.active  # Releasing Y does not stop.
    send(1.28,0)
    send(1.30,1 << 1)
    assert not e.active and e.return_phase in {'BRAKING','RETURNING'}
