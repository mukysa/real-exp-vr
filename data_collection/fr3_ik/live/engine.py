"""Single-owner preview engine. All outputs are visualization-only."""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from ..a3_replay import matrix_pose, pose_matrix, PoseReferenceShaper
from ..kinematics import Kinematics
from ..mount_calibration import load_tabletop_calibration
from ..constraints import Limits
from ..dexarena_tracking import differential_step
from ..joint_reference import BoundedJointReference
from ..preview_mapping import converted_matrix
from data_collection.meta_quest.calibration import PoseCalibration
from data_collection.meta_quest.state_machine import DebouncedButtonEdge

SIDES=('left','right')

class PreviewEngine:
    def __init__(self, mount_path, stale_s=.25, *, single_left=False, position_scale=.5):
        self.mounts,_,_=load_tabletop_calibration(mount_path)
        self.sides=('left',) if single_left else SIDES
        if not np.isfinite(position_scale) or not 0 < position_scale <= 1:
            raise ValueError('position_scale must be in (0, 1]')
        self.single_left=single_left;self.position_scale=float(position_scale)
        self.kin={s:Kinematics(s) for s in self.sides};self.limits=Limits.from_controller()
        if not np.isfinite(stale_s) or stale_s<=0:raise ValueError('stale_s must be positive')
        self.stale_s=stale_s;self.feedback={};self.poses={};self.reference={};self.calibration={}
        self.shapers={s:PoseReferenceShaper(25.) for s in self.sides}
        self.button=DebouncedButtonEdge(.15);self.button_initialized=False
        self.active=False;self.status='WAITING';self.reason='waiting for left input and feedback' if single_left else 'waiting for both inputs and feedback'
        self.connections={};self.previous_source={};self.anchor_source={};self.events=[]
        self.home={};self.robot_anchor={};self.return_phase=None;self.return_elapsed=0.;self.return_duration=0.;self.return_start={}
        self.last_tick=None;self.last_sequence={};self.controller={};self.desired={};self.takeover=False

    def update_feedback(self, side, q, now, source):
        q=np.asarray(q,dtype=float)
        if side not in self.sides or q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Invalid measured joints')
        if np.any(q<self.limits.lower) or np.any(q>self.limits.upper):raise ValueError('Measured joints outside model operational limits')
        self.feedback[side]=dict(q=q.copy(),time=now,source=source)
        if side not in self.reference:self.reference[side]=BoundedJointReference(self.limits,q)

    def stop(self, reason):
        if self.active and reason in {'B stop','X stop'}:self.return_phase='BRAKING'
        elif reason not in {'B stop','X stop'}:self.return_phase=None
        self.active=False;self.reason=reason;self.status='HOLD';self.button.require_release()
        self.events.append(dict(time=time.monotonic(),action='stop',reason=reason));self.events=self.events[-100:]

    def ready(self, now):
        return self.return_phase is None and all(s in self.feedback and s in self.poses and self.poses[s].sample.tracked
            and 0<=now-self.feedback[s]['time']<=self.stale_s
            and 0<=now-self.poses[s].received_monotonic_s<=self.stale_s
            and self.reference[s].stopped for s in self.sides)

    def pose(self, received, now):
        s=received.sample.side
        if received.sample.source!='controller' or s not in self.sides:return
        old=self.poses.get(s)
        if old and (received.sequence<=old.sequence or received.received_monotonic_s<=old.received_monotonic_s):
            self.stop('out-of-order input');return
        if old and received.connection_id!=old.connection_id:
            self.stop('Quest connection changed');self.poses[s]=received;return
        source=converted_matrix(received.sample)
        if old and self.active:
            previous=converted_matrix(old.sample)
            if np.linalg.norm(source[:3,3]-previous[:3,3])>.15 or Rotation.from_matrix(source[:3,:3]@previous[:3,:3].T).magnitude()>.8:
                self.stop('Quest pose discontinuity');self.poses[s]=received;return
        self.poses[s]=received
        if not received.sample.tracked:self.stop('tracking lost');return
        if (s!='left' if self.single_left else s!='right'):return
        pressed=bool(received.sample.clutch)
        if not self.button_initialized:self.button.reset(pressed);self.button_initialized=True
        if not self.button.update(pressed,now):return
        if self.active:
            if self.single_left:
                self.stop('X stop')
                return
            self.stop('B stop');return
        if self.return_phase is not None:
            self.reason='X rejected: virtual return in progress' if self.single_left else 'B rejected: virtual return in progress';return
        if not self.ready(now):self.reason='X rejected: left input/feedback must be fresh' if self.single_left else 'B rejected: both streams/feedback must be fresh and braking finished';return
        for side in self.sides:
            q=self.feedback[side]['q'];self.home[side]=q.copy();self.reference[side]=BoundedJointReference(self.limits,q)
            target=self.mounts[side]@self.kin[side].fk(q)
            self.robot_anchor[side]=target.copy()
            src=converted_matrix(self.poses[side].sample)
            self.calibration[side]=PoseCalibration.from_anchor(matrix_pose(src),matrix_pose(target),motion_mapping='spatial-relative')
            self.anchor_source[side]=src.copy();self.desired[side]=target.copy()
            self.shapers[side].reset(target[:3,3],Rotation.from_matrix(target[:3,:3]))
        self.active=True;self.takeover=True;self.status='FOLLOWING';self.reason=('X engaged from measured left arm' if self.single_left else 'B engaged from measured joints')
        self.events.append(dict(time=now,action='engage'));self.events=self.events[-100:]

    def tick(self, now):
        dt=0.02 if self.last_tick is None else now-self.last_tick;self.last_tick=now
        if dt<=0:return
        if dt>.1:self.stop('preview loop delayed');dt=.02
        fresh=all(s in self.feedback and s in self.poses and self.poses[s].sample.tracked
            and 0<=now-self.feedback[s]['time']<=self.stale_s
            and 0<=now-self.poses[s].received_monotonic_s<=self.stale_s for s in self.sides)
        if self.active and not fresh:self.stop('input or feedback stale')
        if self.return_phase=='RETURNING':
            self.return_elapsed+=dt
            u=min(self.return_elapsed/self.return_duration,1.)
            blend=10*u**3-15*u**4+6*u**5
            for s,ref in self.reference.items():
                previous=ref.position.copy()
                ref.position=self.return_start[s]+blend*(self.home[s]-self.return_start[s])
                ref.velocity=(ref.position-previous)/dt
            if self.return_elapsed>=self.return_duration+dt:
                for ref in self.reference.values():ref.velocity=np.zeros(7)
                self.return_phase=None;self.button.require_release()
                self.reason='virtual return complete; release X then press to engage' if self.single_left else 'virtual return complete; release B then press to engage'
                self.events.append(dict(time=now,action='home_complete'));self.events=self.events[-100:]
            self.status='RETURNING' if self.return_phase else ('READY' if fresh else 'WAITING')
            return
        for s,ref in self.reference.items():
            if self.active and not self.takeover:
                mapped=pose_matrix(self.calibration[s].apply(matrix_pose(converted_matrix(self.poses[s].sample))))
                target=self.desired[s].copy() if self.single_left else mapped
                if self.single_left:
                    delta=mapped[:3,3]-self.robot_anchor[s][:3,3]
                    target[:3,3]=self.robot_anchor[s][:3,3]+self.position_scale*delta
                target=self.shapers[s].update(target,dt);self.desired[s]=target
                base_target=np.linalg.inv(self.mounts[s])@target
                desired,_=differential_step(self.kin[s],ref.position,base_target,dt,self.limits.lower,self.limits.upper,null_gain=0,limit_velocity=False)
                ref.advance(desired,dt)
            elif not self.active and not ref.stopped:ref.advance(ref.position,dt,stop=True)
            elif not self.active and self.return_phase is None and s in self.feedback and 0<=now-self.feedback[s]['time']<=self.stale_s:
                # While idle, align preview with measured pose; retained stop
                # remains visible if feedback/input are stale.
                ref.position=self.feedback[s]['q'].copy()
        if self.return_phase=='BRAKING' and all(r.stopped for r in self.reference.values()):
            self.return_start={s:r.position.copy() for s,r in self.reference.items()}
            self.return_duration=1.
            for s in self.reference:
                distance=np.abs(self.home[s]-self.return_start[s])
                self.return_duration=max(self.return_duration,
                    float(np.max(1.875*distance/np.asarray(self.limits.velocity))),
                    float(np.max(np.sqrt(5.774*distance/np.asarray(self.limits.acceleration)))))
            self.return_elapsed=0.;self.return_phase='RETURNING';self.status='RETURNING'
            self.reason='returning virtual left arm to takeover start; X locked' if self.single_left else 'returning virtual arms to takeover start; B locked'
            self.takeover=False
            return
        self.takeover=False
        if not self.active and any(not r.stopped for r in self.reference.values()):self.status='STOPPING'
        elif not self.active:self.status='READY' if fresh else 'WAITING'
        if not self.active and self.reason in {
            'waiting for left input and feedback','waiting for both inputs and feedback',
            'left input and feedback ready; press X to engage','both inputs and feedback ready; press B to engage'}:
            if fresh:self.reason='left input and feedback ready; press X to engage' if self.single_left else 'both inputs and feedback ready; press B to engage'
            else:self.reason='waiting for left input and feedback' if self.single_left else 'waiting for both inputs and feedback'

    def state(self, now):
        arms={}
        for s in self.sides:
            f=self.feedback.get(s);p=self.poses.get(s);item=dict(ip='172.16.0.3' if s=='left' else '172.16.0.2',feedback_age_s=None if f is None else now-f['time'],pose_age_s=None if p is None else now-p.received_monotonic_s,feedback_source=None if f is None else f['source'])
            for label,q in [('measured',None if f is None else f['q']),('virtual',None if s not in self.reference else self.reference[s].position)]:
                if q is not None:
                    pts,tcp=self.kin[s].geometry(q);item[label]=dict(q=q.tolist(),points=[(self.mounts[s]@np.r_[v,1])[:3].tolist() for v in pts],tcp=(self.mounts[s]@tcp).tolist())
            if p is not None:
                src=converted_matrix(p.sample)
                if s in self.anchor_source:src[:3,3]+=pose_matrix(self.calibration[s].target_anchor)[:3,3]-self.anchor_source[s][:3,3]
                item['controller']=src.tolist();item['tracked']=p.sample.tracked
                item['clutch_pressed']=bool(p.sample.clutch)
            if self.single_left and s in self.robot_anchor:
                mapped=pose_matrix(self.calibration[s].apply(matrix_pose(converted_matrix(p.sample))))
                delta=mapped[:3,3]-self.robot_anchor[s][:3,3]
                requested=self.robot_anchor[s].copy()
                requested[:3,3]+=self.position_scale*delta
                item['targets']=dict(raw_translation_delta_m=delta.tolist(),
                    scaled_translation_delta_m=(self.position_scale*delta).tolist(),
                    requested_tcp=requested.tolist(),shaped_tcp=self.desired[s].tolist(),
                    preview_q=self.reference[s].position.tolist(),accepted_q=None)
            arms[s]=item
        return dict(status=self.status,active=self.active,reason=self.reason,arms=arms,events=self.events[-12:],server_monotonic_s=now,mode='single-left' if self.single_left else 'dual',position_scale=self.position_scale,control_eligible=False,hardware_commands_enabled=False)
