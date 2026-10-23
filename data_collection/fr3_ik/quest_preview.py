"""Offline first-frame anchored geometric IK inspection; never executable targets."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
from html import escape
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.target_preview import TargetPreviewGenerator
from data_collection.meta_quest.replay import load_recording
from data_collection.move_to_target_ee import solve_fr3_ik
from .kinematics import Kinematics
from .constraints import Limits, check_candidate, residual
from .trajectories import Q0
from .preview_mapping import converted_matrix, base_relative_target, world_relative_target
from .mount_calibration import load_tabletop_calibration


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--side',choices=['left','right'],default='right')
    parser.add_argument('--position-scale',type=float,default=1.0)
    parser.add_argument('--mapping',choices=['base-relative','world-relative','legacy'],default='base-relative')
    parser.add_argument('--mount-calibration',type=Path,
                        default=Path(__file__).resolve().parents[2]/'calibration/matrix.md',
                        help='Documented tabletop mounting calibration for world-relative mapping')
    parser.add_argument('--initial-snapshot',type=Path,
                        help='Offline read-only Franka snapshot; use its measured right-arm joints as the initial pose')
    args=parser.parse_args(argv)
    if not np.isfinite(args.position_scale) or args.position_scale<=0: parser.error('position-scale must be positive and finite')
    if args.output.exists(): parser.error('Use a new output directory')
    before=hashlib.sha256(args.recording.read_bytes()).hexdigest()
    recording=load_recording(args.recording)
    poses=[p for p in recording.poses if p.sample.side==args.side and p.sample.source=='controller']
    if not poses or len({p.connection_id for p in poses})!=1:
        parser.error(f'Need {args.side}-controller poses from exactly one connection')
    kin=Kinematics(args.side); limits=Limits.from_controller(); q0=list(Q0)
    initialization=dict(source='default offline test configuration',measured=False)
    snapshot_bytes=None
    if args.initial_snapshot is not None:
        snapshot_bytes=args.initial_snapshot.read_bytes()
        snapshot=json.loads(snapshot_bytes)
        if snapshot.get('schema')!='franka_readonly_initial_snapshot_v1':
            parser.error('Unsupported initial snapshot schema')
        arm=snapshot['arms'][args.side]
        if arm['joint_names']!=[f'{args.side}_fr3_joint{i}' for i in range(1,8)]:
            parser.error('Snapshot joint order does not match the selected arm model')
        q0=np.asarray(arm['q_rad'],dtype=float)
        if q0.shape!=(7,) or not np.isfinite(q0).all() or np.any(q0<limits.lower) or np.any(q0>limits.upper):
            parser.error('Snapshot must contain seven finite right-arm joints within controller limits')
        tool=np.asarray(arm['F_T_EE_matrix'],dtype=float)
        if tool.shape!=(4,4) or not np.allclose(tool,kin.tool,atol=1e-6,rtol=0):
            parser.error('Snapshot tool transform differs from the nominal gripper model')
        q0=q0.tolist()
        initialization=dict(source=f'measured {args.side}-arm snapshot',measured=True,
            snapshot_path=str(args.initial_snapshot.resolve()),
            snapshot_sha256=hashlib.sha256(snapshot_bytes).hexdigest(),
            snapshot_copy='initial_snapshot.json',
            captured_at=arm['host_capture_completed_at'],robot_ip_at_capture=arm['ip'],
            numeric_precision=snapshot.get('numeric_precision'),
            note='Offline initialization from saved measurements; no current robot connection')
    model_provenance=dict(kin.provenance)
    if initialization['measured']:
        model_provenance.update(actual_F_T_EE_verified=True,
            actual_F_T_EE_verification_source='saved snapshot tool compared with nominal tool at absolute tolerance 1e-6',
            actual_F_T_EE_snapshot_sha256=initialization['snapshot_sha256'],
            actual_F_T_EE_captured_at=initialization['captured_at'])
    initial=kin.fk(q0)
    mounting=None; mounting_bytes=None; world_from_base=None
    if args.mapping=='world-relative':
        world_from_bases,mounting,mounting_bytes=load_tabletop_calibration(args.mount_calibration)
        world_from_base=world_from_bases[args.side]
    mapper=TargetPreviewGenerator(RigidPose(tuple(initial[:3,3]),tuple(Rotation.from_matrix(initial[:3,:3]).as_quat())),
                                  side=args.side,anchor='first',frame_id=f'{args.side}_fr3_link0')
    source_anchor=converted_matrix(poses[0].sample)
    q=np.array(q0); velocity=np.zeros(7); rows=[]; start=poses[0].received_monotonic_s
    for i,received in enumerate(poses):
        mapped=mapper.update(received)
        desired=np.eye(4);desired[:3,3]=mapped.pose.position
        desired[:3,:3]=Rotation.from_quat(mapped.pose.quaternion_xyzw).as_matrix()
        if args.mapping=='base-relative':
            desired=base_relative_target(converted_matrix(received.sample),source_anchor,initial)
        elif args.mapping=='world-relative':
            desired=world_relative_target(converted_matrix(received.sample),source_anchor,initial,world_from_base)
        desired[:3,3]=initial[:3,3]+args.position_scale*(desired[:3,3]-initial[:3,3])
        dt=received.received_monotonic_s-poses[i-1].received_monotonic_s if i else 1/30
        if dt<=0: raise ValueError('Non-increasing right-hand arrival timestamps')
        old=q.copy();old_velocity=velocity.copy();candidate=None;checks=None;detail='';geometry_ok=False
        begun=time.perf_counter()
        try:
            if not received.sample.tracked or not mapped.valid: raise ValueError('Invalid tracking or mapping')
            solution=solve_fr3_ik(q,desired,kin.tool,kinematics=kin,try_alternative_seeds=False,max_function_evaluations=80)
            candidate=solution.q
            _,actual=kin.geometry(candidate)
            errors=residual(actual,desired)
            checks,reason,cv,ca=check_candidate(candidate,q,velocity,dt,*errors,limits)
            geometry_ok=checks.residual and checks.joint_limits
            if geometry_ok:
                q=candidate.copy() # DIAGNOSTIC only: do not confuse with checked/accepted targets.
            detail='Geometric IK only; control constraints: '+reason.value
        except (RuntimeError,ValueError,np.linalg.LinAlgError) as exc:
            detail=str(exc)
        elapsed=time.perf_counter()-begun
        points,actual=kin.geometry(q); errors=residual(actual,desired)
        velocity=(q-old)/dt;acceleration=(velocity-old_velocity)/dt
        rows.append(dict(scenario=f'test_{args.side}_geometric_preview',index=i,time=received.received_monotonic_s-start,
            event='first_anchor_inspection' if i==0 else 'inspection',connection_id=received.connection_id,
            sequence=received.sequence,state='GEOMETRIC_ONLY' if geometry_ok else 'GEOMETRIC_FAILED_HOLD',
            reason='none' if checks and checks.all_passed else 'control_constraints_not_passed',detail=detail,
            success=bool(geometry_ok),control_eligible=False,all_control_checks_passed=bool(checks and checks.all_passed),
            raw_position=list(received.sample.position),raw_quaternion_xyzw=list(received.sample.quaternion_xyzw),
            grasp=received.sample.grasp,clutch=received.sample.clutch,record=received.sample.record,source_monotonic_s=received.received_monotonic_s,
            dt_s=dt,q=q.tolist(),velocity=velocity.tolist(),acceleration=acceleration.tolist(),
            desired=desired.tolist(),actual=actual.tolist(),points=points,candidate_points=None,
            candidate_q=candidate.tolist() if candidate is not None else None,
            position_error_m=errors[0],rotation_error_rad=errors[1],solve_time_s=elapsed,
            checks=asdict(checks) if checks else {},period_exceeded=elapsed>dt))
        if world_from_base is not None:
            world_points=(world_from_base[:3,:3]@np.asarray(points).T).T+world_from_base[:3,3]
            rows[-1].update(desired_world=(world_from_base@desired).tolist(),
                            actual_world=(world_from_base@actual).tolist(),points_world=world_points.tolist())
        if (i+1)%100==0: print(f'Computed {i+1}/{len(poses)} {args.side}-hand frames',flush=True)
    after=hashlib.sha256(args.recording.read_bytes()).hexdigest()
    if before!=after: raise RuntimeError('Source file changed during replay')
    scenario=dict(passed=all(r['success'] for r in rows),frames=len(rows),geometric_solutions=sum(r['success'] for r in rows),
                  frames_passing_control_checks=sum(r['all_control_checks_passed'] for r in rows),
                  states=dict(Counter(r['state'] for r in rows)))
    timing=np.array([r['solve_time_s'] for r in rows]);hz=1/np.median([r['dt_s'] for r in rows[1:]])
    summary=dict(passed=scenario['passed'],scope='DIAGNOSTIC first-frame anchor; ignores clutch for inspection; NEVER robot commands',
        source=str(args.recording),source_sha256=before,source_unchanged=True,side=args.side,raw_pose_count=len(recording.poses),
        clutch_true_frames=sum(bool(p.sample.clutch) for p in poses),strict_button_replay='Not evaluated by geometric preview' if any(p.sample.clutch for p in poses) else 'No engage edge: hold q0 throughout',
        robot_connected=False,control_eligible=False,hz=float(hz),limits=asdict(limits),model=model_provenance,
        q0_rad=q0,initialization=initialization,position_scale=args.position_scale,mapping=args.mapping,rotation_convention='tabletop spatial delta conjugated into base frame' if args.mapping=='world-relative' else 'spatial delta R_current R_initial^T left-multiplies robot R0' if args.mapping=='base-relative' else 'legacy SE3 conjugation',scenarios={f'test_{args.side}_geometric_preview':scenario},
        solver_time_ms=dict(zip(('p50','p95','p99','max'),map(float,np.r_[np.percentile(timing,[50,95,99]),max(timing)]*1000))),
        solve_period_overruns=sum(r['period_exceeded'] for r in rows),attempted_solves=len(rows),
        timing_scope='Recorded arrival intervals, unresampled; geometry playback only. Constraints evaluated but not used to advance diagnostic q.')
    if mounting is not None:
        summary.update(mounting_calibration=mounting,display_frame='tabletop_world',
                       ik_frame=f'{args.side}_fr3_link0',base_from_world=np.linalg.inv(world_from_base).tolist(),
                       world_from_base=world_from_base.tolist(),
                       sample_frame_convention=f'desired/actual/points in {args.side} base; *_world in tabletop world',
                       source_heading_assumption='Existing fixed Quest-axis conversion aligned to tabletop axes; extra tracking-space yaw not calibrated')
    args.output.mkdir(parents=True)
    if snapshot_bytes is not None:
        (args.output/'initial_snapshot.json').write_bytes(snapshot_bytes)
    if mounting_bytes is not None:
        (args.output/'mount_calibration.md').write_bytes(mounting_bytes)
    (args.output/'samples.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in rows))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    with (args.output/'joint_angles.csv').open('w') as f:
        f.write('sequence,time_s,geometric_solution,control_checks_passed,'+','.join(f'fr3_joint{i}_rad' for i in range(1,8))+'\n')
        for r in rows:f.write(','.join(map(str,[r['sequence'],r['time'],r['success'],r['all_control_checks_passed'],*r['q']]))+'\n')
    directory=Path(__file__).parent/'visualization'
    html=(directory/'viewer.html').read_text().replace('/*REPORT_DATA*/',json.dumps({'summary':summary,'samples':rows},allow_nan=False).replace('<','\\u003c')).replace('/*VIEWER_JS*/',(directory/'viewer.js').read_text())
    html=html.replace('FR3 / 连续 IK 离线回放','Quest 录制轨迹 / 几何 IK 诊断预览').replace('检查通过目标的 FK','几何解 FK（未控制验收）').replace('通过目标的 FK','几何解 FK').replace('验收与性能','几何解算统计（不代表控制通过）')
    clutch_note = '原记录含接管按钮记录，但本几何预览不按按钮切分轨迹。' if any(p.sample.clutch for p in poses) else '原记录 clutch 全程为 false，严格按钮回放应始终保持初始姿态。'
    html=html.replace('<div class="bar">',f'<p style="color:#ffba70">注意：{clutch_note}此预览使用首帧相对锚定观察完整右手轨迹；动态约束未通过的几何解仍显示，禁止用于控制。</p><div class="bar">',1)
    if args.mapping=='base-relative':
        html=html.replace('Quest 录制轨迹 / 几何 IK 诊断预览','新版 · 固定基轴平移 / 几何 IK 预览')
        html=html.replace('<div class="bar">','<p>新版映射：向前 → 机器人 +X；向左 → +Y；向上 → +Z。初始握柄倾角不再旋转平移方向。旋转使用基轴相对旋转；人体朝向与跟踪空间的额外偏航仍未标定。</p><div class="bar">',1)
    elif args.mapping=='world-relative':
        title=f'{args.recording.stem} · V4 · 桌面坐标与倾斜基座'
        html=html.replace('<title>FR3 · Offline IK Review</title>',f'<title>{escape(title)}</title>')
        html=html.replace('Quest 录制轨迹 / 几何 IK 诊断预览',escape(title))
        html=html.replace('基坐标系 right_fr3_link0','显示坐标系：桌面世界 · IK 坐标系：right_fr3_link0')
        html=html.replace('<option value="front">','<option value="front" selected>')
        tilt=mounting['tilt_from_vertical_deg'][args.side]
        html=html.replace('<div class="bar">',f'<p>已接入历史安装标定：右臂基座倾斜约 {tilt:.1f}°。手柄前后、左右、上下先按桌面轴解释，再转换到机械臂基座求解；显示也转换到桌面坐标系。网格为桌面 Z=0，基座旁小坐标轴随安装倾斜。额外的 Quest 跟踪空间偏航仍未标定。</p><div class="bar">',1)
    initial_note = ('实测右臂起点 · 采集于 '+initialization['captured_at']) if initialization['measured'] else '默认离线测试起点'
    html=html.replace('<h1>',f'<p style="color:#64dbc8">{escape(initial_note)} · 位移比例 {args.position_scale:g} 倍 · 旋转比例 1 倍</p><h1>',1)
    if args.side=='left':
        html=html.replace('右臂','左臂').replace('右手','左手').replace('right_fr3_link0','left_fr3_link0')
    (args.output/'report.html').write_text(html)
    # Independent left/right source comparison in fixed robot-direction axes, no left-arm IK implied.
    comparison={}
    for side in ('left','right'):
        selected=[p for p in recording.poses if p.sample.side==side]
        if not selected: continue
        origin=converted_matrix(selected[0].sample)[:3,3]
        comparison[side]=[dict(t=p.received_monotonic_s-poses[0].received_monotonic_s,
            delta=(converted_matrix(p.sample)[:3,3]-origin).tolist(),grasp=p.sample.grasp or 0) for p in selected]
    (args.output/'bilateral.json').write_text(json.dumps(comparison,allow_nan=False))
    compare_template=(directory/'bilateral.html').read_text()
    (args.output/'bilateral.html').write_text(compare_template.replace('/*DATA*/',json.dumps(comparison,allow_nan=False)))
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
