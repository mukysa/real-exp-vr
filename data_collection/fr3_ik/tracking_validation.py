"""Reproducible, hardware-disconnected section 5.1 validation and replay."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from .a3_replay import replay_bilateral
from .constraints import Limits


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT/'recordings/quest_raw/stage11-openvr-guided-20261020.jsonl'
SNAPSHOT = ROOT/'recordings/robot_state/initial-20261021-103516/snapshot.json'
BASELINE = ROOT/'outputs/fr3_ik/stage11-a3-bilateral-comparison-v2'


def read_rows(path, side):
    return [json.loads(line) for line in (path/side/'samples.jsonl').read_text().splitlines()]


def assess(path, summary):
    result = {}
    limits = Limits.from_controller()
    for side in ('left', 'right'):
        rows = read_rows(path, side)
        q = np.asarray([r['q_reference_rad'] for r in rows])
        dt = np.asarray([r['dt_s'] for r in rows])
        v = np.diff(q, axis=0)/dt[1:, None]
        a = np.diff(np.vstack([np.zeros(7), v]), axis=0)/dt[1:, None]
        assert np.isfinite(q).all() and np.isfinite(v).all() and np.isfinite(a).all()
        assert np.all(q >= limits.lower) and np.all(q <= limits.upper)
        assert np.all(np.abs(v) <= np.asarray(limits.velocity)+1e-9)
        assert np.all(np.abs(a) <= np.asarray(limits.acceleration)+1e-8)
        assert np.max(np.abs(np.diff(q, axis=0))) <= limits.max_step_rad
        entries = [i for i, r in enumerate(rows) if r['command_emitted'] and (i == 0 or not rows[i-1]['command_emitted'])]
        takeover_steps = [float(np.max(np.abs(q[i]-q[i-1]))) for i in entries]
        assert takeover_steps and all(step == 0 for step in takeover_steps)
        for i in entries:
            np.testing.assert_allclose(rows[i]['raw_desired_world'], rows[i]['actual_world'], atol=1e-9, rtol=0)
        assert all(r['measured_joint_state'] is None and not r['control_eligible'] for r in rows)
        result[side] = dict(frames=len(rows), takeover_steps_rad=takeover_steps,
            max_step_rad=float(np.max(np.abs(np.diff(q, axis=0)))),
            max_speed_rad_s=float(np.max(np.abs(v))), max_acceleration_rad_s2=float(np.max(np.abs(a))),
            max_position_lag_m=summary['arms'][side]['max_fk_position_error_m'],
            max_rotation_lag_rad=summary['arms'][side]['max_fk_rotation_error_rad'],
            stopping_frames=sum(r['reference_phase'] == 'STOPPING' for r in rows),
            residual_failure_frames=summary['arms'][side]['strict_check_failures']['residual'],
            trajectory_sha256=summary['arms'][side]['trajectory_sha256'])
    return result


def synthetic_recording(output, fault=None):
    """Derived input is explicitly synthetic; the original is never rewritten."""
    source_lines = [json.loads(line) for line in SOURCE.read_text().splitlines()]
    templates = {r['side']: r for r in source_lines[1:3]}
    rows = []
    # 100 Hz lets an attempted takeover occur during bounded braking while
    # respecting the existing 150 ms button debounce. Motion is within the
    # source discontinuity gate. Afterwards a fixed pose tests settling.
    presses = {50, 250, 268, 330, 500, 580, 1500}
    for i in range(1700):
        time = 100 + i*.01
        offset = 0 if i < 70 else min((i-70)*.0035, .65)
        if i >= 350:
            offset = .65 - min((i-350)*.0025, .45)
        for side in ('left', 'right'):
            row = dict(templates[side])
            row.update(sequence=len(rows)+1, received_monotonic_s=time+(0 if side == 'left' else .0001),
                       frame_id=i, device_timestamp_ns=int(time*1e9),
                       clutch=side == 'right' and i in presses,
                       position=[templates[side]['position'][0]+offset, *templates[side]['position'][1:]])
            if fault == 'tracking' and side == 'left' and i == 420:
                row['tracked'] = False
            if fault == 'jump' and side == 'left' and i == 420:
                row['position'][0] += .25
            if fault == 'gap' and side == 'left' and 420 <= i < 445:
                continue
            if fault == 'connection' and i >= 420:
                row['connection_id'] = 2
            row['sequence'] = len(rows)+1
            rows.append(row)
    metadata = dict(record_type='metadata', schema_version=1, source='synthetic_section_5_1',
                    synthetic=True, template_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(), fault=fault)
    end = dict(record_type='summary', completed=True, pose_records=len(rows), receiver_pose_records=len(rows), subscriber_dropped_records=0)
    output.write_text(''.join(json.dumps(row)+'\n' for row in [metadata, *rows, end]))


def run(output):
    output.mkdir(parents=True, exist_ok=False)
    source_hash = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    summaries = {}
    report = dict(source_sha256=source_hash, scope='Offline sampled references; no hardware/controller/feedback validation', runs={})
    for name in ('bounded', 'repeat', 'strict'):
        summary = replay_bilateral(SOURCE, SNAPSHOT, output/name,
                                   tracking_mode='strict' if name == 'strict' else 'dexarena_velocity')
        summaries[name] = summary
        if name != 'strict':
            report['runs'][name] = assess(output/name, summary)
    for side in ('left', 'right'):
        assert summaries['bounded']['arms'][side]['trajectory_sha256'] == summaries['repeat']['arms'][side]['trajectory_sha256']
        old = json.loads((BASELINE/'strict'/'summary.json').read_text())
        assert summaries['strict']['arms'][side]['trajectory_sha256'] == old['arms'][side]['trajectory_sha256']
    probe_dir = output/'controller-generator-probe'
    probe_dir.mkdir()
    root = ROOT/'gello_software/ros2/src/franka_fr3_arm_controllers'
    probe_source = ROOT/'scripts/fr3_reference_controller_probe.cpp'
    probe_binary = probe_dir/'probe'
    subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(root/'include'), str(probe_source),
        str(root/'src/joint_reference_generator.cpp'), '-o', str(probe_binary)], check=True)
    report['controller_generator_probe'] = {}
    for side in ('left', 'right'):
        rows = read_rows(output/'bounded', side)
        source = probe_dir/f'{side}-waypoints.txt'
        with source.open('w') as stream:
            q = rows[0]['q_reference_rad']
            stream.write('0 0 '+' '.join(map(str, q))+'\n')
            for row in rows:
                q = row['q_reference_rad']
                stream.write(f"{row['source_monotonic_s']-rows[0]['source_monotonic_s']:.12f} "
                    f"{int(row['command_emitted'])} "+' '.join(map(str, q))+'\n')
        trace = probe_dir/f'{side}-1khz.csv'
        measured = json.loads(subprocess.run([str(probe_binary), str(trace)], input=source.read_text(),
            text=True, check=True, capture_output=True).stdout)
        assert measured['constraint_violations'] == 0
        assert trace.is_file() and trace.stat().st_size > 1_000_000
        report['controller_generator_probe'][side] = measured
    report['deterministic_repeat'] = True
    report['strict_v2_trajectory_unchanged'] = True
    report['bilateral_paired_compute_timing'] = summaries['bounded']['bilateral_paired_compute_timing']
    for fault in (None, 'tracking', 'jump', 'gap', 'connection'):
        name = 'synthetic-'+(fault or 'stop-reengage')
        recording = output/(name+'.jsonl')
        synthetic_recording(recording, fault)
        summary = replay_bilateral(recording, SNAPSHOT, output/name, tracking_mode='dexarena_velocity')
        report['runs'][name] = assess(output/name, summary)
        report['runs'][name]['events'] = summary['control_events']
        actions = [e['action'] for e in summary['control_events']]
        assert actions.count('engage_both') >= 3
        assert 'engage_rejected' in actions, 'Takeover during braking must be rejected'
        if fault:
            assert 'fault_hold_both' in actions
        for side in ('left', 'right'):
            rows = read_rows(output/name, side)
            assert any(r['reference_phase'] == 'STOPPING' for r in rows)
            static = [r for r in rows if 112 <= r['source_monotonic_s'] < 114]
            assert static and all(r['q_reference_rad'] == static[0]['q_reference_rad'] for r in static)
            report['runs'][name][side]['static_tail_exactly_constant'] = True
            # Every transition to FOLLOWING uses the final stopped reference.
            # No old requested joint trajectory is resumed after a fault.
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == source_hash
    report['source_unchanged'] = True
    report['offline_checks_passed'] = True
    report['control_eligible'] = False
    (output/'validation.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    links = ''.join(f'<li><a href="{name}/playback.html">{name}</a></li>' for name in ['bounded', 'strict', 'synthetic-stop-reengage', 'synthetic-tracking', 'synthetic-jump', 'synthetic-gap', 'synthetic-connection'])
    (output/'playback.html').write_text(f'''<!doctype html><meta charset="utf-8"><title>Stage11 V3 离线受限参考</title>
<style>body{{font:16px system-ui;max-width:1100px;margin:24px auto;line-height:1.7;background:#101820;color:#e5edf5}}a{{color:#73cfff}}iframe{{width:100%;height:880px;border:1px solid #52677d}}</style>
<h1>Stage11 V3：接管与受限关节参考</h1><p>同一 Stage11、快照及安装映射。首帧不动，关节居中禁用；期望关节步进经速度/加速度受限参考跟随，位置滞后如实保留。未模拟反馈，所有输出 control_eligible=false。动作方向与观感待用户复查。</p>
<p><a href="../stage11-a3-bilateral-comparison-v2/playback.html">保留的 Stage11 V2 三模式对照</a> · <a href="validation.json">数值验证</a> · <a href="bounded/report.html">本次摘要报告</a></p>
<p>完整原始时间轴保留；默认按接管窗口显示机械臂。受限参考回放可以选择“显示停止与未接管机械臂”查看减速。C++ 探针以 1 kHz 驱动仓库现有 JointReferenceGenerator，完整轨迹 CSV 可下载；它不模拟机器人或 ROS。合成回放含运动中停止、刹车中拒绝接管、重新接管、静止与故障。</p><p><a href="controller-generator-probe/left-1khz.csv">左臂 1 kHz 参考</a> · <a href="controller-generator-probe/right-1khz.csv">右臂 1 kHz 参考</a></p><ul>{links}</ul><iframe src="bounded/playback.html" title="Stage11 受限参考"></iframe>''')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.output), indent=2))


if __name__ == '__main__':
    main()
