"""Offline direction audit and common-view comparison; no hardware IO."""
import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation

from .a3_replay import replay_bilateral, render_strict_playback
from .mount_calibration import load_tabletop_calibration
from .preview_mapping import converted_matrix, base_relative_target
from .tracking_validation import SOURCE, SNAPSHOT, ROOT, assess, read_rows, synthetic_recording
from data_collection.meta_quest.replay import load_recording


V2 = ROOT/'outputs/fr3_ik/stage11-a3-bilateral-comparison-v2/dexarena-follow/playback.html'
V3 = ROOT/'outputs/fr3_ik/stage11-a3-bilateral-bounded-reference-v3-review/bounded/playback.html'


def viewer_data(path):
    document = path.read_text()
    match = re.search(r'<script id="report-data" type="application/json">(.*?)</script>', document, re.S)
    return document, json.loads(match.group(1))


def common_view_copy(source, output, title):
    """Replace only presentation: preserve the exact embedded historical data."""
    original, data = viewer_data(source)
    directory = Path(__file__).parent/'visualization'
    template = (directory/'dual_viewer.html').read_text()
    view = re.search(r'<select id="view">.*?</select>', template).group(0)
    document = re.sub(r'<select id="view">.*?</select>', lambda _: view, original)
    document = re.sub(r'(<script id="report-data" type="application/json">.*?</script><script>).*?(</script>)',
        lambda m: m.group(1)+(directory/'dual_viewer.js').read_text()+m.group(2), document, flags=re.S)
    document = document.replace('</h1>', f'</h1><p><strong>{title}：仅更新观看方向和方向箭头，源轨迹未重算。'
        '后方看左侧 left/.3、右侧 right/.2。旧局部锚定会受初始握柄朝向影响。</strong></p>', 1)
    # Resolve all historical report links from this new directory.
    import os
    document = re.sub(r'href="([^"]+)"', lambda m: m.group(0) if '://' in m.group(1) or m.group(1).startswith('#')
        else f'href="{os.path.relpath((source.parent/m.group(1)).resolve(), output.parent)}"', document)
    output.write_text(document)
    assert viewer_data(output)[1] == data
    return dict(source=str(source.relative_to(ROOT)), original_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                embedded_data_unchanged=True)


def audit_mapping(output):
    recording = load_recording(SOURCE)
    by_sequence = {p.sequence:p for p in recording.poses}
    mounts, _, _ = load_tabletop_calibration(ROOT/'calibration/matrix.md')
    _, old = viewer_data(V3)
    result = {}
    for side in ('left', 'right'):
        old_anchor = next(r for r in old['samples'][side] if r['command_emitted'])
        source_anchor = converted_matrix(by_sequence[old_anchor['source_sequence']].sample)
        target_anchor = np.asarray(old_anchor['raw_desired_world'])
        # Ttarget0 * inverse(Tmount * Tsource0) * Tmount * Tsource
        effective = target_anchor[:3,:3] @ source_anchor[:3,:3].T
        rows = read_rows(output/'corrected', side)
        source0 = target0 = None
        max_pos = max_rot = 0.0
        for i, row in enumerate(rows):
            if not row['command_emitted']:
                source0 = target0 = None
                continue
            source = converted_matrix(by_sequence[row['source_sequence']].sample)
            if source0 is None:
                source0 = source.copy()
                target0 = np.asarray(row['raw_desired_world'])
            expected = base_relative_target(source, source0, target0)
            actual = np.asarray(row['raw_desired_world'])
            pos_error = float(np.max(np.abs(expected[:3,3]-actual[:3,3])))
            rot_error = float(Rotation.from_matrix(expected[:3,:3].T@actual[:3,:3]).magnitude())
            max_pos = max(max_pos,pos_error)
            max_rot = max(max_rot,rot_error)
            assert pos_error < 1e-12 and rot_error < 1e-12
            display_delta = np.asarray(row['controller_world'])[:3,3]-np.asarray(rows[i-1]['controller_world'])[:3,3]
            input_delta = source[:3,3]-converted_matrix(by_sequence[rows[i-1]['source_sequence']].sample)[:3,3]
            np.testing.assert_allclose(display_delta,input_delta,atol=1e-12,rtol=0)
        result[side] = dict(device_ip='172.16.0.3' if side=='left' else '172.16.0.2',
            base_position_world_m=mounts[side][:3,3].tolist(),
            legacy_motion_rotation=effective.tolist(),
            legacy_axis_rotation_deg=float(np.rad2deg(Rotation.from_matrix(effective).magnitude())),
            corrected_motion_rotation=np.eye(3).tolist(),
            raw_mapping_max_position_error_m=max_pos, raw_mapping_max_rotation_error_rad=max_rot)
    return result


def run(output, reuse_corrected=False):
    if output.exists() and not reuse_corrected:
        raise FileExistsError('Use a new output directory')
    if (output/'validation.json').exists():
        raise FileExistsError('Do not overwrite a completed review')
    output.mkdir(parents=True,exist_ok=True)
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (SOURCE,SNAPSHOT,V2,V3)}
    source_summary = output/'corrected/summary.json'
    if reuse_corrected:
        summary=json.loads(source_summary.read_text())
        assert all(a['parameters']['motion_mapping']=='tabletop-relative' for a in summary['arms'].values())
    else:
        summary=replay_bilateral(SOURCE,SNAPSHOT,output/'corrected',tracking_mode='dexarena_velocity',motion_mapping='tabletop-relative')
    # Refresh only this newly generated viewer after presentation edits.
    _, data=viewer_data(output/'corrected/playback.html')
    (output/'corrected/playback.html').write_text(render_strict_playback(summary,data['timeline'],
        {s:read_rows(output/'corrected',s) for s in ('left','right')}))
    result=dict(schema='fr3_tabletop_direction_audit_v1',control_eligible=False,
        physical_side_confirmation='User: rear photograph left is left/172.16.0.3; right is right/172.16.0.2',
        tracking_yaw='Assumed zero; photographs do not establish tracking-to-tabletop yaw',
        corrected=assess(output/'corrected',summary),mapping=audit_mapping(output),
        history={})
    for label,source in (('v2',V2),('v3',V3)):
        result['history'][label]=common_view_copy(source,output/f'{label}-same-view.html',label.upper())
    repeated=replay_bilateral(SOURCE,SNAPSHOT,output/'repeat',tracking_mode='dexarena_velocity',motion_mapping='tabletop-relative')
    assert all(summary['arms'][s]['trajectory_sha256']==repeated['arms'][s]['trajectory_sha256'] for s in ('left','right'))
    result['deterministic_repeat']=True
    result['synthetic']={}
    for fault in (None,'tracking','jump','gap','connection'):
        name='synthetic-'+(fault or 'stop-reengage')
        path=output/(name+'.jsonl')
        synthetic_recording(path,fault)
        synthetic=replay_bilateral(path,SNAPSHOT,output/name,tracking_mode='dexarena_velocity',motion_mapping='tabletop-relative')
        assessment=assess(output/name,synthetic)
        actions=[e['action'] for e in synthetic['control_events']]
        assert actions.count('engage_both')>=3 and 'engage_rejected' in actions
        if fault: assert 'fault_hold_both' in actions
        for s in ('left','right'):
            rows=read_rows(output/name,s)
            tail=[r for r in rows if 112<=r['source_monotonic_s']<114]
            assert tail and all(r['q_reference_rad']==tail[0]['q_reference_rad'] for r in tail)
            assert any(r['reference_phase']=='STOPPING' for r in rows)
            assessment[s]['static_tail_exactly_constant']=True
        result['synthetic'][name]=assessment
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    result['preserved_sha256']=hashes
    result['offline_checks_passed']=True
    (output/'validation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    rows=''.join(f'<tr><td>{s}</td><td>{result["mapping"][s]["legacy_axis_rotation_deg"]:.2f}°</td><td>0°（共同桌面轴）</td></tr>' for s in ('left','right'))
    synthetic_links=' · '.join(f'<a href="{n}/playback.html#t=2.5&view=rear-right">{n}</a>' for n in result['synthetic'])
    (output/'playback.html').write_text(f'''<!doctype html><meta charset="utf-8"><title>V4 方向核对</title>
<style>body{{font:16px system-ui;background:#101820;color:#e5edf5;max-width:1450px;margin:24px auto;line-height:1.7}}a{{color:#73cfff}}button,select{{padding:9px;background:#22354f;color:white;margin:6px;border:1px solid #587088}}iframe{{width:100%;height:1000px;border:1px solid #52677d}}table{{border-collapse:collapse}}td,th{{padding:8px 20px;border:1px solid #52677d}}</style>
<h1>V4：按操作者后方视角核对方向</h1>
<p>已确认：图 1 左侧为左臂 left / 172.16.0.3，右侧为右臂 right / 172.16.0.2；左手→左臂、右手→右臂。前方是从基座朝工作区伸出，桌面 +X 前 / +Y 左 / +Z 上。</p>
<p>旧 V2/V3 使用局部位姿锚定，手柄位移受接管握柄姿态旋转；新 V4 将两手位移施加在同一桌面轴，腕部相对旋转也使用共同空间轴。Quest 与桌面的额外偏航暂按 0，照片没有完成该物理标定。所有结果仅离线诊断，方向和观感待复查。</p>
<table><tr><th>机械臂</th><th>旧 V3 接管时位移轴额外旋转</th><th>V4 映射</th></tr>{rows}</table>
<p>下方 V2/V3 为原轨迹数据加新视角控件，未重算或覆盖旧文件。V2/V3/V4 使用同一后方/右后方视角比较；箭头和视角是方向示意，未拟合照片相机。</p>
<label>轨迹 <select id="run"><option value="corrected/playback.html">V4：共同桌面轴（新）</option><option value="v3-same-view.html">V3：旧映射、同视角</option><option value="v2-same-view.html">V2：旧映射、同视角</option></select></label>
<label>观看位置 <select id="camera"><option value="rear">后方（图 1）</option><option value="rear-right">右后方（图 2）</option><option value="top">俯视，前在上</option></select></label>
<button onclick="openView()">切换并跳到 10 秒</button><a id="standalone" href="corrected/playback.html#t=10&view=rear">单独打开</a>
<iframe id="player" src="corrected/playback.html#t=10&view=rear"></iframe>
<p>故障/停止回归：{synthetic_links}</p><p><a href="validation.json">数值验证</a> · <a href="README.md">本轮说明</a> · <a href="../stage11-a3-bilateral-bounded-reference-v3-review/playback.html">未修改的原 V3 入口</a> · <a href="../stage11-a3-bilateral-comparison-v2/playback.html">未修改的原 V2 入口</a></p>
<script>function openView(){{const url=document.getElementById('run').value+'#t=10&view='+document.getElementById('camera').value;document.getElementById('player').src=url;document.getElementById('standalone').href=url;}}document.getElementById('run').onchange=openView;document.getElementById('camera').onchange=openView;</script>''')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--reuse-corrected',action='store_true',help='Reuse one unfinished corrected run; never overwrite completed review')
    args=parser.parse_args()
    print(json.dumps(run(args.output,args.reuse_corrected),indent=2))


if __name__=='__main__': main()
