"""Two independent geometric IK previews on a shared recorded arrival timeline."""
import argparse
from bisect import bisect_right
import contextlib
import hashlib
import json
from pathlib import Path

import numpy as np

from . import quest_preview


def align_samples(samples):
    """Causal last-sample display; never pair hands by alternating row index."""
    times = {side: [row['source_monotonic_s'] for row in rows]
             for side, rows in samples.items()}
    for side, values in times.items():
        if not values or any(b <= a for a, b in zip(values, values[1:])):
            raise ValueError(f'{side} requires strictly increasing source arrival times')
    start = min(values[0] for values in times.values())
    events = sorted(set(value for values in times.values() for value in values))
    timeline = []
    for stamp in events:
        frame = dict(time=stamp-start, source_monotonic_s=stamp, arms={})
        for side, values in times.items():
            index = bisect_right(values, stamp)-1
            frame['arms'][side] = dict(
                index=max(0, index), awaiting_first_source=index < 0,
                sample_age_s=None if index < 0 else stamp-values[index])
        timeline.append(frame)
    return timeline


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--initial-snapshot', required=True, type=Path)
    parser.add_argument('--mount-calibration', type=Path,
                        default=Path(__file__).resolve().parents[2]/'calibration/matrix.md')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output directory')
    source_hash = hashlib.sha256(args.recording.read_bytes()).hexdigest()
    args.output.mkdir(parents=True)
    summaries, samples = {}, {}
    for side in ('left', 'right'):
        directory = args.output/side
        print(f'Computing independent {side} IK...', flush=True)
        with (args.output/f'{side}-generation.log').open('w') as log, contextlib.redirect_stdout(log):
            quest_preview.main([
                str(args.recording), '--side', side, '--mapping', 'world-relative',
                '--initial-snapshot', str(args.initial_snapshot),
                '--mount-calibration', str(args.mount_calibration), '--output', str(directory)])
        summaries[side] = json.loads((directory/'summary.json').read_text())
        samples[side] = [json.loads(line) for line in (directory/'samples.jsonl').read_text().splitlines()]
        assert summaries[side]['source_sha256'] == source_hash
        print(f'{side}: {sum(r["success"] for r in samples[side])}/{len(samples[side])} geometric solutions', flush=True)
    timeline = align_samples(samples)
    # Compare source-derived target displacements; reflect only lateral translation.
    # Robot starting poses are measured separately, not forced to be symmetric.
    differences, left_delta, right_delta = [], [], []
    for frame in timeline:
        if any(a['awaiting_first_source'] for a in frame['arms'].values()):
            continue
        deltas = {}
        for side in ('left', 'right'):
            row = samples[side][frame['arms'][side]['index']]
            deltas[side] = np.asarray(row['desired_world'])[:3, 3]-np.asarray(samples[side][0]['desired_world'])[:3, 3]
        mirrored_left = deltas['left']*np.array([1, -1, 1])
        differences.append(mirrored_left-deltas['right'])
        left_delta.append(mirrored_left)
        right_delta.append(deltas['right'])
    differences = np.asarray(differences)
    correlations = []
    for axis in range(3):
        a, b = np.asarray(left_delta)[:, axis], np.asarray(right_delta)[:, axis]
        correlations.append(float(np.corrcoef(a, b)[0, 1]) if min(np.std(a), np.std(b)) > 1e-12 else None)
    assert source_hash == hashlib.sha256(args.recording.read_bytes()).hexdigest()
    summary = dict(
        title=f'{args.recording.stem} · V5 · 双臂桌面回放',
        source=str(args.recording), source_sha256=source_hash, source_unchanged=True,
        robot_connected=False, control_eligible=False, position_scale=1.0, rotation_scale=1.0,
        scope='Independent left/right geometric IK; ignores clutch; no collision or control acceptance',
        display_frame='tabletop_world', duration_s=timeline[-1]['time'], display_events=len(timeline),
        synchronization='Union of original PC arrival timestamps; each arm displays its latest available sample, no interpolation. Before first arrival only initial q0 is shown. Ages are diagnostics, not safety gates.',
        initialization='Separately captured left/right static measured poses; not a simultaneous hardware snapshot',
        arms=summaries,
        symmetry=dict(
            definition='Relative tabletop displacement: compare right [dx,dy,dz] with left [dx,-dy,dz]. No forced symmetric joints, initial poses or orientations.',
            sample_selection='Causal held samples at union of source arrival times after both streams begin',
            paired_events=len(differences), axis_correlation_xyz=correlations,
            axis_rmse_m=np.sqrt(np.mean(differences**2, axis=0)).tolist(),
            distance_rmse_m=float(np.sqrt(np.mean(np.sum(differences**2, axis=1)))),
            max_sample_age_s={side: max(frame['arms'][side]['sample_age_s'] or 0 for frame in timeline)
                              for side in ('left', 'right')},
            interpretation='Source motion comparison with recorded arrival jitter, not an accuracy score or calibrated device synchronization'))
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    (args.output/'timeline.json').write_text(json.dumps(timeline, allow_nan=False)+'\n')
    data = json.dumps(dict(summary=summary, samples=samples, timeline=timeline), allow_nan=False).replace('<', '\\u003c')
    directory = Path(__file__).parent/'visualization'
    html = (directory/'dual_viewer.html').read_text().replace('/*REPORT_DATA*/', data).replace('/*VIEWER_JS*/', (directory/'dual_viewer.js').read_text())
    (args.output/'report.html').write_text(html)
    print(json.dumps(dict(report=str(args.output/'report.html'), frames={side: len(rows) for side, rows in samples.items()},
                          symmetry=summary['symmetry']), indent=2))


if __name__ == '__main__':
    main()
