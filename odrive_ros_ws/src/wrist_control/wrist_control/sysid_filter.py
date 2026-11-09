"""Flag sysid_excite points whose plate pose came from a different set of AprilTags than at rest.

No ROS needed.

Why: plate tilt from the baseplate camera is only well determined by where the tags are, and
each tag's own tilt is off by several degrees (camera_tracking.md, "Tilt depends on which tags
are seen"). When a tag leaves or enters the frame the fitted pose jumps by 1-7 deg of tilt and
up to ~2.5 mm, within one 8 ms frame. The plate itself cannot move that fast (the fastest sweep
ramp moves it ~0.02 deg and ~0.035 mm per frame), so a frame-to-frame jump above the thresholds
below is a tag-set switch, not motion.

Two kinds of bad point:
  flicker   the switch happens during the averaging window (the pose alternates between two
            values while the motors hold still), so the averaged row mixes both
  between   the switch happened earlier (often while settling) and the pose stayed in the other
            state, so the row is steady but shifted. Every excursion away from 0 deg starts and
            ends in the rest tag set, so points before its first switch and after its last switch
            are trusted and everything in between is flagged. (Summing the jump sizes instead does
            not work: a tag that drifts back into view changes the pose over several frames, each
            below the threshold, so the sum never returns to zero.)

Logs written after 2026-11-09 also carry the tag set itself (tag_mask, tag_mask_changes,
n_tags from /platform_pose/pose_quality); those are checked too.

    sysid_filter ~/.local/share/wrist_sysid/sysid_static_grid_<stamp>_points.csv
      -> <stamp>_points_flagged.csv  every row + pose_ok, pose_flag, offset_tilt_deg, offset_mm
      -> <stamp>_points_clean.csv    pose_ok rows only (pass this to sysid_fit)
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np

N = 6
SAMPLE_COLS = ['t', 'phase', 'point_id', 'tag_visible', 'mount_ok', 'x_mm', 'y_mm', 'z_mm',
               'tilt_deg', 'tilt_dir_deg'] + [f'cmd{i}' for i in range(N)]


def tilt_vec(tilt_deg: np.ndarray, dir_deg: np.ndarray) -> np.ndarray:
    a = np.deg2rad(dir_deg)
    return np.stack([tilt_deg * np.cos(a), tilt_deg * np.sin(a)], -1)


def read_samples(path: str) -> dict[str, np.ndarray]:
    with open(path) as f:
        rd = csv.reader(f)
        head = next(rd)
        cols = SAMPLE_COLS + [c for c in ('tag_mask', 'n_tags') if c in head]
        ix = [head.index(c) for c in cols]
        rows = [[r[i] for i in ix] for r in rd]
    out = {}
    for k, c in enumerate(cols):
        v = [r[k] for r in rows]
        out[c] = np.array(v) if c == 'phase' else np.array(v, dtype=float)
    return out


def jump_frames(s: dict[str, np.ndarray], tilt_jump: float, pos_jump: float, max_dt: float):
    """Per frame: is it a switch, is it between the first and last switch of its excursion
    from 0 deg, and the summed jump since the last rest hold (x, y, z mm, tilt_x, tilt_y deg;
    reported only, it drifts)."""
    tv = tilt_vec(s['tilt_deg'], s['tilt_dir_deg'])
    y = np.column_stack([s['x_mm'], s['y_mm'], s['z_mm'], tv])
    d = np.diff(y, axis=0)
    jump = ((np.linalg.norm(d[:, 3:], axis=1) > tilt_jump) | (np.linalg.norm(d[:, :3], axis=1) > pos_jump)) \
        & (np.diff(s['t']) < max_dt)
    jump = np.concatenate([[False], jump])
    d = np.vstack([np.zeros((1, 5)), d])
    cmd = np.column_stack([s[f'cmd{i}'] for i in range(N)])
    rest = np.all(np.abs(cmd) < 1e-6, axis=1) & (s['phase'] == 'hold')
    acc = np.zeros_like(y)
    run = np.zeros(5)
    for k in range(len(y)):
        if rest[k] and not jump[k]:
            run[:] = 0.0          # back at 0 deg: the rest tag set is the reference
        elif jump[k]:
            run = run + d[k]
        acc[k] = run
    between = np.zeros(len(y), dtype=bool)
    edges = np.flatnonzero(np.diff(rest.astype(int)) != 0) + 1
    for lo, hi in zip(np.r_[0, edges], np.r_[edges, len(y)]):
        if rest[lo]:
            continue
        js = np.flatnonzero(jump[lo:hi]) + lo
        if len(js) == 1:
            between[js[0]:hi] = True      # one switch and no detected return: distrust the rest
        elif len(js) > 1:
            between[js[0]:js[-1] + 1] = True
    return jump, between, acc


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('points', help='*_points.csv from sysid_excite')
    p.add_argument('--samples', default='', help='matching *_samples.csv (default: next to points)')
    p.add_argument('--tilt-jump', type=float, default=0.75, help='deg per frame that counts as a switch')
    p.add_argument('--pos-jump', type=float, default=0.25, help='mm per frame that counts as a switch')
    p.add_argument('--min-tags', type=int, default=3, help='(new logs) fewest tags in the fit')
    a = p.parse_args(argv)

    samples = a.samples or a.points.replace('_points.csv', '_samples.csv')
    if not os.path.exists(samples):
        sys.exit(f'no samples file {samples}; pass --samples')
    s = read_samples(samples)
    jump, between, acc = jump_frames(s, a.tilt_jump, a.pos_jump, 0.05)
    cmd_s = np.column_stack([s[f'cmd{i}'] for i in range(N)])
    good_frame = (s['phase'] == 'hold') & (s['tag_visible'] > 0) & (s['mount_ok'] > 0)

    with open(a.points) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit('no points')
    has_mask = 'tag_mask' in rows[0]
    out_rows, n_bad = [], {}
    for r in rows:
        cmd = np.array([float(r[f'cmd{i}']) for i in range(N)])
        m = good_frame & (s['point_id'] == int(r['point_id'])) & np.all(np.abs(cmd_s - cmd) < 0.5, axis=1)
        flags = []
        if not m.any():
            flags.append('no_samples')
            off_t = off_p = float('nan')
        else:
            if jump[m].any():
                flags.append('flicker')
            off_t = float(np.linalg.norm(acc[m][:, 3:], axis=1).max())
            off_p = float(np.linalg.norm(acc[m][:, :3], axis=1).max())
            if between[m].any() and 'flicker' not in flags:
                flags.append('between')
        if has_mask:
            if float(r.get('tag_mask_changes') or 0) > 0:
                flags.append('tag_set_changed')
            if 'n_tags' in s and m.any() and np.nanmin(s['n_tags'][m]) < a.min_tags:
                flags.append('few_tags')
        for fl in flags:
            n_bad[fl] = n_bad.get(fl, 0) + 1
        r = dict(r)
        r.update(pose_ok=int(not flags), pose_flag='+'.join(flags), offset_tilt_deg=f'{off_t:.3f}',
                 offset_mm=f'{off_p:.3f}')
        out_rows.append(r)

    base = a.points[:-len('_points.csv')] if a.points.endswith('_points.csv') else a.points[:-4]
    for suffix, keep in (('_points_flagged.csv', lambda r: True), ('_points_clean.csv', lambda r: r['pose_ok'])):
        with open(base + suffix, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(out_rows[0]))
            w.writeheader()
            w.writerows(r for r in out_rows if keep(r))
    ok = sum(r['pose_ok'] for r in out_rows)
    print(f'{len(out_rows)} points: {ok} kept, {len(out_rows) - ok} flagged '
          + ', '.join(f'{k} {v}' for k, v in sorted(n_bad.items())))
    print(f'switch frames: {int(jump.sum())} of {len(jump)}')
    print(f'wrote {base}_points_flagged.csv and {base}_points_clean.csv')


if __name__ == '__main__':
    main()
