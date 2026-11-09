"""Unit tests for wrist_control.sysid_filter on synthetic sysid_excite logs (no ROS)."""

import csv

from wrist_control import sysid_excite as se
from wrist_control import sysid_filter as sf

DT = 1 / 120


def write_logs(tmp_path, segments):
    """segments: (point_id, direction, cmd_deg, [tilt per frame]); first half settle, rest hold.
    Writes a sweep-like samples + points pair and returns the points path."""
    samples = tmp_path / 'run_samples.csv'
    points = tmp_path / 'run_points.csv'
    t = 0.0
    with open(samples, 'w', newline='') as fs, open(points, 'w', newline='') as fp:
        ws, wp = csv.writer(fs), csv.writer(fp)
        ws.writerow(se.STATE_COLS)
        wp.writerow(se.POINT_COLS)
        for pid, direction, cmd, tilts in segments:
            for k, tilt in enumerate(tilts):
                row = dict.fromkeys(se.STATE_COLS, 0.0)
                row.update(t=t, phase='settle' if k < len(tilts) // 2 else 'hold', point_id=pid,
                           tag_visible=1, mount_ok=1, n_tags=3, tag_mask=7, fit_rms_px=0.3,
                           tilt_deg=tilt, tilt_dir_deg=0.0)
                row.update({f'cmd{i}': cmd for i in range(6)})
                ws.writerow([row[c] for c in se.STATE_COLS])
                t += DT
            prow = dict.fromkeys(se.POINT_COLS, 0.0)
            prow.update(point_id=pid, direction=direction, tilt_deg=tilts[-1], tag_mask=7, tag_mask_changes=0)
            prow.update({f'cmd{i}': cmd for i in range(6)})
            wp.writerow([prow[c] for c in se.POINT_COLS])
    return points


def flags(points):
    sf.main([str(points)])
    flagged = str(points).replace('_points.csv', '_points_flagged.csv')
    return {int(r['point_id']): r['pose_flag'] for r in csv.DictReader(open(flagged))}


def ramp(a, b, n=40):
    return [a + (b - a) * k / (n - 1) for k in range(n)]


def test_smooth_staircase_is_all_kept(tmp_path):
    p = write_logs(tmp_path, [(0, 'up', 0, [0.5] * 40), (1, 'up', 30, ramp(0.5, 1.0)),
                              (2, 'up', 60, ramp(1.0, 1.5)), (3, 'down', 0, ramp(1.5, 0.5))])
    assert flags(p) == {0: '', 1: '', 2: '', 3: ''}


def test_switch_while_settling_flags_until_the_switch_back(tmp_path):
    # Tag set changes during point 2's settle (+3 deg in one frame) and changes back during
    # point 4's settle. Points 2 and 3 are steady but in the other state.
    p = write_logs(tmp_path, [
        (0, 'up', 0, [0.5] * 40),
        (1, 'up', 30, [1.0] * 40),
        (2, 'up', 60, [1.0] * 5 + [4.0] * 35),
        (3, 'up', 90, [4.2] * 40),
        (4, 'down', 60, [4.0] * 5 + [1.1] * 35),
        (5, 'down', 0, [0.5] * 40),
    ])
    f = flags(p)
    assert f[0] == f[1] == f[4] == f[5] == ''
    assert f[2] == 'between' and f[3] == 'between'


def test_flicker_inside_the_hold_is_flagged(tmp_path):
    flick = [1.0] * 20 + [1.0, 4.0] * 10
    p = write_logs(tmp_path, [(0, 'up', 0, [0.5] * 40), (1, 'up', 30, flick), (2, 'down', 0, [0.5] * 40)])
    f = flags(p)
    assert f[1] == 'flicker' and f[0] == f[2] == ''
