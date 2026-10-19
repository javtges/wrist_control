import csv
import math

import numpy as np

from camera_tracking.tag_monitor import (
    CSV_FIELDS, build_record, draw_overlay, fmt_values, record_to_csv_row)


def test_fmt_values_handles_missing_and_nan():
    assert fmt_values(None) == 'n/a'
    assert fmt_values([]) == 'n/a'
    assert 'nan' in fmt_values([1.0, math.nan])
    assert fmt_values([35.14]).strip() == '35.1'


def test_record_roundtrip_to_csv():
    rec = build_record('2026-10-12T10:00:00.000', 'lost', 3, [3, 7], 0.6,
                       [1.0] * 6, [0.5] * 6, None, '/tracking_cam/image_rect',
                       '/tmp/x.png', '/tmp/x_raw.png')
    assert rec['joint_commands_deg'] is None
    assert rec['since_seen_s'] == 0.6
    row = record_to_csv_row(rec)
    assert set(row) == set(CSV_FIELDS)
    assert row['detected_ids'] == '3 7'
    assert row['joint_commands_deg'] == ''
    assert row['joint_states_deg'].split() == ['1.0000'] * 6


def test_draw_overlay_keeps_shape_and_does_not_touch_input():
    frame = np.zeros((120, 320, 3), dtype=np.uint8)
    tags = [(0, [(10, 10), (60, 10), (60, 60), (10, 60)], (35, 35))]
    for detected in (True, False, None):
        out = draw_overlay(frame, detected, 1.2, tags, [0.0] * 6, [math.nan] * 6, None, 2, "/a/b.png", stale=detected is None)
        assert out.shape == frame.shape
        assert out.any()
    assert not frame.any()


def test_draw_overlay_states_render_distinctly():
    frame = np.zeros((120, 320, 3), dtype=np.uint8)
    tags = [(0, [(10, 10), (60, 10), (60, 60), (10, 60)], (35, 35))]
    detected = draw_overlay(frame, True, 0.0, tags, None, None, None, 0, None)
    missing = draw_overlay(frame, True, 0.3, [], None, None, None, 0, None)
    lost = draw_overlay(frame, False, 0.9, [], None, None, None, 0, None)
    assert not np.array_equal(detected, missing)
    assert not np.array_equal(missing, lost)
