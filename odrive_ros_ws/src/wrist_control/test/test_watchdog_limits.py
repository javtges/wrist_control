"""Tests for watchdog_limits (pure logic, no ROS needed) and the Estop frame packing."""

import math
import struct

import pytest

from wrist_control.watchdog_limits import AxisMonitor, Limits, RollingAverage


# ── RollingAverage ────────────────────────────────────────────────────────────

def test_average_is_time_weighted():
    avg = RollingAverage(60.0)
    avg.add(0.0, 1.0)
    avg.add(1.0, 5.0)   # 1.0 held for 1 s
    avg.add(10.0, 0.0)  # 5.0 held for 9 s
    assert avg.duration() == pytest.approx(10.0)
    assert avg.average() == pytest.approx((1.0 * 1 + 5.0 * 9) / 10)


def test_average_drops_old_data_including_partial_segment():
    avg = RollingAverage(10.0)
    avg.add(0.0, 4.0)
    avg.add(8.0, 0.0)   # 4.0 over [0, 8]
    avg.add(15.0, 0.0)  # 0.0 over [8, 15]; window is now [5, 15]
    assert avg.duration() == pytest.approx(10.0)
    assert avg.average() == pytest.approx(4.0 * 3 / 10)


def test_average_none_until_two_samples():
    avg = RollingAverage(60.0)
    assert avg.average() is None
    avg.add(0.0, 3.0)
    assert avg.average() is None


def test_average_stable_over_long_run():
    avg = RollingAverage(60.0)
    for k in range(100_000):  # ~2.8 h at 10 Hz
        avg.add(k * 0.1, 2.5)
    assert avg.duration() == pytest.approx(60.0)
    assert avg.average() == pytest.approx(2.5, rel=1e-9)


# ── AxisMonitor ───────────────────────────────────────────────────────────────

def feed(m: AxisMonitor, t0: float, t1: float, amps: float, dt: float = 0.1, temp: float = 30.0):
    t = t0
    while t <= t1 + 1e-9:
        m.update_temperature(t, temp, 0.0)
        m.update_current(t, amps)
        t += dt
    return t1


def test_no_violation_when_healthy():
    m = AxisMonitor(Limits())
    feed(m, 0.0, 400.0, 1.0)
    assert m.violations(400.0) == []


def test_temperature_limit():
    m = AxisMonitor(Limits())
    m.update_temperature(0.0, 80.0, 0.0)
    m.update_current(0.0, 0.0)
    assert m.violations(0.0) == []           # "above 80": exactly 80 is OK
    m.update_temperature(0.1, 80.5, 0.0)
    assert any('FET temperature' in v for v in m.violations(0.1))


def test_motor_temperature_only_when_enabled():
    for enabled, expect in ((False, False), (True, True)):
        m = AxisMonitor(Limits(check_motor_temperature=enabled))
        m.update_temperature(0.0, 30.0, 95.0)
        m.update_current(0.0, 0.0)
        assert any('motor temperature' in v for v in m.violations(0.0)) is expect


def test_instantaneous_current_uses_magnitude():
    m = AxisMonitor(Limits())
    m.update_temperature(0.0, 30.0, 0.0)
    m.update_current(0.0, -3.6)  # regen / reverse torque still counts
    assert any(v.startswith('current 3.60 A') for v in m.violations(0.0))


def test_one_minute_average_trips():
    m = AxisMonitor(Limits())
    feed(m, 0.0, 60.0, 3.2)  # under 3.5 A instantaneous, over 3.0 A average
    v = m.violations(60.0)
    assert any('1-min average' in x for x in v)
    assert not any(x.startswith('current ') for x in v)


def test_five_minute_average_trips_but_one_minute_does_not():
    m = AxisMonitor(Limits())
    feed(m, 0.0, 300.0, 2.5)  # under 3.0 A (1-min), over 2.0 A (5-min)
    v = m.violations(300.0)
    assert any('5-min average' in x for x in v)
    assert not any('1-min average' in x for x in v)


def test_average_not_checked_before_min_duration():
    m = AxisMonitor(Limits(min_avg_duration_s=10.0))
    feed(m, 0.0, 5.0, 3.4)
    assert m.violations(5.0) == []
    feed(m, 5.1, 10.5, 3.4)
    assert any('1-min average' in x for x in m.violations(10.5))


def test_stale_data_trips_only_after_data_was_seen():
    m = AxisMonitor(Limits(stale_timeout_s=1.0))
    assert m.violations(100.0) == []          # never seen: waiting, not a fault
    feed(m, 0.0, 1.0, 1.0)
    assert m.violations(1.5) == []
    assert any('no temperature data' in v for v in m.violations(2.5))


def test_nan_current_is_violation_and_does_not_poison_average():
    m = AxisMonitor(Limits())
    feed(m, 0.0, 20.0, 1.0)
    m.update_current(20.1, math.nan)
    assert any('current reading is nan' in v for v in m.violations(20.1))
    assert all(a is not None and math.isfinite(a) for a in m.average_values())


def test_reset_requires_data_and_temperature_margin():
    m = AxisMonitor(Limits(max_temperature_c=80.0, temperature_reset_margin_c=5.0))
    assert m.violations(0.0, resetting=True)  # no data yet
    m.update_temperature(0.0, 77.0, 0.0)
    m.update_current(0.0, 0.0)
    assert m.violations(0.0) == []                       # under the trip limit...
    assert m.violations(0.0, resetting=True)             # ...but not 5 °C under it
    m.update_temperature(0.1, 74.0, 0.0)
    assert m.violations(0.1, resetting=True) == []


# ── Estop CAN frame ───────────────────────────────────────────────────────────

def test_can_frame_layout():
    from wrist_control.odrive_watchdog_node import CMD_ESTOP, can_frame
    frame = can_frame(5, CMD_ESTOP)
    assert len(frame) == 16  # sizeof(struct can_frame)
    can_id, dlc = struct.unpack_from('=IB', frame)
    assert can_id == (5 << 5) | 0x002
    assert dlc == 0
