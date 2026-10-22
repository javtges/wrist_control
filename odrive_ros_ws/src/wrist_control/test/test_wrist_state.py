"""wrist_state_publisher's aggregation, without ROS: build_state from per-source samples."""

import math

from wrist_control.wrist_state_publisher import CONTROLLER_FIELDS, ODRIVE_FIELDS, build_state


def _ctrl(**kw):
    d = {src: 0.0 for src, _ in CONTROLLER_FIELDS}
    d.update({'axis_state': 8, 'active_errors': 0})
    d.update(kw)
    return d


def _odrv(**kw):
    d = {src: 0.0 for src, _ in ODRIVE_FIELDS}
    d.update({'active_errors': 0, 'disarm_reason': 0})
    d.update(kw)
    return d


def test_empty_sources_give_nan_and_zero_columns():
    out = build_state(10.0, 6, None, None, [None] * 6, [None] * 6)
    assert len(out['extension_deg']) == 6 and all(math.isnan(v) for v in out['extension_deg'])
    assert math.isnan(out['extension_age_s']) and math.isnan(out['command_age_s'])
    assert out['axis_state'] == [0] * 6 and out['controller_errors'] == [0] * 6
    assert all(math.isnan(v) for v in out['iq_measured_a'])
    assert all(math.isnan(v) for v in out['controller_age_s'] + out['odrive_age_s'])


def test_samples_land_on_their_axis_with_their_age():
    controller = [None] * 6
    controller[3] = (9.95, _ctrl(pos_estimate=1.5, iq_measured=2.0, axis_state=8, active_errors=4))
    odrive = [None] * 6
    odrive[3] = (9.90, _odrv(bus_voltage=24.1, motor_temperature=41.0, active_errors=4, disarm_reason=1))
    out = build_state(10.0, 6, (9.99, [0, 0, 0, 360, 0, 0]), (9.0, [0, 0, 0, 360, 0, 0]), controller, odrive)
    assert out['motor_pos_turns'][3] == 1.5 and math.isnan(out['motor_pos_turns'][0])
    assert out['iq_measured_a'][3] == 2.0 and out['axis_state'][3] == 8 and out['controller_errors'][3] == 4
    assert abs(out['controller_age_s'][3] - 0.05) < 1e-9 and math.isnan(out['controller_age_s'][2])
    assert out['bus_voltage_v'][3] == 24.1 and out['motor_temperature_c'][3] == 41.0
    assert out['odrive_errors'][3] == 4 and out['disarm_reason'][3] == 1
    assert abs(out['odrive_age_s'][3] - 0.10) < 1e-9
    assert out['extension_deg'][3] == 360 and abs(out['extension_age_s'] - 0.01) < 1e-9
    assert out['command_deg'] == [0, 0, 0, 360, 0, 0] and abs(out['command_age_s'] - 1.0) < 1e-9


def test_short_or_long_joint_arrays_keep_six_columns():
    out = build_state(1.0, 6, (1.0, [1, 2]), (1.0, [1, 2, 3, 4, 5, 6, 7]), [None] * 6, [None] * 6)
    assert out['extension_deg'][:2] == [1, 2] and all(math.isnan(v) for v in out['extension_deg'][2:])
    assert out['command_deg'] == [1, 2, 3, 4, 5, 6]
