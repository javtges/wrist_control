"""
Unit tests for persisting zero offsets in the robot YAML (hsa_robots).

Works on a copy of wrist.yaml in a temp directory; never touches the real one.
"""

import os
import pathlib
import shutil

import pytest
import yaml

from hsa_robots import HSARobot
from hsa_robots.hsa_robot import write_zero_offsets

WRIST_YAML = pathlib.Path(__file__).parent.parent / 'config' / 'wrist.yaml'


@pytest.fixture
def cfg(tmp_path):
    real = tmp_path / 'real.yaml'
    shutil.copy(os.path.realpath(WRIST_YAML), real)
    link = tmp_path / 'wrist.yaml'
    link.symlink_to('real.yaml')
    return link


def test_package_config_is_symlink_to_hsa_robots():
    assert WRIST_YAML.is_symlink()
    assert pathlib.Path(os.path.realpath(WRIST_YAML)).parent.name == 'hsa_robots'


def test_zero_offset_loaded(cfg):
    robot = HSARobot(str(cfg))
    expected = [float(e.get('zero_offset', 0.0)) for e in yaml.safe_load(cfg.read_text())['hsas']]
    assert [h.zero_offset for h in robot.hsas] == expected


def test_save_round_trips_and_keeps_symlink(cfg):
    robot = HSARobot(str(cfg))
    offsets = [-0.0333, 0.0444, 0.1111, -0.0111, 0.0222, 1.25]
    robot.save_zero_offsets(offsets)
    assert cfg.is_symlink()
    assert [h.zero_offset for h in robot.hsas] == offsets
    assert [h.zero_offset for h in HSARobot(str(cfg)).hsas] == offsets


def test_save_changes_only_zero_offset(cfg):
    before_text = cfg.read_text()
    before = yaml.safe_load(before_text)
    HSARobot(str(cfg)).save_zero_offsets([0.5] * len(before['hsas']))
    after_text = cfg.read_text()
    after = yaml.safe_load(after_text)
    for old, new in zip(before['hsas'], after['hsas']):
        assert {k: v for k, v in old.items() if k != 'zero_offset'} == \
               {k: v for k, v in new.items() if k != 'zero_offset'}
    # comments and every non-zero_offset line survive
    keep = [line for line in before_text.splitlines() if 'zero_offset:' not in line or line.startswith('#')]
    assert keep == [line for line in after_text.splitlines() if 'zero_offset:' not in line or line.startswith('#')]


def test_inserts_when_missing(tmp_path):
    p = tmp_path / 'r.yaml'
    p.write_text('# c\nhsas:\n  - handedness: CW\n    zero_point: 0.0\n    index: 0\n'
                 '  - handedness: CCW\n    index: 1\n')
    write_zero_offsets(str(p), [0.1, -0.2])
    data = yaml.safe_load(p.read_text())
    assert [h['zero_offset'] for h in data['hsas']] == [0.1, -0.2]
    assert p.read_text().startswith('# c\n')


def test_wrong_count_rejected(cfg):
    with pytest.raises(ValueError):
        HSARobot(str(cfg)).save_zero_offsets([0.0])
