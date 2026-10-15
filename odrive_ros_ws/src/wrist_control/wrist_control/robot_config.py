"""
Location of the wrist robot config.

hsa_robots/wrist.yaml is the single source of truth for the robot config.
Nodes resolve it from the hsa_robots package itself rather than from an
installed copy, so an edit there takes effect without rebuilding this package.
config/wrist.yaml in this package is a symlink to the same file.
"""

from pathlib import Path

import hsa_robots


def default_robot_config() -> str:
    return str(Path(hsa_robots.__file__).resolve().parent / "wrist.yaml")
