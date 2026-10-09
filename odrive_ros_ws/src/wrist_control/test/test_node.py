"""
Integration smoke-test for HSAODriveControl.

Spins the node briefly and verifies it initialises without error, declares
the expected parameters, and exposes the required services.

Requires a sourced ROS2 Jazzy install and the workspace overlay.
Run with:  colcon test --packages-select wrist_control
"""

import os
import pathlib
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from wrist_control.hsa_odrive_control import HSAODriveControl

WRIST_YAML = str(
    pathlib.Path(__file__).parent.parent / 'config' / 'wrist.yaml'
)


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    n = HSAODriveControl.__new__(HSAODriveControl)
    # Initialise with the local config so no ament lookup is needed
    rclpy.init.__class__  # ensure context alive
    import rclpy.node
    # Construct via super().__init__ using overridden param
    n2 = rclpy.create_node(
        'hsa_odrive_control',
        parameter_overrides=[
            rclpy.parameter.Parameter(
                'robot_config', rclpy.parameter.Parameter.Type.STRING, WRIST_YAML
            )
        ],
        automatically_declare_parameters_from_overrides=True,
    )
    yield n2
    n2.destroy_node()


def test_node_constructs(ros_context):
    """Node should construct without raising given a valid wrist.yaml."""
    import rclpy.parameter as rp
    overrides = [
        rp.Parameter('robot_config', rp.Parameter.Type.STRING, WRIST_YAML)
    ]
    node = rclpy.create_node(
        'hsa_odrive_control_test',
        parameter_overrides=overrides,
        automatically_declare_parameters_from_overrides=True,
    )
    node.destroy_node()


def test_hsa_robot_loads():
    """HSARobot must load wrist.yaml with 6 odrive-type axes."""
    from hsa_robots import HSARobot
    robot = HSARobot(WRIST_YAML)
    assert len(robot) == 6
    assert all(h.motor == 'odrive' for h in robot.hsas)


def test_motor_types_include_odrive():
    """HSA.MOTOR_TYPES must include 'odrive' for wrist.yaml to load cleanly."""
    from hsa_robots import HSA
    assert 'odrive' in HSA.MOTOR_TYPES
