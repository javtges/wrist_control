#!/usr/bin/env python3
"""
spacemouse_commander — drive the HSA wrist from a 3Dconnexion SpaceMouse.

Reads the SpaceMouse (hidraw, via easyhid/pyspacemouse), converts its pose
to six joint angles with the Stewart-platform kinematics from
resource/rigid_dynamics.py, and publishes them the same way
joint_commander_gui does.

Topics published:
  /hsa_odrive_control/joint_commands  (std_msgs/Float32MultiArray)
      6 extension angles in degrees, clamped to each joint's
      [min_angle, max_angle] from wrist.yaml. Only while publishing is on.
  ~/joint_preview  (std_msgs/Float32MultiArray)
      The same 6 angles, published at every tick whether or not publishing
      is on, so the output can be checked without moving the motors.

Services provided:
  ~/set_publishing  (std_srvs/SetBool) — turn publishing on/off

SpaceMouse buttons:
  Button 0 (left)  — toggle publishing on/off
  Button 1 (right) — Go to Zero: publish every joint's zero_point once,
                     then turn publishing off

Parameters:
  robot_config     (str)   — path to wrist.yaml; defaults to package config/wrist.yaml
  publish_rate     (float) — Hz, default 20 (joint_commander_gui's default)
  start_publishing (bool)  — publish from startup, default False
  max_rot          (float) — rad of rotation at full SpaceMouse deflection
  max_lin          (float) — cm of z translation at full SpaceMouse deflection
  clockwise        (bool)  — ODrives 0–5 run clockwise viewed from above,
                             (0,1), (2,3), (4,5) meeting at the top
  leg_offset       (int)   — rotate the motor numbering by this many legs
"""

import os
import threading
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool

from hsa_robots import HSARobot

from . import pyspacemouse
from .stewart_kinematics import (
    MAX_LIN,
    MAX_ROT,
    clamp_to_limits,
    spacemouse_to_joint_angles,
)

BUTTON_TOGGLE_PUBLISH = 0
BUTTON_GO_TO_ZERO = 1
READ_TIMEOUT_MS = 50


class SpacemouseCommander(Node):
    _TARGET = "hsa_odrive_control"

    def __init__(self, default_config: str) -> None:
        super().__init__("spacemouse_commander")

        self.declare_parameter("robot_config", default_config)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("start_publishing", False)
        self.declare_parameter("max_rot", float(MAX_ROT))
        self.declare_parameter("max_lin", float(MAX_LIN))
        self.declare_parameter("clockwise", True)
        self.declare_parameter("leg_offset", 0)

        cfg = self.get_parameter("robot_config").value or default_config
        self.robot = HSARobot(cfg)
        self.n = len(self.robot.hsas)
        if self.n != 6:
            raise ValueError(f"spacemouse_commander needs 6 joints, {cfg} has {self.n}")
        self._low = np.array([h.min_angle for h in self.robot.hsas], dtype=float)
        self._high = np.array([h.max_angle for h in self.robot.hsas], dtype=float)
        self._zero = [float(h.zero_point) for h in self.robot.hsas]

        self._max_rot = float(self.get_parameter("max_rot").value)
        self._max_lin = float(self.get_parameter("max_lin").value)
        self._clockwise = bool(self.get_parameter("clockwise").value)
        self._leg_offset = int(self.get_parameter("leg_offset").value)

        self._cmd_pub = self.create_publisher(
            Float32MultiArray, f"/{self._TARGET}/joint_commands", 10
        )
        self._preview_pub = self.create_publisher(
            Float32MultiArray, "~/joint_preview", 10
        )
        self.create_service(SetBool, "~/set_publishing", self._svc_set_publishing)

        # Latest SpaceMouse state (pyspacemouse namedtuple), written by the
        # reader thread and read by the publish timer.
        self._lock = threading.Lock()
        self._state = None
        self._publishing = False
        self._device_ok = False
        self._stop = threading.Event()

        self._dev = pyspacemouse.open()
        if self._dev is None:
            raise RuntimeError("No SpaceMouse found (check /dev/hidraw* permissions)")
        self._device_ok = True
        # The SpaceMouse sends nothing while untouched, so start from the
        # driver's all-zero (puck at rest) state.
        self._state = self._dev.tuple_state
        self.get_logger().info(self._dev.describe_connection())

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        rate = max(1.0, min(200.0, float(self.get_parameter("publish_rate").value)))
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f"spacemouse_commander ready  →  /{self._TARGET}/joint_commands at {rate:g} Hz "
            f"(clockwise={self._clockwise}, leg_offset={self._leg_offset})"
        )
        self._set_publishing(bool(self.get_parameter("start_publishing").value))

    # ── SpaceMouse ────────────────────────────────────────────────────────────

    def _read_loop(self) -> None:
        # Timed reads (rather than pyspacemouse's blocking read()) so the
        # thread notices shutdown and stops before the device is closed.
        old_buttons = [0] * len(self._dev.tuple_state.buttons)
        while not self._stop.is_set():
            try:
                data = self._dev.device.read(64, timeout=READ_TIMEOUT_MS)
            except Exception as e:  # device unplugged or read error
                self.get_logger().error(f"SpaceMouse read failed: {e}")
                with self._lock:
                    self._device_ok = False
                self._set_publishing(False)
                return
            if not data:
                continue
            self._dev.process(data)
            state = self._dev.tuple_state
            with self._lock:
                self._state = state

            buttons = list(state.buttons)
            if self._pressed(BUTTON_TOGGLE_PUBLISH, buttons, old_buttons):
                with self._lock:
                    publishing = self._publishing
                self._set_publishing(not publishing)
            if self._pressed(BUTTON_GO_TO_ZERO, buttons, old_buttons):
                self._go_to_zero()
            old_buttons = buttons

    @staticmethod
    def _pressed(idx: int, buttons: list[int], old_buttons: list[int]) -> bool:
        return idx < len(buttons) and bool(buttons[idx]) and not old_buttons[idx]

    def _joint_angles(self) -> list[float] | None:
        with self._lock:
            s = self._state
        if s is None:
            return None
        # axis_scale makes full deflection slightly exceed ±1; clamp to the
        # [-1, 1] range rigid_dynamics.py was written for.
        axes = [float(np.clip(v, -1.0, 1.0)) for v in (s.x, s.y, s.z, s.roll, s.pitch, s.yaw)]
        angles = spacemouse_to_joint_angles(
            *axes,
            max_rot=self._max_rot, max_lin=self._max_lin,
            clockwise=self._clockwise, leg_offset=self._leg_offset,
        )
        return [float(a) for a in clamp_to_limits(angles, self._low, self._high)]

    # ── Publishing ────────────────────────────────────────────────────────────

    def _on_timer(self) -> None:
        angles = self._joint_angles()
        if angles is None:
            return
        msg = Float32MultiArray()
        msg.data = angles
        self._preview_pub.publish(msg)
        with self._lock:
            publishing = self._publishing and self._device_ok
        if publishing:
            self._cmd_pub.publish(msg)

    def _set_publishing(self, on: bool) -> bool:
        with self._lock:
            if on and not self._device_ok:
                return False
            changed = on != self._publishing
            self._publishing = on
        if changed:
            self.get_logger().info("PUBLISHING" if on else "NOT PUBLISHING")
        return True

    def _go_to_zero(self) -> None:
        """Publish every joint's zero_point once and stop publishing, like
        joint_commander_gui's Go to Zero followed by toggling publishing off."""
        self._set_publishing(False)
        msg = Float32MultiArray()
        msg.data = list(self._zero)
        self._cmd_pub.publish(msg)
        self.get_logger().info(f"Go to Zero: {self._zero}")

    def _svc_set_publishing(self, request: SetBool.Request, response: SetBool.Response):
        response.success = self._set_publishing(request.data)
        if response.success:
            response.message = "PUBLISHING" if request.data else "NOT PUBLISHING"
        else:
            response.message = "SpaceMouse not connected; publishing stays off"
        return response

    def destroy_node(self) -> None:
        with self._lock:
            self._publishing = False
        self._stop.set()
        self._reader.join(timeout=1.0)
        try:
            self._dev.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        pkg_share = get_package_share_directory("wrist_control")
        default_config = os.path.join(pkg_share, "config", "wrist.yaml")
    except Exception:
        default_config = str(Path(__file__).parent.parent / "config" / "wrist.yaml")

    node = SpacemouseCommander(default_config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
