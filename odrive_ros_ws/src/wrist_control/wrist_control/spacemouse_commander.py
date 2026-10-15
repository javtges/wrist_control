#!/usr/bin/env python3
"""
spacemouse_commander — drive the HSA wrist from a 3Dconnexion SpaceMouse.

Reads the SpaceMouse (hidraw, via easyhid/pyspacemouse), converts its pose
to six joint angles with the Stewart-platform kinematics from
resource/rigid_dynamics.py, and publishes them the same way
joint_commander_gui does.

Topics published:
  /hsa_odrive_control/joint_commands  (std_msgs/Float32MultiArray)
      6 extension angles in degrees, clamped to the HSARobot action space
      ([min_angle, max_angle] from wrist.yaml, i.e. 0–360 from the physical
      zero). Only while publishing is on.
  ~/joint_preview  (std_msgs/Float32MultiArray)
      The same 6 angles, published at every tick whether or not publishing
      is on, so the output can be checked without moving the motors.

Topics subscribed:
  /odrive_axis{N}/controller_status  (odrive_can/ControllerStatus) — axis state,
      to decide whether the right button enables or disables

Services provided:
  ~/set_publishing  (std_srvs/SetBool) — turn publishing on/off

Services called:
  /hsa_odrive_control/enable, /hsa_odrive_control/disable  (std_srvs/Trigger)

SpaceMouse buttons:
  Button 0 (left)  — toggle publishing on/off
  Button 1 (right) — enable/disable the motors: disables if any axis is in
                     closed-loop control, otherwise enables. Either way
                     publishing is turned off first, so motors never jump to
                     the SpaceMouse pose the moment they are enabled.

The software zero is set only from joint_commander_gui.

Parameters:
  robot_config     (str)   — path to wrist.yaml; defaults to hsa_robots/wrist.yaml
  publish_rate     (float) — Hz, default 100 (joint_commander_gui's default)
  start_publishing (bool)  — publish from startup, default False
  max_rot          (float) — rad of rotation at full SpaceMouse deflection
  max_lin          (float) — cm of z translation at full SpaceMouse deflection
  clockwise        (bool)  — ODrives 0–5 run clockwise viewed from above,
                             (0,1), (2,3), (4,5) meeting at the top
  leg_offset       (int)   — rotate the motor numbering by this many legs
"""

import threading

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, Trigger
from odrive_can.msg import ControllerStatus

from hsa_robots import HSARobot

from . import pyspacemouse
from .stewart_kinematics import (
    MAX_LIN,
    MAX_ROT,
    clamp_to_limits,
    spacemouse_to_joint_angles,
)
from .robot_config import default_robot_config

BUTTON_TOGGLE_PUBLISH = 0
BUTTON_TOGGLE_ENABLE = 1
READ_TIMEOUT_MS = 50
AXIS_STATE_CLOSED_LOOP = 8


class SpacemouseCommander(Node):
    _TARGET = "hsa_odrive_control"

    def __init__(self, default_config: str) -> None:
        super().__init__("spacemouse_commander")

        self.declare_parameter("robot_config", default_config)
        self.declare_parameter("publish_rate", 100.0)
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
        self._low = np.asarray(self.robot.action_space["low"], dtype=float)
        self._high = np.asarray(self.robot.action_space["high"], dtype=float)

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
        self._svc_enable = self.create_client(Trigger, f"/{self._TARGET}/enable")
        self._svc_disable = self.create_client(Trigger, f"/{self._TARGET}/disable")

        # Latest ODrive axis_state per axis (None = no status received yet)
        self._axis_states: list[int | None] = [None] * self.n
        for i in range(self.n):
            self.create_subscription(
                ControllerStatus, f"/odrive_axis{i}/controller_status",
                lambda msg, idx=i: self._on_controller_status(idx, msg), 10,
            )

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
            if self._pressed(BUTTON_TOGGLE_ENABLE, buttons, old_buttons):
                self._toggle_enable()
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

    # ── Enable / disable ──────────────────────────────────────────────────────

    def _on_controller_status(self, idx: int, msg: ControllerStatus) -> None:
        with self._lock:
            self._axis_states[idx] = msg.axis_state

    def _toggle_enable(self) -> None:
        """Disable if any axis is in closed loop, otherwise enable."""
        self._set_publishing(False)
        with self._lock:
            any_enabled = any(s == AXIS_STATE_CLOSED_LOOP for s in self._axis_states)
        client, name = (
            (self._svc_disable, "disable") if any_enabled else (self._svc_enable, "enable")
        )
        if not client.service_is_ready():
            self.get_logger().warning(f"{name} service not ready.")
            return
        self.get_logger().info(f"{name} called.")
        client.call_async(Trigger.Request()).add_done_callback(
            lambda fut, name=name: self._on_enable_result(name, fut)
        )

    def _on_enable_result(self, name: str, fut) -> None:
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f"{name} failed: {e}")
            return
        log = self.get_logger().info if res.success else self.get_logger().error
        log(f"{name}: {res.message}")

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
    node = SpacemouseCommander(default_robot_config())
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
