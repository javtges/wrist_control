#!/usr/bin/env python3
"""
joint_commander_gui — Tkinter + ROS2 GUI for commanding HSA wrist joints.

Publishes Float32MultiArray to /hsa_odrive_control/joint_commands and
controls ODrive parameters on the hsa_odrive_control node in real time.
Also shows each ODrive's DC bus current from /odrive_axis{N}/odrive_status.

Usage:
    ros2 run wrist_control joint_commander_gui
    ros2 run wrist_control joint_commander_gui --ros-args -p robot_config:=/path/to/wrist.yaml
"""

import math
import os
import signal
import threading
import time
from pathlib import Path

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty, Trigger
from odrive_can.msg import ODriveStatus

from hsa_robots import HSA, HSARobot

# ── ODrive option maps ────────────────────────────────────────────────────────

CONTROL_MODES = {
    "Position (3)": 3,
    "Velocity (2)": 2,
    "Torque  (1)": 1,
    "Voltage (0)": 0,
}
INPUT_MODES = {
    "Passthrough (1)": 1,
    "Trap Traj  (5)": 5,
}

# ── Canvas geometry for bar visualizations ────────────────────────────────────

CANVAS_W = 90
CANVAS_H = 190
BAR_X0, BAR_X1 = 25, 65
BAR_Y_BASE, BAR_Y_MIN = 178, 12

_CW_FILL = "#4fc3f7"
_CCW_FILL = "#f48fb1"
_MEASURED_COLOR = "#ffeb3b"


# ─────────────────────────────────────────────────────────────────────────────
# Per-joint panel (one per HSA)
# ─────────────────────────────────────────────────────────────────────────────

class JointPanel:
    """Vertical slider + bar canvas for one HSA joint."""

    def __init__(self, parent: tk.Widget, hsa: HSA, on_change=None):
        self.hsa = hsa
        self._on_change = on_change
        fill = _CW_FILL if hsa.handedness == HSA.CW else _CCW_FILL
        hand = "CW" if hsa.handedness == HSA.CW else "CCW"

        self.frame = ttk.LabelFrame(parent, text=f"J{hsa.index}  {hand}")

        # Link checkbox — when checked, this joint moves together with
        # every other checked joint.
        self.link_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            self.frame, text="Link", variable=self.link_var,
        ).pack(pady=(4, 0))

        # Bar canvas
        self.canvas = tk.Canvas(
            self.frame, width=CANVAS_W, height=CANVAS_H,
            bg="#1a1a2e", highlightthickness=0,
        )
        self.canvas.pack(padx=4, pady=(4, 0))
        self.canvas.create_line(
            BAR_X0, BAR_Y_BASE, BAR_X1, BAR_Y_BASE, fill="#555", width=1
        )
        self._bar = self.canvas.create_rectangle(
            BAR_X0, BAR_Y_BASE, BAR_X1, BAR_Y_BASE,
            fill=fill, outline="", width=0,
        )
        self._text = self.canvas.create_text(
            CANVAS_W // 2, 10, text="0.0°", fill="white", font=("Courier", 8),
        )
        self._fill = fill

        # Measured-position marker — a line overlaid on the bar showing the
        # live encoder-derived extension angle, separate from the commanded
        # (slider) position represented by the bar fill.
        self._meas_marker = self.canvas.create_line(
            BAR_X0 - 4, BAR_Y_BASE, BAR_X1 + 4, BAR_Y_BASE,
            fill=_MEASURED_COLOR, width=2, state="hidden",
        )

        # Slider — high value at top
        self._var = tk.DoubleVar(value=float(hsa.zero_point))
        ttk.Scale(
            self.frame,
            from_=hsa.max_angle, to=hsa.min_angle,
            orient="vertical", variable=self._var,
            command=self._on_drag, length=210,
        ).pack(padx=4, pady=2)

        # Numeric readout — commanded (slider) position
        self._lbl = ttk.Label(
            self.frame, text=f"{hsa.zero_point:.1f}°", font=("Courier", 9)
        )
        self._lbl.pack()

        # Numeric readout — live measured (encoder) position
        self._meas_lbl = ttk.Label(
            self.frame, text="meas: —", font=("Courier", 9),
            foreground="black",
        )
        self._meas_lbl.pack()

        # Numeric readout — live DC bus current from odrive_status
        self._ibus_lbl = ttk.Label(
            self.frame, text="Ibus: —", font=("Courier", 9),
            foreground="black",
        )
        self._ibus_lbl.pack()

        # Static range info
        ttk.Label(
            self.frame,
            text=f"{hsa.min_angle:.0f}–{hsa.max_angle:.0f}°\nzero={hsa.zero_point:.0f}°",
            font=("Courier", 7), foreground="#888",
        ).pack(pady=(0, 4))

        self._update_canvas(float(hsa.zero_point))

    # ── Public API ────────────────────────────────────────────────────────────

    def get_angle(self) -> float:
        return self._var.get()

    def set_angle(self, deg: float) -> None:
        self._var.set(float(deg))
        self._update_canvas(float(deg))

    def set_measured_angle(self, deg: float | None) -> None:
        """Update the live encoder-derived readout. None/NaN hides it."""
        if deg is None or math.isnan(deg):
            self.canvas.itemconfig(self._meas_marker, state="hidden")
            self._meas_lbl.config(text="meas: —")
            return
        frac = self._frac_for(deg)
        y = BAR_Y_BASE - frac * (BAR_Y_BASE - BAR_Y_MIN)
        self.canvas.coords(self._meas_marker, BAR_X0 - 4, y, BAR_X1 + 4, y)
        self.canvas.itemconfig(self._meas_marker, state="normal")
        self._meas_lbl.config(text=f"meas: {deg:.1f}°")

    def set_bus_current(self, amps: float | None) -> None:
        """Update the live bus-current readout. None/NaN shows a dash."""
        if amps is None or math.isnan(amps):
            self._ibus_lbl.config(text="Ibus: —")
            return
        self._ibus_lbl.config(text=f"Ibus: {amps:+.2f}A")

    # ── Internals ─────────────────────────────────────────────────────────────

    def _frac_for(self, angle: float) -> float:
        span = self.hsa.max_angle - self.hsa.min_angle
        frac = (angle - self.hsa.min_angle) / span if span > 0 else 0.0
        return max(0.0, min(1.0, frac))

    def _on_drag(self, _val: str) -> None:
        angle = self._var.get()
        self._update_canvas(angle)
        if self._on_change is not None:
            self._on_change(self, angle)

    def _update_canvas(self, angle: float) -> None:
        frac = self._frac_for(angle)
        top = BAR_Y_BASE - frac * (BAR_Y_BASE - BAR_Y_MIN)
        self.canvas.coords(self._bar, BAR_X0, top, BAR_X1, BAR_Y_BASE)
        self.canvas.itemconfig(self._text, text=f"{angle:.1f}°")
        self._lbl.config(text=f"{angle:.1f}°")


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 node — spins in a daemon thread
# ─────────────────────────────────────────────────────────────────────────────

class GUINode(Node):
    _TARGET = "hsa_odrive_control"

    def __init__(self, default_config: str) -> None:
        super().__init__("joint_commander_gui")

        self.declare_parameter("robot_config", default_config)
        cfg = self.get_parameter("robot_config").value or default_config
        self.robot = HSARobot(cfg)
        self.n = len(self.robot.hsas)

        self._pub = self.create_publisher(
            Float32MultiArray, f"/{self._TARGET}/joint_commands", 10
        )

        # Live measured joint states from hsa_odrive_control — written from
        # this node's spin thread, read from the Tk mainloop via a lock.
        self._joint_states_lock = threading.Lock()
        self._latest_joint_states: list[float] = [float("nan")] * self.n
        self.create_subscription(
            Float32MultiArray, f"/{self._TARGET}/joint_states",
            self._on_joint_states, 10,
        )

        # Live DC bus current per ODrive, from each axis's odrive_status.
        # Stored as (amps, monotonic receive time) so stale values can be
        # hidden when an ODrive stops reporting.
        self._bus_current_lock = threading.Lock()
        self._latest_bus_current: list[tuple[float, float] | None] = [None] * self.n
        for i in range(self.n):
            self.create_subscription(
                ODriveStatus, f"/odrive_axis{i}/odrive_status",
                lambda msg, idx=i: self._on_odrive_status(idx, msg), 10,
            )

        self._svc_zero = self.create_client(Empty, f"/{self._TARGET}/zero_encoders")
        self._svc_reset_zero = self.create_client(Empty, f"/{self._TARGET}/reset_zero")
        self._svc_enable = self.create_client(Trigger, f"/{self._TARGET}/enable")
        self._svc_disable = self.create_client(Trigger, f"/{self._TARGET}/disable")

        try:
            from rclpy.parameter_client import AsyncParametersClient
            self._param_client = AsyncParametersClient(self, self._TARGET)
        except Exception:
            self._param_client = None
            self.get_logger().warning(
                "AsyncParametersClient unavailable — remote param changes disabled."
            )

        self.get_logger().info(
            f"joint_commander_gui ready  →  /{self._TARGET}/joint_commands"
        )

    # ── Publish ───────────────────────────────────────────────────────────────

    def publish_angles(self, angles: list[float]) -> None:
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self._pub.publish(msg)

    # ── Measured joint states ────────────────────────────────────────────────

    def _on_joint_states(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != self.n:
            return
        with self._joint_states_lock:
            self._latest_joint_states = list(msg.data)

    def get_measured_angles(self) -> list[float]:
        with self._joint_states_lock:
            return list(self._latest_joint_states)

    # ── ODrive bus current ───────────────────────────────────────────────────

    BUS_CURRENT_STALE_S = 1.0

    def _on_odrive_status(self, idx: int, msg: ODriveStatus) -> None:
        with self._bus_current_lock:
            self._latest_bus_current[idx] = (msg.bus_current, time.monotonic())

    def get_bus_currents(self) -> list[float]:
        """Latest bus current per axis in amps; NaN if none or stale."""
        now = time.monotonic()
        with self._bus_current_lock:
            entries = list(self._latest_bus_current)
        out = []
        for entry in entries:
            if entry is None or now - entry[1] > self.BUS_CURRENT_STALE_S:
                out.append(float("nan"))
            else:
                out.append(entry[0])
        return out

    # ── Service calls (fire-and-forget) ───────────────────────────────────────

    def call_zero_encoders(self) -> None:
        if self._svc_zero.service_is_ready():
            self._svc_zero.call_async(Empty.Request())
            self.get_logger().info("zero_encoders called.")
        else:
            self.get_logger().warning("zero_encoders service not ready.")

    def call_reset_zero(self) -> None:
        if self._svc_reset_zero.service_is_ready():
            self._svc_reset_zero.call_async(Empty.Request())
            self.get_logger().info("reset_zero called.")
        else:
            self.get_logger().warning("reset_zero service not ready.")

    def call_enable(self) -> None:
        if self._svc_enable.service_is_ready():
            self._svc_enable.call_async(Trigger.Request())
            self.get_logger().info("enable called.")
        else:
            self.get_logger().warning("enable service not ready.")

    def call_disable(self) -> None:
        if self._svc_disable.service_is_ready():
            self._svc_disable.call_async(Trigger.Request())
            self.get_logger().info("disable called.")
        else:
            self.get_logger().warning("disable service not ready.")

    # ── Remote parameter changes ──────────────────────────────────────────────

    def set_remote_params(self, **kwargs) -> None:
        if self._param_client is None:
            return
        params = []
        for name, val in kwargs.items():
            if isinstance(val, int):
                params.append(Parameter(name, Parameter.Type.INTEGER, val))
            else:
                params.append(Parameter(name, Parameter.Type.DOUBLE, float(val)))
        self._param_client.set_parameters(params)


# ─────────────────────────────────────────────────────────────────────────────
# Main application
# ─────────────────────────────────────────────────────────────────────────────

class JointCommanderApp:
    """Tkinter application; receives a live GUINode for ROS2 interactions."""

    def __init__(self, root: tk.Tk, node: GUINode) -> None:
        self.root = root
        self.node = node
        self._publishing = False
        self._syncing = False
        self._running = True

        root.title(f"HSA Joint Commander  ·  {node.robot.name}")
        root.resizable(False, False)

        self._build_joint_panels()
        self._build_odrive_params()
        self._build_actions()

        root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._poll_measured()

    # ── Layout ────────────────────────────────────────────────────────────────

    def _build_joint_panels(self) -> None:
        outer = ttk.Frame(self.root, padding=4)
        outer.grid(row=0, column=0, columnspan=2, sticky="ew")
        self.panels: list[JointPanel] = []
        for hsa in self.node.robot.hsas:
            p = JointPanel(outer, hsa, on_change=self._on_slider_change)
            p.frame.grid(row=0, column=hsa.index, padx=3, pady=3, sticky="n")
            self.panels.append(p)

    def _build_odrive_params(self) -> None:
        f = ttk.LabelFrame(self.root, text="ODrive Parameters", padding=8)
        f.grid(row=1, column=0, padx=8, pady=6, sticky="nsew")

        # Control mode
        ttk.Label(f, text="Control mode:").grid(row=0, column=0, sticky="w", pady=4)
        self._ctrl_var = tk.StringVar(value=next(iter(CONTROL_MODES)))
        ctrl_cmb = ttk.Combobox(
            f, textvariable=self._ctrl_var,
            values=list(CONTROL_MODES), state="readonly", width=16,
        )
        ctrl_cmb.grid(row=0, column=1, padx=8, pady=4)
        ctrl_cmb.bind(
            "<<ComboboxSelected>>",
            lambda _: self.node.set_remote_params(
                control_mode=CONTROL_MODES[self._ctrl_var.get()]
            ),
        )

        # Input mode
        ttk.Label(f, text="Input mode:").grid(row=1, column=0, sticky="w", pady=4)
        self._inp_var = tk.StringVar(value=next(iter(INPUT_MODES)))
        inp_cmb = ttk.Combobox(
            f, textvariable=self._inp_var,
            values=list(INPUT_MODES), state="readonly", width=16,
        )
        inp_cmb.grid(row=1, column=1, padx=8, pady=4)
        inp_cmb.bind(
            "<<ComboboxSelected>>",
            lambda _: self.node.set_remote_params(
                input_mode=INPUT_MODES[self._inp_var.get()]
            ),
        )

        # Vel FF
        ttk.Label(f, text="Vel FF (turns/s):").grid(row=2, column=0, sticky="w", pady=4)
        self._vel_var = tk.DoubleVar(value=0.0)
        self._vel_lbl = ttk.Label(f, text="+0.000", width=7, font=("Courier", 10))
        self._vel_lbl.grid(row=2, column=2, padx=4)
        vel_scale = ttk.Scale(
            f, from_=-10.0, to=10.0, orient="horizontal",
            variable=self._vel_var, length=200,
            command=lambda v: self._vel_lbl.config(text=f"{float(v):+.3f}"),
        )
        vel_scale.grid(row=2, column=1, padx=8)
        vel_scale.bind(
            "<ButtonRelease-1>",
            lambda _: self.node.set_remote_params(vel_ff=self._vel_var.get()),
        )
        ttk.Button(
            f, text="0", width=2,
            command=lambda: self._reset_ff(self._vel_var, self._vel_lbl, "vel_ff"),
        ).grid(row=2, column=3, padx=2)

        # Torque FF
        ttk.Label(f, text="Torque FF (Nm):").grid(row=3, column=0, sticky="w", pady=4)
        self._trq_var = tk.DoubleVar(value=0.0)
        self._trq_lbl = ttk.Label(f, text="+0.000", width=7, font=("Courier", 10))
        self._trq_lbl.grid(row=3, column=2, padx=4)
        trq_scale = ttk.Scale(
            f, from_=-5.0, to=5.0, orient="horizontal",
            variable=self._trq_var, length=200,
            command=lambda v: self._trq_lbl.config(text=f"{float(v):+.3f}"),
        )
        trq_scale.grid(row=3, column=1, padx=8)
        trq_scale.bind(
            "<ButtonRelease-1>",
            lambda _: self.node.set_remote_params(torque_ff=self._trq_var.get()),
        )
        ttk.Button(
            f, text="0", width=2,
            command=lambda: self._reset_ff(self._trq_var, self._trq_lbl, "torque_ff"),
        ).grid(row=3, column=3, padx=2)

    def _build_actions(self) -> None:
        f = ttk.LabelFrame(self.root, text="Actions", padding=8)
        f.grid(row=1, column=1, padx=8, pady=6, sticky="nsew")

        # Publish toggle — prominent
        self._pub_btn = tk.Button(
            f,
            text="◉  NOT PUBLISHING",
            bg="#424242", fg="white",
            font=("Courier", 12, "bold"),
            relief="flat", pady=8, padx=12,
            command=self._toggle_publish,
        )
        self._pub_btn.grid(row=0, column=0, columnspan=2, padx=6, pady=(4, 10), sticky="ew")

        # Motor position buttons
        ttk.Button(
            f, text="↺  Go to Zero", width=18, command=self._go_to_zero
        ).grid(row=1, column=0, padx=6, pady=4)

        ttk.Button(
            f, text="⦿  Set Zero Here", width=18, command=self._set_zero_here
        ).grid(row=1, column=1, padx=6, pady=4)

        ttk.Button(
            f, text="⟲  Reset to ODrive Zero", width=18 * 2 + 3,
            command=self.node.call_reset_zero,
        ).grid(row=2, column=0, columnspan=2, padx=6, pady=4, sticky="ew")

        # Enable / Disable
        tk.Button(
            f, text="⚡  Enable", width=16,
            bg="#1b5e20", fg="white",
            font=("Courier", 10, "bold"), relief="flat", pady=4,
            command=self.node.call_enable,
        ).grid(row=3, column=0, padx=6, pady=4)

        tk.Button(
            f, text="✕  Disable", width=16,
            bg="#b71c1c", fg="white",
            font=("Courier", 10, "bold"), relief="flat", pady=4,
            command=self.node.call_disable,
        ).grid(row=3, column=1, padx=6, pady=4)

        # Publish rate
        rate_row = ttk.Frame(f)
        rate_row.grid(row=4, column=0, columnspan=2, pady=(10, 2))
        ttk.Label(rate_row, text="Publish rate (Hz):").pack(side="left")
        self._hz_var = tk.IntVar(value=20)
        ttk.Spinbox(
            rate_row, from_=1, to=200, textvariable=self._hz_var, width=6
        ).pack(side="left", padx=6)

    # ── Measured-state poll loop ─────────────────────────────────────────────

    MEASURED_POLL_MS = 150

    def _poll_measured(self) -> None:
        if not self._running:
            return
        for panel, angle in zip(self.panels, self.node.get_measured_angles()):
            panel.set_measured_angle(angle)
        for panel, amps in zip(self.panels, self.node.get_bus_currents()):
            panel.set_bus_current(amps)
        self.root.after(self.MEASURED_POLL_MS, self._poll_measured)

    # ── Publish loop ──────────────────────────────────────────────────────────

    def _publish_once(self) -> None:
        self.node.publish_angles([p.get_angle() for p in self.panels])

    def _publish_loop(self) -> None:
        if not self._publishing:
            return
        self._publish_once()
        hz = max(1, min(200, self._hz_var.get()))
        self.root.after(1000 // hz, self._publish_loop)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _toggle_publish(self) -> None:
        self._publishing = not self._publishing
        if self._publishing:
            self._pub_btn.config(text="◉  PUBLISHING", bg="#2e7d32")
            self._publish_loop()
        else:
            self._pub_btn.config(text="◉  NOT PUBLISHING", bg="#424242")

    def _on_slider_change(self, panel: "JointPanel", angle: float) -> None:
        """Propagate a slider move to every other 'Link'-checked joint."""
        if self._syncing or not panel.link_var.get():
            return
        self._syncing = True
        try:
            for other in self.panels:
                if other is panel or not other.link_var.get():
                    continue
                clamped = max(other.hsa.min_angle, min(other.hsa.max_angle, angle))
                other.set_angle(clamped)
        finally:
            self._syncing = False

    def _go_to_zero(self) -> None:
        """Move sliders to each joint's zero_point and publish immediately."""
        for p in self.panels:
            p.set_angle(float(p.hsa.zero_point))
        self._publish_once()

    def _set_zero_here(self) -> None:
        """Tell hsa_odrive_control to snapshot current positions as its zero."""
        self.node.call_zero_encoders()

    def _reset_ff(self, var: tk.DoubleVar, lbl: ttk.Label, param: str) -> None:
        var.set(0.0)
        lbl.config(text="+0.000")
        self.node.set_remote_params(**{param: 0.0})

    def _on_close(self) -> None:
        self._publishing = False
        self._running = False
        self.root.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        pkg_share = get_package_share_directory("wrist_control")
        default_config = os.path.join(pkg_share, "config", "wrist.yaml")
    except Exception:
        default_config = str(
            Path(__file__).parent.parent / "config" / "wrist.yaml"
        )

    node = GUINode(default_config)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    _app = JointCommanderApp(root, node)
    signal.signal(signal.SIGINT, lambda _signum, _frame: root.quit())

    try:
        root.mainloop()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
