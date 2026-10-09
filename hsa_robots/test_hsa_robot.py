import tkinter as tk
from tkinter import ttk
import sys
import os
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
from hsa_robot import HSA, HSARobot

CANVAS_W = 300
CANVAS_H = 400
BAR_X0 = 100
BAR_X1 = 200
BAR_Y_BASE = 360
BAR_Y_MIN = 40


class HSAPanel:
    """Single-HSA display panel — replicates the original HSAGui layout."""

    def __init__(self, parent: tk.Widget, hsa: HSA):
        self.hsa = hsa
        self.frame = ttk.LabelFrame(parent, text=f"HSA {hsa.index}  [{hsa.handedness}]")

        self._build_config_display()
        self._build_canvas()
        self._build_slider_frame()
        self._build_readout_frame()
        self._init_display()

    def _build_config_display(self):
        f = ttk.Frame(self.frame)
        f.grid(row=0, column=0, columnspan=2, padx=6, pady=4, sticky="ew")
        ttk.Label(
            f,
            text=f"min: {self.hsa.min_angle}°    max: {self.hsa.max_angle}°    zero: {self.hsa.zero_point}°",
            font=("Courier", 10),
        ).pack()

    def _build_canvas(self):
        self.canvas = tk.Canvas(self.frame, width=CANVAS_W, height=CANVAS_H, bg="#1e1e2e")
        self.canvas.grid(row=1, column=0, padx=10, pady=8)
        self.canvas.create_line(BAR_X0, BAR_Y_BASE, BAR_X1, BAR_Y_BASE, fill="#888", width=2)
        self.bar_rect = self.canvas.create_rectangle(
            BAR_X0, BAR_Y_BASE, BAR_X1, BAR_Y_BASE,
            fill="#4fc3f7", outline="#81d4fa", width=2,
        )
        self.ext_canvas_label = self.canvas.create_text(
            CANVAS_W // 2, 20, text="ext: 0.0°", fill="white", font=("Courier", 11)
        )

    def _build_slider_frame(self):
        frame = ttk.LabelFrame(self.frame, text="Motor Angle")
        frame.grid(row=1, column=1, padx=10, pady=8, sticky="ns")

        self.motor_slider_var = tk.DoubleVar(value=0.0)
        self.slider = ttk.Scale(
            frame, from_=self.hsa.min_angle, to=self.hsa.max_angle,
            orient="vertical", variable=self.motor_slider_var,
            command=self._on_slider, length=320,
        )
        self.slider.grid(row=0, column=0, padx=12, pady=8)

        self.slider_label = ttk.Label(frame, text="0.0°", font=("Courier", 11))
        self.slider_label.grid(row=1, column=0, pady=4)

        ttk.Button(frame, text="Reset", command=self._do_reset).grid(row=2, column=0, pady=8)

    def _build_readout_frame(self):
        frame = ttk.LabelFrame(self.frame, text="State")
        frame.grid(row=2, column=0, columnspan=2, padx=10, pady=8, sticky="ew")

        ttk.Label(frame, text="Motor angle:").grid(row=0, column=0, sticky="w", padx=6)
        self.motor_label = ttk.Label(frame, text="—", font=("Courier", 13, "bold"))
        self.motor_label.grid(row=0, column=1, sticky="w", padx=4)

        ttk.Label(frame, text="Extension angle:").grid(row=0, column=2, sticky="w", padx=6)
        self.ext_label = ttk.Label(frame, text="—", font=("Courier", 13, "bold"))
        self.ext_label.grid(row=0, column=3, sticky="w", padx=4)

    def _init_display(self):
        zero_motor = self.hsa.move(self.hsa.zero_point)
        self.motor_slider_var.set(zero_motor)
        self._update_display(zero_motor)

    def _motor_to_extension(self, motor_angle: float) -> float:
        if self.hsa.handedness == HSA.CW:
            return motor_angle
        return self.hsa.max_angle - motor_angle

    def _on_slider(self, value):
        motor = float(value)
        self.hsa.set_state(motor)
        self._update_display(motor)

    def _do_reset(self):
        self.hsa.reset()
        motor = self.hsa.get_state()
        self.motor_slider_var.set(motor)
        self._update_display(motor)

    def _update_display(self, motor_angle: float):
        ext = self._motor_to_extension(motor_angle)
        span = self.hsa.max_angle - self.hsa.min_angle
        frac = (ext - self.hsa.min_angle) / span if span > 0 else 0.0
        frac = max(0.0, min(1.0, frac))

        bar_top = BAR_Y_BASE - frac * (BAR_Y_BASE - BAR_Y_MIN)
        self.canvas.coords(self.bar_rect, BAR_X0, bar_top, BAR_X1, BAR_Y_BASE)

        r = int(79 + (0 - 79) * frac)
        g = int(195 + (230 - 195) * frac)
        b = int(247 + (118 - 247) * frac)
        self.canvas.itemconfig(self.bar_rect, fill=f"#{r:02x}{g:02x}{b:02x}")

        self.canvas.itemconfig(self.ext_canvas_label, text=f"ext: {ext:.1f}°")
        self.motor_label.config(text=f"{motor_angle:.1f}°")
        self.ext_label.config(text=f"{ext:.1f}°")
        self.slider_label.config(text=f"{motor_angle:.1f}°")


class HSARobotGui:
    def __init__(self, root: tk.Tk, config_path: str):
        robot_name = Path(config_path).stem
        robot = HSARobot(config_path)

        root.title(f"HSA Robot Tester — {robot_name}")
        ttk.Label(root, text=robot_name, font=("Courier", 18, "bold")).grid(
            row=0, column=0, columnspan=len(robot.hsas), pady=(10, 4)
        )

        for hsa in robot.hsas:
            panel = HSAPanel(root, hsa)
            panel.frame.grid(row=1, column=hsa.index, padx=8, pady=8, sticky="n")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python test_hsa_robot.py <robot_config.yaml>")
        sys.exit(1)
    root = tk.Tk()
    HSARobotGui(root, sys.argv[1])
    root.mainloop()
