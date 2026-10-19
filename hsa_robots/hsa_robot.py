import os
import re
import tempfile

import numpy as np
import yaml
from pathlib import Path


class HSA:
    CW = "CW"
    CCW = "CCW"

    MOTOR_TYPES = {"servo", "dynamixel", "bldc", "odrive"}

    def __init__(
        self,
        index: int = 0,
        handedness: str = CW,
        motor: str = "servo",
        min_angle_extension: float = 0.0,
        max_angle_extension: float = 360.0,
        zero_point: float = 0.0,
        gear_ratio: float = 1.0,
        zero_offset: float = 0.0,
    ):
        if motor not in self.MOTOR_TYPES:
            raise ValueError(f"Unknown motor type '{motor}'. Must be one of {self.MOTOR_TYPES}")
        self.index = index
        self.handedness = handedness
        self.motor = motor
        self.min_angle = min_angle_extension
        self.max_angle = max_angle_extension
        self.zero_point = zero_point
        self.gear_ratio = gear_ratio
        # Software zero, in motor turns: raw motor position + zero_offset =
        # motor position measured from the physical zero.
        self.zero_offset = zero_offset
        self._state = zero_point

    def reset(self) -> float:
        # self._state = self.zero_point
        self._state = self.move(self.zero_point)
        return self._state

    def move(self, angle: float) -> float:
        """Return motor angle for a given extension angle, accounting for handedness."""
        if self.handedness == self.CW:
            motor_angle = angle
        else:
            motor_angle = self.max_angle - angle
        clamped = max(self.min_angle, min(self.max_angle, motor_angle))
        self._state = clamped
        return self._state

    def set_state(self, angle: float) -> None:
        self._state = angle

    def get_state(self) -> float:
        return self._state


class HSARobot:
    """Gym-style wrapper around a list of HSA actuators defined by a YAML config."""

    def __init__(self, config_path: str):
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        self.name = Path(config_path).stem
        self.config_path = str(config_path)

        self.hsas: list[HSA] = []
        for list_pos, entry in enumerate(cfg["hsas"]):
            yaml_index = int(entry.get("index", list_pos))
            if yaml_index != list_pos:
                raise ValueError(
                    f"HSA index mismatch at list position {list_pos}: YAML says index={yaml_index}"
                )
            self.hsas.append(
                HSA(
                    index=yaml_index,
                    handedness=entry.get("handedness", HSA.CW),
                    motor=entry.get("motor", "servo"),
                    min_angle_extension=float(entry.get("min_angle", 0.0)),
                    max_angle_extension=float(entry.get("max_angle", 360.0)),
                    zero_point=float(entry.get("zero_point", 0.0)),
                    gear_ratio=float(entry.get("gear_ratio", 1.0)),
                    zero_offset=float(entry.get("zero_offset", 0.0)),
                )
            )

        self.control = [h.motor for h in self.hsas]

        n = len(self.hsas)
        low = np.array([h.min_angle for h in self.hsas], dtype=np.float32)
        high = np.array([h.max_angle for h in self.hsas], dtype=np.float32)
        self.action_space = {"low": low, "high": high, "shape": (n,)}
        self.observation_space = {"low": low, "high": high, "shape": (n,)}

    def reset(self) -> np.ndarray:
        """Reset all HSAs to their zero points and return motor angles."""
        return np.array([h.reset() for h in self.hsas], dtype=np.float32)

    def step(self, extension_angles: np.ndarray) -> tuple[np.ndarray, float, bool, dict]:
        """
        Convert extension angles to motor commands.

        Returns (motor_angles, reward, done, info) following gym convention.
        Reward and done are stubs — wire in task logic as needed.
        """
        motor_angles = np.array(
            [h.move(float(a)) for h, a in zip(self.hsas, extension_angles)],
            dtype=np.float32,
        )
        return motor_angles, 0.0, False, {}

    def save_zero_offsets(self, offsets) -> None:
        """Write each HSA's zero_offset back to this robot's YAML config (and
        update the in-memory HSAs). Only the zero_offset lines change."""
        offsets = [float(o) for o in offsets]
        if len(offsets) != len(self.hsas):
            raise ValueError(f"expected {len(self.hsas)} offsets, got {len(offsets)}")
        write_zero_offsets(self.config_path, offsets)
        for h, o in zip(self.hsas, offsets):
            h.zero_offset = o

    def get_states(self) -> np.ndarray:
        return np.array([h.get_state() for h in self.hsas], dtype=np.float32)

    def __len__(self) -> int:
        return len(self.hsas)


_ENTRY_RE = re.compile(r"^(\s*)-\s+\S")
_ZERO_OFFSET_RE = re.compile(r"^(\s*)zero_offset\s*:.*?(\r?\n)?$")
_ZERO_POINT_RE = re.compile(r"^(\s*)zero_point\s*:")


def write_zero_offsets(config_path: str, offsets: list[float]) -> None:
    """Set `zero_offset` on each `hsas` entry of a robot YAML file.

    Edits the text in place so comments and layout survive: an existing
    zero_offset line is rewritten, otherwise one is inserted after zero_point
    (or at the end of the entry). The result is re-parsed and checked to
    differ from the original only in zero_offset before it atomically
    replaces the file (following symlinks to the real file).
    """
    real_path = os.path.realpath(config_path)
    with open(real_path, "r") as f:
        text = f.read()
    before = yaml.safe_load(text)
    n = len(before["hsas"])
    if len(offsets) != n:
        raise ValueError(f"{config_path} has {n} HSAs, got {len(offsets)} offsets")

    lines = text.splitlines(keepends=True)
    hsas_line = next(i for i, l in enumerate(lines) if re.match(r"^hsas\s*:", l))
    starts = [i for i in range(hsas_line + 1, len(lines)) if _ENTRY_RE.match(lines[i])]
    if len(starts) != n:
        raise ValueError(f"could not locate the {n} hsas entries in {config_path}")

    # Edit from the last entry back so earlier line indices stay valid.
    for k in reversed(range(n)):
        start = starts[k]
        end = starts[k + 1] if k + 1 < n else len(lines)
        value = f"{round(offsets[k], 6)!r}"
        block = range(start, end)
        existing = next((i for i in block if _ZERO_OFFSET_RE.match(lines[i])), None)
        if existing is not None:
            indent = _ZERO_OFFSET_RE.match(lines[existing]).group(1)
            lines[existing] = f"{indent}zero_offset: {value}\n"
            continue
        dash = _ENTRY_RE.match(lines[start])
        indent = " " * (len(dash.group(1)) + 2)
        anchor = next((i for i in block if _ZERO_POINT_RE.match(lines[i])), None)
        if anchor is None:
            # last non-blank line of the entry
            anchor = max(i for i in block if lines[i].strip())
        else:
            indent = _ZERO_POINT_RE.match(lines[anchor]).group(1) or indent
        lines.insert(anchor + 1, f"{indent}zero_offset: {value}\n")

    new_text = "".join(lines)
    after = yaml.safe_load(new_text)
    for k, (old, new) in enumerate(zip(before["hsas"], after["hsas"])):
        if abs(float(new.get("zero_offset")) - round(offsets[k], 6)) > 1e-9:
            raise RuntimeError(f"zero_offset edit failed for entry {k}")
        rest_old = {key: v for key, v in old.items() if key != "zero_offset"}
        rest_new = {key: v for key, v in new.items() if key != "zero_offset"}
        if rest_old != rest_new:
            raise RuntimeError(f"zero_offset edit changed other fields of entry {k}")
    if {k: v for k, v in before.items() if k != "hsas"} != {k: v for k, v in after.items() if k != "hsas"}:
        raise RuntimeError("zero_offset edit changed fields outside hsas")

    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(real_path), prefix=".wrist_yaml_")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(new_text)
        os.chmod(tmp, os.stat(real_path).st_mode & 0o777)
        os.replace(tmp, real_path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
