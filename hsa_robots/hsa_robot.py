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

    def get_states(self) -> np.ndarray:
        return np.array([h.get_state() for h in self.hsas], dtype=np.float32)

    def __len__(self) -> int:
        return len(self.hsas)
