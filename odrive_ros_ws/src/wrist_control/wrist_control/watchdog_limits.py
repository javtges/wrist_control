"""
Pure limit-checking logic for odrive_watchdog_node (no ROS imports), so it can
be unit-tested without a running ROS graph.

Each ODrive axis gets an AxisMonitor that holds its latest temperatures and
current, plus time-weighted rolling averages of |current|. violations() lists
every limit the axis is currently breaking.
"""

from collections import deque
from dataclasses import dataclass, field
import math


@dataclass
class Limits:
    max_temperature_c: float = 80.0
    check_motor_temperature: bool = False
    max_current_a: float = 3.5
    # (window seconds, max average |current| in A) pairs
    avg_windows: list[tuple[float, float]] = field(
        default_factory=lambda: [(60.0, 3.0), (300.0, 2.0)]
    )
    # An average is only checked once it covers at least this much time
    min_avg_duration_s: float = 10.0
    # Data older than this counts as lost
    stale_timeout_s: float = 1.0
    # To reset a fault, temperatures must be this far below the limit
    temperature_reset_margin_c: float = 5.0


class RollingAverage:
    """Time-weighted average of a signal over the last `window_s` seconds.

    Each sample is held until the next one arrives (zero-order hold), so
    irregular message timing doesn't bias the result. Until the window has
    filled, the average covers only the time seen so far."""

    def __init__(self, window_s: float) -> None:
        self.window_s = window_s
        self._segments: deque[tuple[float, float, float]] = deque()  # (t0, t1, value)
        self._integral = 0.0
        self._duration = 0.0
        self._last: tuple[float, float] | None = None  # (t, value)

    def add(self, t: float, value: float) -> None:
        if self._last is not None:
            t0, v0 = self._last
            if t > t0:
                self._segments.append((t0, t, v0))
                self._integral += v0 * (t - t0)
                self._duration += t - t0
        self._last = (t, value)
        self._prune(t)

    def _prune(self, now: float) -> None:
        cutoff = now - self.window_s
        while self._segments and self._segments[0][1] <= cutoff:
            t0, t1, v = self._segments.popleft()
            self._integral -= v * (t1 - t0)
            self._duration -= t1 - t0
        if self._segments and self._segments[0][0] < cutoff:
            t0, t1, v = self._segments[0]
            self._integral -= v * (cutoff - t0)
            self._duration -= cutoff - t0
            self._segments[0] = (cutoff, t1, v)
        if not self._segments:
            # Re-zero so float error from add/subtract doesn't accumulate.
            self._integral = 0.0
            self._duration = 0.0

    def duration(self) -> float:
        return max(self._duration, 0.0)

    def average(self) -> float | None:
        d = self.duration()
        return self._integral / d if d > 0.0 else None


def _window_label(window_s: float) -> str:
    return f'{window_s / 60:g}-min' if window_s >= 60 else f'{window_s:g}-s'


class AxisMonitor:
    def __init__(self, limits: Limits) -> None:
        self.limits = limits
        self.fet_temperature: float | None = None
        self.motor_temperature: float | None = None
        self.current: float | None = None  # |A|, latest sample
        self.temperature_time: float | None = None
        self.current_time: float | None = None
        self.averages = [RollingAverage(w) for w, _ in limits.avg_windows]

    def update_temperature(self, t: float, fet_c: float, motor_c: float) -> None:
        self.fet_temperature = fet_c
        self.motor_temperature = motor_c
        self.temperature_time = t

    def update_current(self, t: float, amps: float) -> None:
        self.current = abs(amps)
        self.current_time = t
        # A NaN would poison the running integral for the whole window; the
        # NaN itself is reported as a violation instead.
        if math.isfinite(amps):
            for avg in self.averages:
                avg.add(t, abs(amps))

    def has_data(self) -> bool:
        return self.temperature_time is not None and self.current_time is not None

    def average_values(self) -> list[float | None]:
        return [avg.average() for avg in self.averages]

    def violations(self, now: float, resetting: bool = False) -> list[str]:
        """Every limit currently broken. With resetting=True, missing data
        also counts, and temperatures must be below the limit by the reset
        margin (so a fault can't be cleared while sitting right at 80 °C)."""
        lim = self.limits
        out: list[str] = []

        for name, last in (('temperature', self.temperature_time),
                           ('current', self.current_time)):
            if last is None:
                if resetting:
                    out.append(f'no {name} data received')
            elif now - last > lim.stale_timeout_s:
                out.append(f'no {name} data for {now - last:.1f} s')

        temp_limit = lim.max_temperature_c - (lim.temperature_reset_margin_c if resetting else 0.0)
        temps = [('FET', self.fet_temperature)]
        if lim.check_motor_temperature:
            temps.append(('motor', self.motor_temperature))
        for name, value in temps:
            if value is None:
                continue
            if not math.isfinite(value):
                out.append(f'{name} temperature reading is {value}')
            elif value > temp_limit:
                out.append(f'{name} temperature {value:.1f} °C > {temp_limit:.1f} °C')

        if self.current is not None:
            if not math.isfinite(self.current):
                out.append(f'current reading is {self.current}')
            elif self.current > lim.max_current_a:
                out.append(f'current {self.current:.2f} A > {lim.max_current_a:.2f} A')

        for avg, (window_s, limit_a) in zip(self.averages, lim.avg_windows):
            value = avg.average()
            if value is not None and avg.duration() >= lim.min_avg_duration_s and value > limit_a:
                out.append(
                    f'{_window_label(window_s)} average current {value:.3f} A > {limit_a:.3f} A'
                )
        return out
