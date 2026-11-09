"""Excite the wrist with its base flat on the table and log (command, measured, pose).

Underactuated ch. 18 ("System Identification"): the data matrix must be
parametrically exciting, and quasi-static vs dynamic data answer different
questions.  Two families of excitation are provided:

  static   hold-and-settle setpoints (kinematic / hysteresis fit).  Like
           soft_stewart's workspace.py: a shuffled grid or random points, each
           followed by a return to 0 so every sample starts from the same
           HSA history; each point is averaged after it settles.
  dynamic  a continuous multisine / chirp on all six legs (ARX / linear fit).

Every pose sample (~20 Hz from /platform_pose) is logged with the latest
/hsa/wrist/state joined on receive time; settled samples of static points go
to a second "points" CSV.  Commands are rate-limited and clamped, and the
node returns to 0 deg on exit.

Pre-flight (doc/sysid.md): CAN up, enable, zero_encoders, command 0,
/set_platform_zero, plate mount OK.
"""
from __future__ import annotations

import csv
import math
import os
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from hsa_wrist_msgs.msg import WristState
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32MultiArray

try:
    from camera_tracking import platform_geometry as pg
except ImportError:  # offline tooling can recompute from the raw pose
    pg = None

N = 6
CMD_TOPIC = '/hsa_odrive_control/joint_commands'
GUARD_WINDOW_S = 1.0  # traffic window for detecting another active command publisher
GUARD_SLACK = 10      # extra messages tolerated per window (one-shot GUI buttons, jitter)
STATE_COLS = ['t', 't_pose', 'phase', 'point_id', 'tag_visible', 'mount_ok', 'n_tags', 'tag_mask', 'fit_rms_px',
              'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw',
              'z_mm', 'x_mm', 'y_mm', 'lateral_mm', 'tilt_deg', 'tilt_dir_deg', 'twist_deg']
STATE_COLS += [f'cmd{i}' for i in range(N)] + [f'meas{i}' for i in range(N)] + [f'motor_turns{i}' for i in range(N)]
STATE_COLS += [f'vel{i}' for i in range(N)] + [f'torque{i}' for i in range(N)] + [f'iq{i}' for i in range(N)]
POINT_COLS = ['point_id', 'direction', 'n', 'z_mm', 'x_mm', 'y_mm', 'lateral_mm', 'tilt_deg', 'tilt_dir_deg',
              'twist_deg', 'z_std_mm', 'lateral_std_mm', 'tilt_std_deg',
              'tag_mask', 'tag_mask_changes', 'fit_rms_px']
POINT_COLS += [f'cmd{i}' for i in range(N)] + [f'meas{i}' for i in range(N)] + [f'motor_turns{i}' for i in range(N)]


def shuffled_grid(levels: np.ndarray, rng: np.random.Generator, n_max: int | None) -> np.ndarray:
    """All level combinations on six legs, shuffled (soft_stewart make_meshgrid)."""
    g = np.stack(np.meshgrid(*([levels] * N), indexing='ij'), -1).reshape(-1, N)
    rng.shuffle(g)
    return g[:n_max] if n_max else g


def random_points(lo: float, hi: float, n: int, rng: np.random.Generator) -> np.ndarray:
    return rng.uniform(lo, hi, (n, N))


def sweeps(lo: float, hi: float, levels: int) -> tuple[np.ndarray, np.ndarray]:
    """Monotone staircases up then down (no zero reset) for all six, each leg, each
    adjacent pair: the only way to see hysteresis (camera_tracking.md 60-deg sweep).
    Returns setpoints and 'up'/'down' labels."""
    ramp = np.linspace(lo, hi, levels)
    sets = [list(range(N))] + [[i] for i in range(N)] + [[i, (i + 1) % N] for i in range(N)]
    seq, direction = [], []
    for legs in sets:
        for v in ramp:
            q = np.full(N, lo); q[legs] = v; seq.append(q); direction.append('up')
        for v in ramp[-2::-1]:
            q = np.full(N, lo); q[legs] = v; seq.append(q); direction.append('down')
    return np.array(seq), np.array(direction)


def multisine(t: float, lo: float, hi: float, freqs: list[float], phases: np.ndarray) -> np.ndarray:
    """Sum of sines per leg with leg-distinct phases.  Centre (lo+hi)/2, span (hi-lo)/2.
    Distinct phases keep the six inputs linearly independent (persistent excitation)."""
    c, a = 0.5 * (hi + lo), 0.5 * (hi - lo) / len(freqs)
    u = np.zeros(N)
    for k, f in enumerate(freqs):
        u += a * np.sin(2 * math.pi * f * t + phases[k])
    return c + u


def chirp(t: float, lo: float, hi: float, f0: float, f1: float, T: float, phases: np.ndarray) -> np.ndarray:
    """Linear chirp f0->f1 over T seconds on all legs, phases per leg."""
    f = f0 + (f1 - f0) * (t % T) / T
    ph = 2 * math.pi * (f0 * (t % T) + 0.5 * (f1 - f0) * (t % T) ** 2 / T)
    return 0.5 * (hi + lo) + 0.5 * (hi - lo) * np.sin(ph + phases)


class SysIdExcite(Node):
    def __init__(self) -> None:
        super().__init__('sysid_excite')
        p = self.declare_parameter
        p('mode', 'static')            # static | dynamic
        p('pattern', 'grid')           # static: grid | random | sweep ; dynamic: multisine | chirp
        p('min_deg', 0.0)
        p('max_deg', 120.0)            # linear region ends ~120 deg (camera_tracking.md)
        p('grid_levels', 4)
        p('n_points', 300)             # cap for grid, count for random
        p('zero_between', True)        # return to 0 between static points (hysteresis reset)
        p('settle_s', 3.0)
        p('avg_s', 0.5)
        p('rate_deg_s', 60.0)          # command slew limit
        p('cmd_rate', 50.0)
        p('freqs', [0.05, 0.13, 0.31, 0.7])  # multisine Hz (keep below wrist bandwidth)
        p('chirp_f0', 0.02)
        p('chirp_f1', 1.0)
        p('chirp_T', 120.0)
        p('duration_s', 600.0)         # dynamic mode
        p('seed', 0)
        p('start_point', 0)            # resume: skip point ids below this (same seed = same order)
        p('require_zero', True)
        p('require_mount_ok', True)
        p('out_dir', os.path.expanduser('~/.local/share/wrist_sysid'))
        g = lambda k: self.get_parameter(k).value  # noqa: E731

        self.mode, self.pattern = str(g('mode')), str(g('pattern'))
        self.lo, self.hi = float(g('min_deg')), float(g('max_deg'))
        self.settle, self.avg_s = float(g('settle_s')), float(g('avg_s'))
        self.slew = float(g('rate_deg_s')) / float(g('cmd_rate'))
        self.rng = np.random.default_rng(int(g('seed')))
        self.phases = self.rng.uniform(0, 2 * math.pi, (len(g('freqs')), N)) if self.pattern == 'multisine' \
            else self.rng.uniform(0, 2 * math.pi, N)

        if self.mode == 'static':
            if self.pattern == 'sweep':
                pts, direction = sweeps(self.lo, self.hi, int(g('grid_levels')))
            elif self.pattern == 'grid':
                pts = shuffled_grid(np.linspace(self.lo, self.hi, int(g('grid_levels'))), self.rng, int(g('n_points')))
            else:
                pts = random_points(self.lo, self.hi, int(g('n_points')), self.rng)
            if self.pattern == 'sweep':
                seq, ids = pts, np.arange(len(pts))
            elif bool(g('zero_between')):
                seq = np.zeros((2 * len(pts), N)); seq[0::2] = pts
                ids = np.repeat(np.arange(len(pts)), 2)
                direction = np.tile(['up', 'down'], len(pts))
            else:
                seq, ids, direction = pts, np.arange(len(pts)), np.array(['up'] * len(pts))
            start = int(g('start_point'))
            if start > 0:
                keep = np.asarray(ids) >= start
                seq, ids, direction = seq[keep], np.asarray(ids)[keep], np.asarray(direction)[keep]
                self.get_logger().info(f'resuming at point {start}: {len(seq)} setpoints left')
            self.seq, self.ids, self.direction = seq, ids, direction
            self.get_logger().info(f'static {self.pattern}: {len(pts)} points, {len(seq)} setpoints, '
                                   f'~{len(seq) * (self.settle + self.avg_s) / 60:.1f} min')
        self.k = 0                      # setpoint index
        self.t_set = None               # time current setpoint was reached
        self.cmd = np.zeros(N)
        self.target = np.zeros(N)
        self.state: WristState | None = None
        self.tag_visible = False
        self.mount_ok = not bool(g('require_mount_ok'))
        self.zero_set = not bool(g('require_zero'))
        self.window: deque = deque()
        self.done = False
        self.t0 = time.monotonic()

        os.makedirs(str(g('out_dir')), exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
        tag = f'{self.mode}_{self.pattern}'
        self.f_state = open(os.path.join(str(g('out_dir')), f'sysid_{tag}_{stamp}_samples.csv'), 'w', newline='')
        self.w_state = csv.writer(self.f_state); self.w_state.writerow(STATE_COLS)
        self.f_pts = open(os.path.join(str(g('out_dir')), f'sysid_{tag}_{stamp}_points.csv'), 'w', newline='')
        self.w_pts = csv.writer(self.f_pts); self.w_pts.writerow(POINT_COLS)
        self.get_logger().info(f'logging to {self.f_state.name}')

        self.pub = self.create_publisher(Float32MultiArray, CMD_TOPIC, 10)
        self.started = False            # first motion sent; another publisher after this aborts
        self.aborted = False
        # Our own commands come back on this subscription too, so another active publisher
        # shows up as more traffic than we sent. Counting endpoints is not enough: the GUI and
        # spacemouse_commander keep a publisher open even with publishing off.
        self.rx_times: deque = deque()
        self.tx_times: deque = deque()
        self.create_subscription(Float32MultiArray, CMD_TOPIC,
                                 lambda _m: self.rx_times.append(time.monotonic()), 50)
        self.create_subscription(WristState, '/hsa/wrist/state', self._state_cb, 10)
        self.create_subscription(PoseStamped, '/platform_pose', self._pose_cb, 10)
        self.create_subscription(Bool, '/platform_tag_visible', lambda m: setattr(self, 'tag_visible', m.data), 10)
        self.create_subscription(Bool, '/plate_mount_ok', lambda m: setattr(self, 'mount_ok', m.data), 10)
        # [n_tags, tag_mask, rms_px, ...] from platform_pose: which tags the pose was fit on.
        self.quality = [float('nan')] * 3
        self.create_subscription(Float32MultiArray, '/platform_pose/pose_quality',
                                 lambda m: setattr(self, 'quality', list(m.data[:3])), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/platform_zero_set', lambda m: setattr(self, 'zero_set', m.data), latched)
        self.create_timer(1.0 / float(g('cmd_rate')), self._tick)
        self.last_pose_t = None

    # ---- callbacks -------------------------------------------------------
    def _state_cb(self, msg: WristState) -> None:
        self.state = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.last_pose_t = time.monotonic()
        o, q = msg.pose.position, msg.pose.orientation
        if pg is not None:
            m = pg.plate_metrics(pg.pose_from_quat_xyz([o.x, o.y, o.z], [q.x, q.y, q.z, q.w]))
        else:
            m = dict.fromkeys(['z_mm', 'x_mm', 'y_mm', 'lateral_mm', 'tilt_deg', 'tilt_dir_deg', 'twist_deg'], float('nan'))
        s = self.state
        meas = list(s.extension_deg) if s else [float('nan')] * N
        turns = list(s.motor_pos_turns) if s and len(s.motor_pos_turns) == N else [float('nan')] * N
        vel = list(s.motor_vel_turns_s) if s else [float('nan')] * N
        tq = list(s.torque_estimate_nm) if s else [float('nan')] * N
        iq = list(s.iq_measured_a) if s else [float('nan')] * N
        t_pose = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        phase = 'settle' if self.t_set is None or time.monotonic() - self.t_set < self.settle else 'hold'
        pid = int(self.ids[self.k]) if self.mode == 'static' and self.k < len(self.seq) else -1
        qual = self.quality
        row = [time.monotonic() - self.t0, t_pose, phase, pid, int(self.tag_visible), int(self.mount_ok), *qual,
               o.x, o.y, o.z, q.x, q.y, q.z, q.w,
               m['z_mm'], m['x_mm'], m['y_mm'], m['lateral_mm'], m['tilt_deg'], m['tilt_dir_deg'], m['twist_deg'],
               *self.cmd.tolist(), *meas, *turns, *vel, *tq, *iq]
        self.w_state.writerow(row)
        if phase == 'hold' and self.tag_visible and self.mount_ok:
            self.window.append((m, meas, turns, qual))

    # ---- control loop ----------------------------------------------------
    def _publish(self) -> None:
        msg = Float32MultiArray(); msg.data = [float(v) for v in np.clip(self.cmd, self.lo, self.hi)]
        self.tx_times.append(time.monotonic())
        self.pub.publish(msg)

    def _slew_to(self, target: np.ndarray) -> bool:
        d = np.clip(target - self.cmd, -self.slew, self.slew)
        self.cmd = self.cmd + d
        return bool(np.all(np.abs(target - self.cmd) < 1e-6))

    def _foreign_traffic(self) -> int:
        """Messages on CMD_TOPIC in the last window beyond the ones we sent."""
        cut = time.monotonic() - GUARD_WINDOW_S
        for q in (self.rx_times, self.tx_times):
            while q and q[0] < cut:
                q.popleft()
        return len(self.rx_times) - len(self.tx_times)

    def _tick(self) -> None:
        if self.done:
            return
        if time.monotonic() - self.t0 < GUARD_WINDOW_S:
            return
        extra = self._foreign_traffic()
        # Two publishers on the joint-command topic interleave their setpoints and the position
        # loop chatters between them (2026-11-08: the GUI held 0 deg at 100 Hz while the sweep
        # ramped to 30 deg and all six motors vibrated).
        if not self.started and extra > 0:
            self.get_logger().error(
                f'something else is publishing {CMD_TOPIC} ({extra} msgs in {GUARD_WINDOW_S:g} s; '
                'joint_commander_gui PUBLISHING or spacemouse_commander on?) - turn it off; waiting',
                throttle_duration_sec=3)
            return
        if self.started and extra > GUARD_SLACK:
            self.get_logger().fatal(
                f'another publisher started on {CMD_TOPIC} mid-run ({extra} extra msgs in '
                f'{GUARD_WINDOW_S:g} s) - aborting without commanding; it now owns the wrist')
            self.aborted = True
            self.done = True
            raise SystemExit
        self.started = True
        if not self.zero_set:
            self.get_logger().warn('waiting for /platform_zero_set (call /set_platform_zero at 0 deg)', throttle_duration_sec=5)
            return
        if self.last_pose_t is not None and time.monotonic() - self.last_pose_t > 2.0:
            self.get_logger().error('no /platform_pose for 2 s (camera stalled or tag lost) — holding')
            self._publish(); return
        t = time.monotonic() - self.t0
        if self.mode == 'dynamic':
            if t > float(self.get_parameter('duration_s').value):
                return self._finish()
            if self.pattern == 'chirp':
                tgt = chirp(t, self.lo, self.hi, float(self.get_parameter('chirp_f0').value),
                            float(self.get_parameter('chirp_f1').value), float(self.get_parameter('chirp_T').value), self.phases)
            else:
                tgt = multisine(t, self.lo, self.hi, list(self.get_parameter('freqs').value), self.phases)
            self._slew_to(tgt); self._publish(); return

        # static: slew to setpoint, settle, average, advance
        if self.k >= len(self.seq):
            return self._finish()
        reached = self._slew_to(self.seq[self.k])
        self._publish()
        if reached and self.t_set is None:
            self.t_set, self.window = time.monotonic(), deque()
        if self.t_set is not None and time.monotonic() - self.t_set >= self.settle + self.avg_s:
            self._write_point()
            self.k += 1; self.t_set = None
            if self.k % 10 == 0:
                self.get_logger().info(f'{self.k}/{len(self.seq)} setpoints')

    def _write_point(self) -> None:
        if not self.window:
            self.get_logger().warn(f'point {self.ids[self.k]}: no valid samples (tag/mount) — skipped')
            return
        ms, meas, turns, qs = zip(*self.window)
        masks = [int(x[1]) for x in qs if x[1] == x[1]]
        mask_mode = max(set(masks), key=masks.count) if masks else -1
        mask_changes = sum(a != b for a, b in zip(masks, masks[1:]))
        rms = float(np.nanmean([x[2] for x in qs])) if any(x[2] == x[2] for x in qs) else float('nan')
        arr = {k: np.array([m[k] for m in ms]) for k in ms[0]}
        ang = np.deg2rad(arr['tilt_dir_deg'])
        tilt_dir = math.degrees(math.atan2(np.sin(ang).mean(), np.cos(ang).mean()))
        self.w_pts.writerow([int(self.ids[self.k]), self.direction[self.k], len(ms),
                             arr['z_mm'].mean(), arr['x_mm'].mean(), arr['y_mm'].mean(),
                             math.hypot(arr['x_mm'].mean(), arr['y_mm'].mean()), arr['tilt_deg'].mean(),
                             tilt_dir, arr['twist_deg'].mean(), arr['z_mm'].std(), arr['lateral_mm'].std(),
                             arr['tilt_deg'].std(), mask_mode, mask_changes, rms, *self.cmd.tolist(),
                             *np.nanmean(np.array(meas, dtype=float), 0).tolist(),
                             *np.nanmean(np.array(turns, dtype=float), 0).tolist()])
        self.f_pts.flush(); self.f_state.flush()

    def _finish(self) -> None:
        self.get_logger().info('excitation complete, returning to 0')
        self.done = True
        self.create_timer(1.0 / 50.0, self._home_tick)

    def _home_tick(self) -> None:
        if self._foreign_traffic() > GUARD_SLACK:
            self.get_logger().fatal(f'another publisher started on {CMD_TOPIC} while homing - '
                                    'stopping; it now owns the wrist')
            self.aborted = True
            raise SystemExit
        reached = self._slew_to(np.zeros(N))
        self._publish()                 # publish before exiting so the final 0 goes out
        if reached:
            self.get_logger().info('at 0 deg — done'); self.f_state.close(); self.f_pts.close()
            raise SystemExit


def main(args=None) -> None:
    # No rclpy signal handler: it shuts the context down on Ctrl-C, after which nothing can be
    # published, so the wrist could not be brought back to 0.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SysIdExcite()
    try:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            if not node.started or node.aborted:
                pass
            elif node.done:
                node.get_logger().warn('Ctrl-C while homing: stopped, holding the current setpoint')
            else:
                node.get_logger().warn(f'Ctrl-C: returning to 0 deg at {node.get_parameter("rate_deg_s").value:g} '
                                       'deg/s; Ctrl-C again to stop and hold')
                node._finish()
                try:
                    rclpy.spin(node)
                except KeyboardInterrupt:
                    node.get_logger().warn('stopped, holding the current setpoint')
    except SystemExit:
        pass
    finally:
        node.f_state.close(); node.f_pts.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
