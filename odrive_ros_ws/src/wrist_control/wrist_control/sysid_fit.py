"""Fit a data-driven kinematic (and optionally dynamic) model of the wrist from sysid_excite logs.

No ROS needed.  Follows Underactuated ch. 18:
  * linear-in-parameters least squares  y = W(theta) alpha   (sec. "Parameter identification",
    "Residual physics models with linear function approximators")
  * identifiability from the SVD / condition number of W      (sec. "Experiment design")
  * sparse basis selection by sequential thresholded LS       (SINDy, same section)
  * hysteresis as a direction-dependent term, like sign(qdot) friction
  * ARX on the continuous log, scored by simulation error, not just equation error
    (sec. "Equation error vs simulation error", "Autoregressive models")

Static (kinematic) fit uses *_points.csv; dynamic ARX fit uses *_samples.csv.

    sysid_fit points.csv [more.csv ...] --degree 2 --sparse 0.02 --out wrist_kin.npz
    sysid_fit samples.csv --arx --na 2 --nb 2 --dt 0.05
"""
from __future__ import annotations

import argparse
import csv
import itertools
import math
import sys

import numpy as np

N = 6
OUT_NAMES = ['x_mm', 'y_mm', 'z_mm', 'tilt_x_deg', 'tilt_y_deg', 'twist_deg']


# ----------------------------------------------------------------------------- data
def read_csv(path: str) -> dict[str, np.ndarray]:
    with open(path) as f:
        rows = list(csv.DictReader(f))
    cols = {}
    for k in rows[0]:
        v = [r[k] for r in rows]
        try:
            cols[k] = np.array(v, dtype=float)
        except ValueError:
            cols[k] = np.array(v)
    return cols


def pose_outputs(d: dict[str, np.ndarray]) -> np.ndarray:
    """6 pose numbers per row: translation (mm), tilt as a 2-vector (deg), twist (deg).
    Tilt as (tilt*cos(dir), tilt*sin(dir)) is smooth at 0, unlike (tilt, dir)."""
    ang = np.deg2rad(d['tilt_dir_deg'])
    return np.column_stack([d['x_mm'], d['y_mm'], d['z_mm'],
                            d['tilt_deg'] * np.cos(ang), d['tilt_deg'] * np.sin(ang), d['twist_deg']])


def load_points(paths: list[str], use_measured: bool) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    th, y, direction, pid = [], [], [], []
    for off, p in enumerate(paths):
        d = read_csv(p)
        key = 'meas' if use_measured and not np.isnan(d['meas0']).all() else 'cmd'
        th.append(np.column_stack([d[f'{key}{i}'] for i in range(N)]))
        y.append(pose_outputs(d))
        direction.append(d['direction'])
        pid.append(d['point_id'] + 100000 * off)
    return np.vstack(th), np.vstack(y), np.concatenate(direction), np.concatenate(pid)


# ----------------------------------------------------------------------------- basis
class Basis:
    """Polynomial library in the 6 leg angles (scaled to [0,1] by `scale` deg),
    plus an optional hysteresis feature h = +1 going up / -1 coming down."""

    def __init__(self, degree: int, scale: float, hysteresis: bool):
        self.degree, self.scale, self.hysteresis = degree, scale, hysteresis
        self.terms: list[tuple[int, ...]] = [()]
        for deg in range(1, degree + 1):
            self.terms += list(itertools.combinations_with_replacement(range(N), deg))
        self.names = ['1'] + ['*'.join(f'q{i}' for i in t) for t in self.terms[1:]]
        if hysteresis:
            self.names += ['h'] + [f'h*q{i}' for i in range(N)]

    def __call__(self, theta: np.ndarray, h: np.ndarray | None = None) -> np.ndarray:
        q = np.atleast_2d(theta) / self.scale
        cols = [np.ones(len(q))] + [np.prod(q[:, list(t)], axis=1) for t in self.terms[1:]]
        if self.hysteresis:
            h = np.zeros(len(q)) if h is None else np.asarray(h, dtype=float)
            cols += [h] + [h * q[:, i] for i in range(N)]
        return np.column_stack(cols)


# ----------------------------------------------------------------------------- fitting
def lstsq_ridge(W: np.ndarray, Y: np.ndarray, lam: float) -> np.ndarray:
    """alpha = argmin ||W a - Y||^2 + lam ||a||^2 (lam=0 -> plain least squares via SVD)."""
    if lam <= 0:
        return np.linalg.lstsq(W, Y, rcond=None)[0]
    return np.linalg.solve(W.T @ W + lam * np.eye(W.shape[1]), W.T @ Y)


def stlsq(W: np.ndarray, Y: np.ndarray, threshold: float, lam: float, iters: int = 10) -> np.ndarray:
    """Sequential thresholded least squares (Brunton SINDy): refit after zeroing small coefficients.
    Coefficients are compared after column scaling so the threshold is unit-free."""
    scale = np.linalg.norm(W, axis=0) + 1e-12
    Ws = W / scale
    A = lstsq_ridge(Ws, Y, lam)
    for _ in range(iters):
        small = np.abs(A) < threshold
        A[small] = 0.0
        for j in range(Y.shape[1]):
            big = ~small[:, j]
            if big.any():
                A[big, j] = lstsq_ridge(Ws[:, big], Y[:, j:j + 1], lam)[:, 0]
    return A / scale[:, None]


def report_conditioning(W: np.ndarray, names: list[str]) -> None:
    s = np.linalg.svd(W, compute_uv=False)
    print(f'regressor W: {W.shape[0]} rows x {W.shape[1]} cols, rank {np.linalg.matrix_rank(W)}, '
          f'cond = {s[0] / s[-1]:.3g}')
    print('  singular values:', np.array2string(s, precision=3, max_line_width=120))
    if s[-1] / s[0] < 1e-8:
        _, _, Vt = np.linalg.svd(W)
        v = Vt[-1]
        idx = np.argsort(-np.abs(v))[:4]
        print('  (near-)unidentifiable direction:', ', '.join(f'{v[i]:+.2f} {names[i]}' for i in idx))


def rmse(Y: np.ndarray, Yh: np.ndarray) -> np.ndarray:
    return np.sqrt(np.mean((Y - Yh) ** 2, axis=0))


def fit_static(a: argparse.Namespace) -> None:
    th, Y, direction, pid = load_points(a.paths, use_measured=not a.use_commanded)
    if not a.keep_down:
        keep = direction == 'up'
        th, Y, direction, pid = th[keep], Y[keep], direction[keep], pid[keep]
    h = np.where(direction == 'up', 1.0, -1.0)
    if a.hysteresis and np.abs(th[direction == 'down']).max() < 1e-6:
        print('WARNING: every "down" point is at 0 deg (zero_between data) — the h*q terms are not '
              'identifiable; collect a pattern:=sweep run for hysteresis.')
    basis = Basis(a.degree, a.scale, a.hysteresis)
    W = basis(th, h)
    print(f'{len(th)} points, outputs {OUT_NAMES}, basis {len(basis.names)} terms (degree {a.degree}'
          f'{", hysteresis" if a.hysteresis else ""})')

    # hold out whole points (not rows) so repeated samples of one point never straddle the split
    rng = np.random.default_rng(a.seed)
    upid = np.unique(pid)
    test_ids = set(rng.choice(upid, size=int(round(a.test_frac * len(upid))), replace=False).tolist())
    te = np.array([p in test_ids for p in pid]); tr = ~te
    report_conditioning(W[tr], basis.names)

    A = stlsq(W[tr], Y[tr], a.sparse, a.ridge) if a.sparse > 0 else lstsq_ridge(W[tr], Y[tr], a.ridge)
    print(f'\nRMSE  (train / test), {tr.sum()} / {te.sum()} rows')
    for j, nm in enumerate(OUT_NAMES):
        print(f'  {nm:12s} {rmse(Y[tr, j:j+1], W[tr] @ A[:, j:j+1])[0]:7.3f} / '
              f'{rmse(Y[te, j:j+1], W[te] @ A[:, j:j+1])[0]:7.3f}   (output std {Y[:, j].std():.3f})')
    nz = (np.abs(A) > 0).sum(axis=0)
    print('  nonzero terms per output:', nz.tolist())
    if a.verbose:
        for j, nm in enumerate(OUT_NAMES):
            terms = [(basis.names[i], A[i, j]) for i in np.argsort(-np.abs(A[:, j])) if A[i, j] != 0][:12]
            print(f'  {nm}: ' + ', '.join(f'{c:+.3g}·{n}' for n, c in terms))

    # linear-region sanity check against the measured numbers in camera_tracking.md
    lin = A[1:1 + N, 2] / a.scale
    print(f'\nd z_mm / d q_i at 0 (mm/deg): {np.array2string(lin, precision=4)}  '
          f'(doc: ~0.07 mm/deg per leg for all-six up to 120 deg => ~0.012 each)')

    if a.out:
        np.savez(a.out, coef=A, degree=a.degree, scale=a.scale, hysteresis=a.hysteresis,
                 names=np.array(basis.names), out_names=np.array(OUT_NAMES),
                 train_rmse=rmse(Y[tr], W[tr] @ A), test_rmse=rmse(Y[te], W[te] @ A))
        print(f'saved {a.out}')
    if a.plot:
        import matplotlib.pyplot as plt
        Yh = W @ A
        fig, ax = plt.subplots(2, 3, figsize=(12, 7))
        for j, (nm, axj) in enumerate(zip(OUT_NAMES, ax.flat)):
            axj.scatter(Y[tr, j], Yh[tr, j], s=6, label='train')
            axj.scatter(Y[te, j], Yh[te, j], s=10, c='r', label='test')
            lim = [Y[:, j].min(), Y[:, j].max()]; axj.plot(lim, lim, 'k--', lw=0.8)
            axj.set_title(nm); axj.set_xlabel('measured'); axj.set_ylabel('predicted')
        ax[0, 0].legend(); fig.tight_layout(); plt.show()


class WristKinematicModel:
    """Evaluate a saved static fit:  model(theta_deg, direction=+1) -> [x,y,z,tilt_x,tilt_y,twist]."""

    def __init__(self, path: str):
        d = np.load(path, allow_pickle=True)
        self.basis = Basis(int(d['degree']), float(d['scale']), bool(d['hysteresis']))
        self.coef = d['coef']

    def __call__(self, theta_deg, direction: float = 1.0) -> np.ndarray:
        th = np.atleast_2d(theta_deg)
        return (self.basis(th, np.full(len(th), direction)) @ self.coef).squeeze()


# ----------------------------------------------------------------------------- dynamic (ARX)
def resample(t: np.ndarray, X: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Uniform grid by linear interpolation (what soft_stewart/koopman_processdata.py does)."""
    tg = np.arange(t[0], t[-1], dt)
    return tg, np.column_stack([np.interp(tg, t, X[:, j]) for j in range(X.shape[1])])


def arx_regressor(Y: np.ndarray, U: np.ndarray, na: int, nb: int) -> tuple[np.ndarray, np.ndarray]:
    k = max(na, nb)
    rows, tgt = [], []
    for n in range(k, len(Y) - 1):
        rows.append(np.concatenate([Y[n - i] for i in range(na)] + [U[n - i] for i in range(nb)] + [[1.0]]))
        tgt.append(Y[n + 1])
    return np.array(rows), np.array(tgt)


def arx_rollout(Y: np.ndarray, U: np.ndarray, A: np.ndarray, na: int, nb: int) -> np.ndarray:
    """Simulation error: feed predictions back in, only the measured input drives it."""
    k = max(na, nb)
    Yh = Y.copy()
    for n in range(k, len(Y) - 1):
        phi = np.concatenate([Yh[n - i] for i in range(na)] + [U[n - i] for i in range(nb)] + [[1.0]])
        Yh[n + 1] = phi @ A
    return Yh


def fit_arx(a: argparse.Namespace) -> None:
    d = read_csv(a.paths[0])
    ok = (d['tag_visible'] > 0) & (d['mount_ok'] > 0)
    t = d['t'][ok]
    Y = pose_outputs({k: d[k][ok] for k in ('x_mm', 'y_mm', 'z_mm', 'tilt_deg', 'tilt_dir_deg', 'twist_deg')})
    U = np.column_stack([d[f'cmd{i}'][ok] for i in range(N)])
    tg, Y = resample(t, Y, a.dt); _, U = resample(t, U, a.dt)
    split = int(len(tg) * (1 - a.test_frac))
    W, T = arx_regressor(Y[:split], U[:split], a.na, a.nb)
    A = lstsq_ridge(W, T, a.ridge)
    print(f'ARX na={a.na} nb={a.nb} dt={a.dt}: {len(W)} train rows, {len(tg) - split} test samples')
    report_conditioning(W, [f'y[n-{i}]' for i in range(a.na)] + [f'u[n-{i}]' for i in range(a.nb)] + ['1'])
    Wt, Tt = arx_regressor(Y[split:], U[split:], a.na, a.nb)
    print('equation-error RMSE (test):  ', np.array2string(rmse(Tt, Wt @ A), precision=3))
    Yh = arx_rollout(Y[split:], U[split:], A, a.na, a.nb)
    print('simulation-error RMSE (test):', np.array2string(rmse(Y[split:], Yh), precision=3))
    print('output std (test):           ', np.array2string(Y[split:].std(axis=0), precision=3))
    if a.out:
        np.savez(a.out, coef=A, na=a.na, nb=a.nb, dt=a.dt, out_names=np.array(OUT_NAMES)); print(f'saved {a.out}')
    if a.plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(6, 1, figsize=(12, 10), sharex=True)
        tt = tg[split:]
        for j, nm in enumerate(OUT_NAMES):
            ax[j].plot(tt, Y[split:, j], label='measured'); ax[j].plot(tt, Yh[:, j], label='rollout'); ax[j].set_ylabel(nm)
        ax[0].legend(); ax[-1].set_xlabel('t [s]'); fig.tight_layout(); plt.show()


# ----------------------------------------------------------------------------- cli
def main(argv=None) -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('paths', nargs='+')
    p.add_argument('--arx', action='store_true', help='dynamic ARX fit on a *_samples.csv')
    p.add_argument('--degree', type=int, default=2)
    p.add_argument('--scale', type=float, default=120.0, help='deg that maps to q=1 (keeps W well scaled)')
    p.add_argument('--hysteresis', action='store_true', help='add up/down direction terms (needs --keep-down)')
    p.add_argument('--keep-down', action='store_true', help='also use the return-to-zero ("down") points')
    p.add_argument('--use-commanded', action='store_true', help='regress on commanded instead of measured deg')
    p.add_argument('--sparse', type=float, default=0.0, help='STLSQ threshold on scaled coefficients (0 = off)')
    p.add_argument('--ridge', type=float, default=0.0)
    p.add_argument('--test-frac', type=float, default=0.2)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--na', type=int, default=2); p.add_argument('--nb', type=int, default=2)
    p.add_argument('--dt', type=float, default=0.05)
    p.add_argument('--out', default='')
    p.add_argument('--plot', action='store_true'); p.add_argument('-v', '--verbose', action='store_true')
    a = p.parse_args(argv)
    if a.hysteresis and not a.keep_down:
        print('--hysteresis implies --keep-down'); a.keep_down = True
    (fit_arx if a.arx else fit_static)(a)


if __name__ == '__main__':
    main(sys.argv[1:])
