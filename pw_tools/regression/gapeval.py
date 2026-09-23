#!/usr/bin/python3
"""Evaluate an XRSLAM trajectory around an induced visual gap against the ARKit reference.

  gapeval.py <est.tum> <arkit_ref.tum> <gap_first_s> <gap_last_s> [label]

Alignment is Umeyama SE3 (and Sim3), the same estimator as viobench-recordings/ate.py,
fitted ONLY on the pre-gap pairs.  The same transform is then applied after the gap:
if the system continues on the same map the post-gap error stays at the pre-gap level;
if it re-initialised a new map (arbitrary new world frame) the error explodes.
All numbers are in cm.  Pairing: nearest reference timestamp within 10 ms (as ate.py).
"""
import sys
import numpy as np


def load(p):
    T, P = [], []
    for ln in open(p):
        f = ln.split()
        if len(f) < 8 or ln.startswith('#'):
            continue
        T.append(float(f[0]))
        P.append([float(f[1]), float(f[2]), float(f[3])])
    return np.array(T), np.array(P)


def umeyama(X, Y, ws=True):
    mx, my = X.mean(1, keepdims=True), Y.mean(1, keepdims=True)
    Xc, Yc = X - mx, Y - my
    S = Yc @ Xc.T / X.shape[1]
    U, D, Vt = np.linalg.svd(S)
    d = np.ones(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        d[2] = -1
    R = U @ np.diag(d) @ Vt
    s = (D * d).sum() / ((Xc ** 2).sum() / X.shape[1]) if ws else 1.0
    return s, R, my - s * R @ mx


def rmse(e):
    return float(np.sqrt((e ** 2).mean())) * 100 if len(e) else float('nan')


est, ref, g0, g1 = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
label = sys.argv[5] if len(sys.argv) > 5 else est
te, Pe = load(est)
tr, Pr = load(ref)
idx = np.clip(np.searchsorted(tr, te), 1, len(tr) - 1)
l, r = np.abs(te - tr[idx - 1]), np.abs(te - tr[idx])
pick = np.where(l < r, idx - 1, idx)
ok = np.minimum(l, r) < 0.010
t = te[ok]
X = Pe[ok].T
Y = Pr[pick[ok]].T

pre = t < g0
post = t > g1
gap = (t >= g0) & (t <= g1)
s, R, tt = umeyama(X[:, pre], Y[:, pre], False)          # SE3 on pre-gap only
e = np.linalg.norm((R @ X + tt) - Y, axis=0)
s3, R3, t3 = umeyama(X[:, pre], Y[:, pre], True)          # Sim3 on pre-gap only
e3 = np.linalg.norm((s3 * R3 @ X + t3) - Y, axis=0)
# whole trajectory, SE3 (ate.py semantics)
_, Rw, tw = umeyama(X, Y, False)
ew = np.linalg.norm((Rw @ X + tw) - Y, axis=0)
# post-gap alone, SE3 self-aligned (shape of the post-gap segment)
if post.sum() >= 10:
    _, Rp, tp = umeyama(X[:, post], Y[:, post], False)
    ep = np.linalg.norm((Rp @ X[:, post] + tp) - Y[:, post], axis=0)
else:
    ep = np.array([])


def at(tq):
    j = np.searchsorted(t, tq)
    j = min(max(j, 0), len(t) - 1)
    return e[j] * 100


# raw output continuity (no reference involved): per-output-step position jump
dP = np.linalg.norm(np.diff(Pe, axis=0), axis=1)
dt = np.diff(te)
after = te[1:] > g1
med_step = np.median(dP[te[1:] < g0])
jmax_i = np.argmax(np.where(after, dP, -1)) if after.any() else None
# output gap: largest hole in the output time series after the gap start
holes = np.where(te[1:] >= g0, dt, 0)
hole_i = int(np.argmax(holes))

print(f'[{label}]')
print(f'  poses {len(te)} | paired {len(t)} (pre {pre.sum()}, gap {gap.sum()}, post {post.sum()})')
print(f'  whole-traj SE3 ATE {rmse(ew):.2f}')
print(f'  pre-gap aligned  : pre SE3 {rmse(e[pre]):.2f} | gap-end err {at(g1):.2f} | '
      f'post SE3 {rmse(e[post]):.2f} (Sim3 {rmse(e3[post]):.2f}) | post max {100*e[post].max() if post.any() else float("nan"):.2f}')
print(f'  err at +0.5s {at(g1+0.5):.2f} | +2s {at(g1+2):.2f} | end {e[-1]*100:.2f}')
print(f'  post self-aligned SE3 {rmse(ep):.2f}')
print(f'  output: median step pre-gap {med_step*100:.3f} cm | max step after gap '
      f'{(dP[jmax_i]*100 if jmax_i is not None else float("nan")):.2f} cm at t={te[jmax_i+1] if jmax_i is not None else float("nan"):.3f} '
      f'| largest output hole {holes[hole_i]:.3f} s at t={te[hole_i]:.3f}')
