#!/usr/bin/env python3
"""ATE(绝对轨迹误差)评估 —— 只依赖 numpy,不用 evo。

为什么自己写而不用 evo:evo 在这台机的 Python 3.14 上装不起来(PEP 668 +
ensurepip 失败)。而 ATE 的定义是明确的,自己实现比跟包管理纠缠快,
而且能把口径写死在代码里 —— 单目 VIO 必须用 **Sim(3)** 对齐(尺度可估),
用 SE(3) 会把尺度误差算进位置误差里,得到一个偏大且没有意义的数。

三个口径都报,因为它们回答不同的问题:
  • ATE(SE3)   固定尺度=1 对齐   → "位姿本身对不对"
  • ATE(Sim3)  同时估尺度       → **论文里报的就是这个**,可与 RD-VIO 对拍
  • scale       Sim3 估出的尺度   → 🔑 **这才是我们产品真正关心的量**
                                    (目标:尺度误差进 1%,即 |s-1| < 0.01)

用法:
  python3 ate.py <est.tum> <gt.tum> [--max-dt 0.02]
"""
import sys
import numpy as np


def load_tum(path):
    """TUM: timestamp tx ty tz qx qy qz qw"""
    ts, xyz = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            c = line.split()
            if len(c) < 8:
                continue
            ts.append(float(c[0]))
            xyz.append([float(c[1]), float(c[2]), float(c[3])])
    return np.asarray(ts), np.asarray(xyz)


def associate(t_est, t_gt, max_dt):
    """按时间戳最近邻配对。
    ⚠️ 必须限 max_dt —— 不限的话轨迹只覆盖一小段时也会"配上",
       ATE 看起来很小但其实只对了几帧。这是个典型的静默失效。
    """
    idx_gt = np.searchsorted(t_gt, t_est)
    pairs = []
    for i, t in enumerate(t_est):
        j = idx_gt[i]
        cands = [k for k in (j - 1, j) if 0 <= k < len(t_gt)]
        if not cands:
            continue
        k = min(cands, key=lambda k: abs(t_gt[k] - t))
        if abs(t_gt[k] - t) <= max_dt:
            pairs.append((i, k))
    return pairs


def umeyama(src, dst, with_scale):
    """Umeyama 对齐:求 s,R,t 使 s*R*src + t ≈ dst。"""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    cov = d0.T @ s0 / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    if with_scale:
        var = (s0 ** 2).sum() / len(src)
        s = float(np.trace(np.diag(D) @ S) / var) if var > 0 else 1.0
    else:
        s = 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def rmse(a, b):
    return float(np.sqrt(((a - b) ** 2).sum(axis=1).mean()))


def main(argv):
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    est_p, gt_p = argv[1], argv[2]
    max_dt = 0.02
    if "--max-dt" in argv:
        max_dt = float(argv[argv.index("--max-dt") + 1])

    t_e, p_e = load_tum(est_p)
    t_g, p_g = load_tum(gt_p)
    if len(t_e) == 0:
        print("❌ 估计轨迹为空 —— 跑失败了,不是精度问题", file=sys.stderr)
        return 1
    if len(t_g) == 0:
        print("❌ 真值为空", file=sys.stderr)
        return 1

    pairs = associate(t_e, t_g, max_dt)
    if len(pairs) < 10:
        print(f"❌ 只配上 {len(pairs)} 对(max_dt={max_dt}s)—— 时间戳可能不同源",
              file=sys.stderr)
        return 1

    E = np.array([p_e[i] for i, _ in pairs])
    G = np.array([p_g[j] for _, j in pairs])

    # 覆盖率:估计轨迹覆盖了真值时间跨度的多少。
    # 🔑 这个必须报 —— 只跑了前 10% 就发散的轨迹,ATE 可能反而很小。
    span_gt = t_g[-1] - t_g[0]
    span_pair = t_g[pairs[-1][1]] - t_g[pairs[0][1]]
    coverage = span_pair / span_gt if span_gt > 0 else 0.0

    s_se3, R_se3, t_se3 = umeyama(E, G, with_scale=False)
    ate_se3 = rmse((R_se3 @ E.T).T + t_se3, G)

    s, R, t = umeyama(E, G, with_scale=True)
    ate_sim3 = rmse(s * (R @ E.T).T + t, G)

    print(f"  轨迹点     {len(t_e)} (配对 {len(pairs)})")
    print(f"  时间覆盖   {coverage*100:.1f}%   <- 低于 90% 说明中途丢了/发散了")
    print(f"  ATE(SE3)   {ate_se3:.4f} m")
    print(f"  ATE(Sim3)  {ate_sim3:.4f} m   <- 论文口径,可与 RD-VIO 对拍")
    print(f"  尺度 s     {s:.6f}  (误差 {abs(s-1)*100:.3f}%)  <- 产品 KPI 关心这个")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
