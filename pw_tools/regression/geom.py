"""Tiny 3D geometry helpers (pure stdlib, no numpy).

Matrices are row-major lists of lists; vectors are 3-tuples/lists.
Quaternions follow the XRSLAM C API layout: [x, y, z, w].
"""

import math

# ---------------------------------------------------------------- vectors ---


def vadd(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vsub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vscale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def vnorm(a):
    return math.sqrt(vdot(a, a))


# --------------------------------------------------------------- matrices ---


def eye3():
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def mmul(A, B):
    return [
        [sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def mT(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def mvec(A, v):
    return [
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    ]


def msub(A, B):
    return [[A[i][j] - B[i][j] for j in range(3)] for i in range(3)]


def mscale(A, s):
    return [[A[i][j] * s for j in range(3)] for i in range(3)]


def rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def skew_to_vec(S):
    """Inverse of the hat operator; expects an (almost) skew-symmetric matrix."""
    return [
        0.5 * (S[2][1] - S[1][2]),
        0.5 * (S[0][2] - S[2][0]),
        0.5 * (S[1][0] - S[0][1]),
    ]


def orthonormalize(R):
    """Gram-Schmidt; keeps numerically-differentiated rotations on SO(3)."""
    c0 = [R[0][0], R[1][0], R[2][0]]
    c1 = [R[0][1], R[1][1], R[2][1]]
    c0 = vscale(c0, 1.0 / vnorm(c0))
    c1 = vsub(c1, vscale(c0, vdot(c0, c1)))
    c1 = vscale(c1, 1.0 / vnorm(c1))
    c2 = vcross(c0, c1)
    return [[c0[i], c1[i], c2[i]] for i in range(3)]


# ------------------------------------------------------------ quaternions ---


def quat_from_mat(R):
    """Rotation matrix -> [x, y, z, w], w >= 0."""
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2][1] - R[1][2]) / s
        y = (R[0][2] - R[2][0]) / s
        z = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return [x / n, y / n, z / n, w / n]


def mat_from_quat(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return eye3()
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def rot_angle_between(qa, qb):
    """Geodesic angle in radians between two [x,y,z,w] quaternions.

    Uses the relative quaternion + atan2 form on purpose: `2*acos(dot)` loses
    all precision near identity (identical inputs come back as ~1e-8 rad of
    fake rotation), which would put a permanent non-zero floor under every
    rotation metric in the harness.
    """
    ax, ay, az, aw = qa
    bx, by, bz, bw = qb
    # q_rel = conj(qa) * qb
    rw = aw * bw + ax * bx + ay * by + az * bz
    rx = aw * bx - ax * bw - ay * bz + az * by
    ry = aw * by + ax * bz - ay * bw - az * bx
    rz = aw * bz - ax * by + ay * bx - az * bw
    v = math.sqrt(rx * rx + ry * ry + rz * rz)
    return 2.0 * math.atan2(v, abs(rw))


def mat_angle(R):
    """Rotation angle in radians of a rotation matrix."""
    c = (R[0][0] + R[1][1] + R[2][2] - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, c)))


# ---------------------------------------------------- symmetric 3x3 eigen ---


def jacobi_eig_sym3(A):
    """Eigen-decomposition of a symmetric 3x3 matrix.

    Returns (eigenvalues[3], V) with V columns the eigenvectors, sorted by
    descending eigenvalue.
    """
    a = [row[:] for row in A]
    V = eye3()
    for _ in range(64):
        off = abs(a[0][1]) + abs(a[0][2]) + abs(a[1][2])
        if off < 1e-18:
            break
        for p, q in ((0, 1), (0, 2), (1, 2)):
            if abs(a[p][q]) < 1e-20:
                continue
            theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
            t = (1.0 if theta >= 0 else -1.0) / (
                abs(theta) + math.sqrt(theta * theta + 1.0)
            )
            c = 1.0 / math.sqrt(t * t + 1.0)
            s = t * c
            J = eye3()
            J[p][p] = c
            J[q][q] = c
            J[p][q] = s
            J[q][p] = -s
            a = mmul(mmul(mT(J), a), J)
            V = mmul(V, J)
    vals = [a[0][0], a[1][1], a[2][2]]
    order = sorted(range(3), key=lambda i: -vals[i])
    vals_s = [vals[i] for i in order]
    V_s = [[V[r][order[c]] for c in range(3)] for r in range(3)]
    return vals_s, V_s


def svd3(M):
    """SVD of a 3x3 matrix: returns (U, s[3], V) with M = U diag(s) V^T."""
    MtM = mmul(mT(M), M)
    vals, V = jacobi_eig_sym3(MtM)
    s = [math.sqrt(max(0.0, v)) for v in vals]
    U = [[0.0] * 3 for _ in range(3)]
    cols = []
    for j in range(3):
        vj = [V[0][j], V[1][j], V[2][j]]
        uj = mvec(M, vj)
        n = vnorm(uj)
        if n > 1e-12:
            uj = vscale(uj, 1.0 / n)
        else:
            uj = None
        cols.append(uj)
    # fill degenerate columns with something orthogonal
    for j in range(3):
        if cols[j] is None:
            known = [c for c in cols if c is not None]
            if len(known) >= 2:
                cols[j] = vcross(known[0], known[1])
            elif len(known) == 1:
                seed = [1.0, 0.0, 0.0]
                if abs(vdot(seed, known[0])) > 0.9:
                    seed = [0.0, 1.0, 0.0]
                cols[j] = vcross(known[0], seed)
            else:
                cols[j] = [1.0 if k == j else 0.0 for k in range(3)]
            n = vnorm(cols[j])
            cols[j] = vscale(cols[j], 1.0 / n) if n > 1e-12 else [
                1.0 if k == j else 0.0 for k in range(3)
            ]
    for r in range(3):
        for j in range(3):
            U[r][j] = cols[j][r]
    return U, s, V


def umeyama(src, dst, with_scale=True):
    """Least-squares similarity that maps `src` onto `dst`.

    Returns (R, t, s) so that  dst ~= s * R * src + t.
    Both inputs are lists of 3-vectors of equal length (>= 3).
    """
    n = len(src)
    if n != len(dst) or n < 3:
        raise ValueError("need >= 3 matched points")
    mu_s = [sum(p[i] for p in src) / n for i in range(3)]
    mu_d = [sum(p[i] for p in dst) / n for i in range(3)]
    sigma_s = 0.0
    Sigma = [[0.0] * 3 for _ in range(3)]
    for ps, pd in zip(src, dst):
        a = vsub(ps, mu_s)
        b = vsub(pd, mu_d)
        sigma_s += vdot(a, a)
        for i in range(3):
            for j in range(3):
                Sigma[i][j] += b[i] * a[j]
    sigma_s /= n
    Sigma = mscale(Sigma, 1.0 / n)

    U, s, V = svd3(Sigma)
    S = eye3()
    detU = _det3(U)
    detV = _det3(V)
    if detU * detV < 0.0:
        S[2][2] = -1.0
    R = mmul(mmul(U, S), mT(V))
    if with_scale and sigma_s > 1e-18:
        scale = (s[0] * S[0][0] + s[1] * S[1][1] + s[2] * S[2][2]) / sigma_s
    else:
        scale = 1.0
    t = vsub(mu_d, vscale(mvec(R, mu_s), scale))
    return R, t, scale


def _det3(A):
    return (
        A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
        - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
        + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])
    )


def exp_so3(v):
    """Rodrigues exponential map: rotation vector -> rotation matrix."""
    th = vnorm(v)
    if th < 1e-12:
        K = [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
        return [[(1.0 if i == j else 0.0) + K[i][j] for j in range(3)] for i in range(3)]
    a = [v[0] / th, v[1] / th, v[2] / th]
    K = [[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]]
    s, c = math.sin(th), math.cos(th)
    K2 = mmul(K, K)
    return [
        [
            (1.0 if i == j else 0.0) + s * K[i][j] + (1.0 - c) * K2[i][j]
            for j in range(3)
        ]
        for i in range(3)
    ]
