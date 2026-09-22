"""Stable text report format + comparison / noise-floor logic.

Format (line oriented, deliberately diff-friendly, fixed field widths):

    # xrslam-regression report v2
    M <key> <value>                       ... metadata, sorted by key
    C idx frame_ts pose_ts state lm fus qx qy qz qw tx ty tz push_rc lm_rc ms
    F 0 100.025000000 0.000000000 0 0 0 0.000000 ... 0 -3 12.4
    S <key> <value>                       ... summary, sorted by key

`M` lines carry things that legitimately change between machines (paths,
timestamps, wall clock).  `F` lines are the payload.  A plain `diff` of two
reports is therefore readable, and `compare()` gives the quantitative version.
"""

import math

import geom

VERSION = "xrslam-regression report v2"

FRAME_FIELDS = (
    "idx",
    "frame_ts",
    "pose_ts",
    "state",
    "landmarks",
    "fusion_total",
    "qx",
    "qy",
    "qz",
    "qw",
    "tx",
    "ty",
    "tz",
    "push_rc",
    "lm_rc",
    "ms",
    # ---- v2: read straight out of XRSLAMHealth ----------------------------
    "det_kp",        # detected_keypoints
    "trk_kp",        # tracked_keypoints   <- the "tracked" count
    "inl_kp",        # inlier_keypoints    <- the "inlier" count
    "mapped_lm",     # mapped_landmarks
    "lm_published",  # landmarks_published (pre-filter)
    "lm_usable",     # landmarks_usable    (isfinite && triangulated)
    "core_seq",      # core_frame_seq  -- did the core actually take a frame?
    "core_imu",      # core_imu_samples for this frame
    "degen",         # latest_pose_degenerate (pose is a zero quaternion)
    "settled",       # 1 = core_seq advanced before we sampled, 0 = we raced it
)

INT_FIELDS = frozenset((
    "idx", "state", "landmarks", "fusion_total", "push_rc", "lm_rc",
    "det_kp", "trk_kp", "inl_kp", "mapped_lm", "lm_published", "lm_usable",
    "core_seq", "core_imu", "degen", "settled",
))

# Metadata keys that must never take part in a regression verdict, because
# they legitimately differ between two runs of the *same* build.
VOLATILE_META = ("wall_start", "host", "cmdline", "outdir", "run_index", "root")


class Report(object):
    def __init__(self):
        self.meta = {}
        self.frames = []  # list of dicts
        self.summary = {}

    # -- io -----------------------------------------------------------------
    def dumps(self):
        out = ["# " + VERSION]
        for k in sorted(self.meta):
            out.append("M %s %s" % (k, _fmt_meta(self.meta[k])))
        out.append("C " + " ".join(FRAME_FIELDS))
        for f in self.frames:
            parts = ["F"]
            for name in FRAME_FIELDS:
                v = f[name]
                if name in INT_FIELDS:
                    parts.append("%d" % v)
                elif name in ("frame_ts", "pose_ts"):
                    parts.append("%.9f" % v)
                elif name == "ms":
                    parts.append("%.3f" % v)
                else:
                    parts.append("%.6f" % v)
            out.append(" ".join(parts))
        for k in sorted(self.summary):
            out.append("S %s %s" % (k, _fmt_meta(self.summary[k])))
        return "\n".join(out) + "\n"

    def save(self, path):
        with open(path, "w") as fh:
            fh.write(self.dumps())

    @staticmethod
    def loads(text):
        r = Report()
        for line in text.splitlines():
            if not line or line.startswith("#") or line.startswith("C "):
                continue
            tag, rest = line.split(" ", 1)
            if tag == "M":
                k, _, v = rest.partition(" ")
                r.meta[k] = v
            elif tag == "S":
                k, _, v = rest.partition(" ")
                r.summary[k] = v
            elif tag == "F":
                vals = rest.split()
                if len(vals) != len(FRAME_FIELDS):
                    raise ValueError("bad F line: %r" % line)
                d = {}
                for name, v in zip(FRAME_FIELDS, vals):
                    d[name] = int(v) if name in INT_FIELDS else float(v)
                r.frames.append(d)
        return r

    @staticmethod
    def load(path):
        with open(path) as fh:
            return Report.loads(fh.read())

    # -- helpers ------------------------------------------------------------
    STATE_TRACKING = 1

    def tracked_frames(self):
        """Frames the library reported as TRACKING_SUCCESS *and* whose pose is
        usable.

        Not `pose_ts > 0`: XRSLAM already stamps a body-pose timestamp while it
        is still INITIALIZING, so that predicate silently counts init frames as
        tracked and poisons every downstream metric (ATE included).

        The `degen` filter matters just as much: measured 2026-08-23, the first
        frame that reports TRACKING_SUCCESS carries q = (0,0,0,0), which is not
        a rotation.  Left in, it contributes a full-magnitude term to ATE and a
        garbage term to every rotation metric.
        """
        return [f for f in self.frames
                if f["state"] == Report.STATE_TRACKING and f.get("degen", 0) == 0]


def _fmt_meta(v):
    if isinstance(v, float):
        return "%.9g" % v
    return str(v).replace("\n", " ")


# ------------------------------------------------------------- comparison ---
METRICS = (
    "pos_max",
    "pos_p95",
    "pos_med",
    "rot_max_deg",
    "rot_p95_deg",
    "aligned_pos_max",
    "aligned_pos_p95",
    "lm_max_abs",
    "lm_mean_abs",
    "trk_kp_max_abs",
    "inl_kp_max_abs",
    "mapped_lm_max_abs",
    "unsettled_delta",
    "state_mismatch",
    "n_frames_delta",
    "n_tracked_delta",
    "first_track_delta",
)


def _pct(vals, q):
    if not vals:
        return 0.0
    s = sorted(vals)
    if len(s) == 1:
        return s[0]
    i = q * (len(s) - 1)
    lo = int(math.floor(i))
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (i - lo)


def first_tracked_idx(rep):
    tf = rep.tracked_frames()
    return tf[0]["idx"] if tf else -1


def compare(a, b, warmup=0):
    """Quantitative difference between two reports. Returns a dict of METRICS.

    `warmup` drops the first N frames after EACH report's own first tracked
    frame before comparing poses.  Measured 2026-08-23: with threading on, the
    initialisation frame wanders over a ~10-frame range (37..46) run to run,
    and the first frames after each init are still converging.  Comparing a run
    that initialised at 37 against one that initialised at 46 therefore lines up
    a settled pose against a transient one, which dominates every position and
    rotation metric.  warmup=0 (the default) changes nothing.
    """
    res = dict((k, 0.0) for k in METRICS)
    warm_a = first_tracked_idx(a) + warmup
    warm_b = first_tracked_idx(b) + warmup
    res["n_frames_delta"] = abs(len(a.frames) - len(b.frames))
    ta, tb = a.tracked_frames(), b.tracked_frames()
    res["n_tracked_delta"] = abs(len(ta) - len(tb))
    fa = ta[0]["idx"] if ta else -1
    fb = tb[0]["idx"] if tb else -1
    res["first_track_delta"] = abs(fa - fb)

    ma = dict((f["idx"], f) for f in a.frames)
    mb = dict((f["idx"], f) for f in b.frames)
    common = sorted(set(ma) & set(mb))

    pos_d, rot_d, lm_d = [], [], []
    trk_d, inl_d, map_d = [], [], []
    pa_pts, pb_pts = [], []
    state_mismatch = 0
    for i in common:
        x, y = ma[i], mb[i]
        if x["state"] != y["state"]:
            state_mismatch += 1
        lm_d.append(abs(x["landmarks"] - y["landmarks"]))
        # v2 fields are absent from a v1 report; degrade instead of crashing
        if "trk_kp" in x and "trk_kp" in y:
            trk_d.append(abs(x["trk_kp"] - y["trk_kp"]))
            inl_d.append(abs(x["inl_kp"] - y["inl_kp"]))
            map_d.append(abs(x["mapped_lm"] - y["mapped_lm"]))
        if (x["state"] == Report.STATE_TRACKING
                and y["state"] == Report.STATE_TRACKING
                and x.get("degen", 0) == 0 and y.get("degen", 0) == 0
                and i >= warm_a and i >= warm_b):
            px = [x["tx"], x["ty"], x["tz"]]
            py = [y["tx"], y["ty"], y["tz"]]
            pos_d.append(geom.vnorm(geom.vsub(px, py)))
            qx = [x["qx"], x["qy"], x["qz"], x["qw"]]
            qy = [y["qx"], y["qy"], y["qz"], y["qw"]]
            rot_d.append(math.degrees(geom.rot_angle_between(qx, qy)))
            pa_pts.append(px)
            pb_pts.append(py)

    res["state_mismatch"] = state_mismatch
    res["pos_max"] = max(pos_d) if pos_d else 0.0
    res["pos_p95"] = _pct(pos_d, 0.95)
    res["pos_med"] = _pct(pos_d, 0.5)
    res["rot_max_deg"] = max(rot_d) if rot_d else 0.0
    res["rot_p95_deg"] = _pct(rot_d, 0.95)
    res["lm_max_abs"] = max(lm_d) if lm_d else 0.0
    res["lm_mean_abs"] = sum(lm_d) / len(lm_d) if lm_d else 0.0
    res["trk_kp_max_abs"] = max(trk_d) if trk_d else 0.0
    res["inl_kp_max_abs"] = max(inl_d) if inl_d else 0.0
    res["mapped_lm_max_abs"] = max(map_d) if map_d else 0.0
    ua = sum(1 for f in a.frames if f.get("settled", 1) == 0)
    ub = sum(1 for f in b.frames if f.get("settled", 1) == 0)
    res["unsettled_delta"] = abs(ua - ub)

    if len(pa_pts) >= 3:
        R, t, s = geom.umeyama(pa_pts, pb_pts, with_scale=False)
        ad = []
        for pa, pb in zip(pa_pts, pb_pts):
            q = geom.vadd(geom.vscale(geom.mvec(R, pa), s), t)
            ad.append(geom.vnorm(geom.vsub(q, pb)))
        res["aligned_pos_max"] = max(ad)
        res["aligned_pos_p95"] = _pct(ad, 0.95)
    return res


def noise_floor(reports, warmup=0):
    """Per-metric noise floor = worst value over every pair of same-config runs.

    MEASURED, 2026-08-23, synth:// 160 frames, this repo @ d052dc3:

      XRSLAM_ENABLE_THREADING=OFF  -> floor is exactly 0 for every position
        metric over 10 pairs of 5 runs (rotation floor 1.6e-15 deg = float
        round-off in the quaternion angle).  The solver is bit-deterministic.
      XRSLAM_ENABLE_THREADING=ON, replayed at realtime pace -> pos_p95 floor
        2.5e-05 m; a residual 1-frame jitter at the initialisation boundary
        (first_track_delta = 1) is the only structural instability.
      XRSLAM_ENABLE_THREADING=ON, replayed free-running -> pos_max floor
        0.53 m.  That is NOT solver noise, it is this harness sampling the
        pose before the background worker produced it.  See run_once.py's
        threading guard.

    A previous revision of this file asserted that XRSLAM seeds its RANSAC
    from std::random_device (random.h:8) and therefore could not repeat.  That
    is wrong and the measurement above is what disproves it: Ransac::solve
    (utility/ransac.h:32) calls `lotbox.seed(seed)` with the member `seed`,
    which defaults to 0.  LotBox's constructor does reach random_device via
    RandomBase, but that value is overwritten before it is ever drawn from.
    Do not restore the claim without re-measuring.

    Any A/B verdict that does not first measure this number is measuring noise.
    """
    if len(reports) < 2:
        raise ValueError("need >= 2 runs to measure a noise floor")
    floor = dict((k, 0.0) for k in METRICS)
    pairs = 0
    for i in range(len(reports)):
        for j in range(i + 1, len(reports)):
            d = compare(reports[i], reports[j], warmup=warmup)
            pairs += 1
            for k in METRICS:
                if d[k] > floor[k]:
                    floor[k] = d[k]
    floor["_pairs"] = pairs
    floor["_runs"] = len(reports)
    return floor


# --------------------------------------------------- permutation A/B test ---
def pairwise(reports, warmup=0):
    """All C(n,2) comparisons, computed once and reused by the permutation."""
    m = {}
    for i in range(len(reports)):
        for j in range(i + 1, len(reports)):
            m[(i, j)] = compare(reports[i], reports[j], warmup=warmup)
    return m


def _cross_stat(mat, group_a, group_b, metric):
    """Worst cross-group difference. Reported, but NOT the test statistic."""
    best = 0.0
    for i in group_a:
        for j in group_b:
            if i == j:
                continue
            v = mat[(i, j) if i < j else (j, i)][metric]
            if v > best:
                best = v
    return best


def _contrast(mat, group_a, group_b, metric):
    """mean(between-group distance) - mean(within-group distance).

    This, not a max, is the permutation statistic.  A max over cross pairs is
    almost useless under relabelling: it is set by the single most-different
    pair, and that pair survives nearly every relabelling, so the null collapses
    onto the observed value and the test never rejects.  Measured 2026-08-23:
    with a max statistic, an unmistakable injected fault on the *bit-
    deterministic* build (within-group distance exactly 0) still came back
    WITHIN_NOISE.  A mean contrast moves when the grouping is informative,
    which is the question being asked.
    """
    a, b = list(group_a), list(group_b)
    between = [mat[(i, j) if i < j else (j, i)][metric] for i in a for j in b
               if i != j]
    within = []
    for g in (a, b):
        for x in range(len(g)):
            for y in range(x + 1, len(g)):
                i, j = g[x], g[y]
                within.append(mat[(i, j) if i < j else (j, i)][metric])
    if not between:
        return 0.0
    mb = sum(between) / len(between)
    mw = (sum(within) / len(within)) if within else 0.0
    return mb - mw


def permutation_ab(reports_a, reports_b, warmup=0, quantile=0.95, max_splits=4000,
                   seed=0):
    """Is the A-vs-B difference bigger than a random relabelling would give?

    Why this replaces "cross max vs within max": those two statistics are maxima
    over DIFFERENT numbers of pairs (with 4+4 runs, 16 cross pairs vs 6+6 within
    pairs), so the cross statistic is larger even when both groups come from the
    same distribution.  Measured 2026-08-23: A/B-ing the mobile build against
    *itself* returned DIFFERENT on 5 metrics at margin 1.0.  A verdict rule that
    fails on identical inputs cannot be used to judge a change.

    Here the observed statistic and the null are the SAME statistic: pool the
    runs, relabel them into groups of the same two sizes every possible way (or
    `max_splits` random ways), and take the `quantile` of the resulting cross
    statistics as the threshold.  Bias cancels because it applies to both sides.

    Returns (label, exceed_list, observed_dict, threshold_dict).  `observed`
    also carries "worst_<metric>" entries: the plain worst cross-pair value, for
    reading, not for the verdict.
    """
    import itertools
    import random

    reps = list(reports_a) + list(reports_b)
    na, n = len(reports_a), len(reps)
    if na < 2 or len(reports_b) < 2:
        raise ValueError("permutation A/B needs >= 2 runs per arm")
    mat = pairwise(reps, warmup=warmup)
    idx = list(range(n))
    observed = dict(
        (m, _contrast(mat, idx[:na], idx[na:], m)) for m in METRICS)
    worst = dict(
        (m, _cross_stat(mat, idx[:na], idx[na:], m)) for m in METRICS)

    combos = list(itertools.combinations(idx, na))
    if len(combos) > max_splits:
        rng = random.Random(seed)
        combos = [tuple(rng.sample(idx, na)) for _ in range(max_splits)]
    null = dict((m, []) for m in METRICS)
    for a in combos:
        sa = set(a)
        b = [i for i in idx if i not in sa]
        for m in METRICS:
            null[m].append(_contrast(mat, a, b, m))

    thresh = dict((m, _pct(sorted(null[m]), quantile)) for m in METRICS)
    exceed = []
    for m in METRICS:
        if observed[m] > thresh[m] + 1e-12:
            exceed.append("%s=%.6g > null p%d %.6g (%d relabellings)"
                          % (m, observed[m], int(quantile * 100), thresh[m],
                             len(combos)))
    for m in METRICS:
        observed["worst_" + m] = worst[m]
    return ("DIFFERENT" if exceed else "WITHIN_NOISE"), exceed, observed, thresh


def verdict(diff, floor, margin=1.0):
    """Classify a comparison against a measured noise floor.

    margin > 1 widens the floor (e.g. 1.5 to be conservative).  Returns
    (label, list_of_exceeding_metric_strings).
    """
    exceed = []
    for k in METRICS:
        lim = floor.get(k, 0.0) * margin
        if diff[k] > lim + 1e-12:
            exceed.append(
                "%s=%.6g > floor %.6g (x%.2f)" % (k, diff[k], floor.get(k, 0.0), margin)
            )
    if not exceed:
        return "WITHIN_NOISE", exceed
    return "DIFFERENT", exceed


def format_diff(diff, floor=None, margin=1.0):
    lines = []
    for k in METRICS:
        if floor is None:
            lines.append("  %-18s %14.6g" % (k, diff[k]))
        else:
            lim = floor.get(k, 0.0) * margin
            flag = "  " if diff[k] <= lim + 1e-12 else "!!"
            lines.append("%s%-18s %14.6g   floor %14.6g" % (flag, k, diff[k], lim))
    return "\n".join(lines)


# ------------------------------------------------------------------- ATE ---
def ate(rep, truth, with_scale=False):
    """Absolute trajectory error against ground truth, after SE3/Sim3 alignment.

    truth: list of (t, quat_xyzw, pos) as produced by dataset.load_truth.
    Frames are matched by nearest timestamp within half a frame interval.
    """
    tf = rep.tracked_frames()
    if len(tf) < 3 or len(truth) < 3:
        return None
    tmap = sorted(truth, key=lambda a: a[0])
    times = [a[0] for a in tmap]
    tol = 0.5 * (times[-1] - times[0]) / max(1, len(times) - 1)
    src, dst = [], []
    for f in tf:
        t = f["frame_ts"]
        lo, hi = 0, len(times) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if times[mid] < t:
                lo = mid + 1
            else:
                hi = mid
        best = lo
        for c in (lo - 1, lo, lo + 1):
            if 0 <= c < len(times) and abs(times[c] - t) < abs(times[best] - t):
                best = c
        if abs(times[best] - t) > tol:
            continue
        src.append([f["tx"], f["ty"], f["tz"]])
        dst.append(list(tmap[best][2]))
    if len(src) < 3:
        return None
    R, tvec, s = geom.umeyama(src, dst, with_scale=with_scale)
    errs = []
    for a, b in zip(src, dst):
        q = geom.vadd(geom.vscale(geom.mvec(R, a), s), tvec)
        errs.append(geom.vnorm(geom.vsub(q, b)))
    rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
    return {
        "n": len(errs),
        "rmse": rmse,
        "max": max(errs),
        "p95": _pct(errs, 0.95),
        "scale": s,
    }
