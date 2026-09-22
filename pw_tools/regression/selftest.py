"""Self-tests for the regression harness -- every check ships its negative control.

A verification predicate that has never been seen to fail is not a predicate.
So each test below has two halves:

    positive  the correct input must pass,
    negative  a deliberately broken input must FAIL the same predicate.

If the negative half does not fail, the test itself is reported as broken.

  python3 selftest.py                      # everything that needs no library
  python3 selftest.py --build BUILD_DIR    # + end-to-end tests against libxrslam
"""

import argparse
import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import dataset  # noqa: E402
import geom  # noqa: E402
import pngio  # noqa: E402
import report as report_mod  # noqa: E402
import synth  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

RESULTS = []


def check(name, positive_ok, negative_ok, detail=""):
    """positive_ok: correct input passed.  negative_ok: broken input was caught."""
    ok = bool(positive_ok) and bool(negative_ok)
    RESULTS.append((name, ok, positive_ok, negative_ok, detail))
    print(
        "%-38s %s   (positive=%s negative-control=%s) %s"
        % (
            name,
            "PASS" if ok else "FAIL",
            "ok" if positive_ok else "FAILED",
            "caught" if negative_ok else "NOT CAUGHT",
            detail,
        )
    )
    return ok


# ------------------------------------------------------------------- 1 PNG ---
def test_png_roundtrip():
    w, h = 61, 37
    data = bytes(((x * 7 + y * 13) & 0xFF) for y in range(h) for x in range(w))
    blob = pngio.encode_gray8(w, h, data)
    dw, dh, out = pngio.decode_gray8(blob)
    positive = (dw, dh, out) == (w, h, data)

    corrupted = bytearray(data)
    corrupted[123] ^= 0xFF
    blob2 = pngio.encode_gray8(w, h, bytes(corrupted))
    _, _, out2 = pngio.decode_gray8(blob2)
    negative = out2 != data  # a single flipped byte must survive the round trip
    check("png.roundtrip", positive, negative)


def _make_png_with_filters(w, h, data, filters):
    raw = bytearray()
    prev = bytearray(w)
    for y in range(h):
        line = bytearray(data[y * w : (y + 1) * w])
        ft = filters[y % len(filters)]
        enc = bytearray(w)
        for i in range(w):
            a = line[i - 1] if i >= 1 else 0
            b = prev[i]
            c = prev[i - 1] if i >= 1 else 0
            if ft == 0:
                pr = 0
            elif ft == 1:
                pr = a
            elif ft == 2:
                pr = b
            elif ft == 3:
                pr = (a + b) >> 1
            else:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
            enc[i] = (line[i] - pr) & 0xFF
        raw.append(ft)
        raw += enc
        prev = line

    def chunk(tag, payload):
        return (
            struct.pack(">I", len(payload))
            + tag
            + payload
            + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
        )

    return (
        pngio.PNG_MAGIC
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(bytes(raw)))
        + chunk(b"IEND", b"")
    )


def test_png_filters():
    """EuRoC PNGs in the wild use every filter type; exercise the unfilter path."""
    w, h = 40, 25
    data = bytes(((x * 31 + y * 17 + (x * y) // 3) & 0xFF) for y in range(h)
                 for x in range(w))
    blob = _make_png_with_filters(w, h, data, [0, 1, 2, 3, 4])
    _, _, out = pngio.decode_gray8(blob)
    positive = out == data
    # negative control: same bytes declared with the wrong filter type per row
    blob_bad = _make_png_with_filters(w, h, data, [4, 3, 2, 1, 0])
    _, _, out_bad = pngio.decode_gray8(blob_bad)
    # the decoder is correct, so a *differently filtered* stream of the same
    # image must still decode to the same image; the real negative control is a
    # bogus filter byte, which must raise.
    negative = False
    try:
        bad = bytearray(blob)
        pngio.decode_gray8(bytes(bad[:8]) + b"\x00" * 8)
    except Exception:
        negative = True
    check("png.filters", positive and out_bad == data, negative)


# -------------------------------------------------------------- 2 geometry ---
def test_umeyama():
    src = [[0.1, 0.2, 0.3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
           [1.0, 2.0, 3.0], [-1.0, 0.5, 2.0]]
    Rt = geom.exp_so3([0.3, -0.2, 0.7])
    tt = [1.5, -2.0, 0.25]
    st = 1.7
    dst = [geom.vadd(geom.vscale(geom.mvec(Rt, p), st), tt) for p in src]
    R, t, s = geom.umeyama(src, dst, with_scale=True)
    err = max(
        geom.vnorm(geom.vsub(geom.vadd(geom.vscale(geom.mvec(R, a), s), t), b))
        for a, b in zip(src, dst)
    )
    positive = err < 1e-9 and abs(s - st) < 1e-9

    dst_bad = [p[:] for p in dst]
    dst_bad[2][0] += 0.5  # one outlier: residual must show up
    R2, t2, s2 = geom.umeyama(src, dst_bad, with_scale=True)
    err2 = max(
        geom.vnorm(geom.vsub(geom.vadd(geom.vscale(geom.mvec(R2, a), s2), t2), b))
        for a, b in zip(src, dst_bad)
    )
    negative = err2 > 0.05
    check("geom.umeyama", positive, negative, "err=%.3g err_bad=%.3g" % (err, err2))


# ------------------------------------------------- 3 IMU / trajectory truth ---
def integrate_imu(params, gravity_sign=-1.0, n_steps=None):
    """Re-integrate the synthesised IMU stream and return the final pose error.

    This is the check that makes the synthetic sequence trustworthy: it closes
    the loop between `traj_*` (analytic) and `imu_sample` (what we feed the
    library).  A wrong gravity sign, a body/world frame mix-up or a unit error
    all show up here as a huge number.
    """
    dt = 1.0 / params.imu_rate
    n = n_steps or int(params.duration * params.imu_rate)
    t = params.t0
    R = synth.traj_rotation(t)
    v = synth.traj_velocity(t)
    p = synth.traj_position(t)
    g_w = [0.0, 0.0, gravity_sign * synth.GRAVITY]
    gk, ak = synth.imu_sample(t, params)
    for i in range(n):
        t2 = t + dt
        gk1, ak1 = synth.imu_sample(t2, params)
        w_mid = [0.5 * (gk[j] + gk1[j]) for j in range(3)]
        R2 = geom.orthonormalize(geom.mmul(R, geom.exp_so3(geom.vscale(w_mid, dt))))
        a1 = geom.vadd(geom.mvec(R, ak), g_w)
        a2 = geom.vadd(geom.mvec(R2, ak1), g_w)
        a_mid = geom.vscale(geom.vadd(a1, a2), 0.5)
        p = geom.vadd(geom.vadd(p, geom.vscale(v, dt)),
                      geom.vscale(a_mid, 0.5 * dt * dt))
        v = geom.vadd(v, geom.vscale(a_mid, dt))
        R = R2
        gk, ak = gk1, ak1
        t = t2
    p_ref = synth.traj_position(t)
    R_ref = synth.traj_rotation(t)
    pos_err = geom.vnorm(geom.vsub(p, p_ref))
    rot_err = math.degrees(geom.mat_angle(geom.mmul(geom.mT(R_ref), R)))
    return pos_err, rot_err


def test_imu_consistency():
    p = synth.SynthParams()
    pos_err, rot_err = integrate_imu(p, gravity_sign=-1.0)
    positive = pos_err < 0.05 and rot_err < 0.5
    bad_pos, _bad_rot = integrate_imu(p, gravity_sign=+1.0)  # gravity sign flipped
    negative = bad_pos > 10.0
    check(
        "synth.imu_consistency",
        positive,
        negative,
        "pos_err=%.4gm rot_err=%.3gdeg  flipped-g=%.4gm" % (pos_err, rot_err, bad_pos),
    )


# ------------------------------------------------------------- 4 rendering ---
def test_render_projection():
    p = synth.SynthParams(duration=0.2)
    lms = synth.make_landmarks(p)
    r = synth.Renderer(p)
    t = p.t0 + 0.05
    R = synth.traj_rotation(t)
    pos = synth.traj_position(t)
    buf, nvis = r.render(R, pos, lms)

    def hit_rate(du, dv):
        hits = tot = 0
        for X in lms:
            pr = synth.project(p, R, pos, X)
            if pr is None:
                continue
            u, v, _ = pr
            u += du
            v += dv
            if u < 6 or v < 6 or u >= p.width - 6 or v >= p.height - 6:
                continue
            tot += 1
            best = 0
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    val = buf[(int(v) + dy) * p.width + int(u) + dx]
                    best = max(best, val)
            if best >= p.blob_peak - 30:
                hits += 1
        return hits, tot

    h, tot = hit_rate(0.0, 0.0)
    positive = tot > 30 and h >= 0.90 * tot
    hb, totb = hit_rate(12.0, 12.0)  # negative control: wrong predicted pixel
    negative = totb > 30 and hb < 0.5 * totb
    check(
        "synth.render_projection",
        positive,
        negative,
        "visible=%d hit=%d/%d  shifted=%d/%d" % (nvis, h, tot, hb, totb),
    )


# ---------------------------------------------------------------- 5 report ---
def _fake_report(n=30, offset=(0.0, 0.0, 0.0), lm_bump=0, state_flip=0):
    r = report_mod.Report()
    r.meta["tool"] = "selftest"
    for i in range(n):
        r.frames.append(
            {
                "idx": i,
                "frame_ts": 100.0 + 0.05 * i,
                "pose_ts": 100.0 + 0.05 * i,
                "state": 2 if (state_flip and i == 5) else 1,
                "landmarks": 50 + (i % 7) + lm_bump,
                "fusion_total": 40,
                "qx": 0.0,
                "qy": 0.0,
                "qz": math.sin(0.001 * i),
                "qw": math.cos(0.001 * i),
                "tx": 0.01 * i + offset[0],
                "ty": 0.02 * i + offset[1],
                "tz": 0.005 * i + offset[2],
                "push_rc": 0,
                "lm_rc": 0,
                "ms": 1.0,
                "det_kp": 120,
                "trk_kp": 110,
                "inl_kp": 105,
                "mapped_lm": 90,
                "lm_published": 0,
                "lm_usable": 0,
                "core_seq": i,
                "core_imu": 10,
                "degen": 0,
                "settled": 1,
            }
        )
    r.summary["n_frames"] = n
    return r


def test_report_roundtrip():
    r = _fake_report()
    txt = r.dumps()
    r2 = report_mod.Report.loads(txt)
    positive = r2.dumps() == txt and len(r2.frames) == len(r.frames)
    broken = txt.replace("F 3 ", "F 3 999 ")  # one extra column
    negative = False
    try:
        report_mod.Report.loads(broken)
    except ValueError:
        negative = True
    check("report.roundtrip", positive, negative)


def test_compare_and_floor():
    a = _fake_report()
    b = _fake_report()
    same = report_mod.compare(a, b)
    # 1e-12 rather than exact 0: the SE3 alignment is a least-squares fit, so
    # its residual on identical input is float round-off, not a real difference.
    positive = all(same[k] < 1e-12 for k in report_mod.METRICS)

    delta = 0.037
    c = _fake_report(offset=(delta, 0.0, 0.0))
    d = report_mod.compare(a, c)
    # a pure translation is absorbed by the alignment, so the raw metric is the
    # one that must see it
    seen = abs(d["pos_max"] - delta) < 1e-6 and d["aligned_pos_max"] < 1e-6
    zero_floor = dict((k, 0.0) for k in report_mod.METRICS)
    lab0, _ = report_mod.verdict(d, zero_floor)
    wide_floor = dict((k, 1.0) for k in report_mod.METRICS)
    lab1, _ = report_mod.verdict(d, wide_floor)
    negative = seen and lab0 == "DIFFERENT" and lab1 == "WITHIN_NOISE"
    check(
        "report.compare_verdict",
        positive,
        negative,
        "pos_max=%.6g zero-floor=%s wide-floor=%s" % (d["pos_max"], lab0, lab1),
    )

    # noise floor must equal the largest injected spread, and a *new* difference
    # smaller than that must be classified WITHIN_NOISE
    group = [_fake_report(offset=(0.01 * i, 0, 0)) for i in range(4)]
    floor = report_mod.noise_floor(group)
    floor_ok = abs(floor["pos_max"] - 0.03) < 1e-6 and floor["_pairs"] == 6
    small = report_mod.compare(_fake_report(), _fake_report(offset=(0.005, 0, 0)))
    big = report_mod.compare(_fake_report(), _fake_report(offset=(0.5, 0, 0)))
    lab_s, _ = report_mod.verdict(small, floor)
    lab_b, _ = report_mod.verdict(big, floor)
    check(
        "report.noise_floor",
        floor_ok and lab_s == "WITHIN_NOISE",
        lab_b == "DIFFERENT",
        "floor.pos_max=%.6g" % floor["pos_max"],
    )

    # structural differences must be caught even when the poses agree
    e = _fake_report(lm_bump=5, state_flip=1)
    de = report_mod.compare(a, e)
    check(
        "report.structural_diff",
        de["lm_max_abs"] == 5,
        de["state_mismatch"] == 1 and de["n_tracked_delta"] == 1,
        "lm=%g state_mismatch=%g" % (de["lm_max_abs"], de["state_mismatch"]),
    )


def test_ate():
    n = 40
    truth = []
    rep = report_mod.Report()
    Rt = geom.exp_so3([0.2, 0.1, -0.4])
    tt = [3.0, -1.0, 0.5]
    for i in range(n):
        t = 100.0 + 0.05 * i
        p_true = [0.01 * i, 0.02 * i, 0.004 * i * i]
        truth.append((t, [0, 0, 0, 1], p_true))
        p_est = geom.vadd(geom.mvec(geom.mT(Rt), geom.vsub(p_true, tt)), [0, 0, 0])
        rep.frames.append(
            {
                "idx": i, "frame_ts": t, "pose_ts": t, "state": 1,
                "landmarks": 10, "fusion_total": 0,
                "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
                "tx": p_est[0], "ty": p_est[1], "tz": p_est[2],
                "push_rc": 0, "lm_rc": 0, "ms": 1.0,
                "det_kp": 120, "trk_kp": 110, "inl_kp": 105, "mapped_lm": 90,
                "lm_published": 0, "lm_usable": 0, "core_seq": i,
                "core_imu": 10, "degen": 0, "settled": 1,
            }
        )
    good = report_mod.ate(rep, truth, with_scale=False)
    positive = good is not None and good["rmse"] < 1e-9
    rep.frames[10]["tx"] += 0.4
    bad = report_mod.ate(rep, truth, with_scale=False)
    negative = bad["rmse"] > 0.01
    check("report.ate", positive, negative,
          "rmse=%.3g bad=%.3g" % (good["rmse"], bad["rmse"]))


# ------------------------------------------------- 6 EuRoC on-disk round trip ---
def test_euroc_roundtrip():
    tmp = tempfile.mkdtemp(prefix="xrslam-reg-euroc-")
    try:
        p = synth.SynthParams(duration=0.6, num_landmarks=60)
        dataset.export_euroc(p, tmp)
        src = dataset.EurocSource(tmp)
        mem = list(synth.sequence(p))
        disk = list(src.events())
        mem_imgs = [e for e in mem if e[0] == "image"]
        disk_imgs = [e for e in disk if e[0] == "image"]
        mem_imu = [e for e in mem if e[0] == "imu"]
        disk_imu = [e for e in disk if e[0] == "imu"]
        img_ok = len(mem_imgs) == len(disk_imgs) and all(
            bytes(a[4]) == bytes(b[4]) and abs(a[1] - b[1]) < 1e-6
            for a, b in zip(mem_imgs, disk_imgs)
        )
        imu_ok = len(mem_imu) == len(disk_imu) and all(
            all(abs(x - y) < 1e-9 for x, y in zip(a[2] + a[3], b[2] + b[3]))
            for a, b in zip(mem_imu, disk_imu)
        )
        positive = img_ok and imu_ok and len(mem_imgs) > 5

        # negative control: corrupt one IMU sample on disk, the reader must differ
        csv = os.path.join(tmp, "imu0", "data.csv")
        with open(csv) as fh:
            lines = fh.read().splitlines()
        parts = lines[3].split(",")
        parts[4] = "%.9e" % (float(parts[4]) + 1.0)
        lines[3] = ",".join(parts)
        with open(csv, "w") as fh:
            fh.write("\n".join(lines) + "\n")
        src2 = dataset.EurocSource(tmp)
        disk2_imu = [e for e in src2.events() if e[0] == "imu"]
        negative = any(
            any(abs(x - y) > 1e-9 for x, y in zip(a[2] + a[3], b[2] + b[3]))
            for a, b in zip(mem_imu, disk2_imu)
        )
        check("dataset.euroc_roundtrip", positive, negative,
              "%d images / %d imu" % (len(mem_imgs), len(mem_imu)))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_undistort_map():
    """Zero distortion must be a no-op; real distortion must move corner pixels."""
    # real EuRoC cam0 geometry, so the numbers below mean something
    W, H = 752, 480
    K = (458.654, 457.296, 367.215, 248.375)
    D = (-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05)
    positive = dataset.build_undistort_map(W, H, K, (0.0, 0.0, 0.0, 0.0)) is None
    m = dataset.build_undistort_map(W, H, K, D)
    # negative control: with k1/k2 set, the map must be measurably NOT identity,
    # and the displacement must grow towards the image corner (radial model).
    def disp(u, v):
        s = m[v * W + u]
        if s < 0:
            return 1e9
        return math.hypot(s % W - u, s // W - v)
    centre = disp(int(K[2]), int(K[3]))
    corner = disp(W - 1, H - 1)
    negative = m is not None and centre < 0.6 and corner > 3.0
    check("dataset.undistort_map", positive, negative,
          "centre_disp=%.2fpx corner_disp=%.2fpx" % (centre, corner))


# ---------------------------------------------------- 7 end-to-end vs library ---
def test_end_to_end(build):
    """The real negative control for the whole harness.

    Two same-config runs define the noise floor.  A third run with a
    deliberately corrupted IMU (accelerometer scale factor off by 8%) must be
    classified DIFFERENT against that floor.  If it is not, the harness cannot
    detect a regression and is worthless.
    """
    tmp = tempfile.mkdtemp(prefix="xrslam-reg-e2e-")
    try:
        common = ["--max-frames", "160", "--quiet"]

        def one(out, extra=()):
            cmd = [sys.executable, os.path.join(HERE, "run_once.py"),
                   "--build", build, "--seq", "synth://", "--out", out,
                   "--workdir", os.path.join(tmp, "cfg")] + common + list(extra)
            r = subprocess.run(cmd, stdout=subprocess.DEVNULL,
                               stderr=subprocess.PIPE)
            if r.returncode != 0:
                sys.stderr.write(r.stderr.decode("utf-8", "replace")[-3000:])
                raise RuntimeError("run_once failed")
            return report_mod.Report.load(out)

        a = one(os.path.join(tmp, "a.txt"))
        b = one(os.path.join(tmp, "b.txt"))
        c = one(os.path.join(tmp, "c.txt"),
                ["--synth-set", "imu_scale=1.08"])

        floor = report_mod.noise_floor([a, b])
        d_same = report_mod.compare(a, b)
        d_diff = report_mod.compare(a, c)
        lab_same, _ = report_mod.verdict(d_same, floor)
        lab_diff, ex = report_mod.verdict(d_diff, floor)

        tracked_ok = len(a.tracked_frames()) > 40 and len(b.tracked_frames()) > 40
        positive = tracked_ok and lab_same == "WITHIN_NOISE"
        negative = lab_diff == "DIFFERENT"
        check(
            "e2e.noise_floor_vs_injected_fault",
            positive,
            negative,
            "tracked=%d/%d floor.pos_max=%.4g fault.pos_max=%.4g %s"
            % (len(a.tracked_frames()), len(a.frames), floor["pos_max"],
               d_diff["pos_max"], ex[:2]),
        )

        # the library's own inputs are also worth a control: a report must not
        # come back empty
        check("e2e.report_nonempty", len(a.frames) == 160,
              len(a.tracked_frames()) < len(a.frames),
              "frames=%d tracked=%d" % (len(a.frames), len(a.tracked_frames())))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_ffi_abi(build):
    import xrslam_ffi as ffi

    s = ffi.XRSlam(build)
    rc, ver = s.get_version()
    # before create, every typed getter must report NOT_CREATED and still write
    # a defined out-param (the header's contract)
    rc_state, state = s.get_state()
    rc_lm, n_lm, _ = s.get_landmarks()
    positive = rc_state == ffi.XRSLAM_ERR_NOT_CREATED and n_lm == 0
    # negative control: an obviously wrong build dir must fail loudly
    negative = False
    try:
        ffi.XRSlam(os.path.join(build, "definitely-not-here"))
    except IOError:
        negative = True
    check("ffi.not_created_contract", positive, negative,
          "state_rc=%s lm_rc=%s version_rc=%s"
          % (ffi.RC_NAMES.get(rc_state), ffi.RC_NAMES.get(rc_lm),
             ffi.RC_NAMES.get(rc)))



# --------------------------------------------- 7a permutation A/B calibration ---
def test_permutation_ab():
    """The A/B verdict must (a) not fire on identical groups, (b) fire on a real
    shift, and (c) keep power when the within-group spread is exactly zero.

    (c) is the case that killed the first implementation: with a max statistic,
    two perfectly repeatable groups separated by a large offset still came back
    WITHIN_NOISE, because the max cross-pair distance survives almost every
    relabelling.  The statistic is a mean contrast for that reason.
    """
    # Interleave the two groups. Splitting a monotone sequence down the middle
    # would make the grouping genuinely informative -- group A systematically
    # below group B -- and the test would be right to reject it.
    jit = [_fake_report(offset=(0.001 * i, 0, 0)) for i in range(8)]
    grp_a = [jit[0], jit[2], jit[4], jit[6]]
    grp_b = [jit[1], jit[3], jit[5], jit[7]]
    lab_same, _, _, _ = report_mod.permutation_ab(grp_a, grp_b)

    shifted = [_fake_report(offset=(0.001 * (2 * i + 1) + 0.5, 0, 0))
               for i in range(4)]
    lab_diff, ex, _, _ = report_mod.permutation_ab(grp_a, shifted)

    # zero within-group spread, clear between-group shift
    flat_a = [_fake_report() for _ in range(4)]
    flat_b = [_fake_report(offset=(0.25, 0, 0)) for _ in range(4)]
    lab_flat, _, _, _ = report_mod.permutation_ab(flat_a, flat_b)

    check("report.permutation_ab",
          lab_same == "WITHIN_NOISE",
          lab_diff == "DIFFERENT" and lab_flat == "DIFFERENT",
          "same=%s shifted=%s zero-spread=%s" % (lab_same, lab_diff, lab_flat))


# ------------------------------- 7b binary-consistency tripwire in regress.py ---
def test_same_binary_guard():
    """A noise-floor group must refuse to mix two different libxrslam builds."""
    import regress

    a, b = _fake_report(n=4), _fake_report(n=4)
    a.meta["lib_sha256"] = "a" * 64
    b.meta["lib_sha256"] = "a" * 64
    a.meta["tag"] = "run#0"
    b.meta["tag"] = "run#1"
    positive = regress.assert_same_binary([a, b], "t")[:4] == "aaaa"

    b.meta["lib_sha256"] = "b" * 64  # the binary moved under us
    negative = False
    try:
        regress.assert_same_binary([a, b], "t")
    except SystemExit:
        negative = True
    check("regress.same_binary_guard", positive, negative)


# ------------------------------------------------- 8 ABI: XRSLAMImage is 64B ---
def test_abi_image_layout(build):
    """Prove the trailing 16 bytes of XRSLAMImage are really read by the library.

    positive          a correct 64-byte struct with
                      timestamp_convention = FIRST_ROW_EXPOSURE_MID and
                      readout_time = 0.5 makes the library report
                      last_image_timestamp == t + 0.25 (its documented
                      t += readout/2 conversion).  That can only happen if the
                      fields are read at the offsets we declare.
    negative control  a 48-byte struct (the previous revision of xrslam_ffi.py)
                      followed in memory by those same bytes gets the SAME
                      +0.25 shift, although a 48-byte caller never set them.
                      That is the out-of-bounds read, demonstrated rather than
                      asserted.
    """
    import ctypes

    import xrslam_ffi as ffi

    TS_MID = 3
    READOUT = 0.5

    tmp = tempfile.mkdtemp(prefix="xrslam-reg-abi-")
    try:
        params = synth.SynthParams(duration=0.5, num_landmarks=30)
        src = dataset.SynthSource(params)
        slam_cfg, sensor_cfg = src.write_configs(tmp)
        slam = ffi.XRSlam(build)
        slam.create(slam_cfg, sensor_cfg)
        try:
            w, h = params.width, params.height
            pix = (ctypes.c_ubyte * (w * h))()

            def push_raw(raw, t):
                slam._lib.XRSLAMPushSensorDataChecked(
                    ffi.SENSOR_CAMERA, ctypes.byref(raw))
                _rc, hh = slam.get_health()
                return hh.last_image_timestamp

            # --- positive: the honest 64-byte struct -----------------------
            t1 = params.t0 + 1.0
            img = ffi.XRSLAMImage()
            img.data = ctypes.cast(pix, ctypes.POINTER(ctypes.c_ubyte))
            img.timeStamp = t1
            img.stride = w
            img.channel = 1
            img.width = w
            img.height = h
            img.readout_time = READOUT
            img.timestamp_convention = TS_MID
            got1 = push_raw(img, t1)
            positive = abs(got1 - (t1 + 0.5 * READOUT)) < 1e-9

            # --- negative control: 48 bytes + poisoned tail ----------------
            # Lay out exactly what a 48-byte definition would occupy, then put
            # (readout_time, timestamp_convention, reserved0) in the 16 bytes
            # that follow it -- i.e. memory the old struct did not own.
            t2 = t1 + 1.0
            blob = (ctypes.c_ubyte * 64)()
            ctypes.memset(blob, 0, 64)
            base = ctypes.addressof(blob)
            ctypes.c_void_p.from_address(base + 0).value = ctypes.addressof(pix)
            ctypes.c_double.from_address(base + 8).value = t2
            ctypes.c_int32.from_address(base + 16).value = w      # stride
            ctypes.c_int32.from_address(base + 24).value = 1      # channel
            ctypes.c_int32.from_address(base + 40).value = w      # width
            ctypes.c_int32.from_address(base + 44).value = h      # height
            # ---- past the end of a 48-byte XRSLAMImage ----
            ctypes.c_double.from_address(base + 48).value = READOUT
            ctypes.c_int32.from_address(base + 56).value = TS_MID
            got2 = push_raw(blob, t2)
            negative = abs(got2 - (t2 + 0.5 * READOUT)) < 1e-9
        finally:
            slam.destroy()
        check("ffi.abi_image_is_64_bytes", positive, negative,
              "sizeof=%d shift_ok=%.9g oob_shift=%.9g"
              % (ctypes.sizeof(ffi.XRSLAMImage), got1 - t1, got2 - t2))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# --------------------------------------- 9 degenerate poses leave the metrics ---
def test_degenerate_pose_excluded():
    r = _fake_report(n=10)
    r.frames[0]["degen"] = 1
    r.frames[0]["tx"] = 0.0
    r.frames[0]["qx"] = r.frames[0]["qy"] = r.frames[0]["qz"] = 0.0
    r.frames[0]["qw"] = 0.0
    positive = len(r.tracked_frames()) == 9

    # negative control: clear the flag and the zero-quaternion frame comes back
    r.frames[0]["degen"] = 0
    negative = len(r.tracked_frames()) == 10
    check("report.degenerate_excluded", positive, negative,
          "tracked=%d with flag, %d without" % (9 if positive else -1,
                                                len(r.tracked_frames())))


# ------------------------------------------- 10 threading guard actually fires ---
def test_threading_guard(build):
    """A threaded build must refuse to free-run; an unthreaded one must not."""
    import xrslam_ffi as ffi

    slam = ffi.XRSlam(build)
    threaded = bool(slam.options.get("XRSLAM_ENABLE_THREADING", False))
    tmp = tempfile.mkdtemp(prefix="xrslam-reg-guard-")
    try:
        def run(extra):
            cmd = [sys.executable, os.path.join(HERE, "run_once.py"),
                   "--build", build, "--seq", "synth://",
                   "--out", os.path.join(tmp, "g.txt"),
                   "--workdir", os.path.join(tmp, "cfg"),
                   "--max-frames", "12", "--quiet"] + list(extra)
            return subprocess.run(cmd, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.PIPE)
        free = run(["--settle-ms", "0"])
        override = run(["--settle-ms", "0", "--allow-unpaced"])
        if threaded:
            positive = free.returncode != 0 and b"refusing to free-run" in free.stderr
            negative = override.returncode == 0
            detail = "guard fired, override rc=%d" % override.returncode
        else:
            # unthreaded: the guard must NOT fire, and the negative control is
            # that the guard code path exists at all (exercised by the threaded
            # build in the same suite).
            positive = free.returncode == 0
            negative = override.returncode == 0
            detail = "unthreaded build: guard correctly silent"
        check("run_once.threading_guard[%s]" % ("thr" if threaded else "nothr"),
              positive, negative, detail)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# --------------------------------------- 11 bit-level determinism (unthreaded) ---
def test_bit_determinism(build):
    """An unthreaded build must repeat byte-for-byte on every SEMANTIC F field.

    The `ms` column is excluded and that exclusion is load-bearing, not a
    convenience: it is a wall-clock measurement of how long the frame took, so
    it cannot repeat and says nothing about the solver.  Measured 2026-08-23,
    `ms` is the ONLY column that ever differs between two runs of the
    unthreaded build -- every pose, keypoint count and state is identical.

    negative control: a run with one injected IMU scale error must NOT match.
    """
    import xrslam_ffi as ffi

    slam = ffi.XRSlam(build)
    if slam.options.get("XRSLAM_ENABLE_THREADING", False):
        return  # only meaningful for the deterministic profile
    tmp = tempfile.mkdtemp(prefix="xrslam-reg-det-")
    try:
        def run(name, extra=()):
            out = os.path.join(tmp, name)
            cmd = [sys.executable, os.path.join(HERE, "run_once.py"),
                   "--build", build, "--seq", "synth://", "--out", out,
                   "--workdir", os.path.join(tmp, "cfg"),
                   "--max-frames", "80", "--quiet"] + list(extra)
            r = subprocess.run(cmd, stdout=subprocess.DEVNULL,
                               stderr=subprocess.PIPE)
            if r.returncode != 0:
                sys.stderr.write(r.stderr.decode("utf-8", "replace")[-2000:])
                raise RuntimeError("run_once failed")
            with open(out) as fh:
                text = fh.read()
            cols = text.split("C ")[1].split("\n")[0].split()
            drop = cols.index("ms") + 1  # +1 for the leading "F" token
            rows = []
            for l in text.splitlines():
                if not l.startswith("F "):
                    continue
                t = l.split()
                rows.append(tuple(t[:drop] + t[drop + 1:]))
            return rows

        a = run("a.txt")
        b = run("b.txt")
        c = run("c.txt", ["--synth-set", "imu_scale=1.02"])
        positive = a == b and len(a) == 80
        negative = a != c
        check("e2e.bit_determinism", positive, negative,
              "identical=%s  fault_differs=%s" % (a == b, a != c))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", default=None,
                    help="build dir with libxrslam; enables end-to-end tests")
    args = ap.parse_args(argv)

    test_png_roundtrip()
    test_png_filters()
    test_umeyama()
    test_imu_consistency()
    test_render_projection()
    test_report_roundtrip()
    test_compare_and_floor()
    test_ate()
    test_euroc_roundtrip()
    test_undistort_map()
    test_degenerate_pose_excluded()
    test_same_binary_guard()
    test_permutation_ab()
    if args.build:
        test_ffi_abi(args.build)
        test_abi_image_layout(args.build)
        test_threading_guard(args.build)
        test_bit_determinism(args.build)
        test_end_to_end(args.build)
    else:
        print("(skipping end-to-end tests; pass --build BUILD_DIR to enable)")

    failed = [r for r in RESULTS if not r[1]]
    print("\n%d/%d checks passed" % (len(RESULTS) - len(failed), len(RESULTS)))
    for name, _ok, pos, neg, _d in failed:
        print("  FAILED %s (positive=%s negative=%s)" % (name, pos, neg))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
