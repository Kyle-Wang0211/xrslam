"""Run one XRSLAM session over one sequence and emit a regression report.

Deliberately one session per process: XRSLAMManager is a singleton and the
landmark inspection slot is process-global, so repeats must be separate
processes to be independent.  `regress.py` drives this file via subprocess.

Usage:
  python3 run_once.py --build BUILD_DIR --seq synth:// --out report.txt
  python3 run_once.py --build BUILD_DIR --seq euroc:///data/MH_01 \
                      --slam-config a.yaml --sensor-config b.yaml --out r.txt
"""

import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import dataset  # noqa: E402
import report as report_mod  # noqa: E402
import synth  # noqa: E402
import xrslam_ffi as ffi  # noqa: E402


def build_parser():
    ap = argparse.ArgumentParser(description="single XRSLAM regression run")
    ap.add_argument("--build", required=True, help="cmake build dir holding libxrslam")
    ap.add_argument("--seq", default="synth://", help="synth:// | euroc://DIR | tum://DIR")
    ap.add_argument("--out", required=True, help="report output path")
    ap.add_argument("--slam-config", default=None)
    ap.add_argument("--sensor-config", default=None)
    ap.add_argument("--truth", default=None, help="ground-truth txt (t qx qy qz qw tx ty tz)")
    ap.add_argument("--workdir", default=None, help="where generated configs go")
    ap.add_argument("--max-frames", type=int, default=0, help="0 = whole sequence")
    ap.add_argument("--pace", type=float, default=0.0,
                    help="sleep to emulate realtime playback (1.0 = realtime)")
    ap.add_argument("--tag", default="", help="free-form label recorded in metadata")
    ap.add_argument("--synth-set", action="append", default=[],
                    metavar="KEY=VALUE", help="override a SynthParams field")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--settle-ms", type=float, default=-1.0,
                    help="after RunOneFrame, poll XRSLAMHealth.core_frame_seq "
                         "for up to this many ms before sampling the pose. "
                         "-1 = auto (200ms when the library reports threading, "
                         "0 otherwise). 0 disables.")
    ap.add_argument("--allow-unpaced", action="store_true",
                    help="permit a threaded build to run with neither --pace "
                         "nor --settle-ms. Produces measurably meaningless "
                         "numbers; see the threading guard in this file.")
    return ap


def parse_synth_overrides(items):
    kw = {}
    for it in items:
        k, _, v = it.partition("=")
        k = k.strip()
        v = v.strip()
        if v.lower() in ("true", "false"):
            kw[k] = v.lower() == "true"
        elif v.startswith("(") or v.startswith("["):
            kw[k] = tuple(float(x) for x in v.strip("()[]").split(","))
        else:
            try:
                kw[k] = int(v)
            except ValueError:
                kw[k] = float(v)
    return kw


def main(argv=None):
    args = build_parser().parse_args(argv)
    scheme, _rest = dataset.parse_uri(args.seq)

    workdir = args.workdir or os.path.dirname(os.path.abspath(args.out)) or "."
    os.makedirs(workdir, exist_ok=True)

    params = None
    if scheme == "synth":
        params = synth.SynthParams(**parse_synth_overrides(args.synth_set))
    src = dataset.open_source(args.seq, params=params)

    if scheme == "synth":
        slam_cfg, sensor_cfg = src.write_configs(workdir)
    else:
        if not args.slam_config or not args.sensor_config:
            raise SystemExit(
                "--slam-config and --sensor-config are required for %s://" % scheme
            )
        slam_cfg = os.path.abspath(args.slam_config)
        sensor_cfg = os.path.abspath(args.sensor_config)

    slam = ffi.XRSlam(args.build)
    rep = report_mod.Report()
    rep.meta.update(
        {
            "tool": "pw_tools/regression",
            "build_dir": slam.build_dir,
            "lib": slam.lib_path,
            "lib_sha256": slam.lib_sha256,
            "config_from_string": int(slam.config_from_string),
            "opt_threading": int(slam.options.get("XRSLAM_ENABLE_THREADING", False)),
            "opt_lowlatency": int(slam.options.get("XRSLAM_LOWLATENCY_POSE", False)),
            "opt_inspection": int(
                slam.options.get("XRSLAM_ENABLE_DEBUG_INSPECTION", False)
            ),
            "seq": args.seq,
            "slam_config": os.path.basename(slam_cfg),
            "sensor_config": os.path.basename(sensor_cfg),
            "max_frames": args.max_frames,
            "pace": args.pace,
            "tag": args.tag or "-",
            "host": socket.gethostname(),
            "wall_start": "%.3f" % time.time(),
            "python": "%d.%d.%d" % sys.version_info[:3],
        }
    )
    for k, v in src.meta().items():
        rep.meta["src_" + k] = v

    slam.create(slam_cfg, sensor_cfg)
    rc, ver = slam.get_version()
    rep.meta["lib_version"] = ver if rc >= 0 else "rc=%d" % rc

    # ---- threading guard -------------------------------------------------
    # Ask the *library* whether it has background workers; the CMakeCache is a
    # text file that can lie about which .dylib is actually loaded.
    _hrc, _h0 = slam.get_health()
    threaded = bool(_h0.threading_enabled) if _hrc == ffi.XRSLAM_OK else False
    rep.meta["lib_threading"] = int(threaded)
    settle_ms = args.settle_ms
    if settle_ms < 0.0:
        settle_ms = 200.0 if threaded else 0.0
    if threaded and args.pace <= 0.0 and settle_ms <= 0.0 and not args.allow_unpaced:
        slam.destroy()
        raise SystemExit(
            "refusing to free-run a threaded build.\n"
            "  XRSLAM_ENABLE_THREADING is ON in %s, so XRSLAMRunOneFrame hands\n"
            "  the frame to a background worker and returns. Sampling the pose\n"
            "  immediately afterwards measures a race, not the solver: measured\n"
            "  2026-08-23, the same binary on the same input gave a 0.53 m\n"
            "  same-config spread free-running vs 2.5e-05 m at realtime pace.\n"
            "  Use --pace 1.0, or --settle-ms N, or --allow-unpaced to override."
            % slam.lib_path
        )
    rep.meta["settle_ms"] = "%.1f" % settle_ms
    rep.meta["allow_unpaced"] = int(args.allow_unpaced)

    have_acc = have_gyr = False
    _hrc_fin, h_fin = -1, None
    idx = 0
    push_fail_imu = 0
    push_fail_img = 0
    imu_ts = []
    frame_ts = []
    t_wall0 = time.time()
    seq_t0 = None
    try:
        for ev in src.events():
            if ev[0] == "imu":
                _, t, g, a = ev
                if seq_t0 is None:
                    seq_t0 = t
                imu_ts.append(t)
                # order matches the upstream players: gyroscope then accelerometer
                r1 = slam.push_gyro(t, g)
                r2 = slam.push_accel(t, a)
                have_gyr = have_gyr or r1 == ffi.XRSLAM_OK
                have_acc = have_acc or r2 == ffi.XRSLAM_OK
                if r1 != ffi.XRSLAM_OK or r2 != ffi.XRSLAM_OK:
                    push_fail_imu += 1
                continue

            _, t, w, h, buf, _nvis, _R, _p = ev
            if seq_t0 is None:
                seq_t0 = t
            if args.max_frames and idx >= args.max_frames:
                break
            if args.pace > 0.0:
                target = t_wall0 + (t - seq_t0) / args.pace
                now = time.time()
                if target > now:
                    time.sleep(target - now)
            _hrc_pre, h_pre = slam.get_health()
            seq_before = h_pre.core_frame_seq
            t_start = time.time()
            push_rc = slam.push_image(t, w, h, buf)
            if push_rc != ffi.XRSLAM_OK:
                push_fail_img += 1
            if have_acc and have_gyr and push_rc == ffi.XRSLAM_OK:
                slam.run_one_frame()
            # Quiesce: wait until the core reports it has taken a NEW frame.
            # core_frame_seq is the only signal in the C API that distinguishes
            # "the worker processed it" from "we asked and came straight back".
            settled = 1
            if settle_ms > 0.0:
                deadline = time.time() + settle_ms / 1000.0
                while True:
                    _hrc, h_now = slam.get_health()
                    if h_now.core_frame_seq != seq_before:
                        break
                    if time.time() >= deadline:
                        settled = 0
                        break
                    time.sleep(0.0005)
            ms = (time.time() - t_start) * 1000.0

            _src, state = slam.get_state()
            _rcp, pose = slam.get_body_pose()
            lm_rc, lm_n, _flat = slam.get_landmarks()
            _seeded, fusion_total = slam.get_depth_fusion_stats()
            _hrc, hh = slam.get_health()

            frame_ts.append(t)
            rep.frames.append(
                {
                    "idx": idx,
                    "frame_ts": t,
                    "pose_ts": pose.timestamp,
                    "state": state,
                    "landmarks": lm_n,
                    "fusion_total": fusion_total,
                    "qx": pose.quaternion[0],
                    "qy": pose.quaternion[1],
                    "qz": pose.quaternion[2],
                    "qw": pose.quaternion[3],
                    "tx": pose.translation[0],
                    "ty": pose.translation[1],
                    "tz": pose.translation[2],
                    "push_rc": push_rc,
                    "lm_rc": lm_rc,
                    "ms": ms,
                    "det_kp": hh.detected_keypoints,
                    "trk_kp": hh.tracked_keypoints,
                    "inl_kp": hh.inlier_keypoints,
                    "mapped_lm": hh.mapped_landmarks,
                    "lm_published": hh.landmarks_published,
                    "lm_usable": hh.landmarks_usable,
                    "core_seq": hh.core_frame_seq,
                    "core_imu": hh.core_imu_samples,
                    # OR the library flag with our own zero-quaternion test.
                    # Measured 2026-08-23: the first TRACKING frame comes back
                    # with q = (0,0,0,0) and p = 0 while
                    # latest_pose_degenerate still reads 0, so trusting the
                    # flag alone lets a zero pose into ATE and into compare().
                    "degen": 1 if (hh.latest_pose_degenerate == 1 or
                                   (pose.quaternion[0] ** 2 +
                                    pose.quaternion[1] ** 2 +
                                    pose.quaternion[2] ** 2 +
                                    pose.quaternion[3] ** 2) < 1e-18) else 0,
                    "settled": settled,
                }
            )
            idx += 1
            if not args.quiet and idx % 25 == 0:
                sys.stderr.write(
                    "  frame %4d state=%-5s lm=%3d pose_ts=%.3f\n"
                    % (idx, ffi.STATE_NAMES.get(state, "?"), lm_n, pose.timestamp)
                )
        _hrc_fin, h_fin = slam.get_health()
    finally:
        slam.destroy()

    _fill_summary(rep, imu_ts, frame_ts, push_fail_imu, push_fail_img,
                  time.time() - t_wall0)
    if _hrc_fin == ffi.XRSLAM_OK:
        for k in ("image_accepted", "image_rejected", "accel_rejected",
                  "gyro_rejected", "reject_non_finite_ts",
                  "reject_non_monotonic_ts", "reject_bad_arg",
                  "domain_mismatch_events", "frames_run",
                  "frames_with_zero_imu", "shadow_overflow_drops",
                  "core_imu_starved_frames", "core_frame_seq",
                  "inspection_compiled_out", "bias_channel_populated"):
            rep.summary["h_" + k] = getattr(h_fin, k)
        rep.summary["h_max_abs_cam_imu_delta"] = "%.6f" % h_fin.max_abs_cam_imu_delta
    rep.summary["n_unsettled"] = sum(
        1 for f in rep.frames if f.get("settled", 1) == 0)
    rep.summary["n_degenerate_pose"] = sum(
        1 for f in rep.frames if f.get("degen", 0) == 1)
    for key, fld in (("trk_kp", "trk_kp"), ("inl_kp", "inl_kp"),
                     ("mapped_lm", "mapped_lm")):
        vals = [f[fld] for f in rep.tracked_frames() if fld in f]
        if vals:
            rep.summary["%s_med" % key] = _median(vals)
            rep.summary["%s_max" % key] = max(vals)

    if args.truth or (scheme == "synth"):
        truth = None
        if args.truth:
            truth = dataset.load_truth(args.truth)
        elif params is not None:
            truth = _synth_truth(params, frame_ts)
        if truth:
            for label, ws in (("se3", False), ("sim3", True)):
                a = report_mod.ate(rep, truth, with_scale=ws)
                if a:
                    rep.summary["ate_%s_rmse" % label] = "%.6f" % a["rmse"]
                    rep.summary["ate_%s_max" % label] = "%.6f" % a["max"]
                    rep.summary["ate_%s_n" % label] = a["n"]
                    if ws:
                        rep.summary["ate_sim3_scale"] = "%.6f" % a["scale"]

    rep.save(args.out)
    if not args.quiet:
        sys.stderr.write("wrote %s\n" % args.out)
        for k in sorted(rep.summary):
            sys.stderr.write("  S %s = %s\n" % (k, rep.summary[k]))
    return 0


def _synth_truth(params, frame_ts):
    import geom

    out = []
    for t in frame_ts:
        R = synth.traj_rotation(t)
        p = synth.traj_position(t)
        out.append((t, geom.quat_from_mat(R), p))
    return out


def _fill_summary(rep, imu_ts, frame_ts, push_fail_imu, push_fail_img, wall):
    tf = rep.tracked_frames()
    rep.summary["n_frames"] = len(rep.frames)
    rep.summary["n_tracked"] = len(tf)
    rep.summary["n_imu"] = len(imu_ts)
    rep.summary["first_track_idx"] = tf[0]["idx"] if tf else -1
    rep.summary["push_fail_imu"] = push_fail_imu
    rep.summary["push_fail_img"] = push_fail_img
    rep.summary["wall_s"] = "%.3f" % wall

    states = {}
    for f in rep.frames:
        states[f["state"]] = states.get(f["state"], 0) + 1
    for s, n in sorted(states.items()):
        rep.summary["state_%s" % ffi.STATE_NAMES.get(s, s)] = n

    lm_rcs = {}
    for f in rep.frames:
        lm_rcs[f["lm_rc"]] = lm_rcs.get(f["lm_rc"], 0) + 1
    rep.summary["lm_rc_hist"] = ";".join(
        "%s:%d" % (ffi.RC_NAMES.get(k, k), v) for k, v in sorted(lm_rcs.items())
    )

    # ---- timestamp diagnostics -------------------------------------------
    if len(frame_ts) > 1:
        d = [frame_ts[i + 1] - frame_ts[i] for i in range(len(frame_ts) - 1)]
        rep.summary["frame_dt_med"] = "%.6f" % _median(d)
        rep.summary["frame_dt_max"] = "%.6f" % max(d)
        rep.summary["frame_dt_min"] = "%.6f" % min(d)
    if len(imu_ts) > 1:
        d = [imu_ts[i + 1] - imu_ts[i] for i in range(len(imu_ts) - 1)]
        rep.summary["imu_dt_med"] = "%.6f" % _median(d)
        rep.summary["imu_dt_max"] = "%.6f" % max(d)
        rep.summary["imu_ts_nonmono"] = sum(1 for x in d if x <= 0.0)

    lag = [f["frame_ts"] - f["pose_ts"] for f in tf]
    if lag:
        rep.summary["pose_lag_med_ms"] = "%.3f" % (_median(lag) * 1000.0)
        rep.summary["pose_lag_max_ms"] = "%.3f" % (max(lag) * 1000.0)
        rep.summary["pose_lag_min_ms"] = "%.3f" % (min(lag) * 1000.0)
    rep.summary["pose_ts_repeat"] = _count_repeats([f["pose_ts"] for f in tf])
    rep.summary["pose_ts_regress"] = _count_regress([f["pose_ts"] for f in tf])

    ms = [f["ms"] for f in rep.frames]
    if ms:
        rep.summary["frame_ms_med"] = "%.3f" % _median(ms)
        rep.summary["frame_ms_max"] = "%.3f" % max(ms)


def _median(v):
    s = sorted(v)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _count_repeats(v):
    return sum(1 for i in range(1, len(v)) if v[i] == v[i - 1])


def _count_regress(v):
    return sum(1 for i in range(1, len(v)) if v[i] < v[i - 1])


if __name__ == "__main__":
    sys.exit(main())
