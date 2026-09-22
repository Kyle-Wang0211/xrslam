"""Regression orchestrator: repeats, noise floor, A/B, baseline check.

Every run is a separate process (XRSLAMManager is a singleton and the landmark
inspection slot is process-global), so repeats are genuinely independent and a
crash in one run does not take the harness with it.

The rule this tool exists to enforce: **no A/B verdict without a measured noise
floor from the same configuration**.  XRSLAM seeds its RANSAC from
std::random_device, so identical binary + identical input still disagree; a
difference that does not exceed that spread is not a finding.

Subcommands
  run       N repeats of one configuration -> reports + floor.txt
  diff      compare two reports, optionally against a floor
  ab        N repeats of A and of B -> within-group floor vs cross-group effect
  baseline  N repeats -> floor, then compare every run against a stored baseline
"""

import argparse
import os
import shlex
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import report as report_mod  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RUN_ONCE = os.path.join(HERE, "run_once.py")


# ------------------------------------------------------------------ driver ---
def run_repeats(build, seq, outdir, repeats, prefix="run", extra=None, quiet=True):
    os.makedirs(outdir, exist_ok=True)
    paths = []
    for i in range(repeats):
        out = os.path.join(outdir, "%s_%d.txt" % (prefix, i))
        cmd = [
            sys.executable,
            RUN_ONCE,
            "--build",
            build,
            "--seq",
            seq,
            "--out",
            out,
            "--workdir",
            os.path.join(outdir, "%s_cfg" % prefix),
            "--tag",
            "%s#%d" % (prefix, i),
        ]
        if quiet:
            cmd.append("--quiet")
        cmd += list(extra or [])
        sys.stderr.write("[run] %s #%d -> %s\n" % (prefix, i, os.path.basename(out)))
        p = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        if p.returncode != 0:
            sys.stderr.write(p.stderr.decode("utf-8", "replace")[-4000:])
            raise SystemExit("run %s#%d failed (rc=%d)" % (prefix, i, p.returncode))
        paths.append(out)
    return paths


def load_baseline(path):
    """A baseline is either one report file or a directory of them.

    A directory is strongly preferred for any build that is not
    bit-deterministic: a single stored run cannot express run-to-run spread, so
    checking against it falls back to the max-vs-max rule that this tool was
    measured to get wrong.
    """
    if os.path.isdir(path):
        files = sorted(f for f in os.listdir(path) if f.endswith(".txt"))
        if not files:
            raise SystemExit("no *.txt reports in baseline dir %s" % path)
        return [report_mod.Report.load(os.path.join(path, f)) for f in files]
    return [report_mod.Report.load(path)]


def load_all(paths):
    return [report_mod.Report.load(p) for p in paths]


def assert_same_binary(reports, label):
    """Every run in one group must have used the SAME libxrslam.

    This is not paranoia.  On 2026-08-23 a sibling build directory (created by
    copying another one, so sharing its `_deps` sub-builds by absolute path)
    relinked `build-v1/xrslam-interface/libxrslam.dylib` *while it was being
    used as the control arm*: the sha256 moved twice during one session.  A
    noise floor computed across two different binaries is not a noise floor,
    and it fails silently -- the numbers look plausible.

    Fix at the source with freeze_arm.py; this is the tripwire.
    """
    shas = {}
    for r in reports:
        shas.setdefault(r.meta.get("lib_sha256", "?"), []).append(
            r.meta.get("tag", "?"))
    if len(shas) > 1:
        lines = ["group %r used %d different libxrslam binaries:" % (label, len(shas))]
        for sha, tags in sorted(shas.items()):
            lines.append("  %s  <- %s" % (sha[:16], ", ".join(tags)))
        lines.append("freeze the build first: python3 freeze_arm.py "
                     "--build <dir> --out <frozen>")
        raise SystemExit("\n".join(lines))
    return list(shas)[0] if shas else "?"


# ------------------------------------------------------------------- floor ---
def save_floor(floor, path):
    with open(path, "w") as fh:
        fh.write("# xrslam-regression noise floor\n")
        for k in sorted(floor):
            fh.write("%s %.9g\n" % (k, floor[k]))


def load_floor(path):
    out = {}
    with open(path) as fh:
        for line in fh:
            if line.startswith("#") or not line.strip():
                continue
            k, v = line.split()
            out[k] = float(v)
    return out


# Meta keys that must agree before a baseline comparison means anything.
# `max_frames` is in here because getting it wrong is silent: the run simply
# plays a different number of frames and every count metric explodes, which
# reads exactly like a regression.
COMPARABLE_META = ("seq", "max_frames", "pace", "settle_ms", "slam_config",
                   "sensor_config", "config_from_string", "opt_threading",
                   "opt_lowlatency", "opt_inspection")


def assert_comparable_meta(base, reports, allow_drift=False):
    """Refuse to compare a baseline against runs configured differently.

    `allow_drift` downgrades the refusal to a printed warning.  It exists for
    deliberate fault injection (change the input, prove the baseline notices),
    which is the only legitimate reason to want a mismatch -- and it prints,
    so a mismatched comparison can never happen quietly.
    """
    diffs = []
    for r in reports:
        for k in COMPARABLE_META:
            bv, rv = base.meta.get(k), r.meta.get(k)
            if bv is None or rv is None:
                continue
            if str(bv) != str(rv):
                d = "%s: baseline=%s current=%s" % (k, bv, rv)
                if d not in diffs:
                    diffs.append(d)
        for k in base.meta:
            if not k.startswith("src_synth_"):
                continue
            if str(base.meta[k]) != str(r.meta.get(k)):
                d = "%s: baseline=%s current=%s" % (k, base.meta[k],
                                                    r.meta.get(k))
                if d not in diffs:
                    diffs.append(d)
    if diffs:
        msg = ("baseline and current runs are not comparable:\n  "
               + "\n  ".join(diffs))
        if not allow_drift:
            raise SystemExit(
                msg + "\nRe-run with matching --extra (the baseline's own "
                      "settings are recorded in its M lines), record a new "
                      "baseline, or pass --allow-meta-drift if you are "
                      "deliberately injecting a fault.")
        sys.stderr.write("WARNING (--allow-meta-drift): " + msg + "\n")
    return diffs


def print_floor(floor):
    print("noise floor (worst over %d pairs of %d same-config runs):"
          % (int(floor.get("_pairs", 0)), int(floor.get("_runs", 0))))
    for k in report_mod.METRICS:
        print("  %-18s %14.6g" % (k, floor.get(k, 0.0)))


def summarise(reports):
    print("per-run summary:")
    keys = ("n_frames", "n_tracked", "first_track_idx", "ate_se3_rmse",
            "ate_sim3_scale", "pose_lag_max_ms", "wall_s")
    print("  " + "  ".join("%-16s" % k for k in keys))
    for r in reports:
        print("  " + "  ".join("%-16s" % str(r.summary.get(k, "-")) for k in keys))


# ---------------------------------------------------------------- commands ---
def cmd_run(args):
    paths = run_repeats(args.build, args.seq, args.out, args.repeats,
                        extra=shlex.split(args.extra))
    reps = load_all(paths)
    sha = assert_same_binary(reps, "run")
    print("lib_sha256 %s (identical across all %d runs)" % (sha[:16], len(reps)))
    summarise(reps)
    if len(reps) >= 2:
        floor = report_mod.noise_floor(reps, warmup=args.warmup)
        print_floor(floor)
        fp = os.path.join(args.out, "floor.txt")
        save_floor(floor, fp)
        print("floor written to %s" % fp)
    return 0


def cmd_diff(args):
    a = report_mod.Report.load(args.a)
    b = report_mod.Report.load(args.b)
    d = report_mod.compare(a, b, warmup=args.warmup)
    floor = load_floor(args.floor) if args.floor else None
    print("diff %s  vs  %s" % (args.a, args.b))
    print(report_mod.format_diff(d, floor, args.margin))
    if floor:
        label, exceed = report_mod.verdict(d, floor, args.margin)
        print("verdict: %s" % label)
        for e in exceed:
            print("  exceeds: %s" % e)
        return 0 if label == "WITHIN_NOISE" else 2
    return 0


def _cross(reps_a, reps_b, warmup=0):
    worst = dict((k, 0.0) for k in report_mod.METRICS)
    for ra in reps_a:
        for rb in reps_b:
            d = report_mod.compare(ra, rb, warmup=warmup)
            for k in report_mod.METRICS:
                if d[k] > worst[k]:
                    worst[k] = d[k]
    return worst


def cmd_ab(args):
    pa = run_repeats(args.build_a, args.seq_a, args.out, args.repeats,
                     prefix="A", extra=shlex.split(args.extra_a))
    pb = run_repeats(args.build_b, args.seq_b or args.seq_a, args.out,
                     args.repeats, prefix="B", extra=shlex.split(args.extra_b))
    ra, rb = load_all(pa), load_all(pb)
    sha_a = assert_same_binary(ra, "A")
    sha_b = assert_same_binary(rb, "B")
    if sha_a == sha_b and args.build_a != args.build_b:
        raise SystemExit(
            "arms A and B are byte-identical libraries (sha %s).\n"
            "  Nothing is being compared. Did one build dir relink the other?"
            % sha_a[:16])
    print("\nA lib_sha256 %s\nB lib_sha256 %s" % (sha_a[:16], sha_b[:16]))
    print("\n--- A ---")
    summarise(ra)
    print("\n--- B ---")
    summarise(rb)

    fa = report_mod.noise_floor(ra, warmup=args.warmup)
    fb = report_mod.noise_floor(rb, warmup=args.warmup)
    floor = dict((k, max(fa[k], fb[k])) for k in report_mod.METRICS)
    floor["_pairs"] = fa["_pairs"] + fb["_pairs"]
    floor["_runs"] = fa["_runs"] + fb["_runs"]
    print("")
    print_floor(floor)
    save_floor(floor, os.path.join(args.out, "floor.txt"))

    effect = _cross(ra, rb, args.warmup)
    print("\nA-vs-B effect (worst over %d cross pairs):" % (len(ra) * len(rb)))
    print(report_mod.format_diff(effect, floor, args.margin))

    # The verdict comes from the permutation test, NOT from cross-max vs
    # within-max: those are maxima over different numbers of pairs, so the
    # naive rule reports DIFFERENT even when both arms are the same binary.
    if len(ra) >= 2 and len(rb) >= 2:
        label, exceed, obs, thr = report_mod.permutation_ab(
            ra, rb, warmup=args.warmup, quantile=args.quantile)
        print("\npermutation test (relabel the %d runs into %d+%d, p%d null):"
              % (len(ra) + len(rb), len(ra), len(rb), int(args.quantile * 100)))
        for m in report_mod.METRICS:
            flag = "  " if obs[m] <= thr[m] + 1e-12 else "!!"
            print("%s%-18s %14.6g   null %14.6g" % (flag, m, obs[m], thr[m]))
    else:
        label, exceed = report_mod.verdict(effect, floor, args.margin)
        print("\n(fewer than 2 runs per arm: falling back to the floor rule)")
    print("verdict: %s" % label)
    for e in exceed:
        print("  exceeds: %s" % e)
    return 0 if label == "WITHIN_NOISE" else 2


def cmd_baseline(args):
    paths = run_repeats(args.build, args.seq, args.out, args.repeats,
                        extra=shlex.split(args.extra))
    reps = load_all(paths)
    assert_same_binary(reps, "baseline")
    summarise(reps)
    if args.set:
        if len(paths) > 1:
            os.makedirs(args.set, exist_ok=True)
            for i, pth in enumerate(paths):
                shutil.copyfile(pth, os.path.join(args.set, "base_%d.txt" % i))
            print("baseline group (%d runs) written to %s/" % (len(paths), args.set))
        else:
            shutil.copyfile(paths[0], args.set)
            print("baseline snapshot written to %s" % args.set)
        return 0
    bases = load_baseline(args.check)
    for b in bases:
        assert_comparable_meta(b, reps, args.allow_meta_drift)
    floor = report_mod.noise_floor(reps, warmup=args.warmup)
    print_floor(floor)
    worst = _cross(bases, reps, args.warmup)
    print("\nbaseline-vs-current (worst over %d pairs):"
          % (len(bases) * len(reps)))
    print(report_mod.format_diff(worst, floor, args.margin))

    if len(bases) >= 2 and len(reps) >= 2:
        # Same reasoning as cmd_ab: a stored-single-run baseline compared
        # against a within-group floor is the biased max-vs-max rule.  With a
        # baseline GROUP the permutation test applies and is calibrated.
        label, exceed, obs, thr = report_mod.permutation_ab(
            bases, reps, warmup=args.warmup, quantile=args.quantile)
        print("\npermutation test (%d baseline runs vs %d current, p%d null):"
              % (len(bases), len(reps), int(args.quantile * 100)))
        for m in report_mod.METRICS:
            flag = "  " if obs[m] <= thr[m] + 1e-12 else "!!"
            print("%s%-18s %14.6g   null %14.6g" % (flag, m, obs[m], thr[m]))
    else:
        print("\n(baseline is a single run: falling back to the floor rule, "
              "which is only calibrated for a bit-deterministic build)")
        label, exceed = report_mod.verdict(worst, floor, args.margin)
    print("verdict: %s" % label)
    for e in exceed:
        print("  exceeds: %s" % e)
    return 0 if label == "WITHIN_NOISE" else 2


def build_parser():
    ap = argparse.ArgumentParser(description="XRSLAM regression harness")
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="N repeats of one config; measure noise floor")
    r.add_argument("--build", required=True)
    r.add_argument("--seq", default="synth://")
    r.add_argument("--out", required=True)
    r.add_argument("--repeats", type=int, default=3)
    r.add_argument("--extra", default="",
                   help="quoted extra args forwarded to run_once.py, "
                        "e.g. --extra \"--pace 1.0 --max-frames 120\"")
    r.add_argument("--warmup", type=int, default=0,
                   help="drop the first N frames after each run's own "
                        "first tracked frame before comparing poses")
    r.set_defaults(fn=cmd_run)

    d = sub.add_parser("diff", help="compare two reports")
    d.add_argument("a")
    d.add_argument("b")
    d.add_argument("--floor", default=None)
    d.add_argument("--margin", type=float, default=1.0)
    d.add_argument("--warmup", type=int, default=0,
                   help="drop the first N frames after each run's own "
                        "first tracked frame before comparing poses")
    d.set_defaults(fn=cmd_diff)

    a = sub.add_parser("ab", help="A/B two builds or two configs")
    a.add_argument("--build-a", required=True)
    a.add_argument("--build-b", required=True)
    a.add_argument("--seq-a", default="synth://")
    a.add_argument("--seq-b", default=None)
    a.add_argument("--out", required=True)
    a.add_argument("--repeats", type=int, default=3)
    a.add_argument("--margin", type=float, default=1.0)
    a.add_argument("--quantile", type=float, default=0.95,
                   help="null quantile used as the permutation threshold")
    a.add_argument("--extra-a", default="", help="quoted extra args for arm A")
    a.add_argument("--extra-b", default="", help="quoted extra args for arm B")
    a.add_argument("--warmup", type=int, default=0,
                   help="drop the first N frames after each run's own "
                        "first tracked frame before comparing poses")
    a.set_defaults(fn=cmd_ab)

    b = sub.add_parser("baseline", help="snapshot or check against a stored baseline")
    b.add_argument("--build", required=True)
    b.add_argument("--seq", default="synth://")
    b.add_argument("--out", required=True)
    b.add_argument("--repeats", type=int, default=3)
    b.add_argument("--margin", type=float, default=1.0)
    b.add_argument("--set", default=None, help="write baseline snapshot here")
    b.add_argument("--check", default=None, help="compare against this baseline")
    b.add_argument("--extra", default="", help="quoted extra args for run_once.py")
    b.add_argument("--quantile", type=float, default=0.95,
                   help="null quantile used as the permutation threshold")
    b.add_argument("--allow-meta-drift", action="store_true",
                   help="compare anyway when the baseline and the current runs "
                        "were configured differently (prints what differs)")
    b.add_argument("--warmup", type=int, default=0,
                   help="drop the first N frames after each run's own "
                        "first tracked frame before comparing poses")
    b.set_defaults(fn=cmd_baseline)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.cmd == "baseline" and not (args.set or args.check):
        raise SystemExit("baseline needs --set or --check")
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
