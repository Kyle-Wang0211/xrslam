# XRSLAM regression harness

XRSLAM is a usable algorithm, not a maintained product: three releases, the last
`v0.6.0` (2024-01); algorithm/accuracy issues open and unanswered; one consumer
outside the repo (SenseTime's own `xrapi`, itself stalled at 2024-06). There is
no upstream CI and the paper numbers are not a prediction of what this tree
does. **This harness is the only regression baseline we get.**

Pure Python 3, standard library only — no numpy, Pillow, cv2. It drives the real
`libxrslam` through `ctypes` using only the typed C getters, mirroring the
`dart:ffi` contract we intend to ship.

## Run it

```sh
# 1. freeze the build under test (see "Why freeze" below -- this is not optional)
python3 freeze_arm.py --build ../../build-v1 --out /tmp/arms/default \
        --verify-option XRSLAM_ENABLE_THREADING=OFF

# 2. self-test: every check ships a negative control
python3 selftest.py --build /tmp/arms/default

# 3. N repeats of one config -> per-metric noise floor
python3 regress.py run --build /tmp/arms/default --out /tmp/out \
        --repeats 5 --extra "--max-frames 160"

# 4. check against the stored baseline GROUP
python3 regress.py baseline --build /tmp/arms/default --out /tmp/chk \
        --repeats 5 --extra "--max-frames 160" \
        --check baselines/synth160_default

# 5. A/B two builds (freeze both first!)
python3 regress.py ab --build-a /tmp/arms/default --build-b /tmp/arms/other \
        --out /tmp/ab --repeats 5 --extra-a "--max-frames 160" \
                                  --extra-b "--max-frames 160"
```

Exit code 0 = `WITHIN_NOISE`, 2 = `DIFFERENT`.

The mobile profile must be replayed with `--pace 1.0` (see below), so its
baseline is checked with `--extra "--max-frames 160 --pace 1.0"` against
`baselines/synth160_mobile`.

## How the verdict is made (read this before trusting a result)

`ab` and `baseline` decide with a **permutation test**, not by comparing a
cross-group maximum against a within-group maximum. Two things forced that,
both measured on 2026-08-23:

* **max-vs-max is biased by construction.** With 4+4 runs there are 16 cross
  pairs but only 6+6 within pairs, so the cross maximum is larger even when both
  groups are the same distribution. A/B-ing the mobile build **against itself**
  returned `DIFFERENT` on 5 metrics. A rule that fails on identical inputs
  cannot judge a change.
* **a permutation test on a maximum has no power.** The largest cross-pair
  distance survives almost every relabelling, so the null collapses onto the
  observed value. With that statistic, an unmistakable injected fault on the
  *bit-deterministic* build (within-group distance exactly 0) still came back
  `WITHIN_NOISE`.

So the statistic is a **mean contrast**: `mean(between-group distance) -
mean(within-group distance)`, and the threshold is the p95 of that same
statistic over every relabelling of the pooled runs into the two group sizes.
Observed and null are then the same statistic and the bias cancels.

Verified in both directions on both profiles:

| case | verdict |
|---|---|
| mobile binary vs itself (4+4) | `WITHIN_NOISE` |
| deterministic build, clean vs `imu_scale=1.02` | `DIFFERENT` |
| mobile build, clean vs `imu_scale=1.02` | `DIFFERENT` |
| stored baseline group vs fresh runs, both profiles | `WITHIN_NOISE` |
| stored baseline group vs `imu_scale=1.02`, both profiles | `DIFFERENT` |

A baseline may be a single report file or a **directory of reports**. Use a
directory for anything that is not bit-deterministic: one stored run cannot
express run-to-run spread, so it falls back to the max-vs-max rule above. The
shipped baselines are directories of 5 runs each.

## Input

We have no captured sequence yet, so the harness manufactures one.

* `synth://` — closed-form trajectory, analytically differentiated to gyro +
  specific-force IMU, with a fixed landmark cloud rendered as anti-aliased
  Gaussian blobs. Ships its own ground truth, so ATE is meaningful.
  `selftest.py` re-integrates the emitted IMU and checks it reproduces the
  analytic trajectory (0.19 mm / 0.00006° over 12 s) — that is what makes the
  frame conventions verifiable instead of merely asserted.
* `euroc://DIR`, `tum://DIR` — the on-disk EuRoC/TUM-VI layout the upstream
  player reads (`xrslam-pc/player/src/IO/`). Timestamps ns, IMU column order
  `t,wx,wy,wz,ax,ay,az`. `dataset.export_euroc()` writes a synthetic sequence in
  that layout, so the real-dataset code path is exercised with no real dataset.

## Output

One line-oriented, diff-friendly text report per run (`report v2`):
`M` metadata, `F` per-frame, `S` summary. Per frame: pose, state, **tracked /
inlier / detected keypoint counts and mapped landmarks** (read from
`XRSLAMGetHealth`), core frame sequence, IMU samples merged into the frame,
degeneracy and settle flags, timestamps.

## Measured facts (2026-08-23, this tree @ `d052dc3`, synth:// 160 frames)

These are measurements, not estimates. Re-measure before trusting them again.

| profile | mode | ATE SE3 RMSE | sim3 scale | noise floor `pos_p95` |
|---|---|---|---|---|
| `THREADING=OFF` | free-running | 0.0212 m | 1.0334 | **0** (bit-exact) |
| `THREADING=ON` (only that flag) | free-running | 0.069–0.127 m | 0.83–0.95 | 1.49 m |
| `THREADING=ON` | auto-settle | 0.0227–0.0239 m | 1.034–1.037 | 0.021 m |
| mobile (`THREADING+LOWLATENCY+CONFIG_FROM_STRING`) | realtime pace | 0.0107–0.0180 m | 1.006–1.032 | 0.039 m |

Three things follow, and each one contradicts something that looked obvious:

1. **The unthreaded solver is bit-deterministic.** Over 10 pairs of 5 runs the
   position floor is exactly 0 and every semantic field of every `F` line is
   byte-identical; only the wall-clock `ms` column moves. An earlier revision of
   `report.py` claimed XRSLAM cannot repeat because it seeds RANSAC from
   `std::random_device`. That is wrong: `Ransac::solve` calls
   `lotbox.seed(seed)` with `seed` defaulting to 0 (`utility/ransac.h:32`).
   `LotBox`'s constructor does reach `random_device`, but the value is
   overwritten before anything is drawn from it.

2. **Free-running a threaded build measures the harness, not the solver.**
   With `XRSLAM_ENABLE_THREADING=ON`, `XRSLAMRunOneFrame` hands the frame to a
   background worker and returns; sampling the pose immediately afterwards is a
   race. Free-running made the threaded arm look 10× worse and wildly
   nondeterministic (scale 0.40–0.53 for the mobile profile). Replayed at
   realtime pace the same binary lands at ATE 0.011 m — *better* than the
   unthreaded arm. `run_once.py` now **refuses** to free-run a threaded build
   unless you pass `--allow-unpaced`, and defaults to quiescing on
   `XRSLAMHealth.core_frame_seq`. Threading changes *when* the answer is
   available, not what it is.

3. **One degenerate frame dominated the accuracy number.** The first frame that
   reports `TRACKING_SUCCESS` carries `q = (0,0,0,0)` — not a rotation — while
   `latest_pose_degenerate` still reads 0. Counting it put ATE at 0.0893 m;
   excluding it gives 0.0116 m. `Report.tracked_frames()` now filters on a
   zero-quaternion test OR'd with the library flag.

## Why freeze (`freeze_arm.py`)

Two build dirs created by copying one another share their `_deps` sub-builds by
absolute path. Building either relinks the other's artifacts: during this
session `build-v1/xrslam-interface/libxrslam.dylib` changed sha256 twice while
it was in use as a control arm. A control arm that mutates between the A run and
the B run is not a control arm, and it fails silently — the numbers stay
plausible.

`freeze_arm.py` copies the dylib plus `CMakeCache.txt` somewhere the build
system has never heard of. `regress.py` additionally refuses to compute a noise
floor or an A/B verdict across runs whose `lib_sha256` disagree.

`freeze_arm.py --verify-option XRSLAM_ENABLE_THREADING=...` checks the
**library's own** `XRSLAMHealth.threading_enabled`, not just the cache text.

## Known gaps

* `--warmup N` (drop the first N frames after each run's own first tracked
  frame) is implemented and defaults to 0. It was added on the hypothesis that
  the threaded arm's spread came from post-initialisation transients. **It did
  not help** — 6/8 runs were flagged either way at every warmup from 0 to 20.
  The hypothesis was wrong; the option is kept because it is harmless and the
  measurement is worth preserving, not because it fixes anything.

* `XRSLAMGetLandmarks` returns 0 points for the whole run while
  `XRSLAMGetHealth.landmarks_published` is also 0 and
  `inspection_compiled_out == 0`. The inspection slot
  `sliding_window_landmarks` is simply never populated in this tree, so the
  `landmarks` / `lm_*` columns are structurally dead. They are still recorded so
  the day they start moving is visible.
* `XRSLAMGetBias` always returns zeros (`bias_channel_populated == 0`); nothing
  in `xrslam/src` writes those inspection slots.
* Nothing here has run against a real capture, and nothing has run on-device.
  ATE numbers are against a synthetic pinhole sequence with a noiseless IMU;
  they bound *self-consistency*, not real-world accuracy.
* The harness only feeds `SENSOR_CAMERA` + accel + gyro. Depth is untouched.
