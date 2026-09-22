"""ctypes bindings for the XRSLAM C API (xrslam-interface/include/XRSLAM.h).

Deliberately mirrors the Dart:ffi contract we intend to ship:
  * only the typed XRSLAMGet* getters are used, never the void* XRSLAMGetResult,
  * out-params are always allocated by the caller and zeroed by the library,
  * the two-call idiom is used for the variable-length landmark channel,
  * XRSLAMCreate keeps its inverted upstream convention (1 = success).
"""

import ctypes
import hashlib
import os
import re

# --------------------------------------------------------------- error codes ---
XRSLAM_OK = 0
XRSLAM_INCOMPLETE = 1
XRSLAM_NO_NEW_DATA = 2
XRSLAM_ERR_BAD_ARG = -1
XRSLAM_ERR_BAD_CHANNEL = -2
XRSLAM_ERR_NOT_CREATED = -3
XRSLAM_ERR_INTERNAL = -4
XRSLAM_ERR_UNAVAILABLE = -5

RC_NAMES = {
    0: "OK",
    1: "INCOMPLETE",
    2: "NO_NEW_DATA",
    -1: "ERR_BAD_ARG",
    -2: "ERR_BAD_CHANNEL",
    -3: "ERR_NOT_CREATED",
    -4: "ERR_INTERNAL",
    -5: "ERR_UNAVAILABLE",
}

SENSOR_CAMERA = 0
SENSOR_DEPTH_CAMERA = 1
SENSOR_ACCELERATION = 2
SENSOR_GYROSCOPE = 3

STATE_INITIALIZING = 0
STATE_TRACKING_SUCCESS = 1
STATE_TRACKING_FAIL = 2
STATE_NAMES = {0: "INIT", 1: "TRACK", 2: "FAIL"}


# ------------------------------------------------------------------ structs ---
class XRSLAMImageExtension(ctypes.Structure):
    _fields_ = [
        ("exposure_time", ctypes.c_double),
        ("default_focus_distance", ctypes.c_double),
        ("focal_length", ctypes.c_double),
        ("focus_distance", ctypes.c_double),
    ]


class XRSLAMImage(ctypes.Structure):
    """MUST match XRSLAM.h exactly -- this struct is caller-allocated.

    [pw-regression] The trailing three fields (readout_time /
    timestamp_convention / reserved0) are NOT optional.  They were appended
    after width/height, taking sizeof from 48 to 64.  A 48-byte definition on
    this side still *runs*: XRSLAMPushSensorDataChecked happily reads 64 bytes
    from the 48-byte object we hand it, i.e. 16 bytes of unrelated Python heap,
    and interprets whatever is there as readout_time + timestamp_convention.
    It appeared to work only because the garbage happened to read back as 0
    (= XRSLAM_TS_UNSPECIFIED).  That is an out-of-bounds read, not a contract.
    `test_abi_image_layout` in selftest.py is the negative control.
    """

    _fields_ = [
        ("data", ctypes.POINTER(ctypes.c_ubyte)),
        ("timeStamp", ctypes.c_double),
        ("stride", ctypes.c_int),
        ("camera_id", ctypes.c_int),
        ("channel", ctypes.c_int),
        ("ext", ctypes.POINTER(XRSLAMImageExtension)),
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("readout_time", ctypes.c_double),
        ("timestamp_convention", ctypes.c_int32),
        ("reserved0", ctypes.c_int32),
    ]


class XRSLAMLandmarkStats(ctypes.Structure):
    _fields_ = [
        ("published", ctypes.c_int32),
        ("rejected_non_finite", ctypes.c_int32),
        ("rejected_untriangulated", ctypes.c_int32),
        ("returned", ctypes.c_int32),
    ]


class XRSLAMHealth(ctypes.Structure):
    """Mirrors XRSLAM.h; sizeof is pinned at 288 by static_assert in
    XRSLAMManager.cpp.  This is the richest diagnostic surface the C API has --
    in particular `core_frame_seq` (does the core actually see new frames?),
    `tracked_keypoints` / `inlier_keypoints` (the counts this harness is
    required to report) and `latest_pose_degenerate` (the pose is a zero
    quaternion; GetBodyPose still returns XRSLAM_OK for it)."""

    _fields_ = (
        [(n, ctypes.c_double) for n in (
            "last_image_timestamp", "last_imu_timestamp", "last_cam_imu_delta",
            "baseline_cam_imu_delta", "max_abs_cam_imu_delta",
            "cam_imu_delta_threshold", "baseline_drift_threshold",
            "max_abs_baseline_drift", "last_frame_ms")]
        + [(n, ctypes.c_int64) for n in (
            "image_accepted", "image_rejected", "accel_accepted",
            "accel_rejected", "gyro_accepted", "gyro_rejected",
            "reject_non_finite_ts", "reject_non_monotonic_ts", "reject_bad_arg",
            "domain_mismatch_events", "baseline_drift_events", "frames_run",
            "frames_with_zero_imu", "shadow_overflow_drops")]
        + [(n, ctypes.c_int32) for n in (
            "overall", "slam_state", "imu_samples_last_frame",
            "landmarks_published", "landmarks_usable",
            "landmarks_rejected_non_finite",
            "landmarks_rejected_untriangulated", "timestamp_convention",
            "domain_mismatch_active", "inspection_compiled_out",
            "threading_enabled", "bias_channel_populated")]
        + [("core_frame_timestamp", ctypes.c_double),
           ("core_frame_seq", ctypes.c_int64),
           ("core_imu_starved_frames", ctypes.c_int64)]
        + [(n, ctypes.c_int32) for n in (
            "core_imu_samples", "core_imu_samples_integrated",
            "detected_keypoints", "tracked_keypoints", "inlier_keypoints",
            "mapped_landmarks", "core_health_available",
            "latest_pose_degenerate")]
    )


class XRSLAMAcceleration(ctypes.Structure):
    _fields_ = [("data", ctypes.c_double * 3), ("timestamp", ctypes.c_double)]


class XRSLAMGyroscope(ctypes.Structure):
    _fields_ = [("data", ctypes.c_double * 3), ("timestamp", ctypes.c_double)]


class XRSLAMPose(ctypes.Structure):
    _fields_ = [
        ("quaternion", ctypes.c_double * 4),
        ("translation", ctypes.c_double * 3),
        ("timestamp", ctypes.c_double),
    ]


class XRSLAMIntrinsics(ctypes.Structure):
    _fields_ = [
        ("fx", ctypes.c_double),
        ("fy", ctypes.c_double),
        ("cx", ctypes.c_double),
        ("cy", ctypes.c_double),
    ]


class XRSLAMBias(ctypes.Structure):
    _fields_ = [("data", ctypes.c_double * 3)]


class XRSLAMIMUBias(ctypes.Structure):
    _fields_ = [("acc_bias", XRSLAMBias), ("gyr_bias", XRSLAMBias)]


# The header pins these with static_assert; mirror the same guard on this side
# so an ABI drift shows up as a Python-side failure instead of a memory bug.
assert ctypes.sizeof(XRSLAMImage) == 64, ctypes.sizeof(XRSLAMImage)
assert XRSLAMImage.width.offset == 40 and XRSLAMImage.height.offset == 44
assert ctypes.sizeof(XRSLAMHealth) == 288, ctypes.sizeof(XRSLAMHealth)
assert XRSLAMHealth.overall.offset == 184, XRSLAMHealth.overall.offset
assert XRSLAMHealth.core_frame_timestamp.offset == 232
assert ctypes.sizeof(XRSLAMLandmarkStats) == 16
assert ctypes.sizeof(XRSLAMPose) == 64, ctypes.sizeof(XRSLAMPose)
assert ctypes.sizeof(XRSLAMIMUBias) == 48, ctypes.sizeof(XRSLAMIMUBias)


# ------------------------------------------------------------- build lookup ---
LIB_NAMES = ("libxrslam.dylib", "libxrslam.so")


def find_library(build_dir):
    for sub in ("xrslam-interface", "", "lib"):
        for name in LIB_NAMES:
            cand = os.path.join(build_dir, sub, name)
            if os.path.isfile(cand):
                return cand
    raise IOError("no libxrslam.{dylib,so} under %s" % build_dir)


def read_cmake_cache(build_dir):
    """Return the XRSLAM_* option values recorded in CMakeCache.txt."""
    out = {}
    path = os.path.join(build_dir, "CMakeCache.txt")
    if not os.path.isfile(path):
        return out
    pat = re.compile(r"^(XRSLAM_[A-Z_]+):BOOL=(\w+)\s*$")
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            m = pat.match(line)
            if m:
                out[m.group(1)] = m.group(2).upper() in ("ON", "TRUE", "1", "YES")
    return out


def sha256_file(path, limit=None):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        while True:
            b = fh.read(1 << 20)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


# ------------------------------------------------------------------ wrapper ---
class XRSlam(object):
    def __init__(self, build_dir):
        self.build_dir = os.path.abspath(build_dir)
        self.lib_path = find_library(self.build_dir)
        self.options = read_cmake_cache(self.build_dir)
        self.config_from_string = bool(
            self.options.get("XRSLAM_CONFIG_FROM_STRING", False)
        )
        self.lib_sha256 = sha256_file(self.lib_path)
        self._lib = ctypes.CDLL(self.lib_path)
        self._created = False
        self._bind()

    def _bind(self):
        L = self._lib
        L.XRSLAMCreate.restype = ctypes.c_int
        L.XRSLAMCreate.argtypes = [
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.POINTER(ctypes.c_void_p),
        ]
        L.XRSLAMPushSensorDataChecked.restype = ctypes.c_int
        L.XRSLAMPushSensorDataChecked.argtypes = [ctypes.c_int, ctypes.c_void_p]
        L.XRSLAMRunOneFrame.restype = None
        L.XRSLAMRunOneFrame.argtypes = []
        L.XRSLAMDestroy.restype = None
        L.XRSLAMDestroy.argtypes = []
        for name, typ in (
            ("XRSLAMGetBodyPose", XRSLAMPose),
            ("XRSLAMGetCameraPose", XRSLAMPose),
            ("XRSLAMGetBias", XRSLAMIMUBias),
            ("XRSLAMGetIntrinsics", XRSLAMIntrinsics),
        ):
            fn = getattr(L, name)
            fn.restype = ctypes.c_int
            fn.argtypes = [ctypes.POINTER(typ)]
        L.XRSLAMGetState.restype = ctypes.c_int
        L.XRSLAMGetState.argtypes = [ctypes.POINTER(ctypes.c_int)]
        L.XRSLAMGetVersion.restype = ctypes.c_int
        L.XRSLAMGetVersion.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int32)]
        L.XRSLAMGetLandmarks.restype = ctypes.c_int
        L.XRSLAMGetLandmarks.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_int32),
        ]
        L.XRSLAMGetDepthFusionStats.restype = None
        L.XRSLAMGetDepthFusionStats.argtypes = [
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
        ]
        L.XRSLAMGetHealth.restype = ctypes.c_int
        L.XRSLAMGetHealth.argtypes = [ctypes.POINTER(XRSLAMHealth)]
        L.XRSLAMGetLandmarksEx.restype = ctypes.c_int
        L.XRSLAMGetLandmarksEx.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(ctypes.c_int32),
            ctypes.POINTER(XRSLAMLandmarkStats),
        ]
        L.XRSLAMTryGetLatestPose.restype = ctypes.c_int
        L.XRSLAMTryGetLatestPose.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]

    # -- lifecycle ----------------------------------------------------------
    def create(self, slam_yaml_path, sensor_yaml_path, product="pw-regression"):
        """Create the SLAM session.

        Honours the build's XRSLAM_CONFIG_FROM_STRING contract: when the build
        was configured with it ON (the mobile profile), XRSLAMCreate expects
        YAML *text*; when OFF it expects file paths.  Same C ABI, two
        contracts -- so the wrapper resolves it from CMakeCache.txt rather than
        making the caller guess.
        """
        if self.config_from_string:
            with open(slam_yaml_path, "rb") as fh:
                a = fh.read()
            with open(sensor_yaml_path, "rb") as fh:
                b = fh.read()
        else:
            a = slam_yaml_path.encode()
            b = sensor_yaml_path.encode()
        cfg = ctypes.c_void_p()
        rc = self._lib.XRSLAMCreate(a, b, b"", product.encode(), ctypes.byref(cfg))
        if rc != 1:  # upstream legacy: 1 = success, 0 = failure
            raise RuntimeError("XRSLAMCreate failed (rc=%d)" % rc)
        self._created = True
        return cfg

    def destroy(self):
        if self._created:
            self._lib.XRSLAMDestroy()
            self._created = False

    # -- input --------------------------------------------------------------
    def push_gyro(self, t, xyz):
        g = XRSLAMGyroscope()
        g.timestamp = t
        g.data[0], g.data[1], g.data[2] = xyz
        return self._lib.XRSLAMPushSensorDataChecked(
            SENSOR_GYROSCOPE, ctypes.byref(g)
        )

    def push_accel(self, t, xyz):
        a = XRSLAMAcceleration()
        a.timestamp = t
        a.data[0], a.data[1], a.data[2] = xyz
        return self._lib.XRSLAMPushSensorDataChecked(
            SENSOR_ACCELERATION, ctypes.byref(a)
        )

    def push_image(self, t, width, height, buf):
        """buf must be a writable ctypes-compatible byte buffer (bytearray ok)."""
        img = XRSLAMImage()  # zero-initialised by ctypes; required for width/height
        cbuf = (ctypes.c_ubyte * len(buf)).from_buffer(buf)
        img.data = ctypes.cast(cbuf, ctypes.POINTER(ctypes.c_ubyte))
        img.timeStamp = t
        img.stride = width
        img.camera_id = 0
        img.channel = 1
        img.width = width
        img.height = height
        # explicit, even though ctypes already zeroed them: these are the
        # fields a stale 48-byte header would have left as heap garbage.
        img.readout_time = 0.0
        img.timestamp_convention = 0  # XRSLAM_TS_UNSPECIFIED
        img.reserved0 = 0
        rc = self._lib.XRSLAMPushSensorDataChecked(SENSOR_CAMERA, ctypes.byref(img))
        del cbuf  # release the exported buffer view before `buf` can be resized
        return rc

    def run_one_frame(self):
        self._lib.XRSLAMRunOneFrame()

    # -- output -------------------------------------------------------------
    def get_state(self):
        v = ctypes.c_int(-999)
        rc = self._lib.XRSLAMGetState(ctypes.byref(v))
        return rc, v.value

    def get_body_pose(self):
        p = XRSLAMPose()
        rc = self._lib.XRSLAMGetBodyPose(ctypes.byref(p))
        return rc, p

    def get_camera_pose(self):
        p = XRSLAMPose()
        rc = self._lib.XRSLAMGetCameraPose(ctypes.byref(p))
        return rc, p

    def get_bias(self):
        b = XRSLAMIMUBias()
        rc = self._lib.XRSLAMGetBias(ctypes.byref(b))
        return rc, b

    def get_intrinsics(self):
        k = XRSLAMIntrinsics()
        rc = self._lib.XRSLAMGetIntrinsics(ctypes.byref(k))
        return rc, k

    def get_version(self):
        n = ctypes.c_int32(0)
        rc = self._lib.XRSLAMGetVersion(None, ctypes.byref(n))
        if rc < 0:
            return rc, ""
        cap = ctypes.c_int32(n.value + 1)
        buf = ctypes.create_string_buffer(cap.value)
        rc = self._lib.XRSLAMGetVersion(buf, ctypes.byref(cap))
        return rc, buf.value.decode("utf-8", "replace")

    _lm_buf = None
    _lm_cap = 0

    def get_landmarks(self):
        """Two-call idiom, buffer reused across frames (0 allocations steady state).

        Returns (rc, count, flat_xyz_list). `flat` is empty when the channel is
        compiled out (XRSLAM_ERR_UNAVAILABLE) so callers can tell that apart
        from 'tracking but no points'.
        """
        if self._lm_buf is None:
            self._lm_cap = 1024
            self._lm_buf = (ctypes.c_double * (3 * self._lm_cap))()
        n = ctypes.c_int32(self._lm_cap)
        rc = self._lib.XRSLAMGetLandmarks(self._lm_buf, ctypes.byref(n))
        if rc == XRSLAM_INCOMPLETE:
            need = ctypes.c_int32(0)
            self._lib.XRSLAMGetLandmarks(None, ctypes.byref(need))
            self._lm_cap = max(need.value * 2, self._lm_cap * 2)
            self._lm_buf = (ctypes.c_double * (3 * self._lm_cap))()
            n = ctypes.c_int32(self._lm_cap)
            rc = self._lib.XRSLAMGetLandmarks(self._lm_buf, ctypes.byref(n))
        if rc < 0:
            return rc, 0, []
        return rc, n.value, self._lm_buf[: 3 * n.value]

    def get_health(self):
        """Full XRSLAMHealth snapshot. Returns (rc, XRSLAMHealth).

        This is where the per-frame `tracked`/`inlier` counts live
        (detected_keypoints / tracked_keypoints / inlier_keypoints /
        mapped_landmarks) and where `core_frame_seq` -- the only honest
        "has the core actually consumed a new frame?" signal -- comes from.
        """
        h = XRSLAMHealth()
        rc = self._lib.XRSLAMGetHealth(ctypes.byref(h))
        return rc, h

    _lmx_buf = None
    _lmx_flags = None
    _lmx_cap = 0

    def get_landmarks_ex(self):
        """Two-call idiom + filter accounting.

        Returns (rc, count, stats).  `stats` explains a zero count: a run can
        report 0 usable landmarks while `published` is large, because every
        point was rejected as non-finite or untriangulated.  Plain
        XRSLAMGetLandmarks cannot tell those two apart.
        """
        if self._lmx_buf is None:
            self._lmx_cap = 1024
            self._lmx_buf = (ctypes.c_double * (3 * self._lmx_cap))()
            self._lmx_flags = (ctypes.c_ubyte * self._lmx_cap)()
        st = XRSLAMLandmarkStats()
        n = ctypes.c_int32(self._lmx_cap)
        rc = self._lib.XRSLAMGetLandmarksEx(
            self._lmx_buf, self._lmx_flags, ctypes.byref(n), ctypes.byref(st)
        )
        if rc == XRSLAM_INCOMPLETE:
            need = ctypes.c_int32(0)
            self._lib.XRSLAMGetLandmarksEx(None, None, ctypes.byref(need), None)
            self._lmx_cap = max(need.value * 2, self._lmx_cap * 2)
            self._lmx_buf = (ctypes.c_double * (3 * self._lmx_cap))()
            self._lmx_flags = (ctypes.c_ubyte * self._lmx_cap)()
            n = ctypes.c_int32(self._lmx_cap)
            rc = self._lib.XRSLAMGetLandmarksEx(
                self._lmx_buf, self._lmx_flags, ctypes.byref(n), ctypes.byref(st)
            )
        return rc, (n.value if rc >= 0 else 0), st

    def try_get_latest_pose(self):
        """Low-latency non-blocking pull. Returns (rc, pose7, timestamp)."""
        buf = (ctypes.c_double * 7)()
        ts = ctypes.c_double(0.0)
        rc = self._lib.XRSLAMTryGetLatestPose(buf, ctypes.byref(ts))
        return rc, list(buf), ts.value

    def get_depth_fusion_stats(self):
        s = ctypes.c_int(0)
        t = ctypes.c_int(0)
        self._lib.XRSLAMGetDepthFusionStats(ctypes.byref(s), ctypes.byref(t))
        return s.value, t.value
