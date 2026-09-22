"""Synthetic visual-inertial sequence with analytic ground truth.

We have no real capture yet, so the regression harness has to be able to
manufacture its own input. This module produces, from a closed-form trajectory:

  * per-IMU-sample gyroscope + accelerometer (specific force, i.e. gravity
    included) expressed in the body frame,
  * per-camera-frame 8-bit grayscale images containing projected Gaussian
    blobs of a fixed 3D landmark cloud,
  * the ground-truth body pose at every camera timestamp.

Conventions (kept deliberately explicit, they are the thing most likely to be
silently wrong):

  * World frame is z-up; gravity vector g_w = (0, 0, -9.81).
  * R_wb maps body -> world.  Accelerometer measures specific force
    f_b = R_wb^T * (a_w - g_w); at rest that is R_wb^T * (0, 0, +9.81).
  * Gyroscope measures body rates omega_b with  d/dt R_wb = R_wb * hat(omega_b).
  * Camera frame equals body frame (q_bc = identity, p_bc = 0) so the optical
    axis is body +z.  The base orientation points that axis along world +x.
  * Pinhole projection, no distortion (the generated sensor yaml sets
    camera_distortion_flag: 0), so nothing has to undistort anything.

`selftest.py` re-integrates the emitted IMU stream and checks that it
reproduces the analytic trajectory; that check is what makes the conventions
above verifiable rather than merely asserted.
"""

import math
import random

import geom

GRAVITY = 9.81


class SynthParams(object):
    def __init__(self, **kw):
        self.width = 752
        self.height = 480
        self.fx = 460.0
        self.fy = 460.0
        self.cx = 376.0
        self.cy = 240.0
        self.cam_rate = 20.0
        self.imu_rate = 200.0
        self.duration = 12.0
        self.t0 = 100.0  # non-zero epoch: catches "timestamp assumed to start at 0"
        self.num_landmarks = 400
        self.seed = 20260823
        self.blob_sigma = 1.25
        self.blob_radius = 3
        self.blob_peak = 240
        self.bg_low = 40
        self.bg_high = 78
        self.imu_seed = 12345
        self.gyro_noise = 0.0
        self.accel_noise = 0.0
        self.accel_bias = (0.0, 0.0, 0.0)
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.imu_scale = 1.0  # fault-injection knob used by the negative controls
        for k, v in kw.items():
            if not hasattr(self, k):
                raise AttributeError("unknown SynthParams field %r" % k)
            setattr(self, k, v)

    def as_meta(self):
        keys = sorted(k for k in vars(self))
        return {k: getattr(self, k) for k in keys}


# base orientation: body x -> world -y, body y -> world -z, body z -> world +x
_R_BASE = [
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
]


def traj_position(t):
    """Analytic body position in world (metres)."""
    return [
        0.05 * math.sin(2.0 * math.pi * 0.11 * t),
        0.60 * math.sin(2.0 * math.pi * 0.20 * t),
        0.25 * math.sin(2.0 * math.pi * 0.13 * t + 0.9),
    ]


def traj_velocity(t):
    return [
        0.05 * (2.0 * math.pi * 0.11) * math.cos(2.0 * math.pi * 0.11 * t),
        0.60 * (2.0 * math.pi * 0.20) * math.cos(2.0 * math.pi * 0.20 * t),
        0.25 * (2.0 * math.pi * 0.13) * math.cos(2.0 * math.pi * 0.13 * t + 0.9),
    ]


def traj_acceleration(t):
    return [
        -0.05 * (2.0 * math.pi * 0.11) ** 2 * math.sin(2.0 * math.pi * 0.11 * t),
        -0.60 * (2.0 * math.pi * 0.20) ** 2 * math.sin(2.0 * math.pi * 0.20 * t),
        -0.25
        * (2.0 * math.pi * 0.13) ** 2
        * math.sin(2.0 * math.pi * 0.13 * t + 0.9),
    ]


def traj_rotation(t):
    """Analytic R_wb (body -> world)."""
    yaw = 0.18 * math.sin(2.0 * math.pi * 0.17 * t)
    pitch = 0.09 * math.sin(2.0 * math.pi * 0.23 * t + 0.7)
    roll = 0.07 * math.sin(2.0 * math.pi * 0.29 * t + 1.3)
    R = geom.mmul(geom.rot_z(yaw), geom.mmul(geom.rot_y(pitch), geom.rot_x(roll)))
    return geom.mmul(R, _R_BASE)


def traj_omega_body(t, h=1e-6):
    """Body angular rate, from a central difference of R_wb."""
    Rp = traj_rotation(t + h)
    Rm = traj_rotation(t - h)
    dR = geom.mscale(geom.msub(Rp, Rm), 1.0 / (2.0 * h))
    R = traj_rotation(t)
    return geom.skew_to_vec(geom.mmul(geom.mT(R), dR))


def imu_sample(t, p):
    """(gyro_b, accel_b) as a real IMU would report them at time t."""
    R = traj_rotation(t)
    omega = traj_omega_body(t)
    a_w = traj_acceleration(t)
    f_w = [a_w[0], a_w[1], a_w[2] + GRAVITY]  # a_w - g_w, with g_w = (0,0,-g)
    f_b = geom.mvec(geom.mT(R), f_w)
    gyro = [omega[i] * p.imu_scale + p.gyro_bias[i] for i in range(3)]
    acc = [f_b[i] * p.imu_scale + p.accel_bias[i] for i in range(3)]
    return gyro, acc


def make_landmarks(p):
    rng = random.Random(p.seed)
    pts = []
    while len(pts) < p.num_landmarks:
        x = rng.uniform(2.0, 9.0)
        y = rng.uniform(-3.2, 3.2)
        z = rng.uniform(-2.4, 2.4)
        pts.append([x, y, z])
    return pts


def project(p, R_wc, p_wc, X):
    d = geom.vsub(X, p_wc)
    Xc = geom.mvec(geom.mT(R_wc), d)
    if Xc[2] < 0.5:
        return None
    u = p.fx * Xc[0] / Xc[2] + p.cx
    v = p.fy * Xc[1] / Xc[2] + p.cy
    return u, v, Xc[2]


class Renderer(object):
    """Renders the landmark cloud as anti-aliased Gaussian blobs."""

    def __init__(self, p):
        self.p = p
        row = bytearray(p.width)
        span = p.bg_high - p.bg_low
        for x in range(p.width):
            row[x] = p.bg_low + (span * x) // max(1, p.width - 1)
        self._bg = bytes(row) * p.height
        self._peak_delta = p.blob_peak - p.bg_high

    def render(self, R_wc, p_wc, landmarks):
        p = self.p
        buf = bytearray(self._bg)
        W, H, r = p.width, p.height, p.blob_radius
        inv2s2 = 1.0 / (2.0 * p.blob_sigma * p.blob_sigma)
        peak = float(p.blob_peak)
        n_visible = 0
        for X in landmarks:
            pr = project(p, R_wc, p_wc, X)
            if pr is None:
                continue
            u, v, _z = pr
            if u < r + 1 or v < r + 1 or u >= W - r - 1 or v >= H - r - 1:
                continue
            n_visible += 1
            cx, cy = int(u), int(v)
            fu, fv = u - cx, v - cy
            for dy in range(-r, r + 1):
                yy = cy + dy
                base = yy * W
                ddy = dy - fv
                for dx in range(-r, r + 1):
                    ddx = dx - fu
                    g = math.exp(-(ddx * ddx + ddy * ddy) * inv2s2)
                    if g < 0.02:
                        continue
                    idx = base + cx + dx
                    val = int(buf[idx] + (peak - buf[idx]) * g)
                    if val > buf[idx]:
                        buf[idx] = 255 if val > 255 else val
        return buf, n_visible


def sequence(p):
    """Yield sensor events in timestamp order.

    Each event is one of:
        ("imu",   t, gyro3, acc3)
        ("image", t, width, height, bytearray, n_visible, R_wb, p_wb)
    """
    landmarks = make_landmarks(p)
    renderer = Renderer(p)
    n_imu = int(round(p.duration * p.imu_rate))
    n_cam = int(round(p.duration * p.cam_rate))
    events = []
    for i in range(n_imu):
        events.append((p.t0 + i / p.imu_rate, 0, i))
    for i in range(n_cam):
        # camera samples deliberately land between IMU samples
        events.append((p.t0 + (i + 0.5) / p.cam_rate, 1, i))
    events.sort()
    rng = random.Random(p.imu_seed)
    for t, kind, _i in events:
        if kind == 0:
            gyro, acc = imu_sample(t, p)
            if p.gyro_noise > 0.0:
                gyro = [g + rng.gauss(0.0, p.gyro_noise) for g in gyro]
            if p.accel_noise > 0.0:
                acc = [a + rng.gauss(0.0, p.accel_noise) for a in acc]
            yield ("imu", t, gyro, acc)
        else:
            R_wb = traj_rotation(t)
            p_wb = traj_position(t)
            buf, nvis = renderer.render(R_wb, p_wb, landmarks)
            yield ("image", t, p.width, p.height, buf, nvis, R_wb, p_wb)


# --------------------------------------------------------------- configs ---

SLAM_YAML = """%YAML:1.0
output:
  q_bo: [ 0.0, 0.0, 0.0, 1.0 ]
  p_bo: [ 0.0, 0.0, 0.0 ]

sliding_window:
  size: 10
  subframe_size: 3
  force_keyframe_landmarks: 35

feature_tracker:
  min_keypoint_distance: 20.0
  max_keypoint_detection: 200
  max_init_frames: 60
  max_frames: 20
  predict_keypoints: true
  clahe_clip_limit: 6.0
  clahe_width: 8
  clahe_height: 8

initializer:
  keyframe_num: 8
  keyframe_gap: 5
  min_matches: 50
  min_parallax: 10.0
  min_triangulation: 20
  min_landmarks: 30
  refine_imu: true

solver:
  iteration_limit: 30
  time_limit: 1.0e6

rotation:
  misalignment_threshold: 0.02
  ransac_threshold: 10

parsac:
  parsac_flag: false
  dynamic_probability: 0.15
  threshold: 1.0
  norm_scale: 1.0
  keyframe_check_size: 1
"""


def sensor_yaml(p):
    return """%%YAML:1.0
imu:
  gyroscope_noise_density: 0.01
  gyroscope_random_walk: 0.0001
  accelerometer_noise_density: 0.1
  accelerometer_random_walk: 0.001
  accelerometer_bias: [0.0, 0.0, 0.0]
  gyroscope_bias: [0.0, 0.0, 0.0]
  extrinsic:
    q_bi: [ 0.0, 0.0, 0.0, 1.0 ]
    p_bi: [ 0.0, 0.0, 0.0 ]
  noise:
    cov_g: [ 2.8791302399999997e-08, 0.0, 0.0,
             0.0, 2.8791302399999997e-08, 0.0,
             0.0, 0.0, 2.8791302399999997e-08]
    cov_a: [ 4.0e-6, 0.0, 0.0,
             0.0, 4.0e-6, 0.0,
             0.0, 0.0, 4.0e-6]
    cov_bg: [ 3.7608844899999997e-10, 0.0, 0.0,
              0.0, 3.7608844899999997e-10, 0.0,
              0.0, 0.0, 3.7608844899999997e-10]
    cov_ba: [ 9.0e-6, 0.0, 0.0,
              0.0, 9.0e-6, 0.0,
              0.0, 0.0, 9.0e-6]

cam0:
  resolution: [%d, %d]
  camera_model: pinhole
  distortion_model: radtan
  intrinsics: [%.6f, %.6f, %.6f, %.6f]
  camera_distortion_flag: 0
  distortion: [0.0, 0.0, 0.0, 0.0]
  camera_readout_time: 0.0
  time_offset: 0.0
  extrinsic:
    q_bc: [ 0.0, 0.0, 0.0, 1.0 ]
    p_bc: [ 0.0, 0.0, 0.0 ]
  noise: [ 0.5, 0.0,
           0.0, 0.5 ]
""" % (
        p.width,
        p.height,
        p.fx,
        p.fy,
        p.cx,
        p.cy,
    )
