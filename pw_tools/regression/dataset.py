"""Sequence sources for the regression harness.

Two kinds:

  synth://...     analytically generated, ships its own ground truth, needs
                  nothing on disk.  This is what makes the harness runnable
                  today, with no captured data.

  euroc://<dir>   on-disk EuRoC layout (<dir>/cam0/data.csv, <dir>/cam0/data/*.png,
  tum://<dir>     <dir>/imu0/data.csv).  Timestamps are nanoseconds, IMU column
                  order is t,wx,wy,wz,ax,ay,az -- matching
                  xrslam-pc/player/src/IO/{euroc,tum}_dataset_reader.h exactly.

Every source yields the same event stream as `synth.sequence`:
    ("imu",   t, gyro3, acc3)
    ("image", t, w, h, bytearray, n_visible_or_-1, R_wb_or_None, p_wb_or_None)

Undistortion: the upstream player undistorts inside the *reader* (cv::undistort
for EuRoC, a fisheye remap for TUM); the SLAM core never sees distortion
coefficients.  We reproduce that only for the radtan model and only with
nearest-neighbour resampling -- see `undistort` below and README.md.
"""

import math
import os

import pngio
import synth


def parse_uri(uri):
    for scheme in ("synth://", "euroc://", "tum://"):
        if uri.startswith(scheme):
            return scheme[:-3], uri[len(scheme) :]
    raise ValueError(
        "unrecognised sequence URI %r (expected synth:// | euroc:// | tum://)" % uri
    )


# ------------------------------------------------------------------- synth ---
class SynthSource(object):
    def __init__(self, params=None):
        self.params = params or synth.SynthParams()
        self.kind = "synth"
        self.width = self.params.width
        self.height = self.params.height

    def meta(self):
        m = {"source": "synth"}
        for k, v in self.params.as_meta().items():
            m["synth_" + k] = v
        return m

    def events(self):
        return synth.sequence(self.params)

    def write_configs(self, outdir):
        os.makedirs(outdir, exist_ok=True)
        slam = os.path.join(outdir, "slam.yaml")
        sensor = os.path.join(outdir, "sensor.yaml")
        with open(slam, "w") as fh:
            fh.write(synth.SLAM_YAML)
        with open(sensor, "w") as fh:
            fh.write(synth.sensor_yaml(self.params))
        return slam, sensor


# ------------------------------------------------------------------- euroc ---
def _read_csv_rows(path):
    rows = []
    if not os.path.isfile(path):
        raise IOError("missing %s" % path)
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append([c.strip() for c in line.split(",")])
    return rows


def build_undistort_map(width, height, K, D):
    """Nearest-neighbour inverse map for the radtan (plumb-bob) model.

    Returns a list of source indices, one per destination pixel, or None when
    the distortion is identically zero.
    """
    if all(abs(c) < 1e-12 for c in D):
        return None
    fx, fy, cx, cy = K
    k1, k2, p1, p2 = D
    idx = [0] * (width * height)
    for v in range(height):
        y = (v - cy) / fy
        for u in range(width):
            x = (u - cx) / fx
            r2 = x * x + y * y
            radial = 1.0 + k1 * r2 + k2 * r2 * r2
            xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
            yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
            su = int(round(fx * xd + cx))
            sv = int(round(fy * yd + cy))
            if su < 0 or sv < 0 or su >= width or sv >= height:
                idx[v * width + u] = -1
            else:
                idx[v * width + u] = sv * width + su
    return idx


class EurocSource(object):
    """EuRoC / TUM-VI on-disk layout (identical directory structure)."""

    def __init__(self, root, kind="euroc", undistort_map=None):
        self.root = os.path.abspath(root)
        self.kind = kind
        self._map = undistort_map
        cam_csv = os.path.join(self.root, "cam0", "data.csv")
        imu_csv = os.path.join(self.root, "imu0", "data.csv")
        self.images = []
        for r in _read_csv_rows(cam_csv):
            if len(r) < 2:
                continue
            self.images.append((float(r[0]) * 1e-9, r[1]))
        self.imu = []
        for r in _read_csv_rows(imu_csv):
            if len(r) < 7:
                continue
            self.imu.append(
                (
                    float(r[0]) * 1e-9,
                    [float(r[1]), float(r[2]), float(r[3])],
                    [float(r[4]), float(r[5]), float(r[6])],
                )
            )
        self.images.sort(key=lambda a: a[0])
        self.imu.sort(key=lambda a: a[0])
        self.width = self.height = 0

    def meta(self):
        return {
            "source": self.kind,
            "root": self.root,
            "n_images": len(self.images),
            "n_imu": len(self.imu),
            "undistort": "nearest" if self._map else "none",
        }

    def events(self):
        merged = []
        for t, _g, _a in self.imu:
            merged.append((t, 0))
        for t, _f in self.images:
            merged.append((t, 1))
        merged.sort(key=lambda a: (a[0], a[1]))
        ii = 0
        ci = 0
        for _t, kind in merged:
            if kind == 0:
                t, g, a = self.imu[ii]
                ii += 1
                yield ("imu", t, g, a)
            else:
                t, name = self.images[ci]
                ci += 1
                path = os.path.join(self.root, "cam0", "data", name)
                with open(path, "rb") as fh:
                    w, h, data = pngio.decode_gray8(fh.read())
                self.width, self.height = w, h
                buf = bytearray(data)
                if self._map is not None and len(self._map) == w * h:
                    src = buf
                    out = bytearray(w * h)
                    for i, s in enumerate(self._map):
                        if s >= 0:
                            out[i] = src[s]
                    buf = out
                yield ("image", t, w, h, buf, -1, None, None)


def open_source(uri, params=None, undistort_K=None, undistort_D=None,
                undistort_size=None):
    scheme, rest = parse_uri(uri)
    if scheme == "synth":
        return SynthSource(params)
    umap = None
    if undistort_D is not None and undistort_K is not None and undistort_size:
        umap = build_undistort_map(
            undistort_size[0], undistort_size[1], undistort_K, undistort_D
        )
    return EurocSource(rest, kind=scheme, undistort_map=umap)


# ------------------------------------------------- synth -> EuRoC on disk ---
def export_euroc(params, outdir):
    """Materialise a synthetic sequence in EuRoC layout.

    This is what lets us exercise the real-dataset code path with no real
    dataset: generate -> write -> read back through EurocSource.
    """
    cam_dir = os.path.join(outdir, "cam0", "data")
    imu_dir = os.path.join(outdir, "imu0")
    os.makedirs(cam_dir, exist_ok=True)
    os.makedirs(imu_dir, exist_ok=True)
    cam_rows = []
    imu_rows = []
    truth = []
    for ev in synth.sequence(params):
        if ev[0] == "imu":
            _, t, g, a = ev
            imu_rows.append(
                "%d,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e"
                % (int(round(t * 1e9)), g[0], g[1], g[2], a[0], a[1], a[2])
            )
        else:
            _, t, w, h, buf, nvis, R, p = ev
            ns = int(round(t * 1e9))
            name = "%d.png" % ns
            with open(os.path.join(cam_dir, name), "wb") as fh:
                fh.write(pngio.encode_gray8(w, h, bytes(buf)))
            cam_rows.append("%d,%s" % (ns, name))
            truth.append((t, R, p))
    with open(os.path.join(outdir, "cam0", "data.csv"), "w") as fh:
        fh.write("#t[ns],filename[string]\n")
        fh.write("\n".join(cam_rows) + "\n")
    with open(os.path.join(imu_dir, "data.csv"), "w") as fh:
        fh.write(
            "#t[ns],w.x[rad/s:double],w.y[rad/s:double],w.z[rad/s:double],"
            "a.x[m/s^2:double],a.y[m/s^2:double],a.z[m/s^2:double]\n"
        )
        fh.write("\n".join(imu_rows) + "\n")
    with open(os.path.join(outdir, "slam.yaml"), "w") as fh:
        fh.write(synth.SLAM_YAML)
    with open(os.path.join(outdir, "sensor.yaml"), "w") as fh:
        fh.write(synth.sensor_yaml(params))
    with open(os.path.join(outdir, "truth.txt"), "w") as fh:
        fh.write("# t qx qy qz qw tx ty tz  (body pose in world, ground truth)\n")
        import geom

        for t, R, p in truth:
            q = geom.quat_from_mat(R)
            fh.write(
                "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n"
                % (t, q[0], q[1], q[2], q[3], p[0], p[1], p[2])
            )
    return outdir


def load_truth(path):
    out = []
    with open(path) as fh:
        for line in fh:
            if line.startswith("#") or not line.strip():
                continue
            f = [float(x) for x in line.split()]
            out.append((f[0], f[1:5], f[5:8]))
    return out


def _unused_math_guard():  # keeps `math` imported for future distortion models
    return math.pi
