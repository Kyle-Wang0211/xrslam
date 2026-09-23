#!/usr/bin/python3
"""Make an induced-visual-loss COPY of an EuRoC-format recording.

  make_gap.py <src_euroc_dir> <dst_dir> <start_s> <dur_s> <black|blur>

start_s is measured from the first camera timestamp.  Every camera frame with
start <= t - t0 < start + dur is replaced; all other frames are symlinks to the
original PNGs, imu0/ is a symlink to the original (IMU untouched), and
cam0/data.csv is copied byte for byte (CRLF kept, see euroc_dataset_reader.h).
  black : a single all-zero PNG of the same size (symlinked per frame)
  blur  : the original frame Gaussian-blurred with radius = 0.05 * width
Prints the replaced interval [first_ns, last_ns] and the frame count.
"""
import os, sys, shutil
from PIL import Image, ImageFilter

src, dst, start_s, dur_s, mode = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
assert mode in ('black', 'blur')
os.makedirs(os.path.join(dst, 'cam0', 'data'))
os.symlink(os.path.abspath(os.path.join(src, 'imu0')), os.path.join(dst, 'imu0'))
shutil.copyfile(os.path.join(src, 'cam0', 'data.csv'), os.path.join(dst, 'cam0', 'data.csv'))

rows = []
with open(os.path.join(src, 'cam0', 'data.csv'), 'rb') as f:
    for ln in f.read().split(b'\r\n')[1:]:
        if not ln.strip():
            continue
        t, name = ln.decode().split(',')
        rows.append((int(t), name))
t0 = rows[0][0]
black = None
replaced = []
for t, name in rows:
    s = os.path.abspath(os.path.join(src, 'cam0', 'data', name))
    d = os.path.join(dst, 'cam0', 'data', name)
    rel = (t - t0) * 1e-9
    if start_s <= rel < start_s + dur_s:
        replaced.append(t)
        if mode == 'black':
            if black is None:
                im = Image.open(s)
                black = os.path.join(dst, 'cam0', '_black.png')
                Image.new(im.mode, im.size, 0).save(black)
            os.symlink(black, d)
        else:
            im = Image.open(s)
            im.filter(ImageFilter.GaussianBlur(radius=0.05 * im.size[0])).save(d)
    else:
        os.symlink(s, d)
print(f'{mode} frames {len(replaced)}  first {replaced[0]}  last {replaced[-1]}  '
      f'({(replaced[0]-t0)*1e-9:.3f}s .. {(replaced[-1]-t0)*1e-9:.3f}s after t0={t0})')
