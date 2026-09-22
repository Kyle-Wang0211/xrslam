#!/usr/bin/env python3
"""EuRoC 真值 → TUM 格式。

EuRoC 的 state_groundtruth_estimate0/data.csv 列序是:
  timestamp[ns], p_RS_R_x,y,z [m], q_RS_w,x,y,z [], v..., b_w..., b_a...
⚠️ 四元数是 **w 在前**,而 TUM 是 **w 在后**(tx ty tz qx qy qz qw)。
   这一处顺序搞错,evo 算出来的旋转误差会完全没有意义 —— 而且不会报错。
"""
import sys, os

def main(seq_dir: str, out_path: str) -> int:
    src = os.path.join(seq_dir, "state_groundtruth_estimate0", "data.csv")
    if not os.path.exists(src):
        print(f"找不到真值: {src}", file=sys.stderr); return 1
    n = 0
    with open(src) as f, open(out_path, "w") as o:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            c = line.split(",")
            if len(c) < 8:
                continue
            t_ns = int(c[0]); px, py, pz = c[1], c[2], c[3]
            qw, qx, qy, qz = c[4], c[5], c[6], c[7]   # ← w 在前
            o.write(f"{t_ns/1e9:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}\n")
            n += 1
    print(f"{out_path}: {n} 行")
    return 0 if n else 1

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: euroc_gt_to_tum.py <seq>/mav0 <out.tum>", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(sys.argv[1], sys.argv[2]))
