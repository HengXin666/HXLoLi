#!/usr/bin/env python3
"""逐帧测内容包围盒, 判据是四个方向的越界量都是 0。

这是"归一化安全区"的验收脚本。用法:
    python3 kfx-boundary-check.py --frames <帧目录> --safe x0,y0,x1,y1 --bg R,G,B

退出码 0 = 零越界; 1 = 有越界。

为什么要逐帧渲染而不是直接读 ASS: 字号自适应、字形宽度估算与字体的真实 advance
三者无法精确一致, 逐帧实测是唯一可靠的收敛手段 (这就是"两遍法"的由来)。
"""
import argparse, os, sys
import numpy as np
from PIL import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True, help="帧目录 (png/jpg)")
    ap.add_argument("--safe", required=True, help="安全区 x0,y0,x1,y1")
    ap.add_argument("--bg", default="0,0,0", help="背景色 R,G,B")
    ap.add_argument("--thr", type=int, default=12, help="与背景的通道差阈值")
    a = ap.parse_args()

    x0, y0, x1, y1 = [int(v) for v in a.safe.split(",")]
    bg = np.array([int(v) for v in a.bg.split(",")])

    exts = (".png", ".jpg", ".jpeg")
    files = sorted(f for f in os.listdir(a.frames) if f.lower().endswith(exts))
    if not files:
        print("没有帧"); return 2

    U = [10**9, 10**9, -10**9, -10**9]
    over = []
    for f in files:
        im = np.asarray(Image.open(os.path.join(a.frames, f)).convert("RGB")).astype(np.int16)
        diff = np.abs(im - bg).sum(2)
        m = diff > a.thr
        if not m.any():
            continue
        ys, xs = np.where(m)
        bx0, bx1, by0, by1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
        U = [min(U[0], bx0), min(U[1], by0), max(U[2], bx1), max(U[3], by1)]
        o = max(0, x0 - bx0) + max(0, bx1 - x1) + max(0, y0 - by0) + max(0, by1 - y1)
        if o:
            over.append((f, bx0, by0, bx1, by1, o))

    print("帧数 %d" % len(files))
    print("安全区 x[%d,%d] y[%d,%d]" % (x0, x1, y0, y1))
    if U[0] == 10**9:
        print("所有帧都是空的"); return 0
    print("内容并集 x[%d,%d] y[%d,%d]" % (U[0], U[2], U[1], U[3]))
    print("越界帧数 %d" % len(over))
    for f, bx0, by0, bx1, by1, o in over[:8]:
        print("  %s  框 x[%d,%d] y[%d,%d]  越界 %d px" % (f, bx0, bx1, by0, by1, o))
    print()
    if over:
        print("FAIL  四个方向的越界量必须都是 0")
        return 1
    print("PASS  四个方向的越界量都是 0")
    return 0


if __name__ == "__main__":
    sys.exit(main())