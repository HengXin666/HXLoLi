#!/usr/bin/env python3
"""渲染验收  本笔记判据的可执行件 (含标定后的绝对阈值)。

实测成绩 (4 缺陷 + 4 干扰): 检出 4/4, 误报 0/4。
不带前提条件的版本同组用例误报 2/4。

阈值来源: 标定流程实测得干扰上界 U 与缺陷幅度 D, 阈值取两者之间。
  U (各变动对指标的最大影响):
    像素   字号 74.0% / 字符数 99.9% / 文本 0.2%  / 字体族 21.9%
    长宽比 字号  0.9% / 字符数 101.0% / 文本 14.5% / 字体族  0.6%
    填充率 字号  7.3% / 字符数  0.6% / 文本 13.1% / 字体族 22.4%
    归一化宽 字号 0.1% / 字符数  0.5% / 文本  0.6% / 字体族  0.6%
  D (缺陷幅度): 像素归零 100% / 长宽比 37.9% / 填充率 22.5% / 归一化宽 25.5%

环境: ffmpeg subtitles 滤镜 (libass), 1280x720 单帧, 字体子集化后输出 ttf。
同一条输入的重复渲染已实测为确定性 (5 次逐像素相同, 线程数无关), 故 L1 层阈值为 0。

用法:
    python3 render-check.py --ref ref.png --test test.png \
        --text 用永不止息的光芒 --size 90 --level L1
"""
import argparse, json, sys
import numpy as np
from PIL import Image

BG_THRESHOLD = 30          # 灰度 > 30 记为非背景

# 标定后的阈值 (单位: 相对变化的绝对值)
TOL = {
    # 层: (像素, 长宽比, 填充率, 归一化宽)
    # None 表示该指标在该层禁用
    "L1": (0.0,  0.10,  0.10,  0.10),   # U=0, 任何变化都是缺陷; 但仍留微小容差防浮点噪声
    "L2": (0.99, None,  0.10,  0.10),   # 允许变: 字号, 字符数
    "L3": (0.99, None,  None,  0.10),   # 允许变: 字号, 字符数, 文本, 字体族
    #                                       填充率在 L3 禁用: 其 U=22.4% 与 D=22.5% 只差 0.1pp, 取不出阈值
}


def stats(path, n_chars, size):
    """把一张渲染图压成四个量。定义见笔记 0x05 与 .hx-info.md 的指标定义节。"""
    a = np.asarray(Image.open(path).convert("L")).astype(np.int32)
    m = a > BG_THRESHOLD
    if not m.any():
        return {"px": 0, "ratio": None, "fill": None, "wn": None}
    ys, xs = np.where(m)
    w = int(xs.max() - xs.min() + 1)
    h = int(ys.max() - ys.min() + 1)
    px = int(m.sum())
    return {
        "px": px,
        "ratio": w / h,              # 包围盒长宽比 (墨迹框)
        "fill": px / (w * h),        # 填充率
        "wn": w / (n_chars * size),  # 归一化宽, CJK 全角约 0.68
    }


def check(ref, test, level="L1"):
    """按层取阈值比对。返回告警列表。"""
    tol_px, tol_ratio, tol_fill, tol_wn = TOL[level]
    alerts = []

    if test["px"] == 0:
        alerts.append("完全不显示: 像素归零")
        return alerts

    if tol_wn is not None and ref["wn"] and test["wn"]:
        d = abs(test["wn"] - ref["wn"]) / ref["wn"]
        if d > tol_wn:
            alerts.append("字形异常: 归一化宽偏差 %.1f%%" % (d * 100))

    if tol_ratio is not None and ref["ratio"] and test["ratio"]:
        d = abs(test["ratio"] - ref["ratio"]) / ref["ratio"]
        if d > tol_ratio:
            alerts.append("字形异常: 长宽比偏差 %.1f%%" % (d * 100))

    if tol_fill is not None and ref["fill"] and test["fill"]:
        d = abs(test["fill"] - ref["fill"]) / ref["fill"]
        if d > tol_fill:
            alerts.append("墨量异常: 填充率偏差 %.1f%%" % (d * 100))

    return alerts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True, help="参考渲染 (正确的那次)")
    ap.add_argument("--test", required=True, help="待测渲染")
    ap.add_argument("--text", required=True, help="ASS 里的文本, 用于算归一化宽")
    ap.add_argument("--size", type=int, required=True, help="ASS 的 Fontsize")
    ap.add_argument("--level", default="L1", choices=["L1", "L2", "L3"],
                    help="L1 同一字幕重复渲染 / L2 同曲不同句 / L3 跨曲复用")
    a = ap.parse_args()

    n = len(a.text)
    ref = stats(a.ref, n, a.size)
    test = stats(a.test, n, a.size)
    alerts = check(ref, test, a.level)

    print(json.dumps({
        "level": a.level,
        "ref": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in ref.items()},
        "test": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in test.items()},
        "alerts": alerts,
    }, ensure_ascii=False, indent=2))
    return 1 if alerts else 0


def make_subset(src, chars, out, font_number=0):
    """子集化。输出必须是 ttf  libass/freetype 不读 woff2。"""
    from fontTools import subset
    subset.main([src, "--text=" + chars, "--output-file=" + out,
                 "--layout-features=*", "--no-hinting",
                 "--font-number=%d" % font_number])


if __name__ == "__main__":
    sys.exit(main())
