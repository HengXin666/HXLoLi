#!/usr/bin/env python3
"""最小可跑样例: 生成一份带三层文字与固定安全区的 ASS 特效字幕。

它演示整套做法里最小的一组要素:
  1. 三层文字同时间轴 (日文主行 / 词级假名注音 / 中文对照)
  2. 整行先出现再逐字点亮 (每字事件时长 = 整行时长, 靠 \t 起始时间错峰)
  3. 所有绘制落在一个固定矩形内 (安全区)
  4. 段落级强度包络 (粒子数按段切换, 段内不变)

跑法:  python3 kfx-sample.py > out.ass
验收:  python3 kfx-sample.py > out.ass && 用 ffmpeg 抽帧后跑 kfx-boundary-check.py
"""
import sys, unicodedata

B = chr(92)
NL = chr(10)

# 固定安全区 (1920x1080 画布)。这就是"裁剪退化成常量矩形"的那个矩形。
SAFE = dict(x0=160, y0=640, x1=1760, y1=1020)
SW, SH = SAFE["x1"] - SAFE["x0"], SAFE["y1"] - SAFE["y0"]

# 示例歌词: (起, 止, 日文, 中文, 注音[(词, 读)])
LINES = [
    (1.0, 5.0, "遠ざかる雲の合間", "在渐渐远去的云隙之间", [("遠", "とお"), ("雲", "くも"), ("合間", "あいま")]),
    (5.5, 9.5, "降り注ぐ輝き", "倾泻而下的光芒", [("降", "ふ"), ("注", "そそ"), ("輝", "かがや")]),
]

# 段落级强度包络: 每段一个档位, 段内不变。这里两句各属一段。
DENSITY = [10, 30]


def ts(t):
    return "%d:%02d:%05.2f" % (int(t // 3600), int((t % 3600) // 60), t % 60)


def cw(ch, fs):
    return fs if unicodedata.east_asian_width(ch) in ("W", "F", "A") else fs * 0.5


def main():
    out = []
    out.append("[Script Info]" + NL + "ScriptType: v4.00+" + NL
               + "PlayResX: 1920" + NL + "PlayResY: 1080" + NL
               + "WrapStyle: 2" + NL + "ScaledBorderAndShadow: yes" + NL + NL
               + "[V4+ Styles]" + NL
               + "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding" + NL
               + "Style: JP,Noto Sans CJK JP,60,&H00FFFFFF,&H00000000,&H00201A10,&H00000000,0,0,0,0,100,100,3,0,1,2.5,0,5,0,0,0,1" + NL
               + "Style: FURI,Noto Sans CJK JP,26,&H00FFFFFF,&H00000000,&H00201A10,&H00000000,0,0,0,0,100,100,1,0,1,1.5,0,5,0,0,0,1" + NL
               + "Style: CN,Noto Sans CJK SC,38,&H00F2F6FA,&H00000000,&H00201A10,&H00000000,0,0,0,0,100,100,2,0,1,2,0,5,0,0,0,1" + NL
               + "Style: PTCL,Noto Sans CJK JP,30,&H00FFFFFF,&H00000000,&H0030B000,&H00000000,0,0,0,0,100,100,0,0,1,0.5,0,5,0,0,0,1" + NL + NL
               + "[Events]" + NL
               + "Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text" + NL)

    for li, (t0, t1, jp, cn, furi) in enumerate(LINES):
        chars = list(jp)
        n = len(chars)
        dur = t1 - t0
        uw = SW - 40
        fs = min(60.0, uw / n * 0.96)
        ws = [cw(c, fs) for c in chars]
        span = sum(ws)
        x0 = SAFE["x0"] + 20 + (uw - span) / 2
        y = SAFE["y0"] + SH * 0.66
        cx, acc = [], 0.0
        for i in range(n):
            cx.append(x0 + acc + ws[i] / 2); acc += ws[i]

        # 整行先出现: 每字事件时长 = 整行时长; 逐字点亮靠 t 的起始时间错峰
        for i, ch in enumerate(chars):
            o = int(dur * 0.88 * i / n * 1000)
            out.append("Dialogue: 1," + ts(t0) + "," + ts(t1) + ",JP,,0,0,0,,{" + B + "an5" + B + "pos(%.1f,%.1f)" % (cx[i], y)
                       + B + "alpha&H73&" + B + "bord1" + B + "1c&H0053453A&" + B + "3c&H0018100A&"
                       + B + "t(" + str(o) + "," + str(o + 220) + "," + B + "alpha&H00&" + B + "bord3.5" + B + "fscx130" + B + "fscy130)"
                       + B + "t(" + str(o + 220) + "," + str(o + 440) + "," + B + "fscx100" + B + "fscy100)}" + ch)

        # 词级注音: 与主字共用同一 x 区间
        for w, yo in furi:
            i = jp.index(w)
            lw = len(w)
            lx = cx[i] - ws[i] / 2
            rx = cx[min(i + lw, n) - 1] + ws[min(i + lw, n) - 1] / 2
            out.append("Dialogue: 2," + ts(t0) + "," + ts(t1) + ",FURI,,0,0,0,,{" + B + "an5" + B + "pos(%.1f,%.1f)" % ((lx + rx) / 2, y - 38)
                       + B + "alpha&H73&" + B + "bord1" + B + "1c&H0053453A&" + B + "t(0,220," + B + "alpha&H00&)}" + yo)

        # 粒子: 只在安全区内, 数量取该段的强度档
        W = 1.278 * (cx[-1] - cx[0]) if n > 1 else fs * 2.2
        for i in range(n):
            k = i / (n - 1) if n > 1 else 0.5
            px = 960.0 + (k - 0.5) * W
            for j in range(DENSITY[li]):
                ang = (j * 2.399963) % 3.14159   # 黄金角: 确定性散开, 不用随机
                d = 40 + (j % 7) * 16
                ex = px + (1 if j % 2 else -1) * d * 0.7
                ey = y - 42 - d * 0.6
                out.append("Dialogue: 0," + ts(t0 + 0.02 * (j % 5)) + "," + ts(t0 + 0.02 * (j % 5) + 0.6)
                           + ",PTCL,,0,0,0,,{" + B + "an5" + B + "move(%.1f,%.1f,%.1f,%.1f)" % (px, y - 42, ex, ey)
                           + B + "p1" + B + "bord0.5" + B + "blur%.1f" % (1.5 + 0.35 * (j % 10))
                           + B + "fscx78" + B + "fscy78" + B + "1c&H00E3C87E&" + B + "1a&H10&}m 0 -5 l 3 0 l 0 5 l -3 0")

        out.append("Dialogue: 0," + ts(t0) + "," + ts(t1) + ",CN,,0,0,0,,{" + B + "an5" + B + "pos(960,%.1f)" % (y + 54)
                   + B + "fad(200,400)}" + cn)

    sys.stdout.write(NL.join(out) + NL)


if __name__ == "__main__":
    main()