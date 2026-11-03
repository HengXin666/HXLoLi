"""A2 的出图脚本. 两类图分开出:

1. 数据图 (cow-bars.png): matplotlib, 数字直接读同目录 measure.txt (fork_cow.py 的输出)
   python3 make_figures.py chart [--font /path/to/NotoSansSC-Regular.otf]
2. 概念图 (cow-hero.png / refcount-touch.png / gc-freeze.png): 生图后端出的位图.
   这里只存提示词, 换后端时照抄: python3 make_figures.py prompts
   本次后端: Hark 会话内的 generate_image 工具. 出图后用 ImageMagick 缩到 1200 宽, 量化到 192 色:
   magick in.png -resize 1200x -strip -colors 192 -define png:compression-level=9 out.png
"""
import argparse
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent

STYLE = (
    "Flat editorial vector illustration, 16:9, muted palette (warm cream background, "
    "slate blue, coral accent), clean geometric shapes, subtle grain, modern tech-blog style. "
)
NO_TEXT = " No text, no letters, no numbers, no watermark."

PROMPTS = {
    "cow-hero.png": STYLE
    + "A shared shelf of identical memory pages between a parent robot (left) and its forked "
    "child (right), joined by a dotted line. The child only reaches out to READ one card, yet "
    "that card splits into a coral photocopy that floats to the child's private stack." + NO_TEXT,
    "refcount-touch.png": STYLE
    + "A row of rounded object boxes on a long memory-page strip, each with a tiny tally dial on "
    "its head. A hand picks one up only to look; its dial clicks and glows coral, the page under "
    "it lights up and a duplicate sheet peels away." + NO_TEXT,
    "gc-freeze.png": STYLE
    + "Two halves. Left: object boxes sealed in a frosted glass vault, a janitor robot with a "
    "broom (the GC) cannot sweep inside. Right: same vault, a hand reaches through a slot and "
    "picks up a box whose dial still clicks coral." + NO_TEXT,
}

LABELS = {
    ("float", "nothing", "-"): "float\n子进程什么也不做",
    ("float", "iterate", "-"): "float\n遍历一遍",
    ("list", "iterate", "-"): "list\n遍历一遍",
    ("list", "gc", "-"): "list\ngc.collect()",
    ("list", "gc", "freeze"): "list + gc.freeze()\ngc.collect()",
    ("list", "iterate+gc", "freeze"): "list + gc.freeze()\n遍历 + gc.collect()",
}


def read_measure():
    text = (HERE / "measure.txt").read_text()
    head = text.splitlines()[0]
    rows = []
    for m in re.finditer(r"^(\w+)\s+(\S+)\s+(\S+)\s+child_dirty=\s*([\d.]+) MB", text, re.M):
        rows.append(((m[1], m[2], m[3]), float(m[4])))
    return head, rows


def chart(font: str | None) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    if font:
        font_manager.fontManager.addfont(font)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=font).get_name()
    head, rows = read_measure()
    labels = [LABELS[k] for k, _ in rows]
    vals = [v for _, v in rows]
    cream, slate, coral = "#F6EBD3", "#5B6E8F", "#EE7B62"
    colors = [coral if v > 10 else slate for v in vals]

    fig, ax = plt.subplots(figsize=(11, 5.4), dpi=150)
    fig.patch.set_facecolor(cream)
    ax.set_facecolor(cream)
    bars = ax.barh(range(len(vals)), vals, color=colors, height=0.6)
    ax.set_yticks(range(len(vals)), labels, fontsize=11)
    ax.invert_yaxis()
    for b, v in zip(bars, vals):
        ax.text(b.get_width() + 1, b.get_y() + b.get_height() / 2, f"{v:.1f} MB",
                va="center", fontsize=11, color="#2B3A55")
    ax.set_xlim(0, max(vals) * 1.18)
    ax.set_xlabel("子进程新增的 Private_Dirty (100 万个对象, 父进程一页没改)", fontsize=11)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.set_title("fork 之后子进程只读, 也会弄脏几十 MB", fontsize=15, loc="left", pad=12)
    fig.text(0.01, 0.01, head, fontsize=8, color="#7A7A7A")
    fig.tight_layout()
    fig.savefig(HERE / "cow-bars.png", facecolor=cream)
    print("wrote", HERE / "cow-bars.png")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["chart", "prompts"])
    ap.add_argument("--font", help="CJK 字体文件; 不给时中文会变成方框")
    a = ap.parse_args()
    if a.what == "chart":
        chart(a.font)
    else:
        for k, v in PROMPTS.items():
            print(f"## {k}\n{v}\n")
