"""B3 出图: 三张图全部从 b3_data.json (measure_b3.py 的输出) 画, 不手抄数字.

python3 make_b3.py [--font /path/to/NotoSansSC-Regular.otf]
"""
import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle

HERE = Path(__file__).resolve().parent
D = json.loads((HERE / "b3_data.json").read_text())
PAGE = D["layout"]["page_size"]
SLOT = {k: D["stride"][k]["mode_bytes"] for k in ("float", "list")}

BG, INK, MUTE = "#FBF7EF", "#22304A", "#8A8F99"
REF, GCH, IDLE, PAD = "#E4573D", "#E9A23B", "#C9D3E3", "#EDEDED"


def setup(font):
    if font:
        font_manager.fontManager.addfont(font)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=font).get_name()
    plt.rcParams.update({"axes.edgecolor": INK, "axes.labelcolor": INK,
                         "xtick.color": INK, "ytick.color": INK, "text.color": INK})


def fig_layout():
    """图 1: 两种对象的槽位, 按字节等比例画, 标出遍历写哪 8 字节, gc 写哪 16 字节."""
    rows = [
        ("float  (槽位 32 B, 一页 4096 B 放 128 个)", [
            ("ob_refcnt", 8, REF), ("ob_type", 8, IDLE), ("ob_fval", 8, IDLE), ("空", 8, PAD)]),
        ("[i] 列表  (槽位 64 B, 一页放 64 个)", [
            ("PyGC_Head", 16, GCH), ("ob_refcnt", 8, REF), ("ob_type", 8, IDLE),
            ("ob_size", 8, IDLE), ("ob_item", 8, IDLE), ("allocated", 8, IDLE), ("空", 8, PAD)]),
    ]
    fig, ax = plt.subplots(figsize=(12, 4.6), dpi=150)
    fig.patch.set_facecolor(BG); ax.set_facecolor(BG)
    unit = 0.17  # 每字节的宽度
    for r, (title, fields) in enumerate(rows):
        y = 1.6 - r * 1.55
        ax.text(0, y + 0.62, title, fontsize=12.5, weight="bold", va="bottom")
        x = 0
        for name, size, color in fields:
            ax.add_patch(Rectangle((x, y), size * unit, 0.5, facecolor=color, edgecolor=INK, lw=1))
            ax.text(x + size * unit / 2, y + 0.25, name, ha="center", va="center", fontsize=10)
            ax.text(x, y - 0.08, f"{x / unit:.0f}", ha="center", va="top", fontsize=8.5, color=MUTE)
            x += size * unit
        ax.text(x, y - 0.08, f"{x / unit:.0f}", ha="center", va="top", fontsize=8.5, color=MUTE)
    ax.add_patch(Rectangle((11.4, 1.95), 0.32, 0.22, facecolor=REF, edgecolor=INK))
    ax.text(11.8, 2.06, "遍历时 +1/-1 (每个对象都写)", va="center", fontsize=10)
    ax.add_patch(Rectangle((11.4, 1.6), 0.32, 0.22, facecolor=GCH, edgecolor=INK))
    ax.text(11.8, 1.71, "全量 gc 写 (gc.freeze 后不写)", va="center", fontsize=10)
    ax.add_patch(Rectangle((11.4, 1.25), 0.32, 0.22, facecolor=IDLE, edgecolor=INK))
    ax.text(11.8, 1.36, "只读, 不写", va="center", fontsize=10)
    lf, ll = D["layout"]["float"], D["layout"]["list"]
    note = (f"核对 (ctypes 从 id(obj) 读): float getsizeof={lf['getsizeof']}, @16 读出 {lf['ob_fval@16']}; "
            f"多一个引用时 @0 从 {lf['ob_refcnt@0 before/after one more ref'][0]} 变 "
            f"{lf['ob_refcnt@0 before/after one more ref'][1]}; list getsizeof={ll['getsizeof']} (含 16 B PyGC_Head), "
            f"@16 ob_size={ll['ob_size@16']}. 槽位 = 相邻对象地址差众数: float {SLOT['float']} B "
            f"({D['stride']['float']['share']:.1%}), list {SLOT['list']} B ({D['stride']['list']['share']:.1%})")
    ax.text(0, -1.25, note, fontsize=8.3, color=MUTE, wrap=True)
    ax.set_xlim(-0.3, 16.5); ax.set_ylim(-1.5, 2.6); ax.axis("off")
    ax.set_title("读一个对象就要写它的 ob_refcnt (8 字节), 写一次就复制它所在的整页 4096 字节", loc="left", fontsize=14, pad=8)
    fig.text(0.01, 0.01, D["env"], fontsize=7.5, color=MUTE)
    fig.savefig(HERE / "b3-1-layout.png", facecolor=BG, bbox_inches="tight")


def fig_step():
    """图 2: 子进程每隔 k 个对象读一个. 每页至少读到一个对象时, 脏页不降."""
    fig, ax = plt.subplots(figsize=(11, 5.2), dpi=150)
    fig.patch.set_facecolor(BG); ax.set_facecolor(BG)
    colors = {"float": REF, "list": "#3B6FB6"}
    for kind in ("float", "list"):
        sw = D["step_sweep"][kind]
        ks = [int(k) for k in sw]
        mb = [sw[str(k)] / 1024 for k in ks]
        per_page = PAGE // SLOT[kind]
        ax.plot(ks, mb, "o-", color=colors[kind], lw=2, ms=6,
                label=f"{kind}: 一页 {per_page} 个对象")
        ax.axvline(per_page, color=colors[kind], ls=":", lw=1.2)
        ax.annotate(f"k = {per_page}\n仍是 {sw[str(per_page)] / 1024:.1f} MB",
                    (per_page, sw[str(per_page)] / 1024), xytext=(-78 if kind == "list" else 10, -34 if kind == "list" else 12),
                    textcoords="offset points", fontsize=9.5, color=colors[kind])
    ax.text(300, 22, "k ≥ 128 时两条线重合:\n每页都只剩 ≤1 个被读的对象,\n脏页 = 读到的对象数 × 4 KB", fontsize=9.5, color=MUTE)
    ax.set_ylim(0, 70)
    ax.set_xscale("log", base=2)
    ax.set_xticks([1, 2, 8, 32, 64, 128, 256, 512, 1024], ["1", "2", "8", "32", "64", "128", "256", "512", "1024"])
    ax.set_xlabel("子进程每隔 k 个对象读一个 (100 万个对象)")
    ax.set_ylabel("子进程新增 Private_Dirty (MB)")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.legend(frameon=False, loc="upper right")
    ax.set_title("只读 1/128 的 float, 照样复制全部页; 跨过一页一个对象才开始减半", loc="left", fontsize=14)
    fig.text(0.01, 0.01, D["env"], fontsize=7.5, color=MUTE)
    fig.tight_layout()
    fig.savefig(HERE / "b3-2-step.png", facecolor=BG)


def fig_n():
    """图 3: 脏页随对象个数线性涨, 斜率就是槽位大小."""
    fig, ax = plt.subplots(figsize=(11, 5.2), dpi=150)
    fig.patch.set_facecolor(BG); ax.set_facecolor(BG)
    colors = {"float": REF, "list": "#3B6FB6"}
    for kind in ("float", "list"):
        sw = D["n_sweep"][kind]
        ns = [int(n) for n in sw]
        mb = [sw[str(n)] / 1024 for n in ns]
        theory = [n * SLOT[kind] / 1024 / 1024 for n in ns]
        ax.plot([n / 1e6 for n in ns], theory, "--", color=colors[kind], lw=1.3,
                label=f"{kind} 理论: N × {SLOT[kind]} B")
        ax.plot([n / 1e6 for n in ns], mb, "o", color=colors[kind], ms=8, label=f"{kind} 实测")
        last = ns[-1]
        ax.annotate(f"{mb[-1]:.1f} MB (理论 {theory[-1]:.1f})", (last / 1e6, mb[-1]),
                    xytext=(-150, 8), textcoords="offset points", fontsize=9.5, color=colors[kind])
    ax.set_xlabel("父进程里的对象个数 N (百万), 子进程全部读一遍")
    ax.set_ylabel("子进程新增 Private_Dirty (MB)")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.legend(frameon=False, loc="upper left")
    ax.set_title("子进程脏页 ≈ N × 每个对象的槽位, 斜率就是 32 B 和 64 B", loc="left", fontsize=14)
    fig.text(0.01, 0.01, D["env"], fontsize=7.5, color=MUTE)
    fig.tight_layout()
    fig.savefig(HERE / "b3-3-n.png", facecolor=BG)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--font", help="CJK 字体文件; 不给时中文会变成方框")
    a = ap.parse_args()
    setup(a.font)
    fig_layout(); fig_step(); fig_n()
    print("wrote b3-1-layout.png b3-2-step.png b3-3-n.png")
