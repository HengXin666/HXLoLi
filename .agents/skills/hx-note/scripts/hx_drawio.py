# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""把一份 JSON 结构图描述编译成 `.drawio.svg` —— 既能在页面上直接显示,
也能在站点的 drawio 编辑器里继续手改。

为什么要脚本而不是让模型手写 SVG:
`.drawio.svg` 的可编辑性来自根元素上那个 `content` 属性 (里面是 HTML 转义后的
mxfile XML), 它必须和肉眼看到的图形逐个对应。手写时这两份东西几乎必然对不上 ——
图看着没问题, 一双击就变成空白画布。这属于"脆弱且必须一致"的低自由度任务,
所以固化成脚本。

用法:
    uv run hx_drawio.py spec.json -o out.drawio.svg
    uv run hx_drawio.py --demo -o demo.drawio.svg     # 自检用

spec 格式 (只支持分层左右布局, 够用且不会画崩):
{
  "title": "可选, 画在左上角",
  "columns": [
    {"title": "列标题", "nodes": [
        {"id": "a", "label": "节点文字", "accent": false}
    ]}
  ],
  "edges": [{"from": "a", "to": "b", "label": "可选"}]
}
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from xml.sax.saxutils import escape, quoteattr

# --- 版式常量 (改这里就能整体调松紧) ---
NODE_W = 190
NODE_H = 58
NODE_GAP = 26
COL_GAP = 96
MARGIN = 36
COL_TITLE_H = 30
TITLE_H = 44
FONT = "12"
TITLE_FONT = "16"
COL_FONT = "11"

# 配色: 低饱和, 深浅两档, 打印和暗色背景下都能看
INK = "#1f2937"
SUB = "#6b7280"
LINE = "#9ca3af"
FILL = "#f4f6fb"
STROKE = "#5b6b9a"
ACCENT_FILL = "#eaf1ff"
ACCENT_STROKE = "#2f57a8"


def char_w(ch: str, size: float) -> float:
    return size if ord(ch) > 0x2E80 else size * 0.56


def wrap(text: str, size: float, max_w: float) -> list[str]:
    lines: list[str] = []
    cur = ""
    cur_w = 0.0
    for ch in text:
        if ch == "\n":
            lines.append(cur)
            cur, cur_w = "", 0.0
            continue
        w = char_w(ch, size)
        if cur and cur_w + w > max_w:
            lines.append(cur)
            cur, cur_w = ch, w
        else:
            cur += ch
            cur_w += w
    if cur:
        lines.append(cur)
    return lines or [""]


class Node:
    def __init__(self, nid: str, label: str, accent: bool, x: int, y: int):
        self.id = nid
        self.label = label
        self.accent = accent
        self.x, self.y = x, y

    @property
    def cx(self) -> int:
        return self.x + NODE_W // 2

    @property
    def cy(self) -> int:
        return self.y + NODE_H // 2


def layout(spec: dict) -> tuple[list[Node], list[dict], int, int, list[tuple[int, int, str]]]:
    cols = spec.get("columns") or []
    if not cols:
        raise SystemExit("error: spec.columns 为空")

    top = MARGIN + (TITLE_H if spec.get("title") else 0)
    tallest = max(len(c.get("nodes") or []) for c in cols)
    body_h = tallest * NODE_H + max(0, tallest - 1) * NODE_GAP

    nodes: list[Node] = []
    col_titles: list[tuple[int, int, str]] = []
    x = MARGIN
    for col in cols:
        cnodes = col.get("nodes") or []
        if col.get("title"):
            col_titles.append((x + NODE_W // 2, top + 14, str(col["title"])))
        # 每列在纵向居中, 少一个节点的列不会顶在上面显得歪
        h = len(cnodes) * NODE_H + max(0, len(cnodes) - 1) * NODE_GAP
        y = top + COL_TITLE_H + (body_h - h) // 2
        for n in cnodes:
            nid = str(n.get("id") or f"n{len(nodes)}")
            nodes.append(Node(nid, str(n.get("label", "")), bool(n.get("accent")), x, y))
            y += NODE_H + NODE_GAP
        x += NODE_W + COL_GAP

    width = x - COL_GAP + MARGIN
    height = top + COL_TITLE_H + body_h + MARGIN
    return nodes, spec.get("edges") or [], width, height, col_titles


def edge_path(a: Node, b: Node) -> str:
    """同列走竖直, 跨列走正交折线。只有这两种情况, 所以不会画出鬼畜走线。"""
    if a.x == b.x:
        # 竖直下行: 从下边缘到上边缘。绕到侧面会和该节点的出边打结。
        if a.y < b.y:
            return f"M {a.cx} {a.y + NODE_H} L {b.cx} {b.y}"
        return f"M {a.cx} {a.y} L {b.cx} {b.y + NODE_H}"
    x1 = a.x + NODE_W
    x2 = b.x
    mid = (x1 + x2) // 2
    if a.cy == b.cy:
        return f"M {x1} {a.cy} L {x2} {b.cy}"
    return f"M {x1} {a.cy} L {mid} {a.cy} L {mid} {b.cy} L {x2} {b.cy}"


def render_svg_body(nodes: list[Node], edges: list[dict],
                    col_titles: list[tuple[int, int, str]], spec: dict) -> str:
    by_id = {n.id: n for n in nodes}
    out: list[str] = []

    if spec.get("title"):
        out.append(
            f'<text x="{MARGIN}" y="{MARGIN + 18}" font-family="Helvetica,Arial,sans-serif" '
            f'font-size="{TITLE_FONT}" font-weight="600" fill="{INK}">'
            f"{escape(str(spec['title']))}</text>"
        )
    for cx, cy, t in col_titles:
        out.append(
            f'<text x="{cx}" y="{cy}" text-anchor="middle" '
            f'font-family="Helvetica,Arial,sans-serif" font-size="{COL_FONT}" '
            f'fill="{SUB}" letter-spacing="0.5">{escape(t)}</text>'
        )

    for e in edges:
        a, b = by_id.get(str(e.get("from"))), by_id.get(str(e.get("to")))
        if not a or not b:
            print(f"warn: 跳过无效边 {e}", file=sys.stderr)
            continue
        out.append(
            f'<path d="{edge_path(a, b)}" fill="none" stroke="{LINE}" '
            f'stroke-width="1.4" marker-end="url(#hxarrow)"/>'
        )
        if e.get("label"):
            if a.x == b.x:  # 竖直边: 标签贴在线的右侧, 否则会飘到画布外
                lx, ly, anchor = a.cx + 8, (a.y + NODE_H + b.y) // 2 + 4, "start"
            elif a.cy == b.cy:  # 同一行: 标签压在水平线上方
                lx, ly, anchor = (a.x + NODE_W + b.x) // 2, a.cy - 8, "middle"
            else:  # 折线: 贴在中间那段竖直线右侧, 否则会压到上一行的节点边框
                lx, ly, anchor = (a.x + NODE_W + b.x) // 2 + 6, (a.cy + b.cy) // 2 + 4, "start"
            out.append(
                f'<text x="{lx}" y="{ly}" text-anchor="{anchor}" '
                f'font-family="Helvetica,Arial,sans-serif" font-size="10" '
                f'fill="{SUB}">{escape(str(e["label"]))}</text>'
            )

    for n in nodes:
        fill = ACCENT_FILL if n.accent else FILL
        stroke = ACCENT_STROKE if n.accent else STROKE
        out.append(
            f'<rect x="{n.x}" y="{n.y}" width="{NODE_W}" height="{NODE_H}" rx="8" ry="8" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="1.3"/>'
        )
        lines = wrap(n.label, float(FONT), NODE_W - 22)[:3]
        start = n.cy - (len(lines) - 1) * 8 + 4
        for i, ln in enumerate(lines):
            out.append(
                f'<text x="{n.cx}" y="{start + i * 16}" text-anchor="middle" '
                f'font-family="Helvetica,Arial,sans-serif" font-size="{FONT}" '
                f'fill="{INK}">{escape(ln)}</text>'
            )
    return "\n".join(out)


def render_mxfile(nodes: list[Node], edges: list[dict], spec: dict) -> str:
    """生成 drawio 可再编辑的 mxGraphModel。几何与上面的 SVG 一一对应。"""
    ids = {n.id: f"hx{i + 2}" for i, n in enumerate(nodes)}
    cells: list[str] = ['<mxCell id="0"/>', '<mxCell id="1" parent="0"/>']
    for n in nodes:
        fill = ACCENT_FILL if n.accent else FILL
        stroke = ACCENT_STROKE if n.accent else STROKE
        style = (f"rounded=1;whiteSpace=wrap;html=1;arcSize=14;fillColor={fill};"
                 f"strokeColor={stroke};fontColor={INK};fontSize=12;align=center;verticalAlign=middle")
        cells.append(
            f'<mxCell id="{ids[n.id]}" value={quoteattr(n.label)} style="{style}" '
            f'vertex="1" parent="1">'
            f'<mxGeometry x="{n.x}" y="{n.y}" width="{NODE_W}" height="{NODE_H}" as="geometry"/>'
            f"</mxCell>"
        )
    for i, e in enumerate(edges):
        src, dst = ids.get(str(e.get("from"))), ids.get(str(e.get("to")))
        if not src or not dst:
            continue
        style = (f"edgeStyle=orthogonalEdgeStyle;rounded=1;html=1;strokeColor={LINE};"
                 f"fontColor={SUB};fontSize=10;endArrow=blockThin;endFill=1")
        cells.append(
            f'<mxCell id="hxe{i}" value={quoteattr(str(e.get("label", "")))} '
            f'style="{style}" edge="1" parent="1" source="{src}" target="{dst}">'
            f'<mxGeometry relative="1" as="geometry"/></mxCell>'
        )
    name = escape(str(spec.get("title") or "HXLoLi"))
    return (
        f'<mxfile host="app.diagrams.net" agent="hx_drawio.py" type="device">'
        f'<diagram id="hx-diagram" name="{name}">'
        f'<mxGraphModel dx="1200" dy="800" grid="0" gridSize="10" guides="1" tooltips="1" '
        f'connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="1169" '
        f'pageHeight="826" math="0" shadow="0">'
        f"<root>{''.join(cells)}</root>"
        f"</mxGraphModel></diagram></mxfile>"
    )


def build(spec: dict) -> str:
    nodes, edges, w, h, col_titles = layout(spec)
    body = render_svg_body(nodes, edges, col_titles, spec)
    content = render_mxfile(nodes, edges, spec)
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
        f'version="1.1" width="{w}" height="{h}" viewBox="0 0 {w} {h}" '
        f"content={quoteattr(content)}>\n"
        '<defs>\n'
        '<marker id="hxarrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
        'markerHeight="7" orient="auto-start-reverse">\n'
        f'<path d="M 0 1 L 9 5 L 0 9 z" fill="{LINE}"/>\n'
        "</marker>\n</defs>\n"
        f'<rect width="{w}" height="{h}" fill="none"/>\n'
        f"{body}\n</svg>\n"
    )


DEMO = {
    "title": "示例: 三段流水线",
    "columns": [
        {"title": "输入", "nodes": [{"id": "a", "label": "素材"}]},
        {"title": "处理", "nodes": [{"id": "b", "label": "解析", "accent": True},
                                    {"id": "c", "label": "校验"}]},
        {"title": "输出", "nodes": [{"id": "d", "label": "产物"}]},
    ],
    "edges": [{"from": "a", "to": "b"}, {"from": "b", "to": "c", "label": "失败重试"},
              {"from": "c", "to": "d"}],
}


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="JSON 结构图 -> 可再编辑的 .drawio.svg")
    p.add_argument("spec", nargs="?", help="spec JSON 路径; 不给则需要 --demo")
    p.add_argument("-o", "--output", required=True, help="输出路径, 必须以 .drawio.svg 结尾")
    p.add_argument("--demo", action="store_true", help="用内置示例 spec")
    args = p.parse_args(argv)

    out = Path(args.output)
    if not out.name.endswith(".drawio.svg"):
        print("error: 输出文件名必须以 .drawio.svg 结尾, 否则站点不会当成可编辑图",
              file=sys.stderr)
        return 2

    if args.demo:
        spec = DEMO
    elif args.spec:
        spec = json.loads(Path(args.spec).read_text(encoding="utf-8"))
    else:
        print("error: 需要 spec 路径或 --demo", file=sys.stderr)
        return 2

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build(spec), encoding="utf-8")
    print(f"created: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
