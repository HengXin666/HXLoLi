"""渲染: SVG 可见层与 drawio 可再编辑的 mxGraphModel。"""
from __future__ import annotations

import sys
from xml.sax.saxutils import escape, quoteattr

from illustration.drawio_layout import Node, edge_path, layout, wrap
from illustration.drawio_theme import (ACCENT_FILL, ACCENT_STROKE, COL_FONT, FILL,
                                       FONT, INK, LINE, MARGIN, NODE_H, NODE_W, STROKE,
                                       SUB, TITLE_FONT)


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
