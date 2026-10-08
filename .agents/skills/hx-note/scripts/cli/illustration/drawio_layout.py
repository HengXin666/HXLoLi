"""文字测量、折行与分层左右布局。"""
from __future__ import annotations

from illustration.drawio_theme import (COL_GAP, COL_TITLE_H, MARGIN, NODE_GAP,
                                       NODE_H, NODE_W, TITLE_H)


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
