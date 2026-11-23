"""结构图生成: 节点, 连线, 色带, 输出套用 kb-note.css 类名的 SVG."""
import math


def esc(s):
    return s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


class Fig:
    def __init__(self, w, h, label):
        self.w, self.h, self.label = w, h, label
        self.bg, self.edges, self.nodes, self.labels = [], [], [], []

    def zone(self, x, y, w, h):
        self.bg.append(f'<rect rx="12" ry="12" x="{x}" y="{y}" width="{w}" height="{h}" class="box zone"/>')

    def band(self, x, y, w, h, cls=''):
        self.bg.append(f'<rect rx="6" ry="6" x="{x}" y="{y}" width="{w}" height="{h}" class="band {cls}"/>')

    def ghost(self, x, y, w, h):
        self.bg.append(f'<rect rx="6" ry="6" x="{x}" y="{y}" width="{w}" height="{h}" class="box ghost"/>')

    def box(self, x, y, w, h, text, cls='', text_cls=''):
        r = h / 2 if 'pill' in cls and h <= 44 else 8
        self.nodes.append(
            f'<g class="nd"><rect rx="{r}" ry="{r}" x="{x}" y="{y}" width="{w}" height="{h}" class="box {cls}"/>'
            f'<text x="{x + w / 2}" y="{y + h / 2 + 5}" text-anchor="middle" class="{text_cls}">{esc(text)}</text></g>')

    def text(self, x, y, text, cls='', anchor='start'):
        self.labels.append(f'<text x="{x}" y="{y}" class="{cls}" text-anchor="{anchor}">{esc(text)}</text>')

    def code(self, x, y, src, cell=14):
        xs = ' '.join(str(x + i * cell + cell / 2) for i in range(len(src)))
        self.labels.append(f'<text x="{xs}" y="{y}" text-anchor="middle">{esc(src)}</text>')

    def line(self, pts, cls='', head=True, both=False):
        d = 'M' + ' L'.join(f'{x} {y}' for x, y in pts)
        self.edges.append(f'<path d="{d}" class="ln {cls}"/>')
        tone = 'pk' if 'pk' in cls.split() else 'gd' if 'gd' in cls.split() else ''
        if head:
            self.edges.append(self._head(pts[-1], pts[-2], tone))
        if both:
            self.edges.append(self._head(pts[0], pts[1], tone))

    def _head(self, tip, prev, tone):
        dx, dy = tip[0] - prev[0], tip[1] - prev[1]
        n = math.hypot(dx, dy) or 1
        ux, uy = dx / n, dy / n
        bx, by = tip[0] - ux * 7, tip[1] - uy * 7
        px, py = -uy * 3.2, ux * 3.2
        d = f'M{tip[0]:.1f} {tip[1]:.1f} L{bx + px:.1f} {by + py:.1f} L{bx - px:.1f} {by - py:.1f}Z'
        cls = f'ah {tone}'.strip()
        return f'<path d="{d}" class="{cls}"/>'

    def save(self, path):
        body = ''.join([
            f'<g class="bg">{"".join(self.bg)}</g>', f'<g class="edges">{"".join(self.edges)}</g>',
            f'<g class="nodes">{"".join(self.nodes)}</g>', f'<g class="labels">{"".join(self.labels)}</g>'])
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" role="img" aria-label="{esc(self.label)}" '
               f'viewBox="0 0 {self.w} {self.h}">{body}</svg>')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(svg)
