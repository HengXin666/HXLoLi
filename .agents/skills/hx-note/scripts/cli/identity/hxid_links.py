"""本地引用体检: 逐条把正文里的路径真去磁盘上走一遍, 并找孤儿侧车。"""
from __future__ import annotations

import re
import sys
from pathlib import Path

from identity.hxid_core import iter_notes

MD_LINK_RE = re.compile(r'''!?\[[^\]]*\]\(\s*<?([^)\s>]*)\s*(?:"[^"]*"|'[^']*')?\s*>?\s*\)''')

HTML_ATTR_RE = re.compile(r'''(?:src|href)\s*=\s*["']([^"']+)["']''')

SKIP_SCHEMES = ("http://", "https://", "mailto:", "tel:", "data:", "file://", "hxid:", "#", "//")

def _iter_sidecars(docs_dir: Path):
    """笔记目录里除 index.md 之外的本地资源 (ppt 侧车 / 图片 / tag.json / spec.json)。"""
    for p in sorted(docs_dir.rglob("*")):
        if not p.is_file() or p.name.startswith("."):
            continue
        if p.suffix.lower() in (".md", ".mdx"):
            continue
        yield p

def cmd_links(args) -> int:
    docs_dir = Path(args.docs_dir)
    if not docs_dir.is_dir():
        print(f"错误: 目录不存在 {docs_dir}", file=sys.stderr)
        return 1

    # 1. 从 md 里收集本地引用
    broken: list[tuple[str, str, str]] = []          # (笔记, 引用, 类型)
    total: dict[str, int] = {}
    for note in iter_notes(docs_dir):
        for m in MD_LINK_RE.finditer(note.text):
            raw = m.group(1).strip()
            if not raw or raw.startswith(SKIP_SCHEMES):
                continue
            target = raw.split("#")[0].split("?")[0]
            if not target:
                continue
            # markdown 里的空格可能写成 %20; 磁盘上是原样字符
            candidates = {target, target.replace("%20", " ")}
            total["md"] = total.get("md", 0) + 1
            if any((note.path.parent / c).exists() for c in candidates):
                continue
            kind = "图片" if target.lower().endswith((".png", ".jpg", ".jpeg", ".gif", ".webp", ".svg")) else "引用"
            broken.append((str(note.path), raw, kind))
        for m in HTML_ATTR_RE.finditer(note.text):
            raw = m.group(1).strip()
            if not raw or raw.startswith(SKIP_SCHEMES):
                continue
            target = raw.split("#")[0].split("?")[0]
            if not target:
                continue
            total["html"] = total.get("html", 0) + 1
            if (note.path.parent / target.replace("%20", " ")).exists():
                continue
            broken.append((str(note.path), raw, "HTML 属性"))

    # 2. 反向: 磁盘上存在、但没有任何笔记引用的侧车 (孤儿资源)
    referenced: set[Path] = set()
    for note in iter_notes(docs_dir):
        for m in MD_LINK_RE.finditer(note.text):
            raw = m.group(1).strip()
            if not raw or raw.startswith(SKIP_SCHEMES):
                continue
            for c in (raw, raw.replace("%20", " ")):
                referenced.add((note.path.parent / c.split("#")[0].split("?")[0]).resolve())
        for m in HTML_ATTR_RE.finditer(note.text):
            raw = m.group(1).strip()
            if not raw or raw.startswith(SKIP_SCHEMES):
                continue
            referenced.add((note.path.parent / raw.replace("%20", " ")).resolve())
    orphans = [p for p in _iter_sidecars(docs_dir) if p.resolve() not in referenced]

    # 3. 报告
    n_notes = sum(1 for _ in iter_notes(docs_dir))
    print(f"笔记 {n_notes} 篇 | 本地引用 md {total.get('md', 0)} 条, html 属性 {total.get('html', 0)} 条")
    if broken:
        print(f"\n[FAIL] {len(broken)} 条本地引用指向不存在的文件:")
        for path, raw, kind in broken:
            print(f"  ({kind}) {Path(path).relative_to(docs_dir)}")
            print(f"      -> {raw}")
    if orphans and args.show_orphans:
        print(f"\n[WARN] {len(orphans)} 个侧车文件没有被任何笔记引用:")
        for p in orphans:
            print(f"  {p.relative_to(docs_dir)}")
    if not broken:
        print("[OK] 本地引用全部可达")
        return 0
    return 1
