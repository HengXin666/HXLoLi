#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""ai-docs 全局唯一 ID (hxid) 工具: 分配 / 校验 / 索引 / 相对链接解析.

设计约束 (与 hx-docs-organize/references/migration.md 一致):
  - hxid 写在 frontmatter, 形如 hxid: "hx-3f9a2c71", 创建时分配一次, 之后**永不变更**.
  - 正文跨文章引用统一写成 [标题](hxid:hx-3f9a2c71) —— 目录被移动/改名后,
    跑 resolve 即可把所有 hxid 链接重算成当前正确的相对路径.
  - 同目录内引用仍用普通相对路径 (同目录一起移动, 相对关系天然不变).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import secrets
import sys
from pathlib import Path

DOCS_DIR_DEFAULT = "ai-docs"
SNAPSHOT_NAME = ".hx-id-snapshot.json"
HXID_RE = re.compile(r"^hx-[0-9a-f]{8}$")
FM_RE = re.compile(r"\A---[ \t]*\r?\n(.*?)\r?\n---[ \t]*\r?\n", re.S)
FM_HXID_RE = re.compile(r"^hxid[ \t]*:[ \t]*[\"']?([^\"'\r\n]+?)[\"']?[ \t]*$", re.M)
# 链接的两种形态:
#   bare   -> [标题](hxid:hx-xxxxxxxx)                      (源码里的可移植形态)
#   tagged -> [标题](../002-乙/index.md "hxid:hx-xxxxxxxx")  (resolve 之后的可渲染形态)
# title 里的 hxid 是链接的持久身份, 让 resolve 可以反复重跑.
LINK_RE = re.compile(
    r"\]\(\s*(?:hxid:(?P<bare>hx-[0-9a-f]{8})"
    r"|(?P<path>[^)\s]*)\s+\"hxid:(?P<tagged>hx-[0-9a-f]{8})\")\s*\)"
)


class Note:
    __slots__ = ("path", "text", "fm", "hxid")

    def __init__(self, path: Path, text: str) -> None:
        self.path = path
        self.text = text
        self.fm = FM_RE.match(text)
        self.hxid = ""
        if self.fm:
            m = FM_HXID_RE.search(self.fm.group(1))
            if m:
                self.hxid = m.group(1).strip()

    @property
    def body_start(self) -> int:
        return self.fm.end() if self.fm else 0


DEFAULT_EXEMPT: tuple[str, ...] = ("001-关于",)
EXEMPT_FILE = ".hx-id-ignore"


def load_exempt(docs_dir: Path) -> list[str]:
    """豁免清单: 这些页面的路径片段允许没有 hxid (非笔记页面)。

    优先级: 环境变量 HX_DOCS_ID_EXEMPT (逗号分隔) > <docs-dir>/.hx-id-ignore
    (每行一个路径子串, # 开头为注释) > 内置默认。
    """
    env = os.getenv("HX_DOCS_ID_EXEMPT")
    if env is not None:
        return [item.strip() for item in env.split(",") if item.strip()]
    ignore_file = docs_dir / EXEMPT_FILE
    if ignore_file.is_file():
        items: list[str] = []
        for line in ignore_file.read_text(encoding="utf-8").splitlines():
            line = line.split("#", 1)[0].strip()
            if line:
                items.append(line)
        return items
    return list(DEFAULT_EXEMPT)


def is_exempt(path: Path, docs_dir: Path, exempt: list[str]) -> bool:
    rel = path.relative_to(docs_dir).as_posix()
    return any(item in rel for item in exempt)


def iter_notes(docs_dir: Path) -> list[Note]:
    """所有"可被引用的笔记页面"。

    涵盖两种形态, 与生成侧边栏的口径保持一致:
      - <任何目录>/index.md  (标准形态)
      - <任何目录>/<名字>.md (散装页面, 如 ai-docs/deck-embed-test.md 也能拿到 ID)
    跳过点开头的目录与点开头的文件 (.hx-mitemite.md 答题卡不是笔记)。
    """
    notes: list[Note] = []
    for p in sorted(docs_dir.rglob("*.md")):
        rel_parts = p.relative_to(docs_dir).parts
        if any(part.startswith(".") for part in rel_parts):
            continue
        # 同目录已有 index.md 时, 其它 .md 是这篇笔记的**内容分片**
        # (被 index.md 显式引用), 不是可被跨文章引用的独立笔记, 不给 hxid.
        if p.name != "index.md" and (p.parent / "index.md").is_file():
            continue
        notes.append(Note(p, p.read_text(encoding="utf-8")))
    return notes


def rel_link(from_note: Path, to_note: Path) -> str:
    """from_note/to_note 都是 index.md 的绝对或相对路径, 返回 POSIX 相对链接."""
    rel = os.path.relpath(to_note.parent, from_note.parent)
    return (rel.replace(os.sep, "/") + "/index.md") if rel != "." else "index.md"


def cmd_assign(args: argparse.Namespace) -> int:
    docs_dir = Path(args.docs_dir)
    notes = iter_notes(docs_dir)
    used = {n.hxid for n in notes if n.hxid}
    todo = [n for n in notes if not n.hxid]
    bad = [n for n in notes if n.hxid and not HXID_RE.match(n.hxid)]

    exempt = load_exempt(docs_dir)
    todo = [n for n in todo if not is_exempt(n.path, docs_dir, exempt)]
    if exempt:
        print(f"[skip] 豁免 (无 hxid 不算问题): {', '.join(exempt)}")
    if not todo and not bad:
        print(f"[OK] {len(notes)} 篇笔记均已有合法 hxid, 无需分配")
        return 0

    plan: list[tuple[Note, str]] = []
    for n in todo:
        while True:
            cand = "hx-" + secrets.token_hex(4)
            if cand not in used:
                used.add(cand)
                break
        plan.append((n, cand))

    for note, new_id in plan:
        tag = "(写入)" if args.write else "(dry-run)"
        print(f"  {tag} {new_id}  {note.path}")

    for n in bad:
        print(f"  [ERROR] 非法 hxid {n.hxid!r}: {n.path}")

    if args.write:
        for note, new_id in plan:
            if note.fm:
                fm_text = note.fm.group(1) + "\n"
                head = "---\n" + f'hxid: "{new_id}"\n' + fm_text + "---\n"
                note.path.write_text(head + note.text[note.fm.end():], encoding="utf-8")
            else:
                note.path.write_text(
                    "---\n" + f'hxid: "{new_id}"\n' + "---\n\n" + note.text, encoding="utf-8")
        print(f"[OK] 已为 {len(plan)} 篇笔记写入 hxid")

    return 1 if bad else 0


def build_index(docs_dir: Path) -> tuple[dict[str, Path], list[str]]:
    errors: list[str] = []
    index: dict[str, Path] = {}
    exempt = load_exempt(docs_dir)
    for n in iter_notes(docs_dir):
        if not n.hxid:
            if not is_exempt(n.path, docs_dir, exempt):
                errors.append(f"缺 hxid: {n.path}")
            continue
        if not HXID_RE.match(n.hxid):
            errors.append(f"hxid 格式非法 {n.hxid!r}: {n.path}")
            continue
        if n.hxid in index:
            errors.append(f"hxid 重复 {n.hxid}: {index[n.hxid]} 与 {n.path}")
            continue
        index[n.hxid] = n.path
    return index, errors


def cmd_check(args: argparse.Namespace) -> int:
    docs_dir = Path(args.docs_dir)
    index, errors = build_index(docs_dir)
    notes = iter_notes(docs_dir)
    dangling: list[str] = []
    stale: list[str] = []
    for n in notes:
        for m in LINK_RE.finditer(n.text):
            target_id = m.group("bare") or m.group("tagged")
            target = index.get(target_id)
            if target is None:
                dangling.append(f"{n.path}: 链接指向不存在的 {target_id}")
                continue
            if m.group("path"):
                want = rel_link(n.path, target)
                if m.group("path") != want:
                    stale.append(f"{n.path}: {m.group('path')} 应为 {want} ({target_id})")
    for e in errors + dangling + stale:
        print("[ERROR] " + e)
    if not errors and not dangling and not stale:
        print(f"[OK] {len(index)} 篇笔记的 hxid 唯一且合法, hxid 链接全部可达且路径最新")
        return 0
    return 1


def snapshot_payload(index: dict[str, Path], docs_dir: Path) -> dict[str, str]:
    """快照只记 hxid -> 笔记位置 (相对 docs_dir 的 POSIX 路径)。"""
    return {k: v.relative_to(docs_dir).as_posix() for k, v in sorted(index.items())}


def cmd_snapshot(args: argparse.Namespace) -> int:
    docs_dir = Path(args.docs_dir)
    index, errors = build_index(docs_dir)
    if errors:
        for e in errors:
            print("[ERROR] " + e, file=sys.stderr)
        print("[ABORT] 索引不健康, 不写快照")
        return 1
    payload = snapshot_payload(index, docs_dir)
    path = Path(args.file) if args.file else docs_dir / SNAPSHOT_NAME
    if args.check:
        if not path.is_file():
            print(f"[WARN] 没有快照 {path}; 先跑 snapshot 建立基线")
            return 0
        old = json.loads(path.read_text(encoding="utf-8"))
        moved = [k for k in old if k in payload and old[k] != payload[k]]
        gone = [k for k in old if k not in payload]
        added = [k for k in payload if k not in old]
        for k in moved:
            print(f"[MOVED]   {k}: {old[k]} -> {payload[k]}  (ID 不变, 位置变了)")
        for k in gone:
            print(f"[GONE]    {k}: {old[k]}  (ID 消失: 被删除或 ID 被改写 -> 检查!)")
        for k in added:
            print(f"[NEW]     {k}: {payload[k]}")
        if not (moved or gone or added):
            print(f"[OK] 与快照一致: {len(payload)} 个 hxid 无一变化")
        return 1 if gone else 0
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[OK] 快照已写入 {path} ({len(payload)} 条)")
    return 0

def cmd_index(args: argparse.Namespace) -> int:
    docs_dir = Path(args.docs_dir)
    index, errors = build_index(docs_dir)
    payload = {k: str(v) for k, v in sorted(index.items())}
    if args.json_out:
        Path(args.json_out).write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[OK] 已写出 {len(payload)} 条 hxid 索引 -> {args.json_out}")
    else:
        for k, v in payload.items():
            print(f"{k}  {v}")
    for e in errors:
        print("[ERROR] " + e, file=sys.stderr)
    return 1 if errors else 0


def cmd_resolve(args: argparse.Namespace) -> int:
    docs_dir = Path(args.docs_dir)
    index, errors = build_index(docs_dir)
    for e in errors:
        print("[ERROR] " + e, file=sys.stderr)
    if errors:
        print("[ABORT] hxid 索引不健康, 先跑 check/assign 修好再 resolve")
        return 1

    changed = 0
    for n in iter_notes(docs_dir):
        def sub(m: re.Match[str]) -> str:
            target_id = m.group("bare") or m.group("tagged")
            target = index.get(target_id)
            if target is None:
                return m.group(0)
            want = rel_link(n.path, target)
            return f']({want} "hxid:{target_id}")'

        new_text = LINK_RE.sub(sub, n.text)
        if new_text != n.text:
            changed += 1
            for m in LINK_RE.finditer(n.text):
                target_id = m.group("bare") or m.group("tagged")
                target = index.get(target_id)
                was = m.group("path") or "(bare)"
                arrow = f"-> {rel_link(n.path, target)}" if target else "-> (悬空)"
                print(f"  {n.path}: {target_id}  {was} {arrow}")
            if args.write:
                n.path.write_text(new_text, encoding="utf-8")

    if changed == 0:
        print("[OK] 无 hxid 链接需要重算")
        return 0
    if args.write:
        print(f"[OK] 已重算 {changed} 篇笔记里的 hxid 链接")
    else:
        print(f"[dry-run] {changed} 篇笔记含需要重算的 hxid 链接 (加 --write 落盘)")
        return 2
    return 0


# ── 本地引用体检 ───────────────────────────────────────────────────────────
# check 只管 hxid 链接的**身份**, 不管普通相对路径是否还在.
# 目录搬家后 "/005-旧名/index.md" 这类链接会静默失效 —— 页面能构建、能打开,
# 只有点那一下才发现 404. 这个子命令把全部本地引用真去磁盘上走一遍.

# 行内链接与图片: [x](target)  ![](target)  [x](target "title")
MD_LINK_RE = re.compile(r'''!?\[[^\]]*\]\(\s*<?([^)\s>]*)\s*(?:"[^"]*"|'[^']*')?\s*>?\s*\)''')
# 行内 HTML: src="..." / href="..."
HTML_ATTR_RE = re.compile(r'''(?:src|href)\s*=\s*["']([^"']+)["']''')
# 这些是"不是本地文件"的引用, 直接跳过
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


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="ai-docs 全局唯一 ID (hxid) 工具")
    p.add_argument("--docs-dir", default=DOCS_DIR_DEFAULT,
                   help=f"笔记目录 (默认 {DOCS_DIR_DEFAULT})")
    sub = p.add_subparsers(required=True)

    a = sub.add_parser("assign", help="为缺 hxid 的笔记分配新 id (默认 dry-run)")
    a.add_argument("--write", action="store_true", help="真正落盘 (默认只打印计划)")
    a.set_defaults(func=cmd_assign)

    sub.add_parser("check", help="校验 hxid 唯一性与链接可达性").set_defaults(func=cmd_check)

    i = sub.add_parser("index", help="打印 hxid -> index.md 的映射")
    i.add_argument("--json-out", default=None, help="把索引写成 JSON 文件")
    i.set_defaults(func=cmd_index)

    s = sub.add_parser("snapshot", help="记录/比对 hxid 快照, 用于证明 ID 没有被改写")
    s.add_argument("--file", default=None, help=f"快照路径 (默认 <docs-dir>/{SNAPSHOT_NAME})")
    s.add_argument("--check", action="store_true", help="与已有快照比对, 而不是写入")
    s.set_defaults(func=cmd_snapshot)

    r = sub.add_parser("resolve", help="把正文的 hxid: 链接重算为当前相对路径")
    r.add_argument("--write", action="store_true", help="真正落盘 (默认 dry-run)")
    r.set_defaults(func=cmd_resolve)

    k = sub.add_parser("links", help="逐条校验正文里的本地引用 (含图片/ppt 侧车) 是否真实存在")
    k.add_argument("--show-orphans", action="store_true",
                   help="同时列出没有被任何笔记引用的侧车文件")
    k.set_defaults(func=cmd_links)
    return p


def main() -> int:
    args = build_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())