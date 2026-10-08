"""hxid 的共享层: 常量、正则、Note 对象、笔记枚举与豁免清单。

hxid 写在 frontmatter, 形如 hxid: "hx-3f9a2c71", 创建时分配一次, 之后**永不变更**。
正文跨文章引用统一写成 [标题](hxid:hx-3f9a2c71)  目录被移动/改名后, 跑 resolve
即可把所有 hxid 链接重算成当前正确的相对路径。同目录内引用仍用普通相对路径。
"""
from __future__ import annotations

import os
import re
from pathlib import Path

DOCS_DIR_DEFAULT = "ai-docs"
SNAPSHOT_NAME = ".hx-id-snapshot.json"
HXID_RE = re.compile(r"^hx-[0-9a-f]{8}$")

FM_RE = re.compile(r"\A---[ \t]*\r?\n(.*?)\r?\n---[ \t]*\r?\n", re.S)

FM_HXID_RE = re.compile(r"^hxid[ \t]*:[ \t]*[\"']?([^\"'\r\n]+?)[\"']?[ \t]*$", re.M)

# 链接的两种形态: bare -> [标题](hxid:hx-xxxxxxxx) (源码里的可移植形态);
# tagged -> [标题](../002-乙/index.md "hxid:hx-xxxxxxxx") (resolve 之后的可渲染形态)。
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

# Agent Notes: 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织; 沉淀必须产出可复用物
# 
# 
def load_exempt(docs_dir: Path) -> list[str]:
    """
    .agents/notes/implemented/architecture/2026-09-26-skill-steps-as-template-method.md
    .agents/notes/implemented/architecture/2026-09-27-sediment-must-yield-reusable-artifacts.md
豁免清单: 这些页面的路径片段允许没有 hxid (非笔记页面)。

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
