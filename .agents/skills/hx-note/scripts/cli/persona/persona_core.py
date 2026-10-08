"""画像的共享层: 常量、数据结构、frontmatter 与日期工具。"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path

DEFAULT_OUT = "ai-docs/.hx-persona.md"
EXTRA_FILE = "ai-docs/.hx-persona.extra.md"
DEFAULT_TTL_DAYS = 14
DEFAULT_MONTHS = 3
DEFAULT_MIN_ITEMS = 6

# 作者的口吻标记。命中越多越可能是一句"只有他会这么写"的原句。
VOICE_MARKERS = [
    "qwq", "awa", "=-=", "\\o/", "喵", "(雾)", "~~", "八嘎", "白嫖", "吃灰",
    "拉胯", "卢瑟", "老登", "哥们", "调教", "镇楼", "艹", "完事了", "就完事",
]

@dataclass
class Section:
    name: str
    ok: bool
    lines: list[str] = field(default_factory=list)
    note: str = ""

@dataclass
class Ctx:
    root: Path
    since: date
    months: int

def repo_root(start: Path | None = None) -> Path:
    cur = (start or Path.cwd()).resolve()
    while True:
        if (cur / "ai-docs").is_dir() and (cur / "blog").is_dir():
            return cur
        if cur.parent == cur:
            return (start or Path.cwd()).resolve()
        cur = cur.parent

def frontmatter(text: str) -> tuple[dict[str, str], str]:
    m = re.match(r"\A---[ \t]*\r?\n(.*?)\r?\n---[ \t]*\r?\n?(.*)\Z", text, re.S)
    if not m:
        return {}, text
    fm: dict[str, str] = {}
    key = None
    for line in m.group(1).splitlines():
        if re.match(r"^\s*-\s+", line) and key:
            fm[key] = (fm.get(key, "") + "," + line.split("-", 1)[1].strip()).strip(",")
            continue
        kv = re.match(r"^([A-Za-z_][\w-]*)\s*:\s*(.*)$", line)
        if kv:
            key = kv.group(1)
            fm[key] = kv.group(2).strip().strip("\"'")
    return fm, m.group(2)

def split_tags(raw: str) -> list[str]:
    """frontmatter 的 tags 有两种写法: 行内 `["a", "b"]` 与多行 `- a`。都要认。"""
    raw = (raw or "").strip().strip("[]")
    return [t.strip().strip("\"'[] ") for t in raw.split(",") if t.strip().strip("\"'[] ")]

def parse_day(raw: str) -> date | None:
    raw = (raw or "").strip().strip("\"'")
    m = re.match(r"(\d{4})[-/](\d{1,2})[-/](\d{1,2})", raw)
    if not m:
        return None
    try:
        return date(int(m.group(1)), int(m.group(2)), int(m.group(3)))
    except ValueError:
        return None

def months_ago(d: date, months: int) -> date:
    y, m = d.year, d.month - months
    while m <= 0:
        m += 12
        y -= 1
    return date(y, m, min(d.day, 28))

def voice_score(line: str) -> int:
    s = sum(3 for mk in VOICE_MARKERS if mk in line)
    if re.search(r"(?<![A-Za-z])我(?![A-Za-z])", line):
        s += 2
    if 12 <= len(line) <= 70:
        s += 2
    if line.count("(") and line.count(")"):
        s += 1
    if line.rstrip().endswith("~"):
        s += 1
    return s
