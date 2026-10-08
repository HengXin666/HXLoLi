"""tag 归一化、字符级相似证据、frontmatter 解析与内容指纹。

归一化与相似证据只用**确定性字符信号** (归一化相同 / 字级 bigram Dice /
公共前后缀 / 整词包含); 真正的语义近义字符上无证据, 不猜。
"""
from __future__ import annotations

import hashlib
import re
import sys
import unicodedata
from pathlib import Path


CJK_RE = re.compile(r"[\u3400-\u9fff\uf900-\ufaff]")
DICE_THRESHOLD = 0.6



def normalize_tag(value: str) -> str:
    """把 tag 归一到比较键. 生成端与查询端**共用**这一个函数。"""
    text = unicodedata.normalize("NFKC", str(value))
    text = text.strip().strip('"').strip("'")
    text = re.sub(r"[\s\u3000]+", "", text)
    text = text.replace("-", "").replace("_", "")
    return text.lower()


def display_tag(value: str) -> str:
    return unicodedata.normalize("NFKC", str(value)).strip().strip('"').strip("'").strip()


def bigrams(key: str) -> set[str]:
    if len(key) < 2:
        return {key} if key else set()
    return {key[i:i + 2] for i in range(len(key) - 1)}


def dice(a: str, b: str) -> float:
    left, right = bigrams(a), bigrams(b)
    if not left or not right:
        return 0.0
    return 2 * len(left & right) / (len(left) + len(right))


def shared_affix(a: str, b: str) -> str:
    """最长公共前缀或后缀。

    含中文要求 >=2 字 (中文双字即有意义); 纯 ASCII 要求 >=5 字符, 或含非字母数字符号
    (如 "c++" / ".net")。这样能滤掉 "compaction"/"tokenization" 共享的构词后缀 "tion"。
    """
    limit = min(len(a), len(b))
    for size in range(limit, 1, -1):
        candidates = []
        if a[:size] == b[:size]:
            candidates.append(a[:size])
        if a[-size:] == b[-size:]:
            candidates.append(a[-size:])
        for candidate in candidates:
            if CJK_RE.search(candidate) or size >= 5 or re.search(r"[^0-9a-z]", candidate):
                return candidate
    return ""


def containment(a: str, b: str) -> str:
    """整词包含; 被包含者需含中文或长度 >=3, 避免 "ai" 这种前缀造成噪声。"""
    short, long = (a, b) if len(a) <= len(b) else (b, a)
    if short and short != long and short in long:
        if CJK_RE.search(short) or len(short) >= 3:
            return short
    return ""



FM_RE = re.compile(r"^---\r?\n(.*?)\r?\n---\r?\n?", re.DOTALL)


TAGS_INLINE_RE = re.compile(r"^\s*\[?(.*?)\]?\s*$")


def split_inline(raw: str) -> list[str]:
    body = raw.strip()
    if body.startswith("[") and body.endswith("]"):
        body = body[1:-1]
    if not body:
        return []
    return [display_tag(item) for item in body.split(",") if display_tag(item)]


def read_frontmatter_tags(text: str) -> list[str] | None:
    """返回 frontmatter 的 tags; 没有该字段返回 None (与"空列表"区分)。"""
    match = FM_RE.match(text)
    if not match:
        return None
    lines = match.group(1).splitlines()
    for index, line in enumerate(lines):
        if not re.match(r"^tags\s*:", line):
            continue
        rest = line.split(":", 1)[1]
        if rest.strip():
            return split_inline(rest)
        collected: list[str] = []
        for follower in lines[index + 1:]:
            if re.match(r"^\s+-\s+", follower):
                item = re.sub(r"^\s+-\s+", "", follower)
                if display_tag(item):
                    collected.append(display_tag(item))
            elif not follower.strip():
                continue
            else:
                break
        return collected
    return None


def note_files(docs_dir: Path, ignore: list[str] | None = None) -> list[Path]:
    if not docs_dir.is_dir():
        print(f"错误: 找不到目录 {docs_dir}", file=sys.stderr)
        raise SystemExit(2)
    found = sorted(path for path in docs_dir.rglob("index.md")
                   if not any(part.startswith(".") for part in path.relative_to(docs_dir).parts))
    patterns = ignore or []
    return [path for path in found
            if not any(pattern in path.as_posix() or pattern in path.parent.name
                       for pattern in patterns)]


class Note:
    def __init__(self, path: Path, tags: list[str] | None) -> None:
        self.path = path
        self.tags = tags or []
        self.has_field = tags is not None


def content_digest(notes: list[Note]) -> str:
    """内容指纹: 参与计算的只有"哪些笔记用了哪些规范前 tag", 与格式无关。"""
    payload = "\n".join(
        f"{note.path.as_posix()}::{normalize_tag(tag)}"
        for note in notes for tag in note.tags
    )
    return "sha256:" + hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]
