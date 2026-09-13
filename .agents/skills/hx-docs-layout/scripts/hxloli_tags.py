# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
r"""HXLoLi ai-docs tag 注册表工具。

注册表是**外挂配置文件** (默认 ai-docs/.hx-tags.toml), 分两层:

  - curated 层 (人类维护): [tags."<规范名>"] 的 desc / aliases. 决定"什么算同一个 tag".
  - generated 层 (脚本重建): 用法频次 / 内容指纹 / 待合并候选. 全部可全量重建.

子命令:

    init                     首次生成注册表 (规范名 = 现有 tag, 别名留空, 由人类合并)
    scan                     扫描内容, 打印统计与**近义候选簇** (不写文件)
    generate                 只重建 generated 区块 (内容变了就重跑)
    suggest "<词>"            给一个新词找最相近的规范 tag (选择而不是新造)
    merge "<别名>" --into "<规范>"   记录合并关系 (写 curated 层)
    check [文件...]           校验笔记 tag 是否规范; 有问题 exit 1
    apply [--write]          把别名改写为规范 tag (默认 dry-run)

必须在 HXLoLi 仓库根目录运行。

近义检测只用**确定性的字符信号** (归一化相同 / 字级 bigram Dice / 公共前后缀 / 整词包含);
真正的语义近义 (如 "记忆系统" vs "记忆架构") 字符上无证据, 脚本**不猜**, 只把候选簇摆出来,
由人类或模型判定后用 merge 固化。阈值声明: DICE_THRESHOLD = 0.6。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import tomllib
import unicodedata
from datetime import datetime, timezone
from pathlib import Path

SCHEMA_VERSION = 1
REGISTRY_FILENAME = ".hx-tags.toml"
DEFAULT_DOCS_DIR = "ai-docs"
GEN_BEGIN = "# >>> hx-tags:generated begin"
GEN_END = "# <<< hx-tags:generated end"
DICE_THRESHOLD = 0.6
CJK_RE = re.compile(r"[\u3400-\u9fff\uf900-\ufaff]")

# ---------------------------------------------------------------- 归一化

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


# ---------------------------------------------------------------- 内容读取

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


# ---------------------------------------------------------------- 注册表

class Registry:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.text = path.read_text(encoding="utf-8") if path.is_file() else ""
        try:
            self.data = tomllib.loads(self.text) if self.text else {}
        except tomllib.TOMLDecodeError as error:
            print(f"错误: 注册表不是合法 TOML: {error}", file=sys.stderr)
            raise SystemExit(2)
        self.ignore: list[str] = [str(item) for item in
                                  ((self.data.get("settings") or {}).get("ignore_notes") or [])]
        self.canonical: dict[str, str] = {}
        self.aliases: dict[str, str] = {}
        self.descriptions: dict[str, str] = {}
        self.parents: dict[str, str] = {}
        for name, entry in (self.data.get("tags") or {}).items():
            display = display_tag(name)
            self.canonical[normalize_tag(display)] = display
            if isinstance(entry, dict):
                if entry.get("desc"):
                    self.descriptions[display] = str(entry["desc"])
                if entry.get("parent"):
                    self.parents[display] = str(entry["parent"])
                for alias in entry.get("aliases") or []:
                    self.aliases[normalize_tag(alias)] = display
        conflicts = [alias for alias in self.aliases if alias in self.canonical]
        self.conflicts = conflicts

    def resolve(self, tag: str) -> tuple[str, str]:
        """返回 (规范化后的规范名, 判定): 'canonical' | 'alias' | 'unknown'。"""
        key = normalize_tag(tag)
        if key in self.canonical:
            return self.canonical[key], "canonical"
        if key in self.aliases:
            return self.aliases[key], "alias"
        return display_tag(tag), "unknown"

    @property
    def digest(self) -> str:
        return str((self.data.get("generated") or {}).get("source_digest") or "")


def content_digest(notes: list[Note]) -> str:
    """内容指纹: 参与计算的只有"哪些笔记用了哪些规范前 tag", 与格式无关。"""
    payload = "\n".join(
        f"{note.path.as_posix()}::{normalize_tag(tag)}"
        for note in notes for tag in note.tags
    )
    return "sha256:" + hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


# ---------------------------------------------------------------- 候选簇

def clusters(notes: list[Note]) -> list[dict]:
    keys: dict[str, str] = {}
    for note in notes:
        for tag in note.tags:
            key = normalize_tag(tag)
            keys.setdefault(key, display_tag(tag))

    parent = {key: key for key in keys}

    def find(node: str) -> str:
        while parent[node] != node:
            parent[node] = parent[parent[node]]
            node = parent[node]
        return node

    def union(a: str, b: str) -> None:
        root_a, root_b = find(a), find(b)
        if root_a != root_b:
            parent[root_b] = root_a

    evidence: dict[tuple[str, str], str] = {}
    ordered = sorted(keys)
    for i, left in enumerate(ordered):
        for right in ordered[i + 1:]:
            why = ""
            if left == right:
                why = "归一化相同"
            elif shared_affix(left, right):
                why = f"公共前后缀 {shared_affix(left, right)!r}"
            elif containment(left, right):
                why = f"整词包含 {containment(left, right)!r}"
            elif dice(left, right) >= DICE_THRESHOLD:
                why = f"字级相似 {dice(left, right):.2f}"
            if why:
                union(left, right)
                evidence[(left, right)] = why

    grouped: dict[str, list[str]] = {}
    for key in ordered:
        grouped.setdefault(find(key), []).append(key)

    result = []
    for members in grouped.values():
        if len(members) < 2:
            continue
        reasons = sorted({why for (left, right), why in evidence.items()
                          if left in members and right in members})
        result.append({
            "tags": [keys[key] for key in members],
            "keys": members,
            "reasons": reasons,
        })
    result.sort(key=lambda item: (-len(item["tags"]), item["tags"]))
    return result


def usage_counts(notes: list[Note]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for note in notes:
        for tag in note.tags:
            counts[normalize_tag(tag)] = counts.get(normalize_tag(tag), 0) + 1
    return counts


def build_generated_block(notes: list[Note]) -> str:
    counts = usage_counts(notes)
    keys = {normalize_tag(tag) for note in notes for tag in note.tags}
    labels = {key: display_tag(next(tag for note in notes for tag in note.tags
                                   if normalize_tag(tag) == key)) for key in keys}
    lines = [
        GEN_BEGIN + " (由 hxloli_tags.py generate 重建, 请勿手改) >>>",
        "[generated]",
        f"schema_version = {SCHEMA_VERSION}",
        f'source_digest = "{content_digest(notes)}"',
        f'generated_at = "{datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}"',
        f"notes_scanned = {len(notes)}",
        f"tags_in_use = {len(keys)}",
        "",
        "[generated.usage]",
    ]
    for key in sorted(counts, key=lambda item: (-counts[item], item)):
        lines.append(f"{json.dumps(labels[key], ensure_ascii=False)} = {counts[key]}")
    lines.extend(["", "# 待合并候选 (字符层面可证的近义; 语义近义需人工判定)"])
    found = clusters(notes)
    if not found:
        lines.append("candidates = []")
    else:
        lines.append("candidates = [")
        for cluster in found:
            payload = ", ".join(json.dumps(name, ensure_ascii=False) for name in cluster["tags"])
            lines.append(f"  [{payload}],  # {'; '.join(cluster['reasons'])}")
        lines.append("]")
    lines.append(GEN_END + " <<<")
    return "\n".join(lines)


def write_generated(registry: Registry, notes: list[Note]) -> None:
    block = build_generated_block(notes)
    text = registry.text
    pattern = re.compile(re.escape(GEN_BEGIN) + r".*?" + re.escape(GEN_END) + r" <<<", re.DOTALL)
    if pattern.search(text):
        text = pattern.sub(lambda _: block, text)
    else:
        # 兼容: 存在不带 marker 的 [generated] 表时整段替换, 避免追加出重复表而弄坏 TOML
        bare = re.search(r"^\[generated\][ \t]*(?:#.*)?$", text, re.MULTILINE)
        if bare:
            rest = text[bare.end():]
            stop = re.search(r"^(?=\[)(?!\[generated)", rest, re.MULTILINE)
            end = bare.end() + (stop.start() if stop else len(rest))
            text = text[:bare.start()] + block + "\n" + text[end:]
        else:
            text = text.rstrip("\n") + "\n\n" + block + "\n"
    registry.path.write_text(text, encoding="utf-8")
    registry.text = text
    registry.data = tomllib.loads(text)


# ---------------------------------------------------------------- 命令

def registry_path(args) -> Path:
    return Path(args.registry) if args.registry else Path(args.docs_dir) / REGISTRY_FILENAME


def load(args) -> tuple[Registry, list[Note]]:
    path = registry_path(args)
    if not path.is_file():
        print(f"错误: 注册表不存在: {path}\n      先运行: hxloli_tags.py init", file=sys.stderr)
        raise SystemExit(2)
    registry = Registry(path)
    docs_dir = Path(args.docs_dir)
    notes = [Note(path, read_frontmatter_tags(path.read_text(encoding="utf-8")))
             for path in note_files(docs_dir, registry.ignore)]
    return registry, notes


def cmd_init(args) -> int:
    path = registry_path(args)
    if path.is_file() and not args.force:
        print(f"错误: {path} 已存在; 需要重建请加 --force", file=sys.stderr)
        return 1
    docs_dir = Path(args.docs_dir)
    existing = Registry(path)
    notes = [Note(p, read_frontmatter_tags(p.read_text(encoding="utf-8")))
             for p in note_files(docs_dir, existing.ignore)]
    counts = usage_counts(notes)
    labels = {normalize_tag(tag): display_tag(tag) for note in notes for tag in note.tags}
    lines = [
        "# HXLoLi ai-docs tag 注册表",
        "#",
        "# 用途: 沉淀笔记时从这里**选** tag, 而不是每次现编近义词。",
        "# curated 层 (本区块) 由人类维护: desc 写这个 tag 管什么, aliases 写要合并进来的近义词。",
        "# generated 区块由 `uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py generate` 重建, 不要手改。",
        "# 常用命令:",
        "#   scan                        看统计与待合并候选",
        "#   merge \"记忆架构\" --into \"记忆系统\"   固化一次合并",
        "#   suggest \"新词\"                新词该归到哪个已有 tag",
        "#   check                       校验全库 tag 是否规范",
        f"schema_version = {SCHEMA_VERSION}",
        "",
        "[settings]",
        "# 合理不需要 tag 的页面 (子串匹配, 相对仓库根或目录名)。",
        'ignore_notes = ["001-关于"]',
        "",
    ]
    for key in sorted(counts, key=lambda item: (-counts[item], item)):
        lines.append(f'[tags."{labels[key]}"]  # {counts[key]} 篇在用')
        lines.append('desc = ""')
        lines.append('aliases = []')
        lines.append("")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")
    registry = Registry(path)
    write_generated(registry, notes)
    print(f"已生成 {path}: {len(counts)} 个规范 tag, {len(notes)} 篇笔记")
    return 0


def cmd_generate(args) -> int:
    registry, notes = load(args)
    before = registry.digest
    write_generated(registry, notes)
    after = registry.digest
    flag = "未变化" if before == after else "已重建"
    print(f"generated 区块{flag}: {registry.path} (digest {before or '(空)'} -> {after})")
    return 0


def cmd_scan(args) -> int:
    _registry, notes = load(args)
    counts = usage_counts(notes)
    labels = {normalize_tag(tag): display_tag(tag) for note in notes for tag in note.tags}
    print(f"笔记 {len(notes)} 篇, tag {len(counts)} 个\n")
    for key in sorted(counts, key=lambda item: (-counts[item], item)):
        print(f"  {counts[key]:3d}  {labels[key]}")
    found = clusters(notes)
    print(f"\n待合并候选 {len(found)} 簇 (纯字符信号, 语义近义需人工判定):")
    if not found:
        print("  (无)")
    for cluster in found:
        print(f"  · {' | '.join(cluster['tags'])}")
        print(f"      {', '.join(cluster['reasons'])}")
    return 0


def cmd_suggest(args) -> int:
    registry, _notes = load(args)
    key = normalize_tag(args.term)
    scored = []
    for canonical_key, display in registry.canonical.items():
        if canonical_key == key:
            scored.append((1.0, display, "就是它 (已归一化相同)"))
            continue
        affix = shared_affix(key, canonical_key)
        contained = containment(key, canonical_key)
        score = dice(key, canonical_key)
        reason = f"字级相似 {score:.2f}"
        if affix:
            score = max(score, 0.75)
            reason = f"公共前后缀 {affix!r}"
        if contained:
            score = max(score, 0.8)
            reason = f"整词包含 {contained!r}"
        scored.append((score, display, reason))
    scored.sort(key=lambda item: -item[0])
    print(f"查询 {args.term!r} (归一化 {key!r}) 的候选规范 tag:")
    for score, display, reason in scored[:args.top]:
        print(f"  {score:.2f}  {display}  — {reason}")
    print("\n若都不合适: 先确认没有可复用的概念, 再新增规范 tag (并在 hx-docs-layout 记录理由)。")
    return 0


def cmd_merge(args) -> int:
    registry, notes = load(args)
    target_key = normalize_tag(args.into)
    if target_key not in registry.canonical:
        print(f"错误: 规范 tag {args.into!r} 不在注册表里", file=sys.stderr)
        return 1
    canonical = registry.canonical[target_key]
    alias = display_tag(args.alias)
    if normalize_tag(alias) == target_key:
        print(f"提示: {alias!r} 与 {canonical!r} 归一化后相同, 无需合并")
        return 0
    drop_source = getattr(args, "drop_source", False)
    if normalize_tag(alias) in registry.canonical:
        existing = registry.canonical[normalize_tag(alias)]
        if not drop_source:
            print(f"错误: {alias!r} 自己是一个规范 tag ({existing!r}); 请先合并它 (hxloli_tags.py merge \"{existing}\" --into \"{canonical}\") 或用 --drop-source 删掉它的表",
                  file=sys.stderr)
            return 1
        # --drop-source: 把源 tag 的表整段删除, 它的用法由 aliases 承接。
        # 只允许在 generated 区块**之前**操作, 且到下一个 [tags. 或区块标记为止,
        # 否则会吃掉后续表 (曾把 85 个表削到 5 个)。
        gen_at = registry.text.find(GEN_BEGIN)
        head = registry.text if gen_at < 0 else registry.text[:gen_at]
        tail = "" if gen_at < 0 else registry.text[gen_at:]
        drop_pattern = re.compile(
            r'^\[tags\."' + re.escape(existing) + r'"\][ \t]*(?:#.*)?\n'
            r'(?:(?!^\[tags\.|^# >>>).*\n)*',
            re.MULTILINE)
        stripped, count = drop_pattern.subn("", head)
        if count:
            registry.path.write_text(stripped + tail, encoding="utf-8")
            registry = Registry(registry.path)
            print(f"已删除源 tag 表 [tags.\"{existing}\"]")

    text = registry.text
    header = f'[tags."{canonical}"]'
    pattern = re.compile(r"^" + re.escape(header) + r"[ \t]*(?:#.*)?$(?P<body>.*?)(?=^\[|\Z)",
                         re.MULTILINE | re.DOTALL)
    match = pattern.search(text)
    if not match:
        print(f"错误: 注册表里找不到 {header}", file=sys.stderr)
        return 1
    body = match.group("body")
    aliases_line = re.search(r"^aliases[ \t]*=[ \t]*\[(.*?)\][ \t]*$", body, re.MULTILINE)
    existing = [] if not aliases_line else [display_tag(x) for x in aliases_line.group(1).split(",") if display_tag(x)]
    if any(normalize_tag(item) == normalize_tag(alias) for item in existing):
        print(f"提示: {alias!r} 已经是 {canonical!r} 的别名")
        return 0
    existing.append(alias)
    rendered = "aliases = [" + ", ".join(json.dumps(item, ensure_ascii=False) for item in existing) + "]"
    if aliases_line:
        new_body = body[:aliases_line.start()] + rendered + body[aliases_line.end():]
    else:
        new_body = body.rstrip("\n") + "\n" + rendered + "\n"
    text = text[:match.start("body")] + new_body + text[match.end("body"):]
    registry.path.write_text(text, encoding="utf-8")

    registry = Registry(registry.path)
    write_generated(registry, notes)
    print(f"已合并: {alias} -> {canonical}  ({registry.path})")
    if args.drop_source:
        print("注意: 若该别名此前是独立规范 tag, 它的 [tags.…] 表需要手动删除。")
    return 0


def tag_health(registry: Registry, notes: list[Note]) -> list[str]:
    """tag 体系的健康度报告。

    注意: check 的主判据是"正文 tag 是否在注册表里", 而注册表由正文自动生成,
    因此它天然恒过 (自己校验自己)。真正指示失效的是下面这些分布指标。
    """
    counts = usage_counts(notes)
    total = len(counts)
    if total == 0:
        return ["tag 总数为 0"]

    # usage_counts 的键是 normalize_tag 后的小写形式, 而 descriptions/parents 用原始大小写,
    # 直接 get 会把 "C++"/"LLM" 这类英文 tag 全部误判成"没有 desc"。
    described_keys = {normalize_tag(t) for t in registry.descriptions}
    parented_keys = {normalize_tag(t) for t in registry.parents}
    singletons = [tag for tag, n in counts.items() if n == 1]
    described = [tag for tag in counts if tag in described_keys]
    parented = [tag for tag in counts if tag in parented_keys]
    lines = [
        f"tag 总数 {total}; 只出现 1 次的 {len(singletons)} 个 ({len(singletons) * 100 // total}%)",
        f"curated desc 覆盖率 {len(described)}/{total}; 声明 parent 的 {len(parented)}/{total}",
    ]

    per_note = sorted(((len(note.tags), note.path) for note in notes), reverse=True)
    crowded = [(n, path) for n, path in per_note if n > 6]
    sparse = [(n, path) for n, path in per_note if n < 3]
    if crowded:
        lines.append(f"tag 过多 (>6) 的笔记: " + ", ".join(f"{path}({n})" for n, path in crowded[:5]))
    if sparse:
        lines.append(f"tag 过少 (<3) 的笔记: " + ", ".join(f"{path}({n})" for n, path in sparse[:5]))

    # 单字符 tag 才有歧义 ("前" 无法区分 前端/前进)。中文里两字词是最常见的词长
    # (前端/协程/检索/爬虫/评测/通勤), 用 len<=2 会把正常词全判成"过细"。
    too_small = [tag for tag, n in counts.items() if n == 1 and len(tag) <= 1]
    if too_small:
        lines.append("粒度过细/无检索价值的候选: " + ", ".join(sorted(too_small)))

    # 单次 tag 的占比有一个由语料规模决定的下限: 若每个 tag 至少出现 2 次,
    # 槽位数 S 必须 >= 2T, 所以至少 max(0, 2T-S) 个 tag 只能出现一次。
    # 拿绝对值 50% 当阈值, 会让语料越小越必然报警 —— 那种"永远响的警报"
    # 只会训练人忽略它。要判的是"超出下限多少"。
    slots = sum(counts.values())
    floor = max(0, 2 * total - slots)
    excess = len(singletons) - floor
    lines.append(f"单次 tag 的理论下限 {floor}/{total} ({floor * 100 // max(total, 1)}%); 实际 {len(singletons)} 个, 超出 {excess} 个")
    if floor * 2 > total:
        lines.append("判定: 语料规模决定下限已过半次, 此项不作为未收敛的证据")
    elif len(singletons) * 2 > total and excess > total // 5:
        lines.append("判定: 单次 tag 显著多于语料下限 -> tag 体系未收敛, 应按 taxonomy 重做归并")
    elif excess > 0:
        lines.append("判定: 单次 tag 接近语料下限, 属正常 (多为待成长的 L2 领域)")
    if len(described) * 2 < total:
        lines.append("判定: 过半 tag 没有 desc -> curated 层未维护, 站点标签页没有解释")
    return lines


def cmd_check(args) -> int:
    registry, notes = load(args)
    if args.paths:
        wanted = {Path(p).resolve() for p in args.paths}
        notes = [note for note in notes if note.path.resolve() in wanted]
    errors, warns = [], []
    if registry.conflicts:
        errors.append("注册表自身冲突: " + ", ".join(registry.conflicts) + " 同时是规范 tag 与别名")
    counts = usage_counts(notes)
    current = content_digest([note for note in notes]) if args.paths else content_digest(notes)
    if not registry.digest:
        warns.append("generated 区块缺失, 请运行 generate")
    elif not args.paths and registry.digest != current:
        warns.append(f"内容已变化 (注册表 digest {registry.digest} != 实际 {current}), 请运行 generate 重建")
    for note in notes:
        if not note.has_field:
            warns.append(f"{note.path}: 没有 tags 字段")
            continue
        for tag in note.tags:
            resolved, kind = registry.resolve(tag)
            if kind == "unknown":
                guess = cmd_suggest_reason(registry, tag)
                errors.append(f"{note.path}: 非规范 tag {tag!r}{guess}")
            elif kind == "alias":
                warns.append(f"{note.path}: {tag!r} 是别名, 规范名是 {resolved!r} (可用 apply --write)")
    for warning in warns:
        print(f"WARN  {warning}")
    for error in errors:
        print(f"ERROR {error}")
    if not errors and not warns:
        print(f"PASS  {len(notes)} 篇笔记的 tag 全部规范 ({len(counts)} 个 tag 在用)")
    if getattr(args, "health", False):
        print("\n-- tag 健康度 (PASS 只说明名字合法, 不代表体系健康) --")
        for line in tag_health(registry, notes):
            print("  " + line)
    return 1 if errors else 0


def cmd_suggest_reason(registry: Registry, tag: str) -> str:
    key = normalize_tag(tag)
    best, reason = "", ""
    for canonical_key, display in registry.canonical.items():
        affix = shared_affix(key, canonical_key)
        contained = containment(key, canonical_key)
        score = dice(key, canonical_key)
        if affix:
            score, reason = max(score, 0.75), f"公共前后缀 {affix!r}"
        if contained:
            score, reason = max(score, 0.8), f"整词包含 {contained!r}"
        if score > 0.5 and score > (0.0 if not best else 0):
            best, reason = display, reason or f"字级相似 {score:.2f}"
    return f" (最接近 {best!r}: {reason})" if best else ""


FM_TAGS_LINE_RE = re.compile(r"^(?P<indent>\s*)tags\s*:(?P<rest>.*)$")


def rewrite_tags(text: str, mapping: dict[str, str]) -> tuple[str, list[tuple[str, str]]]:
    match = FM_RE.match(text)
    if not match:
        return text, []
    lines = text.splitlines(keepends=True)
    changes: list[tuple[str, str]] = []
    for index, line in enumerate(lines):
        stripped = line.rstrip("\n")
        found = FM_TAGS_LINE_RE.match(stripped)
        if not found or index > len(match.group(1).splitlines()):
            continue
        rest = found.group("rest")
        if rest.strip():
            items = split_inline(rest)
            if not items:
                return text, []
            new_items, dedup = [], []
            for item in items:
                resolved, kind = mapping.get(normalize_tag(item), (item, "keep"))
                if kind != "keep" and normalize_tag(resolved) != normalize_tag(item):
                    changes.append((item, resolved))
                if normalize_tag(resolved) not in {normalize_tag(x) for x in new_items}:
                    new_items.append(resolved)
            rendered = "[" + ", ".join(json.dumps(item, ensure_ascii=False) for item in new_items) + "]"
            lines[index] = f"{found.group('indent')}tags: {rendered}\n"
            return "".join(lines), changes
        collected, positions = [], []
        for offset in range(index + 1, len(lines)):
            item = re.match(r"^\s+-\s+(.*?)\s*$", lines[offset].rstrip("\n"))
            if not item:
                break
            collected.append(display_tag(item.group(1)))
            positions.append(offset)
        if not collected:
            return text, []
        new_items = []
        for item, offset in zip(collected, positions):
            resolved = mapping.get(normalize_tag(item), (item, "keep"))[0]
            if normalize_tag(resolved) != normalize_tag(item):
                changes.append((item, resolved))
            if normalize_tag(resolved) not in {normalize_tag(x) for x in new_items}:
                new_items.append(resolved)
        lines[positions[0]:positions[-1] + 1] = [f"    - {item}\n" for item in new_items]
        return "".join(lines), changes
    return text, []


def cmd_apply(args) -> int:
    registry, notes = load(args)
    if args.paths:
        wanted = {Path(p).resolve() for p in args.paths}
        notes = [note for note in notes if note.path.resolve() in wanted]
    total = 0
    for note in notes:
        mapping = {}
        for tag in note.tags:
            resolved, kind = registry.resolve(tag)
            if kind == "alias":
                mapping[normalize_tag(tag)] = (resolved, "alias")
        if not mapping:
            continue
        updated, changes = rewrite_tags(note.path.read_text(encoding="utf-8"), mapping)
        if not changes:
            continue
        total += len(changes)
        print(f"{note.path}")
        for old, new in changes:
            print(f"   {old}  ->  {new}")
        if args.write:
            note.path.write_text(updated, encoding="utf-8")
    if not total:
        print("没有需要改写的 tag")
    elif not args.write:
        print(f"\n(dry-run) 共 {total} 处可改写; 加 --write 落盘")
    else:
        print(f"\n已改写 {total} 处")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="HXLoLi ai-docs tag 注册表工具")
    parser.add_argument("--registry", default=None,
                        help=f"注册表路径 (默认 <docs-dir>/{REGISTRY_FILENAME})")
    parser.add_argument("--docs-dir", default=DEFAULT_DOCS_DIR, help=f"笔记目录 (默认 {DEFAULT_DOCS_DIR})")
    subs = parser.add_subparsers(dest="command", required=True)

    init = subs.add_parser("init", help="首次生成注册表")
    init.add_argument("--force", action="store_true", help="覆盖已存在的注册表")
    init.set_defaults(func=cmd_init)

    subs.add_parser("generate", help="重建 generated 区块").set_defaults(func=cmd_generate)
    subs.add_parser("scan", help="打印统计与待合并候选").set_defaults(func=cmd_scan)

    suggest = subs.add_parser("suggest", help="为新词找最接近的规范 tag")
    suggest.add_argument("term")
    suggest.add_argument("--top", type=int, default=5)
    suggest.set_defaults(func=cmd_suggest)

    merge = subs.add_parser("merge", help="把近义词合并到规范 tag")
    merge.add_argument("alias")
    merge.add_argument("--into", required=True, help="目标规范 tag")
    merge.add_argument("--drop-source", action="store_true", help="别名本身是规范 tag 时仍继续")
    merge.set_defaults(func=cmd_merge)

    check = subs.add_parser("check", help="校验笔记 tag 是否规范")
    check.add_argument("paths", nargs="*")
    check.add_argument("--health", action="store_true",
                       help="额外打印 tag 体系健康度 (粒度/desc 覆盖率/孤儿 tag)")
    check.set_defaults(func=cmd_check)

    apply_cmd = subs.add_parser("apply", help="把别名改写为规范 tag")
    apply_cmd.add_argument("paths", nargs="*")
    apply_cmd.add_argument("--write", action="store_true", help="真正落盘 (默认 dry-run)")
    apply_cmd.set_defaults(func=cmd_apply)

    return parser


def main() -> int:
    args = build_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
