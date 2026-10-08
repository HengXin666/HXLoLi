"""读侧子命令: init / generate / scan / suggest / merge / 健康度。"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

from taxonomy.tag_cluster import GEN_BEGIN, SCHEMA_VERSION, clusters, usage_counts
from taxonomy.tag_notes import containment, dice, display_tag, normalize_tag, shared_affix
from taxonomy.tag_notes import Note, note_files, read_frontmatter_tags
from taxonomy.tag_registry import Registry, load, registry_path, write_generated


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
        "# generated 区块由 `uv run .agents/skills/hx-note/scripts/cli/taxonomy/hxloli_tags.py generate` 重建, 不要手改。",
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
    print("\n若都不合适: 先确认没有可复用的概念, 再新增规范 tag (并在 hx-note 的步骤 5 记录理由)。")
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
    # 拿绝对值 50% 当阈值, 会让语料越小越必然报警  那种"永远响的警报"
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
