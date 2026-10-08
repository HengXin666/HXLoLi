"""写侧子命令: check 与 apply (把别名改写为规范 tag)。"""
from __future__ import annotations

import json
import re
from pathlib import Path

from taxonomy.tag_cluster import usage_counts
from taxonomy.tag_notes import containment, dice, display_tag, normalize_tag, shared_affix
from taxonomy.tag_commands import tag_health
from taxonomy.tag_notes import FM_RE, content_digest, split_inline
from taxonomy.tag_registry import Registry, load


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
    """Agent Notes
    .agents/notes/implemented/architecture/2026-09-27-sediment-must-yield-reusable-artifacts.md
    """
    key = normalize_tag(tag)
    best, reason = "", ""
    for canonical_key, display in registry.canonical.items():
        # Agent Notes: 沉淀必须产出可复用物
        # 
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
            # Agent Notes: 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织
            # 
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
    """Agent Notes
    .agents/notes/implemented/architecture/2026-09-26-skill-steps-as-template-method.md
    """
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
