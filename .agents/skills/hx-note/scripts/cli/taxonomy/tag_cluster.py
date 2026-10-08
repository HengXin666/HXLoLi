"""候选簇与 generated 区块的构造。"""
from __future__ import annotations

import json
from datetime import datetime, timezone

from taxonomy.tag_notes import (DICE_THRESHOLD, containment, dice, display_tag,
                                normalize_tag, shared_affix)
from taxonomy.tag_notes import Note, content_digest

SCHEMA_VERSION = 1
GEN_BEGIN = "# >>> hx-tags:generated begin"
GEN_END = "# <<< hx-tags:generated end"



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
