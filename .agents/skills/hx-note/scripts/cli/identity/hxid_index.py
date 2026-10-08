"""hxid 索引与三类闸门: assign / check / snapshot / index / resolve。"""
from __future__ import annotations

import argparse
import json
import re
import secrets
import sys
from pathlib import Path

from identity.hxid_core import (HXID_RE, LINK_RE, SNAPSHOT_NAME, Note, iter_notes,
                                is_exempt, load_exempt, rel_link)


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
