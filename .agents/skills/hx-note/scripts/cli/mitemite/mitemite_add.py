"""追加问题块: add_question 与它的命令行入口。"""
from __future__ import annotations

import sys
from pathlib import Path

from mitemite.mitemite_core import (MITEMITE_FILE, SEQ_RE, _format_block, _hash,
                                    _parse_blocks, _read_stdin_or_arg)


# Agent Notes: 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织; 沉淀必须产出可复用物
# 
# 
def add_question(seq: str, question: str) -> Path:
    """
    .agents/notes/implemented/architecture/2026-09-26-skill-steps-as-template-method.md
    .agents/notes/implemented/architecture/2026-09-27-sediment-must-yield-reusable-artifacts.md
添加/更新问题到答题卡。

    Args:
        seq:  序号，如 "0x00"
        question: 问题内容 (可多行)

    Returns:
        答题卡文件路径。
    """
    path = Path.cwd() / MITEMITE_FILE

    # 读取已有 blocks
    if path.is_file():
        blocks = _parse_blocks(path.read_text("utf-8"))
    else:
        blocks = []

    q_hash = _hash(question)

    # 查找是否已有相同序号的 block
    found = False
    for b in blocks:
        if b["seq"] == seq:
            old_hash = b["hash"]
            b["hash"] = q_hash
            b["question"] = question
            b["answer"] = ""  # 问题变更后清空旧答案
            found = True
            if q_hash != old_hash:
                print(f"⚠ 序号 {seq} 已存在，已更新问题并清空答案", file=sys.stderr)
            else:
                print(f"⚠ 序号 {seq} 已存在，内容未变", file=sys.stderr)
            break

    if not found:
        blocks.append({
            "seq": seq,
            "hash": q_hash,
            "question": question,
            "answer": "",
            "raw": "",
        })
        print(f"✓ 已添加问题 {seq} (hash: {q_hash})", file=sys.stderr)

    # 按序号排序后写出
    blocks.sort(key=lambda b: int(b["seq"], 16))
    content = "\n".join(
        _format_block(b["seq"], b["hash"], b["question"], b["answer"])
        for b in blocks
    )
    path.write_text(content, "utf-8")
    print(f"  文件: {path}", file=sys.stderr)
    return path

def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help", "help"):
        print(__doc__, file=sys.stderr)
        return 0 if len(sys.argv) > 1 else 1

    seq = sys.argv[1].lower()

    if not SEQ_RE.match(seq):
        print(f"错误: 序号格式无效 '{seq}'，应为 0x00 ~ 0xFF", file=sys.stderr)
        return 1

    content_arg = sys.argv[2] if len(sys.argv) > 2 else None
    question = _read_stdin_or_arg(
        f"请输入序号 {seq} 的问题内容 (Ctrl+D 结束):",
        content_arg,
    )

    if not question.strip():
        print("错误: 问题内容不能为空", file=sys.stderr)
        return 1

    add_question(seq, question)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
