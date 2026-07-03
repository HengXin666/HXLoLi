# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
r"""添加问题到 .hx-mitemite.md 答题卡。

Usage:
    uv run hx_mitemite_add.py <序号> [问题内容]

    序号格式: 0x00, 0x01, ..., 0x0A, 0x0B, ..., 0xFF
    问题内容: 可选，若未提供则从 stdin 读取。

Examples:
    uv run hx_mitemite_add.py "0x00" "这篇笔记的核心目标是什么?"
    uv run hx_mitemite_add.py "0x01" "$(cat question.md)"
    echo "多行问题内容" | uv run hx_mitemite_add.py "0x02"

文件格式 (.hx-mitemite.md):
    ## 0x00 a1b2c3d4 begin {
    **Q**:
    问题内容
    可多行
    **A**:
    答案内容
    可多行
    }
"""

from __future__ import annotations

import hashlib
import re
import sys
from pathlib import Path

MITEMITE_FILE = ".hx-mitemite.md"

# regex: 匹配 block 起始行 "## 0x00 a1b2c3d4 begin {"
BLOCK_START_RE = re.compile(
    r'^##\s+(0x[0-9A-Fa-f]{2})\s+([0-9a-f]{8})\s+begin\s+\{$',
    re.MULTILINE,
)

# regex: 匹配序号 (用于校验)
SEQ_RE = re.compile(r'^0x[0-9A-Fa-f]{2}$')


# ---------- helpers ----------

def _hash(content: str) -> str:
    """生成内容的 8 位 hex hash。"""
    return hashlib.md5(content.encode("utf-8")).hexdigest()[:8]


def _read_stdin_or_arg(prompt: str, arg: str | None) -> str:
    """从 arg 或 stdin 获取内容。"""
    if arg is not None:
        return arg
    if not sys.stdin.isatty():
        return sys.stdin.read().rstrip("\n")
    print(prompt, file=sys.stderr)
    return sys.stdin.read().rstrip("\n")


# ---------- block 解析 ----------

def _parse_blocks(text: str) -> list[dict]:
    """解析 .hx-mitemite.md 中的所有问题块。

    Returns:
        [
            {
                "seq": "0x00",
                "hash": "a1b2c3d4",
                "question": "问题内容...",
                "answer": "答案内容...",
                "raw": "原始 block 文本 (含首尾行)"
            },
            ...
        ]
        按文件中出现顺序返回。
    """
    blocks: list[dict] = []

    for m in BLOCK_START_RE.finditer(text):
        seq = m.group(1)
        q_hash = m.group(2)

        # block 内容: 从 begin { 之后到下一个 block 之前 (或 EOF)
        body_start = m.end()
        next_m = BLOCK_START_RE.search(text, body_start)
        body_end = next_m.start() if next_m else len(text)
        body = text[body_start:body_end]

        # 分离 Q 和 A
        q_marker = re.search(r'^\*\*Q\*\*:\s*$', body, re.MULTILINE)
        a_marker = re.search(r'^\*\*A\*\*:\s*$', body, re.MULTILINE)

        question = ""
        answer = ""

        if q_marker:
            q_start = q_marker.end()
            if a_marker:
                q_end = a_marker.start()
                a_start = a_marker.end()
            else:
                q_end = len(body)
                a_start = len(body)

            question = body[q_start:q_end].strip()

            # 去掉末尾的 } 闭包符和空白
            if a_marker:
                answer_part = body[a_start:]
                closing = answer_part.rfind("}")
                if closing != -1:
                    answer = answer_part[:closing].strip()
                else:
                    answer = answer_part.strip()

        blocks.append({
            "seq": seq,
            "hash": q_hash,
            "question": question,
            "answer": answer,
            "raw": m.group(0) + body,
        })

    return blocks


def _format_block(seq: str, q_hash: str, question: str, answer: str) -> str:
    """格式化单个问题块。"""
    a_section = f"\n{answer}\n" if answer else "\n"
    return (
        f"## {seq} {q_hash} begin {{\n"
        f"**Q**:\n"
        f"{question}\n"
        f"**A**:{a_section}"
        f"}}\n"
    )


# ---------- 核心操作 ----------

def add_question(seq: str, question: str) -> Path:
    """添加/更新问题到答题卡。

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


# ---------- main ----------

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
