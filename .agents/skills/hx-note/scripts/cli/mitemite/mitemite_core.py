"""答题卡 `.hx-mitemite.md` 的共享常量与 block 解析。

add 与 res 按同一个正则识别块头; 分开写会漂移, 于是解析结果对不上。
"""
from __future__ import annotations

import hashlib
import re
import sys
from pathlib import Path

MITEMITE_FILE = ".hx-mitemite.md"

# 块起始行: "## 0x00 a1b2c3d4 begin {"
BLOCK_START_RE = re.compile(
    r'^##\s+(0x[0-9A-Fa-f]{2})\s+([0-9a-f]{8})\s+begin\s+\{$',
    re.MULTILINE,
)

# 序号: "0x00"
SEQ_RE = re.compile(r'^0x[0-9A-Fa-f]{2}$')

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

def _load_file(path: Path) -> str:
    """读取答题卡文件。"""
    if not path.is_file():
        print(f"错误: 找不到 {MITEMITE_FILE} (当前目录: {Path.cwd()})", file=sys.stderr)
        raise SystemExit(1)
    return path.read_text("utf-8")

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
