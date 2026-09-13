# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
r"""填写 .hx-mitemite.md 答题卡中某个问题的答案。

Usage:
    uv run hx_mitemite_res.py <序号> [答案内容]

    序号格式: 0x00 ~ 0xFF
    答案内容: 可选，若未提供则从 stdin 读取。

Examples:
    uv run hx_mitemite_res.py "0x00" "我认为核心目标是..."
    uv run hx_mitemite_res.py "0x00" < answer.txt
    echo "多行答案" | uv run hx_mitemite_res.py "0x01"

    未提供序号时列出当前答题卡中所有问题的状态:
    uv run hx_mitemite_res.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

MITEMITE_FILE = ".hx-mitemite.md"

# regex: 匹配 block 起始行
BLOCK_START_RE = re.compile(
    r'^##\s+(0x[0-9A-Fa-f]{2})\s+([0-9a-f]{8})\s+begin\s+\{$',
    re.MULTILINE,
)

# regex: 校验序号
SEQ_RE = re.compile(r'^0x[0-9A-Fa-f]{2}$')


# ---------- helpers ----------

def _read_stdin_or_arg(prompt: str, arg: str | None) -> str:
    """从 arg 或 stdin 获取内容。"""
    if arg is not None:
        return arg
    if not sys.stdin.isatty():
        return sys.stdin.read().rstrip("\n")
    print(prompt, file=sys.stderr)
    return sys.stdin.read().rstrip("\n")


# ---------- 文件读写 ----------

def _load_file(path: Path) -> str:
    """读取答题卡文件。"""
    if not path.is_file():
        print(f"错误: 找不到 {MITEMITE_FILE} (当前目录: {Path.cwd()})", file=sys.stderr)
        raise SystemExit(1)
    return path.read_text("utf-8")


def _rebuild_content(
    text: str,
    target_seq: str,
    answer: str,
) -> str:
    """替换指定序号问题的答案，返回完整文件内容。

    Args:
        text: 原始文件内容
        target_seq: 目标序号
        answer: 新答案内容

    Returns:
        修改后的完整内容。如果找不到 target_seq 则直接返回原文。
    """
    # 按 block 拆分并处理
    result_parts: list[str] = []
    found = False

    pos = 0
    for m in BLOCK_START_RE.finditer(text):
        seq = m.group(1)
        q_hash = m.group(2)

        # 当前 block 之前的内容
        result_parts.append(text[pos:m.start()])
        pos = m.end()

        # 找到下一个 block 的起始位置
        next_m = BLOCK_START_RE.search(text, pos)
        block_end = next_m.start() if next_m else len(text)
        body = text[pos:block_end]
        pos = block_end

        if seq == target_seq:
            found = True
            # 重建该 block
            q_marker = re.search(r'^\*\*Q\*\*:\s*$', body, re.MULTILINE)
            a_marker = re.search(r'^\*\*A\*\*:\s*$', body, re.MULTILINE)

            if q_marker:
                q_start = q_marker.end()
                if a_marker:
                    q_end = a_marker.start()
                else:
                    q_end = len(body)

                question = body[q_start:q_end].strip()

                a_section = f"\n{answer}\n" if answer else "\n"
                result_parts.append(
                    f"## {seq} {q_hash} begin {{\n"
                    f"**Q**:\n"
                    f"{question}\n"
                    f"**A**:{a_section}"
                    f"}}\n"
                )
            else:
                # 格式异常，保留原样
                result_parts.append(m.group(0) + body)
        else:
            # 非目标 block，保留原样
            result_parts.append(m.group(0) + body)

    # 尾部 (最后一个 block 之后的内容)
    result_parts.append(text[pos:])

    if not found:
        print(
            f"错误: 未找到序号 {target_seq} 的问题。"
            f"请使用 'uv run hx_mitemite_res.py' 查看所有问题。",
            file=sys.stderr,
        )
        raise SystemExit(1)

    return "".join(result_parts)


# ---------- 状态展示 ----------

def _list_status(path: Path) -> None:
    """列出所有问题及其状态。"""
    text = _load_file(path)
    blocks = []

    for m in BLOCK_START_RE.finditer(text):
        seq = m.group(1)
        q_hash = m.group(2)
        body_start = m.end()

        next_m = BLOCK_START_RE.search(text, body_start)
        body_end = next_m.start() if next_m else len(text)
        body = text[body_start:body_end]

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
            if a_marker:
                answer_part = body[a_start:]
                closing = answer_part.rfind("}")
                answer = answer_part[:closing].strip() if closing != -1 else answer_part.strip()

        # 截取问题的第一行用于预览
        q_preview = question.split("\n")[0] if question else "(空)"
        if len(q_preview) > 60:
            q_preview = q_preview[:57] + "..."

        status = "✓ 已答" if answer.strip() else "○ 待答"
        blocks.append((seq, q_hash, q_preview, status))

    if not blocks:
        print("(答题卡为空)", file=sys.stderr)
        return

    print(f"\n答题卡: {path}\n", file=sys.stderr)
    print(f"{'序号':<6} {'Hash':<10} {'状态':<8} 问题预览", file=sys.stderr)
    print("-" * 70, file=sys.stderr)
    for seq, q_hash, preview, status in blocks:
        print(f"{seq:<6} {q_hash:<10} {status:<8} {preview}", file=sys.stderr)
    print(file=sys.stderr)


# ---------- 填写答案 ----------

def resolve_answer(seq: str, answer: str) -> Path:
    """填写指定问题的答案。"""
    path = Path.cwd() / MITEMITE_FILE
    original = _load_file(path)
    updated = _rebuild_content(original, seq, answer)

    if updated == original:
        print("⚠ 内容未发生变化", file=sys.stderr)
    else:
        path.write_text(updated, "utf-8")
        print(f"✓ 已填写序号 {seq} 的答案", file=sys.stderr)

    print(f"  文件: {path}", file=sys.stderr)
    return path


# ---------- main ----------

def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] in ("-h", "--help", "help"):
        print(__doc__, file=sys.stderr)
        return 0

    path = Path.cwd() / MITEMITE_FILE

    # 无参数: 列出所有问题状态
    if len(sys.argv) == 1:
        _list_status(path)
        return 0

    seq = sys.argv[1].lower()

    if not SEQ_RE.match(seq):
        print(f"错误: 序号格式无效 '{seq}'，应为 0x00 ~ 0xFF", file=sys.stderr)
        return 1

    content_arg = sys.argv[2] if len(sys.argv) > 2 else None
    answer = _read_stdin_or_arg(
        f"请输入序号 {seq} 的答案 (Ctrl+D 结束):",
        content_arg,
    )

    if not answer.strip():
        print("⚠ 警告: 答案为空，将清空该问题的答案", file=sys.stderr)

    resolve_answer(seq, answer)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
