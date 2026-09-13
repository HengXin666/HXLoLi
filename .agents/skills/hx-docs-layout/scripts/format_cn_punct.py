# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
r"""HXLoLi 中文笔记标点归一化工具.

把中文笔记里的全角标点直接转换成英文标点, 并在中文字符后补一个空格.

转换: ，。：；？！（） -> , . : ; ? ! ( )
保留: 、 顿号, 《》「」“” 引号/书名号, — 破折号 (这些在中文正文中按需使用).

示例:
    你好，世界。 -> 你好, 世界.
    注意：这里。 -> 注意: 这里.
    红、绿、蓝 -> 红、绿、蓝
    思考（深度） -> 思考(深度)
    《标题：副题》 -> 《标题: 副题》

保护: frontmatter、围栏代码块、行内代码、URL 原样保留.

Usage:
    uv run format_cn_punct.py [--check|--diff] [FILE...]

    FILE        要处理的 .md 文件; 缺省读 stdin 写 stdout.
    --check     只检查是否已规范 (有则 exit 1, 供 hook/CI).
    --diff      打印 before/after 差异, 不写文件.

Examples:
    uv run format_cn_punct.py note.md
    uv run format_cn_punct.py --check note.md
    uv run format_cn_punct.py --diff note.md
    cat draft.md | uv run format_cn_punct.py
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _cjk_set() -> set[str]:
    # 用运行时 chr 构造 CJK 码位集合, 避免源码内联转义.
    chars: set[str] = set()
    for lo, hi in ((0x4E00, 0x9FFF), (0x3400, 0x4DBF), (0xF900, 0xFAFF)):
        chars.update(chr(c) for c in range(lo, hi + 1))
    return chars

_CJK: set[str] = _cjk_set()

# 全角 -> 半角. 值里不含空格, 空格由规则按语境补.
_FW2HW: dict[str, str] = {
    chr(0xFF0C): chr(0x2C),   # ， -> ,
    chr(0x3002): chr(0x2E),   # 。 -> .
    chr(0xFF1A): chr(0x3A),   # ： -> :
    chr(0xFF1B): chr(0x3B),   # ； -> ;
    chr(0xFF1F): chr(0x3F),   # ？ -> ?
    chr(0xFF01): chr(0x21),   # ！ -> !
    chr(0xFF08): chr(0x28),   # （ -> (
    chr(0xFF09): chr(0x29),   # ） -> )
}

# 需要在中文字符后补空格的英文标点
_NEED_SPACE_AFTER: set[str] = {',', '.', ':', ';', '!', '?'}


_CJK_PUNCT: set[str] = {
    chr(0x300A), chr(0x300B),  # 《 》
    chr(0x300C), chr(0x300D),  # 「 」
    chr(0x300E), chr(0x300F),  # 『 』
    chr(0x2018), chr(0x2019),  # ‘ ’
    chr(0x201C), chr(0x201D),  # “ ”
}


def _is_cjk(ch: str) -> bool:
    return ch in _CJK or ch in _CJK_PUNCT


def _fence_can_close(text: str, i: int, mlen: int, n: int) -> bool:
    # 标记后仅剩空白到行尾/EOF 才算真正关闭
    j = i + mlen
    while j < n and text[j] in ' \t':
        j += 1
    return j >= n or text[j] == chr(10)


def normalize(text: str) -> str:
    """扫描文本, 保护 frontmatter/代码块/行内代码后做标点转换."""
    out: list[str] = []
    n = len(text)
    i = 0
    # frontmatter 保护: 开头 --- 到下一个 --- 行
    if text.startswith('---'):
        nl = text.find(chr(10), 3)
        end = text.find(chr(10) + '---', nl + 1) if nl != -1 else -1
        if end != -1:
            out.append(text[: end + 4])
            i = end + 4
    # fence 与行内码保护: 用状态机逐字符扫
    fence = False
    inline = False
    tok: list[str] = []
    fence_marker = ''

    def flush_tok() -> None:
        # 保护片段原样入 out
        out.append(''.join(tok))
        tok.clear()

    while i < n:
        ch = text[i]
        if fence:
            if text.startswith(fence_marker, i) and _fence_can_close(text, i, len(fence_marker), n):
                # 关闭: 整行(含标记)进 tok, 原样保留后退出围栏态
                k = i
                while k < n and text[k] != chr(10):
                    tok.append(text[k])
                    k += 1
                if k < n:
                    tok.append(chr(10))
                    k += 1
                flush_tok()
                fence = False
                fence_marker = ''
                i = k
                continue
            tok.append(ch)
            i += 1
            continue
        if inline:
            tok.append(ch)
            if ch == '`':
                flush_tok()
                inline = False
            i += 1
            continue
        # 非保护态
        if ch == '`':
            # 判断是否围栏(连续3个及以上)还是行内
            cnt = 0
            while i + cnt < n and text[i + cnt] == '`':
                cnt += 1
            if cnt >= 3:
                fence = True
                fence_marker = '`' * cnt
                for _ in range(cnt):
                    tok.append('`')
                i += cnt
                # 若同行还有语言标注, 一并保护直到行尾
                while i < n and text[i] != chr(10):
                    tok.append(text[i])
                    i += 1
                if i < n:
                    tok.append(chr(10))
                    i += 1
                continue
            inline = True
            tok.append(ch)
            i += 1
            continue
        # 普通字符: 可能转换
        repl = _FW2HW.get(ch)
        if repl is None:
            out.append(ch)
            i += 1
            continue
        prev = text[i - 1] if i > 0 else ''
        nxt = text[i + 1] if i + 1 < n else ''
        if repl in _NEED_SPACE_AFTER:
            # 中文语境下英文标点后补一个空格 (除非已经是空格/换行/行尾)
            if nxt and not nxt.isspace() and not prev.isspace():
                out.append(repl + ' ')
            else:
                out.append(repl)
        else:
            out.append(repl)
        i += 1
    if tok:
        out.append(''.join(tok))
    return ''.join(out)


def _print_diff(a: str, b: str) -> None:
    import difflib
    for line in difflib.unified_diff(
        a.splitlines(keepends=True),
        b.splitlines(keepends=True),
        fromfile='before',
        tofile='after',
        lineterm='',
    ):
        sys.stdout.write(line)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description='HXLoLi 中文笔记标点归一化')
    ap.add_argument('files', nargs='*', help='markdown 文件; 缺省读 stdin')
    ap.add_argument('--check', action='store_true', help='仅检查是否已规范')
    ap.add_argument('--diff', action='store_true', help='打印差异不写文件')
    args = ap.parse_args(argv)

    if not args.files:
        data = sys.stdin.read()
        conv = normalize(data)
        if args.check:
            return 0 if data == conv else 1
        if args.diff:
            _print_diff(data, conv)
        else:
            sys.stdout.write(conv)
        return 0

    rc = 0
    for name in args.files:
        p = Path(name)
        if not p.exists():
            print('[format_cn_punct] missing:', p, file=sys.stderr)
            rc = 2
            continue
        orig = p.read_text(encoding='utf-8')
        conv = normalize(orig)
        if orig == conv:
            continue
        if args.check:
            print('[format_cn_punct] needs format:', p, file=sys.stderr)
            rc = 1
        elif args.diff:
            print('-----', p, '-----')
            _print_diff(orig, conv)
        else:
            p.write_text(conv, encoding='utf-8')
            print('[format_cn_punct] formatted:', p)
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
