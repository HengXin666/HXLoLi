# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""标点归一化的命令行入口。

Usage:
    uv run format_cn_punct.py [--check|--diff] [PATH...]

    PATH        文件或目录; 给目录就递归找 `*.md`; 缺省读 stdin 写 stdout.
    --check     只检查是否已规范 (任一文件不合规即 exit 1, 供 hook/CI).
    --diff      打印 before/after 差异, 不写文件.

Examples:
    uv run format_cn_punct.py note.md
    uv run format_cn_punct.py --check note.md
    uv run format_cn_punct.py --check .agents/skills/hx-note
    cat draft.md | uv run format_cn_punct.py
"""
from __future__ import annotations

import argparse
import difflib
import sys
from pathlib import Path

# scripts/ 与 scripts/cli/ 都上 path: 前者供 `lib.*`, 后者供各域包。
for _p in (Path(__file__).resolve().parents[1], Path(__file__).resolve().parents[2]):
    sys.path.insert(0, str(_p))

from lib.textpaths import expand_markdown  # noqa: E402
from textfmt.punct_core import normalize  # noqa: E402


def _print_diff(a: str, b: str) -> None:
    for line in difflib.unified_diff(
        a.splitlines(keepends=True),
        b.splitlines(keepends=True),
        fromfile='before',
        tofile='after',
        lineterm='',
    ):
        sys.stdout.write(line)


def main(argv: list[str] | None = None) -> int:
    """Agent Notes
    .agents/notes/implemented/architecture/2026-10-03-doc-path-expansion-single-source.md
    """
    ap = argparse.ArgumentParser(description='HXLoLi 中文笔记标点归一化')
    ap.add_argument('paths', nargs='*',
                    help='markdown 文件或目录 (目录递归找 *.md); 缺省读 stdin')
    ap.add_argument('--check', action='store_true', help='仅检查是否已规范')
    ap.add_argument('--diff', action='store_true', help='打印差异不写文件')
    args = ap.parse_args(argv)

    if not args.paths:
        data = sys.stdin.read()
        conv = normalize(data)
        if args.check:
            return 0 if data == conv else 1
        if args.diff:
            _print_diff(data, conv)
        else:
            sys.stdout.write(conv)
        return 0

    # Agent Notes: 文风脚本的目录递归只留一份路径展开实现
    # 
    files, missing = expand_markdown(args.paths)
    rc = 0
    for name in missing:
        print('[format_cn_punct] missing:', name, file=sys.stderr)
        rc = 2
    for p in files:
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

if __name__ == "__main__":
    raise SystemExit(main())
