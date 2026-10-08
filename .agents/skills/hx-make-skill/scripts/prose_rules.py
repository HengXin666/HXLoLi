#!/usr/bin/env python3
"""Prose gate for skill artifacts: English punctuation, no terminal punctuation, single-line paragraphs.

Every skill produced with hx-make-skill inherits this gate, so its own text artifacts must pass it too.
Rule statement and rationale: references/prose-rules.md

Checks:
  P1 fullwidth   -- , 。 ？ ！ ： ； “ ” must not appear
  P2 spacing     -- , . ? ! : ; needs one following space unless nothing follows
  P3 terminal    -- a prose line must not end with . , ; : ! (only ? may close a sentence)
  P4 wrap        -- a sentence broken across two lines
  P5 lead        -- a prose line must not start with a break mark

Protected regions are never inspected: YAML frontmatter, fenced blocks, indented code, inline code,
link targets, bold runs, paths, URLs. Headings and table rows are checked for P1/P2 only, since a
table cell is not a line end.

Usage:
    uv run prose_rules.py --check PATH...     # exit 1 on any violation, 0 when clean
    uv run prose_rules.py --fix PATH...       # rewrite in place, then report what changed
    uv run prose_rules.py --list PATH...      # print violations without failing
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

from prose_core import (CJK_RE, FULLWIDTH_TO_ASCII, MARK_FOLLOWED_BY, NEEDS_SPACE, TERMINAL_ALLOWED,
                        TERMINAL_BANNED, analyze, code_spans, expand, group_violations,
                        mask_inline, needs_space_at, prose_blocks)


# Agent Notes: skill 产物受自己那套文本与体量门禁约束, 站点标点规范仍不覆盖 skill
# 
def check_spacing(body: str, lineno: int, out: list[tuple[int, str, str]]) -> None:
    """
    .agents/notes/implemented/process/2026-10-05-skill-artifacts-obey-prose-and-size-gates.md
P2: a mark that ends a clause must be followed by one space.

    Skill text carries its own rules, not the site's: the site face lets a sentence end on `.`,
    this face allows only `?` (see ).
    """
    for match in re.finditer(MARK_FOLLOWED_BY, body):
        if needs_space_at(body, match.start()):
            out.append((lineno, 'P2', 'no space after ' + repr(match.group(0))))


def _descends_from(path: Path, root: Path) -> bool:
    """True when `path` is inside `root`, or inside a directory that declares itself frozen."""
    current = path.resolve()
    anchor = root.resolve()
    while True:
        if (current / 'layout.json').is_file() and _frozen_here(current):
            return True
        if current == anchor or current.parent == current:
            return False
        current = current.parent


def _frozen_here(directory: Path) -> bool:
    declaration = directory / 'layout.json'
    if not declaration.is_file():
        return False
    try:
        data = json.loads(declaration.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return False
    return bool(isinstance(data, dict) and data.get('frozen'))


def _under(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


def vendored_reason(root: Path) -> str | None:
    """The upstream that owns this skill's text, when the skill root declares itself vendored.

    A vendored skill's prose is written upstream: rewriting it here would fork it, and the next
    upstream sync would become a three-way merge with hand edits in it. The declaration lives in
    `layout.json` so the exemption is a recorded decision the gate prints, not a silent pass.
    """
    if not root.is_dir():
        return None
    declaration = root / 'layout.json'
    if not declaration.is_file():
        return None
    try:
        data = json.loads(declaration.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None
    if isinstance(data, dict) and data.get('vendored') and data.get('reason'):
        return str(data['reason'])
    return None


def scan(text: str) -> list[tuple[int, str, str]]:
    """Return (line number, rule id, message) for every violation in `text`."""
    out: list[tuple[int, str, str]] = []
    masked, roles, lines, _ = analyze(text)
    for idx, role in enumerate(roles):
        if role == 'skip':
            continue
        body = masked[idx]
        for char, ascii_form in FULLWIDTH_TO_ASCII.items():
            if char in body:
                out.append((idx + 1, 'P1', 'fullwidth ' + char + ' -> ' + ascii_form))
        check_spacing(body, idx + 1, out)
    tail_view = [mask_inline(line, 'x') for line in lines]
    for begin, end in prose_blocks(roles, lines):
        for idx in range(begin, end):
            tail = tail_view[idx].rstrip()
            if not tail:
                continue
            last = tail[-1]
            if idx < end - 1:
                # A paragraph is one line. Every continuation line is a hard wrap unless it
                # legitimately closes a sentence of its own, which only `?` can do.
                if last not in TERMINAL_ALLOWED:
                    out.append((idx + 1, 'P4', 'hard wrap: the paragraph continues on the next line'))
            elif last in TERMINAL_BANNED:
                out.append((idx + 1, 'P3', 'line ends with ' + repr(last) + '; only ? may close a sentence'))
            if tail.lstrip()[:1] in TERMINAL_BANNED:
                out.append((idx + 1, 'P5', 'line starts with a break mark; merge it into the previous line'))
    return sorted(out)

def _respace(line: str) -> str:
    """Insert the missing space after a clause mark. Inline code is left untouched."""
    parts = re.split(r'(`[^`]*`)', line)
    for idx in range(0, len(parts), 2):
        parts[idx] = _respace_plain(parts[idx])
    return ''.join(parts)


def _respace_plain(text: str) -> str:
    def replace(match: re.Match) -> str:
        if not needs_space_at(text, match.start()):
            return match.group(0)
        return match.group(0) + ' '

    return re.sub(MARK_FOLLOWED_BY, replace, text)


def normalize(text: str) -> str:
    """Rewrite `text` into the gate's shape: halfwidth marks, one space after, no hard wraps."""
    _, roles, lines, _ = analyze(text)
    for idx, role in enumerate(roles):
        if role != 'skip' and lines[idx].strip():
            lines[idx] = _respace(_convert(lines[idx]))
    for begin, end in reversed(prose_blocks(roles, lines)):
        lines[begin:end] = [_merge(lines[begin:end])]
    return '\n'.join(lines)


def _convert(line: str) -> str:
    """Halfwidth marks, one space after each. Code spans are located the same way the judge does."""
    spans = code_spans(line)
    protected = [False] * len(line)
    for start, stop in spans:
        for index in range(start, stop):
            protected[index] = True
    out: list[str] = []
    for pos, char in enumerate(line):
        if protected[pos]:
            out.append(char)
            continue
        ascii_form = FULLWIDTH_TO_ASCII.get(char)
        if ascii_form is None:
            out.append(char)
            continue
        nxt = line[pos + 1] if pos + 1 < len(line) else ''
        if ascii_form in NEEDS_SPACE and nxt and not nxt.isspace():
            out.append(ascii_form + ' ')
        else:
            out.append(ascii_form)
    return ''.join(out)


def _merge(block: list[str]) -> str:
    """One paragraph is one line. A mark that ends a line mid-paragraph stays: rule 2 wants a
    space after it, so the merged line keeps both the mark and the space. Only the last line's
    break mark goes, because a sentence must not end on one."""
    merged = block[0].rstrip()
    for piece in block[1:]:
        nxt = piece.strip()
        if nxt[:1] in TERMINAL_BANNED:
            nxt = nxt[1:].lstrip()
        if _needs_space(merged, nxt):
            merged += ' '
        merged += nxt
    # The paragraph's own end loses its mark (P3). A mark where two lines joined is a real
    # sentence boundary and survives -- it is the final mark only that has to go.
    while merged and merged[-1] in TERMINAL_BANNED:
        merged = merged[:-1]
    return merged


def _needs_space(left: str, right: str) -> bool:
    """A mark's trailing space (rule 2) is mandatory; two CJK runs join bare."""
    if not left or not right:
        return False
    if left[-1] in TERMINAL_BANNED or left[-1] in TERMINAL_ALLOWED:
        return True
    if CJK_RE.search(left[-1]) is not None:
        return False
    return right[:1] not in '、。，：；）」』】》?!'



def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Skill artifact prose gate')
    parser.add_argument('paths', nargs='*', help='markdown files or directories; default reads stdin')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--check', action='store_true', help='report violations, exit 1 on any')
    group.add_argument('--fix', action='store_true', help='rewrite files in place')
    parser.add_argument('--list', action='store_true', dest='list_only', help='print violations without failing')
    args = parser.parse_args(argv)

    if not args.paths:
        data = sys.stdin.read()
        if args.fix:
            sys.stdout.write(normalize(data))
            return 0
        for line, rule, message in scan(data):
            print('stdin:' + str(line) + ': ' + rule + ' ' + message)
        return 0

    files, missing = expand(args.paths)
    status = 0
    for name in missing:
        print('[prose_rules] missing: ' + name, file=sys.stderr)
        status = 2
    skips = [name for name in args.paths if vendored_reason(Path(name)) is not None]
    for name in skips:
        print('VENDORED ' + name + ': upstream owns this text — ' + str(vendored_reason(Path(name))))
    total = 0
    rows: list[tuple[str, int, str, str]] = []
    order: list[str] = []
    for path in files:
        if any(_under(path, Path(name)) for name in skips):
            continue
        if any(_descends_from(path, Path(name)) for name in args.paths if Path(name).is_dir()):
            continue
        original = path.read_text(encoding='utf-8')
        if args.fix:
            fixed = normalize(original)
            if fixed != original:
                path.write_text(fixed, encoding='utf-8')
                print('[prose_rules] fixed: ' + str(path))
            continue
        hits = scan(original)
        if not hits:
            continue
        order.append(str(path))
        total += len(hits)
        rows.extend((str(path), line, rule, message) for line, rule, message in hits)
    if args.fix:
        return status
    if rows:
        for row in group_violations(order, rows):
            print(row)
    if total:
        print('[prose_rules] ' + str(total) + ' violation(s) in ' + str(len(files)) + ' file(s)', file=sys.stderr)
        if not args.list_only:
            status = 1
    return status


if __name__ == '__main__':
    raise SystemExit(main())
