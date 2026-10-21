#!/usr/bin/env python3
"""Shared vocabulary for the prose gate: the mark tables, the line-role analysis, and path expansion.

`prose_rules.py` judges with it; `prose_fix.py` rewrites with it. Neither owns it.
"""
from __future__ import annotations

import re
from pathlib import Path

FULLWIDTH_TO_ASCII = {
    '\uFF0C': ',', '\u3002': '.', '\uFF1F': '?', '\uFF01': '!',
    '\uFF1A': ':', '\uFF1B': ';', '\u201C': '"', '\u201D': '"',
    '\uFF08': '(', '\uFF09': ')',
}
TERMINAL_BANNED = set('.,;:!') | {chr(0xFF0C), chr(0x3002), chr(0xFF1B), chr(0xFF1A), chr(0xFF01)}
TERMINAL_ALLOWED = {'?', chr(0xFF1F)}
NEEDS_SPACE = set(',.?!:;')
LIST_RE = re.compile(r'^\s*(?:[-*+]|\d+[.)])\s')
SKIP_RE = re.compile(r'^\s*(?:```|~~~|    |\t|<)')
CJK_RE = re.compile(r'[\u3000-\u303f\u4e00-\u9fff\uff00-\uffef]')


def code_spans(line: str) -> list[tuple[int, int]]:
    """Inline code spans, paired the way markdown pairs them: a run of n backticks closes on the
    next run of exactly n.

    Pairing matters because `mask_inline` (judging) and the fixer (rewriting) must agree on which
    characters are code. Toggling per backtick character splits a run of three into ` + ``, so the
    two disagreed and a real replacement happened inside what the judge had masked out.
    """
    runs: list[tuple[int, int]] = []
    index = 0
    while index < len(line):
        if line[index] != '`':
            index += 1
            continue
        end = index
        while end < len(line) and line[end] == '`':
            end += 1
        runs.append((index, end))
        index = end
    spans: list[tuple[int, int]] = []
    used = [False] * len(runs)
    for i, (start, stop) in enumerate(runs):
        if used[i]:
            continue
        width = stop - start
        for j in range(i + 1, len(runs)):
            other_start, other_stop = runs[j]
            if used[j] or other_stop - other_start != width:
                continue
            used[i] = used[j] = True
            spans.append((start, other_stop))
            break
    return sorted(spans)


# A relative path is an identifier, not prose: its dots and dashes take no spaces.
PATH_RE = re.compile(r'(?<![\w/])(?:\.{1,2}/)?[\w.*\-]*[\w\-/*]+\.(?:md|ts|mjs|py|json|ya?ml|sh|html)')
# `.agents`, `.tsx`, `.hx-info.md`: a dot that opens a word is part of a name, not a clause mark.
# A trailing sentence dot is not swallowed, and a dot right after a word, `**`, or a quote is prose.
DOT_IDENT_RE = re.compile(r'(?<![\w.*)\]"\'\u201d\u2019])\.[A-Za-z][\w-]*(?:\.[a-z0-9][\w-]*)*')
URL_RE = re.compile(r'https?://\S+')


def mask_code(line: str, filler: str = ' ') -> str:
    """Blank inline code spans and URLs only. Used where bold and path shapes must stay visible."""
    out = line
    for start, stop in reversed(code_spans(line)):
        out = out[:start] + filler * (stop - start) + out[stop:]
    return URL_RE.sub(lambda m: filler * len(m.group(0)), out)


def mask_identifiers(line: str, filler: str = ' ') -> str:
    """Blank everything that is a name rather than prose: code, link targets, URLs, paths, and
    dot-led identifiers. Bold is left alone, so the fixer still respaces inside it.

    The judge and the fixer both start from this view. Every pattern is matched against the
    original line, never against an already blanked copy: once a path became blanks, the
    sentence dot in `.hx-info.md.Then` looked like the start of a dot-led name and vanished
    from both the check and the fix.
    """
    chars = list(line)
    protected = [False] * len(line)

    def blank(start: int, stop: int) -> None:
        for index in range(start, stop):
            chars[index] = filler
            protected[index] = True

    for start, stop in code_spans(line):
        blank(start, stop)
    for pattern in (URL_RE, re.compile(r'\]\([^)]*\)'), PATH_RE):
        for match in pattern.finditer(line):
            blank(match.start(), match.end())
    for match in DOT_IDENT_RE.finditer(line):
        if not any(protected[match.start():match.end()]):
            blank(match.start(), match.end())
    return ''.join(chars)


def mask_inline(line: str, filler: str = ' ') -> str:
    """Blank out inline code, link targets, bold runs, URLs, and identifiers, keeping the length aligned.

    A clause mark directly before a closing `**` or `)` is followed by a space in practice; the
    mark itself is still checked for being halfwidth, so blanking the run loses no finding.

    `filler` is the character used where a code span or path was removed. Spacing checks want a
    space, so a bold key followed by a code span is not read as a missing space; the end-of-line
    checks want a non-blank character, so a trailing code span does not look like a break mark.
    """
    out = mask_identifiers(line, filler)
    return re.sub(r'\*\*([^*]*)\*\*', lambda m: '**' + 'x' * len(m.group(1)) + '**', out)


GLOB_MARK_RE = re.compile(r'(?<=/)\*\*|\*\*(?=/)')


def bold_marks(view: str) -> list[int]:
    """Start index of every `**` that can delimit bold. A `**` inside a glob (`src/**/*.ts`
    or `**/*.md`) is part of a path, so counting it shifts every closer after it."""
    return [m.start() for m in re.finditer(r'\*\*', view)
            if not GLOB_MARK_RE.match(view, m.start())]


def bold_closers(view: str) -> list[int]:
    """Start index of every `**` that closes a bold run (the 2nd, 4th, ... on the line)."""
    return bold_marks(view)[1::2]


SPLIT_PATH_RE = re.compile(r'/\. (?=[A-Za-z])')
SPLIT_IDENT_RE = re.compile(r'(?:(?<=\s)|^)\. (?=[a-z][\w-]*\.[A-Za-z])')
CJK_GAP_RE = re.compile(CJK_RE.pattern + r' {2,}(?=' + CJK_RE.pattern + ')')


def split_bold_spaces(view: str) -> list[int]:
    """Index of the space in `X. **` where that `**` closes bold: the space kills the closer."""
    out = []
    for pos in bold_closers(view):
        if pos >= 3 and view[pos - 1] == ' ' and view[pos - 2] in NEEDS_SPACE and not view[pos - 3].isspace():
            out.append(pos - 1)
    return out


def damage(line: str) -> list[str]:
    """P6: shapes only an earlier bad respace leaves behind. Code spans and URLs are not inspected."""
    view = mask_code(line, 'x')  # a blanked code span must not read as a gap between CJK
    found = ['space between a mark and its closing ** at col ' + str(i + 1) for i in split_bold_spaces(view)]
    found += ['path split after /. at col ' + str(m.start() + 1) for m in SPLIT_PATH_RE.finditer(view)]
    found += ['dot-led name split at col ' + str(m.start() + 1) for m in SPLIT_IDENT_RE.finditer(view)]
    # A double space between CJK is this repo's house style for a mid-sentence pause, not
    # damage: the em dash was retired, so the gap is deliberate. Flagging it marked 157 real
    # paragraphs as broken and no fixer can restore text that was never lost.
    return found


def heal(line: str) -> str:
    """Undo the respace damage P6 reports. The repair is text-preserving: only a space that a
    bad respace inserted is dropped, never text that was lost.

    `X. **` closing bold becomes `X.**`, with the space moved after the `**` when text follows;
    `/. agents` and ` . drawio.svg` lose the inserted space.

    The view must blank code the same way `damage` does: with a non-space filler. A space
    filler makes a code span look like the damage ` . build.sh`, and the fixer then
    then deletes a legitimate sentence space.
    """
    view = mask_code(line, 'x')
    drop = set(split_bold_spaces(view))
    drop |= {m.end() - 1 for m in SPLIT_PATH_RE.finditer(view)}
    drop |= {m.end() - 1 for m in SPLIT_IDENT_RE.finditer(view)}
    add = {i + 3 for i in split_bold_spaces(view)}
    out: list[str] = []
    for pos, char in enumerate(line):
        if pos in add and not char.isspace() and char not in ')]}.,;:!?"\'':
            out.append(' ')
        if pos not in drop:
            out.append(char)
    return ''.join(out)


def role_of(line: str) -> str:
    if SKIP_RE.match(line):
        return 'skip'
    if line[:1] in ('#', '|', '>'):
        return 'shallow'
    stripped = line.strip()
    # A paragraph that opens with bold emphasis is a definition line, not a wrapped sentence:
    # `**Contract**: the rest of the line` is one clause, and its colon takes no space.
    if stripped.startswith('**') and '**:' in stripped:
        return 'shallow'
    return 'prose'


def prose_blocks(roles: list[str], lines: list[str]) -> list[tuple[int, int]]:
    """Index ranges of each run of prose lines; a list item starts a new run."""
    blocks: list[tuple[int, int]] = []
    start: int | None = None
    for idx, role in enumerate(roles):
        if role == 'prose' and start is None:
            start = idx
            continue
        if role == 'prose' and LIST_RE.match(lines[idx]):
            blocks.append((start, idx))
            start = idx
            continue
        if role != 'prose' and start is not None:
            blocks.append((start, idx))
            start = None
    if start is not None:
        blocks.append((start, len(roles)))
    return blocks


def analyze(text: str) -> tuple[list[str], list[str], list[str], int]:
    """Return (masked line, role, raw line, body start index)."""
    lines = text.split('\n')
    roles: list[str] = ['skip'] * len(lines)
    masked: list[str] = [''] * len(lines)
    start = 0
    if lines and lines[0].strip() == '---':
        for idx in range(1, len(lines)):
            if lines[idx].strip() == '---':
                start = idx + 1
                break
    fence = False
    for idx in range(start, len(lines)):
        raw = lines[idx]
        if raw.lstrip().startswith(('```', '~~~')):
            fence = not fence
            continue
        if fence or not raw.strip():
            continue
        roles[idx] = role_of(raw)
        masked[idx] = mask_inline(raw, ' ')
    return masked, roles, lines, start

# A clause mark must be followed by a space -- except before a closing mark, a path
# separator, a digit (3.14), or `.` (a range or an ellipsis: `01..03`, `../assets`).
MARK_FOLLOWED_BY = r'[,\.\?!:;](?=[^\s\d\)\]\}/\-*_~."\'\u201d\u2019]|[\u4e00-\u9fff])'

def expand(paths: list[str]) -> tuple[list[Path], list[str]]:
    files: list[Path] = []
    missing: list[str] = []
    for name in paths:
        path = Path(name)
        if path.is_dir():
            files.extend(sorted(p for p in path.rglob('*.md') if p.is_file()))
        elif path.is_file():
            files.append(path)
        else:
            missing.append(name)
    return files, missing


def needs_space_at(text: str, index: int) -> bool:
    """True when the clause mark at `index` should be followed by a space.

    The single source of this judgement: the gate reports on it and the fixer rewrites on it, so
    the two can never disagree. A dot inside a word, a decimal, and a dot run are not clause marks.
    """
    mark = text[index]
    before = text[index - 1] if index > 0 else ''
    after = text[index + 1] if index + 1 < len(text) else ''
    if mark == '.':
        # `a.mjs`, `1.5`, `v2.0`: an ASCII word character on both sides. CJK is `isalnum()` too,
        # so the guard must be ASCII-only or every Chinese sentence break is silently skipped.
        if before.isascii() and after.isascii() and before.isalnum() and after.isalnum():
            return False
        if '..' in text[max(0, index - 1):index + 2]:
            return False
        if re.search(r'\d\.\d', text[max(0, index - 1):index + 2]):
            return False
    return True


def group_violations(names: list[str], rows: list[tuple[str, int, str, str]]) -> list[str]:
    """
    .agents/notes/implemented/process/2026-10-06-prose-gate-groups-violations-by-file-and-rule.md
Collapse one row per violation into one row per file, with line numbers grouped by rule.

    Shape and rationale: 
    """
    # Agent Notes: 标点门禁按文件与规则聚合输出, 不再一行一条
    # 
    by_file: dict[str, dict[str, list[int]]] = {}
    for name, line, rule, _message in rows:
        by_file.setdefault(name, {}).setdefault(rule, []).append(line)
    out: list[str] = []
    for name in names:
        groups = by_file.get(name)
        if not groups:
            continue
        out.append(name)
        for rule, lines in groups.items():
            shown = ', '.join(str(n) for n in lines[:12])
            extra = '' if len(lines) <= 12 else ' (+' + str(len(lines) - 12) + ')'
            out.append('  ' + rule + ' x' + str(len(lines)) + ' @ ' + shown + extra)
    return out
