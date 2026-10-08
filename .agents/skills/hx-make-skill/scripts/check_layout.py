#!/usr/bin/env python3
"""Code-quality gate for a skill directory: directory fan-out, file length, and module form.

Limits and rationale: references/code-quality.md

Checks:
  C1 files-per-dir  -- more than MAX_FILES_PER_DIR files in one directory
  C2 file-length    -- more than MAX_LINES lines in one source file
  C3 module-form    -- a .js / .mjs / .cjs source file; sources must be .ts
  C4 comment-block  -- a comment block longer than MAX_COMMENT_BLOCK lines

A directory may declare its own budget in a local `layout.json`:
  { "maxFiles": 78, "maxLines": 400, "reason": "<why this directory is exempt>" }
The declaration travels with the directory, so a vendored copy keeps it. A declaration at the skill
root is inherited by every directory below it.

A skill whose *upstream* owns its layout declares that at the root instead of splitting:
  { "vendored": true, "reason": "<upstream repo + why local edits would fork it>" }
That is a recorded decision, not a silent pass: the gate still prints it (VENDORED line) so a reader
can see the skill was considered and deliberately exempted.

Usage:
    uv run check_layout.py <skill-dir> [...]    # exit 1 on any violation, 0 when clean
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

MAX_FILES_PER_DIR = 6
MAX_LINES = 300
MAX_COMMENT_BLOCK = 10
BANNED_SUFFIXES = {'.js', '.mjs', '.cjs'}
SOURCE_SUFFIXES = {'.ts', '.tsx', '.py', '.sh', '.lua'}
IGNORED_DIRS = {'node_modules', '.git', 'vendor', '__pycache__', '.venv', 'venv', 'dist', 'build'}
IGNORED_SUFFIXES = {'.pyc', '.pyo', '.pyd'}
# Running assets are data, not source: fonts, wasm, images, and recorded fixtures.
ASSET_SUFFIXES = {
    '.ttf', '.otf', '.woff', '.woff2', '.wasm', '.png', '.jpg', '.jpeg', '.webp', '.gif', '.svg',
    '.mp3', '.wav', '.mp4', '.webm', '.json', '.yaml', '.yml', '.toml', '.csv', '.lock',
}
BUDGET_FILE = 'layout.json'


def interesting(path: Path, root: Path) -> bool:
    rel = path.relative_to(root)
    if any(part in IGNORED_DIRS for part in rel.parts):
        return False
    if path.suffix in IGNORED_SUFFIXES:
        return False
    return not any(part.startswith('.') for part in rel.parts)


def budget_of(directory: Path) -> dict:
    path = directory / BUDGET_FILE
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def budgets_for(root: Path) -> dict:
    """Budgets in effect for every directory, with the skill-root declaration inherited downward."""
    root_budget = {k: v for k, v in budget_of(root).items() if k != 'vendored'}
    return root_budget


def vendored_reason(root: Path) -> str | None:
    """The upstream that owns this skill's layout, when the root declares itself vendored."""
    data = budget_of(root)
    if data.get('vendored') and data.get('reason'):
        return str(data['reason'])
    return None


def check_directories(root: Path, files: list[Path], inherited: dict) -> list[str]:
    """C1 counts every file that is neither a dotfile nor running data.

    The limit keeps a directory scannable in one look. A directory that genuinely needs more states
    it in its own `layout.json` with a reason, so the exception travels with the directory instead of
    living in a reviewer's memory.
    """
    problems: list[str] = []
    groups: dict[Path, list[Path]] = {}
    for path in files:
        if path.suffix in ASSET_SUFFIXES:
            continue
        groups.setdefault(path.parent, []).append(path)
    for directory, members in sorted(groups.items()):
        limit = int({**inherited, **budget_of(directory)}.get('maxFiles', MAX_FILES_PER_DIR))
        if len(members) <= limit:
            continue
        rel = directory.relative_to(root).as_posix() or '.'
        reason = budget_of(directory).get('reason')
        suffix = '' if reason else '  -> split the directory, or declare a budget in ' + BUDGET_FILE
        problems.append(rel + ': ' + str(len(members)) + ' files (limit ' + str(limit) + '): '
                        + ', '.join(p.name for p in members[:8]) + suffix)
    return problems


def check_files(root: Path, files: list[Path], inherited: dict) -> list[str]:
    problems: list[str] = []
    for path in files:
        rel = path.relative_to(root).as_posix()
        if path.suffix in BANNED_SUFFIXES:
            problems.append(rel + ': browser-style module in source position; write .ts (see references/code-quality.md)')
        if path.suffix not in SOURCE_SUFFIXES:
            continue
        try:
            count = len(path.read_text(encoding='utf-8', errors='replace').splitlines())
        except OSError:
            continue
        limit = int({**inherited, **budget_of(path.parent)}.get('maxLines', MAX_LINES))
        if count > limit:
            problems.append(rel + ': ' + str(count) + ' lines (limit ' + str(limit) + ')  -> split the module')
    return problems


def comment_blocks(path: Path) -> list[tuple[int, int]]:
    """Runs of consecutive comment lines, as (start line, length).

    A `/** */` header is counted as one block; the point is to catch a docstring that grew into an
    essay. Four lines of usage explain what a script does; twelve are history nobody asked for.
    """
    try:
        lines = path.read_text(encoding='utf-8', errors='replace').split('\n')
    except OSError:
        return []
    blocks: list[tuple[int, int]] = []
    run_start = 0
    run_len = 0

    def is_comment(text: str) -> bool:
        stripped = text.strip()
        return stripped.startswith(('#', '//', '/*', '*', '<!--'))

    for index, line in enumerate(lines, 1):
        if is_comment(line):
            if run_len == 0:
                run_start = index
            run_len += 1
            continue
        if run_len > 0:
            blocks.append((run_start, run_len))
            run_len = 0
    if run_len > 0:
        blocks.append((run_start, run_len))
    return blocks


def check_comments(root: Path, files: list[Path], inherited: dict) -> list[str]:
    problems: list[str] = []
    for path in files:
        if path.suffix not in SOURCE_SUFFIXES:
            continue
        limit = int({**inherited, **budget_of(path.parent)}.get('maxCommentBlock', MAX_COMMENT_BLOCK))
        rel = path.relative_to(root).as_posix()
        for start, length in comment_blocks(path):
            if length > limit:
                problems.append(rel + ':' + str(start) + ': ' + str(length) + '-line comment block (limit '
                                + str(limit) + ')  -> state what it does; move the reasoning to an Agent Note')
    return problems


def run(roots: list[Path]) -> tuple[list[str], list[str]]:
    """Return (problems, notes). A vendored skill contributes a note instead of problems."""
    problems: list[str] = []
    notes: list[str] = []
    for root in roots:
        upstream = vendored_reason(root)
        if upstream is not None:
            notes.append(root.name + ': vendored, layout owned by upstream — ' + upstream)
            continue
        inherited = budgets_for(root)
        files = [p for p in sorted(root.rglob('*')) if p.is_file() and interesting(p, root)]
        problems.extend(check_directories(root, files, inherited))
        problems.extend(check_files(root, files, inherited))
        problems.extend(check_comments(root, files, inherited))
    return problems, notes


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Skill code-quality gate')
    parser.add_argument('paths', nargs='+', help='skill directories')
    args = parser.parse_args(argv)
    roots = []
    for name in args.paths:
        path = Path(name).resolve()
        if not path.is_dir():
            print('error: ' + str(path) + ' is not a directory', file=sys.stderr)
            return 2
        roots.append(path)
    problems, notes = run(roots)
    for note in notes:
        print('VENDORED ' + note)
    for problem in problems:
        print('ERROR ' + problem)
    if problems:
        print('[check_layout] ' + str(len(problems)) + ' violation(s)')
        return 1
    print('[check_layout] PASS (' + ', '.join(r.name for r in roots) + ')')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
