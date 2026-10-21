#!/usr/bin/env python3
"""Budget, information-architecture, and L3 reference-integrity checks for a skill directory."""
from __future__ import annotations

import re
from pathlib import Path

from frontmatter import frontmatter

MAX_NAME = 64
MAX_DESC = 1024
MAX_COMPAT = 500
MAX_BODY_LINES = 500
BIG_REFERENCE_WORDS = 10_000

IGNORED_DIRS = {'__pycache__', 'node_modules', '.git', '.venv', 'venv'}
IGNORED_SUFFIXES = {'.pyc', '.pyo', '.pyd'}
BANNED_FILES = {
    'README.md', 'CHANGELOG.md', 'INSTALLATION_GUIDE.md',
    'QUICK_REFERENCE.md', 'CONTRIBUTING.md',
}
NAME_RE = re.compile(r'^[a-z0-9]+(-[a-z0-9]+)*$')
RESERVED_NAME_RE = re.compile(r'anthropic|claude', re.IGNORECASE)
# `->` and `=>` stay WARN-level so existing pipelines written as `a -> b` keep passing by default.
ARROW_RE = re.compile(r'(?:-+|=)>')
FIRST_PERSON_RE = re.compile(r'^\s*(?:i\s+can\b|you\s+can\b|\u6211\u53ef\u4ee5)', re.IGNORECASE)
INDEX_PATH_RE = re.compile(r'\b(?:references|scripts|assets|templates|steps|entries|shared)/[\w./-]+')
# Written as escapes on purpose: the prose fixer rewrites a literal fullwidth colon or a dash.
# The list holds an em dash, an en dash, a spaced hyphen, a double hyphen, both colons, and the
# two Chinese cue words; a bracket right after a path counts too (see path_described).
DESC_MARKERS = ('\u2014', '\u2013', ' - ', '--', ':', '\uff1a', '\u4ec0\u4e48\u65f6\u5019\u8bfb', '\u7528\u4e8e')
WHEN_TO_USE_RE = re.compile(
    r'^#{1,6}\s*(when\s+to\s+use|何时使用|什么时候用|使用时机)',
    re.IGNORECASE | re.MULTILINE,
)
# DSH enforces only: a first-level dir with SKILL.md, parseable name+description, and the name
# pattern. Everything else here is community convention DSH never checks, so it defaults to WARN.
STRICT = False


def soft(errors: list[str], warns: list[str], message: str) -> None:
    (errors if STRICT else warns).append(message)


def is_noise(path: Path, root: Path) -> bool:
    rel = path.relative_to(root)
    if any(part in IGNORED_DIRS for part in rel.parts):
        return True
    if any(part.startswith('.') for part in rel.parts):
        return True
    return path.suffix in IGNORED_SUFFIXES


def check_name(fm: dict, skill_dir: Path, errors: list[str], warns: list[str]) -> None:
    name = fm.get('name')
    if not name or not isinstance(name, str):
        errors.append('frontmatter: `name` is required')
        return
    if '<' in name or '>' in name:
        errors.append('name must not contain `<` or `>` (got ' + repr(name) + ')')
    if RESERVED_NAME_RE.search(name):
        errors.append('name must not contain the reserved words `anthropic` or `claude` (got ' + repr(name) + ')')
    if len(name) > MAX_NAME:
        soft(errors, warns, 'name is ' + str(len(name)) + ' chars; the spec suggests <= ' + str(MAX_NAME))
    if not NAME_RE.match(name):
        errors.append(
            'name must be lowercase alphanumeric words joined by single hyphens '
            '(got ' + repr(name) + '); no uppercase, no leading/trailing hyphen, no consecutive hyphens'
        )
    if name != skill_dir.name:
        soft(errors, warns, 'name ' + repr(name) + ' and directory ' + repr(skill_dir.name) + ' differ; the spec suggests they match')


def check_description(fm: dict, errors: list[str], warns: list[str]) -> None:
    desc = fm.get('description')
    if not desc or not isinstance(desc, str) or not desc.strip():
        errors.append('frontmatter: `description` is required')
        return
    if len(desc) > MAX_DESC:
        errors.append('description is ' + str(len(desc)) + ' chars; the limit is ' + str(MAX_DESC))
    if '<' in desc or ARROW_RE.sub('', desc).count('>'):
        errors.append('description must not contain `<` or `>`; they can inject markup into the system prompt')
    elif '>' in desc:
        soft(errors, warns, 'description uses an arrow such as `->`; the official rule bans `>` outright, so prefer words')
    if FIRST_PERSON_RE.match(desc):
        warns.append('description opens in first or second person (`I can` / `You can`); write it in third person or as an imperative')
    if not re.search(r'\buse when\b|\buse this\b|Use when|用于|当用户', desc, re.I):
        warns.append('description has no `Use when ...` trigger phrase; the model only sees name+description, so triggering info must live here')
    if len(desc) < 40:
        warns.append('description is very short; add concrete keywords')


def check_optional(fm: dict, errors: list[str], warns: list[str]) -> None:
    compat = fm.get('compatibility')
    if compat is not None and (not isinstance(compat, str) or len(compat) > MAX_COMPAT):
        soft(errors, warns, 'compatibility must be a string of at most ' + str(MAX_COMPAT) + ' chars')
    metadata = fm.get('metadata')
    if metadata is not None and not isinstance(metadata, dict):
        soft(errors, warns, 'metadata must be a YAML mapping')


def check_references(skill_dir: Path, text: str, errors: list[str], warns: list[str]) -> None:
    """Every L3 file must be reachable from SKILL.md, directly or through a directory entry page."""
    for dirname in ('references', 'scripts', 'assets'):
        directory = skill_dir / dirname
        if not directory.is_dir():
            continue
        for child in sorted(directory.iterdir()):
            rel = child.relative_to(skill_dir).as_posix()
            if child.is_dir():
                if is_noise(child, skill_dir):
                    continue
                has_direct = any(
                    g.relative_to(skill_dir).as_posix() in text
                    for g in child.rglob('*') if g.is_file() and not is_noise(g, skill_dir)
                )
                has_entry = any(name in text for name in (rel + '.md', rel + '/index.md'))
                if not (has_direct or has_entry):
                    soft(errors, warns, rel + '/ is neither referenced nor given an entry page; the model cannot see it exists')
                continue
            if not child.is_file() or is_noise(child, skill_dir):
                continue
            if rel not in text:
                soft(errors, warns, rel + ' exists but is never referenced from SKILL.md')


def check_body(body: str, skill_dir: Path, errors: list[str], warns: list[str]) -> None:
    lines = body.splitlines()
    if len(lines) > MAX_BODY_LINES:
        soft(errors, warns, 'SKILL.md body is ' + str(len(lines)) + ' lines; the budget is under ' + str(MAX_BODY_LINES))
    if WHEN_TO_USE_RE.search(body):
        soft(errors, warns, 'the body carries a When-to-use section; that information only works in `description`')
    for child in sorted(skill_dir.iterdir()):
        if child.name in BANNED_FILES:
            soft(errors, warns, child.name + ' inside a skill; skills are for agents, not for human READMEs')


def path_described(line: str, matches: list[re.Match], index: int) -> bool:
    """A path is described when the text up to the next path carries a marker or opens with a bracket."""
    end = matches[index + 1].start() if index + 1 < len(matches) else len(line)
    tail = line[matches[index].end():end]
    if tail.lstrip().startswith(('(', '\uff08')):
        return True
    return any(marker in tail for marker in DESC_MARKERS)


def check_index_style(body: str, errors: list[str], warns: list[str]) -> None:
    """Bare paths beat markdown links in an index; a bare path still needs a description."""
    prose_lines: list[str] = []
    in_fence = False
    for line in body.splitlines():
        if line.lstrip().startswith('```'):
            in_fence = not in_fence
            continue
        if not in_fence:
            prose_lines.append(re.sub(r'`[^`]*`', '', line))
    prose = chr(10).join(prose_lines)
    links = [
        m.group(0) for m in re.finditer(r'\[([^\]]+)\]\(([^)]+)\)', prose)
        if not m.group(2).startswith(('http://', 'https://', '#'))
    ]
    if links:
        warns.append(
            'index has ' + str(len(links)) + " markdown link(s); a bare path costs one copy instead of two. Example: " + links[0]
        )
    for line in prose_lines:
        stripped = line.strip()
        matches = list(INDEX_PATH_RE.finditer(stripped))
        if len(matches) < 3:
            continue
        bare = [m.group(0) for i, m in enumerate(matches) if not path_described(stripped, matches, i)]
        if bare:
            soft(
                errors, warns,
                'one line lists ' + str(len(matches)) + ' paths, ' + str(len(bare)) + ' with no description: '
                + stripped[:60] + ' ...; add what each one is and when to read it',
            )
    for match in re.finditer(r'\]\(([^)]+)\)', body):
        target = match.group(1).strip()
        if target.startswith(('http://', 'https://', '#', 'mailto:')):
            continue
        depth = len([part for part in Path(target).parts if part not in ('.', '..')])
        if depth > 2:
            warns.append('reference ' + repr(target) + ' nests more than one level deep; the spec suggests flattening')


def check_reference_size(skill_dir: Path, warns: list[str]) -> None:
    directory = skill_dir / 'references'
    if not directory.is_dir():
        return
    for path in sorted(directory.rglob('*.md')):
        if is_noise(path, skill_dir):
            continue
        words = len(path.read_text(encoding='utf-8', errors='replace').split())
        if words > BIG_REFERENCE_WORDS:
            rel = path.relative_to(skill_dir).as_posix()
            warns.append(rel + ' is ~' + str(words) + ' words; add a grep hint in SKILL.md so the agent can locate the right part')


def run(skill_dir: Path) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warns: list[str] = []
    skill_md = skill_dir / 'SKILL.md'
    if not skill_md.is_file():
        return ['SKILL.md not found in ' + str(skill_dir)], warns
    text = skill_md.read_text(encoding='utf-8')
    fm, body, error = frontmatter(text)
    if error:
        return [error], warns
    assert fm is not None
    check_name(fm, skill_dir, errors, warns)
    check_description(fm, errors, warns)
    check_optional(fm, errors, warns)
    check_body(body, skill_dir, errors, warns)
    check_references(skill_dir, text, errors, warns)
    check_reference_size(skill_dir, warns)
    check_index_style(body, errors, warns)
    return errors, warns
