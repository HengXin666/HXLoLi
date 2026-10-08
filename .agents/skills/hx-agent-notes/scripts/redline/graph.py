"""Validate exact bidirectional edges and one anchor per decision per directory."""
import re
import posixpath
from dataclasses import dataclass
from pathlib import PurePosixPath

from anchors import REF, scan
from snapshot import NOTE_ROOT, canonical, guarded

CLASSES = 'architecture|feature|bug-fix|simplification|process|testing'
NOTE = re.compile(r'^\.agents/notes/(implemented|proposed|rejected)/(' + CLASSES + r')/\d{4}-\d{2}-\d{2}-[a-z0-9]+(?:-[a-z0-9]+)*\.md$')


@dataclass
class Note:
    path: str
    identity: str
    code: list[str]
    code_lines: dict[str, int]

    @property
    def directories(self):
        return {str(PurePosixPath(p).parent) for p in self.code}


def diagnostic(rule, path, message, line=1, severity='error', related=None):
    return dict(rule=rule, path=path, line=line, message=message, severity=severity, related=related or [])


def sections(text):
    result, heading, fenced = {}, None, False
    for number, line in enumerate(text.splitlines(), 1):
        if line.startswith(('```', '~~~')):
            fenced = not fenced
            if heading:
                result[heading].append((number, line))
            continue
        if not fenced and line.startswith('## '):
            heading = line[3:]
            if heading in result:
                raise ValueError(f'Duplicate section: {heading}')
            result[heading] = []
        elif heading:
            result[heading].append((number, line))
    return result


def parse_note(path, text):
    match = NOTE.fullmatch(path)
    if not match:
        raise ValueError('Use lifecycle/class/YYYY-MM-DD-topic.md under .agents/notes')
    lines = text.splitlines()
    if len(lines) < 4 or not lines[0].startswith('# Agent Note: ') or lines[1] or lines[3]:
        raise ValueError('Expected title, blank line, Status, blank line')
    if lines[2] != 'Status: ' + match[1]:
        raise ValueError('Status must equal the path lifecycle')
    ids = re.findall(r'^Decision-ID: ([a-z0-9]+(?:-[a-z0-9]+)*)$', text, re.M)
    if len(ids) != 1:
        raise ValueError('Exactly one Decision-ID: lowercase-slug is required')
    if re.search(r'<(?:title|decision-id|code-path|The concrete|One decision|Strongest alternative|its strongest|What this)', text):
        raise ValueError('Replace the generated skeleton placeholders with actual rationale')
    raw = sections(text)
    parts = {key: '\n'.join(line for _, line in value).strip() for key, value in raw.items()}
    for heading in ('Code', 'Problem', 'Decision', 'Alternatives considered', 'Consequences'):
        if not parts.get(heading):
            raise ValueError(f'Missing or empty section: {heading}')
    if not re.search(r'do nothing|reuse|什么都不做|复用', parts['Alternatives considered'], re.I):
        raise ValueError('Alternatives considered must include do nothing / reuse existing')
    code, locations = [], {}
    entries = raw['Code']
    while entries and not entries[0][1].strip():
        entries = entries[1:]
    while entries and not entries[-1][1].strip():
        entries = entries[:-1]
    for number, line in entries:
        entry = re.fullmatch(r'- `([^`]+)`', line)
        if not entry or not canonical(entry[1]):
            raise ValueError('Code must contain only exact repository paths: - `src/example.py`; no globs, braces, directories or prose')
        code.append(entry[1])
        locations[entry[1]] = number
    if len({str(PurePosixPath(p).parent) for p in code}) != len(code):
        raise ValueError('One representative code file per direct parent directory per decision')
    return Note(path, ids[0], code, locations)


def markdown_links(text):
    """Read prose links, excluding fenced examples and inline code spans."""
    prose, fenced = [], None
    for line in text.splitlines():
        marker = re.match(r'^\s*(`{3,}|~{3,})', line)
        if marker:
            if fenced is None:
                fenced = marker[1][0]
            elif marker[1][0] == fenced:
                fenced = None
            continue
        if fenced is None:
            prose.append(re.sub(r'(`+).*?\1', '', line))
    for match in re.finditer(r'\[[^\]]*\]\(\s*(<[^>]+>|[^\s)]+)(?:\s+[\x22\x27].*?[\x22\x27])?\s*\)', '\n'.join(prose)):
        target = match[1].strip('<>')
        if not re.match(r'^[A-Za-z][A-Za-z0-9+.-]*:', target) and not target.startswith('#'):
            yield target


class Graph:
    def __init__(self, snapshot, config, scan_directories=None):
        self.snapshot, self.config = snapshot, config
        self.scan_directories = scan_directories
        self.notes, self.issues, self.sites = {}, [], {}
        self.build()

    def build(self):
        """
        Enforce the bidirectional decision graph
        .agents/notes/implemented/process/2026-10-07-agent-notes-ast-redline.md
        """
        identities = {}
        for path in sorted(self.snapshot.modes):
            if not path.startswith(NOTE_ROOT) or not path.endswith('.md') or PurePosixPath(path).name in {'AGENTS.md', 'README.md'}:
                continue
            if path.startswith(NOTE_ROOT + 'archived/'):
                continue
            try:
                note = parse_note(path, self.snapshot.read(path))
                self.notes[path] = note
                if note.identity in identities:
                    other = identities[note.identity]
                    self.issues.append(diagnostic('duplicate-decision', path, f'Decision-ID already owned by {other}', related=[other]))
                identities[note.identity] = path
            except ValueError as exc:
                self.issues.append(diagnostic('note-format', path, str(exc)))
        sources = {p for p in self.snapshot.modes if guarded(p, self.config)}
        for note in self.notes.values():
            text = self.snapshot.read(note.path)
            for target in REF.findall(text):
                if not canonical(target) or target not in self.snapshot.modes:
                    self.issues.append(diagnostic('note-link', note.path, f'Broken repository note reference: {target}', related=[target]))
            for target in markdown_links(text):
                link = target.split('#', 1)[0]
                resolved = link if link.startswith(NOTE_ROOT) else posixpath.normpath(posixpath.join(posixpath.dirname(note.path), link))
                if not canonical(resolved) or resolved not in self.snapshot.modes:
                    self.issues.append(diagnostic('note-link', note.path, f'Broken markdown link: {target}', related=[resolved]))
            for code in note.code:
                if code not in sources:
                    self.issues.append(diagnostic('code-path', note.path, f'Code path must be an existing guarded source file: {code}', note.code_lines[code], related=[code]))
        for path in sorted(sources):
            if self.scan_directories is not None and str(PurePosixPath(path).parent) not in self.scan_directories:
                continue
            try:
                sites, errors = scan(path, self.snapshot.read(path))
            except (ValueError, UnicodeError) as exc:
                self.issues.append(diagnostic('source-read', path, str(exc)))
                continue
            for rule, line, message in errors:
                self.issues.append(diagnostic(rule, path, message, line))
            self.sites[path] = sites
            for target, line in sites:
                if target not in self.notes:
                    self.issues.append(diagnostic('note-path', path, f'No active note at exact repository path: {target}', line, related=[target]))
                elif path not in self.notes[target].code:
                    self.issues.append(diagnostic('reverse-edge', path, f'{target} must list this exact file under Code', line, related=[target]))
        for note in self.notes.values():
            for code in note.code:
                if self.scan_directories is not None and str(PurePosixPath(code).parent) not in self.scan_directories:
                    continue
                occurrences = [(p, line) for p, sites in self.sites.items() for target, line in sites
                               if target == note.path and str(PurePosixPath(p).parent) == str(PurePosixPath(code).parent)]
                if len(occurrences) != 1 or occurrences[0][0] != code:
                    self.issues.append(diagnostic('anchor-cardinality', note.path,
                        f'Expected exactly one AST declaration anchor in {code}; found {len(occurrences)} in its directory', note.code_lines[code], related=[code]))


def directory_coverage(graph):
    owned = {directory for note in graph.notes.values() for directory in note.directories}
    representatives = {}
    for path in sorted(graph.sites):
        representatives.setdefault(str(PurePosixPath(path).parent), path)
    return [diagnostic('unowned-directory', path,
                       f'Guarded source directory {directory} has no active decision and backlink', severity='review')
            for directory, path in representatives.items() if directory not in owned]


def paired_changes(before, after, changed):
    issues = []
    all_notes = list(before.notes.values()) + list(after.notes.values())
    by_note = {}
    for note in all_notes:
        by_note.setdefault(note.path, set()).update(note.directories)
    changed_code = {p for p in changed if guarded(p, before.config) or guarded(p, after.config)}
    for code in sorted(changed_code):
        directory = str(PurePosixPath(code).parent)
        owners = {path for path, dirs in by_note.items() if directory in dirs}
        if not owners:
            issues.append(diagnostic('unowned-code', code, 'Changed guarded code has no decision for its direct parent directory', severity='review'))
        for note in sorted(owners - changed):
            issues.append(diagnostic('code-only-diff', code, f'Code changed without its decision: {note}; review required', severity='review', related=[note]))
    for path, directories in sorted(by_note.items()):
        if path not in changed:
            continue
        for directory in sorted(directories):
            if not any(str(PurePosixPath(code).parent) == directory for code in changed_code):
                issues.append(diagnostic('note-only-diff', path, f'Note changed without code in {directory}; review required', severity='review'))
    return issues


def affected_paths(before, after, changed):
    affected = set(changed)
    for graph in (before, after):
        for note in graph.notes.values():
            if note.path in changed or any(str(PurePosixPath(p).parent) in note.directories for p in changed):
                affected.add(note.path)
                affected.update(p for p in graph.sites if str(PurePosixPath(p).parent) in note.directories)
                affected.update(note.code)
    return affected
