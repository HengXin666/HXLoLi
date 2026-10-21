#!/usr/bin/env python3
"""Frontmatter reader for validate_skill.py: YAML when available, a strict fallback otherwise."""
from __future__ import annotations

import re

MAX_COMPAT = 500


class MiniYamlError(ValueError):
    """Raised by the no-PyYAML fallback when the frontmatter is not valid YAML."""


def mini_yaml(raw: str) -> dict:
    """Minimal top-level key: value parser used when PyYAML is unavailable.

    Raises MiniYamlError instead of silently accepting an unquoted scalar containing ': '.
    That shape is a YAML compact-mapping error, and DSH drops the whole skill over it — silently,
    with one logger.warn. A fallback that swallowed it would certify a skill that cannot load.
    """
    out: dict = {}
    lines = raw.splitlines()
    index = 0
    while index < len(lines):
        line = lines[index]
        index += 1
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        if line.startswith((' ', '\t', '-')) or ':' not in line:
            continue
        key, _, val = line.partition(':')
        val = val.strip()
        if val.startswith(('|', '>')):
            folded = val[0] == '>'
            chomping = '-' if '-' in val[1:] else '+' if '+' in val[1:] else ''
            indent_hint = next((int(char) for char in val[1:] if char.isdigit()), 0)
            block: list[str] = []
            while index < len(lines) and (not lines[index].strip() or lines[index].startswith((' ', '\t'))):
                block.append(lines[index])
                index += 1
            nonempty = [item for item in block if item.strip()]
            indent = min((len(item) - len(item.lstrip()) for item in nonempty), default=0)
            if indent_hint:
                indent = indent_hint
            content = [item[indent:] if item.strip() else '' for item in block]
            text = (' '.join(item.strip() for item in content if item.strip())
                    if folded else '\n'.join(content))
            out[key.strip()] = text if chomping != '-' else text.rstrip()
            continue
        quoted = val[:1] in ('"', "'")
        if val and not quoted and ': ' in val:
            raise MiniYamlError(
                "bare scalar contains ': ' — quote the whole value (got: " + line.strip()[:60] + ")"
            )
        out[key.strip()] = val.strip("\"'")
    return out


def frontmatter(text: str) -> tuple[dict | None, str, str]:
    """Return (frontmatter mapping, body, error)."""
    if not text.startswith('---'):
        return None, '', 'SKILL.md does not start with YAML frontmatter'
    match = re.match(r'^---\r?\n(.*?)\r?\n---\r?\n?(.*)$', text, re.DOTALL)
    if not match:
        return None, '', 'frontmatter is not closed with a --- line'
    raw, body = match.group(1), match.group(2)
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(raw)
    except ImportError:
        try:
            data = mini_yaml(raw)
        except MiniYamlError as exc:
            return None, body, 'frontmatter is not valid YAML: ' + str(exc)
    except Exception as exc:  # noqa: BLE001
        return None, body, 'frontmatter is not valid YAML: ' + str(exc)
    if not isinstance(data, dict):
        return None, body, 'frontmatter must be a YAML mapping'
    return data, body, ''
