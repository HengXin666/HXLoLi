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
    for line in raw.splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        if line.startswith((' ', '\t', '-')):
            continue
        if ':' not in line:
            continue
        key, _, val = line.partition(':')
        val = val.strip()
        quoted = val[:1] in ('"', "'")
        if val and not quoted and ': ' in val:
            raise MiniYamlError(
                "bare scalar contains ': ' — quote the whole value (got: " + line.strip()[:60] + ")"
            )
        val = val.strip("\"'")
        if val:
            out[key.strip()] = val
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
