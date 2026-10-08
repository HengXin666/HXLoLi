#!/usr/bin/env python3
"""Validate an Agent Skill against the public Agent Skills specification.

Checks live in checks.py (structure and budget) and frontmatter.py (reading the header);
the rule sources are references/spec-checklist.md and references/code-quality.md.

Usage:
    uv run scripts/validate_skill.py <skill-dir> [--json] [--strict]

Exit codes: 0 = pass (warnings allowed), 1 = errors, 2 = usage error.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import checks  # noqa: E402


def main(argv=None) -> int:
    # Agent Notes: 校验器的无依赖回落不得静默通过非法 frontmatter
    # 
    """Agent Notes
    .agents/notes/implemented/process/2026-09-29-validator-fallback-must-not-pass-invalid-frontmatter.md
    """
    parser = argparse.ArgumentParser(description='Validate an Agent Skill directory.')
    parser.add_argument('skill_dir', help='Path to the skill directory containing SKILL.md')
    parser.add_argument('--json', action='store_true', help='emit a JSON report')
    parser.add_argument('--strict', action='store_true',
                        help='treat community conventions as errors too (default: only what DSH enforces)')
    args = parser.parse_args(argv)
    checks.STRICT = bool(args.strict)

    skill_dir = Path(args.skill_dir).resolve()
    if not skill_dir.is_dir():
        print('error: ' + str(skill_dir) + ' is not a directory', file=sys.stderr)
        return 2

    errors, warns = checks.run(skill_dir)
    if args.json:
        print(json.dumps({'skill': str(skill_dir), 'errors': errors, 'warnings': warns, 'ok': not errors}, indent=2))
    else:
        for warning in warns:
            print('WARN  ' + warning)
        for error in errors:
            print('ERROR ' + error)
        print(('PASS' if not errors else 'FAIL') + ' ' + skill_dir.name
              + ' (' + str(len(errors)) + ' errors, ' + str(len(warns)) + ' warnings)')
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
