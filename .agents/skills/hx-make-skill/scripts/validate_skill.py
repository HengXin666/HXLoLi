#!/usr/bin/env python3
"""Validate an Agent Skill against the public Agent Skills specification.

Checks implemented (see references/spec-checklist.md for the source of each):
  R  name: <=64 chars, [a-z0-9-], no leading/trailing hyphen, no "--",
     and must equal the parent directory name
  R  description: present, 1..1024 chars
  R  optional fields: license / compatibility (<=500) / metadata / allowed-tools
  S  SKILL.md body under 500 lines (L2 budget)
  S  no "When to Use" style section in the body (belongs in description)
  S  no README.md / CHANGELOG.md / INSTALLATION_GUIDE.md / QUICK_REFERENCE.md
  S  every references/ and scripts/ file is referenced from SKILL.md
  S  file references stay one level deep from SKILL.md
  W  references/ files larger than 10k words should have a grep hint

Exit codes: 0 = pass (warnings allowed), 1 = errors, 2 = usage error.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

MAX_NAME = 64
MAX_DESC = 1024
MAX_COMPAT = 500
MAX_BODY_LINES = 500
BIG_REFERENCE_WORDS = 10_000

# Build/tooling artifacts that are never agent-facing and must not be
# reported as "unreferenced". Kept deliberately narrow.
IGNORED_DIRS = {"__pycache__", "node_modules", ".git", ".venv", "venv"}
IGNORED_SUFFIXES = {".pyc", ".pyo", ".pyd"}


def _is_noise(path: Path, root: Path) -> bool:
    """True for tooling artifacts that should be skipped entirely."""
    rel = path.relative_to(root)
    if any(part in IGNORED_DIRS for part in rel.parts):
        return True
    if any(part.startswith(".") for part in rel.parts):
        return True
    return path.suffix in IGNORED_SUFFIXES


BANNED_FILES = {
    "README.md", "CHANGELOG.md", "INSTALLATION_GUIDE.md",
    "QUICK_REFERENCE.md", "CONTRIBUTING.md",
}

NAME_RE = re.compile(r"^[a-z0-9]+(-[a-z0-9]+)*$")
WHEN_TO_USE_RE = re.compile(
    r"^#{1,6}\s*(when\s+to\s+use|何时使用|什么时候用|使用时机)",
    re.IGNORECASE | re.MULTILINE,
)


def _frontmatter(text: str):
    """Return (frontmatter_dict, body, error). Parses YAML without a dependency."""
    if not text.startswith("---"):
        return None, "", "SKILL.md does not start with YAML frontmatter"
    m = re.match(r"^---\r?\n(.*?)\r?\n---\r?\n?(.*)$", text, re.DOTALL)
    if not m:
        return None, "", "frontmatter is not closed with a --- line"
    raw, body = m.group(1), m.group(2)
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(raw)
    except ImportError:
        data = _mini_yaml(raw)
    except Exception as exc:  # noqa: BLE001
        return None, body, f"frontmatter is not valid YAML: {exc}"
    if not isinstance(data, dict):
        return None, body, "frontmatter must be a YAML mapping"
    return data, body, ""


def _mini_yaml(raw: str) -> dict:
    """Minimal top-level key: value parser used when PyYAML is unavailable."""
    out: dict = {}
    for line in raw.splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if line.startswith((" ", "\t", "-")):
            continue
        if ":" not in line:
            continue
        key, _, val = line.partition(":")
        val = val.strip().strip("\"'")
        if val:
            out[key.strip()] = val
    return out


def validate(skill_dir: Path) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warns: list[str] = []
    skill_md = skill_dir / "SKILL.md"
    if not skill_md.is_file():
        return ["SKILL.md not found in " + str(skill_dir)], warns

    text = skill_md.read_text(encoding="utf-8")
    fm, body, err = _frontmatter(text)
    if err:
        return [err], warns
    assert fm is not None

    # --- name ---
    name = fm.get("name")
    if not name or not isinstance(name, str):
        errors.append("frontmatter: `name` is required")
    else:
        if len(name) > MAX_NAME:
            errors.append(f"name is {len(name)} chars; max is {MAX_NAME}")
        if not NAME_RE.match(name):
            errors.append(
                "name must be lowercase alphanumeric words joined by single "
                f"hyphens (got {name!r}); no uppercase, no leading/trailing "
                "hyphen, no consecutive hyphens"
            )
        if name != skill_dir.name:
            errors.append(
                f"name {name!r} must match the parent directory name "
                f"{skill_dir.name!r}"
            )

    # --- description ---
    desc = fm.get("description")
    if not desc or not isinstance(desc, str) or not desc.strip():
        errors.append("frontmatter: `description` is required")
    else:
        if len(desc) > MAX_DESC:
            errors.append(f"description is {len(desc)} chars; max is {MAX_DESC}")
        has_trigger = bool(re.search(r"\buse when\b|\buse this\b|Use when|用于|当用户", desc, re.I))
        if not has_trigger:
            warns.append(
                "description has no `Use when ...` trigger phrase; the model "
                "only sees name+description, so triggering info must live here"
            )
        if len(desc) < 40:
            warns.append("description is very short; add concrete keywords")

    # --- optional fields ---
    compat = fm.get("compatibility")
    if compat is not None and (not isinstance(compat, str) or len(compat) > MAX_COMPAT):
        errors.append(f"compatibility must be a string of at most {MAX_COMPAT} chars")
    md = fm.get("metadata")
    if md is not None and not isinstance(md, dict):
        errors.append("metadata must be a YAML mapping")

    # --- L2 budget ---
    body_lines = body.splitlines()
    if len(body_lines) > MAX_BODY_LINES:
        errors.append(
            f"SKILL.md body is {len(body_lines)} lines; keep it under "
            f"{MAX_BODY_LINES} and split detail into references/"
        )

    # --- information architecture ---
    if WHEN_TO_USE_RE.search(body):
        errors.append(
            "body contains a \"When to use\" heading; that information is "
            "never read before triggering -- move it into `description`"
        )

    for child in sorted(skill_dir.iterdir()):
        if child.name in BANNED_FILES:
            errors.append(
                f"{child.name} must not live inside a skill; skills are for the "
                "agent, not a human-facing project"
            )

    # --- reference integrity ---
    ref_dirs = [d for d in ("references", "scripts", "assets") if (skill_dir / d).is_dir()]
    for d in ref_dirs:
        for f in sorted((skill_dir / d).rglob("*")):
            if not f.is_file() or _is_noise(f, skill_dir):
                continue
            rel = f.relative_to(skill_dir).as_posix()
            if rel not in text:
                errors.append(f"{rel} exists but is never referenced from SKILL.md")

    for d in ("references",):
        ref_dir = skill_dir / d
        if not ref_dir.is_dir():
            continue
        for f in sorted(ref_dir.rglob("*.md")):
            if _is_noise(f, skill_dir):
                continue
            words = len(f.read_text(encoding="utf-8", errors="replace").split())
            rel = f.relative_to(skill_dir).as_posix()
            if words > BIG_REFERENCE_WORDS:
                warns.append(
                    f"{rel} is ~{words} words; add a grep hint in SKILL.md so "
                    "the agent can locate the right part"
                )

    # --- reference depth ---
    for m in re.finditer(r"\]\(([^)]+)\)", body):
        target = m.group(1).strip()
        if target.startswith(("http://", "https://", "#", "mailto:")):
            continue
        depth = len([p for p in Path(target).parts if p not in (".", "..")])
        if depth > 2:
            errors.append(
                f"reference {target!r} is nested deeper than one level below "
                "SKILL.md; flatten it"
            )

    return errors, warns


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Validate an Agent Skill directory.")
    p.add_argument("skill_dir", help="Path to the skill directory containing SKILL.md")
    p.add_argument("--json", action="store_true", help="emit a JSON report")
    args = p.parse_args(argv)

    skill_dir = Path(args.skill_dir).resolve()
    if not skill_dir.is_dir():
        print(f"error: {skill_dir} is not a directory", file=sys.stderr)
        return 2

    errors, warns = validate(skill_dir)
    if args.json:
        print(json.dumps({"skill": str(skill_dir), "errors": errors,
                          "warnings": warns, "ok": not errors}, indent=2))
    else:
        for w in warns:
            print(f"WARN  {w}")
        for e in errors:
            print(f"ERROR {e}")
        status = "PASS" if not errors else "FAIL"
        print(f"{status} {skill_dir.name} ({len(errors)} errors, {len(warns)} warnings)")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
