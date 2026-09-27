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


# ── 硬约束 vs 软约定 ─────────────────────────────────────────────
#
# DSH 的 skill 加载器 (dsh-skill-filesystem) 真正强制的只有三件事:
#   1. 一级目录下有 SKILL.md, 且首行是 ---;
#   2. frontmatter 能解析出 name 与 description;
#   3. name 匹配 /^[a-z0-9]+(?:-[a-z0-9]+)*$/。
# 违反任一条 = 该 skill 被**静默跳过** (只打一行 logger.warn)。
#
# 其余检查 (行数预算、引用完整性、禁用文件、索引写法……) 全是社区规范的软约定,
# DSH **一条都不查** —— 实测确认。所以默认降级成 WARN: 自定义目录结构
# (如 steps/ entries/ shared/) 不该被自己的工具挡住。
# 需要按完整规范校验时加 --strict, 它们会重新变成 ERROR。
STRICT = False


def _soft(errors: list[str], warns: list[str], message: str) -> None:
    """按 STRICT 决定一条软约定算 ERROR 还是 WARN。"""
    (errors if STRICT else warns).append(message)


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
            _soft(errors, warns, f"name is {len(name)} chars; 规范建议 <= {MAX_NAME}")
        if not NAME_RE.match(name):
            errors.append(
                "name must be lowercase alphanumeric words joined by single "
                f"hyphens (got {name!r}); no uppercase, no leading/trailing "
                "hyphen, no consecutive hyphens"
            )
        # DSH 不要求 name == 目录名 (它只读 frontmatter 的 name)。不一致会让
        # 「按目录找 skill」的人困惑, 所以只在 strict 下算错。
        if name != skill_dir.name:
            _soft(errors, warns,
                  f"name {name!r} 与目录名 {skill_dir.name!r} 不一致; 规范建议一致")

    # --- description ---
    desc = fm.get("description")
    if not desc or not isinstance(desc, str) or not desc.strip():
        errors.append("frontmatter: `description` is required")
    else:
        if len(desc) > MAX_DESC:
            _soft(errors, warns, f"description is {len(desc)} chars; 规范建议 <= {MAX_DESC}")
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
    # DSH 只读 name 与 description; compatibility/metadata 它不解析。
    if compat is not None and (not isinstance(compat, str) or len(compat) > MAX_COMPAT):
        _soft(errors, warns, f"compatibility must be a string of at most {MAX_COMPAT} chars")
    md = fm.get("metadata")
    if md is not None and not isinstance(md, dict):
        _soft(errors, warns, "metadata must be a YAML mapping")

    # --- L2 budget ---
    body_lines = body.splitlines()
    # DSH 不查行数。超长不会报错, 但每次触发整篇读入, 这份上下文是物理成本。
    if len(body_lines) > MAX_BODY_LINES:
        _soft(errors, warns,
              f"SKILL.md body is {len(body_lines)} lines; 建议 < {MAX_BODY_LINES} "
              "(超了能跑, 但每次触发都要付这份上下文)")

    # --- information architecture ---
    if WHEN_TO_USE_RE.search(body):
        _soft(errors, warns,
              "body 里有 When to use 段; 触发信息写在 description 里才有效 "
              "(body 触发前不可见) —— 这样写等于白写")

    for child in sorted(skill_dir.iterdir()):
        # DSH 不查。这属于「给模型的目录里别放给人看的东西」的纪律。
        if child.name in BANNED_FILES:
            _soft(errors, warns,
                  f"{child.name} inside a skill; skills 是给 agent 的, 一般不放给人看的 README")

    # --- reference integrity ---
    #
    # 规则 (2026-09 放宽, 支持"步骤文件夹"式扩展):
    #   · 顶层文件 (references/foo.md) 必须被 SKILL.md 直接引用;
    #   · 子目录 (references/collect/) 必须有一个以目录名命名的入口页
    #     (references/collect.md 或 references/collect/index.md) 被 SKILL.md 引用,
    #     入口页再负责指引该目录下的细节 —— 深链发生在 reference 之间, 不发生在 SKILL.md。
    #   这样既允许按步骤分组建子目录, 又保住"模型知道它存在"这条规范意图。
    ref_dirs = [d for d in ("references", "scripts", "assets") if (skill_dir / d).is_dir()]
    for d in ref_dirs:
        for f in sorted((skill_dir / d).iterdir()):
            rel = f.relative_to(skill_dir).as_posix()
            if f.is_dir():
                if _is_noise(f, skill_dir):
                    continue
                # 子目录: 满足以下任一即可 ——
                #   (a) 目录里有文件被 SKILL.md 直接引用 (如 assets/ci/github-actions.yml), 或
                #   (b) 有入口页被引用 (references/collect.md 或 references/collect/index.md)。
                # 两者缺一才报错。只查入口页会误伤"直接引用子目录文件"这种更精确的写法。
                has_direct = any(
                    g.relative_to(skill_dir).as_posix() in text
                    for g in f.rglob("*") if g.is_file() and not _is_noise(g, skill_dir)
                )
                entry_names = [f"{rel}.md", f"{rel}/index.md"]
                has_entry = any(e in text for e in entry_names)
                if not (has_direct or has_entry):
                    _soft(errors, warns,
                          f"{rel}/ 既没被直接引用、也没有入口页; 模型不知道它存在")
                continue
            if not f.is_file() or _is_noise(f, skill_dir):
                continue
            if rel not in text:
                _soft(errors, warns, f"{rel} exists but is never referenced from SKILL.md")

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

    # --- 索引写法 (两条硬约定) ---
    # 跳过代码块: 规范文档里会用反例演示错误写法, 那些不该被当成违规.
    prose_lines: list[str] = []
    _in_fence = False
    for _line in body.splitlines():
        if _line.lstrip().startswith("```"):
            _in_fence = not _in_fence
            continue
        if not _in_fence:
            # 剥掉行内 code: 规范文档会用它示范错误写法, 那些不该算违规
            prose_lines.append(re.sub(r'`[^`]*`', '', _line))
    prose = chr(10).join(prose_lines)

    md_links = [
        m.group(0)
        for m in re.finditer(r'\[([^\]]+)\]\(([^)]+)\)', prose)
        if not m.group(2).startswith(('http://', 'https://', '#'))
    ]
    if md_links:
        warns.append(
            '索引里有 ' + str(len(md_links)) + ' 个 markdown 链接; 给 AI 读的索引用裸路径即可,'
            ' 写成 [a](a) 只是把路径重复两遍、多付一份 token. 例: ' + md_links[0]
        )

    for _line in prose_lines:
        _s = _line.strip()
        _paths = re.findall(r'\b(references|scripts|assets|templates|steps|entries|shared)/[\w./-]+', _s)
        if len(_paths) >= 3:
            _has_desc = any(ch in _s for ch in ('——', ':', '：', '什么时候读', '用于'))
            if not _has_desc:
                warns.append(
                    '有一行平铺了 ' + str(len(_paths)) + ' 个路径却没有描述: '
                    + _s[:60] + ' ...; 每个路径后面补一句 这是什么/什么时候读'
                )

    # --- reference depth ---
    for m in re.finditer(r"\]\(([^)]+)\)", body):
        target = m.group(1).strip()
        if target.startswith(("http://", "https://", "#", "mailto:")):
            continue
        depth = len([p for p in Path(target).parts if p not in (".", "..")])
        if depth > 2:
            _soft(errors, warns,
                  f"reference {target!r} 嵌套超过一层; 规范建议拍平")

    return errors, warns


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Validate an Agent Skill directory.")
    p.add_argument("skill_dir", help="Path to the skill directory containing SKILL.md")
    p.add_argument("--json", action="store_true", help="emit a JSON report")
    p.add_argument("--strict", action="store_true",
                   help="把社区规范的软约定也当 ERROR (默认只报 DSH 真正强制的)")
    args = p.parse_args(argv)
    global STRICT
    STRICT = bool(args.strict)

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
