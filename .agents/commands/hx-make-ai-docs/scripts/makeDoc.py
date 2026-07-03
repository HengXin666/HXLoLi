# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from datetime import date
from pathlib import Path


DEFAULT_SKILL = "hx-make-ai-docs"
DEFAULT_AUTHOR = "Heng_Xin"
ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
CONTROL_CHAR_RE = re.compile(r"[\x00-\x08\x0b\x0c\x0e-\x1f\x7f]")
LITERAL_SGR_RE = re.compile(r"\[(?:\d|;)+m\]?")


def clean_meta(value: str) -> str:
    value = ANSI_ESCAPE_RE.sub("", value)
    value = CONTROL_CHAR_RE.sub("", value)
    value = LITERAL_SGR_RE.sub("", value)
    return value.strip()


def get_current_model() -> str:
    env_keys: tuple[str, ...] = (
        "ANTHROPIC_MODEL",
        "CLAUDE_MODEL",
        "MODEL",
        "OPENAI_MODEL",
    )

    for key in env_keys:
        value: str | None = os.getenv(key)
        if value:
            return clean_meta(value)

    config_paths: tuple[Path, ...] = (
        Path.cwd() / ".claude" / "settings.json",
        Path.home() / ".claude" / "settings.json",
        Path.cwd() / ".codex" / "config.json",
        Path.home() / ".codex" / "config.json",
    )

    for config_path in config_paths:
        if not config_path.is_file():
            continue

        try:
            data: dict = json.loads(config_path.read_text(encoding="utf-8"))

            for key in env_keys:
                value = data.get(key)
                if value:
                    return clean_meta(str(value))

            env = data.get("env", {})
            if isinstance(env, dict):
                for key in env_keys:
                    value = env.get(key)
                    if value:
                        return clean_meta(str(value))

            model = data.get("model")
            if model:
                return clean_meta(str(model))
        except Exception:
            pass

    return "Unknown"


def strip_order_prefix(name: str) -> str:
    return clean_meta(re.sub(r"^\d+[-_]", "", name))


def infer_title(output: Path | None) -> str:
    if output is not None:
        if output.name in ("index.md", "index.mdx"):
            return strip_order_prefix(output.parent.name)
        return strip_order_prefix(output.stem)
    return strip_order_prefix(Path.cwd().name) or "未命名笔记"


def yaml_string(value: str) -> str:
    return json.dumps(value, ensure_ascii=False)


def yaml_list(values: list[str]) -> str:
    if not values:
        return "[]"
    return "[" + ", ".join(yaml_string(value) for value in values) + "]"


def parse_tags(raw_tags: list[str]) -> list[str]:
    tags: list[str] = []
    seen: set[str] = set()

    for raw in raw_tags:
        for item in raw.split(","):
            tag = clean_meta(item)
            if tag and tag not in seen:
                seen.add(tag)
                tags.append(tag)

    return tags


def build_doc(
    *,
    title: str,
    created_at: str,
    model: str,
    skill: str,
    author: str,
    tags: list[str],
) -> str:
    return f"""---
title: {yaml_string(title)}
created_at: {yaml_string(created_at)}
model: {yaml_string(model)}
skill: {yaml_string(skill)}
authors: {yaml_string(author)}
tags: {yaml_list(tags)}
---

# {title}

> [!NOTE]
> 本文由 AI 辅助沉淀, 需要用户 review 后再提交.

## 0x00 背景

TODO: 说明为什么要沉淀这篇笔记, 以及它解决什么问题.

## 0x01 核心结论

TODO: 先给出可以被复用的结论, 再展开推导.

## 0x02 关键细节

TODO: 记录关键概念、实现细节、设计取舍或源码依据.

## 0x03 验证与引用

TODO: 记录验证方式、相关文件、提交链接、参考资料或后续问题.
"""


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate an HXLoLi AI-doc Markdown template.",
    )
    parser.add_argument(
        "title_arg",
        nargs="?",
        help="Document title. Overrides the title inferred from --output or cwd.",
    )
    parser.add_argument(
        "-t",
        "--title",
        help="Document title. Takes precedence over the positional title.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Write the generated template to this file instead of stdout.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite --output when it already exists.",
    )
    parser.add_argument(
        "--model",
        default=get_current_model(),
        help="AI model written to frontmatter.",
    )
    parser.add_argument(
        "--skill",
        default=DEFAULT_SKILL,
        help="Skill or command name written to frontmatter.",
    )
    parser.add_argument(
        "--author",
        default=DEFAULT_AUTHOR,
        help="Author written to frontmatter.",
    )
    parser.add_argument(
        "--tag",
        dest="tags",
        action="append",
        default=[],
        help="Tag for frontmatter. Can be repeated or comma-separated.",
    )
    parser.add_argument(
        "--date",
        default=date.today().isoformat(),
        help="created_at value written to frontmatter.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    title = clean_meta(args.title or args.title_arg or infer_title(args.output))
    content = build_doc(
        title=title,
        created_at=clean_meta(args.date),
        model=clean_meta(args.model),
        skill=clean_meta(args.skill),
        author=clean_meta(args.author),
        tags=parse_tags(args.tags),
    )

    if args.output is None:
        print(content, end="")
        return 0

    output: Path = args.output
    if output.exists() and not args.force:
        print(
            f"error: {output} already exists; pass --force to overwrite",
            file=sys.stderr,
        )
        return 1

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(content, encoding="utf-8")
    print(f"created: {output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
