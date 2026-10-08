# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""生成 HXLoLi ai-docs 的笔记模板 (入口)。"""
from __future__ import annotations

import argparse
import sys
from datetime import date
from pathlib import Path

# scripts/ 与 scripts/cli/ 都上 path: 前者供 `lib.*`, 后者供各域包。
for _p in (Path(__file__).resolve().parents[1], Path(__file__).resolve().parents[2]):
    sys.path.insert(0, str(_p))

from authoring.makedoc_meta import (DEFAULT_AUTHOR, clean_meta,  # noqa: E402
                                    get_project_model, infer_title, parse_skills,
                                    parse_tags)
from authoring.makedoc_template import (build_doc,  # noqa: E402
                                        new_hxid, read_existing_hxid)


# Agent Notes: 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织; 沉淀必须产出可复用物
# 
# 
def parse_args(argv: list[str]) -> argparse.Namespace:
    """Agent Notes
    .agents/notes/implemented/architecture/2026-09-26-skill-steps-as-template-method.md
    .agents/notes/implemented/architecture/2026-09-27-sediment-must-yield-reusable-artifacts.md
    """
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
        default=get_project_model(),
        help=(
            "AI model written to frontmatter. Prefer passing this explicitly; "
            "otherwise makeDoc.py tries HX_AI_DOCS_MODEL/CODEX_MODEL/etc, "
            "then optional config.toml fallback."
        ),
    )
    parser.add_argument(
        "--skill",
        dest="skills",
        action="append",
        default=[],
        help=(
            "Skill or command name written to frontmatter. Repeatable or "
            "comma-separated; emits a YAML list in the `skill` field."
        ),
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
    parser.add_argument(
        "--hxid",
        default="",
        help=(
            "全局唯一笔记 ID (hx-xxxxxxxx). 默认自动生成; 仅在需要沿用已有 ID 时显式传入. "
            "ID 用于跨文章引用: [标题](hxid:hx-xxxxxxxx), 目录移动后可由 hx_docs_id.py resolve 重算."
        ),
    )
    return parser.parse_args(argv)

def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    title = clean_meta(args.title or args.title_arg or infer_title(args.output))

    # hxid 不可变: 显式传入 > 已有文件里的旧值 > 新生成。
    # --force 覆盖已有笔记时必须沿用旧 ID, 否则引用会断。
    existing_hxid = read_existing_hxid(args.output) if args.output else ""
    hxid = clean_meta(args.hxid) or existing_hxid or new_hxid()

    content = build_doc(
        hxid=hxid,
        title=title,
        created_at=clean_meta(args.date),
        model=clean_meta(args.model),
        skills=parse_skills(args.skills),
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
    origin = " (沿用已有 hxid)" if existing_hxid and not clean_meta(args.hxid) else ""
    print(f"created: {output}{origin}", file=sys.stderr)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
