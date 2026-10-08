# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""HXLoLi ai-docs tag 注册表工具 (入口)。

注册表是**外挂配置文件** (默认 ai-docs/.hx-tags.toml), 分两层:

  - curated 层 (人类维护): [tags."<规范名>"] 的 desc / aliases. 决定"什么算同一个 tag".
  - generated 层 (脚本重建): 用法频次 / 内容指纹 / 待合并候选. 全部可全量重建.

子命令: init / scan / generate / suggest / merge / check / apply
必须在 HXLoLi 仓库根目录运行。近义检测只用确定性的字符信号 (归一化相同 / 字级 bigram
Dice / 公共前后缀 / 整词包含); 语义近义不猜, 只把候选簇摆出来。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# scripts/ 与 scripts/cli/ 都上 path: 前者供 `lib.*`, 后者供各域包。
for _p in (Path(__file__).resolve().parents[1], Path(__file__).resolve().parents[2]):
    sys.path.insert(0, str(_p))

from taxonomy.tag_apply import cmd_apply, cmd_check  # noqa: E402
from taxonomy.tag_commands import (cmd_generate, cmd_init, cmd_merge,  # noqa: E402
                                   cmd_scan, cmd_suggest)
from taxonomy.tag_registry import REGISTRY_FILENAME  # noqa: E402

DEFAULT_DOCS_DIR = "ai-docs"



def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="HXLoLi ai-docs tag 注册表工具")
    parser.add_argument("--registry", default=None,
                        help=f"注册表路径 (默认 <docs-dir>/{REGISTRY_FILENAME})")
    parser.add_argument("--docs-dir", default=DEFAULT_DOCS_DIR, help=f"笔记目录 (默认 {DEFAULT_DOCS_DIR})")
    subs = parser.add_subparsers(dest="command", required=True)

    init = subs.add_parser("init", help="首次生成注册表")
    init.add_argument("--force", action="store_true", help="覆盖已存在的注册表")
    init.set_defaults(func=cmd_init)

    subs.add_parser("generate", help="重建 generated 区块").set_defaults(func=cmd_generate)
    subs.add_parser("scan", help="打印统计与待合并候选").set_defaults(func=cmd_scan)

    suggest = subs.add_parser("suggest", help="为新词找最接近的规范 tag")
    suggest.add_argument("term")
    suggest.add_argument("--top", type=int, default=5)
    suggest.set_defaults(func=cmd_suggest)

    merge = subs.add_parser("merge", help="把近义词合并到规范 tag")
    merge.add_argument("alias")
    merge.add_argument("--into", required=True, help="目标规范 tag")
    merge.add_argument("--drop-source", action="store_true", help="别名本身是规范 tag 时仍继续")
    merge.set_defaults(func=cmd_merge)

    check = subs.add_parser("check", help="校验笔记 tag 是否规范")
    check.add_argument("paths", nargs="*")
    check.add_argument("--health", action="store_true",
                       help="额外打印 tag 体系健康度 (粒度/desc 覆盖率/孤儿 tag)")
    check.set_defaults(func=cmd_check)

    apply_cmd = subs.add_parser("apply", help="把别名改写为规范 tag")
    apply_cmd.add_argument("paths", nargs="*")
    apply_cmd.add_argument("--write", action="store_true", help="真正落盘 (默认 dry-run)")
    apply_cmd.set_defaults(func=cmd_apply)

    return parser



def main() -> int:
    args = build_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
