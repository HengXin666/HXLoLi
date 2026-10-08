# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""从仓库里的既有痕迹构建 [用户画像], 缓存到 ai-docs/.hx-persona.md (入口)。

画像每篇重算一次纯属浪费, 而且结果不稳定会让"引入/展望"段的口吻在不同笔记之间
漂移, 所以默认 TTL 14 天。provider 可插拔: 数据源的可用性是环境相关的, 每个
provider 必须能独立降级, 缺一个就在"数据缺口"里写明缺了什么, 而不是让整次构建失败。

新增数据源 = 写一个 `collect_xxx(ctx) -> Section` 并登记进 PROVIDERS。
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import date
from pathlib import Path

# scripts/ 与 scripts/cli/ 都上 path: 前者供 `lib.*`, 后者供各域包。
for _p in (Path(__file__).resolve().parents[1], Path(__file__).resolve().parents[2]):
    sys.path.insert(0, str(_p))

from persona.persona_build import build, is_stale  # noqa: E402
from persona.persona_core import (DEFAULT_MONTHS, DEFAULT_OUT, DEFAULT_TTL_DAYS,  # noqa: E402
                                  Ctx, months_ago, repo_root)


# Agent Notes: 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织
# 
def cmd_build(args) -> int:
    """Agent Notes
    .agents/notes/implemented/architecture/2026-09-26-persona-moved-to-private-repo.md
    .agents/notes/implemented/architecture/2026-09-26-skill-steps-as-template-method.md
    """
    root = repo_root(Path(args.root) if args.root else None)
    ctx = Ctx(root=root, since=months_ago(date.today(), args.months), months=args.months)
    out_path = root / (args.out or DEFAULT_OUT)
    stale, why = is_stale(out_path, args.ttl)
    if not stale and not args.refresh:
        print(f"skip: {why}; 加 --refresh 强制重建", file=sys.stderr)
        print(out_path)
        return 0
    # Agent Notes: 用户画像移入私有仓, 公开仓只留映射
    # 
    text = build(ctx, args.ttl)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # 写入前的守卫: 画像含"最近在做什么项目", 不能进公开仓。它的家是私有仓,
    # 通过 setup-private 映射成符号链接。没跑过就会在公开仓新建真文件, 所以拦一道。
    # (see )
    if not out_path.is_symlink() and not out_path.is_file():
        print(
            "error: 画像应写入私有仓, 但映射还没建立。\n"
            "  预期路径是符号链接: ai-docs/.hx-persona.md -> ../HXLoLi-imouto/ai-docs/.hx-persona.md\n"
            "  先跑: node scripts/setup-private.mjs\n"
            "  确实要写到别处 (如临时文件), 用 --out 显式指定。",
            file=sys.stderr,
        )
        return 2
    if not out_path.is_symlink() and args.out is None:
        print(
            "error: 目标不是符号链接  会在公开仓里新建真文件, 拒绝写入。\n"
            "  先跑 node scripts/setup-private.mjs 建立映射, 或用 --out 显式指定路径。",
            file=sys.stderr,
        )
        return 2

    out_path.write_text(text, encoding="utf-8")
    print(f"built: {out_path} ({why})", file=sys.stderr)
    print(out_path)
    return 0

def cmd_show(args) -> int:
    root = repo_root(Path(args.root) if args.root else None)
    out_path = root / (args.out or DEFAULT_OUT)
    stale, why = is_stale(out_path, args.ttl)
    if stale:
        print(f"error: {why}; 先跑 `hx_persona.py build`", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps({"path": str(out_path), "status": why,
                          "content": out_path.read_text(encoding="utf-8")},
                         ensure_ascii=False, indent=2))
    else:
        print(out_path.read_text(encoding="utf-8"))
    return 0

def main(argv=None) -> int:
    # Agent Notes: 沉淀必须产出可复用物
    # 
    """Agent Notes
    .agents/notes/implemented/architecture/2026-09-27-sediment-must-yield-reusable-artifacts.md
    """
    p = argparse.ArgumentParser(description="构建/读取 HXLoLi 用户画像")
    p.add_argument("--root", help="仓库根, 默认向上找同时含 ai-docs/ 与 blog/ 的目录")
    p.add_argument("--out", help=f"输出路径, 默认 {DEFAULT_OUT}")
    p.add_argument("--ttl", type=int, default=DEFAULT_TTL_DAYS, help="缓存有效天数")
    sub = p.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("build", help="构建画像 (缓存未过期则跳过)")
    b.add_argument("--months", type=int, default=DEFAULT_MONTHS, help="回溯窗口月数")
    b.add_argument("--refresh", action="store_true", help="忽略 TTL 强制重建")
    b.set_defaults(func=cmd_build)

    s = sub.add_parser("show", help="打印缓存的画像 (过期则报错)")
    s.add_argument("--json", action="store_true")
    s.set_defaults(func=cmd_show)

    args = p.parse_args(argv)
    return args.func(args)

if __name__ == "__main__":
    raise SystemExit(main())
