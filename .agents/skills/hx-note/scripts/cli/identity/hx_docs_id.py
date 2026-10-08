# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""ai-docs 全局唯一 ID (hxid) 工具 (入口): 分配 / 校验 / 索引 / 相对链接解析。"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# scripts/ 与 scripts/cli/ 都上 path: 前者供 `lib.*`, 后者供各域包。
for _p in (Path(__file__).resolve().parents[1], Path(__file__).resolve().parents[2]):
    sys.path.insert(0, str(_p))

from identity.hxid_core import DOCS_DIR_DEFAULT, SNAPSHOT_NAME  # noqa: E402
from identity.hxid_index import (cmd_assign, cmd_check, cmd_index,  # noqa: E402
                                 cmd_resolve, cmd_snapshot)
from identity.hxid_links import cmd_links  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="ai-docs 全局唯一 ID (hxid) 工具")
    p.add_argument("--docs-dir", default=DOCS_DIR_DEFAULT,
                   help=f"笔记目录 (默认 {DOCS_DIR_DEFAULT})")
    sub = p.add_subparsers(required=True)

    a = sub.add_parser("assign", help="为缺 hxid 的笔记分配新 id (默认 dry-run)")
    a.add_argument("--write", action="store_true", help="真正落盘 (默认只打印计划)")
    a.set_defaults(func=cmd_assign)

    sub.add_parser("check", help="校验 hxid 唯一性与链接可达性").set_defaults(func=cmd_check)

    i = sub.add_parser("index", help="打印 hxid -> index.md 的映射")
    i.add_argument("--json-out", default=None, help="把索引写成 JSON 文件")
    i.set_defaults(func=cmd_index)

    s = sub.add_parser("snapshot", help="记录/比对 hxid 快照, 用于证明 ID 没有被改写")
    s.add_argument("--file", default=None, help=f"快照路径 (默认 <docs-dir>/{SNAPSHOT_NAME})")
    s.add_argument("--check", action="store_true", help="与已有快照比对, 而不是写入")
    s.set_defaults(func=cmd_snapshot)

    r = sub.add_parser("resolve", help="把正文的 hxid: 链接重算为当前相对路径")
    r.add_argument("--write", action="store_true", help="真正落盘 (默认 dry-run)")
    r.set_defaults(func=cmd_resolve)

    k = sub.add_parser("links", help="逐条校验正文里的本地引用 (含图片/ppt 侧车) 是否真实存在")
    k.add_argument("--show-orphans", action="store_true",
                   help="同时列出没有被任何笔记引用的侧车文件")
    k.set_defaults(func=cmd_links)
    return p


def main() -> int:
    args = build_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
