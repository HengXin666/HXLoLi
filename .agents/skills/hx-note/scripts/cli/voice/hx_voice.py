# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""AI 味静态检测器 + 可增长表达语料库 (入口)。

设计前提: "去 AI 味" 如果只写成一句要求, 模型会自评通过。所以判据必须是
**可机械检测的**, 并且阈值来自这个仓库的真实证据。

三条子命令:
  lint     按 profile 检查文件, 命中 E 级规则则 exit 1
  learn    把一条"好/坏表达"追加进项目级语料库 ai-docs/.hx-voice.toml
  samples  打印语料库里已积累的表达 (写作前读, 盲审后补)

profile 决定用哪套规则:
  atom     .hx-info.md  面向知识库的原子知识点 (最严: 不许铺垫/反问/第一人称/配图)
  article  index.md     面向人类的派生文章
  blog     blog/*.md    随笔 (最松, 只查 AI 套话)

规则与阈值是**数据**不是代码: RULES 表里改一行就能调, ai-docs/.hx-voice.toml
里加一条 [[bad]] 就能长出新规则。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# scripts/ 与 scripts/cli/ 都上 path: 前者供 `lib.*`, 后者供各域包。
for _p in (Path(__file__).resolve().parents[1], Path(__file__).resolve().parents[2]):
    sys.path.insert(0, str(_p))

from voice.voice_corpus import cmd_learn, cmd_samples  # noqa: E402
from voice.voice_lint import cmd_fix, cmd_lint  # noqa: E402


# Agent Notes: 加句长检查, 阈值来自手写语料的实测分布; 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织
# 
# 
def main(argv=None) -> int:
    # Agent Notes: 沉淀必须产出可复用物
    # 
    """Agent Notes
    .agents/notes/implemented/architecture/2026-09-26-sentence-length-gate.md
    .agents/notes/implemented/architecture/2026-09-26-skill-steps-as-template-method.md
    .agents/notes/implemented/architecture/2026-09-27-sediment-must-yield-reusable-artifacts.md
    .agents/notes/implemented/architecture/2026-10-03-doc-path-expansion-single-source.md
    """
    p = argparse.ArgumentParser(description="AI 味静态检测 + 表达语料库")
    p.add_argument("--db", help="语料库路径, 默认向上找 ai-docs/.hx-voice.toml")
    sub = p.add_subparsers(dest="cmd", required=True)

    # Agent Notes: 文风脚本的目录递归只留一份路径展开实现
    # 
    lt = sub.add_parser("lint", help="检查文件的 AI 味")
    lt.add_argument("paths", nargs="+",
                    help="文件或目录 (目录递归找 *.md, 也接受 *?[ 通配符)")
    lt.add_argument("--profile", choices=["atom", "article", "blog"],
                    help="不传则按路径段推断 (.hx-info.md -> atom, blog/ -> blog, 其余 article)")
    lt.add_argument("--allow-table", action="store_true", help="放开 article 的禁表格规则")
    lt.add_argument("--soft", action="store_true", help="只报告, 总是 exit 0")
    lt.add_argument("--json", action="store_true")
    lt.set_defaults(func=cmd_lint)

    ln = sub.add_parser("learn", help="把一条表达记进语料库")
    g = ln.add_mutually_exclusive_group(required=True)
    g.add_argument("--bad", help="坏表达原文")
    g.add_argument("--good", help="好表达原文")
    g.add_argument("--meme", help="梗 / 热词 (只用作者确认过的; 机器编不出来, 编错会变成新的 AI 味)")
    ln.add_argument("--why", help="为什么好/为什么坏; 梗填「什么场合用」")
    ln.add_argument("--pattern", help="可选正则; 给了就会变成 lint 的 E 级规则")
    ln.add_argument("--source", help="出处 (文件或链接); 梗必填 (从哪听来的)")
    ln.set_defaults(func=cmd_learn)

    fx = sub.add_parser("fix", help="把中文正文里被手工折行的段落合并为一行")
    fx.add_argument("paths", nargs="+",
                    help="文件或目录 (目录递归找 *.md, 也接受 *?[ 通配符)")
    fx.add_argument("--write", action="store_true", help="落盘 (默认 dry-run)")
    fx.set_defaults(func=cmd_fix)

    sp = sub.add_parser("samples", help="打印语料库")
    sp.add_argument("--json", action="store_true")
    sp.set_defaults(func=cmd_samples)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
