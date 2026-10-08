# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""沉淀流水线的状态机 (入口)。

把一次沉淀切成 9 个阶段, 每个阶段只做一件事。边界必须由脚本强制, 否则模型会在
一个阶段里顺手把下一个阶段也做了  路径决策的上下文里混进闲聊, 写文章的上下文里
混进目录编号的推理。三条强制: `status` 每次只讲当前一个阶段; `done` 检查产物是否
落盘, 没落盘就拒绝推进; 阶段间传递全部走文件, 于是任何阶段都能在新鲜的 agent 里
从磁盘恢复, 不依赖对话历史。

暂存区: ai-docs/.hx-staging/<slug>/  (点开头, 不被 Docusaurus / sidebar / tag 索引 /
quality-gate 收录)。阶段 5 `place` 之后笔记产物搬进正式目录, 过程产物留在暂存区。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# scripts/ 上 path, 于是 `import flow.*` 能找到同级目录里的包。
# scripts/ 与 scripts/cli/ 都上 path: 前者供 `lib.*`, 后者供各域包。
for _p in (Path(__file__).resolve().parents[1], Path(__file__).resolve().parents[2]):
    sys.path.insert(0, str(_p))

from flow.flow_doctor import cmd_doctor  # noqa: E402
from flow.flow_place import cmd_brief, cmd_cover, cmd_place  # noqa: E402
from flow.flow_read import cmd_list, cmd_status  # noqa: E402
from flow.flow_steps import cmd_done, cmd_init  # noqa: E402


def main(argv=None) -> int:
    # Agent Notes: 沉淀必须产出可复用物
    # 
    """Agent Notes
    .agents/notes/implemented/architecture/2026-09-26-skill-steps-as-template-method.md
    .agents/notes/implemented/architecture/2026-09-27-sediment-must-yield-reusable-artifacts.md
    .agents/notes/implemented/architecture/2026-09-28-divergence-operators-gate.md
    .agents/notes/implemented/architecture/2026-09-28-human-review-must-actually-happen.md
    .agents/notes/implemented/architecture/2026-09-28-requirements-as-acceptance-clauses.md
    .agents/notes/implemented/feature/2026-09-27-repo-source-kind-and-scratch-isolation.md
    """
    p = argparse.ArgumentParser(description="ai-docs 沉淀流水线状态机")
    p.add_argument("--root", help="仓库根, 默认向上找含 ai-docs/ 与 docusaurus.config.ts 的目录")
    sub = p.add_subparsers(dest="cmd", required=True)

    i = sub.add_parser("init", help="开一条新流程")
    i.add_argument("--slug", help="暂存目录名; 不给则从 --title 推")
    i.add_argument("--title", help="拟定标题")
    i.add_argument("--source", help="素材 URL 或路径")
    i.add_argument("--kind", default="article",
                   choices=["video", "article", "custom", "local", "research", "repo"])
    i.add_argument("--force", action="store_true")
    i.set_defaults(func=cmd_init)

    # Agent Notes: 人类审核必须真的发生, 且问题必须被端到人类面前
    # 
    s = sub.add_parser("status", help="当前该做哪一件事")
    s.add_argument("--slug", required=True)
    s.add_argument("--json", action="store_true")
    s.set_defaults(func=cmd_status)

    # Agent Notes: 发散由算子驱动, 并由 done atom 硬闸门强制
    # 
    d = sub.add_parser("done", help="标记某阶段完成 (产物缺失会被拒)")
    d.add_argument("stage", help="阶段号或 key")
    d.add_argument("--slug", required=True)
    d.add_argument("--note", help="这一阶段的结论/取舍, 会写进 flow.json")
    d.add_argument("--force", action="store_true", help="绕过产物检查, 必须配 --note")
    d.set_defaults(func=cmd_done)

    # Agent Notes: 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织; 外部源码项目作为一等素材类型, 且探索一律隔离到临时目录
    # 
    # 
    pl = sub.add_parser("place", help="阶段 5: 建正式目录 + 初始化 index.md + 搬迁产物")
    pl.add_argument("--slug", required=True)
    pl.add_argument("--to", required=True, help="ai-docs 下的目标目录 (相对仓库根)")
    pl.add_argument("--title")
    pl.add_argument("--tag", action="append")
    pl.add_argument("--model")
    pl.add_argument("--force", action="store_true")
    pl.set_defaults(func=cmd_place)

    # Agent Notes: 需求必须落成可判定的验收条款, 且对齐是对话不是问卷
    # 
    cv = sub.add_parser("cover", help="需求覆盖: 每条验收条款被多少条知识点支撑")
    cv.add_argument("--slug", required=True)
    cv.set_defaults(func=cmd_cover)

    bf = sub.add_parser("brief", help="把待审的题、被审对象、作答方法一次打全 (给人看的)")
    bf.add_argument("--slug", required=True)
    bf.set_defaults(func=cmd_brief)

    dc = sub.add_parser("doctor", help="阶段 9: 跑全部交付闸门")
    dc.add_argument("--slug", required=True)
    dc.set_defaults(func=cmd_doctor)

    ls = sub.add_parser("list", help="列出暂存区里所有流程")
    ls.set_defaults(func=cmd_list)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
