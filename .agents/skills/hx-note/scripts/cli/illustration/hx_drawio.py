# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""把一份 JSON 结构图描述编译成 `.drawio.svg` (入口)。

既能在页面上直接显示, 也能在站点的 drawio 编辑器里继续手改。可编辑性来自根元素上
那个 `content` 属性 (HTML 转义后的 mxfile XML), 它必须与肉眼看到的图形逐个对应 
手写时这两份必然对不上 (图看着没问题, 一双击就变成空白画布), 所以固化成脚本。

spec 格式 (只支持分层左右布局, 够用且不会画崩):
{
  "title": "可选, 画在左上角",
  "columns": [{"title": "列标题", "nodes": [{"id": "a", "label": "节点文字", "accent": false}]}],
  "edges": [{"from": "a", "to": "b", "label": "可选"}]
}
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# scripts/ 与 scripts/cli/ 都上 path: 前者供 `lib.*`, 后者供各域包。
for _p in (Path(__file__).resolve().parents[1], Path(__file__).resolve().parents[2]):
    sys.path.insert(0, str(_p))

from illustration.drawio_render import build  # noqa: E402

DEMO = {
    "title": "示例: 三段流水线",
    "columns": [
        {"title": "输入", "nodes": [{"id": "a", "label": "素材"}]},
        {"title": "处理", "nodes": [{"id": "b", "label": "解析", "accent": True},
                                    {"id": "c", "label": "校验"}]},
        {"title": "输出", "nodes": [{"id": "d", "label": "产物"}]},
    ],
    "edges": [{"from": "a", "to": "b"}, {"from": "b", "to": "c", "label": "失败重试"},
              {"from": "c", "to": "d"}],
}


# Agent Notes: 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织
# 
def main(argv=None) -> int:
    # Agent Notes: 沉淀必须产出可复用物
    # 
    """Agent Notes
    .agents/notes/implemented/architecture/2026-09-26-skill-steps-as-template-method.md
    .agents/notes/implemented/architecture/2026-09-27-sediment-must-yield-reusable-artifacts.md
    """
    p = argparse.ArgumentParser(description="JSON 结构图 -> 可再编辑的 .drawio.svg")
    p.add_argument("spec", nargs="?", help="spec JSON 路径; 不给则需要 --demo")
    p.add_argument("-o", "--output", required=True, help="输出路径, 必须以 .drawio.svg 结尾")
    p.add_argument("--demo", action="store_true", help="用内置示例 spec")
    args = p.parse_args(argv)

    out = Path(args.output)
    if not out.name.endswith(".drawio.svg"):
        print("error: 输出文件名必须以 .drawio.svg 结尾, 否则站点不会当成可编辑图",
              file=sys.stderr)
        return 2

    if args.demo:
        spec = DEMO
    elif args.spec:
        spec = json.loads(Path(args.spec).read_text(encoding="utf-8"))
    else:
        print("error: 需要 spec 路径或 --demo", file=sys.stderr)
        return 2

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build(spec), encoding="utf-8")
    print(f"created: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
