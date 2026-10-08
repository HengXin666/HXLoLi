# 9 land — 站点认不认

## 只做这一件事

落地注册: 让站点真正认这份笔记. 跑全部交付闸门, 全绿才算完

## 做法

```bash
node scripts/generateAiDocsSidebar.js      # 新增/改名/移动目录后必跑
uv run .agents/skills/hx-note/scripts/cli/flow/hx_flow.py doctor --slug "<slug>"
```

## 实现

闸门清单 (以 doctor 实跑项数为准, 不要在这里写死数字)、产物落点约定、跨文章引用、知识库索引现状: `impl/gates.md`

## 过关条件

`doctor` 全绿. 未通过的项必须修完, 或在回复里写明为什么跳过
