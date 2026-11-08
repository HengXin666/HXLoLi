---
id: cs/skill/vercel-must-invoke-wording
concepts: [Skill触发]
type: pitfall
tier: B
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L90]
action: check
---
据 Vercel, "You MUST invoke the skill" 让 agent 先读文档、锚定文档写法、漏掉项目上下文, "先探索项目再调用" 结果更好
> 据 Vercel: 措辞改动导致结果大幅摆动: "You MUST invoke the skill" 让 agent 先读文档、锚定文档写法、漏掉项目上下文 (漏改 `next.config.ts`); "先探索项目再调用" 结果更好
动作:
- [ ] 调用指令写成 "先探索项目再调用", 不写 "You MUST invoke the skill"
- [ ] 改措辞后重跑评测
