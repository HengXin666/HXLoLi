---
id: cs/skill/vercel-unused-skill-hurts
concepts: [Skill触发, Skill评测]
type: pitfall
tier: B
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L91]
action: check
---
据 Vercel, 未被使用的 skill 留在环境里, Test 指标反而从 63% 降到 58%
> 据 Vercel: 未被使用的 skill 留在环境里, Test 指标反而从 63% 降到 58%
动作:
- [ ] 评测里从未被调用的 skill 移出环境再测
