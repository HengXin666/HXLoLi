---
id: cs/skill/claude-code-listing-budget
concepts: [Skill触发, Skill宿主]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L30]
action: check
---
Claude Code 列表整体预算为上下文窗口的 1%, 超出时先丢最少被调用的 skill 的描述, 名字始终保留
> 列表整体预算为上下文窗口的 1%, 超出时先丢最少被调用的 skill 的描述, 名字始终保留
动作:
- [ ] 装的 skill 很多时, 确认少用的 skill 描述没被丢掉
- [ ] 不用的 skill 卸掉, 给列表预算腾地方
