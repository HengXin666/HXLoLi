---
id: cs/skill/codex-listing-budget
concepts: [Skill触发, Skill宿主]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L31]
action: check
---
Codex 初始 skill 列表默认最多占上下文窗口 2%, 窗口未知时 8000 字符, 超出时先缩短描述, 仍超就省略部分 skill 并告警
> Codex: 初始 skill 列表默认最多占上下文窗口 2%, 窗口未知时 8000 字符; 超出时先缩短描述, 仍超就省略部分 skill 并告警
动作:
- [ ] Codex 出现 skill 被省略的告警时, 精简描述或减少 skill 数
