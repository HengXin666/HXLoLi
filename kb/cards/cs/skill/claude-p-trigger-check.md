---
id: cs/skill/claude-p-trigger-check
concepts: [Skill触发, Skill评测, Skill宿主]
type: cmd
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L73]
action: check
---
Claude Code 可用 `claude -p` 加 `--output-format json` 检查有无 `Skill` 工具调用来判定是否触发
> Claude Code 可用 `claude -p ... --output-format json` 检查有无 `Skill` 工具调用来判定是否触发
动作:
- [ ] 用 `claude -p ... --output-format json` 跑查询, 看输出里有无 `Skill` 工具调用
