---
id: cs/skill/claude-code-context-fork
concepts: [Skill生命周期, Skill宿主]
type: decide
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L54, raw/2026-11-08-skill/material.md#L59]
action: question
---
Claude Code 里 `context: fork` 让 skill 在子 agent 里跑, 子 agent 看不到对话历史
> 运行期生命周期 (Claude Code)
> `context: fork` 让 skill 在子 agent 里跑, 子 agent 看不到对话历史
动作:
- ? 这个 skill 要不要对话历史, 不要才用 `context: fork`
