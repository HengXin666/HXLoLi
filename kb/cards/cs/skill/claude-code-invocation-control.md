---
id: cs/skill/claude-code-invocation-control
concepts: [Skill触发, Skill宿主]
type: decide
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L32]
action: question
---
Claude Code: `disable-model-invocation: true` 时 description 不进上下文, 只能用户 `/name` 调用, `user-invocable: false` 时只有 Claude 能调
> Claude Code: `disable-model-invocation: true` 时 description 不进上下文, 只能用户 `/name` 调用; `user-invocable: false` 时只有 Claude 能调
动作:
- ? 这个 skill 只让用户 `/name` 调 (`disable-model-invocation: true`), 只让 Claude 调 (`user-invocable: false`), 还是都行
