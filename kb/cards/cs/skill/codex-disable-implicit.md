---
id: cs/skill/codex-disable-implicit
concepts: [Skill触发, Skill宿主]
type: decide
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L33]
action: question
---
Codex 在 `agents/openai.yaml` 里设 `policy.allow_implicit_invocation: false` 关闭隐式调用, 显式 `$skill` 仍可用
> Codex: `agents/openai.yaml` 里 `policy.allow_implicit_invocation: false` 关闭隐式调用, 显式 `$skill` 仍可用
动作:
- ? 这个 skill 在 Codex 里要不要关掉隐式调用, 只留显式 `$skill`
