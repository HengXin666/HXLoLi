---
id: cs/skill/one-step-tasks-not-triggered
concepts: [Skill触发]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L29]
action: check
---
agent 通常只为自己独立做不好的任务查 skill, 一步就能完成的请求即使描述完全匹配也可能不触发
> agent 通常只为自己独立做不好的任务查 skill; 一步就能完成的请求 (如 "read this PDF") 即使描述完全匹配也可能不触发
动作:
- [ ] 这个 skill 针对的不是一步就能完成的请求
- [ ] 触发评测里别拿一步就能完成的请求当正例
