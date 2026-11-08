---
id: cs/skill/eval-clean-context
concepts: [Skill评测]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L81]
action: check
---
每次运行要从干净上下文开始, 写 skill 时残留的上下文会掩盖指令缺口
> 每次运行要从干净上下文开始, 写 skill 时残留的上下文会掩盖指令缺口
动作:
- [ ] 每次评测运行都是新会话, 没有写 skill 时的上下文
