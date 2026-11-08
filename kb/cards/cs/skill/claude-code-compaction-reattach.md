---
id: cs/skill/claude-code-compaction-reattach
concepts: [Skill生命周期]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L54, raw/2026-11-08-skill/material.md#L57]
action: check
---
Claude Code 自动压缩后, 每个 skill 只重新附上最后一次调用的前 5000 token, 所有 skill 合计 25000 token, 旧 skill 可能整个被丢
> 运行期生命周期 (Claude Code)
> 自动压缩后, 每个 skill 只重新附上最近一次调用的前 5000 token, 所有 skill 合计 25000 token, 从最近调用的开始填, 旧 skill 可能整个被丢
动作:
- [ ] 关键指令落在正文前 5000 token 内
- [ ] 压缩后 skill 指令失效时, 检查它是否超出所有 skill 合计 25000 token 而被丢
