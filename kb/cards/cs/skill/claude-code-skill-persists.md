---
id: cs/skill/claude-code-skill-persists
concepts: [Skill生命周期, Skill正文]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L54, raw/2026-11-08-skill/material.md#L56]
action: check
---
Claude Code 被调用的 skill 作为一条消息进入对话并跨轮常驻, 每一行都是逐轮成本, 要写成持续生效的指令
> 运行期生命周期 (Claude Code)
> 被调用的 skill 渲染后作为一条消息进入对话并跨轮常驻, 不会再读文件; 所以每一行都是逐轮成本, 要写成持续生效的指令而非一次性步骤
动作:
- [ ] 正文写成持续生效的指令而非一次性步骤
- [ ] 删掉不需要逐轮常驻的行
