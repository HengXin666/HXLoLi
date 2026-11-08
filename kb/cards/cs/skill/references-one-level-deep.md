---
id: cs/skill/references-one-level-deep
concepts: [Skill正文, Skill结构]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L50]
action: check
---
引用保持离 SKILL.md 只深一层, 嵌套引用时 Claude 可能用 `head -100` 预览而读不全
> 引用保持离 SKILL.md 只深一层; 嵌套引用时 Claude 可能用 `head -100` 预览而读不全
动作:
- [ ] 所有引用文件都直接由 SKILL.md 链接, 没有文件引文件
