---
id: cs/skill/three-tier-loading
concepts: [Skill正文]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L8]
action: check
---
三层加载: 元数据启动时为所有 skill 载入, 正文在激活时整份载入, 资源文件只在需要时读
> 三层加载: 元数据 (name + description, 约 100 token) 启动时为所有 skill 载入; 正文 (建议 < 5000 token) 在激活时整份载入; 资源文件只在需要时读
动作:
- [ ] 元数据 (name + description) 控制在约 100 token
- [ ] 正文建议 < 5000 token, 其余内容放资源文件按需读
