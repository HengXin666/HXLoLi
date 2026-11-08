---
id: cs/skill/nonspec-fields-hard-error
concepts: [Skill元数据, Skill宿主]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L20]
action: check
---
Claude Code 接受规范外字段, 上传 claude.ai、Skills API、`package_skill.py` 打包只接受规范内 6 个字段, 多出的硬报错
> Claude Code 接受规范外字段 (如 `disable-model-invocation` `context` `paths`); 上传 claude.ai、Skills API、`package_skill.py` 打包只接受规范内 6 个字段, 多出的字段直接硬报错 `Unexpected key(s) in SKILL.md frontmatter`
动作:
- [ ] 要上传 claude.ai、走 Skills API 或用 `package_skill.py` 打包时, frontmatter 只留规范内 6 个字段
- [ ] 见到 `Unexpected key(s) in SKILL.md frontmatter` 就去删规范外字段
