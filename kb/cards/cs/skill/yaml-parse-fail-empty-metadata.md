---
id: cs/skill/yaml-parse-fail-empty-metadata
concepts: [Skill元数据, Skill触发, Skill宿主]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L21]
action: check
---
Claude Code 在 YAML 解析失败时仍加载正文但元数据为空, `/name` 能用而 Claude 无法按 description 匹配
> Claude Code 在 YAML 解析失败时仍加载正文但元数据为空, `/name` 能用而 Claude 无法按 description 匹配
动作:
- [ ] `/name` 能用但不自动触发时, 先检查 frontmatter 的 YAML 能否解析
