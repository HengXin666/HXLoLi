---
id: cs/skill/codex-skill-dirs
concepts: [Skill宿主]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L64]
action: check
---
Codex 从当前目录到仓库根逐级扫 `.agents/skills`, 另有 `$HOME/.agents/skills`、`/etc/codex/skills`, 同名不合并
> Codex: 从当前目录到仓库根逐级扫 `.agents/skills`, 另有 `$HOME/.agents/skills`、`/etc/codex/skills`、内置 system skill; 同名不合并
动作:
- [ ] Codex 的 skill 放在 `.agents/skills` 或 `$HOME/.agents/skills`
- [ ] 各位置没有同名 skill
