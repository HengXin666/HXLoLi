---
id: cs/skill/vercel-skill-never-invoked
concepts: [Skill触发, Skill评测]
type: pitfall
tier: B
as_of: 2026-11-08
src: [raw/2026-11-08-skill/material.md#L88]
action: check
---
据 Vercel, 给 Next.js 16 文档做成 skill, 56% 的评测用例里 skill 从未被调用, 通过率 53% 与无文档基线相同
> 据 Vercel: 给 Next.js 16 文档做成 skill, 56% 的评测用例里 skill 从未被调用, 通过率 53% 与无文档基线相同
动作:
- [ ] 文档类 skill 上线前先测它在用例里有没有被调用
- [ ] 对比无文档基线的通过率
