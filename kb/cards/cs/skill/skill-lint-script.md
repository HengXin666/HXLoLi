---
id: cs/skill/skill-lint-script
concepts: [Skill结构, Skill元数据]
type: cmd
tier: B
as_of: 2026-11-08
src: [raw/2026-11-08-skill/check/skill_lint.py#L2-L6, raw/2026-11-08-skill/check/skill_lint.py#L65-L66, raw/2026-11-08-skill/check/skill_lint.py#L89-L90]
action: cmd
tested: 2026-11-09 本地 Python 3.12, 对 check/AI-Animation-Skill 报 [E] name-dir 与 [W] abs-path, 退出码 1
---
skill_lint.py 体检一个 skill 目录, 有 E 级命中时退出码为 1
> 体检一个 skill 目录. 只用标准库.
> 退出码: 有 E 级命中时为 1, 否则 0.
> hits.append(("E", "name-dir", f"name={name!r} 与目录名 {d.name!r} 不一致 (规范要求一致)"))
> hits.append(("W", "abs-path", f"正文写死绝对路径 {m}: 换一台机器就失效"))
动作:
- `python3 kb/raw/2026-11-08-skill/check/skill_lint.py <skill-dir>`
- [ ] [E] name-dir: 让 name 与目录名一致, 两者都要满足小写字母数字加连字符
- [ ] [W] abs-path: 正文里的 /home/... 绝对路径改成相对路径或参数
