# Agent Note: 校验器的无依赖回落不得静默通过非法 frontmatter

Status: implemented

Decision-ID: validator-fallback-must-not-pass-invalid-frontmatter


## Code

- `.agents/skills/hx-make-skill/scripts/validate_skill.py`

## Problem

`hx-make-skill` 的 `description` 明确承诺能 `review why a skill is not being loaded`。
但它**遗漏了最凶的一类加载失败**: frontmatter 解析不了 → DSH 的 `dsh-skill-filesystem`
只打一行 `logger.warn` 就把**整个 skill 丢弃**, 不报错、不降级、不重试。调用侧看到的
现象是 `skill "<name>" is unknown or no longer available`, 而目录、正文、引用全都在,
同会话的其它 skill 也照常可用  于是它被误判成"模型叫错名字"或"目录没被扫描"。

实测事故 (2026-09-26): 本仓 `hx-web-reverse` 因 `description` 是裸标量却含 `": "`
(`… 协议化: 需要摸清全部链路 …`) 而被静默丢弃; 同一时刻的 catalog 里有其它 5 个 skill,
唯独没有它。取证方法: 解压 `~/.dsh/sessions/**/session*.jsonl.zstd`, 定位该轮系统提示。

**更要紧的是校验器本身有洞**: `_mini_yaml` 是 PyYAML 缺失时的回落, 它 `partition(":")`
后 `strip` 就收下, 于是把上述非法值**解析成功**。而 skill 文档里写的验收命令正是
`uv run scripts/validate_skill.py <dir>`  该 runner 里**没有 PyYAML**
(实测 `ModuleNotFoundError: No module named 'yaml'`), 即这条命令走的**默认就是回落路径**。
实测: 真实事故样本得到 `PASS hx-web-reverse (0 errors, 2 warnings)`。

危险之处不是"漏报一个错", 而是**把回落的宽松度当成了判定标准**: PyYAML 在位时同一份
输入报 `mapping values are not allowed here`, 不在位时报 PASS。判据随环境漂移,
而漂移的方向恰好是"放行不可加载的 skill"。

## Decision

让回落解析器与 PyYAML 在**这一条**上严格一致, 而不是保持宽松:

1. 新增 `_MiniYamlError`; `_mini_yaml` 遇到**未加引号且含 `": "`** 的标量值时抛它。
   含 `": "` 的裸标量在 YAML 里就是 compact mapping 嵌套, 是语法错误, 不是风格问题。
2. `_frontmatter` 捕获该异常, 转成已有的 `frontmatter is not valid YAML: …` 错误返回
   (与 PyYAML 分支同一出口, 因此 `validate()` 把它算 ERROR, 退出码 1)。
3. `SKILL.md` 的"常见失败"从两类扩到三类, 第三类写明"整个 skill 被丢弃"及其现象,
   并给出判据速记 (值含 `": "` 就加引号 / 用折叠标量).
4. `references/spec-checklist.md` 的"实现要点"补上该回落的严格性及理由.

## Alternatives considered

- **什么都不做**: 最强理由是"这类错误迟早有人肉眼发现, 且 PyYAML 装了就没事"。
  否决: PyYAML 在本 skill 的默认 runner 里恰恰**不在**, 所以这不是偶发路径而是默认路径;
  且失败是静默的 (校验器发 PASS、DSH 只 warn), 没有任何机制会提醒人去发现它。
- **给 `scripts/` 补 `pyproject.toml`, 把 PyYAML 装成依赖**: 最强理由是"根治 
  让判定只有一个来源, 回落到不了就无需讨论严格性"。
  否决: 本 skill 的明确卖点之一是 `无第三方依赖` (spec-checklist 原文), 装依赖会破坏它
  在离线/裸环境可用的性质; 且回落代码仍要存在 (import 失败时), 仍需被修。
- **只修文档 (写清判据), 不改脚本**: 否决。文档靠人读, 而本次事故正是"没人读文档"造成的;
  校验器的价值恰在于不需人读。
- **让 `_mini_yaml` 一律抛错, 彻底弃用回落**: 否决。那样每一份合法 frontmatter 在无 PyYAML
  环境下都会失败, 把"漏报"换成"全拒", 是更差的判据。

## Consequences

- 无 PyYAML 路径实测三臂: 病灶样本 → `FAIL`/`exit=1` 且指出具体字段; 双引号写法 → `PASS`;
  折叠标量 `>-` → `PASS`。即抓住病灶且对两种合法写法零假阳性。
- 回归: HX-Jungle 13 个 skill + HX-Memory 3 个 skill 共 16 个, 全部仍 `PASS`, 无新增 ERROR。
- 有 PyYAML 时行为完全不变 (走的仍是原分支)。
- `hx-make-skill` 的"排查 skill 加载不了"从此覆盖三类失败, 与它 description 里的承诺对齐。
- 已知局限: 该门禁需**主动运行**; HX-Jungle 侧已把它接进 pre-commit
  (`.agents/tools/skill_gate.py`), 但本仓尚无同等自动门禁。是否在本仓也接, 留待单独决定。
