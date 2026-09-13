---
name: hx-make-skill
description: Write, restructure, or audit an Agent Skill (a SKILL.md directory bundle) so it follows the public Agent Skills specification instead of guessing. Use when the user wants to create a new skill, turn a workflow or conversation into a reusable skill, split an oversized SKILL.md into progressive-disclosure layers, fix a skill that never triggers, review why a skill is not being loaded, or validate a skill directory before publishing. Covers frontmatter rules, the L1/L2/L3 context budget, description-based triggering, and reference validation.
license: MIT
metadata:
  author: Heng_Xin
  version: "1.0"
---

# hx-make-skill

把一次对话、一套流程或一份草稿变成**符合规范**的 Agent Skill. 产出的是一个目录: `SKILL.md` 加可选的 `references/` `scripts/` `assets/`.

## 三条不可违反的原则

1. **规范优先, 不要即兴发挥.** 字段与约束见 [references/spec-checklist.md](references/spec-checklist.md). 拿不准就查, 不要猜.
2. **超长不是压缩问题, 是拆分问题.** 有文件系统时, skill 能携带的上下文总量基本无上限 —— 前提是按"是否总是需要"把内容分到 L2 (SKILL.md) 和 L3 (references/scripts/assets). 把 15 万 token 压到 1.5 万 token 仍是错的, 因为它没回答"这些字该不该在 L2".
3. **触发信息只能写一处.** `when to use` 全部写在 frontmatter 的 `description` 里. 正文**禁止**出现 "When to Use" 之类的段 —— 那段永远不会被读到, 因为 body 只在触发之后才加载.

## 预算表 (必须记住的三个数字)

| 级别 | 内容 | 何时进上下文 | 预算 |
|---|---|---|---|
| L1 | `name` + `description` | 所有 skill 启动时预载 | 约 100 词 |
| L2 | `SKILL.md` body | 触发时整篇读入 | **少于 500 行 / 5000 token** |
| L3 | `references/` `scripts/` `assets/` | 按需 | 基本无上限 |

L2 超过 500 行时, **不要删减, 要外移**: 把"只在特定场景才需要"的内容搬进 `references/`, 并在 SKILL.md 里用相对路径引用它、说明什么时候去读.

## 工作流

### 第一步: 先问清楚, 不要急着写

写之前必须先拿到**具体用例**, 而不是抽象描述. 至少问清楚:

- 用户会说什么话才会用到它? (这句话直接决定 `description`)
- 典型任务长什么样? 给一个真实例子.
- 有没有需要反复重写的代码? (-> `scripts/`)是否有要查的资料? (-> `references/`)产出里要复用的文件? (-> `assets/`)

信息不足时**一次只问一个问题**, 并附上推荐答案. 不要批量抛问卷.

### 第二步: 先写 L3, 再写 L2

先落 `scripts/` `references/` `assets/`, 最后才写 `SKILL.md`. 理由: SKILL.md 的职责是**索引和指路**, 得先知道有哪些东西才写得出来.

如果 `scripts/` 里有脚本, **必须真的跑一遍**确认它能工作, 不能只写不验.

### 第三步: 写 frontmatter

```yaml
---
name: kebab-case-must-match-directory-name
description: [做什么: 能力清单 + 关键名词] + [什么时候用: 场景 + 用户原话]
license: MIT            # 可选
metadata:               # 可选
  author: Heng_Xin
  version: "1.0"
---
```

`description` 是**唯一**的触发机制. 长度上限 1024 字符, 用它把"做什么"和"什么时候用"都讲清楚. 写法和反例见 [references/patterns.md](references/patterns.md) 第一节.

### 第四步: 写 body

- 用**祈使句**直接对模型下指令, 不要写 "the agent should...".
- 每写一段, 先问自己: "模型真的需要这段解释吗? 这段的 token 成本值得吗?" **默认假设是模型已经足够聪明** —— 能被它自行推导的内容是纯亏损.
- 正文里**每个** `references/` 与 `scripts/` 下的文件都要被引用, 并说明**什么时候**去读.
- 信息不能在 SKILL.md 和 `references/` 之间重复, 只能存一份.

### 第五步: 校验

```bash
uv run scripts/validate_skill.py <skill-dir>
```

退出码 0 表示通过. 有 ERROR 必须修; WARN 建议修. 每一条检查的规范依据见 [references/spec-checklist.md](references/spec-checklist.md).

脚本覆盖: 命名规则、description 长度与触发词、500 行预算、正文里的 "When to Use" 段、禁止的杂项文档文件、L3 文件是否被引用、引用深度是否超过一层.

### 第六步: forward-test (复杂 skill 必做)

不要只靠"看起来好了". 开一个**新鲜的、不知道自己在被测的** agent 去用这个 skill:

```
正确: Use $skill-name at <path> to solve <真实任务>
错误: 请审查 <path> 这个 skill, 假装有用户来问你...
```

如果只有在被测 agent 看到泄漏上下文时才能成功, 那这个 skill 或这个测试设置就还不成立. 完整清单见 [references/patterns.md](references/patterns.md) 第六节.

## 常见失败: skill 不触发

90% 的情况是下面两种:

1. `description` 只写了"做什么", 没写"什么时候用". 模型没有任何依据判断该不该加载.
2. 触发信息写在 body 的 `## When to Use` 段里. body 在触发前不可见, 等于没写.

两种都是**静默失效** —— 不会报错, skill 只是从不出现. 所以必须用校验器兜住, 不能靠肉眼. 其它反模式见 [references/patterns.md](references/patterns.md).

## 落地位置

| 场景 | 路径 | 扫描优先级 |
|---|---|---|
| 项目级 | `<projectRoot>/.dsh/skills` 或 `<projectRoot>/.agents/skills` | 最高 |
| 自定义 | `Config.customSkillDirs` | 中 |
| 用户级 | `~/.dsh/skills` 或 `~/.agents/skills` | 最低 |

`<name>/SKILL.md` 与 `<name>.md` 两种形态都能被发现; **嵌套的 `**/SKILL.md` 不会被发现** —— 必须放在扫描根的一级目录下.

## 参考文件

- [references/spec-checklist.md](references/spec-checklist.md) —— 字段约束、预算表、每条校验规则的标准依据. **写 frontmatter 前先读它.**
- [references/patterns.md](references/patterns.md) —— description 构造公式、反模式、自由度匹配、forward-testing 协议. **想让 skill 真的被触发时读它.**
- [scripts/validate_skill.py](scripts/validate_skill.py) —— 规范校验器, 无第三方依赖.
