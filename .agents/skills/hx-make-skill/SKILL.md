---
name: hx-make-skill
description: Write, restructure, or audit an Agent Skill (a SKILL.md directory bundle) so it follows the public Agent Skills specification instead of guessing. Use when the user wants to create a new skill, turn a workflow or conversation into a reusable skill, split an oversized SKILL.md into progressive-disclosure layers, fix a skill that never triggers, review why a skill is not being loaded, or validate a skill directory before publishing. Covers frontmatter rules, the L1/L2/L3 context budget, description-based triggering, reference validation, the mandatory prose rules for every artifact a skill emits, and the code-quality limits its scripts must respect.
license: MIT
disable-model-invocation: true
metadata:
  author: Heng_Xin
  version: "2.0"
---

# hx-make-skill

把一次对话、一套流程或一份草稿变成**符合规范**的 Agent Skill. 产出的是一个目录: `SKILL.md` 加可选的 `references/` `scripts/` `assets/`. 新建或改写的 skill 必须同时满足两组要求: 规范形态 (本文) 与**产物质量** (文本规范、代码质量, 见下). 两组都有可执行门禁, 不是纸面约定

## 三条不可违反的原则

1. **规范优先, 不要即兴发挥.** 字段与约束见 `references/spec-checklist.md`. 拿不准就查, 不要猜
2. **超长不是压缩问题, 是拆分问题.** 有文件系统时, skill 能携带的上下文总量基本无上限  前提是按"是否总是需要"把内容分到 L2 (SKILL.md) 和 L3 (references/scripts/assets). 把 15 万 token 压到 1.5 万 token 仍是错的, 因为它没回答"这些字该不该在 L2"
3. **触发信息只能写一处.** `when to use` 全部写在 frontmatter 的 `description` 里. 正文**禁止**出现 "When to Use" 之类的段  那段永远不会被读到, 因为 body 只在触发之后才加载

## 四条硬约定 (写任何 skill 都要遵守)

### 一、索引写**裸路径 + 一句描述**, 不要写链接

给 AI 读的索引不是给人点的. 写成 `[references/x.md](references/x.md)` 除了把路径重复两遍, **不产生任何信息**, 却要付两份 token

```
坏: - [references/spec.md](references/spec.md)  字段约束. 写 frontmatter 前读它
坏: - 见 references/spec.md / references/patterns.md / references/anti.md
好: - `references/spec.md`: 写 frontmatter 前, 查字段约束与预算表
     `references/patterns.md`: 想让 skill 真的被触发时
```

第二行「坏」是另一种病: **平铺一串路径没有描述**. 模型看不出哪个该读, 等于没写. **每个路径后面必须跟一句「这是什么 / 什么时候读」**  这是索引的唯一价值

### 二、一个步骤 = 一个文件夹 = `index.md` + `impl/`

流程型 skill 用**模板方法模式**组织, 不要把所有步骤平铺在 `references/` 里

```
steps/
├── 1-intake/
│   └── index.md          契约: 只做这一件事 / 产物 / 过关条件
├── 2-collect/
│   ├── index.md          契约
│   └── impl/             可插拔实现: 这一步的多种做法各一个文件
│       ├── article.md
│       └── video.md
└── ...
shared/                  跨步骤能力 (不属于任何单步的)
entries/                 独立入口 (不重叠于主流程的)
```

- `index.md` 是**契约**, 写清「这一步只做什么、产物是什么、什么条件算过」
- `impl/` 是**实现**, 按场景/类型分文件. 加一种新做法 = 加一个文件, **契约不用改**
- SKILL.md 只列**步骤表** (谁 / 关心什么 / 契约在哪), 不写具体做法

这样扩展时改动面永远是「加一个文件」, 而不是「改索引 + 改正文 + 改引用」

### 三、产物文本必须过标点门禁

skill 的正文是给模型读的, 它**写出来的**每一份文本 (SKILL.md、references/、步骤契约、模板、以及该 skill 运行时生成的文档) 都要满足同一套文本规范

| 规则 | 要求 |
|---|---|
| 标点形态 | 只用英文标点, 全角 `，。？！：；（）“”` 一律判违规 |
| 标点后空格 | 标点后接内容时补**一个**空格 |
| 句末 | 正文行末尾**只允许** `?` |
| 折行 | 禁止把一句话拆到两行 |

完整判据 (P1..P5)、自动修复与豁免边界见 `references/prose-rules.md`. 门禁

```bash
uv run scripts/prose_rules.py --check <skill-dir>
```

新建 skill 时**必须把这条要求写进它自己的 SKILL.md** 它的产物归它管, 不能只靠上游记得. 做法是让它引用同一份 `references/prose-rules.md` 与同一条命令, 不要复制规则正文

### 四、代码质量是硬上限

skill 若带 `scripts/`, 这些脚本按代码算, 不按文档算. 上限与禁止项见 `references/code-quality.md`

- 一个目录内文件数 **不超过 6**
- 单文件 **不超过 300 行**
- **禁止 `.js` / `.mjs`**: 源码一律 `.ts`; 需要转译产出时把产出单独放进一个目录 (默认 `dist/`), 不要和源码同层
- **注释只说明做什么**: 禁止讲解来龙去脉、历史变更、踩坑过程; 一段连续注释不超过 10 行

门禁

```bash
uv run scripts/check_layout.py <skill-dir>
```

## 预算表 (必须记住的三个数字)

| 级别 | 内容 | 何时进上下文 | 预算 |
|---|---|---|---|
| L1 | `name` + `description` | 所有 skill 启动时预载 | 约 100 词 |
| L2 | `SKILL.md` body | 触发时整篇读入 | **少于 500 行 / 5000 token** |
| L3 | `references/` `scripts/` `assets/` | 按需 | 基本无上限 |

L2 超过 500 行时, **不要删减, 要外移**: 把"只在特定场景才需要"的内容搬进 `references/`, 并在 SKILL.md 里用相对路径引用它、说明什么时候去读

## 工作流

### 第一步: 先问清楚, 不要急着写

写之前必须先拿到**具体用例**, 而不是抽象描述. 至少问清楚

- 用户会说什么话才会用到它? (这句话直接决定 `description`)
- 典型任务长什么样? 给一个真实例子
- 有没有需要反复重写的代码? (-> `scripts/`)是否有要查的资料? (-> `references/`)产出里要复用的文件? (-> `assets/`)

信息不足时**一次只问一个问题**, 并附上推荐答案. 不要批量抛问卷

### 第二步: 先写 L3, 再写 L2

先落 `scripts/` `references/` `assets/`, 最后才写 `SKILL.md`. 理由: SKILL.md 的职责是**索引和指路**, 得先知道有哪些东西才写得出来

如果 `scripts/` 里有脚本, **必须真的跑一遍**确认它能工作, 不能只写不验

### 第三步: 写 frontmatter

```yaml
---
name: kebab-case-must-match-directory-name
description: [做什么: 能力清单 + 关键名词] + [什么时候用: 场景 + 用户原话]
disable-model-invocation: false # 依据用户是否需主动触发
license: MIT # 可选
metadata: # 可选
  author: Heng_Xin
  version: "1.0"
---
```

`description` 是**唯一**的触发机制. 长度上限 1024 字符, 用它把"做什么"和"什么时候用"都讲清楚. 写法和反例见 `references/patterns.md` 第一节

### 第四步: 写 body

- 用**祈使句**直接对模型下指令, 不要写 "the agent should..."
- 每写一段, 先问自己: "模型真的需要这段解释吗? 这段的 token 成本值得吗?" **默认假设是模型已经足够聪明**  能被它自行推导的内容是纯亏损
- 正文里**每个** `references/` 与 `scripts/` 下的文件都要被引用, 并说明**什么时候**去读
- 信息不能在 SKILL.md 和 `references/` 之间重复, 只能存一份
- 把硬约定三与硬约定四写进新 skill 的 SKILL.md: 指明它的产物受那两组规则约束, 并给出它自己那条门禁命令

### 第五步: 校验

```bash
uv run scripts/validate_skill.py <skill-dir> # 规范形态
uv run scripts/prose_rules.py --check <skill-dir> # 产物文本
uv run scripts/check_layout.py <skill-dir> # 代码质量上限
```

三条都退出码 0 才算通过. 有 ERROR 必须修; WARN 建议修. 每条检查的规范依据见 `references/spec-checklist.md`. `scripts/validate_skill.py` 覆盖命名、description 触发词、500 行预算、正文里的 "When to Use" 段、禁止的杂项文档文件、L3 文件是否被引用、引用深度是否超过一层. `scripts/prose_rules.py` 覆盖全角残留、标点后空格、句末标点、折行. `scripts/check_layout.py` 覆盖目录文件数、单文件行数、`.js`/`.mjs` 禁令

### 第六步: forward-test (复杂 skill 必做)

不要只靠"看起来好了". 开一个**新鲜的、不知道自己在被测的** agent 去用这个 skill

```
正确: Use $skill-name at <path> to solve <真实任务>
错误: 请审查 <path> 这个 skill, 假装有用户来问你...
```

如果只有在被测 agent 看到泄漏上下文时才能成功, 那这个 skill 或这个测试设置就还不成立. 完整清单见 `references/patterns.md` 第六节

## 常见失败: skill 不触发 / 加载不了
前两类是"没被触发", 第三类是"根本不存在" **症状一模一样, 根因完全不同**, 排查时先分清

1. `description` 只写了"做什么", 没写"什么时候用". 模型没有任何依据判断该不该加载
2. 触发信息写在 body 的 `## When to Use` 段里. body 在触发前不可见, 等于没写
3. **frontmatter 解析失败, 整个 skill 被丢弃.** 典型判据: `description` 是裸标量却含 `: ` (如 `description: 工作台  协议化: 需要摸清链路`). YAML 视其为 compact mapping 嵌套而报错, DSH 的 loader 只打一行 `logger.warn` 就跳过该 skill  **不报错、不降级、不重试**. 现象: 调用时得到 `skill "<name>" is unknown or no longer available`, 而目录与正文都在, 别的 skill 照常可用

三类都是**静默失效**  不会报错, skill 只是从不出现. 所以必须用校验器兜住, 不能靠肉眼

**判据速记**: frontmatter 的值只要含 `:` + 空格, 就给整个值加引号
(`description: "a: b"`); 或用折叠标量 `>-`. 单引号/双引号/折叠标量三种写法都合法, 裸标量不行

其它反模式见 `references/patterns.md`

## 落地位置

| 场景 | 路径 | 扫描优先级 |
|---|---|---|
| 项目级 | `<projectRoot>/.dsh/skills` 或 `<projectRoot>/.agents/skills` | 最高 |
| 自定义 | `Config.customSkillDirs` | 中 |
| 用户级 | `~/.dsh/skills` 或 `~/.agents/skills` | 最低 |

`<name>/SKILL.md` 与 `<name>.md` 两种形态都能被发现; **嵌套的 `**/SKILL.md` 不会被发现**  必须放在扫描根的一级目录下

## 参考文件

- `references/spec-checklist.md`  什么时候读: 写 frontmatter 前, 查字段约束、预算表与每条校验规则的规范依据
- `references/patterns.md`  什么时候读: 想让 skill 真的被触发时; 或要挑一个流程型/知识型结构时
- `references/prose-rules.md`  什么时候读: 写或改任何文本产物的前后; 要判一条标点/折行是否违规时
- `references/code-quality.md`  什么时候读: skill 带 `scripts/` 时; 要拆目录或决定注释写到什么程度时
- `references/legacy-audit.md`  什么时候读: 要改造一个**存量** skill 时; 先看它已经欠了什么, 以及哪几类改动不能顺手做

## 脚本

- `scripts/validate_skill.py`  规范形态校验器, 无第三方依赖. 退出码 0 为通过
- `scripts/prose_rules.py`  文本规范门禁与自动修复. 退出码 0 为通过
- `scripts/check_layout.py`  目录文件数与单文件行数上限, 以及 `.js`/`.mjs` 禁令
- `scripts/checks.py`  上面校验器的检查主体, 被 `validate_skill.py` 引用. 内部件, 不直接运行
- `scripts/frontmatter.py`  frontmatter 读取与无 PyYAML 回落解析. 内部件, 不直接运行
- `scripts/prose_core.py`  文本规范判定与修复共用的词表与行角色分析. 内部件, 不直接运行
