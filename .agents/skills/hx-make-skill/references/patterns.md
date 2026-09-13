# 编写模式与反模式 (patterns)

本文件的每个模式都标注了来源等级:

- **[规范]** 公开标准强制, 违反会被拒
- **[官方]** 厂商文档明确建议
- **[共识]** 多个独立来源一致

---

## 一、description 是唯一真正重要的字段 [官方]

`name` 和 `description` 是模型**唯一**用来判断"要不要加载这个 skill"的输入. body 在触发之前完全不可见.

### 构造公式

```
[做什么: 具体能力清单, 含关键名词] + [什么时候用: 触发场景 + 用户可能说的原话]
```

**好**:

```yaml
description: Extracts text and tables from PDF files, fills PDF forms, and merges multiple PDFs. Use when working with PDF documents or when the user mentions PDFs, forms, or document extraction.
```

**坏**:

```yaml
description: Helps with PDFs.
```

### 反模式 A: 把触发条件写在 body 的 "When to Use" 段 [官方]

这是新手最自然的写法, 也是最致命的:

```markdown
---
name: my-skill
description: 帮我处理文档
---
# My Skill
## When to Use This Skill     # <- 永远不会被读到
当用户需要处理 docx 时使用...
```

模型只看到 `description: 帮我处理文档`, 不知道什么时候该用, **这个 skill 永远不会被触发**.

### 反模式 B: description 只写"做什么", 不写"什么时候用"

同样是静默失效 —— 不是被拒绝, 而是从不触发. 这类错误没有报错信息, 只能靠校验器兜住.

---

## 二、"简洁"的判据不是字数, 而是提问 [官方]

不要问"这段是不是太长", 要问:

> 模型是否**真的需要**这段解释? 这段的 token 成本**是否值得**?

官方把上下文窗口称为 **"公共资源" (a public good)** —— 它与 system prompt、对话历史、其他 skill 的元数据、用户的真实请求共享. **默认假设是"模型已经足够聪明"**, 只补充它不知道的东西.

**能被强模型自行推导出来的解释, 其 token 成本是纯亏损.**

这就是为什么"10 万字压缩到 1 万字"仍然是错的: 它没有解决"这些字是否都该在 L2"的问题.

---

## 三、自由度匹配 (degrees of freedom) [官方]

不同内容应该有不同的具体程度. 打个比方: **窄桥加悬崖需要护栏 (低自由度), 开阔原野允许多条路 (高自由度).**

| 自由度 | 形式 | 用在哪 |
|---|---|---|
| 高 | 纯文本指令 | 多种做法都行、依赖上下文判断、靠启发式 |
| 中 | 伪代码 / 带参数的脚本 | 有偏好做法、允许一定变化、配置影响行为 |
| 低 | 具体脚本、极少参数 | 操作脆弱易错、必须一致、必须按固定顺序 |

**这条直接决定"什么该放进 `scripts/`"**: 一个脆弱的多步流程应该写成低自由度的可执行脚本; 而"多个方案怎么选"应该留在 SKILL.md 里当高自由度指引.

---

## 四、三类 bundle 的边界 [官方]

| 目录 | 定义 | 何时建 | 是否进上下文 |
|---|---|---|---|
| `scripts/` | 可执行代码 | 同一段代码被反复重写, 或需要确定性可靠性 | 否 (可执行而不必读入) |
| `references/` | 供查阅的文档 | 模型工作时需要查 schema、API 文档、政策 | 是 (按需) |
| `assets/` | 产出用资源 | 模板、图标、字体、boilerplate | 否 |

`references/` 与 `assets/` 的区别就是**是否进上下文** —— 最容易混淆的一点.

**拆分原则**: SKILL.md 只留核心流程与选择指引; 变体细节、schema、长示例移到 `references/`.

**如果是多个互斥或很少同时使用的场景, 分开成多个 reference 文件能真正降低单次 token 消耗** —— 因为模型只会加载它当下需要的那一个.

---

## 五、正文语气: 祈使句 [共识]

写祈使句, 直接对模型下指令:

```
好: When the user asks for X, always do Y first.
坏: The agent should consider doing Y.
```

---

## 六、怎么证明 skill 真的有用: forward-testing [官方]

这是整份材料里方法论价值最高的一段, 也是最少人做的.

**核心要求**: 让被测 agent **不知道**自己在被测, 把它当作一个接到任务的普通 agent.

```
正确: Use $skill-x at /path/to/skill-x to solve problem y
错误: Review the skill at /path/to/skill-x; pretend a user asks you to...
```

**污染控制清单**:

- 用 fresh threads 做独立验证
- 传**原始 artifact**, 不传自己的结论
- 不要给预期答案、怀疑的 bug、打算怎么修
- 每轮迭代后从源 artifact 重建上下文
- 清理上一轮留下的产物, 避免跨轮污染
- 只给最少的任务局部上下文

**判定规则**: 如果只有在被测 agent 看到泄漏的上下文时才能成功, 那么要么收紧 skill, 要么收紧测试设置 —— 不能相信这个结果.

**为什么重要**: 它给出了"skill 变好了"的**可证伪判据**. 只靠"分数涨了"而不控制信息泄漏, 无法区分"skill 有效"和"测试者已经知道答案".

---

## 七、命名建议 [官方]

- 全小写 + 连字符; 把用户给的标题规范化 (如 "Plan Mode" -> `plan-mode`)
- 少于 64 字符
- 优先用**动词开头**的短语描述动作
- 需要时用工具名做前缀提高可读性 (如 `gh-address-comments`)
- **skill 目录名必须与 `name` 完全一致**
