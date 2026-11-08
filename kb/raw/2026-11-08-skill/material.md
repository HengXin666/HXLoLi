# material: Agent Skill 编写最佳实践

收敛后的问题: 写一个给 Claude Code / Codex 等 agent 用的 SKILL.md 时, (1) description 怎么写才会被正确触发, (2) 正文与附属文件怎么分才不浪费上下文, (3) 渐进加载在各宿主里实际怎么实现, (4) 怎么验证这个 skill 真的有用. 时间点: 2026-11-08.

## 结构与渐进加载

- skill 是一个至少含 `SKILL.md` 的目录; 可选 `scripts/` `references/` `assets/` [S2 §Directory structure] [S6 §Build skills 目录树]
- 三层加载: 元数据 (name + description, 约 100 token) 启动时为所有 skill 载入; 正文 (建议 < 5000 token) 在激活时整份载入; 资源文件只在需要时读 [S2 §Progressive disclosure] [S1 §Runtime environment]
- 启动时只预载元数据; Claude 在 skill 相关时才读 SKILL.md, 其他文件按需读 [S1 §Concise is key]
- Codex 也是先给 name + description (外加文件路径), 决定使用时再读完整 SKILL.md [S6 §progressive disclosure 一段]
- 脚本通过 bash 运行, 内容不进上下文, 只有输出占 token [S1 §Runtime environment "Scripts run efficiently"]
- 参考文件、数据在被读之前不占上下文 [S1 §Runtime environment "No context penalty for large files"] [S5 §Add supporting files]

## frontmatter 与命名

- 规范必填 `name` 与 `description`; 可选 `license` `compatibility` `metadata` `allowed-tools` (实验性) [S2 §Frontmatter]
- `name`: 1 到 64 字符, 只含小写字母数字与连字符, 不以连字符开头结尾, 不含连续连字符, 必须与父目录名一致 [S2 §name field]; 不能含保留词 `anthropic` `claude`, 不能含 XML 标签 [S1 §Skill structure]
- `description`: 1 到 1024 字符, 非空, 写清做什么与什么时候用 [S2 §description field] [S1 §Skill structure]
- 命名建议动名词形式 (`processing-pdfs`), 避免 `helper` `utils` 这类空名 [S1 §Naming conventions]
- Claude Code 接受规范外字段 (如 `disable-model-invocation` `context` `paths`); 上传 claude.ai、Skills API、`package_skill.py` 打包只接受规范内 6 个字段, 多出的字段直接硬报错 `Unexpected key(s) in SKILL.md frontmatter` [S5 §Using skill frontmatter outside Claude Code]
- Claude Code 在 YAML 解析失败时仍加载正文但元数据为空, `/name` 能用而 Claude 无法按 description 匹配 [S5 §Skill not triggering]

## description 与触发

- description 是隐式触发的主要机制, 从可能 100+ 个 skill 里选 [S1 §Writing effective descriptions] [S3 §How skill triggering works] [S6 §How ChatGPT and Codex use skills]
- 写第三人称, 因为它被注入系统提示, 人称不一致会导致发现问题 [S1 §Writing effective descriptions]
- 用祈使句 "Use this skill when…", 聚焦用户意图, 可以写得 pushy, 把用户没点名领域的情形也列进去 [S3 §Writing effective descriptions]
- Codex: 隐式匹配依赖 description, 把关键用例与触发词放在前面, 以便描述被缩短时仍能匹配 [S6 §How ChatGPT and Codex use skills]
- agent 通常只为自己独立做不好的任务查 skill; 一步就能完成的请求 (如 "read this PDF") 即使描述完全匹配也可能不触发 [S3 §How skill triggering works]
- Claude Code: description 与 `when_to_use` 合计在列表中截断于 1536 字符 (可用 `skillListingMaxDescChars` 改); 列表整体预算为上下文窗口的 1%, 超出时先丢最少被调用的 skill 的描述, 名字始终保留 [S5 §Frontmatter reference] [S5 §Skill descriptions are cut short]
- Codex: 初始 skill 列表默认最多占上下文窗口 2%, 窗口未知时 8000 字符; 超出时先缩短描述, 仍超就省略部分 skill 并告警; `skills.max_context_tokens` 最多可设 10000 token [S6 §progressive disclosure 一段]
- Claude Code: `disable-model-invocation: true` 时 description 不进上下文, 只能用户 `/name` 调用; `user-invocable: false` 时只有 Claude 能调 [S5 §Control who invokes a skill]
- Codex: `agents/openai.yaml` 里 `policy.allow_implicit_invocation: false` 关闭隐式调用, 显式 `$skill` 仍可用 [S6 §Optional metadata]
- Claude Code 的 `paths` 字段可把自动激活限制到匹配的文件 [S5 §Frontmatter reference]

## 正文写法

- 默认假设模型已经很聪明, 只写它不知道的; 每段问 "这段配得上它的 token 吗" [S1 §Concise is key]
- 自由度按任务脆弱程度定: 多解任务给文字指引 (高自由), 有偏好模式给伪代码或带参数脚本 (中), 易错且要求一致的操作给精确脚本并写 "不要改命令" (低) [S1 §Set appropriate degrees of freedom]
- 不要并列给太多选项, 给一个默认加一个逃生口 [S1 §Avoid offering too many options]
- 术语全篇统一 [S1 §Use consistent terminology]; 避免时间敏感信息, 旧做法放进 "old patterns" 折叠段 [S1 §Avoid time-sensitive information]
- 复杂流程给可复制的 checklist; 质量关键任务加 "跑校验 -> 修 -> 再跑" 的反馈环 [S1 §Use workflows for complex tasks] [S1 §Implement feedback loops]
- 脚本要自己处理错误而不是甩给模型, 常量要写理由 (不要 voodoo constants); 写清是 "运行脚本" 还是 "读脚本当参考" [S1 §Solve, don't defer] [S1 §Provide utility scripts]
- 规范对正文没有格式限制, 推荐写步骤、输入输出示例、边界情况 [S2 §Body content]
- 推理型指令 ("做 X 因为 Y 容易导致 Z") 比 ALWAYS/NEVER 式硬指令更可靠; 规则越加通过率越平, 可以试着删指令 [S4 §Iterating on the skill]

## 正文长度与引用深度

- SKILL.md 正文 < 500 行, 接近时拆文件 [S1 §Progressive disclosure patterns] [S2 §Progressive disclosure] [S5 §Add supporting files]
- 引用保持离 SKILL.md 只深一层; 嵌套引用时 Claude 可能用 `head -100` 预览而读不全 [S1 §Avoid deeply nested references] [S2 §File references]
- 超过 100 行的参考文件顶部放目录 [S1 §Structure longer reference files with a table of contents]
- 按领域拆参考文件 (`reference/finance.md` `reference/sales.md`), 让无关领域零占用 [S1 §Pattern 2: Domain-specific organization]

## 运行期生命周期 (Claude Code)

- 被调用的 skill 渲染后作为一条消息进入对话并跨轮常驻, 不会再读文件; 所以每一行都是逐轮成本, 要写成持续生效的指令而非一次性步骤 [S5 §Skill content lifecycle] [S5 §Types of skill content]
- 自动压缩后, 每个 skill 只重新附上最近一次调用的前 5000 token, 所有 skill 合计 25000 token, 从最近调用的开始填, 旧 skill 可能整个被丢 [S5 §Skill content lifecycle]
- 重要指令放在 SKILL.md 靠前处; 每次都必须成立的规则移进 hook [S5 §Claude stops following a skill]
- `context: fork` 让 skill 在子 agent 里跑, 子 agent 看不到对话历史 [S5 §Run skills in a subagent]

## 宿主目录

- Claude Code: 个人 `~/.claude/skills/<name>/SKILL.md`, 项目 `.claude/skills/<name>/SKILL.md`, 嵌套子目录、`--add-dir`、插件、企业托管 [S5 §Choose where skills load]
- Codex: 从当前目录到仓库根逐级扫 `.agents/skills`, 另有 `$HOME/.agents/skills`、`/etc/codex/skills`、内置 system skill; 同名不合并 [S6 §Where Codex loads local skills]
- 二者都遵循 agentskills.io 开放标准 [S5 开头] [S6 开头]

## 验证: 触发评测

- 约 20 条查询, 8 到 10 条应触发, 8 到 10 条不应触发; 负例要选共享关键词的近似请求 [S3 §Designing trigger eval queries]
- 每条跑 3 次算触发率, 0.5 为默认阈值; 20 条 x 3 次 = 60 次调用 [S3 §Running multiple times]
- 训练集约 60% / 验证集约 40%, 只用训练集失败来改描述, 按验证集通过率选最佳版本; 通常 5 轮足够 [S3 §Avoiding overfitting] [S3 §The optimization loop]
- 不要把失败查询里的具体关键词塞进描述, 那是过拟合 [S3 §The optimization loop]
- Claude Code 可用 `claude -p ... --output-format json` 检查有无 `Skill` 工具调用来判定是否触发 [S3 §Running multiple times 脚本]
- Codex 可用 `codex exec --json` 拿 JSONL 事件流写确定性检查 (是否跑了某命令、是否生成某文件) [S8 §5]; 小规模 10 到 20 条提示即可抓回归, 至少放一条 should_trigger=false 的负例 [S8 §4] [S3 §Should-not-trigger queries]

## 验证: 输出评测

- 先写评测再写大量文档: 无 skill 跑代表任务记录失败, 建 3 个场景, 测基线, 只写刚好够过的指令 [S1 §Build evaluations first]
- 每个用例跑 "有 skill" 与 "无 skill (或旧版本)" 两组, 比较通过率增量与 token、时间开销 [S4 §Running evals] [S5 §Evaluate and iterate on a skill]
- 用例从 2 到 3 个开始; 断言在看过第一轮输出后再写; 能用脚本查的断言用脚本 [S4 §Designing test cases] [S4 §Writing assertions] [S4 §Grading outputs]
- 每次运行要从干净上下文开始, 写 skill 时残留的上下文会掩盖指令缺口 [S4 §Spawning runs] [S5 §Evaluate and iterate on a skill]
- 两组都总是通过的断言要删, 它们只是抬高通过率 [S4 §Analyzing patterns]
- Claude Code 提供 `claude plugin eval` (插件 skill) 与 skill-creator 插件 (evals/evals.json, 盲 A/B, 描述调优) [S5 §Run evals with skill-creator]
- 用一个 Claude 实例写 skill, 另一个全新实例拿它干真活, 观察它读文件的路径与漏掉的规则 [S1 §Develop Skills iteratively with Claude] [S1 §Observe how Claude navigates Skills]

## 反例

- 据 Vercel: 给 Next.js 16 文档做成 skill, 56% 的评测用例里 skill 从未被调用, 通过率 53% 与无文档基线相同 [S7 §Skills weren't being triggered reliably]; 机制上与 "agent 只为自己做不好的任务查 skill" 一致 [S3 §How skill triggering works]; OpenAI 博客也指出描述含糊或过载时 skill 不会可靠触发 [S8 §2]
- 据 Vercel: 在 AGENTS.md 里明确要求调用后触发率 95%+、通过率 79%; 而 8KB 压缩文档索引直接放进 AGENTS.md 达 100% (Build/Lint/Test 全 100%) [S7 §results]
- 据 Vercel: 措辞改动导致结果大幅摆动: "You MUST invoke the skill" 让 agent 先读文档、锚定文档写法、漏掉项目上下文 (漏改 `next.config.ts`); "先探索项目再调用" 结果更好 [S7 §Explicit instructions helped, but wording was fragile]
- 据 Vercel: 未被使用的 skill 留在环境里, Test 指标反而从 63% 降到 58% [S7 §Skills weren't being triggered reliably]
- Vercel 自己的边界: skill 更适合用户显式触发的纵向动作 (升级、迁移), 通用框架知识走被动上下文 [S7 §What this means for framework authors]
- 指令强度建议互相冲突: S1 在迭代示例里允许把 "always filter" 改成 "MUST filter" [S1 §Develop Skills iteratively]; S4 建议解释原因而不是写 ALWAYS/NEVER [S4 §Iterating on the skill]
