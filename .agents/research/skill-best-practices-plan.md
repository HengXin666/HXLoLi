# hx-make-skill 与 hx-note 改造 plan (只含 plan, 不含实施)

事实底座: `HengXin666/HXLoLi` main 分支 `aa21ee9`, 2026-10-21 稀疏克隆 `.agents/skills` `ai-docs` `scripts` 后逐文件阅读并实跑门禁. 文中 `文件:行号` 均指该提交. 外部资料与仓库冲突处以仓库为准, 冲突点在条目里标 `[冲突]`. 查不到的写 `[未核实]`

## 1. 一页结论

### hx-make-skill 最致命的三个问题

1. 门禁假通过. `scripts/checks.py:142` 的描述判定元组首项是空串 `''`, 而 `'' in s` 恒为真, 所以 "一行平铺三个以上路径却没有描述" 这条检查永远不会报. 这正是它自己硬约定一要拦的病, 而 hx-note 的 `SKILL.md:115` 就是这种行, 校验器报 0 warning
2. 文档给出的是错误的安全感. `references/spec-checklist.md:334-369` 把命名不一致, body 里的 When to Use, 未引用的 L3 文件, 引用超过一层都标成 ERROR, 但 `checks.py:29` 默认 `STRICT = False`, 这些全部降级为 WARN 且退出码为 0. 引用深度检查 (`checks.py:148-154`) 只看 markdown 链接, 而本 skill 强制裸路径, 所以这条检查对合规 skill 永远不生效. `references/legacy-audit.md:90` 写着 hx-note 体量上限 PASS, 实跑 `check_layout.py` 是 FAIL
3. 调用开关自相矛盾. `SKILL.md:5` 是 `disable-model-invocation: true`, 而 `agents/openai.yaml:8` 是 `allow_implicit_invocation: true`, description 又按隐式触发写成长句. Claude Code 文档说明开了这个开关后 description 根本不进上下文, 所以它在 Claude Code 里永远不会自动触发, 在 Codex 里却会. DSH 怎么处理这个字段 `[未核实]`

### hx-note 最致命的三个问题

1. doctor 跑了但没拦. `scripts/cli/flow/flow_doctor.py:173-174` 对两份盲审报告只查文件存在, 不查判定. 步骤 8 契约 (`steps/8-fidelity/index.md` 判定一节) 写的是 FAIL 不许交付, 新增与歪曲两节必须为空, 这两条没有任何机械检查
2. doctor 当前对每一篇笔记都是红的. `_broken_skill_paths` 扫的是整个 `.agents/skills`, 实跑发现 3 处与笔记无关的断链 (`hx-agent-notes/SKILL.md:59`, `hx-make-skill/references/legacy-audit.md:30` 与 `:49`). 一道永远红的闸门会训练人和 agent 去 `--force` 或在回复里写 "跳过", 等于没有闸门
3. 知识库与 "面向 AI 的知识库" 目标没对齐. 42 篇 `index.md` 里只有 13 篇有 `.hx-info.md` (31%), 全库只有 2 条 `hxid:` 跨文双链, 26 张答题卡里 8 张仍有未答题 (49 题中 14 题未答). 事实源单一与链接密度这两个核心不变量在存量里大面积不成立

### 最值得先做的三个动作

1. 修两条假通过: M1 (空串判定) 与 N2 (doctor 只查本 skill 断链, 跨 skill 断链交给 hx-make-skill 校验器). 改动小, 立刻恢复门禁的可信度
2. N1: 让两份盲审报告带一行机器可读的判定头, doctor 解析它, FAIL 或新增/歪曲非空就拦
3. M2 + N3: 统一两个 skill 的调用开关与各家元数据, 并在校验器里加一致性检查, 杜绝 "看起来合规但从不触发"

## 2. hx-make-skill

### 2.1 gap 表

| # | 议题 | 现状 (证据) | 外部最佳实践 | 差距 | 建议动作 | 优先级 |
|---|---|---|---|---|---|---|
| M1 | 平铺路径检查失效 | `scripts/checks.py:142` 元组首项为 `''`; 构造 3 个裸路径无描述的夹具, 校验器 0 warning | Anthropic best practices: SKILL.md 要说明每个文件是什么, 何时读 | 硬约定一没有真门禁 | 换成真实描述标记并加回归夹具 | P0 |
| M2 | 调用开关与元数据矛盾 | `SKILL.md:5` disable 为 true; `agents/openai.yaml:8` 隐式调用为 true | Claude Code: disable 后 description 不进上下文; Codex: `allow_implicit_invocation` 控制隐式调用 | 同一 skill 在两家行为相反, 在 DSH 上 `[未核实]` | 人类拍板后统一; 校验器加交叉检查 | P0 |
| M3 | 文档等级与代码等级不符 | `spec-checklist.md:334-369` 多项写 ERROR; `checks.py:29` 默认 WARN; 第五步命令不带 `--strict` | 门禁要给出真实可执行的判据 (agentskills.io evaluating) | 读文档的人以为被拦了 | 文档改成双列 (默认/strict); hx 自家 skill 的校验命令带 `--strict` | P0 |
| M4 | 存量审计表过期且不可复现 | `references/legacy-audit.md:85-94` 静态表; `:76` 硬编码 `/home/hx/...`; `:77` 的复现命令带 `--list`, 而 `prose_rules.py:222` 定义 `--list` 为只打印不失败, 实测有违规时退出码仍为 0; 实跑 hx-note 体量 FAIL 而表写 PASS | Anthropic: 避免时效性信息 | 审计结论会腐烂且无法在别人机器复现 | 改成脚本实时生成, 文档只留做法与三条不顺手改 | P0 |
| M5 | 引用可达只查三个目录 | `checks.py:85` 只遍历 `references` `scripts` `assets` | 规范: 所有引用文件都要从 SKILL.md 可达 | 本 skill 硬约定二引入的 `steps/` `entries/` `shared/` `templates/` 不受检; hx-note PASS 无意义 | 扩到这些目录, 并对裸路径做存在性与深度检查 | P1 |
| M6 | forward-test 只有自然语言 | `SKILL.md:156-165` 与 `references/patterns.md` 第六节, 无脚本无数据 | agentskills.io: 约 20 条 should/should-not 查询, 每条跑 3 次算触发率, 60/40 拆训练与验证; Claude Code: 开关 skill 做基线对比 | 触发率与误触发率无法度量 | 新增触发回归集与与 agent 无关的跑分脚本 | P1 |
| M7 | description 检查过弱 | `checks.py:61-71` 只匹配 `Use when` 等关键词, 且只 WARN | Anthropic: description 不得含 XML 标签, name 不得含保留词, 第三人称; Claude Code: description 加 when_to_use 超 1536 字符被截断, 关键用途要前置 | 合规但从不触发的 description 能过 | 加 `<` `>` 与保留词 ERROR, 前置度与人称 WARN | P1 |
| M8 | prose `--fix` 会破坏路径与粗体 | `spec-checklist.md:380` 出现 `HXLoLi/. agents/notes`; hx-note `SKILL.md:97` 出现 `. drawio.svg`; 多处 `第三份. **` 让粗体闭合失效; `SKILL.md:18` 等处有破折号被删后的双空格残留 | CommonMark: 闭合 `**` 前不能是空白 | 自动修复引入新错误, 检查发现不了 | fixer 跳过点开头标识符与粗体闭合; 加 P6 检查 | P1 |
| M9 | 落地位置只写 DSH | `SKILL.md:181-189` | Codex 从 CWD 到仓库根逐级扫 `.agents/skills`; Claude Code 只扫 `.claude/skills`, YAML 坏时仍以空元数据加载 body | 跨 agent 行为差异没写, 排障表会误导 | 加一张三家对照表, 标 `[冲突]` | P1 |
| M10 | 自身常量与引用漂移 | `scripts/frontmatter.py:10` 有未使用的 `MAX_COMPAT`; `scripts/prose_core.py:4` 提到不存在的 `prose_fix.py` | `references/code-quality.md:45` 自己规定常量只存一处 | 违反自家规则 | 删除重复常量, 修正文档串 | P2 |

### 2.2 分项 plan

#### M1 修平铺路径检查

- 目标: 一行列出三个以上路径且没有任何描述时必出 WARN (strict 下 ERROR)
- 改哪些文件: `.agents/skills/hx-make-skill/scripts/checks.py` 的 `check_index_style`
- 改法: 把第 142 行元组里的 `''` 换成真实描述标记 (破折号 `\u2014`, 英文冒号 `:`, 中文冒号 `\uff1a`, `什么时候读`, `用于`), 用 `chr()` 或转义写, 避免再被文本 fixer 吃掉; 同时把 `warns.append` 改成走 `soft()` 以便 strict 生效
- 验收: 用 `/tmp` 下的夹具 (SKILL.md 正文一行 `See references/x.md references/y.md scripts/z.py`) 跑 `uv run scripts/validate_skill.py <夹具>`, 期望 1 条 WARN; 再对 hx-note 跑, 期望指出 `SKILL.md` 第 115 行
- 风险: 现存 skill 会冒出新 WARN, 默认不改退出码, 可控

#### M2 统一调用开关并加交叉检查

- 目标: 每个 skill 在 DSH, Claude Code, Codex 三处的调用行为一致且有意为之
- 改哪些文件: `hx-make-skill/SKILL.md` 第 5 行, `hx-make-skill/agents/openai.yaml`, `scripts/checks.py` (新增 `check_invocation`), `references/spec-checklist.md` 第 2 节字段表
- 改法: 按开放问题 Q1 的答案二选一 (推荐把 disable 改为 false, 理由是知识型 skill 没有副作用, 且它要处理 "skill 不触发" 这类请求). 校验器新增: disable 为 true 但 description 含触发短语时 WARN (提示这些字在 Claude Code 里不进上下文); 存在 `agents/openai.yaml` 时, 其 `allow_implicit_invocation` 与 disable 取值相反则 WARN
- 验收: 夹具一 (disable true, openai.yaml true) 出 1 条 WARN; 修正后的 hx-make-skill 对自身 0 条相关 WARN
- 风险: DSH 是否读 `disable-model-invocation` `[未核实]`, 实施前需在 DSH loader 源码确认

#### M3 让文档等级说实话

- 目标: 读 spec-checklist 的人知道每条检查在默认与 strict 下各是什么等级
- 改哪些文件: `references/spec-checklist.md` 第 2 节与第 4 节表格, `SKILL.md` 第五步代码块
- 改法: "校验器对应" 一列拆成 "默认" 与 "--strict" 两列, 按 `checks.py` 实际行为填; 第五步的 hx 自家 skill 校验命令加 `--strict`, 并写一句: 外部或 vendored skill 用默认档
- 验收: 逐条对照 `checks.py` 中 `errors.append` 与 `soft()` 的调用点, 表格与代码一一对应; `uv run scripts/validate_skill.py hx-make-skill --strict` 退出码 0
- 风险: hx 系列 skill 在 strict 下可能出现 ERROR, 需要先跑一遍清单再推

#### M4 存量审计改为实时生成

- 目标: 存量状态不再写死在文档里, 任何人 clone 后一条命令复现
- 改哪些文件: `references/legacy-audit.md` (删当前状态表与硬编码路径), `scripts/` 目录重组
- 改法: `scripts/` 已有 6 个文件, 到了自家 C1 上限, 不能直接加第 7 个. 先拆成 `scripts/gates/` (三个入口) 与 `scripts/lib/` (checks, frontmatter, prose_core), 再在 `gates/` 加 `audit_all.py`: 遍历 `.agents/skills/*`, 对每个 skill 跑三条门禁并打印表格. 文档只保留 "怎么复现" 与 "三条不可以顺手做的改动"
- 验收: `uv run scripts/gates/audit_all.py .agents/skills` 输出包含 hx-note 体量 FAIL, 且有任一 FAIL 时退出码非 0 (复现命令不得再带 `--list`); `rg -n '/home/hx' .agents/skills/hx-make-skill` 无结果; `check_layout.py hx-make-skill` 仍 PASS
- 风险: 脚本搬目录会断掉所有调用方 (hx-note doctor 的路径检查, SKILL.md 内的命令). 必须同一次改动改全并跑 hx-note doctor 的断链检查

#### M5 引用可达扩到流程型目录

- 目标: 流程型 skill 的每个 `steps/*/index.md` `entries/*/index.md` `shared/*` `templates/*` 都能从 SKILL.md 走到, 且正文里的裸路径都指向真实文件
- 改哪些文件: `scripts/checks.py` 的 `check_references` 与 `check_index_style`
- 改法: 目录集合扩为 references scripts assets steps entries shared templates; 对正文里匹配到的裸路径做存在性检查 (复用 hx-note `flow_doctor.py` 的 `REL_RE` 与三基准解析逻辑, 把它迁成 hx-make-skill 的唯一实现); 深度检查对裸路径同样生效
- 验收: 对 hx-note 跑, 期望列出 SKILL.md 未直接或经入口页引用的文件 (若有); 对 hx-agent-notes 跑, 期望报出 `SKILL.md:59 -> scripts/github/package.js`
- 风险: 与 N2 联动, 两边要同一 PR 合入, 否则会出现两份断链检查

#### M6 触发回归集与 forward-test 协议

- 目标: 每个 hx skill 能给出触发率与误触发率两个数, 并能对比改 description 前后
- 改哪些文件: 新建 `references/forward-test.md` (协议), 新建 `evals/trigger_queries.json` (模板), 跑分脚本放 `scripts/gates/` (受 M4 重组约束), `SKILL.md` 第六步改为指向协议与命令
- 改法: 照 agentskills.io 的做法: 约 20 条真实口吻查询, 8 到 10 条应触发, 8 到 10 条近似但不应触发; 每条跑 3 次, 触发率阈值 0.5; 固定 60/40 拆训练与验证, 只看训练集失败改 description, 用验证集选版本. 跑分脚本只负责调度与统计, "怎么判断触发" 由环境变量给出的命令模板决定 (Claude Code 可解析 `--output-format json` 里的 Skill 工具调用; DSH 的判定方式 `[未核实]`)
- 验收: 对 hx-make-skill 自身产出 `benchmark` 一份, 含训练与验证两组触发率; 协议文档过 prose 门禁
- 风险: 每次 60 次调用有成本; 若 M2 保留 disable 为 true, 隐式触发率按定义为 0, 需在协议里区分 "显式调用质量评估" 与 "隐式触发评估"

#### M7 加强 description 与 name 检查

- 目标: 能拦住官方明令禁止的写法, 并对影响触发的写法给出提示
- 改哪些文件: `scripts/checks.py` 的 `check_name` `check_description`, `references/spec-checklist.md`
- 改法: name 或 description 含 `<` `>` 判 ERROR (Anthropic 约束, Claude Code 也会转义尖括号); name 含 `anthropic` 或 `claude` 判 ERROR; description 以 `I can` `You can` `我可以` 开头判 WARN (第三人称); description 加 `when_to_use` 超 1536 字符判 WARN, 并提示关键用途放前 200 字符
- 验收: 四个夹具各触发对应一条; 现有 hx 系列 skill 不新增 ERROR
- 风险: "第三人称" 与 agentskills.io 推荐的祈使句 "Use this skill when" 存在张力, 只拦第一与第二人称自称, 不拦祈使句

#### M8 修 prose fixer 的两类误伤

- 目标: `--fix` 不再往点开头标识符里插空格, 不再把粗体闭合拆开; 已造成的破损可被检查发现
- 改哪些文件: `scripts/prose_core.py` (行角色与掩码), `scripts/prose_rules.py`, `references/prose-rules.md` (新增 P6)
- 改法: 掩码阶段把 `\.[A-Za-z][\w.-]*` 形态 (如 `.agents` `.tsx` `.hx-info.md`) 视为标识符; 句末标点紧跟 `**` 时把空格补在 `**` 之后; 新增 P6: 命中 `\S\. \*\*` 或 `/\. [a-z]` 或两个以上连续空格夹在 CJK 之间时报违规
- 验收: 对 `spec-checklist.md` 与 hx-note 的 `SKILL.md` `shared/pipeline.md` 跑 `--check`, 期望 P6 报出已知破损行; 修完后对夹具 `--fix` 前后做逐字 diff, 只有预期改动
- 风险: P6 首次上线会在多个 hx skill 报大量历史破损, 需配一次机械清扫 (见 N8)

#### M9 落地位置补三家对照

- 目标: 排障时能分清是哪家 loader 的行为
- 改哪些文件: `SKILL.md` 落地位置一节, 详细表外移到 `references/spec-checklist.md` 新增一节, SKILL.md 只留一句索引
- 改法: 对照表列: 扫描根, 嵌套发现, frontmatter 解析失败时的行为, 隐式调用开关所在处. DSH 一列以仓库现有描述为准; Claude Code 一列写 `.claude/skills` 与 YAML 失败仍加载 body; Codex 一列写 `.agents/skills` 逐级扫描与 `agents/openai.yaml`
- 验收: `validate_skill.py --strict` 与 `prose_rules.py --check` 通过; SKILL.md 行数仍低于 500
- 风险: `[冲突]` Claude Code 在 YAML 解析失败时降级加载, 与 `SKILL.md:172` 描述的 DSH 静默丢弃相反, 表里两种都写, 不要用一家覆盖另一家

#### M10 自身漂移清理

- 目标: 自家规则 "常量只存一处" 对自己成立
- 改哪些文件: `scripts/frontmatter.py`, `scripts/prose_core.py`
- 改法: 删 `frontmatter.py` 的 `MAX_COMPAT`; 把 `prose_core.py` 文档串里的 `prose_fix.py` 改为实际的 `prose_rules.py --fix`
- 验收: `rg -n 'MAX_COMPAT' scripts` 只剩 `checks.py` 一处; `rg -n 'prose_fix' .agents/skills/hx-make-skill` 无结果
- 风险: 无

## 3. hx-note

### 3.1 gap 表

| # | 议题 | 现状 (证据) | 外部最佳实践 | 差距 | 建议动作 | 优先级 |
|---|---|---|---|---|---|---|
| N1 | 盲审判定不被机械检查 | `flow_doctor.py:173-174` 只查两份报告存在 | agentskills.io evaluating: PASS 必须有具体证据, 能用脚本查的不交给判断 | 契约写 FAIL 不许交付, 但 FAIL 报告照样过 doctor | 报告加判定头, doctor 解析 | P0 |
| N2 | doctor 永远红 | `flow_doctor.py:72-115` 扫整个 `.agents/skills`; 实跑 3 处与笔记无关的断链 | Claude Code 排障: 一个检查只对它负责的对象报错 | 闸门失去信号价值 | 只扫 hx-note 自身, 跨 skill 交给 M5 | P0 |
| N3 | 调用开关与长 description | `SKILL.md:5` disable 为 true; description 543 字符, 几乎全是触发词, 且含全角 `。` | Claude Code: disable 后 description 不进模型上下文, 只用于菜单 | 543 字符触发词在 Claude Code 里是死重; DSH 行为 `[未核实]` | 保留 disable, 把 description 收成给人看的一两句 | P1 |
| N4 | 步骤 4 契约自相矛盾 | `steps/4-align/index.md:350` 一次只问一题; 同文件 `:354` 每轮问当前整个 frontier | `impl/grill.md:17` 一次只问一题 | 契约与实现打架, agent 可任选一条 | 改第 354 行 | P1 |
| N5 | cover 不进任何闸门, 条款判据过宽 | `flow_place.py:88-131` cover 只打印; `flow_read.py:176-177` 动作词含 `看` `文件` | Zettelkasten 与 docs-as-code: 验收要可执行, 进 CI | "看起来好看" 能过 intake; 零支撑需求能过 doctor | doctor 调 cover; 收紧动作词 | P1 |
| N6 | 自家代码超体量上限 | `check_layout.py hx-note` 报 `scripts/cli/voice/voice_rules.py:27` 20 行注释块, 第 39-42 行两行重复 | `hx-make-skill/references/code-quality.md` C4 | 与上游规则不一致, 审计表还写 PASS | 理由迁出注释, 去重 | P1 |
| N7 | fixer 破损残留 | `SKILL.md:13` `:97`; `shared/pipeline.md:142`; `steps/1-intake/index.md` `steps/3-atom/index.md` `steps/6-derive/index.md` 多处 `. **` | CommonMark 粗体规则 | 加载给模型的正文有渲染与路径错误 | M8 落地后机械清扫 | P1 |
| N8 | 索引写法违反上游硬约定一 | `SKILL.md:115` 一行串 11 个路径 | `hx-make-skill/SKILL.md:23-34` | 自家规范自家不守 | 改成表格 | P1 |
| N9 | 存量与 AI 知识库目标偏离 | 13/42 篇有 `.hx-info.md`; 全库 2 条 `hxid:` 链接; 8/26 张答题卡有未答题 | Matuschak: evergreen notes 要原子且密集链接; Anthropic contextual retrieval: 切块丢上下文会让检索失败 | 事实源单一与链接密度在存量不成立 | 先度量后治理: 健康报告 + 新笔记链接下限 | P1 |
| N10 | 九步摩擦 | 9 步全串行, 步骤 5 契约自称纯机械 (`steps/5-place/index.md`) | Anthropic: 工作流要清晰, 但能交给脚本的确定性步骤交给脚本 | 人类要多推一次纯机械步骤 | 步骤 5 可由 `done align` 后自动提示一条完整 `place` 命令 | P2 |
| N11 | 笔记升级成 skill 的判据 | `SKILL.md:128` 只说 "用 hx-make-skill" | Anthropic: 先在任务里观察反复提供的上下文, 再沉淀成 skill | 没有触发条件, 永远不会升级 | 写一条可判定的升级判据 | P2 |
| N12 | 发散记录在 doctor 里显示为通过 | `flow_doctor.py:216-218` 恒为 True | 无 (内部一致性) | 报告型项被计入 "N/M 项通过" | 显示为信息项, 不计数 | P2 |

### 3.2 分项 plan

#### N1 盲审判定进 doctor

- 目标: 保真盲审 FAIL, 或新增/歪曲两节非空时, doctor 必红; 味道盲审未逐条落实时 doctor 必红
- 改哪些文件: `assets/fidelity-audit-prompt.md`, `assets/voice-audit-prompt.md`, `scripts/cli/flow/flow_doctor.py`, `steps/8-fidelity/index.md` 判定一节
- 改法: 两份 prompt 要求报告首行输出 `verdict: PASS` 或 `verdict: FAIL`; 保真报告保留固定小节标题 `## 新增` `## 歪曲`. doctor 读首行与两节正文 (去掉 `无` `-` 等占位后为空才算空). `flow/` 已有 6 个文件到了 C1 上限, 新逻辑放进 `flow_doctor.py` (现 233 行, 预计加 30 行, 仍低于 300), 不新建文件
- 验收: 三份夹具报告 (PASS 且两节空 / FAIL / PASS 但新增节有一条) 对应 doctor 结果为 通过 / 拦 / 拦; `check_layout.py hx-note` 不新增违规
- 风险: 已落地的历史笔记报告没有判定头, 会被拦. 对 `created_at` 早于本次改动的流程只报告不拦 (与 N12 的发散记录同一处理口径)

#### N2 doctor 只对本 skill 负责

- 目标: 笔记的 doctor 结果只由这篇笔记与 hx-note 自身决定
- 改哪些文件: `scripts/cli/flow/flow_doctor.py` 的 `_broken_skill_paths` 与调用处
- 改法: 扫描根从 `.agents/skills` 改为 hx-note 自身目录; 全仓 skill 断链由 hx-make-skill 校验器 (M5) 负责. 另在各自 owner 的改动里修掉现存 3 处断链 (`hx-agent-notes/SKILL.md:59`; `legacy-audit.md` 两处改成带 skill 前缀的路径)
- 验收: 在当前仓库对任一已落地笔记跑 `hx_flow.py doctor`, "技能文档里的路径全部可达" 一项为通过; 人为在 hx-note 的某个 md 里写一个不存在的 `impl/x.md`, 该项变红
- 风险: 必须与 M5 同批合入, 否则会有一段时间全仓断链无人检查

#### N3 收窄 description

- 目标: 调用开关的选择有意为之, description 的字数花在真正读它的地方
- 改哪些文件: `SKILL.md` frontmatter
- 改法: 推荐保留 `disable-model-invocation: true` (理由见 Q2). description 收到 300 字符以内, 一句做什么, 一句三个入口, 改用英文标点. 触发词清单不删, 移到正文首段的 "三条入口" 表下面作为路由提示 (显式调用后才读到, 用于分流到正确入口)
- 验收: `validate_skill.py hx-note --strict` 通过; description 长度低于 300; 用 M6 的协议做显式调用 3 条用例, 均能分流到正确入口
- 风险: 若 DSH 并不读这个开关而依赖 description 自动触发, 收窄会降低触发率. 实施前先确认 DSH loader 行为

#### N4 修步骤 4 契约

- 目标: 契约只有一种说法
- 改哪些文件: `steps/4-align/index.md`
- 改法: 第 354 行 "每轮问当前整个 frontier, 每题附推荐答案" 改为 "每轮从当前 frontier 取一题, 附推荐答案, 然后停下等人类", 与 `impl/grill.md` 一致
- 验收: `rg -n '整个 frontier' .agents/skills/hx-note` 无结果
- 风险: 无

#### N5 cover 进 doctor 并收紧条款判据

- 目标: 零支撑的需求不能悄悄交付; 写不出检查动作的条款不能混进 goal
- 改哪些文件: `scripts/cli/flow/flow_doctor.py` (调用 cover 的计算部分), `scripts/cli/flow/flow_place.py` (把计算与打印拆开, 供 doctor 复用), `scripts/cli/flow/flow_read.py` 的 `ACCEPT_ACTION_RE`
- 改法: doctor 新增一项 "每条验收条款至少一条知识点支撑, 或在 goal 里标为已放弃并写原因". 动作词去掉单字 `看` 与泛词 `文件`, 改为要求出现命令, 路径, 计数或截图之一 (`grep` `uv run` `node` 反引号路径 `行数` `截图` `退出码` 等)
- 验收: 夹具 goal 含条款 "看起来要好看", `done intake` 被拒; 夹具 info 缺 `满足: G2`, doctor 该项变红
- 风险: `满足: Gx` 仍是自报, 只能证明 "有人声称实现过". 这是机械判据的天花板, 真伪仍靠步骤 8 盲审

#### N6 修 voice_rules 注释块

- 目标: hx-note 过上游体量门禁
- 改哪些文件: `scripts/cli/voice/voice_rules.py`
- 改法: 第 27-42 行的校准过程与否决案例迁到一份 Agent Note (路径沿用文件里已引用的那篇), 代码里只留一句做什么加 note 路径; 删掉重复的两行 `Agent Notes:`
- 验收: `uv run .agents/skills/hx-make-skill/scripts/check_layout.py .agents/skills/hx-note` 退出码 0
- 风险: 否决案例迁走后, 新增套话词的人可能看不到; 在 `CLICHE_COUNTS` 上方保留一句 "新增前先看 note" 即可

#### N7 清扫 fixer 破损

- 目标: hx-note 正文里没有被拆开的粗体与被插空格的路径
- 改哪些文件: hx-note 下所有 md (机械改动)
- 改法: 等 M8 的 P6 上线后跑一遍 `prose_rules.py --check --list`, 按报告逐处修; 修复本身用新版 `--fix`
- 验收: P6 在 hx-note 下 0 命中; `git diff --stat` 只涉及标点与空格
- 风险: 大面积机械 diff 不好审, 单独一个 commit, 与内容改动分开

#### N8 SKILL.md 模板与素材一节改表

- 目标: 符合上游硬约定一
- 改哪些文件: `SKILL.md` 模板与素材一节
- 改法: 第 115 行拆成两列表格 (路径 / 是什么, 什么时候读), 与脚本一节同形
- 验收: M1 修好后 `validate_skill.py hx-note` 不再报该行
- 风险: SKILL.md 增加约 10 行, 仍远低于 500

#### N9 先度量, 再治理存量

- 目标: 让 "面向 AI 的知识库" 有可看的数, 并让新笔记不再降低这些数
- 改哪些文件: `scripts/cli/flow/hx_flow.py` 的 `list` 子命令 (加 `--health`), `steps/6-derive/index.md` 过关条件, `entries/organize/impl/batch-audit.md`
- 改法: `list --health` 输出四个数: 有 `.hx-info.md` 的笔记占比, 每篇平均 `hxid:` 出链, 未答题的答题卡数, 无可复用物的笔记数. 新笔记在步骤 6 增加一条 WARN 级要求: 至少一条 `hxid:` 链到已有笔记, 或在答题卡里写明为什么没有. 存量不批量回填, 只在 organize 入口碰到时补 (理由见 Q5)
- 验收: 在当前仓库跑 `hx_flow.py list --health`, 输出 13/42, 2 条链接, 8 张未答完的卡; 新建一篇无链接的笔记, 步骤 6 出 WARN
- 风险: 链接下限可能催生凑数链接. 只要求一条, 且允许写明理由豁免

#### N10 步骤 5 降摩擦

- 目标: 人类不必为纯机械步骤单独推进一次
- 改哪些文件: `scripts/cli/flow/flow_steps.py` 的 `cmd_done` 输出, `steps/5-place/index.md`
- 改法: `done align` 成功后, 根据 tag 建议与目录现状直接打印一条填好的 `place` 命令供确认; 不自动执行, 因为新建分类目录必须人类同意
- 验收: 夹具流程 `done align` 后的输出包含可直接执行的 `place` 命令
- 风险: 自动建议的编号可能与并发新增冲突, 命令执行时 `place` 仍重新校验

#### N11 笔记升级成 skill 的判据

- 目标: 可复用规格在满足条件时被提示升级
- 改哪些文件: `SKILL.md` 怎么扩展一节最后一行, `steps/6-derive/impl/reusable-spec.md`
- 改法: 写一条判据: 同一份可复用规格在两次以上不同任务里被照做, 或它带了需要反复执行的脚本, 就提示用 hx-make-skill 升级, 笔记保留为该 skill 的 explanation, skill 只放流程与脚本
- 验收: 判据中的两个条件都能用 `rg` 计数或目录检查判定
- 风险: 需人类拍板阈值 (Q6)

#### N12 发散记录项改为信息项

- 目标: doctor 的通过计数只含真正的闸门
- 改哪些文件: `scripts/cli/flow/flow_doctor.py`
- 改法: `add()` 增加 `kind` 参数, 信息项打印为 `[i]`, 不计入分母
- 验收: 输出的 `N/M 项通过` 中 M 比当前少 1
- 风险: 无

## 4. 不建议改

| 考虑过的改动 | 否决理由 |
|---|---|
| 把步骤 7 与 8 合并成一次盲审 | 先改文风会动句子, 动句子才可能带偏事实; 合并后保真审看到的是改前稿, 等于没审 (`steps/8-fidelity/index.md` 与步骤 7 一节). 外部评测实践也要求每次评估用干净上下文 |
| 放宽 6 文件 / 300 行上限, 好给 M4 M6 N1 腾地方 | 上限挡住的是真实故障: 常量两处漂移与模块形态混用 (`references/code-quality.md:25` 与 `:45`). 正确做法是拆目录, 并且 `flow/` 与 `scripts/` 两处都已到上限, 本 plan 已按此规划 |
| 索引改用 markdown 链接, 跟 Anthropic 示例一致 | `[冲突]` 官方示例用链接, 但只要求 "说明每个文件是什么, 何时读". 裸路径加描述同样满足, 且少一份 token. 以仓库为准 |
| 按 Anthropic 建议把 skill 改成动名词命名 | 官方只说 "consider", 名词短语也列为可接受; 改名会断掉 HXLoLis 符号链接与全部调用方 |
| 把校验器默认改成 strict | 校验器也用于外部与 vendored skill, 默认档对齐 loader 真实强制的最小集合 (`checks.py:27-29`) 是对的. 改为在 hx 自家命令上带 `--strict` (M3) |
| doctor 对发散记录改成硬拦 | `done atom` 已硬拦; doctor 再拦只会卡住早于这道闸门的历史流程 (`flow_doctor.py:213-215` 注释). 只改显示方式 (N12) |
| 放宽 "一次只问一题" | 有实测失败作证 (`steps/4-align/impl/grill.md:7-9`, 16 张卡 11 张未填). 本 plan 只修同文件里与它矛盾的那一行 |
| 让 `assets/pre-commit-note.sh` 阻挡提交 | 它被设计成提示 (全文 `exit 0`); 真闸门是 doctor. 把提示升级为阻挡会让人类提交被 AI 流程绑架, 违反红线 "责任在人类" |
| 清理 pyc 残留 | 已核实: `git ls-tree -r HEAD` 中 0 个 `.pyc` 与 `__pycache__`, 残留只存在于某个本地工作区, 不是仓库问题. 两个门禁也已忽略它们 (`checks.py:16-17`) |
| 存量 29 篇笔记批量回填 `.hx-info.md` | 回填要逐篇走 intake 与 align, 需要人类答题, 批量做只会产出没人审过的事实源, 违背 "审核必须真的发生". 改为碰到再补 (N9) |
| SKILL.md 把 "常见失败" 一节外移 | hx-make-skill 正文 206 行, 远低于 500, 且这一节是它最常被调用的场景; 外移会多一跳读取 |

## 5. 待人类拍板的开放问题

| # | 问题 | 推荐答案与理由 |
|---|---|---|
| Q1 | hx-make-skill 允许模型自动调用吗? | 允许, 把 disable 改为 false, openai.yaml 保持 true. 它是知识型 skill, 无副作用, 而 "我的 skill 为什么不触发" 正需要它被自动拉起 |
| Q2 | hx-note 允许模型自动调用吗? | 不允许, 保留 true. 它会建目录, 写文件, 提交; 红线 "禁止自作主张开始沉淀" 与 Claude Code 对有副作用工作流的建议一致. 同时把 openai.yaml (若新增) 的隐式调用设为 false |
| Q3 | 要不要把 Claude Code 列为正式支持的宿主? | 要, 成本只是在仓库加 `.claude/skills` 指向 `.agents/skills` 的符号链接 (Claude Code 支持符号链接目录). 但是否有人在用 Claude Code `[未核实]`, 请确认 |
| Q4 | 触发回归集先对接哪个宿主? | 先对接能输出结构化工具调用日志的 CLI (Claude Code 的 `-p --output-format json` 或 Codex 的非交互模式), DSH 待确认它有无等价日志后再接 |
| Q5 | 存量笔记补 `.hx-info.md` 的策略? | 只在 organize 入口碰到时补, 不批量回填; 先用 N9 的健康报告看清规模 |
| Q6 | 可复用规格升级成 skill 的阈值? | 被照做两次以上, 或带需反复执行的脚本, 二者满足其一即提示 (只提示, 不自动建 skill) |
| Q7 | 盲审判定头上线后, 历史流程怎么办? | 以本次改动日期为界, 之前创建的流程只报告不拦, 与发散记录同口径 |

## 6. 来源清单

| 标题 | URL | 支撑的结论 |
|---|---|---|
| Skill authoring best practices (Claude Platform Docs) | https://docs.claude.com/en/docs/agents-and-tools/agent-skills/best-practices | description 不含 XML 标签, name 不含保留词, 第三人称; 引用一层深; 500 行; 先建评估; 避免时效信息 (M4 M7, 不建议改中的命名与链接两条) |
| Extend Claude with skills (Claude Code Docs) | https://code.claude.com/docs/en/skills | disable-model-invocation 让 description 不进上下文; description 与 when_to_use 合计 1536 字符截断; 扫描根 `.claude/skills`; YAML 失败仍加载 body; 支持符号链接; 开关 skill 做基线对比 (M2 M6 M7 M9 N3 Q2 Q3) |
| Build skills (OpenAI Codex) | https://developers.openai.com/codex/skills | Codex 逐级扫描 `.agents/skills`; `agents/openai.yaml` 的 `allow_implicit_invocation`; 初始列表上下文预算与描述缩短 (M2 M9) |
| Specification (Agent Skills) | https://agentskills.io/specification | 六个标准字段与约束; L1 约 100 token, L2 低于 5000 token; 引用一层深 (M7 M9) |
| Optimizing skill descriptions (Agent Skills) | https://agentskills.io/skill-creation/optimizing-descriptions | 约 20 条正反查询, 每条 3 次, 阈值 0.5, 60/40 训练验证拆分, 近似反例最有价值 (M6) |
| Evaluating skill output quality (Agent Skills) | https://agentskills.io/skill-creation/evaluating-skills | 有无 skill 基线对比; 断言要可验证; PASS 需证据; 能用脚本查的用脚本 (M6 N1) |
| Equipping agents for the real world with Agent Skills (Anthropic Engineering) | https://www.anthropic.com/engineering/equipping-agents-for-the-real-world-with-agent-skills | 三级渐进披露; 先从评估出发; 互斥内容分文件; 脚本既可执行也可当文档 (M5 N11) |
| Evergreen notes (Andy Matuschak) | https://notes.andymatuschak.org/Evergreen_notes | 原子, 概念导向, 密集链接, 关联优于层级 (N9) |
| Evergreen notes should be atomic (Andy Matuschak) | https://notes.andymatuschak.org/zNUaiGAXp21eorsER1Jm9yU | 原子性与关注点分离类比, 过碎会割裂链接网 (N9 的链接下限只设一条) |
| Introducing Contextual Retrieval (Anthropic) | https://www.anthropic.com/news/contextual-retrieval | 切块丢失上下文会让检索失败, 给块补上下文可降低失败率 (N9: `.hx-info.md` 每块自包含的价值) |
| Docs as code (Write the Docs) | https://www.writethedocs.org/guide/docs-as-code/ | 文档用版本控制, 代码审查与自动化测试, 可在缺文档时阻止合并 (N5 把 cover 放进闸门) |
| Diátaxis | https://diataxis.fr/ | 教程, 操作指南, 参考, 解释四种文档分开组织 (N11: 升级后 skill 放操作, 笔记留解释) |
| DSH skill loader 对 `disable-model-invocation` 的处理 | `[未核实]` 本次未能读到 DSH 源码 | M2 N3 的风险项 |
