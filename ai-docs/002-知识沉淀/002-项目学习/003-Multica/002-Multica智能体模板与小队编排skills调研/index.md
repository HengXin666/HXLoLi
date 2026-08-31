---
title: "Multica智能体模板与小队编排skills调研"
created_at: "2026-07-05"
model: "GPT-5 Codex"
skill: ["hx-make-ai-docs", "mp-research"]
authors: "Heng_Xin"
tags: ["Multica", "AI Agent", "Skill", "工程协作"]
---

# Multica智能体模板与小队编排skills调研

> [!NOTE]
> 本文由 AI 辅助沉淀, 需要用户 review 后再提交.

## 0x00 背景

这次调研的直接目标是: 为 Multica 工作区设计一组可复用的 skill, 用来创建智能体、选择和绑定能力、组建小队、编排 issue/stage/autopilot, 并让这些能力真正做到可定制、可快速集成、持续可靠可维护.

现有参考是 `/home/hx/Loli/code/HXLoLis/ref/HX-AiKaNaRaZu`. 它已经有两个核心 skill:

- `hx-init`: 给目标项目安装 AI Coding 强约束, 包含 hooks、规则文档、Python/React 校验模板.
- `hx-libs-sentaku`: 按 Python 后端、React 前端、数据库三类读取选型参考.

但用户指出它还没有达到交付级: 当用户告诉 AI "可用技术栈"时, AI 容易把所有技术栈都当成必须使用, 而不是根据项目实际情况做取舍; 用户也不确定 hooks 是否在各类 runtime 下稳定触发.

因此本文不只整理资料, 还要给出下一步设计方向: Multica skill 应该把"平台契约"和"工程决策"分开. 平台契约回答什么会触发 agent、squad、autopilot; 工程决策回答在当前项目里到底该装什么、用什么、跳过什么.

## 0x01 核心结论

### 1.1 公开 Multica 资料不多, 但主仓和官方文档足够作为一手来源

本次搜索到的可验证公开资料主要是:

- GitHub 主仓: [`multica-ai/multica`](https://github.com/multica-ai/multica), README 将 Multica 定位为 open-source managed agents platform, 支持 issue 分配、agent teammate、squad、autopilot、reusable skills、runtime/daemon.
- 官方站点与文档: [`multica.ai`](https://multica.ai/) 和 [`multica.ai/docs`](https://multica.ai/docs).
- 当前 workspace 内置 Multica skills: `multica-creating-agents`, `multica-squads`, `multica-mentioning`, `multica-autopilots`, 这些是本次最贴近实际行为的契约说明.
- `multica skill search multica --output json` 能搜到少量 ClawHub 条目, 如 `Multica Manager` 和 `Multica Sdd Workflow`, 但 install_count 为 0, 描述质量不稳定, 只能作为竞品样例, 不能直接作为可靠模板来源.

没有找到大量独立第三方 Multica 文章或教程. 所以当前阶段应以官方 repo/docs、本地 CLI contract、已有 issue 沉淀为主, 外部同类项目只作为架构模式参考.

### 1.2 Multica 编排的稳定底座不是"互相 @", 而是 issue + stage + leader routing

Multica 里 skill 本身不是调度器. 它只提供上下文和工作规则. 真正会产生任务的机制是:

| 场景 | 机制 | 关键边界 |
| --- | --- | --- |
| 创建 agent | `multica agent create` + `agent skills add/set` | `description` 是展示元数据, `instructions` 才是 runtime 行为契约 |
| 创建 squad | `multica squad create --leader ...` | squad 不会全员 fan-out, 工作路由到 leader |
| 临时触发 agent | `mention://agent/<uuid>` | 会入队 agent run, 不能用于寒暄或结束语 |
| 临时触发 squad | `mention://squad/<uuid>` | 实际触发 squad leader |
| 串并行任务 | 子 issue + `--stage` + `todo/backlog` | 比 agent 互相 @ 更可恢复、可审查 |
| 周期任务 | autopilot schedule/webhook/manual | `create_issue` 更可追踪, `run_only` 要额外指定汇报位置 |

因此应该设计"创建与编排 skill", 而不是设计"agent 互相喊人 skill".

### 1.3 当前最值得交付的不是一个万能 skill, 而是一组小而硬的 skill

推荐拆成 5 个互补 skill:

| Skill | 目标 | 主要产物 |
| --- | --- | --- |
| `hx-agent-architect` | 根据目标创建或调整 agent | agent instructions 草案、skill 绑定建议、创建命令 |
| `hx-squad-orchestrator` | 组建 squad 并拆分 staged issue | squad leader 策略、成员角色、stage 计划 |
| `hx-skill-curator` | 搜索、评估、导入 workspace skill | 候选 skill 清单、来源可信度、冲突策略 |
| `hx-project-guardrails` | 改造 `hx-init`, 给项目安装强约束 | 扫描报告、安装提案、hooks 可用性矩阵 |
| `hx-autopilot-maintainer` | 设计周期性审计/维护任务 | autopilot create/update 提案、运行模式建议 |

这组 skill 要共享一个原则: 用户给出的"可用技术栈"是候选输入, 不是必须全部采用的需求. 每个 skill 都要先把输入拆成 `must_use`、`already_present`、`optional`、`reject_or_defer` 四类, 再决定行动.

### 1.4 hooks 只能作为辅助质量门, 不能作为唯一可靠性边界

`HX-AiKaNaRaZu` 的 hooks 已经做了几个正确选择: 输出摘要, 完整日志进 `.git/hx-init/logs`; Stop hook 检测递归; format hook 只处理对应语言. 但 hook 可靠性仍受以下因素影响:

- 不同 provider 的 hook 机制不同. 现有模板主要覆盖 Claude/Codex 风格, 对其他 Multica runtime 不能默认等价.
- provider 是否会读取 `.claude/`, `.codex/`, `.agent_context/` 取决于 daemon 写入路径和 CLI 自身约定.
- Stop hook 是强阻断能力, 在历史债较多项目里容易误伤. 大项目默认应该 `baseline` 或 `advisory`.
- hook 触发本身需要 smoke test: 同一套模板至少要验证 Claude Code、Codex 两条路径是否真实执行 PostToolUse/Stop.

所以交付标准不应写成"装了 hook 就稳定", 而应写成"安装后生成 runtime hook matrix, 每个 runtime 标记 supported / advisory / unsupported / unverified".

## 0x02 关键细节

### 2.1 市面同类模式给 Multica 的启发

| 来源 | 可借鉴点 | 对 Multica skill 的落地方式 |
| --- | --- | --- |
| Agent Skills 标准 | `SKILL.md` + supporting files, 按需加载, 渐进披露 | 每个 skill 只解决一个工作流, 大参考放 `references/` |
| Claude Code skills/subagents | agent 有明确边界, 可配置工具和模型; skill 通过 frontmatter 描述触发 | agent instructions 要写职责边界, 不要只写"你很强" |
| OpenAI Codex `AGENTS.md`/subagents | repo instructions 与专门 subagent 分层 | Multica agent instructions、repo `AGENTS.md`、workspace skill 要分工 |
| CrewAI Crews/Flows | crew 适合协作角色, flow 适合确定性状态机 | Multica squad 负责路由, stage/autopilot 负责状态推进 |
| LangGraph | 状态、checkpoint、human-in-loop、可观测性是长流程核心 | Multica 长流程必须用 issue/stage/metadata/comment 留痕 |
| `obsidian-second-brain` | 一个源头编译到多平台, 命令 + references + scripts 分层 | HX skill 包也应维护单一源和多 runtime 适配表 |
| `gpt-docs` | 调研/决策/playbook/prompt 分目录沉淀 | Multica skill 设计也要拆 research、decision、playbook、prompt |

结论: Multica 的优势是平台已经有 issue、agent、squad、autopilot 和 runtime. 不需要把 CrewAI/LangGraph 这类框架搬进来; 需要学习的是它们的显式状态、角色边界和可观察性.

### 2.2 `HX-AiKaNaRaZu` 当前状态评估

现有优点:

- `hx-init` 已经强调"先扫描, 再提案; 用户确认前不得安装".
- Python/React 参考都要求复用现有包管理器、lint/typecheck/test, 不整文件覆盖.
- hooks 失败时输出摘要和日志路径, 没有把全量错误塞回上下文.
- `install.sh` 支持全局/项目安装, `--force` 明确替换.

主要缺口:

1. 技术栈选择过于静态. `hx-libs-sentaku` 只是路由到 Python/DB/React 文档, 没有要求 agent 判断"用户给的栈是否真的适合当前项目".
2. 缺少 agent/squad 交付对象. 现有模板是项目 guardrails, 不是 Multica agent template.
3. hook 可靠性没有矩阵. 文档没有告诉用户哪些 runtime 已验证、哪些只是 best effort.
4. 缺少 skill 来源治理. 没有定义从 `multica skill search`、GitHub、ClawHub 导入时的可信度、冲突策略、版本锁定和回滚.
5. 缺少持续维护闭环. 没有 autopilot 定期检查 hook、skill URL、agent/squad 配置是否腐化.

### 2.3 关键设计: "可用技术栈"四分类

为解决"AI 把所有可用栈都用上"的问题, 所有创建/初始化类 skill 都应内置下面的判断:

| 分类 | 含义 | 行为 |
| --- | --- | --- |
| `must_use` | 用户明确要求或项目已经强依赖 | 必须纳入方案, 若有风险先说明 |
| `already_present` | 仓库事实证明已经在用 | 优先复用, 不重复引入替代品 |
| `optional` | 用户说可用, 但当前目标不需要 | 只记录为候选, 不安装、不写进强约束 |
| `reject_or_defer` | 与目标冲突、维护成本过高或证据不足 | 明确拒绝或延后, 给出原因 |

示例规则:

```text
用户说: 可用 Python、React、PostgreSQL、Redis.
仓库扫描: 只有 Vite React 前端, 无后端, 无数据库访问.
正确结论: React/TypeScript 是 already_present; Python/PostgreSQL/Redis 是 optional 或 defer.
错误结论: 自动创建 FastAPI + PostgreSQL + Redis 全家桶.
```

这条规则应写进 `hx-agent-architect` 和 `hx-project-guardrails` 的不可跳过原则.

### 2.4 推荐 skill 1: `hx-agent-architect`

用途: 创建或改造 Multica agent.

必须做:

- 先读需求、runtime、项目资源和现有 skill, 生成提案.
- 区分 `description` 与 `instructions`: 前者是展示摘要, 后者才是 runtime 行为契约.
- 推荐 skill 绑定时, 说明每个 skill 解决什么问题、为什么不选其他 skill.
- secrets 只允许 `--custom-env-stdin` 或 `--custom-env-file`, 不建议 inline.
- `agent skills add` 默认优先于 `set`; 只有用户要求替换全部绑定时才用 `set`.

提案输出形态:

```md
## Agent Proposal

- name:
- description:
- runtime:
- model/thinking:
- responsibility:
- boundaries:
- required skills:
- optional skills:
- commands to run:
- validation:
```

### 2.5 推荐 skill 2: `hx-squad-orchestrator`

用途: 创建小队、分配成员、拆 issue 阶段.

必须写入的事实:

- squad 不是 agent, squad-routed work 会路由到 `leader_id`.
- member role 只是 leader briefing 的上下文, 不产生自动调度或权限.
- squad mention/assignment/autopilot 都触发 leader, 不 fan-out.
- 并行/串行用 issue children + `--stage`, 不靠 agent 互相 @.

推荐流程:

1. 确定目标: 这个 squad 是长期组织, 还是单个项目的临时编排?
2. 选择 leader: leader 应该是能做规划、验收、创建子 issue 的 agent.
3. 选择成员: 每个成员只绑定能解释其能力的 skill.
4. 生成 stage plan: 每个子 issue 都有输入、输出、验收标准.
5. 用户确认后再执行 `multica squad create` / `member add` / `issue create`.

### 2.6 推荐 skill 3: `hx-skill-curator`

用途: 搜索、评估、导入 skill.

本次实测:

```bash
multica skill search multica --output json
multica skill search agent --output json
multica skill search review --output json
```

观察:

- `multica` 查询返回 `Multica Manager`、`Multica Sdd Workflow` 等 ClawHub 条目, 但 install_count 为 0.
- `review` 查询返回不少 review/code review/architecture review 条目, 但同名 `Review` 较多, 需要人工区分.
- 搜索耗时约 10 秒以上, 不适合每次 agent 创建都实时多次查询.

设计建议:

- 先维护 curated allowlist, 每条记录 `name/source_url/tags/适用场景/风险/最近验证时间`.
- 允许实时 `multica skill search`, 但 LLM 不得自由编造 URL.
- 导入默认用 `--on-conflict skip` 或 `rename`, 避免覆盖用户改过的 workspace skill.
- 每个导入后的 skill 必须绑定到具体 agent 才会在任务里生效; 创建 skill 不等于 agent 会使用它.

### 2.7 推荐 skill 4: `hx-project-guardrails`

用途: 作为 `hx-init` 的升级版, 给项目安装强约束.

新增硬规则:

- 先扫描项目事实, 再解释技术栈分类.
- 对每个 hook 给出 provider support:

| Runtime | PostToolUse | Stop | Skill path | 状态 |
| --- | --- | --- | --- | --- |
| Claude Code | 已有模板 | 已有模板 | `.claude/skills` | 需 smoke test |
| Codex | 已有模板 | 已有模板 | Codex home skills | 需 smoke test |
| OpenCode/Cursor/Pi 等 | 依 daemon/provider 约定 | 不确定 | provider 原生目录或 fallback | 不承诺阻断 |

- 大项目默认 `baseline/advisory`, 不默认 strict.
- 安装后必须输出"实际写入文件 + 跳过文件 + 验证命令 + 日志路径".

### 2.8 推荐 skill 5: `hx-autopilot-maintainer`

用途: 给稳定重复任务创建 autopilot, 如每日 triage、每周 skill URL 检查、每周 hook smoke test.

规则:

- 默认推荐 `create_issue`, 因为结果可追踪、可评论、可审查.
- 只有任务确实不需要 ticket 时才用 `run_only`, 且必须指定输出到哪里.
- 对 squad-assigned autopilot, 要提醒实际执行者是 squad leader.
- webhook token 不得写入评论、文档或日志.

### 2.9 建议目录结构

可以在 `HX-AiKaNaRaZu` 中逐步演进为:

```text
skills/
  hx-agent-architect/
    SKILL.md
    references/
      agent-instructions-template.md
      tech-stack-classifier.md
  hx-squad-orchestrator/
    SKILL.md
    references/
      staged-issue-template.md
      squad-leader-policy.md
  hx-skill-curator/
    SKILL.md
    references/
      curated-skills.json
      trust-rubric.md
  hx-project-guardrails/
    SKILL.md
    references/
      _shared/
      python/
      react/
      runtime-hook-matrix.md
  hx-autopilot-maintainer/
    SKILL.md
    references/
      autopilot-template.md
templates/
  agents/
    code-reviewer.json
    docs-writer.json
    squad-leader.json
  squads/
    research-review-squad.json
    web-feature-squad.json
```

如果只做第一版, 推荐先交付 `hx-agent-architect`、`hx-squad-orchestrator`、`hx-skill-curator`. 它们能直接解决用户最关心的"创建智能体以编排小队"问题.

### 2.10 分阶段落地计划

**Phase 0: 只写 skill, 不改 Multica 产品代码**

- 在 `HX-AiKaNaRaZu` 增加 3 个 skill: `hx-agent-architect`, `hx-squad-orchestrator`, `hx-skill-curator`.
- 用现有 CLI 完成交付: `multica agent create`, `agent skills add`, `squad create`, `squad member add`, `issue create --stage`, `skill search/import`.
- 每个 skill 都必须先给 proposal, 用户确认后才执行 side effect.

**Phase 1: 模板化**

- 用静态 JSON/YAML 描述 agent template 和 squad template.
- 对齐 Multica 主仓 `docs/agent-quick-create-plan.md` 的 Template 思路: template = instructions + skill 引用.
- 冲突策略优先 reuse/skip, 不覆盖用户已有 skill.

**Phase 2: AI 推荐 skill**

- 先用 curated allowlist, 后续再接实时 search.
- 当前 `multica skill search` 已存在, 但实时结果较慢且质量不稳定, 不应让 LLM 自由选择任意 URL.
- 推荐结果必须说明"为什么选"和"为什么没选".

**Phase 3: AI 创建 agent/squad**

- 只在用户确认后运行创建命令.
- 创建后进入 review 状态: 用户检查 instructions、skill bindings、runtime、权限、autopilot 后再正式使用.

### 2.11 验收标准

第一版可以用以下场景验收:

1. 用户说"可用 Python/React/PostgreSQL/Redis, 帮我做前端页面 agent". 输出中不能默认安装 FastAPI/PostgreSQL/Redis.
2. 用户说"组一个前端小队". 输出必须说明 squad leader-only routing, 并创建 staged 子 issue, 而不是 @ 所有成员.
3. 用户说"找 code review skill". 输出至少包含来源、可信度、冲突策略, 不直接导入低可信 skill.
4. 安装 hooks 后, 文档必须给出 Claude/Codex smoke test 结果或标记未验证.
5. 任一命令涉及 secrets, 必须推荐 stdin/file, 不允许 inline.
6. 最终评论或报告不包含真实 `mention://agent/<uuid>`, 除非明确要触发 agent.

## 0x03 验证与引用

### 3.1 本次执行过的关键命令

Multica issue 与上下文:

```bash
multica issue get 0b9b97d6-9c23-4531-833b-959584455767 --output json
multica issue metadata list 0b9b97d6-9c23-4531-833b-959584455767 --output json
multica issue comment list 0b9b97d6-9c23-4531-833b-959584455767 --recent 10 --output json
```

公开仓库与 skill 搜索:

```bash
gh search repos multica --limit 20 --json fullName,description,url,stargazersCount,updatedAt,language,isArchived
gh repo view multica-ai/multica --json description,homepageUrl,licenseInfo,stargazerCount,updatedAt,url
gh api repos/multica-ai/multica/contents/README.md --jq '.content' | base64 -d
multica skill search multica --output json
multica skill search agent --output json
multica skill search review --output json
```

本地参考:

```bash
sed -n '1,260p' ref/HX-AiKaNaRaZu/skills/hx-init/SKILL.md
sed -n '1,260p' ref/HX-AiKaNaRaZu/skills/hx-libs-sentaku/SKILL.md
sed -n '1,220p' ref/obsidian-second-brain/architecture.md
sed -n '1,220p' ref/gpt-docs/AGENTS.md
```

### 3.2 本文初始化命令

本文按 `hx-make-ai-docs` 约束先用模板脚本初始化:

```bash
XDG_CACHE_HOME=/tmp/uv-cache uv run .agents/skills/hx-make-ai-docs/scripts/makeDoc.py \
  --title "Multica智能体模板与小队编排skills调研" \
  --tag "Multica" \
  --tag "AI Agent" \
  --tag "Skill" \
  --tag "工程协作" \
  --model "GPT-5 Codex" \
  --skill "hx-make-ai-docs + mp-research" \
  --author "Heng_Xin" \
  --output "ai-docs/002-知识沉淀/002-项目学习/003-Multica/002-Multica智能体模板与小队编排skills调研/index.md"
```

### 3.3 主要引用

Multica 官方/公开资料:

- [`multica-ai/multica` README](https://github.com/multica-ai/multica/blob/main/README.md)
- [Multica Docs](https://multica.ai/docs)
- [Multica Docs: Skills](https://multica.ai/docs/skills)
- [Multica Docs: Squads](https://multica.ai/docs/squads)
- [Multica Docs: Mentioning Agents](https://multica.ai/docs/mentioning-agents)
- [Multica Docs: Autopilots](https://multica.ai/docs/autopilots)
- [`docs/product-overview.md`](https://github.com/multica-ai/multica/blob/main/docs/product-overview.md)
- [`docs/agent-quick-create-plan.md`](https://github.com/multica-ai/multica/blob/main/docs/agent-quick-create-plan.md)
- [`CLI_AND_DAEMON.md`](https://github.com/multica-ai/multica/blob/main/CLI_AND_DAEMON.md)

同类模式参考:

- [Agent Skills](https://agentskills.io/)
- [Anthropic Claude Code Skills](https://docs.anthropic.com/en/docs/claude-code/skills)
- [Anthropic Claude Code Subagents](https://docs.anthropic.com/en/docs/claude-code/sub-agents)
- [OpenAI Codex AGENTS.md](https://developers.openai.com/codex/guides/agents-md)
- [OpenAI Codex Subagents](https://developers.openai.com/codex/subagents)
- [CrewAI Crews](https://docs.crewai.com/concepts/crews)
- [CrewAI Flows](https://docs.crewai.com/concepts/flows)
- [LangGraph Overview](https://docs.langchain.com/oss/python/langgraph/overview)

本地资料:

- `/home/hx/Loli/code/HXLoLis/ref/HX-AiKaNaRaZu/README.md`
- `/home/hx/Loli/code/HXLoLis/ref/HX-AiKaNaRaZu/skills/hx-init/SKILL.md`
- `/home/hx/Loli/code/HXLoLis/ref/HX-AiKaNaRaZu/skills/hx-libs-sentaku/SKILL.md`
- `/home/hx/Loli/code/HXLoLis/ref/obsidian-second-brain/architecture.md`
- `/home/hx/Loli/code/HXLoLis/ref/gpt-docs/AGENTS.md`
- `multica-creating-agents`, `multica-squads`, `multica-mentioning`, `multica-autopilots` 内置 skill 文档.

### 3.4 仍需用户确认的一个问题

下一步如果要从调研进入实现, 我推荐先做 **Phase 0: 只在 `HX-AiKaNaRaZu` 增加 3 个 skill (`hx-agent-architect`, `hx-squad-orchestrator`, `hx-skill-curator`), 暂不改 Multica 产品代码**.

推荐理由: 这能最快验证"可定制创建 agent + squad 编排"是否解决真实问题, 并且不会碰 Multica 主仓产品功能. 等 skill 工作流稳定后, 再考虑把 template/skill finder 做进 Multica 产品.
