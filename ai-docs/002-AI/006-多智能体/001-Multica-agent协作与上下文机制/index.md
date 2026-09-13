---
hxid: "hx-6ab86c81"
title: "Multica agent协作与上下文机制"
created_at: "2026-07-05"
model: "GPT-5 Codex"
skill: ["hx-docs-sediment"]
authors: "Heng_Xin"
tags: ["多智能体", "Multica", "AI Agent"]
---

# Multica agent协作与上下文机制

> [!NOTE]
> 在一个工作区里让一个 agent 去唤起另一个 agent 继续干活, 听起来只需要一句 `@`; 但它最容易长成的却是停不下来的成本循环 —— 谁该被叫醒, 被叫醒时它究竟带着哪些上下文?
> 更反直觉的是: 决定一次运行的并不是上一条评论, 而是 runtime 为这次触发临时注入的任务 brief. agent 一旦凭"上次聊到哪"继续发挥, 就会和协作系统的真实状态悄悄错位.
> 那么, 如果 skill 本身不是调度器, 一次可靠的委派究竟该由谁、依据什么来触发?

## 0x00 背景

这次讨论从一个实践问题开始: 在 Multica 工作区里, 一个 agent 应该如何触发另一个 agent 继续工作? 用户进一步追问了两个更底层的问题:

- 是否存在一个能让 agent 互相反复 `@` 的 skill.
- 每次 issue 评论触发 agent 时, 上下文究竟是如何确定的, 是否只是 `codex --continue <conversation_id>`.

这些问题背后真正需要沉淀的是一套可维护的 agent 协作模型. 如果把 agent 协作理解成"互相 @ 接力", 很容易形成成本循环、重复触发和上下文误判. 如果把它理解成"issue 任务、runtime brief、skills、资源指针和平台触发规则共同决定的一次任务运行", 就能设计出更稳定的编排方式.

本文沉淀的是当前讨论得到的阶段性结论: Multica 里的 agent 协作不应依赖无边界互相 mention, 而应以父 issue 规划、staged 子 issue、明确验收标准和一次性委派为主.

## 0x01 核心结论

最重要的结论是: skill 本身不是触发机制. 触发 agent 的机制来自 Multica 平台对任务、评论 mention、issue assignee/status 和 autopilot 的处理; skill 只能约束 agent 在什么条件下、以什么格式、安全地使用这些机制.

可靠的协作方式可以按以下优先级理解:

| 场景 | 推荐机制 | 关键边界 |
| --- | --- | --- |
| 临时委派一个具体任务 | 评论里使用真实 `mention://agent/<uuid>` | 会产生 side effect, 不能用于感谢、确认或结束语 |
| 临时委派一个团队 | 评论里使用 `mention://squad/<uuid>` | 实际触发 squad leader, 不是所有成员同时跑 |
| 可维护的多 agent 流程 | 创建子 issue, 设置 assignee 和 `todo`/`backlog`/`stage` | 适合串行、并行、阶段屏障和验收 |
| 定时或 webhook 工作 | autopilot | 不应靠 agent 互相 @ 实现定时循环 |
| 可复用工作流规则 | 编写安全委派 skill | skill 规定行为, 不直接触发平台任务 |

不建议设计"agent 反复相互 @"的 skill. 更合理的是设计"单次委派/编排 skill", 例如:

```text
delegate-once
- 只有存在明确交付物时才允许触发下游 agent
- 每次最多触发 1 个 agent 或 1 个 squad
- 禁止 @ 刚刚触发自己的 agent
- 必须写清任务输入、输出路径、验收标准
- 必须有 max_rounds 或 stop condition
- 优先创建子 issue + assignee + status todo
- 串行流程用 stage/backlog, 不靠互相 @ 接力
```

一句话总结: 临时喊人用 agent mention; 可复用流程用 staged 子 issue; 定时或外部事件用 autopilot; skill 用来约束 agent 的安全行为, 不替代平台调度.

## 0x02 关键细节

### 2.1 agent mention 的副作用

评论中的 agent mention 不是普通 Markdown 链接. 平台会解析 `mention://agent/<uuid>` 并入队一条对应 agent 的 comment task. 因此在说明文档或总结里应使用占位符, 不要随意贴真实 agent mention 链接.

需要区分几类 mention:

| 类型 | 效果 |
| --- | --- |
| `mention://agent/<uuid>` | 触发对应 agent 运行 |
| `mention://squad/<uuid>` | 触发 squad leader 运行 |
| `mention://member/<uuid>` | 通知人类成员, 不触发 agent |
| `mention://issue/<uuid>` | 渲染为 issue 链接, 不触发 agent |

已知限制也要保留在设计里:

- 目标 agent 已有同 issue pending task 时, 平台可能跳过新触发.
- 目标 agent 被归档、没有权限或 UUID 错误时, 可能静默不触发.
- `@all` 是广播语义, 不是指定 agent 跑任务的编排机制.
- agent 回复另一个 agent 时默认不应再 mention 对方, 否则容易形成循环.

### 2.2 子 issue/stage 更适合编排

多 agent 工作流不应靠评论互相接力, 而应把任务拆成可验收的子 issue:

1. manager agent 在父 issue 里做规划.
2. 创建多个子 issue, 每个子 issue 明确输入、输出和验收标准.
3. 需要立即启动的子 issue 使用 `--status todo`.
4. 需要等待上游完成的子 issue 使用 `--status backlog`.
5. 多阶段流程使用 `--stage 1/2/3` 表达屏障顺序.
6. worker agent 完成后只汇报结果, 不再主动 @ 回 manager.

这种方式的优势是状态可见、任务可恢复、结果可审查, 也更容易避免隐式循环.

### 2.3 runtime 上下文不是简单续聊

当前讨论明确了一个容易误解的点: issue 里评论触发 agent, 不是简单等价于 `codex --continue <conversation_id>`.

更准确的链路是:

1. 用户在 issue 评论中 mention agent.
2. Multica 后端解析 mention, 创建带有 `issue_id`、`trigger_comment_id`、任务类型和运行资源的 agent task.
3. 本地 daemon 认领任务.
4. daemon 读取 agent 持久化配置、project context、资源指针、issue 信息和绑定 skills.
5. runtime 注入任务 brief, 明确本次触发评论、必须读取的 issue/thread、回复 parent comment、可用 CLI 和平台规则.
6. agent 再主动调用 `multica issue get`、`multica issue comment list`、`multica issue metadata list` 等命令获取事实.

因此每次 agent 运行都应该以"本次触发评论"为主, 而不是凭上一轮聊天记忆继续发挥. 这也是为什么 Multica runtime 会反复强调: 先读 issue, 再读触发线程, 最终回复必须发回指定 parent comment.

### 2.4 agent 能力来自多层约束

agent 知道自己能做什么, 不是来自单一 prompt. 至少有以下层:

| 来源 | 作用 |
| --- | --- |
| agent row | 持久化 instructions 和身份信息 |
| runtime brief | 注入本次任务、issue、评论、metadata、mention、安全规则 |
| bound skills | 提供特定工作流, 如研究、代码审查、笔记沉淀、Multica 平台操作 |
| project context/resources | 告诉 agent 当前项目是什么、有哪些本地目录或仓库资源 |
| repo instructions | `AGENTS.md`、`CLAUDE.md` 等仓库级工程约束 |
| runtime permissions | 文件系统、网络、CLI、sandbox 等实际能力边界 |

一个可维护的 agent 协作系统, 应把这些层次当作契约, 而不是让 agent 自己猜上下文.

### 2.5 对 HXLoLi 笔记工作流的启发

这次沉淀本身也验证了一个模式: agent 可以通过 Multica CLI 读取 issue 讨论, 然后把阶段性结论写入 HXLoLi 的 `ai-docs` 工作区.

但需要保留两个边界:

- 写入其他工作区时, agent 的文件权限必须允许对应路径; 否则需要用户授权或在目标 workspace 直接触发.
- 讨论沉淀应先形成阶段性成果, 不应在信息不足时直接写成确定性手册. 对不确定方向, 应一次只追问一个关键问题.

## 0x03 拓展升华展望

Multica 这套机制的价值不在"多智能体"这个名号, 而在于它把协作的责任分给了三层不同的东西: **平台负责触发与调度, 契约负责可见性与权限, 文档负责让下一次委派有据可依**.

**事实层面** … 本文描述的机制全部来自平台当前的运行时行为: skill 不是调度器, 真正产生任务的是 mention、assignee/status/stage 与 autopilot; 每次触发的上下文由 runtime brief 注入, agent 需要靠 CLI 主动读取 issue 与触发线程; 而 agent 的能力边界由 instructions、skills、仓库工程约束、project resources 与 runtime permissions 共同决定. 这些层次目前都已存在, 且可以用一条 CLI 命令观察到.

**个人判断** … agent 协作接下来最稀缺的能力不是"能互相叫醒", 而是**可审计的委派**: 一次触发留下了什么输入、按什么验收、由谁验收. 一旦缺少这层留痕, 编排规模越大, 系统反而越难解释自己为什么花了这些成本. 另一个正在成形的方向是**能力发现** —— 当 agent 数量变多, "我现在有哪些 agent、各自能干什么、该找谁"会先于"怎么写循环"成为瓶颈; 这也是下一步最值得投入的地方.

## 0x04 参考来源

- [multica-ai/multica](https://github.com/multica-ai/multica) —— 开源托管 agent 平台主仓; agent、squad、autopilot 与 runtime/daemon 的平台契约出处.
- [Multica Docs](https://multica.ai/docs) —— 官方文档入口; 触发方式、mention 语义与平台规则的核对依据.
