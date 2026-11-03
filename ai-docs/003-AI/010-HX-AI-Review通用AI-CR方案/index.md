---
hxid: "hx-10e4a03c"
title: "HX-AI-Review 通用 AI CR 方案"
created_at: "2026-10-28"
model: "deepseek-v4.1-flash"
skill: ["hx-note"]
authors: "Heng_Xin"
tags: ["上下文工程"]
---

本文是 HX-AI-Review 下一版 code 审查的实现规格. 以 hx-context-review 0.3.0 为起点, 首个落地版本是 v0.4. 文中 "AI CR" 默认指 code 审查 (kind=code, 规则 CODE-AI); context 审查是另一种独立审查, 只在共用设施处出现.

## 0x00 触发场景

这套规格只给一种项目用: 已接入 HX-AI-Review 的 TypeScript + GitHub Actions 业务仓库. 由 PR 触发 code 审查. 审查对象是 diff 的新增行. 审查者额外拿到仓库在 merge-base 上的上下文文件, 作为背景.

以下情形不适用:

- 需要跨服务按接口字段追踪消费方的仓库. 契约轴不在本方案内.
- 以 C++, 构建图, proto, clangd, MR 平台为前提的仓库.
- 审查上下文文件本身的质量. 那是 context 审查的职责.

通用指: 接入一个新仓库只靠配置与仓库内已有的上下文文件, 不改 runner 代码, 不假设语言栈.

### 接入必填项

| 项 | 放在哪里 | 取值 |
|---|---|---|
| .hx-context-review.json | 被审仓库根目录 | 仓库配置文件, 沿用 v0.3 格式 |
| HX_CTX_TOKEN | 被审仓库 secret | 仓库侧唯一 secret, 沿用 v0.3 |
| 总览页仓库登记 | cf-admin 总览页 | repos.kind 取 code 或 both |
| code 审查 job | 被审仓库 workflow | kind=code 的独立 job, 与 context 审查的 job 分开 |
| 上下文文件 | 被审仓库任意目录 | 可选; AGENTS.md, README.md, SKILL.md, docs/ 下的 .md; 没有时背景为空并在运行记录里写明 |

### 前提缺失时的降级

| 缺失的前提 | 降级做法 | 落在哪里 |
|---|---|---|
| 没有构建图 | 空间范围用目录祖先关系, 代码邻居用本地 import 边与 git grep 引用方 | context-cr/src/scope/index.ts, context-cr/src/review/evidence.ts |
| 没有 LSP | 由只读工具 read/grep/find/ls 取上下文, 由 verifier 新会话复核 | context-cr/src/review/tools.ts |
| 没有 proto | 契约轴不做, 字段级消费方属方案外 | 见本文 "非目标" |
| 不是 PR 触发 | base 回退 origin/<default_branch> | context-cr/src/scope/base.ts |

## 0x01 契约

下面七条换掉任何一条, 整套做法就失效. 每条后面是能跑或会失败的判据.

### K1 Error 只来自确定性检查, AI 发现恒为 Warning

- 规则: 只有 severity=error 且 origin=new 的发现让 job 失败; CODE-AI 永远是 Warning, 不阻塞合并.
- 判据: grep -n "isBlocking" context-cr/src/contract/report.ts 只命中一处定义, 条件是 error 且 new; 构造一份只含 CODE-AI 发现的报告, counts.blocking 必须为 0.

### K2 每条发现带 file:line, line 是 hunk 新侧行号

- 规则: 候选必须指向本批文件里真实存在的新增行, 非抑制, 非重复; 删除行没有新侧行号, 永不报告.
- 判据: 在夹具仓库里让 finder 报一个不存在的行号, 确定性闸门 (context-cr/src/review/index.ts gate) 必须丢弃它, 不花 verifier 会话.

### K3 只有运行完成才能解决发现

- 规则: 运行完成 = AI 未中断, 候选队列清空, notReviewed 为空. 不完整的运行不推进 last_sha, 不把任何 AI 规则 (CX-AI, CODE-AI) 的发现标 resolved, 报告标覆盖不足且不得写 "无问题".
- 判据: 在 cf-admin 测试里提交一次 complete=false 的 code 运行, resolved 必须为 0; 提交一次 notReviewed 为 2 的运行, 报告 coverage.complete 必须为 false.

### K4 背景只取 base 侧, 以数据身份进入 user 消息

- 规则: 背景是 computeScope 从改动路径反推出的上下文文件在 merge-base 上的内容; 本 PR 改动的上下文文件不当背景; 背景以引用块放进 user 消息, 标注 "仓库约定, 供参考", 不进系统提示; isolatedLoader 保持全部返回空.
- 判据: 对一个同时改 src/x/AGENTS.md 与 src/x/a.ts 的 PR, sessions.jsonl 里 code finder 的 user 消息中引用块内容等于 git show <base>:src/x/AGENTS.md, 系统提示中不出现该文件内容; git diff 显示 context-cr/src/review/pi.ts 未改动.

### K5 驳回反馈按 kind 隔离, 仓库维度默认只进本仓

- 规则: 抑制表按 (仓库, 指纹); prompt 附录与驳回笔记按 kind 存, 默认只对本仓生效, 管理员 fold 时可显式提升为全局并留痕.
- 判据: 在 code 运行里驳回一条并 fold, 下一次 context 运行与另一仓库的 code 运行, 其 system prompt 都不含该驳回理由.

### K6 确认环节不接收发现者的论证

- 规则: verifier 的输入只有候选的 path, line, category 与 evidence 里的 file:line 位置, finder 的推理文字不进入.
- 判据: 取任一 verifier 会话的首条消息, 其中不含 finder 会话写下的推理文字.

### K7 输出稳定

- 规则: 报告按 path, line, ruleId, fingerprint 排序, 不含时间戳; AI 发现的指纹为 fingerprint(ruleId, path, category, 该行去首尾空白后的内容).
- 判据: 同一输入跑两次, 报告 findings 顺序逐字相同; 只改 title 措辞而不改代码, 指纹不变.

### 与 v0.3 基线的关系

| 基线 | 保持/改动 | 理由 |
|---|---|---|
| 双 CR kind (`2026-10-25-dual-cr-kind`) | 改动 | kind 维度与两个 job 保持; 该篇的提示词附录共享改为按 kind 隔离, 因为独立审查要求驳回反馈按 kind 分开 |
| pi 在 runner 直连模型 (`2026-10-11-pi-runner-direct-model`) | 保持 | 背景在 runner 内拼装, 不经 CF |
| CF 只管配置, 存储与总览 | 保持 | 新增字段都是存储与下发, 不在 CF 上计算 |
| Error 只来自确定性检查 | 保持 | 见 K1; `2026-10-18-rounds-and-prompts` 的附录追加不替换, 附录不能把 AI 输出变 Error, 一并保持 |

v0.3 文档有一处与源码不符: docs/architecture/context-cr.md 第 63 行写两种 kind 都跑规则, 而同文件第 115-119 行与 context-cr/src/cli.ts:129 表明 code 运行跳过规则. 本方案以源码为准.

## 0x02 约定

同一件事有多种做法时, 下面已经选定一种.

### 七层的做法

层序按 [claude.dev 原文](https://claude.dev/blog/solving-code-review-at-anthropic/) "Layer by layer" 的编号.

| 层 | 取舍 | 理由 |
|---|---|---|
| 1 提 PR 前自验证 | 不做 | 发生在作者本地, 不在 CI 侧审查器范围; 代码质量规范已规定本地只查工作区 diff |
| 2 每个 PR 的 AI review | 改造 | code 审查加 base 侧上下文文件作背景 |
| 3 可阻塞的安全 review | 改造 | 不另起一遍; 安全边界已是 CODE-AI 六类之一, 且不阻塞 (K1) |
| 4 必过 CI | 采纳 | L0 与仓库规则的 origin=new Error 让 job 失败, v0.3 已是如此 |
| 5 按风险分级人审 | 不做 | 属于仓库分支保护设置; 本方案只提供档位与未答复 AI 发现数 |
| 6 merge queue 与 flag | 不做 | 超出审查器职责 |
| 7 快速发现与回滚 | 改造 | 引发事故的 PR 及其 revert 进 eval 集 |

### 评审器九个机制

| 机制 | v0.3 现状 | 方案取舍 |
|---|---|---|
| 多票 verifier | 每个候选一个新会话, 单票, 无 verdict 即驳回 | Standard 档 1 票门槛 1/1; Deep 档 3 票门槛 2/3; 模型由 config.models.verifier 配置 |
| 丢弃纯风格 | CODE_FINDER_SYSTEM 禁报风格, 命名, 缺测试, 等价写法偏好 | 保持, 不加 A/B 过滤器 |
| Important/Nit/Pre-existing 标签 | 无标签 | 只产 Important; 不产 Nit (风格已禁报); 不产 Pre-existing (AI 发现 origin 恒为 new) |
| 每条 finding 必须答复 | 无 | 不阻塞; 报告与总览页显示未答复 AI 发现数 |
| REVIEW.md | 无 | 不新增文件类型; code owner 把审查要点写进 AGENTS.md 或 docs/ 下, 经背景进入审查 |
| Quick/Standard/Deep 三档 | 无 | 只设 Standard 与 Deep, 按路径谓词机械定档 |
| 轮次上限 | 无 | 3 轮; 第 4 轮起报告首行提示作者重新考虑方案, 不阻塞 |
| 若干轮后只审新 diff | 每次 push 重审整个 PR | 第 3 轮起只审相对上一完成运行 head 的新 diff |
| 按缺陷类别标记且不重复 | 只有指纹去重, 指纹含模型写的 title | 每条记 category (六类之一); 指纹改用 category 加该行内容 |

### 与代码质量规范对齐

01-AI代码质量.md 2.4.1 的三个 @todo 各落一个组件:

| 规范步骤 | 落点组件 | 做法 |
|---|---|---|
| 代码质量工作流第 5 步 "AI CR: 警告级别的内容, 筛选出真问题" | verifier | 代码质量工作流产出的 Warning 汇总 json 作为外部候选, 跳过 finder, 直接进确定性闸门与 verifier, 确认的以 CODE-AI Warning 发布 |
| 代码质量工作流第 6 步 "AI CR: 基于 diff 上下文 和 commit 语义的代码评审" | code 审查 finder | diff 上下文 = 背景, 证据包, 只读工具; commit 语义 = commit message 与 PR 标题描述以数据进入 user 消息 |
| 文档规范工作流第 7 步 "AI CR 文档 review (暂时没有)" | context 审查 | v0.3 已有, 作为独立审查运行, 不并入 code 审查 |

等级映射:

| 规范等级 | 含义 | 发现等级 | 是否阻塞 |
|---|---|---|---|
| 错误 | 确定性检查明确扫到 | Error | 仅 origin=new 时阻塞 |
| 错误 (历史) | baseline 中已存在 | Error, origin=historical | 不阻塞 |
| 警告 | 静态工具无法断定 | Warning, 含全部 CODE-AI | 不阻塞 |
| 规则崩溃 | 检查本身失败 | tool-error | 按 report-contract 处理 |

元门禁六条 (01-AI代码质量.md 第四章) 对 CODE-AI 的满足方式:

| 元门禁 | CODE-AI 如何满足 |
|---|---|
| 可单独测试 | 夹具仓库回放 |
| 有正例 | eval 集里引发事故的 PR 与 revert |
| 有反例 | 已知干净的 PR, 计数有害报告 |
| 输出稳定 | 排序不含时间戳, 指纹用 category 加该行内容 (K7) |
| 唯一规则 ID | CODE-AI |
| 等级明确 | 恒为 Warning |

#### Diff 入口与 Baseline

- 01-AI代码质量.md 第五章 "Diff 级别不等于只看 Diff": 以 diff 为入口, 背景, 证据包与只读工具提供 diff 之外的上下文.
- 第五章 "Baseline 区分历史问题与新增问题": origin=historical 只由 L0 按可信 baseline 赋值 (context-cr/src/l0/index.ts); AI 发现的 origin 恒为 new, finder 禁报 diff 之前已有的问题, 所以 AI 不需要 baseline. 存量问题不进本方案.

### 作用域

13-scope-and-retrieval.md 第 11 节十条决策在本方案下的取舍:

| 决策 | 取舍 | HX 中的行为 |
|---|---|---|
| R1 作用域为一级字段 | 改造 | 上下文文件的作用域由所在目录给出, computeScope 按祖先关系计算, 不另设字段 |
| R2 三根轴 | 改造 | 空间 = 目录祖先; 契约 = 不做; 时间 = 背景取 merge-base 版本, 回放带 as_of |
| R3 不对称传播 | 采纳 | 落在驳回反馈作用域: 抑制表按仓库与指纹, fold 按 kind 与仓库 |
| R4 作用域值只用机器标识 | 采纳 | 路径, ruleId, 指纹 |
| R5 构建图作骨架 | 不做 | TS 业务仓库没有构建图, 降级为目录祖先加 import 边 |
| R6 懒标注按触及触发 | 不做 | 不维护独立知识库 |
| R7 push 为主, pull 有上限 | 改造 | push = 背景与证据包; pull = 只读工具, 不设次数上限, 由预算截停 |
| R8 旁路只做懒标注与压制审计 | 不做 | 不设旁路 agent |
| R9 人只在审批门放宽压制型 | 采纳 | fold 默认同 kind 同仓, 管理员显式提升并留痕 |
| R10 检索记录入账 | 改造 | 每次运行记录背景文件, reasons, base sha, 是否截断 |

四项在源码上的落点:

- 三根轴: 空间由 context-cr/src/scope/index.ts 的 computeScope 计算; 时间由 context-cr/src/scope/base.ts 的 resolveBase 给出 merge-base; 契约不做.
- 不对称传播: 抑制集合在 context-cr/src/review/index.ts 读取, fold 收窄改 prompt_versions 与 prompt_notes 的作用域.
- 四层来源: 构建图无; 路径与文件类型 = context-cr/src/l0/snapshot.ts 的 isContextFile 与 ignored 路径; 框架谓词无; AI 懒标注不做; 人 = 人写的上下文文件与 fold. 代码邻居来自 context-cr/src/review/evidence.ts.
- push 为主 pull 为辅: push 是背景与证据包, pull 是 context-cr/src/review/tools.ts 的 repoTools.

14889 在本方案下 (TS 同构: 后端把响应字段改为可缺省, 两个只经 fetch 读它的前端页面, 一个判了空, 一个没判):

- K-契约 (该字段可缺省, 对所有消费方成立): 只有当 base 侧某个适用的上下文文件写明这一点时才进入审查者输入; 证据包按 basename 找不到 fetch 消费方. 本方案下这类问题通常漏报, 列为已知漏报类.
- K-守卫 (某页面已判空, 只对该页面成立): 若写在该页面目录的 AGENTS.md 里, computeScope 只把它交给该目录及其子目录下的改动; 另一个页面的改动收不到, 不会被越域压制.

### 主流程

14-full-architecture.md 第 1 节六步在 HX 中的对应:

| 步骤 | 模块 | 现有/改造/新增 | 输入 | 输出 |
|---|---|---|---|---|
| 1 commit 意图识别 | code finder 的 user 消息 | 新增 | commit message, PR 标题与描述 | 原样作为数据, 不产意图标签 |
| 2 diff 分组 | scope/code.ts computeCodeScope, review/context.ts batchHunks | 现有 | base, head | hunks 与批次 |
| 3 找问题 | review/index.ts finder, prompts.ts CODE_FINDER_SYSTEM | 改造 | 批次, 背景, 证据包 | 候选 (path, line, category, evidence) |
| 4 校验证据 | review/index.ts gate | 现有 | 候选 | 行存在, 属于本批, 非抑制, 非重复的候选 |
| 5 工具确认 | review/index.ts verifyCode | 改造 | 候选的 path, line, category 与 file:line 位置 | confirmed 或 rejected |
| 6 写入 | stepPublish, POST /runs, resolveRound | 改造 | 报告与 coverage | 发现入库; 只在运行完成时触发 resolved |

14-full-architecture.md 第 11 节缺口:

| 缺口 | 处理 |
|---|---|
| 1 只有预算停止 | 补: "运行完成" 即完成停止, 预算截停一律记不完整 |
| 2 覆盖记账无门禁 | 补: 任一批次未审即覆盖不足, 阈值 0 |
| 3 分块后无跨块一致性检查 | 不补: 不加代码检索, 证据包与只读工具部分覆盖 |
| 4 意图识别无输出契约 | 不补: 不按意图路由 |
| 5 无分诊 | 补: 路径谓词定 Standard 或 Deep |
| 6 工具确认与发现同族自证 | 补: verifier 只收代码与 file:line (K6) |
| 7 反馈无拒绝错误反馈路径 | 部分补: fold 由人触发且默认最窄, 人工驳回带原因分类 |
| 8 无单次成本时延硬门禁 | 补: 单次成本上限与 RUN_DEADLINE_MS, 超限输出部分结果并标不完整 |

- 封闭题与程序定级 (第 7 节): 只引入程序定级; 不引入封闭题问卷与等级矩阵, 因为 AI 发现只有一个等级.
- 五条可证性质 (第 10 节): 不引入穷举证明, 等级函数只有一个输出, 没有可穷举的状态空间.
- 旁路 agent: 不做.

### 证据, 去噪与定级

- 证据: 每条发现带 file:line (K2); evidence 必须引用新增行与证明它错的代码位置.
- 确认: Standard 档每个候选 1 个 verifier 会话, 通过门槛 1/1; Deep 档 3 个会话, 通过门槛 2/3; 会话无 verdict 计为驳回; verifier 输入不含 finder 论证 (K6).
- 去噪: finder 只报六类: 逻辑错误, 边界, 错误处理, 并发与资源, 安全边界, 与既有设计冲突.
- 定级: 查表. ruleId 为 CODE-AI 即 Warning, origin 恒为 new, 标签恒为 Important.

档位谓词: CodeScope.changed 中任一路径命中仓库根的 package.json, 锁文件 (pnpm-lock.yaml, package-lock.json, yarn.lock), tsconfig*.json, .github/workflows/ 下任意文件即 Deep, 否则 Standard; 同一 diff 永远同一档.

### 收敛与停止

| 参数 | 数值或规则 |
|---|---|
| 轮次上限 | 3; 轮次 = 同一 (仓库, PR, kind) 的运行完成次数; 第 4 轮起报告首行提示作者重新考虑方案, 不阻塞 |
| 只审新 diff 的起始轮 | 第 3 轮; base 取该 PR 上一运行完成时的 head |
| 完成停止 | AI 未中断, 候选队列清空, notReviewed 为空 |
| 覆盖不足 | 任一批次未审即覆盖不足, 报告不得写 "无问题" |
| 单次成本与时延上限 | 成本 min(剩余月预算, 月预算的 10%), 10% 为默认值, 拍板人甲方; 时延 RUN_DEADLINE_MS 20 分钟, JOB_TIMEOUT_MS 30 分钟; 超限输出部分结果并标不完整 |

### 背景注入

背景总量上限 16000 字节, 按以下顺序进入:

1. 离改动最近的祖先 AGENTS.md.
2. 引用该文件的上下文文件.
3. 引用其目录的上下文文件.
4. 下级 AGENTS.md.

超出部分不进入. 运行记录里列出被丢弃的路径.

照着做的最短路径, 是把同目录的侧车 [code-background.ts](./code-background.ts) 接进 stepScope 的 code 分支. 它读 base 侧版本, 跳过本 PR 改动的上下文文件. 它按上述顺序与上限, 拼出 user 消息里的引用块. 它同时产出要写进 background.json 的记录.

```ts [stepScope 接线]
const code = computeCodeScope(/* 现有参数 */);
const ctx = computeScope(snapshot, config, changes, base, head, hint);
const bg = buildBackground({
  repoDir: process.cwd(),
  baseSha: base,
  changed: code.changed,
  contexts: ctx.contexts,
});
// bg.userBlock 拼到 codeFinderPrompt(...) 结果之前, 属于 user 消息
// bg.record 写入 R2 对象 background.json
```

### 评估闭环

- eval 来源: cr-demo/eval; 引发事故的 PR 及其 revert 与 fix-forward; 已知干净的 PR.
- 指标: 召回 (eval 正例中被报出的比例), 噪声率 (说错, 有意为之, 不重要三类占全部已答复发现的比例), rounds-to-pass (PR 从首轮到无 open AI 发现的轮数), 采纳率 (沿用 v0.3).
- 改 prompt, 工具或模型时对全集重评, 分升则留, 否则弃.
- 时间轴: 回放时 GET /init 带 as_of, 抑制表, prompt_versions, prompt_notes 按 created_at <= as_of 过滤; 仓库内文件随 checkout 回到当时版本.

作者回应分类:

| 回应 | 判定方式 |
|---|---|
| 已修 | 后续提交使该发现 resolved |
| 说错 | 人工驳回, 原因分类为说错 |
| 有意为之 | 人工驳回, 原因分类为有意为之 |
| 不重要 | 人工驳回, 原因分类为不重要 |

### 契约与存储改动

所有新增字段都可省略, 省略时按现行行为处理, 与 dual-cr-kind 的缺省兼容写法一致, 老 runner 不受影响.

| 位置 | 改动 |
|---|---|
| report-contract | Finding 新增可选 category; 报告顶层新增可选 coverage (complete, notReviewed 批次数), round, unanswered |
| rules 命名空间 | 不变, code 仍只有 CODE-AI |
| D1 迁移 | prompt_versions 与 prompt_notes 加 kind; prompt_versions 加可空 repo_id (空 = 全局); 人工反馈加原因分类; 老行 kind 缺省迁移为 'context'; resolveRound 保护条件扩到 CX-AI 与 CODE-AI |
| R2 对象 | 每次 code 运行新增 background.json: 背景文件路径, reasons, base sha, 是否截断 |
| cf-admin.yaml | GET /init 新增可选 as_of; POST /runs 接收可选 coverage 与 round; POST /feedback 接收可选原因分类 |

### 演进路线

| 版本 | 产出 | 验收 | 覆盖 G |
|---|---|---|---|
| v0.4 | resolveRound 保护全部 AI 规则; 运行完成定义与覆盖不足; 单次成本上限; 背景注入; fold 按 kind 与仓库收窄 | K3, K4, K5 判据全部通过 | G4, G5, G6, G7, G9, G11 |
| v0.5 | 轮次计数与上限; 第 3 轮起只审新 diff; 未答复 AI 发现数; category 与新指纹; verifier 去论证; 外部 Warning 候选; commit 语义输入 | K6, K7 判据通过; 报告出现 round 与 unanswered | G2, G3, G8, G9, G11 |
| v0.6 | eval 集与 as_of 回放; 回应原因分类与四个指标; 路径谓词定档; Deep 档 3 票 | 全集重评产出召回, 噪声率, rounds-to-pass, 采纳率 | G1, G2, G8, G10 |

### 非目标

- 不做 diff 之外的代码检索增强 (契约轴, 字段级消费方).
- 不在 code 审查里审上下文文件本身.
- 不让 AI 产出 Error, 不让 AI 发现阻塞合并.
- 不审纯删除行.
- 不修改 HX-AI-Review 仓库; 本文只给方案.

### 待拍板

| 编号 | 问题 | 默认选择 | 拍板人 |
|---|---|---|---|
| P1 | 是否接受不对称传播 | 接受, 落在驳回反馈作用域 | 已定 (甲方) |
| P2 | Fact 审批无人时是否降级知识库 | 不适用: 没有独立知识库, 知识就是上下文文件, 审批就是改它的 PR | 已定 (甲方) |
| P3 | pull 上限 N | 不设次数, 由预算截停 | 已定 (甲方) |
| P4 | 作用域是否加到规则表条目 | 不做, 由 computeScope 目录关系决定 | 已定 (甲方) |
| P5 | 别名表是否保留 | 不做, 无 proto 全名别名问题 | 已定 (甲方) |
| 12.1 | Fact 审批无人时是否降级知识库 | 不适用: 知识就是上下文文件, 审批就是改它的 PR | 已定 (甲方) |
| 12.2 | 是否接受不对称传播 | 接受, 落在驳回反馈作用域 | 已定 (甲方) |
| 12.3 | 完成停止判据 | 采用 "运行完成" 定义 | 已定 (甲方) |
| 12.4 | 覆盖门禁阈值 | 固定为 0 | 已定 (甲方) |
| 12.5 | 旁路 agent 范围 | 不做 | 已定 (甲方) |

另有一项数值待定: 单次成本上限默认月预算的 10%, 拍板人甲方.

## 0x03 偏好

每条是一句禁令和一个检查动作.

| 禁令 | 检查 |
|---|---|
| 禁止把被审仓库的任何文件内容放进系统提示 | 对任一 code 运行的 sessions.jsonl, 系统提示中搜不到背景里任何文件的内容 |
| 禁止 head 侧的上下文文件进入背景 | background.json 的 included 与本次 changed 无交集 |
| 禁止 finder 报风格, 命名, 缺测试, 等价写法偏好与 diff 之前已有的问题 | 夹具仓库里只有风格问题的 PR, CODE-AI 发现数为 0 |
| 禁止在覆盖不足时出现 "无问题" | coverage.complete 为 false 的报告中搜不到 "无问题" |
| 禁止 fold 的默认作用域是全局 | 新 fold 产生的 prompt_versions 行 repo_id 非空, kind 非空 |
| 禁止 prompt 附录替换内置系统提示或把 AI 输出变 Error | 发布一份附录后, 系统提示仍以内置提示开头, CODE-AI 仍为 Warning |
| 禁止发现不带 file:line | 报告中每条 CODE-AI 的 path 与 line 都非空 |
| 禁止报告排序依赖时间戳 | 报告 findings 中不含时间字段参与排序, 两次运行顺序相同 |
| 禁止新增上下文文件类型 | isContextFile 的判定列表保持 AGENTS.md, README.md, SKILL.md, docs/*.md |

## 0x04 可直接拷走的件

| 件 | 用途 | 放到哪里 | 版本 |
|---|---|---|---|
| [code-background.ts](./code-background.ts) | 读 base 侧上下文文件, 排序, 截断, 生成 user 消息引用块与 background.json 记录; 只依赖 Node.js 内置 node:child_process | context-cr/src/review/ 下, 由 steps.ts stepScope 的 code 分支调用 | v0.4 |
| resolveRound 保护条件 | 不完整运行不解决任何 AI 规则的发现 | cf-admin/worker/store/d1.ts resolveRound | v0.4 |
| AI 发现指纹 | 指纹不随 title 措辞变化 | context-cr/src/review/index.ts toFinding | v0.5 |
| 档位谓词 | 机械定 Standard 或 Deep | context-cr/src/scope/code.ts 之后 | v0.6 |

```sql [resolveRound 保护条件]
-- 原条件: (?6 = 1 OR rule_id != 'CX-AI')
-- ?6 为 aiRechecked; 改为对全部 AI 规则生效
(?6 = 1 OR rule_id NOT IN ('CX-AI', 'CODE-AI'))
```

```ts [AI 发现指纹]
// 原: fingerprint(ruleId, candidate.path, candidate.title, "")
fingerprint(ruleId, candidate.path, candidate.category, lineText.trim())
```

```ts [档位谓词]
const DEEP = [/^package\.json$/, /(^|\/)(pnpm-lock\.yaml|package-lock\.json|yarn\.lock)$/, /(^|\/)tsconfig[^/]*\.json$/, /^\.github\/workflows\//];
export const tier = (changed: string[]): "Standard" | "Deep" =>
  changed.some((p) => DEEP.some((re) => re.test(p))) ? "Deep" : "Standard";
```
