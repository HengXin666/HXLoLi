---
hxid: "hx-9eda4e5a"
title: "DSH Agent Loop 源码剖析"
created_at: "2026-09-05"
model: "Unknown"
skill: ["hx-docs-sediment", "hx-archify"]
authors: "Heng_Xin"
tags: ["AI Agent", "Harness", "DSH"]
---

# DSH Agent Loop 源码剖析

> [!NOTE]
> 用户说一句"用 echo 回显 ping", 会话里到底发生了什么? 没有聊天记录, 只有一条 turn/start -> step/start -> tool/call -> tool/result -> step/end 的事件串, 模型每次看到的历史都是现场推导出来的.
> 更反常识的是: 一次 step/start 到 step/end 之间, 模型可能被请求了两次; 而"谁拥有哪些能力、崩溃后还能相信什么", 全部落在同一份只追加的日志里.
> 这里不讲设计思想, 只把 ReactLoopAgent 的驱动循环、工具回灌与压缩钩子按真实方法名和事件名拆开看 —— 图中每个节点都是真实代码里的名字, 一一可查.

## 0x00 一条真实的事件链

先用一个真实用例建立直觉: 用户说「用 echo 工具回显 ping」, 模型先请求工具, 再基于工具结果给最终答复. 源码测试 loop.spec.ts 断言了这条链 (注意 step/start 到 step/end 之间发生了两次模型请求):

```text
turn/start  step/start  assistant/chunk...  tool/call  tool/result  assistant/chunk...  assistant/message  step/end  turn/end(completed)
```

会话里没有一条「聊天记录」, 只有这串追加的事件; 模型每次请求看到的消息, 是由事件派生出来的 (deriveMessages). 压缩、回放、继续对话都基于这套事件日志.

## 0x01 主线图 (节点即源码)

> [DSH Agent Loop 主线图 #ppt ##w100%##](dsh-loop-main.html)


每个节点背后都是真实文件里的真实方法/事件:

| 图中节点 | 源码位置 | 干什么 |
| --- | --- | --- |
| Inbox | packages/core/agent/src/inbox.ts | 三类入口都先入队: followup/steer → next-turn/next-step; inject 不唤醒 |
| preStep() | packages/core/agent-loop/src/agent.ts (L232) | claim 队列 → assemble 系统提示 → 投射运行时快照 → 派发 agent/pre-step 瀑布 |
| step(): stream | 同文件 step() (L339) | buildRequest → llm.stream; 每个 chunk 追加 assistant/chunk |
| 工具执行 | src/tool-calls.ts | 每调用 append tool/call; 结果 append tool/result; 附加上下文 splice 回 inbox next-step |
| 回灌 | step() 内 while(true) | 下一轮 buildRequest 用 deriveMessages(), 于是工具结果进入第二次模型请求的可见历史 |
| turn/end | agent.ts turn() (L253) | reason: completed / max-tokens / blocked / error; 日志最后一条 |
| 压缩 (旁路) | packages/compaction/compaction-basic/src/index.ts | 见 0x02 |

## 0x02 两层循环的真相

同一个文件里其实叠着两个 while, 别混:

### 外层: turn() 的 while(true) — 一个 turn 内多个 step

每次循环: preStep() 取队列并派发 agent/pre-step → append step/start → 追加用户消息 → step() → append step/end. 当 step 结束且 inbox 的 next-step 队列空了, 才 append turn/end 退出. 一个 turn 可以含多个 step (例如工具结果又触发新的用户级消息).

### 内层: step() 的 while(true) — 一个 step 内多轮模型请求

模型回复若含 tool-call, 执行工具后结果落进会话与 inbox, 内层 while 直接进入下一轮 buildRequest — 所以一次 step/start..step/end 可以包着多次 llm 请求 (上面的例子就是 2 次). 只有当回复不含工具调用时, step 才返回 completed.

### kick() 驱动

send() 带 wakeup 时 wakeDriver(): phase 变 running, kick() 里 while(await turn()) 反复开 turn, 直到 inbox 不再有 pending. 维护 (maintenance) 或取消时会 latch wakeRequested, 收敛后再补开.

## 0x03 Compaction: 两个钩子

压缩不是独立线程, 它只是两个事件上的监听器 (compaction-basic/src/index.ts), 通过修改 surface 投影来降低下一次请求的压力:

| 钩子 | 触发条件 | 动作 |
| --- | --- | --- |
| agent/pre-step | tokenMeter.measure ≥ 0.8 × contextWindow | 先跑 tool-result 剪枝, 再选可压缩区间 (config.ts DEFAULT_RATIO 0.8/0.16), 摘要替换 |
| agent/request-error | failure.code == CONTEXT_WINDOW_EXCEEDED | 无视阈值强制压缩 (region.ts selectCompactableRange 尾保留 0), 成功后返回 { kind: retry } |

### 区间怎么选 (region.ts selectCompactableRange)

1. 从 surface 末尾往回累加 token, 直到 ≥ retainTokens (默认 0.16 × 窗口) — 最近的内容不压;
2. 继续回退到 toolPairingBalancedBefore 满足 — 不让工具调用与其结果被拆到摘要两侧;
3. 从首节点到该边界整体作为一个 span, 交给一次 LLM 摘要, 产出包在 <compacted-summary> 里的用户消息 checkpoint (summarizer.ts frameSummary), 替换被 shadow 的 surface 节点.

全程有 compaction/start..end 事务事件, 且要求摘要帧比原文小, 否则失败重试.

## 0x04 文件地图

- packages/core/agent/src/inbox.ts — 消息队列 (next-turn / next-step)
- packages/core/agent-loop/src/agent.ts — ReactLoopAgent: kick/turn/preStep/step/buildRequest
- packages/core/agent-loop/src/runtime-context.ts — 运行时快照投影
- packages/core/agent-loop/src/tool-calls.ts — 工具调度与 tool/call、tool/result
- packages/compaction/compaction-basic/src/ — 监听器 index.ts + 选段 region.ts + 摘要 summarizer.ts + 配置 config.ts

## 0x05 拓展升华展望

把这条事件链放大来看, 它回答的是一个更普遍的问题: 当执行者是模型而不是函数时, "发生了什么"该由谁来记账.

**事实层面** … 这套实现把会话简化成一条只追加的事件流: turn/step 是边界, assistant/chunk 是流式证据, tool/call 与 tool/result 成对出现, 模型每次看到的历史由 deriveMessages 从同一份日志推导而来; 压缩则挂在 agent/pre-step 与 agent/request-error 两个钩子上, 用摘要替换 surface 投影里的一段旧内容, 原始节点不删. 工具结果的回灌也走同一条路: 结果先落成工具结果, 上下文的补充 splice 回 inbox 的 next-step, 下一次 buildRequest 就能看见它.

**个人判断** … 我倾向于认为, "事件日志 + 投影"会成为 agent 会话的默认形态, 因为一旦模型能改真实文件、发真实请求, 事后重建"它到底看见过什么、做过什么"就比任何优化都重要. 反过来, 把消息数组当作唯一真相的系统会越来越难维护: 压缩、回放、分叉、崩溃恢复每多一个入口, 就多一份需要各自解释历史的方式. 判断一套 agent 是否可靠, 其实可以先问一句: 它的"模型所见"能不能只从日志重算出来?

## 0x06 参考来源

- 项目: [deepseek-ai/dsh](https://github.com/deepseek-ai/dsh) — 本文逐节点对照的源码仓库 (packages/core/agent-loop 等), 事件链断言取自其测试 loop.spec.ts. 行号以 0.1.x 为准, 上游小版本间可能有行号漂移, 但方法名与事件名稳定.
