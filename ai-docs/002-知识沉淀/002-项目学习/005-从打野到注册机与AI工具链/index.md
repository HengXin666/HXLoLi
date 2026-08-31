---
title: "从打野到注册机：一条真实项目工具链的工程化学习"
created_at: "2026-08-02"
model: "GPT-5 Codex"
skill: ["hx-make-ai-docs"]
authors: "Heng_Xin"
tags: ["项目学习", "AI Agent", "逆向工程", "Codex", "MCP"]
---

# 从打野到注册机：一条真实项目工具链的工程化学习

> [!NOTE]
> 本文由 AI 辅助沉淀, 需要用户 review 后再提交.

## 0x00 背景

本文沉淀 Discord 论坛帖子《个人常用项目推荐（打野到注册机一条龙，简易教程）》. 帖子发表于 2026-07-01, 作者为 `𝑏𝑎𝑖𝑞𝑖_𝑞𝑎𝑞`, 所在服务器为 `KYL`, 论坛频道为 `🌐-各类教程-chatgpt🚀`.

原帖的主题不是“推荐几个 GitHub 项目”这么简单. 作者沿着自己的实际使用路径, 把资源搜集、协议探查、浏览器自动化、邮箱、代理、注册流程、服务封装和 AI 工具串成一条连续的工程链. 前半段看起来像是在解决一个具体的注册流程, 后半段则把同一种“找现成能力并组织起来”的方法迁移到了 Codex、Skills 和逆向实验室.

为了保留原汁原味的学习心流, 本文先按原帖叙事阅读, 再做事实核验, 不把它改写成脱离上下文的安全摘要或库选型排行榜.

本文使用两个数量口径:

- 原帖有 13 个编号步骤.
- 13 个步骤实际链接了 15 个 GitHub 仓库, 因为代理和 Cloudflare Worker 两处各有并列项目.

正文中的判断分为三类:

- `原文路径`: 作者把项目放在链路中的位置.
- `仓库事实`: 截至 2026-08-02 对默认分支、README、源码结构、维护信息和最新 commit 的核验.
- `迁移判断`: 结合 HXLoLi 当前的 `ai-docs`、Skills 和 Codex 工作流得到的学习结论.

本文保留注册机、邮箱、验证码、代理和账号池在原文链路中的工程角色, 但练习边界停留在授权测试环境、自有服务和 `fake_protocol` 这类本地假链路, 不扩写成批量注册真实第三方服务、验证码绕过或风控规避的可直接执行手册.

## 0x01 核心结论

这篇帖子真正展示的是一条逐层补齐契约的工具链:

```text
项目搜集
  -> 目标与协议探查
  -> 浏览器路线 / 协议路线
  -> 邮箱与验证码输入
  -> IP / 代理与节点管理
  -> 注册流程模板
  -> HTTP 服务封装
  -> Codex 约束
  -> Skills 反问与反馈
  -> ReverseLab 知识路由与 MCP
```

可以迁移的核心经验有五条:

1. 不要从“我要写一个注册机”直接跳到代码. 先把目标拆成输入、状态、外部依赖、输出和失败恢复.
2. “打野”本质上是能力发现和能力拼装. 一个仓库的价值要放回前后调用链中判断.
3. 协议自动化和浏览器自动化不是谁取代谁, 而是两种观测粒度不同的实现路线: 前者靠请求契约, 后者靠页面状态与真实交互.
4. AI 工程的瓶颈不只是生成代码. `ponytail` 约束重复实现, `mattpocock/skills` 约束目标澄清、共享语言和反馈循环, `open-reverselab` 约束知识与工具的路由.
5. 任何推荐都要带证据和版本. 作者的使用感受、README 的自我描述、当前源码和 GitHub 维护状态不能混成一个事实.

## 0x02 原文路径

### 2.1 13 个编号步骤与 15 个仓库

下表先恢复原帖的叙事顺序. 并列仓库保留在同一个工程节点中, 但不会因为它们在同一行就把能力误认为相同.

| 步骤 | 仓库 | 原文中的位置 | 输入 -> 输出 |
| --- | --- | --- | --- |
| 1 | [MasterAlanLab/Register](https://github.com/MasterAlanLab/Register) | “打野”的资源入口和注册相关脚本汇总 | 公开项目与资源线索 -> 可继续追踪的工具入口 |
| 2 | [chenyme/grok2api](https://github.com/chenyme/grok2api) | Grok API / 多账号网关 | 上游账号、凭据、客户端请求和出口配置 -> OpenAI/Anthropic 兼容 API、管理台和账号状态 |
| 3 | [GALIAIS/CTF-Sandbox-Orchestrator](https://github.com/GALIAIS/CTF-Sandbox-Orchestrator) | 探查、抓包和技能路由 | 授权沙盒中的目标信号、附件、流量或题目类型 -> 对应专项 Skill 和实验路径 |
| 4 | [LoseNine/ruyipage](https://github.com/LoseNine/ruyipage) | 浏览器自动化路线 | URL、页面动作、浏览器上下文和网络事件 -> 页面状态、请求/响应观测和自动化结果 |
| 5 | [ydddp/mail-hub](https://github.com/ydddp/mail-hub) | 邮箱聚合和验证码输入 | 邮箱渠道配置、目标域名和收件箱请求 -> 统一收件箱、邮件、验证码或验证链接 |
| 6 | [MetaCubeX/mihomo](https://github.com/MetaCubeX/mihomo) + [Resinat/Resin](https://github.com/Resinat/Resin) | 原文意图是 IP / 代理管理; 当前事实中 Resin 接收节点订阅并提供代理入口, mihomo 接收 UID/语言并输出游戏数据模型 | Resin: 节点订阅、代理节点和业务会话标识 -> 代理入口、健康状态、粘性路由和可观测数据; mihomo: UID/语言 -> Pydantic 模型 |
| 7 | [GuangYiDing/cfspider](https://github.com/GuangYiDing/cfspider) + [baiqigo/baiqi-cf-worker-mihomo](https://github.com/baiqigo/baiqi-cf-worker-mihomo) | Cloudflare Worker 和节点分发 | Cloudflare 配置、UUID、KV 和节点来源 -> Worker 地址、VLESS/Mihomo 订阅和节点列表 |
| 8 | [baiqigo/baiqi-register-template](https://github.com/baiqigo/baiqi-register-template) | 注册流程模板 | 抓包得到的流程、邮箱/打码/代理适配器和配置 -> 结构化运行结果、导入对象和日志 |
| 9 | [fastapi/fastapi](https://github.com/fastapi/fastapi) | 服务封装和反向代理 | Python 类型声明、路由和内部流程 -> HTTP/ASGI 服务、OpenAPI 文档和上层调用接口 |
| 10 | [DietrichGebert/ponytail](https://github.com/DietrichGebert/ponytail) | Codex 的最小实现约束 | 编码任务、现有调用链和约束 -> 更小的实现、最小验证和较少的重复代码 |
| 11 | [mattpocock/skills](https://github.com/mattpocock/skills) | 反问、共享语言和工程反馈 | 模糊需求、领域术语、代码库和反馈 -> 明确问题、ADR、TDD 切片和可复用工作流 |
| 12 | [basketikun/chatgpt2api](https://github.com/basketikun/chatgpt2api) | 协议适配、图像服务和资源管理 | 授权测试账号、官网协议请求和图片任务 -> OpenAI 兼容图像 API、Web 面板、存储和账号状态 |
| 13 | [LING71671/open-reverselab](https://github.com/LING71671/open-reverselab) | 逆向知识库、MCP 和实验室自动化 | 授权样本、board 信号和分析任务 -> 知识路由、工具映射、exports 和报告 |

这里的两个核验警报很重要:

- `MetaCubeX/mihomo` 当前 GitHub README 实际描述的是 Honkai: Star Rail 的 Python Pydantic 数据模型, 输入是 UID/语言, 输出是解析后的游戏数据. 它与原帖中“代理/IP 管理”的角色不一致. `Resin` 才是这批链接里明确的代理池网关. 本文保留原帖链接位置, 但不替它把错误链接解释成代理核心.
- `MasterAlanLab/Register` 的当前 README 明确写着“脚本往后不再更新”. 它仍然有资源发现价值, 但不能作为维护中的运行时依赖.

### 2.2 从“打野”到可运行链路

#### 2.2.1 先找入口, 再决定自己写什么

第 1 步的 `Register` 更像一个资源目录, 不是链路中的核心服务. 这一步教的是发现能力: 先判断别人已经解决了哪些问题, 再决定哪些地方值得自己实现. 但资源目录也最容易失真, 因为它同时混入主观推荐、外部服务链接和已经停止更新的脚本.

第 2 步的 `grok2api` 把“资源入口”变成了一个较清晰的工程边界: 它不是单一脚本, 而是一个带 React 管理端的 Go 网关, 将账号、模型、密钥、出口、路由、审计和媒体能力分开. 学习重点从“找到一个 API”变成“观察一个多 Provider 网关如何管理状态和失败”.

#### 2.2.2 探查和浏览器是两个观测面

第 3 步的 `CTF-Sandbox-Orchestrator` 把探查放入“总控 Skill -> 专项 Skill -> 参考资料”的路由模型. 它的工程价值是上下文编排: 先判断信号属于 Web、Android、Windows、Cloud、Crypto 还是其他板块, 再只加载当前需要的专项能力.

第 4 步的 `ruyiPage` 则代表浏览器路线. 它基于 Firefox 和 WebDriver BiDi, 提供页面动作、网络拦截、浏览器上下文和事件观测. 与协议路线相比, 浏览器路线保留了更多页面状态和真实交互信息, 代价是运行时更重、浏览器版本和环境差异更明显.

因此, “抓包”和“浏览器自动化”不是两个互斥的教程章节. 前者帮助建立请求契约, 后者帮助确认页面状态、事件顺序和浏览器侧副作用. 合法测试时可以用同一个本地假服务同时观察两条路线, 而不是直接对真实第三方服务做批量尝试.

#### 2.2.3 外部输入要变成稳定接口

第 5 步的 `mail-hub` 把邮箱从某一个供应商的细节中抽出来. 它对上提供统一 REST API、自动轮询和验证码提取, 对下连接多个邮件渠道、Outlook/IMAP 账号池和自定义 REST/GraphQL provider.

这一步的学习重点不是“如何拿验证码”, 而是外部输入如何进入状态机:

```text
acquire inbox
  -> wait for message
  -> extract code or link
  -> hand result to the flow
  -> release / mark success or failure
```

这种接口一旦稳定, 上层流程就不需要知道邮件来自哪个 provider. 同样的思想也会出现在代理和打码适配器中.

#### 2.2.4 代理层是基础设施, 不是一个字符串

第 6 步和第 7 步把网络出口拆成两层:

- `Resin` 负责把大量订阅节点变成稳定、可观测、支持会话保持的代理入口, 处理健康检查、节点选择、故障切换和接入协议.
- `cfspider` 与 `baiqi-cf-worker-mihomo` 负责 Cloudflare Worker、VLESS 和订阅内容的生成/分发, 更接近节点来源和边缘部署适配层.

原帖把 `MetaCubeX/mihomo` 放在这个位置, 但当前仓库事实不支持“它是代理核心”的解释. 这正好说明工程学习不能只沿着名字和上下文猜: 需要打开 README、入口文件和依赖文件确认输入输出.

代理层真正需要学习的是契约:

- 上游输入是什么格式, 是订阅 URL、URI 列表、JSON/YAML 还是节点对象.
- 下游需要的是 HTTP、SOCKS5、反向代理还是订阅文本.
- 业务会话是否需要稳定出口.
- 健康检查、失败重试、审计和凭据脱敏在哪里发生.

#### 2.2.5 模板把流程变成可替换的 Channel

第 8 步的 `baiqi-register-template` 是前半段最适合做源码学习的项目. 它没有绑定真实渠道, 而是把流程抽象为 `channels/<name>/manifest.py` 和 `flow.py`, 再把邮箱、验证码、代理和 HTTP 能力放进 `registrar/context.py`、`services.py` 等适配边界.

它的本地假链路输出是:

```text
output/<channel>/<run_id>/
  results.json
  accounts_for_import.jsonl
  run.log
```

因此可以先用 `fake_protocol` 跑通状态机, 观察每一步的开始、成功、失败、耗时、HTTP 状态和脱敏日志, 再讨论如何为自有测试服务写一个 Channel. 这比一开始把所有真实外部服务接进来更能看清工程结构.

#### 2.2.6 FastAPI 是链路的边界层

第 9 步的 FastAPI 并不负责“注册”本身. 它的作用是把内部流程封装成可调用的 HTTP 服务, 让 CLI、Worker、管理端或其他 AI 工具通过稳定的 API 接入.

到这里, 前半段的主题完成了一次提升:

```text
零散脚本
  -> 外部依赖适配器
  -> 可观察的状态机
  -> 可复用 Channel
  -> HTTP 服务边界
```

### 2.3 从业务链路转向 AI 工具链

#### 2.3.1 Ponytail: 先问“这段代码是否需要存在”

第 10 步不是换一个 API, 而是把前半段积累的工程习惯施加给 AI Agent. `ponytail` 的阶梯顺序是:

```text
YAGNI
  -> 复用现有代码
  -> 标准库
  -> 原生平台能力
  -> 已安装依赖
  -> 一行实现
  -> 最小自实现
```

它并不是鼓励 Agent 盲目少写代码. 规则明确要求先理解问题、读取相关代码并追踪真实调用链; 输入校验、错误处理、安全、无障碍和必要测试不能被“最小化”删掉.

#### 2.3.2 Skills: 先对齐, 再实现

第 11 步的 `mattpocock/skills` 把“AI 写得不对”拆成几个工程原因:

- 目标没有对齐, 所以先 grilling.
- 术语没有对齐, 所以建立 shared language 和 `CONTEXT.md`.
- 代码没有反馈, 所以补静态检查、浏览器验证和 TDD.
- 系统不断变复杂, 所以用领域建模、ADR 和架构治理约束变化.

这与本文的答题卡流程直接对应: 方向没有闭环时只问一个上游问题, 用户回答后再进入下一个依赖问题. 反问不是拖延, 而是把隐含约束变成可复查的接口.

#### 2.3.3 ChatGPT2API: 协议适配和资源管理的综合例子

第 12 步的 `chatgpt2api` 将官网协议逆向整理、OpenAI 兼容图像 API、Web 面板、图片任务、存储和账号资源管理放在一起. 它把“一个协议适配器”推进到可运行服务, 也把账号、代理、额度、任务状态和持久化带来的工程复杂度暴露出来.

这里最适合学习的不是复制某个官网协议, 而是观察以下边界如何协作:

```text
兼容 API
  -> 任务与媒体服务
  -> 上游协议适配
  -> 账号/额度状态
  -> 存储与代理运行时
```

仓库 README 自身已经声明授权、条款和禁止批量滥用的边界. 因此本文只把它作为协议适配和服务工程案例, 不把账号池或官网接口扩写成可滥用教程.

#### 2.3.4 ReverseLab: 从工具链升级为可路由的学习环境

第 13 步的 `open-reverselab` 把前面所有经验再抽象一层. 它不是把逆向工具简单堆在一个目录里, 而是用 board、case、KB、tools、scripts、exports、reports 组织工作, 用目录约定让 Agent 能恢复上下文.

它的核心数据流是:

```text
Signal
  -> kb_router(board=...)
  -> kb_read_file
  -> attack / analysis chain
  -> MCP tool mapping
  -> evidence in exports / reports
```

到这里, 原帖的主题完成闭环: 前半段是在拼装一个业务工具链, 后半段是在拼装一个能帮助自己继续学习和实现的 Agent 工具链.

## 0x03 仓库级验证

以下是 2026-08-02 的 GitHub API 快照. 星标、Fork、更新时间会变化; “维护判断”只依据当前 API 元数据、默认分支、README 和目录结构, 不等于我已经在生产环境运行过这些项目.

| # | 仓库 / 最新 commit | 角色核验: 输入 -> 输出 | 主要依赖或外部边界 | 维护判断 |
| --- | --- | --- | --- | --- |
| 1 | [MasterAlanLab/register](https://github.com/MasterAlanLab/register) / [79a1f0b](https://github.com/MasterAlanLab/register/commit/79a1f0b0a1e83403a3e62f9d8f41602890122758) | 公开资源和注册脚本线索 -> 汇总 README | 无明确运行时依赖; 资源指向多个外部服务 | README 明示脚本不再更新; 912 stars / 350 forks, 适合作为发现入口, 不适合作为稳定依赖 |
| 2 | [chenyme/grok2api](https://github.com/chenyme/grok2api) / [0901045](https://github.com/chenyme/grok2api/commit/090104504b403d65675a01dab9c92b3a235ee832) | 上游 Provider 账号、凭据、API 请求和出口 -> OpenAI/Anthropic 兼容响应、管理台和审计状态 | Go 后端、React 管理端、SQLite/PostgreSQL、Memory/Redis、Docker; 依赖上游服务条款 | 2026-08-01 仍有更新, 6,931 stars / 2,133 forks, 活跃但外部 Provider 变化风险高 |
| 3 | [GALIAIS/CTF-Sandbox-Orchestrator](https://github.com/GALIAIS/CTF-Sandbox-Orchestrator) / [bf07172](https://github.com/GALIAIS/CTF-Sandbox-Orchestrator/commit/bf07172369015201c48a449596a9d7e725a3057f) | 授权沙盒信号和附件 -> 总控路由、专项 Skill 和 references | Markdown `SKILL.md`、`agents/openai.yaml` 和 Agent/Skill 运行时; 具体工具按板块安装 | 2026-03-30 最近代码推送, 198 stars / 37 forks; 专项方向完整, 使用前应检查版本和工具覆盖 |
| 4 | [LoseNine/ruyipage](https://github.com/LoseNine/ruyipage) / [5d22e8a](https://github.com/LoseNine/ruyipage/commit/5d22e8a4d6b289389f4c8ee46745c1a123f8c752) | URL、浏览器动作、Firefox 上下文和网络事件 -> 页面结果、请求/响应捕获和浏览器状态 | Python、Firefox、WebDriver BiDi; async 路线可用 `greenlet` / `websockets`; 需要本地浏览器运行时 | 2026-08-01 仍有更新, 1,820 stars / 205 forks; 过检测能力属于作者声明, 不应当作独立验证结论 |
| 5 | [ydddp/mail-hub](https://github.com/ydddp/mail-hub) / [4fafa33](https://github.com/ydddp/mail-hub/commit/4fafa33bce63cb8e4815483076b98b1a6fe225ae) | 渠道配置、收件箱请求和目标域名 -> 统一邮件、验证码/链接、渠道和配额状态 | Node.js 18+、TypeScript、Docker/PM2、SQLite 数据; 依赖外部邮件 provider 和凭据 | 2026-08-01 仍有更新, 73 stars / 8 forks; 功能明确但供应商可用性和合规边界需要自行验证 |
| 6 | [MetaCubeX/mihomo](https://github.com/MetaCubeX/mihomo) / [008b91b](https://github.com/MetaCubeX/mihomo/commit/008b91bfe8c0e2daca0ab69061efd9ea1ad71bd2) | UID、语言 -> Honkai: Star Rail Pydantic 解析模型 | Python、Pydantic、`api.mihomo.me` | 2026-08-01 仍有更新, 32,855 stars / 4,318 forks; 当前实际角色与原帖代理/IP 位置不一致, 是链接核验警报 |
| 7 | [Resinat/Resin](https://github.com/Resinat/Resin) / [9b8ef8e](https://github.com/Resinat/Resin/commit/9b8ef8e5cf83071fbac4de29bd7187268b9cff7b) | 订阅 URL、节点列表、平台筛选和会话身份 -> HTTP/SOCKS5/反向代理、粘性路由、健康和审计数据 | Go、Docker、Clash/sing-box/URI 节点格式、持久化状态 | 2026-08-01 仍有更新, 1,932 stars / 305 forks; 代理网关角色与原帖位置相符 |
| 8 | [GuangYiDing/cfspider](https://github.com/GuangYiDing/cfspider) / [c2e8b05](https://github.com/GuangYiDing/cfspider/commit/c2e8b05b1a73ce9902b34c547565bebb879adc5b) | Cloudflare API 配置、Worker 参数和请求 -> Worker 代理对象、Python 请求/浏览器辅助能力和节点池 | Python、Cloudflare Workers、VLESS; 依赖 Cloudflare 账户和部署配置 | 最近代码推送为 2026-01-26, 38 stars / 7 forks; 低活跃小项目, 需要先审计脚本、权限和部署风险 |
| 9 | [baiqigo/baiqi-cf-worker-mihomo](https://github.com/baiqigo/baiqi-cf-worker-mihomo) / [e36453b](https://github.com/baiqigo/baiqi-cf-worker-mihomo/commit/e36453bedf61fbcb78763eedf55b3056867f308a) | UUID、ADMIN/KEY、KV 和 preferred-IP 来源 -> `/mihomo` 与 `/sub` 订阅响应 | Cloudflare Worker、KV、Wrangler、VLESS/WS/TLS; 需要自己管理密钥和 UUID | 最近代码推送为 2026-06-25, 59 stars / 10 forks; 定位是清理后的发布副本, 不应提交真实凭据 |
| 10 | [baiqigo/baiqi-register-template](https://github.com/baiqigo/baiqi-register-template) / [4791c6a](https://github.com/baiqigo/baiqi-register-template/commit/4791c6a0cb877abbd50a1ed843a6c468e9f2bb4f) | Channel flow、邮箱/验证码/代理适配器、count 和配置 -> `results.json`、`accounts_for_import.jsonl`、`run.log` | Python、`requirements.txt`; 默认可用 mock/none/fake protocol, 接入外部服务才增加外部依赖 | 最近代码推送为 2026-06-25, 54 stars / 10 forks; 适合作为本地状态机教学模板 |
| 11 | [fastapi/fastapi](https://github.com/fastapi/fastapi) / [95f8322](https://github.com/fastapi/fastapi/commit/95f8322ee1dcda7ceace7b1c4f6c9915b36d748f) | Python 类型声明和路由处理 -> ASGI HTTP 服务、OpenAPI 和 JSON Schema | Starlette、Pydantic、ASGI server; 标准 Python 类型提示 | 2026-08-01 仍有更新, 101,137 stars / 9,718 forks; 成熟的服务边界基础设施 |
| 12 | [DietrichGebert/ponytail](https://github.com/DietrichGebert/ponytail) / [16f2980](https://github.com/DietrichGebert/ponytail/commit/16f29800fd2681bdf24f3eb4ccffe38be3baec6b) | 编码任务、现有调用链和约束 -> 最小实现、最小检查和较短 diff | Agent Skill、规则文件、Codex/Claude 等插件和 Node lifecycle hooks | 2026-07-15 最近代码推送, 93,470 stars / 5,133 forks; 活跃且影响面大, README benchmark 是作者实验结果 |
| 13 | [mattpocock/skills](https://github.com/mattpocock/skills) / [2ab9580](https://github.com/mattpocock/skills/commit/2ab958093e83e0ec752e6c1c5932da465bf23e0c) | 模糊需求、领域术语、仓库上下文和反馈 -> grilling、shared language、ADR、TDD 和架构反馈 | Markdown Skill、ADR/docs、Agent CLI; TDD 需要项目自己的公共 seam 和测试运行时 | 2026-07-31 最近代码推送, 198,850 stars / 17,132 forks; 活跃, 适合借鉴工作流而不是照抄目录 |
| 14 | [basketikun/chatgpt2api](https://github.com/basketikun/chatgpt2api) / [dc105e5](https://github.com/basketikun/chatgpt2api/commit/dc105e51bd486bd75c8ef4f74be4bc4724bdfc33) | 授权协议请求、图片任务、账号和存储配置 -> OpenAI 兼容图片 API、Web UI、任务和账号状态 | Python/uv、Docker; JSON/SQLite/PostgreSQL/Git 存储; WARP/Privoxy/FlareSolverr 可选 | 2026-07-29 最近代码推送, 5,493 stars / 1,490 forks; 功能活跃但官网协议、账号和条款风险必须单独评估 |
| 15 | [LING71671/open-reverselab](https://github.com/LING71671/open-reverselab) / [b6522eb](https://github.com/LING71671/open-reverselab/commit/b6522ebb402be8d53161ae8fe70995b060f8b79f) | board 信号、授权样本和分析任务 -> KB 路由、MCP 工具调用、exports、notes 和 reports | Python >=3.10、uv、`mcp>=1.2.0,<2`、`pycryptodome`; Android/PE/CTF 工具按板块可选 | 2026-08-01 仍有更新, 954 stars / 244 forks; 结构活跃, 但 README 的 178 篇与 GitHub description 的 197 篇统计不一致 |

## 0x04 三个重点项目的源码级学习

### 4.1 Ponytail: 把“少写代码”变成可执行规则

重点源码和规则:

- [`.agents/rules/ponytail.md`](https://github.com/DietrichGebert/ponytail/blob/16f29800fd2681bdf24f3eb4ccffe38be3baec6b/.agents/rules/ponytail.md): always-on 的最小实现阶梯、调用链追踪和安全边界.
- [`skills/ponytail/SKILL.md`](https://github.com/DietrichGebert/ponytail/blob/16f29800fd2681bdf24f3eb4ccffe38be3baec6b/skills/ponytail/SKILL.md): Skill 元数据、持久化激活、强度级别和输出约束.
- [`skills/ponytail-review/SKILL.md`](https://github.com/DietrichGebert/ponytail/blob/16f29800fd2681bdf24f3eb4ccffe38be3baec6b/skills/ponytail-review/SKILL.md): 只审查过度工程化, 不把正确性和安全问题混进“删代码”结论.
- [`.codex-plugin/plugin.json`](https://github.com/DietrichGebert/ponytail/blob/16f29800fd2681bdf24f3eb4ccffe38be3baec6b/.codex-plugin/plugin.json): 将规则、Skills 和 lifecycle hooks 接入 Codex.

最值得迁移的是顺序, 不是口号:

```text
先完整理解问题和调用链
  -> 再从 YAGNI 开始逐级寻找现成能力
  -> 最后只写第一处真正需要自实现的代码
```

它还给出了一个重要的负面定义: “最小”不等于删除校验、错误处理、安全、无障碍和必要测试. 这对本文尤其重要, 因为注册流程、账号状态、外部输入和网络请求恰好都处在信任边界上.

README 的 benchmark 声称在真实 FastAPI + React 仓库、12 个 feature tasks、Haiku 4.5、`n=4` 的 Agent session 上平均减少约 54% 代码. 这是项目自己的实验结果, 可以作为研究假设和复现入口, 不能当作独立基准事实.

### 4.2 mattpocock/skills: 把工程常识变成反馈回路

重点文件:

- [`grill-me/SKILL.md`](https://github.com/mattpocock/skills/blob/2ab958093e83e0ec752e6c1c5932da465bf23e0c/skills/productivity/grill-me/SKILL.md): 用户显式触发的 grilling 入口.
- [`grill-with-docs/SKILL.md`](https://github.com/mattpocock/skills/blob/2ab958093e83e0ec752e6c1c5932da465bf23e0c/skills/engineering/grill-with-docs/SKILL.md): 将 grilling 与领域建模、shared language 和 ADR 结合.
- [`tdd/SKILL.md`](https://github.com/mattpocock/skills/blob/2ab958093e83e0ec752e6c1c5932da465bf23e0c/skills/engineering/tdd/SKILL.md): 以公共 seam、行为测试、red-green-refactor 和垂直切片为核心.
- [`setup-matt-pocock-skills/SKILL.md`](https://github.com/mattpocock/skills/blob/2ab958093e83e0ec752e6c1c5932da465bf23e0c/skills/engineering/setup-matt-pocock-skills/SKILL.md): 先探索代码库, 再逐段确认 issue tracker、标签和领域文档布局.

它解决的是 AI 工程中的四种熵:

| 熵的来源 | 对应机制 | 学习结果 |
| --- | --- | --- |
| 目标含糊 | grilling | 先闭环上游约束, 再做下游设计 |
| 术语分裂 | shared language / `CONTEXT.md` | 人、Agent、代码里的名字对齐 |
| 没有反馈 | TDD、类型检查、浏览器验证 | 每个垂直切片都有可观察结果 |
| 结构腐化 | ADR、领域建模、架构治理 | 让变化留下决策记录, 而不是只留下代码 |

本次沉淀实际上已经采用了这个方法: 用户先确认“保留原文主题和心流”, 再确认“13 个步骤全部核验、三个项目源码深挖”, 之后才进入大规模事实调研. 答题卡只保留一个依赖最上游的问题, 用户回答后才继续.

### 4.3 open-reverselab: 目录即约定, 路由即上下文

重点文件:

- [`README.md`](https://github.com/LING71671/open-reverselab/blob/b6522ebb402be8d53161ae8fe70995b060f8b79f/README.md): board、KB、MCP 和 Signal 路由.
- [`AGENTS.md`](https://github.com/LING71671/open-reverselab/blob/b6522ebb402be8d53161ae8fe70995b060f8b79f/AGENTS.md): 全局路由、证据落盘、样本边界和板块分析流程.
- [`AI-USAGE.md`](https://github.com/LING71671/open-reverselab/blob/b6522ebb402be8d53161ae8fe70995b060f8b79f/AI-USAGE.md): `board -> context -> KB -> tools -> evidence` 的 Agent 操作入口.
- [`boards/ctf-website/AI-USAGE.md`](https://github.com/LING71671/open-reverselab/blob/b6522ebb402be8d53161ae8fe70995b060f8b79f/boards/ctf-website/AI-USAGE.md): 进入具体板块后的进一步路由.
- [`ReverseLabToolsMCP/pyproject.toml`](https://github.com/LING71671/open-reverselab/blob/b6522ebb402be8d53161ae8fe70995b060f8b79f/tools/skills/mcp/ReverseLabToolsMCP/pyproject.toml): MCP 依赖边界 `mcp>=1.2.0,<2` 和 `pycryptodome`.

它的关键不是工具数量, 而是让 Agent 在每次任务中都回答同一组问题:

1. 这是什么板块, 当前信号是什么?
2. 哪一篇 KB 文章先提供假设和方法?
3. 哪个 MCP 工具是这一步的公共 seam?
4. 输入、输出、证据和下一步放在哪里?

README 宣称 178 篇知识库文章和 100+ MCP 工具, GitHub API 的 description 则写 197 篇. 这不是必须立刻解决的错误, 但必须记录为版本统计差异, 不能在笔记里选择一个数字伪装成稳定事实.

本次只读取了 ReverseLab 的架构、规则和 MCP 依赖, 没有执行其中的 CTF、攻击链或样本分析工作流. 后续学习应使用授权的 CTF、crackme、自有样本或本地沙盒, 并保留原始样本与证据链.

## 0x05 与 HXLoLi 的对应关系

HXLoLi 已经有一部分相同的思想, 但组织方式还没有完全变成 ReverseLab 那样的 board/case/KB/tools 路由系统. 可对照已有的 [obsidian-second-brain 中文译注](../001-obsidian-second-brain/index.md), 以及当前的 [`hx-make-ai-docs`](../../../../.agents/skills/hx-make-ai-docs/SKILL.md).

| ReverseLab 的概念 | HXLoLi 当前对应物 | 当前状态 |
| --- | --- | --- |
| Board | `ai-docs` 的分类目录和文章标签 | 有目录, 还没有统一的信号路由器 |
| Case | 一篇项目学习笔记、`.hx-mitemite.md` 和阶段性草稿 | 已经能记录一个主题的调研和问答状态 |
| KB | `ai-docs`、已有项目译注、源码引用 | 有内容, 但来源、置信度和版本快照仍主要靠人工维护 |
| Tools / MCP | `.agents/skills`、脚本和可用 MCP | 有 Skill 和脚本, 尚未形成统一的 `Signal -> Tool` 映射 |
| Exports / Reports | 引用、验证章节和最终笔记 | 目前以 Markdown 审计记录为主, 还没有统一证据目录 |
| Context chain | AGENTS.md、Skill、现有笔记和会话上下文 | 本次已通过 Skill + 答题卡实现一轮轻量路由 |

这条帖子对 HXLoLi 最有价值的不是引入某个具体注册项目, 而是提供了一套可以继续实现的学习闭环:

```text
用户主题
  -> 一个上游澄清问题
  -> 原文路径保真
  -> 项目输入/输出/依赖核验
  -> 最新 commit 和状态快照
  -> 源码深挖
  -> HXLoLi 迁移判断
  -> 用户 review
```

具体迁移建议:

1. 给每篇项目学习笔记保留“原文路径”和“事实核验”两层, 不让事实纠错破坏原作者的叙事顺序.
2. 把每个项目写成可检查的契约: `input`, `output`, `dependency`, `failure`, `evidence`.
3. 保留最新 commit URL 和核验日期, 避免 README 在未来变化后无法解释旧结论.
4. 对复杂主题使用一问一答的 grilling, 把方向责任交还给用户, 但由 Agent 先做能自行完成的探索.
5. 对逆向和协议学习优先建立本地假服务、`fake_protocol`、CTF 或 crackme 环境, 先学习状态机和证据链, 再讨论适配器边界.
6. 让 `ai-docs` 成为可 review 的中间层: AI 可以快速沉淀, 但没有用户 review 就不应当升级为正式 docs 或可复用操作手册.

## 0x06 验证与引用

### 6.1 原文来源

- [Discord 原文: 个人常用项目推荐（打野到注册机一条龙，简易教程）](https://discord.com/channels/1325587675314520175/1521896780805968013/1521896780805968013)
- 作者: `𝑏𝑎𝑖𝑞𝑖_𝑞𝑎𝑞`
- 发布时间: `2026-07-01`
- 本次通过登录态的有头 Playwright MCP 读取了帖子页面; 之后只保留文章内容所需的事实, 没有把登录凭据或浏览器状态写入仓库.

### 6.2 GitHub 核验范围

本次核验了 15 个仓库的:

- 默认分支、描述、许可证、stars、forks、open issues、`updated_at` 和最近代码推送时间.
- README、目录结构、入口文件、依赖文件和三个重点项目的关键 Skill/规则文件.
- 当前默认分支的最新 commit URL, 已在仓库表和源码深挖章节中关联.

没有把作者的主观评分、README 的营销表述或“过检测”声明当成独立运行结论. 没有对第三方真实注册流程、验证码平台、风控策略或官网私有协议进行实操验证.

## 0x07 总结

读完这条从“打野”到注册机、再到 AI 工具链的路径, 真正需要带走的不是某一个站点的注册脚本, 而是三个连续的问题:

- 你能否在每个工具节点说清楚它接收什么、产出什么、依赖什么、失败后谁负责?
- 你在让 Agent 写代码之前, 是否先用一个足够小但真正关键的问题把目标、术语和边界对齐?
- 当任务从业务自动化进入逆向学习时, 你是否已经建立了 `signal -> knowledge -> tool -> evidence` 的可回放链路?

如果这三个问题都能回答, “注册机”只是一个具体主题; 真正可复用的是从现成项目中发现能力、从源码中核验契约、再把能力组织成可维护 Agent 工作流的工程方法.
