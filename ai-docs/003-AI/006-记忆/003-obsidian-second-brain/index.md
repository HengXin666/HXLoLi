---
hxid: "hx-a953a087"
title: "obsidian-second-brain 怎么让 AI 自己维护知识库: 三个钩子加一套写给模型的规则"
created_at: "2026-09-13"
model: "hark"
skill: ["hx-ai-docs"]
authors: "Heng_Xin"
tags: ["记忆系统", "知识库", "Skill", "AI Agent"]
mdx:
  format: mdx
---

# obsidian-second-brain 怎么让 AI 自己维护知识库: 三个钩子加一套写给模型的规则

<Lead
why="让 AI 长期维护一个笔记库, 难的不是写, 是它每次开会话都忘了上次定下的规矩"claim="它把最要紧的规矩做成了三个钩子, 每次必定执行. 剩下的写成给模型读的规则, 靠模型自觉"use="给自己的知识库或 Agent 定规矩时, 分得清哪条该写成钩子, 哪条写成文字就够"/>

来源是 [obsidian-second-brain](https://github.com/eugeniughelbur/obsidian-second-brain) 提交 `bf63932` 的源码. 它不是 Obsidian 插件, 是一套跑在 Claude Code 这类 CLI 里的 skill, vault 对它来说就是一堆 Markdown 文件

## 模型每次都失忆, 规矩要放在一定会执行的地方

这个项目想让 Claude 长期维护一个 vault: 存决策, 记人物, 整理研究. 它定了一大套写法, 比如每篇笔记开头写 `## For future Claude`, frontmatter 必须有 `date`, `type`, `tags`, `ai-first: true`

麻烦在于 Claude 每次开新会话都从零开始. 规矩写在 skill 里, 它要想起来去读才算数. 会话一长, 上下文被压缩, 刚定下的决定也跟着丢了

它的办法是挑出三个最容易出事的时刻, 各挂一个钩子. 钩子是 Claude Code 在固定事件上调用的脚本, <Red>模型忘没忘都会跑</Red>

<Figure src={require('./fig-hooks.svg').default} alt="会话开始, 写笔记, 上下文压缩三处各挂一个钩子" caption="开头补记忆, 写完查格式, 压缩前抢救决定" />

下面三节一个钩子一节, 再看那些只能靠模型自觉的规则

## 会话一开始, 就把操作手册塞进上下文

朴素做法是在 skill 里写 "先读 vault 根目录的 `_CLAUDE.md`". 模型多半会读, 但每条命令都可能重读一遍, 还会先 `ls`, `Glob` 一通去找 vault 在哪

`hooks/load_vault_context.py` 挂在 SessionStart 上. 它只在会话的工作目录位于 `$OBSIDIAN_VAULT_PATH` 里面时才动作, 不在就什么都不输出, 免得往无关会话里塞内容

动作是把整篇 `_CLAUDE.md` 作为 `additionalContext` 注入, 前面加一段头部: vault 根目录, `index.md` 和 `log.md` 的绝对路径, 再加一句 "不要用 ls, Glob, Bash 去发现 vault 的目录"

`_CLAUDE.md` 是每个 vault 自己的手册: 目录地图, 每种笔记的 frontmatter, 哪些能自动存, 哪些要先问. <Gold>它和 skill 冲突时, 以 `_CLAUDE.md` 为准</Gold>

## 每次写完都查格式, 只警告不回滚

假设模型存了一篇人物笔记, frontmatter 里漏了 `ai-first: true`, 正文没写 `## For future Claude`, 还顺手用了一个全角破折号 `—`. 这种错单篇看无所谓, 积累一年, 检索和批处理脚本就开始漏东西

`hooks/validate-ai-first.sh` 挂在 PostToolUse 上, 每次 Write 或 Edit 之后检查写出的文件. 只查 vault 里的 `.md`, 跳过 `raw/`, `templates/`, `.git/` 这些目录

```text
1. frontmatter 的 --- 成对
2. frontmatter 里没有 tab
3. 有 date, type, tags, ai-first: true
4. 正文有 ## For future Claude
5. 没有禁用字符: 弯引号, 破折号, ≥, 省略号, 不换行空格
```

有问题就把每条警告和建议替换写到 stderr, 退出码 1. <Red>写入不回滚</Red>, 模型在工具结果里看到警告, 自己去改

为什么不直接拦下? 拦下会让一次正常的保存失败, 模型可能换个方式重写, 反而丢内容. 只警告的代价是, 模型不理警告时错误会留在库里

## 压缩前定下的事, 由后台 agent 抢救进 vault

会话聊了两小时, 中间定了 "API 先不用 Rust 重写". 上下文一压缩, 这个决定只剩摘要里的一句话, 会话结束就没了

`hooks/obsidian-bg-agent.sh` 挂在 PostCompact 上. 它从 transcript JSONL 里找出最新一条 `isCompactSummary` 的摘要, 拼成一段提示, 在 vault 目录里后台起一个 `claude -p` 去写

那段提示规定了要抓什么 (决策, 任务, 人物, 开发记录, 想法) 和怎么写: 先搜再建, 更新当天的 daily note, 不删除, 不归档, 不合并, 只增改

这个后台 agent 带着 `--dangerously-skip-permissions`, 没人看着就会改文件. 所以它默认是关的: 要 `OBSIDIAN_VAULT_PATH` 和 `OBSIDIAN_BG_AGENT_ENABLED=1` 都设了才运行. 安装脚本只设前一个, <Gold>第二个开关必须用户自己打开</Gold>

## 会重复生成的页面, 用标记分出人工区

`/obsidian-architect` 扫代码库生成架构笔记, 之后可以反复重跑. 朴素做法是每次整篇覆盖, 用户在里面补的几行说明第一次刷新就没了

<Compare caption="刷新时只换标记里面的内容" left={{
title: "整篇覆盖", items: ["每次重新生成整篇", "人工补的内容被冲掉", "用户不敢重跑"], }} right={{title: "sentinel 标记", items: ["机器内容包在 @generated:start 和 end 之间", "刷新只替换这一段", "标记外和 @user 段一律当人工内容"], }} />

第一次生成时还没有标记, 规则要求把生成的内容先包进 `@generated`, 以后的刷新才安全

注意这一条只写在 `references/write-rules.md` 里, 没有脚本去检查. 刷新时守不守规矩, 全看模型

## 事实变了不删旧值, 记两个时间

某人从 CTO 变成 Architect. 朴素做法是直接改成 Architect, 于是 "一月时谁是 CTO" 就查不到了

它的 schema 要求顶层字段写现状, 历史放进 `timeline`, 每条带两种时间: `from` 和 `until` 是现实中什么时候为真, `learned` 是 vault 什么时候知道的

<Figure src={require('./fig-bitemporal.svg').default} alt="CTO 和 Architect 两段任期, 以及 vault 分别在什么时候得知" caption="两条时间分开记, 才能区分 '当时是什么' 和 '我们当时以为是什么'" />

有了 `learned`, "我周三之后为什么改了看法" 也能查: 那天 vault 得知了什么. 这同样只是写在 schema 里的规则, 没有脚本校验

## 健康检查只报告, 破坏性的修复要先问

`scripts/vault_health.py` 扫整个 vault, 报孤立笔记, 疑似重复, 过期任务, 缺 frontmatter 这些问题

查重是两步: 文件名规范化后相同的分到一组, 再比正文前 1000 字的相似度, 不低于 0.6 才报 warning, 否则只是 info. 早期版本把日期也去掉了, 一整个日期系列的笔记全被报成重复

报告出来之后, 补 frontmatter 这类安全的修复可以直接做. 归档, 合并, 调和矛盾要列出来, 等用户明确确认

## 哪些规则真的被执行

把上面的规则按 "谁来保证" 分一下, 项目的设计就很清楚了

<Compare caption="同一套写法, 一半靠代码, 一半靠模型" left={{
title: "只靠模型自觉", items: ["写入后传播到 daily note 和看板", "先搜再建, 不重复", "sentinel 刷新", "双时间 timeline", "没搜完不许说不存在"], }} right={{title: "代码保证", items: ["会话开始注入 _CLAUDE.md", "写完检查格式", "压缩后后台保存", "vault_health.py 扫孤立和重复"], }} />

<Red>出事后最难发现的那几条, 恰好在左边</Red>. 格式错了有警告, 传播漏了, sentinel 被冲掉了, 没人会告诉你

## 放到我的项目里

- **hx-ai-docs 的门禁接成钩子**: 现在 `check-kb-note.mjs` 和 `prose_rules.py` 要手动跑, 和它的 `validate-ai-first.sh` 是一回事, 只差挂到写入之后自动跑. 也照它的做法只警告
- **记忆层先挑三个时刻**: 我的 Agent 记忆设计里, 会话开始注入手册, 写入后校验, 压缩前抢救, 这三处值得做成确定性代码, 其余写成规则. 见 [自维护可插拔记忆层设计](../002-自维护可插拔记忆层设计/index.md "hxid:hx-37f17262")
- **会重复生成的文档加 sentinel**: 生成器产出又允许人工补充的文件, 都按 `@generated` 和 `@user` 分区

## 什么时候不成立

<Boundary items={[
"钩子依赖 Claude Code 的 SessionStart, PostToolUse, PostCompact 事件. 别的 CLI 没有这些事件, 只剩文字规则", "后台 agent 用 --dangerously-skip-permissions 无人值守写 vault, 写错了要靠事后看 /tmp/obsidian-bg-agent.log 才发现", "格式检查只警告不回滚, 模型不理警告时错误照样留在库里", "传播, 查重, sentinel, 双时间都没有脚本保证, 换一个不守规矩的模型就会退化", "禁用字符表是按英文笔记定的 (弯引号, 破折号, 省略号). 中文正文常用这些符号, 照搬到中文库会频繁报警", ]} />

## 留着查

- 三个钩子: [load_vault_context.py](https://github.com/eugeniughelbur/obsidian-second-brain/blob/bf63932d73e50de334fe46eabfb72cf18f7239bd/hooks/load_vault_context.py), [validate-ai-first.sh](https://github.com/eugeniughelbur/obsidian-second-brain/blob/bf63932d73e50de334fe46eabfb72cf18f7239bd/hooks/validate-ai-first.sh), [obsidian-bg-agent.sh](https://github.com/eugeniughelbur/obsidian-second-brain/blob/bf63932d73e50de334fe46eabfb72cf18f7239bd/hooks/obsidian-bg-agent.sh)
- 写入规则 (传播, sentinel, 先搜再建): [references/write-rules.md](https://github.com/eugeniughelbur/obsidian-second-brain/blob/bf63932d73e50de334fe46eabfb72cf18f7239bd/references/write-rules.md)
- 双时间 schema: [references/vault-schema.md](https://github.com/eugeniughelbur/obsidian-second-brain/blob/bf63932d73e50de334fe46eabfb72cf18f7239bd/references/vault-schema.md)
- 健康检查: [scripts/vault_health.py](https://github.com/eugeniughelbur/obsidian-second-brain/blob/bf63932d73e50de334fe46eabfb72cf18f7239bd/scripts/vault_health.py)
- 本地副本在 `HXLoLis/ref/obsidian-second-brain`
