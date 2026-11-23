---
hxid: "hx-00d11db9"
title: "把 Agent 的纠正变成确定性检查"
created_at: "2026-11-22"
model: "hark"
skill: ["hx-ai-docs"]
authors: "Heng_Xin"
tags: ["AI Agent", "Harness", "工程实践"]
mdx:
  format: mdx
---

# 把 Agent 的纠正变成确定性检查

<Lead
why="同一套规则, 换个模型写出来的东西就不一样. 规则越写越长, 效果还是看模型心情"claim="纠正写成文字, 每个模型理解不同; 写成检查, 谁来都一样"use="拿到一条新纠正时, 知道先往哪里放: 架构, lint, 最后才是文字"/>

来源是 Grok Bot 的 Lauren Tan (poteto) 在 AI Engineer Code Summit 的演讲 [All Code Is Slop (Except Mine)](https://www.youtube.com/watch?v=O_zcc6Xl12g), 17 分钟

## 一个月 3000 个 PR, 回滚率约 0.5%

<Stats items={[
{ value: "3,000", label: "她个人上个月上线的 PR", source: "演讲 0:44", verified: false }, { value: "11,000", label: "团队上个月上线的 PR", source: "演讲 13:37", verified: false }, { value: "约 0.5%", label: "被回滚的 PR 比例", source: "演讲 14:21", verified: false }, { value: "227", label: "10 月至少提过一个 PR 的人", source: "演讲 14:04", verified: false }, ]} />

速度这么猛, <Gold>回滚率反而稳定</Gold>, 应用也比刚上线时快. 她把这叫做信任: 不是盲目信任 Agent, 是投入它的运行环境, 让人有底气相信它会做对. 这一步她花了 6 个月

## 倒金字塔: 先改代码, 最后才写字

<Figure src={require('./fig-pyramid.svg').default} alt="倒金字塔: 架构, lint, 验证, 规则四层" caption="越靠上投入越多. 前两层是确定性的: 同一段代码, 换谁来跑结果都一样" />

## 每纠正一次, 先问能不能写成检查

<Figure src={require('./fig-fork.svg').default} alt="纠正 Agent 后的三条去路" caption="按顺序试: 能用架构整类消灭就不写 lint, 能写 lint 就不写文字. 前两条走完, review 要盯的东西越来越少" />

做够次数之后, review 要盯的东西越来越少. 她说到某一天发现没什么可审了, <Gold>大多数 PR 直接让 Agent 自己合</Gold>

<Evidence source="Lauren Tan, All Code Is Slop" at="3:12">
whenever you're correcting your agent, think about how you can make that a deterministic check instead of adding another line to your rules or your agents.md</Evidence>

## 代码库是下一个 Agent 的范本

2024 年她审同事写的 3 个 PR, 现在是约 300 个, 每个工程师手下都是成百上千个 Agent. Agent 写代码前会读现有代码, 临时变通和反模式会被当成「这里就是这么写的」照抄

<Figure src={require('./fig-garden.svg').default} alt="一处反模式被多个 Agent 照抄" caption="她把代码库比作花园, 工程师的活越来越像除草" />

所以立新规则时, 顺手把旧的反模式一起清掉. <Red>只挡新的不清旧的, 下一个 Agent 照样能从旧代码里学坏</Red>

## 要分寸才能用好的东西, 关进框架里

她的经验: <Red>凡是需要分寸感才能用好的东西, 往往导致垃圾代码</Red>. React 的 `useEffect` 在业务代码里直接禁掉, 只在自研框架 Dune 内部开放

另一个例子是用目录传递边界. Agent 常把 Electron 主线程代码导入渲染线程, 导致卡顿. 他们设了 `electron-main` 和 `electron-renderer` 两个顶层目录, 用依赖图和 lint 禁止互相导入

<Figure src={require('./fig-boundary.svg').default} alt="useEffect 关进 Dune, electron 两个目录禁止互相导入" caption="两种关法: 只在框架里开放, 或者用目录把边界画出来" />

## 注释给了 Agent 一个找借口的地方

Agent 用 setTimeout 给架构冲突打补丁, 再写一大段注释说明为什么合理, 结果定时器蔓延, 频繁 GC, 应用变卡

<Figure src={require('./fig-comments.svg').default} alt="注释和 lint 两条路的对照" caption="Grok Bot 禁掉了 99% 的注释, 理由是注释无法强制执行. 同样是打补丁, lint 会把 Agent 推去修根因" />

她总结的另一个问题: 你随口纠正一句, Agent 会反应过度, 当成永不能破的铁律. <Red>写在文字里的东西, 总会被这样过度解读或者找借口</Red>

## review 看的是验证产物

他们现在 review 主要看视频、截图、图表和形式化验证的结果, 少看代码本身

<Figure src={require('./fig-review.svg').default} alt="功能地图, 自定义 CLI 和验证产物如何汇到 review" caption="Agent 借功能地图定位, 用自定义 CLI 驱动应用, 产出的视频, 截图, 图表和形式化结果才是 review 的对象" />

- **运行时验证**: Agent 自己把应用跑起来, 用 Chrome DevTools Protocol 调试
- **功能地图**: 从用户视角列出所有功能, 你说「左侧栏那个东西」, Agent 不用猜
- **自定义 CLI**: 驱动应用的操作封成 CLI, skill 每次复用, 省 token, CLI 也能测
- **形式化**: TLA+ 检查跨设备通知流程, Lean 证明虚拟化库不闪烁, 仍在试验

这些工具都不新, 以前贵在难写难维护, Agent 把成本压下来了. 她引用 Cindy Sridharan 的说法: <Gold>每天约 70% 的 token 花在验证上</Gold>

## 先修内循环, 再搭外循环

<Figure src={require('./fig-loops.svg').default} alt="先修内循环, 再搭外循环" caption="底座不牢就自动化外循环, 只会更快地产出垃圾" />

## 放到我的仓库里看

代码侧我已经在这条路上了. 工作区根目录的 `.agents/coding-rules.md` 按执行方式分了三档: 自动拦截, 手动运行, 约定尚无 checker. 按她的思路, 一条规则只有一个方向: 往上升档

<Figure src={require('./fig-ladder.svg').default} alt="约定, 手动运行, 自动拦截三档" caption="我的仓库里三档各有一个例子, 下面是具体要升的那几条" />

<Apply tiers={["约定", "手动运行", "自动拦截"]} items={[
{ path: ".agents/coding-rules.md", now: "hx-agent-code 的 scripts 只写 .ts, 还是约定", next: "加一个扩展名检查, 升到自动档", from: 0, to: 2, label: "scripts 只写 .ts", target: "扩展名检查" }, { path: "HXLoLi/.agents/skills/hx-make-skill", now: "校验要手动跑, 没有 hook 或 CI", next: "接上 hook, 就是自动档", from: 1, to: 2, label: "hx-make-skill 校验", target: "接上 hook" }, { path: "HXLoLi/ai-docs/.hx-voice.toml", now: "69 条坏表达, 只有 21 条带 pattern 能被 lint 拦", next: "剩下的逐条判断: 能写 pattern 就写, 写不了就承认只是提醒", from: 0, to: 1, label: "48 条坏表达", target: "能写 pattern 的" }, ]} />

Agent Notes 的锚点注释和「禁注释」不冲突. 她反对的是无法强制的注释, 我的锚点由 `verify.py` 核对配对, 是受检查的引用

花园也一样. 45 篇 ai-docs 里 44 篇用 `## 0xNN` 作节标题, 而我手写的东西里一处都没有. 新模型写笔记会先读旧笔记, <Red>它学到的是这 44 篇</Red>

## 什么时候不成立

<Boundary items={[
"3,000 / 11,000 / 约 0.5% / 70% 都是演讲自述, 没有公开数据可核对; 回滚率和这套做法的因果, 她也没给对照", "她的前提是代码有编译器、类型和测试兜底. 文字没有真值, 写作检查会误报, 只能拦一部分", "禁注释建立在有 lint 能替代上. 没有对应检查的地方直接删注释, 会丢信息", "形式化验证她自己说仍在试验, 不是银弹", ]} />
