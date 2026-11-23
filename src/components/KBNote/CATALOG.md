# KBNote 组件目录

给写知识库笔记的模型看的. 每个组件一段真实范例, 都来自 `ai-docs/003-AI/004-评测与验证/002-把Agent的纠正变成确定性检查/index.md`. 照着写, 不要发明新组件, 不要传样式.

写完跑:

```bash
node scripts/check-kb-note.mjs <笔记目录>
```

退出码不是 0 就按报错改, 改到通过为止.

## 先定骨架

- frontmatter 加两行 `mdx:` 和缩进的 `  format: mdx` (顶层 `format:` Docusaurus 不认), 其它字段照旧
- `# 标题` 之后紧跟一个 `<Lead>`, 再写一句来源
- 每个 `##` 是一个论点, 标题本身就是结论, 不写 `## 0xNN`
- 每节最多一个组件, 组件前后各一两句话把它接住. 组件负责结构, 正文负责为什么
- 最后两节固定: `## 放到我的仓库里看` (用 `<Apply>`), `## 什么时候不成立` (用 `<Boundary>`)
- 没有合适的组件就写 Markdown 列表, 不要硬套

## Lead: 开头三行

什么时候用: 每篇都要, 只能一个. 第一次读的人靠它知道为什么要读, 重读的人靠它三秒回忆起内容

```mdx
<Lead
  why="同一套规则, 换个模型写出来的东西就不一样. 规则越写越长, 效果还是看模型心情"
  claim="纠正写成文字, 每个模型理解不同; 写成检查, 谁来都一样"
  use="拿到一条新纠正时, 知道先往哪里放: 架构, lint, 最后才是文字"
/>
```

## Stats: 关键数字

什么时候用: 一节的论点靠几个数字撑着. 每个数字必须带 `source`; 来源是自述或没核实, 写 `verified: false`, 页面会自动标 [不确定]

```mdx
<Stats items={[
  { value: "3,000", label: "她个人上个月上线的 PR", source: "演讲 0:44", verified: false },
  { value: "约 0.5%", label: "被回滚的 PR 比例", source: "演讲 14:21", verified: false },
]} />
```

## Layers: 分层, 金字塔

什么时候用: 几样东西有上下层次, 而且有"多少"之分. `weight` 是真实的相对投入或体量, 条宽按它等比算, 不要为了好看改数

```mdx
<Layers caption="越靠上投入越多" items={[
  { name: "API 设计 / 架构 / 重构", desc: "让错的写法写不出来", weight: 4, tag: "确定性", tone: "good" },
  { name: "规则 / skill / AGENTS.md", desc: "只留真要判断力的", weight: 1, tone: "last" },
]} />
```

## Fork: 一个起点, 按顺序试的几条路

什么时候用: 碰到某种情况时有几种处理, 而且有先后. 顺序就是数组顺序

```mdx
<Fork from="纠正了 Agent 一次" caption="按顺序试, 前一条走不通才往下" options={[
  { name: "用 API 或架构整类消灭", desc: "这类错以后写不出来", tone: "good" },
  { name: "写进规则文字", desc: "最后手段", tone: "last" },
]} />
```

## Flow: 线性流程

什么时候用: A 导致 B 导致 C, 或者一条流水线. 三到五步, 再多就该拆

```mdx
<Flow caption="她把代码库比作花园" steps={[
  { name: "旧代码里有一处反模式", tone: "bad" },
  { name: "Agent 读到, 当成惯例", tone: "ok" },
  { name: "一处变十处", tone: "bad" },
]} />
```

## Compare: 两边对照

什么时候用: 以前和现在, 错的和对的. 左边默认是坏的, 右边默认是好的

```mdx
<Compare caption="理由是注释无法强制执行" left={{
  title: "以前: 补丁 + 注释",
  items: ["Agent 用 setTimeout 打补丁", "再写一段注释说明为什么合理"],
}} right={{
  title: "现在: lint",
  items: ["这类写法直接报错", "报错信息指向根因"],
}} />
```

## Evidence: 折叠的原文

什么时候用: 一句关键结论要给出原话. 默认收起, 不打断阅读

```mdx
<Evidence source="Lauren Tan, All Code Is Slop" at="3:12">
whenever you're correcting your agent, think about how you can make that a deterministic check
</Evidence>
```

## Apply: 落到我的项目

什么时候用: 每篇的 `## 放到我的仓库里看`. `path` 相对 HXLoLis 根目录, 检查脚本会核对它存在. `now` 写现状, `next` 写具体下一步, 不写"可以考虑"

优先用升档图: 给 `tiers` (从低到高的几档), 每项再写 `from` / `to` (档的序号, 从 0 数, from < to) 和两颗胶囊上的短字 `label` / `target` (各不超过 16 字, 不算空格). 画出来是一行一条: 虚线胶囊在现在那档, 粉框胶囊在目标档, 中间一根箭头; 行下小字是路径和 `next`, 悬停看完整的现在/下一步. 不给 `tiers` 时退回一行一条的列表 (旧写法)

```mdx
<Apply tiers={["约定", "手动运行", "自动拦截"]} items={[
  { path: "HXLoLi/.agents/skills/hx-make-skill", now: "校验要手动跑, 没有 hook 或 CI", next: "接上 hook, 就是自动档", from: 1, to: 2, label: "hx-make-skill 校验", target: "接上 hook" },
]} />
```

档要按这篇自己的语境起名 (例如 约定 / 手动 / 自动, 或 文字 / lint / 架构), 不要硬套

## Boundary: 什么时候不成立

什么时候用: 每篇都要. 写这套结论失效的具体条件, 和来源自己没证明的地方

```mdx
<Boundary items={[
  "3,000 / 约 0.5% 都是演讲自述, 没有公开数据可核对",
  "禁注释建立在有 lint 能替代上. 没有对应检查的地方直接删注释, 会丢信息",
]} />
```

## Figure: 结构图片

什么时候用: 上面几个组件画不出的结构, 比如架构图. 图片放笔记同目录; `alt` 和 `caption` 必填. 不放装饰图

```mdx
<Figure src={require('./crawler-layers.png').default} alt="爬虫库分层" caption="三层: 抓取, 解析, 调度" />
```

## Chart: 数据对比图

什么时候用: 前面讲清楚了量什么, 后面拿同一把尺子量出来的一组数横着比. 至少 4 项, 而且必须是同一来源, 同一测试条件. 凑不齐就别画, 给原始榜单链接

`kind="bar"` 横向条形. `group` 相同的排在一起, 不同测试条件分组, 只在组内比. `muted: true` 表示只见于摘要, 画成虚条. `kind="range"` 区间条, 比如官方价到现价. 数据顺序就是显示顺序

```mdx
<Chart kind="bar" unit="h" source="Notebookcheck Wi-Fi 网页续航" caption="同一套测试才能横着比" data={[
  { name: "小米 18 Fold", value: 26.8, label: "26h46m" },
  { name: "vivo X Fold6", value: 20.2 },
]} />
<Chart kind="range" unit="元" source="官方价, 2026-11 电商价" caption="左端是现价" data={[
  { name: "iPhone Air", from: 8799, to: 4989, label: "-43%" },
]} />
```

## 写不进组件的

- 正文每段不超过 100 字, 超了拆段或改成列表
- 不写 Markdown 表格, 要对照就用 `<Compare>`, 要列数字就用 `<Stats>`, 一组可比的数用 `<Chart>`
- 不写 HTML 标签, 不传 `style` / `className`
- 块组件包正文时 (比如 `<Evidence>`), 开标签和闭标签各自单独一行, 和正文之间各空一行. 提交钩子会合并折行, 不空行就会把闭标签吸到正文行尾, MDX 编译失败

## Red / Gold: 行内标注

一句话里最该记住的几个字. 一节最多一处, 每处不超过 30 字, 不要整段上色. 普通 `**加粗**` 只加粗不变色.

- `<Red>`: 判断和警示, 读者要带走的结论或踩坑点
- `<Gold>`: 关键事实, 数字或原话里的要点

```mdx
所以立新规则时, 顺手把旧的反模式一起清掉. <Red>只挡新的不清旧的, 下一个 Agent 照样能从旧代码里学坏</Red>
```
