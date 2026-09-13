---
name: hx-to-ai-docs
description: "HXLoLi ai-docs 知识沉淀的人类调度入口 (已设 disable-model-invocation, 模型不会自动加载). 负责把一次沉淀请求路由到正确的子技能并守住整条流水线 —— hx-docs-grill (反问达成共识 + .hx-mitemite.md 答题卡)、hx-docs-sediment (视频/文章/定制页面/本地变更/调研五条解析路径 + makeDoc.py 模板初始化)、hx-docs-layout (目录命名/HXLoLiTag/双链/成文结构/标点)、hx-docs-ppt (#PPT 侧车演示页)、hx-docs-organize (存量重分类/hxid 唯一 ID/tag 倒排树). Use when 人类显式调用本技能, 或说 `沉淀到 ai-docs`, `写一篇 ai-docs`, `用 hx-to-ai-docs 沉淀`, `整理一下 ai-docs`."
disable-model-invocation: true
---

# hx-to-ai-docs

沉淀一篇 HXLoLi ai-docs 笔记的**路由入口**. 本身不定义写作规范, 只做三件事: 路由、守顺序、守红线.

## 子技能分工 (按需加载, 不要全读)

| 子技能 | 管什么 | 什么时候调 |
|---|---|---|
| `hx-docs-grill` | design-tree/frontier 反问; `.hx-mitemite.md` 答题卡读写脚本 | 目标/读者/边界/取舍没谈成共识时 (几乎每次都要) |
| `hx-docs-sediment` | 五条解析路径 (视频/文章/定制/本地变更/调研) + `makeDoc.py` 建模板 | 需要采集素材或初始化 `index.md` 时 |
| `hx-docs-layout` | 目录命名与分类、HXLoLi-MD 规范、HXLoLiTag 词表、双链、成文结构、标点脚本 | 定目录/写正文/收尾/单篇体检时 |
| `hx-docs-organize` | 存量重分类 (聚合/独立/整类重做)、hxid 唯一 ID 与链接重算、tag 倒排树、迁移协议 | 对**已有**笔记重新归类/改名/搬家/整理 tag 时 |
| `hx-docs-ppt` | `#ppt` 自包含 16:9 演示页侧车 | 笔记需要演示页时 |
| `hx-look-video` | 视频/音频/字幕 → transcript | 素材是视频且要用到口播内容时 |
| `archify` | 架构/流程/时序/数据流/状态图 → 可交互 HTML | 正文需要机制图时 |

**术语对照** (人类可能用口述名): 脑电波同步 = `hx-docs-grill`; `hx-html-ppt` = `hx-docs-ppt`.

## 两条入口

- **新增一篇** -> 走下面的固定流水线.
- **整理存量** (重新分类/改名/搬家/整理 tag) -> 走 `hx-docs-organize`, 它有自己的体检 + 申请 + 执行 + 闸门顺序; 不要用沉淀流水线去改已有笔记的归属.

## 固定流水线 (新增)

1. **谈方向** → `hx-docs-grill`. frontier 为空 (每条分支都走过、没有静默假设) 且人类确认前, 不要动笔.
2. **采素材** → `hx-docs-sediment` 选路径. 视频先过 `hx-look-video` 拿 transcript.
3. **建目录 + 初始化模板** → `makeDoc.py` (经 `hx-docs-sediment`). 禁止手建空 `index.md`; 已存在则读取后编辑.
4. **成文** → `hx-docs-layout`: 开头引入 / 正文图文并茂 / 结尾升华展望 / 末章列来源; 全程 HXLoLi-MD 与标点规范.
5. **配图与演示页** → `archify` (图) 与 `hx-docs-ppt` (演示页), 产物与 `index.md` **同目录**, 正文用 `#ppt` 链接内嵌.
6. **收尾注册** → `node scripts/generateAiDocsSidebar.js` (新增/改名/移动目录后必跑) + `format_cn_punct.py --check`.

跨文章引用一律写成 `[标题](hxid:hx-xxxxxxxx)` (ID 由 `makeDoc.py` 创建时生成), 而不是写死相对路径 —— 这样后续重分类时链接不会断. 同目录内引用仍用普通相对路径.

完整的产出物清单、验证闸门、本地开发联动 (VS Code 跳转 / 答题卡 UI) 见 [references/pipeline.md](references/pipeline.md).

## 红线 (任何子技能都不得违反)

- **责任在人类**: 方向由人类把握, 你只辅助. 禁止自作主张开始沉淀. 一次只问当前最关键的一个问题, 并附推荐答案.
- **先出阶段性成果再反问**: 大纲、章节概要、标题候选 — 有了半成品再问, 不要问空问题.
- **读者视角铁律**: 正文只写面向读者的内容. 生成过程、工具链、本地路径、验证步骤、provenance、审核提示**一律不进正文**; review 要点只写进 `.hx-mitemite.md` 与对话.
- **不臆造**: 查不到就说查不到; 无法确认的信息标 `[不确定]`; 不确定当前模型时 frontmatter 允许 `Unknown`, 禁止编造.
- **可扩展**: 新素材类型 = 新增/扩展一个子技能的 reference, 而不是往入口里堆分支. 入口保持简单明了.

## 参考文件

- [references/pipeline.md](references/pipeline.md) —— 什么时候读: 需要确认产出物清单与目录约定、收尾验证闸门、或本地开发联动 (`run.sh` / dev-edit-server / 答题卡 UI) 时.
- [references/model-selection.md](references/model-selection.md) —— 什么时候读: 要决定 `--model` 传什么、或需要向人类说明"该用哪个模型来沉淀"时.
