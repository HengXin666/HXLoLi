---
title: HXLoLi 采纳方案
---

# HXLoLi 采纳方案

本文不是翻译，而是把 `obsidian-second-brain` 的机制映射到 `HXLoLi` 当前项目结构中。

## 当前项目已经有的好基础

### 独立 AI 知识库

`docusaurus.config.ts` 里已经把 `ai-docs/` 配成第二个 `@docusaurus/plugin-content-docs` 实例，路由是 `/knowledge-base`。这和 `obsidian-second-brain` 的 “AI-first 区域” 思路高度匹配。

现状优势:

- AI 生成内容不会直接污染主 `docs/`。
- 独立 sidebar: `sidebarsAiDocs.ts`。
- 独立样式: `src/css/ai-kb.css`。
- 顶部元数据组件: `src/components/AIDocHeader/index.tsx`。

建议: 保持 `docs/` 人类阅读优先，`ai-docs/` AI-first 生成/半生成优先。

### 自动侧边栏生成

已有:

- `scripts/generateSidebar.js`
- `scripts/generateAiDocsSidebar.js`

这相当于 Docusaurus 版本的 `index.md` 生成器。它已经在做目录发现、数字前缀剥离、图标和 tag.json 读取。

可学习点: `obsidian-second-brain` 强调 “Claude 先读 index，再搜索”。`HXLoLi` 可以把 sidebar 生成结果再导出一个 AI 可读索引，例如:

```text
ai-docs/_generated/ai-docs-index.json
ai-docs/_generated/docs-index.json
```

### 文档关系图

已有 `scripts/generate-docs-graph.mjs` 和 `src/pages/docs-graph.tsx`，它能从 Markdown link 和 HTML href 生成图谱。

这已经接近 `/obsidian-visualize` 的基础设施，但目前主要扫描 `docs/`。可以扩展成:

- `docs` 图。
- `blog` 图。
- `ai-docs` 图。
- 全站图。
- current doc neighborhood。
- orphan documents。
- hub documents。

### Feed 和搜索索引

已有:

- `plugins/docs-rss-plugin.mjs`
- `scripts/split-search-index.mjs`
- `static/docs-atom.xml`

这说明项目已经有“文档作为数据源”的意识。可以进一步加入 retrieval eval，衡量搜索质量，而不是只生成搜索索引。

### 私有内容链路

已有:

- `scripts/setup-private.mjs`
- `scripts/encrypt-private.mjs`
- `scripts/clean-private.mjs`
- `static/protected-pages.json`
- `static/encrypted-meta.json`

这比普通 Obsidian vault 更强。引入 AI 工作流时必须尊重这条边界: AI 自动写入默认不能碰私有/加密内容，除非显式要求。

## 和 obsidian-second-brain 的主要差距

### 1. 缺少 AI 写入规范

现在 `ai-docs/001-关于/index.md` 已经写了“使用 skill 按规范和工作流生成，需要用户 review”，但规范还没有落成机器可检查规则。

建议新增:

```text
HXLoLi/AI_RULES.md
HXLoLi/ai-docs/000-规范/index.md
```

最小规则:

- `ai-docs/` 每篇 AI 生成笔记必须有 `## 给未来 AI`。
- frontmatter 至少包含 `date`、`type`、`tags`、`ai-first: true`、`source` 或 `sources`。
- 外部事实必须有 URL 和日期。
- 推断必须标注 `confidence`。
- 不确定项写 `TBD`。
- AI 生成内容默认只能进入 `ai-docs/`，不能直接进入 `docs/` 或 `blog/`。

### 2. 缺少“写入传播”

现在生成一篇 AI 文档后，主要靠 sidebar 出现在站点里。但它不会自动:

- 记录来源。
- 更新相关专题页。
- 更新关系图描述。
- 更新操作日志。
- 关联到博客草稿。
- 关联到项目页。

建议为 `ai-docs/` 建立传播规则:

| 写入内容 | 也要更新 |
|---|---|
| 项目学习笔记 | 项目学习索引、相关项目架构页 |
| C++ 技术沉淀 | 相关 `docs/002-程序语言/001-C++/` 专题页或待 review 清单 |
| 博客草稿素材 | 对应 blog draft / idea note |
| 外部资料译注 | `_sources/` 原始来源记录、译注索引 |
| 代码库架构扫描 | `ai-docs/项目学习/HXLoLi架构/` 的 generated block |
| 任意 AI 写入 | `ai-docs/_logs/YYYY-MM-DD.md` |

### 3. 缺少 raw/source 不可变区

`obsidian-second-brain` 的 `raw/` 不可变规则很关键。对博客项目可以改成:

```text
ai-docs/_sources/
├── articles/
├── repos/
├── transcripts/
├── issue-dumps/
├── compiler-bugs/
└── screenshots/
```

规则:

- `_sources/` 只追加，不手动润色。
- 正式知识页引用 `_sources/`。
- 如果译注或综合页有争议，可以回源重建。

### 4. 缺少 health check

这次 `npm run build` 已暴露几个非阻塞 warning:

- AI docs 中已有 tag 链接到 `/knowledge-base/tags/C%2B%2B` 和 `/knowledge-base/tags/HXLibs`，但对应页面不存在。
- 既有博客 tags 未在 `tags.yml` 定义。
- 一些 drawio svg 不能被 `image-size` 正确读取。
- Browserslist 数据过旧。
- 部分既有数学段落触发 KaTeX 中文 warning。

建议新增:

```text
scripts/check-ai-docs.mjs
scripts/check-content-health.mjs
```

检查项:

- `ai-docs/` 是否有 `## 给未来 AI`。
- frontmatter 是否有 `date/type/tags/ai-first/source`。
- Markdown 内链是否存在。
- 图片是否存在。
- `AIDocHeader` 渲染的 tags 是否有对应 tags 页面，或改为不可点击。
- `docs/`、`blog/`、`ai-docs/` 是否有重复标题。
- 是否存在孤立 AI 文档。
- 是否有未转义 `$` 误触 KaTeX。
- 私有内容是否进入 feed/search。

### 5. 缺少 sentinel 安全再生成

你已经有自动生成文件，例如 `sidebarsAiDocs.ts`、`static/docusaurus-graph.json`。如果之后让 AI 生成架构文档、项目状态、翻译索引，必须避免覆盖人工补充。

建议在 AI 维护文档里采用:

```markdown
<!-- @generated:start -->
自动生成，可刷新
<!-- @generated:end -->

<!-- @user:start -->
人工补充，禁止覆盖
<!-- @user:end -->
```

### 6. 缺少 retrieval eval

现在项目有搜索和图谱，但没有“检索质量数字”。可以学习 `/obsidian-retrieval-eval`:

```text
scripts/eval-search.mjs
```

评估集格式:

```json
{"question":"MSVC requires 缺陷那篇在哪里?","gold":"blog/2025/09/07/01-MSVC的requires缺陷.md"}
{"question":"协程调度器知识沉淀在哪?","gold":"ai-docs/002-知识沉淀/001-现代C++/001-日常探索/001-HXLibs编写串行协程调度器/index.md"}
```

指标:

- recall@1
- recall@3
- recall@5
- MRR
- 错排第一的文档是谁

这样以后优化中文搜索、标题、tags、摘要时有依据。

## 建议先落地的 5 个动作

### 动作 1: 写 `AI_RULES.md`

这是最小但最关键的迁移。没有规则，AI 生成内容会很快变成不可审计的散文。

建议内容:

- 三个内容区边界: `blog/`、`docs/`、`ai-docs/`。
- AI-first frontmatter。
- `## 给未来 AI` 模板。
- 来源和置信度规则。
- 私有内容规则。
- sentinel 规则。
- review 后毕业到 `docs/`/`blog/` 的规则。

### 动作 2: 建 health check

先做只读检查，不自动修:

```bash
node scripts/check-ai-docs.mjs
node scripts/check-content-health.mjs
```

这比一开始做自动 agent 安全。

### 动作 3: 扩展文档图谱

把 `scripts/generate-docs-graph.mjs` 泛化:

```bash
node scripts/generate-content-graph.mjs --scope docs
node scripts/generate-content-graph.mjs --scope ai-docs
node scripts/generate-content-graph.mjs --scope blog
node scripts/generate-content-graph.mjs --scope all
```

图谱页面可以加 scope 切换。

### 动作 4: 做 `hx-architect`

为 `HXLoLi` 自己生成架构笔记:

```text
ai-docs/002-知识沉淀/002-项目学习/002-HXLoLi架构/
├── index.md
├── docusaurus-content-pipeline.md
├── ai-docs-system.md
├── private-content-pipeline.md
├── anime-module.md
├── music-player-module.md
└── decisions.md
```

每篇用 sentinel，把扫描结果和人工解释分开。

### 动作 5: 建保存对话流程

类似 `/obsidian-save`，但不要自动改正式文档。建议命名:

```text
hx-save
```

默认行为:

- 从当前 AI 对话提取要点。
- 生成到 `ai-docs/002-知识沉淀/...` 或 `ai-docs/_inbox/`。
- 写 `ai-docs/_logs/YYYY-MM-DD.md`。
- 标注 `review_status: pending`。
- 人工确认后再毕业到 `docs/` 或 `blog/`。

## 不建议现在做的事

- 不建议把全部 `obsidian-second-brain` 命令复制进来。Docusaurus 博客和 Obsidian vault 的写入语义不同。
- 不建议启用后台 agent 自动改仓库。博客是公开输出，默认应生成 pending review 文档。
- 不建议先做复杂向量库。先用现有搜索和图谱做 retrieval eval，确认瓶颈。
- 不建议让 AI 直接修改加密私有内容链路。

## 一阶段目标

第一阶段做到这些就够:

- AI 写入有规则。
- AI 文档有来源、有时间、有置信度。
- 自动生成内容不会覆盖人工内容。
- 站点健康检查能发现问题。
- 图谱覆盖 `ai-docs/`。
- 对话沉淀先进入 pending review。

这就是 `obsidian-second-brain` 最适合 `HXLoLi` 的部分: 不是换工具，而是给已有博客知识系统加上记忆规则、审计规则和传播规则。
