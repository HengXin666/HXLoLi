---
title: HXLoLi 可借鉴点
---

# HXLoLi 可借鉴点

本页基于 `ref/obsidian-second-brain` 的中文译注，以及对 `HXLoLi` 当前结构的快速阅读。它不是最终改造方案，而是下一步可以落地的学习清单。

## 当前 HXLoLi 已具备的基础

`HXLoLi` 已经不是单纯博客，项目里有这些可承接点:

- `blog/`: 时间序列文章，适合作为公开输出。
- `docs/`: 人类阅读优先的系统笔记。
- `ai-docs/`: AI 沉淀区，README 已明确“使用 skill 按规范和工作流生成，需要用户 review 后提交”。
- `scripts/generateAiDocsSidebar.js`: 可自动生成 AI 知识库侧边栏。
- `scripts/generate-docs-graph.mjs`: 已有文档关系图数据生成能力。
- `plugins/docs-rss-plugin.mjs`: docs 也具备 RSS/Atom 输出。
- `scripts/count-md-words.ts`、`data/wordStats.ts`: 已经在统计文档/文章。
- `scripts/encrypt-private.mjs`、`static/protected-pages.json`: 已有私有内容处理链路。

因此，最值得学习的不是 UI，而是工作流和约束。

## 最值得迁移的 8 个设计

### 1. 给 `ai-docs/` 建立 AI-first 写作规范

借鉴 `references/ai-first-rules.md`，为 `HXLoLi/ai-docs/` 建立一份 `_AI_DOCS_RULES.md` 或 `ai-docs/000-规范/index.md`。

最低要求:

- 每篇 AI 沉淀笔记有 `## 给未来 AI` 前言。
- 有 frontmatter: `date`、`type`、`tags`、`ai-first: true`、`source`。
- 外部事实带日期和 URL。
- 推断标注置信度。
- 不确定项写 `TBD`，不编造。

### 2. 保存对话时自动传播

现在 `ai-docs/` 可以存沉淀，但缺少“写一处，更新多处”的传播规则。

可以引入:

- 新项目知识 -> 更新项目页、相关技术页、索引页。
- 新 C++ 结论 -> 更新对应专题页和相关博客草稿。
- 新工具/依赖经验 -> 更新 DevOps/项目文档。
- 新问题复盘 -> 更新“常见坑”或“调试日志”。

这会让知识库从“文件堆”变成会累积的系统。

### 3. 建立 `raw/` 或 `sources/` 原始资料区

`obsidian-second-brain` 的 `raw/` 不可变规则很值得学。

对 `HXLoLi` 可以是:

```text
ai-docs/_sources/
├── articles/
├── transcripts/
├── issue-dumps/
├── compiler-bugs/
└── code-snippets/
```

规则: 原始资料只新增，不改写。整理后的中文知识页从这里派生。

### 4. 为可重复生成内容加 sentinel

`HXLoLi` 已有自动生成侧边栏、图谱、统计数据。后续如果让 AI 生成架构文档或项目说明，应采用:

```markdown
<!-- @generated:start -->
自动生成内容
<!-- @generated:end -->

<!-- @user:start -->
人工补充
<!-- @user:end -->
```

刷新时只替换 generated block，避免 AI 或脚本覆盖人工内容。

### 5. 做 `hx-health` 健康检查脚本

可以仿 `/obsidian-health`，但适配 Docusaurus:

- 检查 `docs/`、`blog/`、`ai-docs/` 内部链接。
- 检查图片引用是否存在。
- 检查 frontmatter 缺失或字段异常。
- 检查重复标题。
- 检查孤立文档。
- 检查 `ai-docs/` 是否缺少 AI-first 前言。
- 检查博客草稿/私有文章是否被错误放进公开 feed。

### 6. 做 `hx-architect`

`/obsidian-architect` 对 `HXLoLi` 很合适。它可以定期扫描项目并生成:

- Docusaurus 架构总览。
- docs/blog/ai-docs 三套内容系统说明。
- 插件和脚本职责说明。
- 私有文章加密流程。
- 资源/CDN/音乐/番剧模块关系图。
- 关键设计决策记录。

生成位置建议:

```text
HXLoLi/ai-docs/002-知识沉淀/002-项目学习/002-HXLoLi架构/
```

### 7. 建立检索评估

`obsidian-retrieval-eval` 的核心思想值得直接搬: 搜索质量要用问题集评估。

对 `HXLoLi` 可以生成这样的 eval:

- 问题: “MSVC requires 缺陷那篇在哪里?”
- Gold: 对应博客或 docs 页面。
- 测量: 本地搜索、图谱搜索、AI 检索能否在 top 1/3/5 找到。

这能指导是否需要改搜索权重、标题、标签、摘要，而不是盲目加向量库。

### 8. 区分人类笔记和 AI 知识库

你已经把 `docs/` 和 `ai-docs/` 分开，这是好设计。

建议继续保持:

- `docs/`: 人类阅读优先，可发布、可长期维护。
- `blog/`: 时间线输出，有个人表达。
- `ai-docs/`: AI 生成/半生成，结构化、可检索、可被未来 AI 使用。

不要把 AI 生成的临时沉淀直接塞进正式 `docs/`。先在 `ai-docs/` 经过 review，再毕业到 `docs/` 或 `blog/`。

## 不建议直接照搬的部分

- 不必把主系统迁移到 Obsidian。`HXLoLi` 是 Docusaurus，静态站点和 Git 工作流已经很完整。
- 不必一次性复制 44 个命令。先做 3 到 5 个最贴合的命令即可。
- 不必强行使用英文 `For future Claude`。可以在 `HXLoLi` 中使用 `## 给未来 AI`，但字段名和脚本接口应稳定。
- 不必引入后台 agent 自动改仓库。博客内容更公开，自动写入应默认生成草稿或 PR，不直接推送。

## 最小落地路线

1. 给 `ai-docs/` 增加 AI-first 中文写作规范。
2. 写 `scripts/health-ai-docs.mjs`，先检查 AI-first 前言、frontmatter、断链。
3. 写 `scripts/architect-scan.mjs` 或复用 Python，生成 `HXLoLi` 架构 JSON。
4. 用 sentinel 生成架构笔记，人工补充不被覆盖。
5. 把保存对话的流程规范化: 先进入 `ai-docs/`，review 后再升级到 `docs/` 或 `blog/`。

## 一句话结论

`obsidian-second-brain` 最值得 `HXLoLi` 学的是“让知识写入可审计、可传播、可刷新、可被未来 AI 检索”，而不是学习它的 Obsidian 目录本身。
