# 产物约定与交付闸门

## 一次沉淀的产物落在哪

```
ai-docs/.hx-staging/<slug>/          过程产物, 永久留在暂存区, 站点看不到
├── flow.json                        状态机
├── source/material.md               素材要点 (带定位)
├── source/provenance.md             来源、获取方式、可信度
├── review/voice-audit.md            AI 味盲审报告
└── review/fidelity-audit.md         保真盲审报告

ai-docs/<NNN-一级>/<NNN-二级>/<NNN-标题>/     笔记产物
├── index.md                         ✅ 面向人类, 由 makeDoc.py 初始化
├── .hx-info.md                      ✅ 面向知识库, hxid 与 index.md 一致
├── .hx-mitemite.md                  视情况, 待人类拍板的问题
├── tag.json                         可选, 侧边栏标签与图标
├── *.drawio.svg / *.png             可选, 配图, 必须同目录
├── *.html (archify)                 可选, 机制图侧车
└── *-deck.tsx / *-ppt.html          可选, 演示页侧车
```

**过程产物一律不进笔记目录。** 盲审报告、素材转写、provenance 都是过程 —— 读者看不到,
也不该出现在笔记目录里增加噪声。唯一的例外是 `.hx-mitemite.md`: 它有本地开发 UI 支持
(见下), 所以跟着笔记走。

暂存区是点开头目录, `generateAiDocsSidebar.js` / `quality-gate.mjs` 用
`entry.startsWith('.')` 跳过, `tag-index-plugin.mjs` 跳过 `.` 与 `_` 开头 —— 所以
它对站点完全透明。

## 闸门

阶段 9 跑:

```bash
node scripts/generateAiDocsSidebar.js
uv run .agents/skills/hx-note/scripts/hx_flow.py doctor --slug "<slug>"
```

`doctor` 逐项检查:

| 检查 | 不通过怎么办 |
|---|---|
| `index.md` / `.hx-info.md` 存在 | 回对应阶段 |
| frontmatter 七字段齐备 (`hxid` `title` `created_at` `model` `skill` `authors` `tags`) | 补; `skill` 要如实记录本次用到的技能 |
| 正文无 `TODO` 残留 | 删模板占位 |
| 正文无过程痕迹 (本地路径 / `uv run` / `.agents/` / `.hx-staging`) | 移出正文 |
| `index.md` AI 味检查 (`hx_voice.py lint`) | 按命中逐条改 |
| `.hx-info.md` 原子度检查 | 同上, profile 是 `atom` |
| 标点规范 (`format_cn_punct.py --check`) | 先 `--diff` 看改动再原地归一化 |
| tag 是注册表规范名 (`hxloli_tags.py check`) | `suggest` 查复用, 或 `merge` 合并近义词 |
| 两份文件 `hxid` 一致 | 以 `index.md` 为准改 `.hx-info.md` |
| 已注册进 `sidebarsAiDocs.ts` | 跑 `node scripts/generateAiDocsSidebar.js` |
| 两份盲审报告存在 | 回阶段 7 / 8, 不许跳 |

`doctor` 有任何一项不通过时, **回复里必须写明**是哪一项、为什么跳过。
不允许悄悄交付一个 FAIL 的流程。

## 跨文章引用

一律写成 `[标题](hxid:hx-xxxxxxxx)`, 不要写死相对路径 —— 后续重分类时链接不会断。
同目录内引用仍用普通相对路径。

移动过目录之后:

```bash
uv run .agents/skills/hx-note/scripts/hx_docs_id.py resolve --write
uv run .agents/skills/hx-note/scripts/hx_docs_id.py check
```

## 知识库索引的现状 (已知缺口)

目前站点的 tag 索引 (`plugins/tag-index-plugin.mjs`) 读的仍然是 `index.md`, 因为它
跳过所有 `.` 开头文件。也就是说 **`.hx-info.md` 目前还没有被真正索引**。

要把索引切到 `.hx-info.md`, 需要在插件里加一处判断: 同目录存在 `.hx-info.md` 时优先
用它的正文做索引内容 (标题/permalink 仍取 `index.md`)。这是**博客构建层的改动**,
按 `AGENTS.md` 属于非平凡改动, 要人类批准 + 同提交写一篇 agent note。

**不要在沉淀流程里顺手改它。** 本流水线的职责是保证 `.hx-info.md` 是一份自包含、
可直接喂给任何索引方案的单文件; 接不接进去是另一件事。

## 本地开发联动

人类用 `run.sh` 启动站点时会同时起 `scripts/dev-edit-server.mjs` (localhost:3310, 仅本地),
它给 ai-docs 页面注入仅本地可见的 UI:

- **本地工具条**: 「在 VS Code 中打开」跳到该 `index.md` 的标题行。
- **选中正文右键**: 「跳转到 VS Code」定位到源码行列。
- **答题卡可折叠区** (`.hx-mitemite.md`): 默认折叠, 每问独立框体, 可点「编辑」原地改并写回本地。
  **这是审核事项的唯一落盘处**, 读者看不到。
- 组件探测 `localhost:3310/health` 才渲染, 生产构建为 null, 产物不含本地编辑 UI。

交互路径: 审核要点走答题卡; 要改源码用「选中右键 -> 跳转 VS Code」; 改完 Docusaurus HMR 即时重渲染。
