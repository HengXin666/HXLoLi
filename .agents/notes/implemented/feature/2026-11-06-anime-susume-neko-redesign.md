# Agent Note: 番剧专题页骨架 Stage (kyara 用全套, susume 只用墙) + 两侧立绘墙 StagePaper

Status: implemented

Decision-ID: anime-susume-neko-redesign

## Code

- `src/components/anime/stage/Stage.tsx`

## Problem

susume 推荐页和 kyara 角色配图页需要一套共用的页面骨架 (页头 / 节标题 / 统计 / 正文墙). 最早两版 (推荐板 + 探头猫娘, 之后杂志特辑式的封面墙 Hero + 逐字弹入 + 舞台猫娘) 被站长评为 "ai 味太多": 点阵、光晕、马克笔大标题、皇冠、舞台里的猫娘, 以及两侧立绘与正文之间没有边界. 之后又发现 susume 页两侧立绘的比例和系列页对不上.

## Decision

1. `src/components/anime/stage/` (`Stage.tsx` + `stage.module.css`) 是 kyara 的页面骨架 (susume 现在只用其中的 `StagePaper`, 见第 6 条), 用首页 (`src/components/home/page.module.css`) 的语汇: 1px 细线、20px 节标题 + 右侧灰字 (`StageHead`)、28px 标题、1px 分格统计、等宽数字、150ms 悬停. 不要封面墙、视差、逐字弹入、数字滚动、舞台猫娘; `backdrop` / `say` 字段保留但不渲染, 免得调用方报错.
2. 页头 `StageHero` 只有面包屑、标题、右侧操作、统计; 小标 (`kicker`)、罗马音、说明句都是可选, susume 一律不传 (站长 2026-11-06 21:10: 小标、罗马音、说明句 "全是 ai 味, 毫无意义").
3. 正文墙 `StagePaper`: 直接写出 BlogWithCats 的同款 DOM (`.blog-container > SideCats > .blog-content-wrapper > .blog-content > main`), 墙宽 1200, 与系列页 / 番剧详情 / 番剧库同一套 pair 立绘 (最大 380, 身体藏进墙后). 只用具名导出 `SideCats`; 用 BlogWithCats 默认导出包一层时渲染拿到 undefined, 总览页崩溃, 所以不用.
4. 原因 (StagePaper 一节): susume 原来只挂 `paperClass` (纸宽 1180), 立绘走 `RouteCats` 的表外兜底 gutter, 墙选择器 `main .container` / `main.container` 量不到这张纸, 只能按 1140 / 1320 推算, 立绘离纸要么隔一条缝要么压边, 宽度还被 gutter 的 120-260 卡住. kyara 两页同病, 也换成 StagePaper; `paperClass` 现在没有调用方, 保留导出.
5. 颜色只取 `--hx-*` 与 Infima 令牌, 分类色经 `--stage-accent` 注入; `prefers-reduced-motion` 时 `Rise` 直接出现.
6. 2026-11-08 起 susume 页面本体照搬番剧库 (页头、子导航、FilterBar、PosterCard 都用番剧库的), 只沿用这里的 `StagePaper` 当墙, 不再用 `StageHero` / `StageHead`; 见 `.agents/notes/implemented/feature/2026-11-08-anime-susume-wall.md`.

## Alternatives considered

- 什么都不做: 最强理由是 ui 原子组件 (`src/components/anime/ui`) 已经能拼出页面, 不必再造一层. 否决: 那套组件是 "标题 + pill + 卡片网格" 的后台式拼装, susume 和 kyara 要同一套页头和墙, 只能抽出来.
- 复用现有 BlogWithCats 默认导出包正文: 改动最小. 否决: 运行时拿到 undefined 导致总览页崩溃, 改为展开同款 DOM.
- 保留封面墙 Hero, 只删猫娘和点阵: 改动小. 否决: 站长点名的是整体 "ai 味", 封面墙和光晕正是来源.
- 继续用 paperClass + 路由兜底立绘: 不动结构. 否决: 墙选择器量不到纸, 两侧比例永远对不上系列页.

## Consequences

`src/components/anime/kyara/` 的页头、节标题和墙都用 Stage, 本地模式面板、导入弹窗、灯箱仍用 ui 原子组件, 见 `2026-11-06-anime-kyara-pages.md`. `src/css/custom.css` 是非 AST 资源, 走 resource-review. 改 StagePaper 的 DOM 时要同时核对系列页, 两边立绘必须一致.
