# Agent Note: 番剧库「系列」视图 (同一番剧各季聚成一张卡 + 系列过渡页)

Status: implemented

Decision-ID: anime-series-fold

## Code

- `src/utils/anime/series.ts`
- `src/components/anime/AnimeCard/SeriesCard.tsx`
- `src/components/anime/Tag/FilterBar.tsx`
- `src/components/anime/animeShareLink.ts`
- `src/pages/anime/series.tsx`

## Problem

番剧库 `/anime` 一张卡 = 一个 bgm 条目 (一季). 有续作的番 (例: 间谍过家家 两季 + 剧场版) 在库里散成多张卡, 排序 / 分组后各季还会分到不同的季度组里, 想看 "这部番整体追到哪" 只能一张张点. 用户要求参考 NAS 媒体库刮削后的 "剧集 -> 季 -> 集" 展示: 主页把同一番剧的各季聚成一个展示, 点进去是按顺序列出所有季的过渡页, 再进单季详情. 设计稿: `HXLoLis/hx-anime-series-fold-design.md`.

## Decision

1. 命名: 「系列」视图, URL `?fold=series` (默认 `none` 不写), 工具栏 `单部 | 系列` 两段切换 (`FilterBar.tsx`). 与 `sort` 正交: 聚合决定卡片单位, 排序决定卡片顺序; 不是筛选条件, `clearConditions` 保留它; 番单标识 `listKeyOf` 同排序一样剔除它 (`animeShareLink.ts`).
2. 系列判定 (`src/utils/anime/series.ts`): 直接 import 关系图的 `buildSeriesIndex` / `collectSeries` (`src/components/anime/AnimeGraphPage/buildGraph.ts`, 纯数据无 React), 一个连通分量 = 一个系列, 单部也是长度 1 的系列. 分量内: 经 前传 / 续集 边与起点连通的为正篇, 按首播日期排第 1..N 季 (不信标题里的 "第二季"); 只经 番外篇 / 总集篇 / 主线故事 挂上的为特别篇; 只靠名字兜底连进来、没有库内 relations 的条目正片集数 >= 6 算正篇. 系列 key = 第 1 季 id. 系列名 = 第 1 季中文名剥季度后缀. relations 指向但不在库里的 前传 / 续集 (动画) 记为 `missing`. 结果按记录数组引用记忆化 (`getSeriesList`), 库 / 过渡页 / 详情页共用.
3. 番剧库 (`src/pages/anime/index.tsx`): 筛选仍按单季 (`applyFilters` 不变), 命中季按系列聚合 (`seriesHits`), `groupSeries` 用与 `groupRecords` 相同的分组规则, 取值换成系列 (更新 = 系列内最新, 首播 = 第 1 季, 评分 = 最高分, 名称 = 系列名). 卡片 `SeriesCard.tsx`: 单季系列直接渲染 `PosterCard`; 多季: 叠卡封面 (在看那季否则第 1 季)、聚合状态角标、`N 季 + M 特别篇`、正篇合计进度、评分区间、年份区间、标签并集, 只命中部分季时标 "命中 x/N" 并把命中季经 `&hit=` 带进过渡页; 列表密度多一行每季小进度 chip. `data-anime-id` 填系列 key 接入现有过渡动画. 结果计数显示 "N 个系列 · M / 总 部".
4. 过渡页 `src/pages/anime/series.tsx` (`/anime/series?id=`): 任一季 id 进来 replace 成系列 key, 单季系列 replace 到详情. Hero (沿用详情页样式) + 继续观看 (第一个在看 / 未看完的正篇季, 全看完为 "重温第 1 季") + 关系图 / Bangumi; 简介 (第 1 季, 2026-11-06 起挪到时间线之前, 见下文「简介卡宽度修正」); 正篇季卡网格 (未收录的前传放最前、续集放最后, 灰显去 bgm); 特别篇; 首播时间线. 三态同详情页. 不放讨论区.
5. 详情页 `src/pages/anime/details.tsx`: 返回链接下加 `SeriesNav` (系列名 · 第 N 季 / 共 M 季, 上一季 / 下一季), 单季不渲染.

## 简介卡宽度修正 (2026-11-06)

问题: 过渡页的简介借用详情页 `AnimeDetailPage.module.css` 的 `.summary` / `.summaryFolded` (`max-width: 72ch`), 外层只有一个无样式 `div`. 在通栏的 `SeriesPage.module.css .block` 里它成了一条比时间线 / 观看顺序都窄的左对齐文字列, 右侧大片留白, 与上下区块对不齐, 而且排在页面最底部. `.summary` 与 `details.tsx` 共用, 改它会连带详情页.

决定 (只动 `src/pages/anime/series.tsx` 的 `Summary` 与简介 section, `SeriesPage.module.css` 只追加 `.summary*` 类):

1. 简介挪到 Hero 之后、放送时间线之前 (`id="summary"` 在 `id="timeline"` 之前).
2. 简介是一张撑满 `.block` 宽度的卡 (`.summaryCard`, 外宽 = 时间线 `.tlWrap` 外宽), 只有文字内层 `.summaryText` 限 `max-width: var(--hx-measure, 68ch)`; 左侧主题色竖条 + `--hx-accent-soft` 角落光晕, 顶部小标 `あらすじ · 第 1 季名`.
3. 视口 >= 1280 时卡片右侧一栏「系列速览」(年份跨度 / 正片话数 / 正片部数 / 作品线数, 直接用页面已算好的 `years` / `agg` / `lines`); 更窄时折到正文下面, <= 640 两列.
4. 折叠: 按真实行高量 6 行 (`ResizeObserver`), 超出才折; 折叠态 `mask-image` 底部渐隐, framer `motion` 高度动画展开 / 收起, 按钮文案用 `AnimatePresence` 切换; 切换后卡片顶部不在视口上部时滚回简介顶部. SSR / 首帧按字数 (`SUMMARY_FOLD`) 先猜, 并用 `calc(1.8em * 6)` 兜底高度.
5. 空简介显示弱样式「暂无简介」, 不出折叠按钮. `prefers-reduced-motion` 下入场 / 高度 / 文案切换全部瞬时, 滚动用 `auto`.
6. `AnimeDetailPage.module.css` 与 `details.tsx` 零改动; `series.tsx` 不再引用 `detail.summary*` / `detail.linkButton`.
7. 窄屏横向溢出 (2026-11-06 补): 390 宽下放送时间线把整页撑到 1200px. 页面外壳加 `.shellFit` (min-width 0 + overflow-x clip, 只在系列页), 横向轨道只在 `.tlTrack` 内滚动; <=640 叠封面右侧留 16px, 不再压住标题小字.

被否决: 直接改共用的 `.summary` 去掉 `72ch` (最省事, 但详情页正文会变成超长行, 且 `details.tsx` 不在本次范围); 维持现状只把 section 挪到前面 (最小改动, 但宽度不对齐的根因还在).

## Alternatives considered

- 什么都不做 (继续靠关系图看续作): 最强理由是关系图聚焦层已经能列出一个系列的全部条目. 否决: 关系图是力导向图, 不表达季的顺序与观看进度, 也不是番剧库的浏览入口.
- 复用现有排序, 把 "系列" 做成 `sort=series` 的第五种排序: 改动最少, 下拉里多一项. 否决: 聚合后仍需要按更新 / 首播 / 评分 / 名称排系列, 做成排序就只能固定一种排法.
- 把 `buildSeriesIndex` 搬到 `src/utils/anime/` 再让关系图反向 import: 依赖方向更干净. 否决 (本轮): 要连带改关系图目录与其 note 的代码端, 本轮只读引用, 行为零改动; 以后关系图再动时顺手搬.
- 在 `HXLoLi-ANiMe` 爬虫侧预先产出 `series.json`: 前端零计算. 否决: 160 部规模前端计算可忽略, 且前端能与本地 overlay 一起热更新; 规模到几千部再迁.
- 筛选在系列层重新定义 (系列整体满足 "评分 ≥ 8" 等): 否决: 状态 / 年份 / 评分天然是季的属性, 系列层重定义会让分面计数与 "去掉哪个条件" 提示失真.

## Consequences

`src/utils/anime/series.ts` 依赖 `AnimeGraphPage/buildGraph.ts` 的两个纯函数, 关系图那边改系列判定会同时影响系列视图 (期望行为: 同一概念一处定义). 同目录兄弟 `src/utils/anime/filters.ts` (加 `fold`), `src/pages/anime/index.tsx` / `details.tsx` / `SeriesPage.module.css` / `AnimeDetailPage.module.css`, `src/components/anime/AnimeCard/SeriesCard.module.css` 是本决策的代码端, 分别由上面同目录的代表文件代表. 自动判定会有错例 (分割放送显示成第 2 季、名字兜底误连), 人工 merge / split / 系列名覆盖 (`data/animeSeries.json`) 留作第二期. 分享番单仍按单部.
