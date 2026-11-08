# Agent Note: susume 推荐页改成番剧库同款 (限定在一个分类里, 默认按番名 A-Z), 画面上不放名次和说明文案

Status: implemented

Decision-ID: anime-susume-wall

## Code

- `src/components/anime/susume/SusumeCategoryPage.tsx`
- `src/pages/anime/susume/index.tsx`
- `src/pages/anime/details.tsx`
- `src/components/anime/AnimeCard/SeriesCard.tsx`

## Problem

站长 2026-11-08: susume 页 (总览 `/anime/susume` 与四个分类页) 要 "平铺, 按番剧 A-Z 排序, 而不是列表, 这里主要就是为了展示的爽". 上一版分类页是第 1 名特写 + 第 2 / 3 名横卡 + 第 4 名起海报格, 仍是排行榜, 还带观后感、话数、状态等文字. 另外 "别在画面上显示历史的东西, 比如这里之前是妹控这种文本, 完全影响沉浸感, 特别 ai 味道": シスコン 的旧名「妹控」经 `subtitle` 出现在子导航和详情页徽章的悬停提示里, 页面上还有「按我的评分排, 同分看 Bangumi」「还没写观后感.」「三个分类合在一起」以及空状态里教人怎么上榜的一段话. 之后两版 (按首字母分段 + 吸顶 A-Z 条; 复用首页封面格的一面墙) 站长都不满意: 不要字母标题 ("我要连贯的"), 并且要 "对标 anime 主页那种, 交互, ui 动画, 特效".

## Decision

1. 分类页 (`SusumeCategoryPage.tsx`) 就是番剧库 `/anime` (`src/pages/anime/index.tsx`) 限定在这一类的记录里: 同一套页头 (标题 + 一行等宽副标) 与下划线子导航 (全部 | 四个推荐分类 | キャラ | 关系图 →, 当前分类 `aria-current`), 同一条 `FilterBar` (搜索 / 状态 / 排序 / 单部·系列 / 海报·列表 / 筛选面板 / 生效条件), 同一张 `PosterCard` / `SeriesCard`, 同一套骨架 / 空态 / 错误态, 样式直接用 `AnimeIndexPage.module.css`. 筛选切换走 `utils/anime/listMotion.ts` 的 `runListTransition` (View Transitions), 后退 / 前进与不支持的浏览器用 `useListFlip`; 视图密度与番剧库共用本地键 `/anime/:view`. 不放分享按钮.
2. 与番剧库唯一的不同是默认排序「名称」: 地址栏没有 `sort` 时按名称 (`parseSusumeFilters`), 选回名称时去掉 `sort`, 选「最近更新」时显式写 `sort=update` (`serializeSusumeFilters`). 名称排序时 `groupRecords` 只给一组, 一面墙连着排到底, 没有字母标题和跳转条.
3. 墙用 `StagePaper` (墙宽 1200 + 两侧立绘, 与番剧库 / 系列页同一套 DOM), 不用 BlogWithCats 默认导出, 原因见 `.agents/notes/implemented/feature/2026-11-06-anime-susume-neko-redesign.md`.
4. 总览 (`SusumeHubPage.tsx`): 番剧库同款页头与子导航, 每个分类一节 (番剧库的分组标题 + 数量), 节内是按番名 A-Z 的前 12 张海报密度 `PosterCard`, 超过 12 部时节尾「全部 N 部 →」. 卡片上的普通标签点了去 `/anime?tag=` 筛选. 「评分最高」整节删除.
5. 画面上的说明 / 旧名一律不显示: 删掉 `SusumeEntry` 链接的 `title={def.subtitle}`、详情页 (`details.tsx`) 徽章的「妹控: 进入「シスコン」专属页」、`SeriesCard` / `PosterCard` 徽章的「推荐分类「…」: 去专属页」; 页面 meta description 是「HXLoLi 的推荐: 分类名」. `data/animeTags.ts` 的 `subtitle` / `rule` 字段保留 (是 labelVocab.json 的镜像), 只是不再渲染.
6. `useSusume.ts` 只剩每类的记录 (按番名) 与三态; 删掉 `SUSUME_COPY`、`SusumePick`、首字母分组、`backdrop`. `susume.module.css` 不再被引用.

## Alternatives considered

- 什么都不做: 最强理由是上一版刚按站长意见把封面放大、立绘对齐, 版面稳定. 否决: 站长明确要平铺 A-Z, 排行榜和观后感正是不要的东西.
- 复用现有海报格, 只去掉前三档: 改动最小. 否决: 海报格带名次和两行观后感, 排序仍按评分, 不是 A-Z 平铺.
- 按评分平铺 (保留排序, 只去掉特写): 少写排序代码. 否决: 站长点名要 A-Z.
- 按首字母分段 + 吸顶 A-Z 跳转条 (11-08 13:08 做过一版): 番多时好翻. 否决: 站长要一面连贯的墙.
- 复用首页「在追的番」的封面格 (11-08 13:50 做过一版): 改动小. 否决: 站长要番剧库那样的交互和动效, 封面格只有悬停描边.
- 把番剧库页面抽成带 scope 参数的组件, susume 只传分类: 没有重复代码. 否决 (暂缓): 要改 `src/pages/anime/index.tsx` 和它挂着的几篇 note, 分享、旧链接跳转等逻辑 susume 用不上; 现在是照着番剧库写一份, 两边改交互时要一起改.

## Consequences

susume 分类页与番剧库是两份几乎一样的页面代码, 改番剧库的列表交互 (FilterBar 用法、骨架、空态、过渡) 时要同步 `SusumeCategoryPage.tsx`. `AnimeIndexPage.module.css` 改样式会同时影响两边, 这是有意的. 以后想在画面上加说明或旧名提示, 先回看本 note 第 5 条.
