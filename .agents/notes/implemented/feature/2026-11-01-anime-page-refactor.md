# Agent Note: 番剧库重构 (URL 化组合筛选 + 标签分组复核 + 海报 / 列表视图 + 加载三态)

Status: implemented

Decision-ID: anime-page-refactor

## Code

- `src/utils/anime/filters.ts`
- `src/components/anime/Tag/FilterBar.tsx`
- `src/components/anime/AnimeCard/PosterCard.tsx`
- `src/pages/anime/index.tsx`

## Problem

番剧页 (`/anime`) 的筛选是三套互不相干的状态: 观看状态单选页签 (默认"在看", 存 localStorage)、标签单选 (唯一写在 URL 上的 `?tag=`)、排序页签 (存 localStorage). 不能多选标签, 不能组合多个状态, 分享链接只带标签, 别人打开看到的是自己本地存的状态; 看不出当前生效了哪些条件, 也没有清空. 带 tag 进来时靠 effect 强制把状态切成"全部", 是补丁. 160 部里在看只有 13 部, 默认进来像是库里就这么点.

展示是一条长时间线, 每部一张带整段简介的宽卡, 两侧各 200px 猫娘压缩主区, 一屏只看到 2 到 3 部. 季度分组把 1 到 3 月叫"春"、4 到 6 月"夏"、7 到 9 月"秋"、10 到 12 月"冬", 而日本动画按开播月份归季是 1 月冬、4 月春、7 月夏、10 月秋, 所以每个分组标题都错一季 (不是 quarter 从 0 起算, 季度下标本身没错, 是下标对应的季名整体后移了一位); 另外它用 `new Date("YYYY-MM-DD")` 按 UTC 解析, 在西半球时区月初的番会退回上一天、落到上一季. 加载中显示一张假的占位卡, CDN 失败时永远停在加载. 详情页冷启动有两处闪 404: 静态生成的 HTML 没有查询串 (id 为空) 直接渲染 404; 浏览器端 12 MB 的 JSON 到达前 `records` 为空, find 不到也渲染 404.

标签方面, 8 个分类混着三种维度 (萝莉是角色属性, 异世界是舞台, 其余是基调), 随手标签 (百合 / 舰娘 / 妹控 / 纯爱) 平铺在后面, "纯爱"与"恋爱"重叠; 缺一个覆盖面很大的舞台类"校园". 萝莉 34 部按口径复核有误标与漏标.

## Decision

筛选状态收成一个对象, 完整写进 URL (`src/utils/anime/filters.ts`): `?q=&status=watching,watched&tag=萝莉,日常&match=any&year=2024&score=8&sort=air`, 默认值不写, 所以 `/anime` 就是默认视图 (全部状态、按最近更新). 兼容旧链接 `?tag=萝莉`. 组合语义: 维度之间"且", 观看状态内"或", 标签默认"全部命中"(且), 选两个及以上时可切"任一命中"(或); UI 上用一句话写明 ("同时带有「萝莉」和「日常」的番"), 生效条件行里标签之间也写出"且 / 或". 点选类改动 push 一条历史 (后退撤回上一步筛选), 搜索输入防抖后 replace. 只有视图密度 (海报 / 列表) 存本地.

工具栏 (`src/components/anime/Tag/FilterBar.tsx`) 一行放搜索 (按 "/" 聚焦)、观看状态多选 chips、"标签与更多"按钮、排序下拉、视图切换; 标签 / 首播年份 / 评分下限收进原生 `<dialog>` 面板, 桌面居中, 窄屏贴底成抽屉. 每个选项旁是分面计数 (其他条件不变时选上它会剩几部), 为 0 的置灰. 下方一行列出全部生效条件, 每个可单独去掉, 外加"清空全部"和"N / 160 部". 工具栏 sticky 在导航栏下.

列表 (`src/components/anime/AnimeCard/PosterCard.tsx`) 改成同一份数据两种密度: 海报 (2:3 封面, 状态角标、评分、进度条, 悬停或聚焦浮出短评, 桌面 5 到 6 列、手机 2 列) 与列表行 (小封面 + 名称 / 原名 / 年份 / 进度 / 短评 / 全部标签). 卡上的标签是独立按钮, 点了加进或移出当前筛选. 首播日期排序按番剧季度分组 ("2026 秋 · 10 月番"), 最近更新排序按年月分组, 评分 / 名称排序不分组. 去掉两侧猫娘与深灰圆角盒. `src/pages/anime/index.tsx` 加载中出骨架, 失败出错误与重试 (`animeStore` 新增 `useAnimeRecordsState`, 失败后清掉在途 promise 让重试真的重新请求), 筛空时逐个试去掉一个条件, 给出"去掉 X, 有 N 部"的一键撤销.

详情页 (`src/pages/anime/details.tsx`, 与番剧库同目录) SSR 与水合首帧 (`useIsBrowser` 为 false) 只出骨架, 浏览器端按加载状态区分骨架 / 错误 / 404; 去掉猫娘, 头部用封面虚化做背景, 加"← 番剧库".

标签 (`data/animeTags.ts`): `ANIME_TAG_DEFS` 加 `group` (角色 / 舞台 / 基调, 表外随手标签归"其他"), 百合 / 妹控 / 舰娘作为 `userOnly` 定义归入"角色"; `ANIME_TAG_ALIASES` 把"纯爱"并入"恋爱"; 新增"校园" (28 部). 萝莉复核: 请问您今天要来点兔子吗 两季去掉 (智乃是初中生, 小学生三人组是配角); 学生会的一己之见 两季 (会长樱野胡桃, 合法萝莉、核心、身高是卖点) 与 NEKOPARA OVA Extra (讲仔猫时期, 幼女外形就是这部的看点) 补上; 口径补一句"初中生不算". 萝莉 34 -> 35. `getRecordTags` 按记录对象缓存 (筛选时每次输入都要对 160 部算一遍).

## Alternatives considered

- 什么都不做: 最强理由是现有列表页能看能筛, 不动就没有回归风险, 也不必新增 data/animeTags.ts 与 filters.ts 两个模块. 否决: 用户点名要重构交互与展示, 且季度标签错位 (1 月被算成春) 与详情页 404 闪烁是两个真实缺陷, 不修就是留着 bug.
- 复用现有 WatchStatusTabs + TagFilter 再补几个控件: 最强理由是组件已存在, 改动面最小. 否决: 旧实现把状态与排序放 localStorage, 链接分享不出去, 且从首页带标签进来时要靠 effect 补丁对齐, 越加维度越乱.
- 保留两排页签, 只把标签改成多选: 改动最小. 否决是因为状态与排序仍在 localStorage, 分享链接还原不了; 两排页签加一排标签已经占掉首屏三行, 再加维度只会更长.
- 筛选状态全放 localStorage: 刷新能记住. 否决是因为不能分享, 而且从首页 / 详情页带标签进来时要和本地状态打架 (就是旧实现那个 effect 补丁).
- 标签多选默认"任一命中": 结果更多. 否决是因为这个库的主要用法是收窄 ("萝莉 且 日常" 13 部), 任一命中选这两个就有 52 部, 三分之一个库, 多选失去意义; 保留为可切换项.
- 引入 headless UI 库 (Radix / Headless UI) 做弹层与 combobox: 可访问性现成. 否决是因为要加依赖 (约 10 到 30 KB gzip) 且要先批; 原生 `<dialog>` 已给出焦点陷阱、Esc 关闭、`::backdrop`, 够用.
- 虚拟列表 (react-window): 只渲染可见行. 否决是因为 160 部不需要, 且海报网格列数随宽度变, 虚拟化要自己算行高; 用 CSS `content-visibility: auto` 跳过屏外卡片的布局与绘制即可.
- 按 `.agents/research/anime-page-redesign.md` (同日另一份待确认方案) 接 HX-UI 令牌作用域: 颜色单一来源. 本轮没做, 因为要新增全局 CSS 并改 `docusaurus.config.ts`, 超出本轮范围; 本轮样式只用 Infima 变量与半透明白, 以后接令牌时只换 CSS module, 不动结构.

## Consequences

番剧库的分享链接能还原全部筛选; 旧的 `?tag=X` 链接 (首页番剧块、详情页) 继续有效. 旧 localStorage 键 `/anime/:watchStatus` 与 `/anime/:sortMode` 不再读, 留在用户浏览器里无害. `src/components/anime/AnimeCard/index.tsx` (旧时间线卡)、`src/components/anime/SortTabs/index.tsx`、`src/components/anime/WatchStatusTabs/index.tsx` 不再被番剧库引用, 本轮未删 (SortMode 枚举仍被旧卡引用, 删除留给确认后单独一轮). `src/components/anime/Tag/TagFilter.tsx` 改成多选面板, 被 FilterBar 引用; `src/components/anime/AnimeCard/PosterCard.module.css`、`src/components/anime/Tag/FilterBar.module.css`、`src/pages/anime/AnimeIndexPage.module.css`、`src/pages/anime/AnimeDetailPage.module.css` 是非 AST 资源, 需人工核对. `src/utils/anime/animeStore.ts` 的 `useAnimeRecords` 保持旧签名, 首页番剧块不用改. `src/utils/anime/tags.ts` 的 `TagCount` 多了 `group` 字段, `countTags` 不再列出计数为 0 的随手标签定义. 详情页路由仍是 `/anime/details?id=<id>`, 不是 `/anime/details/<id>` (后者需要动态路由插件). 萝莉判定仍是人工口径; 有争议、本轮没动的: 工作细胞 (血小板是配角)、无职转生 (洛琪希)、盾之勇者成名录 (菲洛)、关于我转生变成史莱姆这档事 (米莉姆)、魔法禁书目录 (最后之作 / 小萌是配角)、干物妹！小埋 与 碧蓝航线：微速前行！ (Q 版, 口径排除)、章鱼哔的原罪 (主角是小学生但作品是致郁向, 不是卖点).
