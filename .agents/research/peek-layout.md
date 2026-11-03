# 探头立绘 (NekoStage) 全站布局设计

> 2026-11-03 CST 起稿, 2026-11-04 CST 对齐到代码最终状态 (见 §8.8). 范围: HXLoLi 整站每一类路由. 代码: `src/components/BlogWithCats/index.tsx` / `BlogWithCats.css` / `src/theme/Root.tsx`.
> 几何规格以 `.agents/research/catgirl-spec.md` 为准 (5:12 竖图, 人物贴底占下 55%, 上 45% 透明, 裁切线 = 画布内侧边, fade 0.84-0.94 由数据给).
> 硬约束: 不碰 `static/img/`, 不碰 `src/components/anime/` 与 `src/pages/anime/details.tsx`.

## 0. 术语

| 词 | 含义 |
|---|---|
| 墙 | 立绘身体藏在其后的那条竖边. 可以是正文块 (实心、有底色) 的外沿, 也可以是视口边 |
| 锚点 | 立绘槽 (`.neko-slot`, fixed) 的横向定位基准: 墙的外沿 (JS 实测 px) 或视口 left/right 0 |
| 探出量 | 立绘画布越过墙、露在可见区域的那部分. pair 用"内侧侵入墙后的比例 --neko-in", 视口墙变体用"推出视口的比例" |
| 留白 room | 墙外沿到下一个障碍物 (视口边 / 侧边栏 / TOC) 的距离, 立绘只许放在留白里 |
| 障碍物 | 不能被立绘盖住的东西: 文档侧边栏、博客侧边栏、TOC、导航栏、返回顶部按钮、卡片墙 |

## 1. 盘点表 (改动前现状)

路由来源: 本机 `build/sitemap.xml` (1114 条) + `src/pages/` 逐个 tsx + Docusaurus 不进 sitemap 的固定页 (404, /anime/share, docs 标签页).
"现状" 一列是改动前的代码行为.

| # | 路由 (类) | 页数 | 来源 | 改动前立绘 | 改动前变体 | 打算 |
|---|---|---|---|---|---|---|
| 1 | `/` 首页 | 1 | src/pages/index.tsx -> HomePage | 有 | RouteCats edges | 保留 edges, 修返回顶部遮挡 |
| 2 | `/anime` 番剧库 | 1 | src/pages/anime/index.tsx | 有 | RouteCats screen | 保留 screen |
| 3 | `/anime/details` 番剧详情 | 1 | src/pages/anime/details.tsx (页内 BlogWithCats) | 有, **叠两组** | 页内 pair + RouteCats side | 去掉路由表 side, 只留页内 pair; 加挂载登记防叠 |
| 4 | `/anime/graph` 番剧关系图 | 1 | src/pages/anime/graph.tsx | 有 | RouteCats edges | 保留 edges |
| 5 | `/anime/cv` 声优页 | 1 | src/pages/anime/cv.tsx (BlogWithCats) | 有 | 页内 pair | 保留, 加障碍物感知 |
| 6 | `/anime/share` 分享番单 | 1 | src/pages/anime/share.tsx (SideCats 1200) | 有 | 页内 pair (无 .blog-content 墙, 推算) | 保留 |
| 7 | `/friends` 友链 | 1 | src/pages/friends/index.tsx (BlogWithCats) | 有 | 页内 pair | 保留 |
| 8 | `/blog` + `/blog/page/2..5` 博客列表 | 5 | src/theme/BlogListPage (BlogWithCats 880) | 有 | 页内 pair | 保留, 加障碍物感知 |
| 9 | `/blog/<date>/<slug>` 博客文章 | 43 | src/theme/BlogPostPage (BlogWithCats 在 BlogLayout 里) | 有, **压侧边栏与 TOC** | 页内 pair | pair + 障碍物感知: 左留白量到博客侧边栏右沿, 右留白量到 TOC 左沿 |
| 10 | `/blog/archive` 归档 | 1 | src/theme/BlogArchivePage (BlogWithCats) | 有 | 页内 pair | 保留 |
| 11 | `/blog/tags` 博客标签总览 | 1 | Docusaurus BlogTagsListPage + 本仓 BlogLayout | 无 | - | 新增 side |
| 12 | `/blog/tags/<tag>` 单标签 | 18 | BlogTagsPostsPage + BlogLayout | 无 | - | 新增 side |
| 13 | `/blog/authors` 作者 | 1 | BlogAuthorsListPage + BlogLayout | 无 | - | 新增 side |
| 14 | `/docs/**` 笔记文档 | 935 | docs/ (DocRoot + 本仓 DocItem/Layout) | 无 | - | 新增 gutter (右侧, 墙 = 文档容器右沿) |
| 15 | `/docs/tags` + `/docs/tags/<tag>` | 2+ (按标签数) | src/theme/DocTagsListPage / DocTagDocListPage | 无 | - | 新增 gutter (两侧, 墙 = 1120 内容列) |
| 16 | `/knowledge-base/**` 知识库文档 | 42 | ai-docs/ (同 DocItem/Layout, html.ai-kb-page) | 无 | - | 新增 gutter (同 #14) |
| 17 | `/knowledge-base/tags` + `/<tag>` | 56 | tag-index-plugin + DocTag* 主题 | 无 | - | 新增 gutter (同 #15) |
| 18 | `/docs-graph` 笔记关系图 | 1 | src/pages/docs-graph.tsx | 无 | - | 新增 edges (与 /anime/graph 一致) |
| 19 | `/search` 站内搜索 | 1 | @easyops-cn/docusaurus-search-local | 无 | - | 新增 gutter (两侧, 墙 = main .container) |
| 20 | `/404` 与任意未知路由 | 1 | Docusaurus NotFound | 无 | - | 新增 gutter (两侧, 墙 = main.container) |
| 21 | `/hxdeck` 演示组件自检页 | 1 | src/pages/hxdeck.tsx -> hxdeck/Demo | 无 | - | 明确不渲染 (off), 理由见 §2 |
| 22 | `/hxdeck-editor` 主题编辑器 | 1 | src/pages/hxdeck-editor.tsx | 无 | - | 明确不渲染 (off) |
| 23 | `/ppt` 演示独立播放页 | 1 | src/pages/ppt.tsx | 无 | - | 明确不渲染 (off) |
| 24 | `/drawio` draw.io 编辑器 | 1 | src/pages/drawio/index.tsx | 无 | - | 明确不渲染 (off) |

汇总 (按路由类): 24 类; 改动前有立绘 10 类 (其中 #3 叠两组, #9 压侧边栏 / TOC); 缺 14 类.
汇总 (按页数, sitemap 1114 条 + 404 + share): 改动前有立绘 56 页, 缺 1060 页 (935 docs 页是大头).
计划: 新增 10 类 (#11-#20) 渲染立绘, 4 类 (#21-#24) 写进路由表为 `off` 并在本文写明原因; 修 2 处已有破绽 (#3 叠组, #9 压障碍物).

## 2. 逐页设计表

断点记号: **D** = 桌面 >= 997px (Docusaurus 桌面布局, 侧边栏 / TOC 在场), **S** = 小屏 769-996px, **T** = 平板 481-768px, **N** = 窄屏 <= 480px. 手机端 pocket 的参数与判据见 §3.
"room" 一律是 JS 实测 (墙外沿到障碍物或视口边, 减 16px 间隙), 不再用 `50% ± 正文宽/2` 推算.

| 路由 | 变体 | 墙 | 锚点 | 探出量 | D | S 769-996 | 手机端 T / N (pocket) | 为什么不碍眼 |
|---|---|---|---|---|---|---|---|---|
| `/` | edges | 视口左右边 + 底边 | 槽贴 left:0 / right:0, bottom:0 | 静止下沉 30%, 滚一屏收到 12%; 宽 clamp(160, 12vw, 220) | >= 1100 渲染 | 不渲染 | pocket: 页脚进视口且实测不压才出 | 首页是全宽 hero + 卡片, 没有正文块可躲; 只在视口最下两角露半身, 返回顶部按钮 z 提到立绘之上 |
| `/anime` | screen | 视口左右边 | left:0 / right:0, 底边抬 --neko-lift | 推出视口 14% -> 滚满 4%; 宽 clamp(180, 留白/2+130, 270) | >= 1360 渲染 | 不渲染 | pocket: 页脚进视口且实测不压才出 | 卡片墙 1200 铺满无正文块; 1360 起两侧留白 >= 80, 只有手和发梢贴最外一列海报边 |
| `/anime/details` | **gutter** (路由表; 页内不挂 SideCats). 不是 pair, 也不是 off | `DETAIL_WALL` = `[class*=shell]` 外沿 (shell 是 CSS Module 哈希类名, 所以按子串匹配) | 槽贴墙左右外沿 | 0 侵入, 内侧 10% 水平淡出; 宽 clamp(120, room-16, 260) | room >= 136 渲染 | 不渲染 | pocket: 页脚进视口且实测不压才出 | 详情页是卡片式布局, shell 透明且无 z-index, pair 的"身体藏进墙后"不成立 (B21); 改为零侵入站在 shell 两侧留白里, 不碰卡片 |
| `/anime/graph` | edges | 视口边 | 同首页 | 同首页 | >= 1100 | 不渲染 | pocket: 页脚进视口且实测不压才出 | 画布交互在中部, 两下角是空白; pointer-events:none, 节点点击穿透 |
| `/anime/cv` | pair (页内) | .blog-content | 实测 | 6% | 同上 | 不渲染 | pocket: 页脚进视口且实测不压才出 | 同博客 |
| `/anime/share` | pair (页内, 无墙推算 1200) | 推算的 1200 内容列 | 视口中线 ± 600 | 6% | room 够才渲染 (约 >= 1500) | 不渲染 | pocket: 页脚进视口且实测不压才出 | 分享页正文本就 1200 宽, 留白不够宁可不出, 不压缩正文 |
| `/friends` | pair (页内) | .blog-content | 实测 | 6% | 同上 | 不渲染 | pocket: 页脚进视口且实测不压才出 | 友链卡片都在实心块内 |
| `/blog`, `/blog/page/N` | pair (页内 880) | .blog-content | 实测 | 6% | 同上 | 不渲染 | pocket: 页脚进视口且实测不压才出 | 列表单栏 880, 留白最大 |
| `/blog/<post>` | pair (页内 contentWidth 900, >= 1536 收成 min(900, 正文列 - 240)) + 障碍物 | .blog-content | 实测; 左留白截到博客侧边栏右沿, 右留白截到 TOC 左沿 | 完整 6% 侵入 / 紧凑 0 侵入; 双侧等宽按较窄一侧; 1920 约 157px (完整), 1536-1800 约 112px (紧凑) | **< 1536 不渲染; >= 1536 双侧** (门槛见 §2.2); 较窄侧放不下时退化为 room 大的单侧 | 不渲染 | 不出: 有博客侧栏 / TOC (用户定), reason pocket-page-off | 改动前左人压在"所有文章"侧边栏上、右人压 TOC; 现在立绘只住在侧边栏与正文、正文与 TOC 之间的缝里, 缝不够就不出 |
| `/blog/archive` | pair (页内) | .blog-content | 实测 | 6% | 同上 | 不渲染 | pocket: 页脚进视口且实测不压才出 | 同列表 |
| `/blog/tags`, `/blog/tags/*`, `/blog/authors` | side | 视口右边 | right:0, 底边抬 --neko-lift | 推出 8% -> 滚满 0%; 快速滚动缩回 78% | >= 1100 渲染 | 不渲染 | pocket: 页脚进视口且实测不压才出 | BlogLayout 右边 21.75% 列在这些页是空的 (无 TOC), 立绘正好住在空列里; 左边是博客侧边栏所以只出单侧 |
| `/docs/**`, `/knowledge-base/**` | gutter (新) 右侧 | 文档容器 `[class*=docMainContainer] > .container` 右沿 (TOC 外侧) | 槽 left = 墙右沿 (零侵入, 墙是透明的) | 0 侵入, 内侧 10% 水平淡出; 宽 clamp(120, room-16, 260) | room >= 136 渲染 (1920 下约 134 -> 刚好不出; 侧边栏收起或 >= 1940 才出) | 不渲染 | 不出: 有文档侧边栏 / TOC (用户定), reason pocket-page-off | 文档是阅读主场: 立绘只放在 TOC 再往右的空白里, 绝不进正文 / TOC / 侧边栏; 容器透明所以不侵入, 裁切线用淡出收口 |
| `/docs/tags*`, `/knowledge-base/tags*` | gutter 两侧 | 1120 内容列 `[class*=inner_]`, 找不到按 1120 居中推算 | 墙左右外沿 | 0 侵入; 宽 clamp(120, room-16, 260) | 1440 起两侧各约 144 | 不渲染 | pocket: 页脚进视口且实测不压才出 | 标签页无侧边栏, 两侧是背景网格, 立绘不碰标签卡 |
| `/docs-graph` | edges | 视口边 | 同首页 | 同首页 | >= 1100 | 不渲染 | pocket: 页脚进视口且实测不压才出 | 与 /anime/graph 同型画布页, 保持一致 |
| `/search` | gutter 两侧 | `main .container` | 墙外沿 | 0 侵入 | 约 >= 1440 (容器 1140/1320) | 不渲染 | pocket: 页脚进视口且实测不压才出 | 搜索结果列表居中, 两侧空白 |
| `/404` | gutter 两侧 | `main.container` | 墙外沿 | 0 侵入 | 同上 | 不渲染 | pocket: 页脚进视口且实测不压才出 | 404 文案居中 |
| `/hxdeck` | off | - | - | - | 不渲染 | 不渲染 | 不出 (off) | 组件自检页, 演示卡片与控件铺满全宽, 任何位置都会压控件 |
| `/hxdeck-editor` | off | - | - | - | 不渲染 | 不渲染 | 不出 (off) | 主题编辑器四周都是调色 / 上传 / 导出面板 |
| `/ppt` | off | - | - | - | 不渲染 | 不渲染 | 不出 (off) | 投屏页, 页面注释写明"只渲染演示页本身", 立绘是干扰 |
| `/drawio` | off | - | - | - | 不渲染 | 不渲染 | 不出 (off) | 全屏 iframe 编辑器, 四边都是工具栏 |

### 2.1 探出量与竖直位置的统一规则

- 竖直: pair / side / screen / gutter 底边抬 `--neko-lift = clamp(165px, 8vh + 125px, 245px)`, 让返回顶部按钮 (右下 1.3rem, 高约 3rem) 和页脚上沿永远在立绘下方; 槽 `max-height = 100vh - 导航栏 - lift`, 头顶不会钻进导航栏.
- 矮视口 (高 <= 640px): lift 收到 `clamp(96px, 6vh + 60px, 165px)`, 宽再乘 0.85; 高 <= 500px (横屏手机) 一律不渲染.
- pair 宽: `min(320, room - 16)`, 下限 130 (原 240). 原下限在有侧边栏的博客文章页上永远放不下, 结果是靠"推算 room"硬放, 压到侧边栏; 现在 room 是实测并扣掉障碍物, 宁小不压.
- gutter 宽: `clamp(120, room - 16, 260)`; 墙透明, 所以不侵入, 内侧裁切线靠 10% 水平淡出收口.
- pair 宽 (2026-11-04 改): 完整 `min(320, (room - 16) / 0.94)`, 需 >= 130 (NEKO_FULL); 否则紧凑 `min(320, room - 8)` (NEKO_GAP_COMPACT = 8, 不侵入, 内侧 82% 起淡出), 需 >= 72 (NEKO_MIN). 双侧优先等宽按较窄一侧; 较窄侧放不下时只出 room 大的一侧.
- gutter 双侧 (2026-11-04 改): 两侧都放得下才双侧等宽, 否则只出放得下的一侧 (原来右侧不出时左侧也不出). 文档 / 知识库 DOC_WALL 改为 sides both, 左缝 (容器左沿到文档侧边栏右沿) 够就出左侧.

### 2.2 博客文章页 (`/blog/<post>`) 版面门槛

分栏 (src/theme/BlogLayout + BlogSidebar/Desktop): row > [aside col--2 + margin-left 2rem] + main.col; main > [正文列 maxWidth 78.25%] + [TOC 列 21.75%].
正文列 = 0.7825 * (vw - vw/6 - 32). 墙 = BlogPostPage 传的 contentWidth 900; >= 1536 时 BlogWithCats.css (html.blog-post-page) 收成 min(900, 正文列 - 240).
单侧缝 room = (正文列 - 墙) / 2. 紧凑成立: room - 8 >= 72 (room >= 80); 完整成立: (room - 16) / 0.94 >= 130 (room >= 138.2).

| 视口 | 正文列 | 墙 | 单侧缝 | 结果 |
|---|---|---|---|---|
| 1280 | 810 | 810 | 0 | 不渲染 (物理放不下, 不强塞) |
| 1440 | 914 | 900 | 7 | 不渲染 |
| 1536 | 977 | 737 | 120 | 双侧紧凑, 约 112px |
| 1600 | 1018 | 778 | 120 | 双侧紧凑, 约 112px |
| 1800 | 1149 | 900 | 124 | 双侧紧凑, 约 116px |
| 1920 | 1227 | 900 | 163 | 双侧完整, 约 157px |
| 2560 | 1644 | 900 | 372 | 双侧完整, 320px (上限) |

门槛: **vw >= 1536 双侧, < 1536 不渲染**, 由 1536 媒体查询的收缝规则决定. 若撤掉收缝、墙回到 960: 门槛变成正文列 >= 1120; 按 col--3 侧栏算是 vw >= 1905 (1920 下缝 83.4, 紧凑宽 75.4), 按实际 col--2 + 2rem 是 vw >= 1756.
矮视口 (高 <= 640) 宽再乘 0.85, 门槛相应上移.

## 3. 手机端策略 (2026-11-03 重写: 方向 A, 右下角窄条 pocket)

原策略 "S <= 996 全部不渲染" 作废: 用户需求是 "所有页面都有, 并且同时适配手机端", 手机上完全不出现不算适配.
定稿 = 方向 A, 参数由用户在 390 视口实算后给定, 代码见 `NekoPocket` (index.tsx) 与 `.neko-pocket` (BlogWithCats.css).

| 断点 | 墙后探头 (.neko-sides) | pocket | 宽 | 高 | 位置 | 理由 / 数字 |
|---|---|---|---|---|---|---|
| D >= 997 (且高 > 500) | 按 §2 | 不出 | - | - | - | 桌面有固定留白 |
| S 769-996 | 不出 | 不出 (保留) | - | - | - | 侧边栏 / TOC 已收起但正文仍宽, 右下角是正文行尾; 用户定为保留不出 |
| T 481-768 | 不出 | 出 | 44px | 18vh (768x1024 下 184px) | 视口右下角, right 0, 底边抬 72px | 返回顶部按钮在右下 1.3rem, 高约 3rem (底边到按钮顶约 69px), 抬 72 在它上方 |
| N <= 480 | 不出 | 出 | 36px | 14vh (390x844 下 118px; 375x667 下 93px) | 右下角, 底边抬 76px (原 64, 与回顶按钮交 4.8px, B18) | 390 宽: main 左右 padding 16, 正文有效宽 358; 36px 窄条只剩 20px 落在 padding 外, 所以必须靠下面的 "只在页脚可见时出 + 实测" 才能保证不压字 |
| 高 <= 640 | lift 收小, 宽 x0.85 | 同所在宽度档 | | | | 矮视口 |
| 高 <= 500 且宽 > 768 | 不出 | 不出 | | | | 横屏手机: 宽 > 768 不进 pocket 档 |

**取景**: 用 pair 的 left 图 (裁切线在右), 右对齐贴视口右边, 屏幕边当墙 (与 `/anime` screen 变体同一朝向). 素材上 45% 全透明 (catgirl-spec), 图高 = 窄条高 / 0.30, 图上移 0.45 x 图高, 窄条里显示的是图高 45%-75% 那一段 (头到肩); 下沿 25% 淡出.

**何时出现** (不推算, 实测):
1. 只在页脚 (`.footer`) 进入视口时考虑出现 (IntersectionObserver), 也就是 pocket 只在页脚进视口时实测求交; 无页脚的页面视作一直在页尾 (等同页脚始终可见, 直接进入第 2 步实测).
2. 出现前与每次滚动 / 尺寸变化后, 用 `getBoundingClientRect` 量窄条矩形, 与所有可点击元素 (`a[href], button, input, select, textarea, [role=button], .theme-back-to-top-button` 等) 和正文文字块 (`p, li, h1-h6, pre, table, blockquote, figure, img`) 求交; **含页脚链接**. 有任何交集就保持 `visibility: hidden` (不可见、不吃点击), 交集写进 `__nekoDebug().pocket.blockedBy`.
3. 量的时候窄条已在 DOM 里但不可见, 所以量到的是真实位置.

**哪些页不出** (逐页见 §2 "手机端" 列):
- 有侧边栏 / TOC 的文档页 (`/docs/**`, `/knowledge-base/**`, 路由表 DOC_WALL) 与博客文章页 (`/blog/<post>`): 用户定为不出, reason `pocket-page-off`.
- off 路由 (`/hxdeck`, `/hxdeck-editor`, `/ppt`, `/drawio`): 不出.
- 其余页面挂 pocket, 但某一刻实测压到东西就不显示 (reason `pocket-blocked`); 每页实测结果见 §8.

**机械判据** (每个视口每个路由都跑, 结果见 §8):
- `document.documentElement.scrollWidth <= window.innerWidth` (pocket 是 position: fixed, 不进文档流)
- `__nekoDebug().overlaps` 为空 (且 rendered 时 `pocket.blockedBy` 为空)
- `__nekoDebug().stages` 为 0 或 1 (`.neko-sides` 与 `.neko-pocket` 合计)

**__nekoDebug 窄屏返回**: `mode: 'pocket'`, `rendered: true|false`, `reason` 取 `rendered` / `narrow-or-short` (S 档) / `pocket-page-off` / `pocket-wait-footer` (页脚还没进视口) / `pocket-blocked` / `loading` / `img-error`, 以及 `pocket: { w, h, bottom, blockedBy[] }`. 桌面返回 `mode: 'desktop'`.

**其他状态**:
| 条件 | 行为 |
|---|---|
| hover: none / pointer: coarse | 立绘 pointer-events: none, 没有 hover 态; side 的 data-shy 由滚动触发, 触摸同样适用 |
| prefers-reduced-motion | 不挂滚动监听, 不放入场 / 呼吸 / 躲藏动画; pocket 无滑入动画 |
| print | 都不渲染 |

**横向滚动条**: 槽与 pocket 都是 `position: fixed`, 越出视口的部分不计入 scrollWidth; `.neko-sides` 自身 `width:0; height:0`.

**点击区域**: 所有槽与 pocket 都设 `pointer-events: none` + `user-select: none`; 返回顶部按钮的 z-index 是 5, 比立绘 (最高 3) 和页脚 (4) 都高.

## 4. 素材来源: manifest 优先, pair1..13 回落

- 运行时请求一次 `/img/catgirl/manifest.json` (模块级缓存, 失败或 `entries` 为空则回落).
- 接受的条目形状 (宽容解析): `{ slug?, left: { file|src, w|width, h|height, fade }, right: {...} }`, 或分开的单图条目 `{ file, side: 'left'|'right', w, h, fade, slug, index }` (按 slug+index 配对).
- 逐图按规格卡校验: `w >= 600`, `|w/h - 5/12| <= 0.002`, `0.84 <= fade <= 0.94`; 不合规的单图按缺失处理. 条目无 slug 且左右文件名角色段相同的整条丢弃 (禁止同角色镜像成一对). 合格条目为 0 时回落到 NEKO_PAIRS (13 对).
- **单侧条目** (B20): `parseManifest` 只在 `!left && !right` 时跳过, 单侧条目 (needsGen) 按单侧收, 另一侧 null. `slotImgs` 对单侧图按它自己的锚点进槽 (裁切线在右进左槽, 在左进右槽), 永不镜像, 另一槽空着; 宽度只按有图那一侧的 room 算, 不被另一侧拖累. `eligible(need)` 保证 side / pocket 这类只用左槽图的变体不会抽到只有右图的条目.
- 最终状态: manifest 有 25 条 (22 套左右对 + 3 条单侧) 和 9 个 excluded 文件, 运行时 `assets: 'manifest'`; 清单见 `catgirl-normalize.md` "最终清单".

## 5. 防叠组

- 页面自己挂的 SideCats (BlogWithCats 内部) 挂载时在模块级登记 (+1), 卸载 -1; RouteCats 用 `useSyncExternalStore` 订阅, 计数 > 0 时返回 null.
- `/anime/details` 最终不挂页内 SideCats (B21), 由路由表的 gutter 渲染, 因此也只有一组.
- 结果: 任何页面最多一组舞台. dev 自检函数返回 `stages` 数, 只数 `.neko-sides[data-on]` 与 `.neko-pocket[data-on]` (B19), 应恒为 0 或 1; no-room / 隐藏中的舞台不计.

## 6. dev 自检函数

`window.__nekoDebug()` (全站可用, 无副作用) 返回:

```
{ path, variant | null, source: 'page' | 'route' | null, mode: 'desktop' | 'pocket' | null, rendered, reason, pair, files,
  assets: 'manifest' | 'builtin', room, stages, slots: [{ side, rect }], hScroll: boolean, overlaps: [选择器...],
  pocket: { w, h, bottom, blockedBy } | null, reducedMotion, viewport }
```

- `overlaps`: 立绘槽矩形 (按 alpha 不透明区近似为槽矩形下 55%) 与可点击元素 (`a, button, input, select, textarea, [role=button], .navbar, .theme-doc-sidebar-container, .table-of-contents, .theme-back-to-top-button`) 的交集列表, 期望为空.
- `reason`: `rendered` 或不渲染的原因 (`narrow-or-short` / `no-room` / `off` / `no-route` / `page-owns` / `img-error` / `loading` / `pocket-page-off` / `pocket-wait-footer` / `pocket-blocked`).

## 7. 破绽清单

| # | 破绽 | 位置 | 状态 |
|---|---|---|---|
| B1 | /anime/details 右侧叠两个人 (页内 pair + 路由 side) | 路由表 | 已修: 删路由 + 挂载登记 |
| B2 | 博客文章页左立绘压"所有文章"侧边栏, 右立绘压 TOC | pair room 推算 | 已修: room 实测并扣障碍物 |
| B3 | edges 右下角立绘压返回顶部按钮 | 首页 / 关系图 | 已修: 返回顶部 z 5 |
| B4 | pair 下限 240 导致"放不下就靠推算硬放" | NEKO_MIN | 已修: 下限 130 + 实测 |
| B5 | 透明墙 (文档容器) 后放立绘会透出内侧 6% | 新 gutter | 设计规避: gutter 零侵入 + 内侧淡出 |
| B6 | 镜像槽的水平淡出方向 | pair neko-flip | 复查: 现有 CSS 已按翻转写反方向, 保持 |
| B7 | 正文块圆角 25px 处, 立绘内侧竖边在圆角外露出一条 | pair 墙沿 | 已修: 立绘竖直范围下沿之上的墙段是直边; 侵入 6% 大于圆角在 lift 高度处的内缩 (lift >= 165 > 25), 圆角只出现在正文块底部, 届时立绘已沉到页脚后 |
| B8 | 滚动条宽度让 50% 推算偏差 | pair | 已修于 2026-11-03 (实测墙), 本轮沿用 |
| B9 | 矮视口头顶钻进导航栏 | 全变体 | 已修: max-height + 矮视口 lift |
| B10 | 语言切换浮层 | 全站 | 不适用: i18n 只有 zh-Hans, 无语言切换下拉; 导航栏下拉 (更多 / CDN / 音乐) z >= 200, 恒在立绘之上 |
| B11 | 模态 / 浮层 (ZoomImage 9999, 歌词窗 9999/10000, 音乐面板 9998) | 全站 | 不冲突: 立绘最高 z 3 |
| B12 | reduced-motion 下 side 仍可能因 data-shy 位移 | side | 已修: reduced-motion 不挂监听, CSS 也把 data-shy 位移取消 |
| B13 | 图片 404 时留下空槽 | 全变体 | 已修: img onError 隐藏整组 |
| B14 | `/search` 与 404 没有 `<main>`, CENTER_WALL 找不到墙也没有 fallbackWidth, 桌面端恒不渲染 (与 §2 写的 ">= 1440 渲染" 矛盾); `/anime/details` 可能被 FALLBACK 的 gutter 捕到 | 路由表 / useWallFit | 已修: CENTER_WALL 加 `fallbackWidth: INFIMA_CONTAINER` (1140), 推算时 >= 1440 宽改用 1320, 墙位 = 视口中线 ± 宽/2; 路由表加 `/anime/details` 显式 off, 由页内 SideCats 渲染 |
| B15 | pocketBlockers 没豁免页脚, 滚到页底时 pocket 必与 `li.footer__item` 相交, 手机上几乎永远 pocket-blocked | NekoPocket | 已修: 求交时跳过祖先含 `.footer` 的元素 (只豁免页脚, 其他元素照判) |
| B16 | `__nekoDebug` 把 visibility:hidden 的 pocket 算进 slots / overlaps / stages, 手机端判据误报 | nekoDebug | 已修: 只取 `.neko-pocket[data-on]` |
| B17 | gutter / pair 放不下时宽度 = room - 16 可能为负, 自检报告里出现负宽度 | useWallFit | 已修: wl / wr / pair w 夹到 >= 0; 此时 show 为 false, reason = no-room |
| B18 | <= 480 档 pocket 底边抬 64px, 回顶按钮顶在 20.8 + 48 = 68.8px, 两者交 4.8px; 按钮显示时 pocket 被判 pocket-blocked, 回滑时立绘闪退 (static-projection-v2 N2) | BlogWithCats.css `--pk-lift` | 已修: N 档 `--pk-lift: 76px` (T 档 72 不变), 高出按钮顶约 7px |
| B19 | no-room 时空的 `.neko-sides` (无 data-on) 仍计入 stages, 判据要看 reason 才能读 (N3) | nekoDebug | 已修: stages 只数 `.neko-sides[data-on], .neko-pocket[data-on]` |
| B20 | manifest 单侧条目 (mahiro-02 / milim-02 / unk-blonde-twintail-01, needsGen) 被 parseManifest 整条丢掉; 单侧图若走双侧逻辑还会被镜像或按两侧较小 room 算宽 | parseManifest / slotImgs / useWallFit | 已修: 只在 `!left && !right` 时跳过; slotImgs 单侧按锚点进槽不镜像; 宽度只按有图那侧 room 算 |
| B21 | `/anime/details` 页内 pair 找不到 `.blog-content`, 按 contentWidth 推算墙位与真实 shell (max-width 1200) 不符, 立绘浮在正文上 | 路由表 / details.tsx | 已修: 番剧详情是卡片式布局, shell 透明且无 z-index, pair 变体的"身体藏进墙后"无法成立; 去掉页内 SideCats, 路由表改用 gutter, 墙取 shell 外沿 (`DETAIL_WALL`, `[class*=shell]`), 零侵入 + 内侧 10% 水平淡出. 同时解决 static-projection-v2 的 N1 (详情页整页无立绘) |

备注: static-projection-v2 的 N4 (B14 注释称 404 无 `<main>`, 实际有 `main.container`) 只涉及注释, 不影响行为; 404 本来就量到墙, 只有 `/search` 走 fallback.

## 8. 自检记录

验证环境说明: 我 (Hark) 不能在你的机器上执行命令, 只能通过映射地址读写文件. 所以下面每一项都标明来源: **[用户本机]** = 你在工作树实测并告诉我的; **[沙箱镜像]** = 我在沙箱里用 GitHub main (95d7d61) 克隆, 叠加你盘上的 src 与本次三个文件跑的; **[未验证]** = 都没跑过.

### 8.1 tsc --noEmit

| 来源 | 改前 | 改后 | 新增 |
|---|---|---|---|
| [用户本机] | 10 | 10 | 0 |
| [沙箱镜像] | 52 | 52 (B14-B17 修复前测得; 修复后的复测见 CHECKPOINT) | 0 (中途出过 1 条 TS2322: RouteCats 里 'off' 没被收窄; 已修, 修后 BlogWithCats 目录 0 条) |

用户本机的 10 条, 都是仓里原有的 (按用户给的分布): `docusaurus.config.ts` 5 条, `DevTools/InlineEditor` 1 条, `Giscus` 1 条, `AnimeGraphPage` 1 条, `docs-graph` 1 条, `MDXContent` 1 条. 这些文件本次都没动. 镜像多出的 42 条来自镜像缺少你本机的依赖和生成物, 改前改后完全相同.

### 8.2 npm run gate:fast

[未验证: 用户本机]. 我没法在你的工作树执行; 已在对话里给出要在本机跑的命令. gate:fast 的检查项里, 只有 "TS 类型检查 (无新增错误)" 会碰到本次改动, 这一项 8.1 两边都是 0 新增; 其余检查 (hxid / tag 词表 / 标点归一化 / 生成物新鲜度 / 演示页引用) 只看 docs / blog / data, 本次没改.

### 8.3 uv run scripts/redlines/agent_notes.py --diff

[沙箱镜像] 在只叠加本次 4 个文件的 worktree 里跑 (三个代码文件 + catgirl-sides note; 另外把你盘上的 `src/pages/anime/share.tsx`, `src/theme/BlogListPage/index.tsx` 作为基线提交进去, 因为 GitHub main 里没有这两个文件). 原始输出:

```
错误 src/pages/anime/share.tsx:10 [note-path] 精确仓库路径上不存在有效的决策记录: .agents/notes/implemented/feature/2026-11-01-anime-share-link.md
错误 .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md:43 [anchor-cardinality] src/theme/BlogListPage/index.tsx 中必须有且仅有一个 AST 声明锚点; 当前在其目录内找到 0 个
待审核 src/components/BlogWithCats/index.tsx:1 [code-only-diff] 代码已改动, 对应决策记录未同步修改: .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md; 请审核
待审核 src/theme/Root.tsx:1 [code-only-diff] 代码已改动, 对应决策记录未同步修改: .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md; 请审核
待审核 .agents/notes/implemented/feature/2026-11-02-blog-catgirl-sides.md:1 [note-only-diff] 决策记录已改动, 目录 src/pages/anime 中的代码未同步修改; 请审核
待审核 .agents/notes/implemented/feature/2026-11-02-blog-catgirl-sides.md:1 [note-only-diff] 决策记录已改动, 目录 src/theme/BlogListPage 中的代码未同步修改; 请审核
待审核 src/components/BlogWithCats/BlogWithCats.css:1 [resource-review] 受保护资源已改动, 其格式不在 AST 支持范围内; 请审核对应决策及引用
Agent Notes: 需处理, 2 项错误, 5 项待审核
exit 1
```

逐条判读:
- 2 项错误, 都不是本次改动引起的: (1) `share.tsx:10` 引用的 `2026-11-01-anime-share-link.md` 在 GitHub main 里不存在, 你本机应该有, 是镜像缺文件; (2) adoption note 要求 `BlogListPage/index.tsx` 有锚点, 是你本机另一路的改动, 镜像里 adoption note 是旧版. 本次三个代码文件没有错误.
- 第一次跑时, 本次改动自己引出过 1 项真错误: 重写 index.tsx 时把原来 `Props` 上方的 adoption note 锚点删掉了 (`anchor-cardinality ... BlogWithCats/index.tsx 0 个`). 已补回 (index.tsx `interface Props` 上方), 重跑后这项消失.
- 5 项待审核 (需要人工确认):
  - `code-only-diff` x2: index.tsx 与 Root.tsx 也被 adoption note 引用, 但本次是 catgirl 功能改动, adoption note 内容不需要改. 本次改动对应的决策记录是 catgirl-sides note, 已同步更新.
  - `note-only-diff` x2: catgirl-sides note 的 Code 列表里有 `src/pages/anime/share.tsx` 与 `src/theme/BlogListPage/index.tsx`, 本次没改这两个文件 (anime 目录是禁区). note 改动只是追加了 2026-11-03 的决策段和 `src/theme/Root.tsx` 这一条, 那两条原有条目没动.
  - `resource-review`: BlogWithCats.css 不是 AST 可解析的格式, 需人工看. 它的决策依据在 catgirl-sides note 的 2026-11-03 段, 和 index.tsx `NekoStage` / `NekoPocket` 注释里.

### 8.4 改动了既有决策记录 (给 review 的说明)

`.agents/notes/implemented/feature/2026-11-02-blog-catgirl-sides.md` 显示为 M, 是有意修改, 不是误改. 原因: Agent Notes 规则要求代码改动和决策记录成对, 本次改的是这条 note 管的代码. 改了什么 (只增不删):
- `## Decision` 末尾追加一段 "2026-11-03 全站覆盖与手机端": 新增 gutter 变体, 路由表, 挂载登记防叠组, 实测留白扣障碍物, manifest 优先, `__nekoDebug`. 手机端那句写的是 "<= 996 不渲染", 已经过时, 以 §3 的 pocket 为准 (见下面待办).
- `## Alternatives considered` 追加 3 条 2026-11-03 被否决的方案.
- `## Code` 追加 `src/theme/Root.tsx` (RouteCats 的挂载点, 锚点在 Root 上方的注释块).

待办: note 里 "<= 996 不渲染" 那句需要改成 pocket 策略, 等截图验收通过后一起改, 免得 note 先于代码定稿.

### 8.5 手工检查过的路由

截图验收见 8.6. 截图之前我做的是源码层面的逐页走查 (路由表 `ruleFor` + 各页是否挂 `SideCats`), 结论已写进 §2 的 "手机端" 列: `/`, `/anime`, `/anime/details`, `/anime/graph`, `/anime/cv`, `/anime/share`, `/friends`, `/blog`, `/blog/page/N`, `/blog/archive`, `/blog/tags*`, `/blog/authors`, `/docs/tags*`, `/knowledge-base/tags*`, `/docs-graph`, `/search`, `/404` 都挂 pocket; `/blog/<post>`, `/docs/**`, `/knowledge-base/**` 是 page-off; 4 个 off 路由不挂.

### 8.6 截图与 __nekoDebug

**未完成.** peek-shots/ 目前没有截图. 进度:
- 沙箱里用裁剪仓库 (`docs` 只留 000-关于 与 010-面试, blog 只留 2026, 侧边栏改 autogenerated) 跑 build, 已产出 build/ (404, anime, anime/details, blog, blog/2026/*, docs/关于, docs/面试/*, search 等都有 html); 只有 `/` 的 SSG 失败 (首页可以从 404.html 走客户端路由渲染).
- 之前有一次 build 把沙箱内存吃爆 (exit 137), 沙箱重启后 playwright 要重装; 现在已装到 /workspace (chromium headless shell v1248).
- 下一步: 用 `python3 -m http.server` 起 build/ (首页回落 404.html), 跑截图脚本 (5 视口 x 7 路由, 存 `<宽>px-<slug>.png` 与 `debug-<宽>px.json`), 再 PUT 进 peek-shots/.
- 注意: 这次 build 用的是 B14-B17 修复**之前**的代码, 截图前要用当前 index.tsx 重新 build.

### 8.7 三条机械判据

| 判据 | 结论 | 来源 |
|---|---|---|
| scrollWidth <= innerWidth | 预期成立: 所有槽与 pocket 都是 position: fixed, `.neko-sides` 宽高为 0 | 源码推演, **未实测** |
| overlaps 为空 | 桌面: 预期成立, 因为 room 是实测的, 已扣掉侧边栏和 TOC. 手机: B16 修好之后, 隐藏的 pocket 不再误报; 显示中的 pocket 已经用 pocketBlockers 实测过不压 | 源码推演, **未实测** |
| stages 为 0 或 1 | 预期成立: 页内 SideCats 挂载后, RouteCats 会让位; 墙后探头只在 > 996 宽时出, pocket 只在 <= 768 宽时出, 两者不会同时出现 | 源码推演, **未实测** |

三条都要等 8.6 截图时跑 `__nekoDebug()` 才算验收通过.

### 8.8 最终自检记录 (2026-11-04 CST, 代码收口后)

来源全部是 **[用户本机]** (用户在工作树实测后告知), 本节只记录结论.

| 项 | 结果 |
|---|---|
| `npx tsc --noEmit` | 10 条, 全部是仓里原有 (分布见 8.1), **无新增** |
| characterPrompt.check | 22 项全通 |
| 立绘素材机械校验 | 56 张全过 (AR 5:12 / top 0.450 / 内侧 gap=0 / 无深色残带 >= 5 列); 9 张曾截断的图已重传并验证可读. 明细见 `catgirl-normalize.md` "校验结果" |
| manifest 覆盖 | 25 entries + 9 excluded = 覆盖 `static/img/catgirl/` 下 56 个文件 (pair1..13 内置回落另计) |
| B14-B21 | 全部已修, 逐条修法与验证方式见 `peek-shots/README.md` "最终状态" |

代码最终状态要点:
- `/anime/details`: gutter + `DETAIL_WALL` (`[class*=shell]`), 不是 pair 也不是 off (B21).
- pocket: T 档底边抬 72px, N 档 76px (B18); `__nekoDebug().stages` 只数 data-on (B19).
- manifest 单侧条目可用 (B20).

**手机端 pocket 求交时机说明**: pocket 只在页脚 (`.footer`) 进入视口时才实测求交并决定是否显示, 页脚之外的滚动位置一律 `pocket-wait-footer`; 若页面没有页脚, 视为一直处在页尾, 每次滚动 / 尺寸变化都直接实测求交.

8.6 / 8.7 的 "未完成 / 未实测" 状态说明: 云端仍没有真实截图; 本机复验方法见 `peek-shots/README.md` (`npm run build && python3 -m http.server 3001 -d build` 后逐路由取 `window.__nekoDebug()`).
