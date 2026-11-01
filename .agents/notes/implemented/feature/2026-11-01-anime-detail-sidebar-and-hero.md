# Agent Note: 番剧详情页 Hero 重做 + 页内目录改左侧侧栏

Status: implemented

Decision-ID: anime-detail-sidebar-and-hero

## Problem

用户原话: 「这个详细页面的顶部封面……看起来是有一点难看的」「顶部那个简介剧集角色的悬浮栏根本就不需要, 要么放到侧边, 要么就不要放在那里」「详细页面比较难看」.

对照 `2026-11-01-anime-detail-redesign` 落地后的页面, 具体毛病:

- 页内导航 `.sectionNav` 是 `position: sticky; top: var(--ifm-navbar-height)` 的横条, 滚动全程压在正文上方, 和 Docusaurus 顶栏叠成两层, 没有当前段高亮, 只是一排链接.
- Hero 整块铺 `blur(40px)` 的封面再压 88% → 50% 的黑罩: 封面颜色一脏整块就发灰发泥, 亮色主题下也是一块黑板, 文字被迫固定为浅色; 封面 240px + `0 18px 40px rgba(0,0,0,.45)` 的重阴影, 首屏偏大偏重.
- 元信息 (季度 / 首播 / 话数 / 评分) 全是同样大小的半透明胶囊, 和状态卡、标签胶囊、操作按钮一共四排胶囊堆在一起, 没有层次.
- 主题色是 `#ff88ff`, 已看剧集方块和"主角"徽章用白字压在主题色上, 对比度只有 2.06:1; 辅助文字用 `emphasis-600`, 亮色下只有约 3:1.
- 间距节奏不一 (区块 32px, 组内 18px / 14px / 8px 混用), 圆角 18 / 14 / 12 / 10 / 8 / 6 混用, 骨架屏用了静态内联 style.

## Decision

只改 `details.tsx` 与 `AnimeDetailPage.module.css`, 不新增组件文件和依赖.

页内目录:

- 横向吸顶条删除, 改为文件内 `SectionNav` + `useActiveSection`. 正文区包一层 `.layout`: 宽屏 (>= 996px, 与 Infima 桌面断点一致) 两栏 `160px | 1fr`, 栏距 48px, 目录在自己那一栏 `position: sticky; top: navbar + 24px`, 竖排、左侧 1px 轨道 + 2px 主题色指示当前段, 不覆盖正文.
- 窄屏 (< 996px) 收成正文上方一条可横滑的细条 (底边 1px 分隔), 不吸顶.
- 高亮用 `IntersectionObserver` (rootMargin `-80px 0px -55% 0px`), 取视口上部可见的最靠前一段; 点击时立即高亮, 锚点跳转照旧 (`scroll-margin-top: navbar + 16px`). 不支持 IO 时停在第一段.
- "讨论" 也改用 `Section`, 有 h2 标题, 目录能定位并高亮它.

Hero:

- 改为主题色卡片 (`--ifm-background-surface-color`, 1px `emphasis-200` 边, 圆角 16px), 文字走主题色, 明暗都可读. 虚化封面不删, 降成左上角 18% 不透明度的光晕, 用 160° 渐变 mask 在 65% 处淡出, 没有硬边.
- 封面 `aspect-ratio: 2 / 3`, `max-width: 220px` (窄屏 104px), 外框 `.coverFrame` 圆角 12px、`overflow: hidden`, 图片 `object-fit: cover` 填满不变形; 阴影为 1px 描边 + `0 2px 6px rgba(0,0,0,.08)` + `0 16px 36px -12px rgba(0,0,0,.35)`.
- 网格 areas: 宽屏 `cover head / cover body`; 窄屏 (<= 760px) 封面缩到左上 104px, 标题在右, 状态 / 标签 / 操作占满整行.
- 三级层次: 标题 32px / 800 / 正文色; 原名 15px / 500 / 辅助色; 元信息 14px / 400 / 辅助色, 一行以 "·" 分隔 (评分放首位, 星标金色, 分数 16px / 700 正文色). 观看进度在状态卡内: 已看数 18px / 700 正文色, 其余 14px 辅助色.
- 返回链接移到 Hero 上方; 无标签时不再显示 "暂无标签" 占位; 操作按钮改为描边次级按钮, "番剧-角色-声优关系图" 缩为 "关系图".

整体:

- 颜色变量: 辅助色 `color-mix(正文色 68%, 背景色)`; 主题色做文字时亮色压暗为 `color-mix(primary 55%, #000)` (`#8c4b8c`), 暗色直接用主题色; 主题色块上的字统一 `#2b0a2b`.
- 间距: 区块 48px (<= 760px 为 36px), 区块标题到内容 16px, 分组之间 24px, 网格 / 卡片间距 8-16px.
- 圆角: 大卡 16px, 卡片 12px, 小件 8px, 胶囊 999px. 边框统一 `emphasis-200`.
- 字号: h2 20px / 700, 正文 15px (行高 1.8, 简介限 72ch), 辅助 13px.
- 剧集方块 44px 起 / 40px 高 / 间距 8px, 数字等宽; 角色头像 64px 宽 3:4; 海报 120px 2:3, 悬停改为主题色描边而不是位移; 状态徽章换成更浅的同色系保证深字可读.
- 骨架屏的宽高从内联 style 改成类 (`skeletonTitle` / `skeletonW40` / `W55` / `W75`).

数值表 (供核对):

| 项 | 值 |
| --- | --- |
| 封面 aspect-ratio | 2 / 3 |
| 封面 max-width | 220px (<= 760px: 104px) |
| 侧栏宽度 | 160px, 与正文栏距 48px |
| 侧栏断点 | >= 996px 侧栏; < 996px 横滑细条 |
| 区块间距 | 48px (<= 760px: 36px); 标题到内容 16px |
| h1 | 32px / 800 (<= 760px: 22px) |
| h2 | 20px / 700 |
| 正文 | 15px, 行高 1.8 |
| 辅助文字 | 13px (元信息 14px) |
| 正文对比度 | 亮色 #1c1e21 / #ffffff 16.71:1; 暗色 #e3e3e3 / #121212 14.6:1 (卡片 #242526 上 11.96:1) |
| 辅助文字对比度 | 亮色 #656668 / #fff 5.75:1; 暗色 #a0a0a0 / #121212 7.16:1 |
| 主题色文字 | 亮色 #8c4b8c / #fff 6.0:1; 暗色 #ff88ff / #121212 9.11:1 |
| 主题色块上的字 | #2b0a2b / #ff88ff 8.65:1 (原白字 2.06:1) |

对比度按 WCAG 2 相对亮度公式算, 颜色取 Infima 默认值与 `src/css/custom.css` 覆盖值 (亮色背景 #ffffff, 暗色背景 #121212).

## Alternatives considered

- 什么都不做 / 只调颜色: 改动最小, 不碰上一轮刚落地的结构. 否决: 用户点名的是吸顶横条和 Hero 观感, 换色解决不了横条压内容和四排胶囊堆叠.
- 复用现有: 直接删掉目录、只留锚点 (用户也接受). 最强理由是页面短的番 (没有相似、关联少) 根本不需要目录, 删掉最干净. 否决: 角色多的番页面很长, 宽屏左侧本来就是空白, 放一个不占正文宽度的侧栏几乎零成本, 还能看到读到哪; 窄屏用不吸顶的细条, 不再盖内容.
- 复用 Docusaurus 文档页的 TOC 组件 (`@theme/TOCItems` 等): 风格与站内文档一致. 否决: 那套依赖 Markdown 生成的 toc 数据与 `.theme-doc-toc` 布局, 本页是自定义 page, 喂数据要绕路, 且它在右侧, 用户要的是左侧.
- Hero 完全去掉虚化背景: 最干净. 否决: 完全去掉后 Hero 与下方卡片没有区分, 也丢了每部番自己的色调; 降到 18% 并渐隐后既保留氛围又不再发泥.
- 封面 3:4: 更矮, 首屏更省. 否决: Bangumi 封面本身多为 2:3 左右, 3:4 会裁掉标题字或人物头顶; 关联 / 相似海报和骨架屏已是 2:3, 保持一致.

## Consequences

`src/pages/anime/AnimeDetailPage.module.css` 重写 (非 AST 资源, 需人工核对): 删除 `sectionNav` / `heroInfo` / `tagEmpty` 类, 新增 `layout` / `main` / `toc` / `tocTitle` / `tocList` / `tocLink` / `tocActive` / `coverFrame` / `heroHead` / `heroBody` / `epGroup` / 骨架宽度类, `backLink` 从 Hero 绝对定位改为普通流. 只有本页引用这些类.

`2026-11-01-anime-detail-redesign.md` 里原先的 "吸顶页内导航" 和 "Hero 封面虚化铺底 + 固定浅色字" 已就地改写为本决策的侧栏目录与主题色 Hero, 两篇 note 事实一致. Giscus 的 term 未变, 旧讨论不丢. 依赖 `color-mix()` (Chrome 111 / Safari 16.2 / Firefox 113 起), 上一轮已在用. 本机 `npx tsc --noEmit` 已复核: 错误全在 `scripts/quality-baseline.json` 基线内, `details.tsx` 零错误.

## Code

- `src/pages/anime/details.tsx`
