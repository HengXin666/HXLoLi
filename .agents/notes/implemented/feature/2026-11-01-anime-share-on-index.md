# Agent Note: 分享番单接到番剧库列表页 (/anime), 并回退列表页审美重做

Status: implemented

Decision-ID: anime-share-on-index

## Problem
用户说难看的是详情页, 不是列表页 (/anime). 列表页"差不多可以". 同一天做的列表页审美重做 (token、固定列数、5:7 封面、工具栏一行化) 属于误解, 要回退.
另外, 分享番单 (animeShareImage / animeShareText) 只有独立路由 /anime/share, 列表页上没有入口. 用户要求入口放在列表页, 不放详情页.

2026-11-02 反馈: 弹窗里挂的是旧的 AnimeSharePanel (只能出图、出链接), 而勾选、表格视图、分享图预览只在 /anime/share 的 AnimeShareBoard 里, 成了两套页面两套逻辑. 用户原话: "他在第1次就是主页那里点击有1个弹窗，在这里，他应该就能预览到这个分享出去的效果啊，或者说图片的效果啊". 要求弹窗里就能用全部功能, 独立页保留给别人打开链接落地, 两处同一个组件.

## Decision
- 列表页 4 个视觉文件回到重做前的内容:
  - index.tsx、AnimeIndexPage.module.css、PosterCard.tsx、PosterCard.module.css 用 .hark-bak 里的重做前版本恢复.
  - FilterBar.module.css 按原文逐字重写, 9262 B, 与原文件同大小.
  - FilterBar.tsx 用 .hark-bak 恢复.
  - 筛选动画、骨架、错误重试、空态建议、URL 状态都在原版本里, 照常保留.
- FilterBar 新增可选 prop `onShare`: 给了就在工具区 (搜索 / 状态 / 标签与更多 / 排序 / 视图那一行) 最右放"分享"按钮, 样式复用 panelBtn.
- index.tsx 传入 onShare, 打开一个 <dialog>, 里面渲染 `<AnimeShareBoard embedded />` (与 /anime/share 同一个组件). 只在打开时挂载, 关上就卸载, 不在后台画图.
  - 弹窗宽 min(1200px, 96vw), 高 min(90vh, 1000px), 内部滚动; 背景与边框改用 Infima 变量, 明暗主题都可读.
  - 弹窗里: 切卡片 / 表格、勾选 (与独立页同一份 localStorage 勾选)、页内筛选、生成链接、复制清单、生成并预览分享图; 分享区在右侧吸顶栏, 预览图在弹窗内显示.
  - 打开弹窗不改地址栏: 视图模式只存组件 state; 只有点"在独立页面打开"才跳到 /anime/share (带上番单筛选、勾选与视图).
  - 关闭方式: 右上 ×、Esc、点遮罩.
  - 窄屏 (≤720px) 贴底.
- AnimeShareBoard 的 `embedded` prop (`AnimeShareBoardProps`): 为 true 时不读写 URL 的 view, 隐藏"← 回番剧库", 改为"在独立页面打开 →", 吸顶位置从 0 算. 番单定义照旧读 location.search (即番剧库当前筛选).
- AnimeSharePanel.tsx / AnimeSharePanel.module.css 删除; 它的"复制清单 (一行一部 / 逗号分隔)"并入 AnimeShareBoard 的分享区, 按状态起标题 (shareTitle) 不再保留, 图片标题统一为"我看过的番 · N/M"或"番单 · N 部".
- 详情页不放分享按钮.

## Alternatives considered
- 什么都不做, 入口继续只在独立路由 /anime/share: 最强理由是列表页零改动, 不增加 /anime 的打包体积, 也不会和并行维护列表页的改动冲突. 否决: 用户明确要求入口放在列表页, 独立路由要靠手改地址才能找到.
- 复用现有独立页, 列表页只放一个跳转链接: 最强理由是只加一个 <Link>, 分享逻辑完全不进列表页. 否决: 跳转会离开列表页、丢掉滚动位置, 不符合"挂在列表页"; 弹层里直接复用 AnimeShareBoard 同样零重复代码.
- 保留 AnimeSharePanel, 只在里面补上表格与预览: 最强理由是改动面小, 不碰已稳定的分享页. 否决: 勾选、筛选、视图、预览会在两个组件里各写一份, 正是用户说的"两个页面"; 合成一个组件后两处行为天然一致.
- 经 lazyChunks.js 动态 import 面板: lazyChunks 在 src/components/home/, 本轮不许动. 先静态 import, 面板只在打开时挂载.

## Consequences
- /anime 的 JS 多了 AnimeShareBoard、animeShareLink、animeShareImage、animeShareText 的体积 (源码约 70 KB, 未压缩).
- /anime/share 独立路由保留, 与弹窗同一组件, 见 `.agents/notes/implemented/feature/2026-11-01-anime-share-link.md` 与 `.agents/notes/implemented/feature/2026-11-01-anime-share-table-view-and-preview.md`.
- 弹窗里勾选写的是与独立页同一个 localStorage 键 (按番单筛选区分), 在弹窗里勾过的, 打开同一番单的分享页仍在.
- 2026-11-01-anime-index-visual-redo.md 描述的改动已全部回退, 那份 note 作废.
- 2026-11-02 晚: 关闭按钮改为弹窗右上角绝对定位的 38px 圆形 ✕ (不透明背景 + 边框); 弹窗自身 overflow: hidden, 内容放进 .shareBody 滚动, 顶部留 56px, 内容永远滚不到按钮下面.

## Code
- `src/pages/anime/index.tsx`
- `src/components/anime/Tag/FilterBar.tsx`
