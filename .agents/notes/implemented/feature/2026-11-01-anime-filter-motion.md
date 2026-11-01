# Agent Note: 番剧列表筛选切换过渡

Status: implemented

Decision-ID: anime-filter-motion

## Problem
切换状态 / 标签 / 排序时列表是硬切, 卡片瞬间换掉, 看不出哪些留下、哪些是新来的.

## Decision
- 不用 framer-motion (首页刻意去掉过). 新增 `src/utils/anime/listMotion.ts`, 两层:
  1. `runListTransition(update)`: 支持 `document.startViewTransition` 时, 包住 `history.push/replace` (内部 `flushSync`), 视口上下各一屏内的卡片临时挂 `view-transition-name: anime-<id>`. 留下的卡片平移到新位置, 新来的淡入 (`::view-transition-new(*):only-child`), 离开的淡出 (`::view-transition-old(*):only-child`), 其余区域交叉淡化. 结束后清掉名字和 `html[data-anime-vt]`.
  2. `useListFlip(ref, groups)`: 不支持 View Transitions, 或这次变化不经 `setFilters` (浏览器后退 / 前进) 时, 用 Web Animations 做 FLIP 重排 + 新项淡入. 离开的项直接移除.
- PosterCard 的两种 `<li>` 加 `data-anime-id`, 供定位.
- `prefers-reduced-motion: reduce`: JS 两条都不跑 (直接 update / 不做 FLIP), CSS 里 `::view-transition-*` 的 animation 也置 none.
- 过渡样式只在 `html[data-anime-vt]` 下生效, 不影响站点其它页面.

## Alternatives considered
- 什么都不做: 最强理由与结论 —— 筛选切换本来就是瞬时的, 功能上完全正确, 不加代码就不加动效维护成本和减少动效偏好下的风险. 否决: 用户明确指出切筛选像"瞬间闪烁"不好看, 这是纯粹的体验缺陷, 而修法不需要新依赖.
- 复用现有方案 (直接用 CSS transition 作用于列表容器): 最强理由与结论 —— 最强理由是最省代码, 一行 transition 就能有过渡. 否决: 筛选变化时是做增删与重排, 容器高度与子项位置都变, 单纯 transition 不会给移除项出场时间, 也做不出重排后的位移动画.
- framer-motion `AnimatePresence` + `layout`: 体积大, 首页特意移除过, 否决.
- 给全部卡片挂 view-transition-name: 几百部时快照开销大, 只挂视口附近的.
- 纯 CSS 入场 keyframes: 做不了重排位移, 且 grid ↔ list 切换会整页重复播放.

## Consequences
- Chrome/Edge 111+、Safari 18+、Firefox 144+ 走 View Transitions; 其余浏览器只有重排和入场, 没有退场.
- 过渡期间 (260ms) 页面点击由浏览器挂起, 连续快速点选时后一次会跳过前一次的动画.

## Code

- `src/utils/anime/listMotion.ts`
- `src/components/anime/AnimeCard/PosterCard.tsx`
