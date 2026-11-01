# Agent Note: 首页"更多"关系图浮层补关闭过渡

Status: implemented

Decision-ID: home-graph-close-transition

## Problem
首页两个"更多": WakaTime 的"更多 · 展开趋势图" (折线图) 展开和收起都有过渡 (grid-rows + clip-path + 光带, page.module.css 的 .magic). 项目区"更多"打开关系图浮层有 260ms sheetIn / fadeIn, 但关闭时 HomePage 直接卸载 EcosystemPanel, 浮层瞬间消失, 是硬切.

## Decision
- EcosystemPanel 加 `closing` 状态: Esc / 点遮罩 / 关闭按钮都改走 `requestClose`, 先给 .scrim 和 .sheet 挂 `data-closing`, 播 180ms 淡出 + 下沉 (`sheetOut` / `fadeOut`). 浮层自身 animationend 后再调 HomePage 的 onClose 卸载; 320ms 定时器兜底.
- 关闭中 `pointer-events: none`, 防止二次点击.
- prefers-reduced-motion: JS 直接 onClose, CSS 同时把关闭动画置 none.
- 折线图那条已经完整, 不动.

## Alternatives considered
- 什么都不做: 最强理由与结论 —— 浮层直接消失功能上没坏, 不加动画就不加维护点. 否决: 打开有过渡而关闭瞬间消失, 观感不一致, 属于用户点名的不协调.
- 复用现有打开动画的方向反过来播放: 最强理由与结论 —— 最强理由是只改一个 animation-direction, 代码量最小. 否决: 打开是 scale 由小到大, 反向播放会把浮层缩到看不见才开始位移, 与关闭的整体收拢观感不符.
- 在 HomePage 里延迟卸载: 要把动画时长同步到两处, 不如让浮层自己管.
- View Transitions: 浮层是 fixed 的单元素, CSS keyframes 足够.

## Consequences
- 关闭到卸载有 180ms 延迟; 焦点在卸载后回到"更多"按钮, 行为不变.

## Code

- `src/components/home/EcosystemPanel.tsx`
