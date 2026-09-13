# 演示页(PPT)卡片: 滚动 / 侧栏不同步 / 预览放大隐患

- 日期: 2026-09-12
- 状态: implemented
- 类别: bug-fix
- 影响: `src/hxdeck/{Deck,PptCard,PptEmbed,diagram,slide-state}.*`

## Problem

`[##PPT##](/ ##PPT 3##)` 内嵌演示页在笔记里出现三类问题:

1. **无法滚动** —— 鼠标停在卡片上时页面滚不动 (把卡片换成"指定页码"的写法后更明显)。
2. **侧边栏不随页面变化** —— 一旦链接里写了页码 (`[##PPT 3##]`), 点 deck 自己的章节栏 / 进度点 /
   翻页按钮后, 长条滚到了新的一页, 但高亮、圆点、进度条全部不动。
3. **预览态放大是全屏网页, 退出不明显且背景不对** —— 在卡片预览里点架构图的放大,
   浮层会被卡片的 `stage scale()` 裁住/算错尺寸, 看着像"整页变成了演示页", 退不出去。

## Root cause

- (2) **页码受控即冻结**: `Deck` 里 `current = index ?? innerIndex`, 而 `applyGo` 只在
  `index === undefined` 时才写 `innerIndex`。PptEmbed 传了 `index`, 于是 `current` 恒等于 props,
  内部 state 更新了也没人读 —— 侧栏/圆点/进度条自然不动。
- (1) **滚动链被两件事吃掉**: 预览态 `interactive=false` 本身不接管滚轮是对的, 但
  ①`onWheel` 里先 `preventDefault()` 再判动画锁, 锁期间"吃掉一格又没翻页";
  ②卡片预览外面还有一层 `.preview{overflow:hidden}`, 指针落在其中一侧的判定链里会吞掉滚动。
  另外旧的"指定页码"写法会渲染**受控 Deck**, 与(2)叠加后观感更糟。
- (3) **放大弹层的外观其实整体失效**: `PptCard` 复用的是
  `src/components/PptHtmlViewer/styles.module.css`(CSS Modules)。弹层走
  `ReactDOM.createPortal(..., document.body)`, 已不在卡片的 DOM 子树里,
  于是 `styles.modalShell / modalButton / modalFrame` 在弹层里全部是 `undefined` ——
  弹层没有边框、按钮塌成无尺寸方块、底页透明; 再叠加"全屏撤销后弹层不关",
  就成了"退出不明显 + 背景不对"。
  架构图自己的页内全屏 (`Diagram`) 也没考虑"祖先 transform 是 position:fixed 的包含块"。

## Decision

1. **Deck 页码以内部 state 为准, `index` 只作初始定位 + 外部改值**
   (`useState(index ?? urlIndex)` + `useEffect` 跟随 `index` 变化)。
   受控与不受控两条路径行为一致, 侧栏一定跟着走。
2. **预览/交互两种模式显式下发**: 新增 `DeckInteractiveProvider`
   (默认 `true`), 在每一屏内容外层按 `interactive` 提供。
   `Diagram` 据此降级: 预览卡片里**不渲染**缩放/全屏工具条 (要缩放就点开卡片),
   彻底消除"预览里进全屏"的隐患; 独立播放页/放大弹层仍保留全部能力。
3. **卡片外观自带一份**: `PptCard.module.css` 写全 (预览外框 + 弹层 + 下拉),
   不再 import `PptHtmlViewer` 的 module。两份样式外观保持一致, 但互不依赖。
4. **全屏状态机明确**: 第一次 `Esc` 退出浏览器全屏, 第二次关闭弹层;
   由 `onKey` 主动发起的退出用 `selfExitRef` 标记, 避免被 `fullscreenchange` 二次处理;
   外部撤销全屏 (F11 / 系统手势) 时弹层**跟随关闭**。
5. **小修**: `onWheel` 先判锁再 `preventDefault`; 遮罩关闭补 `onClick` 拦截
   (拖动出弹层不再误关); 移除 `PptCard.tsx` 从未使用的 `useLocation` 导入。

## Alternatives considered

- **继续复用 PptHtmlViewer 的 CSS module**: 用 ` :global(...)` 把 class 提升为全局 —— 否决,
  等于放弃模块隔离去迁就 Portal, 且会让两个组件的样式互相污染。
- **把弹层渲染在卡片子树内 (不用 Portal)**: 否决, 弹层需要脱离 `.preview` 的
  `overflow:hidden` 与卡片层叠上下文, 否则又被裁住。
- **预览态改成"可交互但只翻页不接管滚轮"**: 否决, 与卡片"点击即打开"的整体热区冲突,
  而且用户要的是能正常滚页面。
- **保留 `index` 受控语义、要求调用方回传 `onIndexChange`**: 否决, MDX 链接没法回传状态,
  等于把 bug 转嫁给笔记作者。

## Evidence

Playwright (chromium) 实测, 见会话记录:
- 预览态每张卡片 `.hxd-diagram__tools` 数量 = 0; 弹层内 = 2。
- `[##PPT 3##]` 卡片点侧栏后: `strip -50%`, `active "02 数据"`, `dot 3` —— 三者一致。
- 卡片上滚轮: `window.scrollY` 正常变化 (不再被吃掉)。
- 弹层: shell 1227x734 带 1px 边框、三个按钮均为 28x28 (修复前 class 为 undefined)。
- `Esc` 第一次 `{fullscreen:false, modal:false}`, 第二次 `{fullscreen:false, modal:false}`
  —— 弹层不会残留; 独立播放页 `/ppt?deck=...&page=3` 页码与 `?page=N` 同步正常。
