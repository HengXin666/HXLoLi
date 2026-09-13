# Agent Note: 演示页滚轮会"漏"给宿主页面, 导致整个 PPT 整体上滑

Status: implemented

- 影响: `src/hxdeck/Deck.tsx` / `src/css/custom.css` / `src/pages/ppt.tsx` / `src/hxdeck/charts.tsx`
- 现象: 约 10 屏以上的演示页, 用滚轮从第 1 屏往下翻, **偶尔**整块 PPT 内容一起向下滚, 显示不全

## Problem

用户报告: "感觉是在某一个触发状态的时候, 它那个判定消失了, 就变成了真正的那种滚动条的滚动模式导致其下滑".

关键词是**偶尔**与**某一个触发状态** —— 不是几何算错(那样每次都错), 而是某条分支上少了东西.

## Root cause

一共两个缺陷叠加, 主因是第一个.

### 1. `preventDefault()` 写在了动画锁判断之后 (主因)

`Deck.tsx` 的 `onWheel` 原本是:

```ts
if (!interactiveRef.current) return;
if (isWheelConsumed(e)) return;
if (inScrollable(e.target)) return;
if (!visibleEnough()) return;
if (lockedRef.current) return;   // ← 动画进行中
e.preventDefault();              // ← 到不了这里
```

翻页动画要跑 `PAGE_DURATION + LOCK_TAIL = 950ms`, 这期间 `lockedRef.current === true`.
于是这 950ms 里每一次滚轮都**既不翻页、也不 `preventDefault`** —— 默认行为原样落到宿主页面上.
连续滚动时 (触控板惯性一次几十个事件) 命中这个窗口的概率很高, 所以表现为"偶尔".

判定并没有消失, 是这一支上根本没写 `preventDefault`. `LOCK_TAIL` 的注释本就写着这是
"吸收惯性滚轮"期, 而**吸收的正确做法恰恰是 `preventDefault`**, 不是放行.

### 2. `inScrollable()` 只判断"能不能滚", 不看"还能不能往这个方向滚"

```ts
if (/(auto|scroll)/.test(cs.overflowY) && n.scrollHeight > n.clientHeight + 2) return true;
```

侧栏 `.hxd-nav__list` (`max-height: 524px`) 一旦内容超长就恒为 true.
它滚到底之后, 滚轮仍然被让给浏览器 → 滚动链传给宿主页面. 仓库里其实早就写了
`canScrollFurther(el, deltaY)` 这个带方向判断的函数, 但**从未被任何地方引用**(死代码).

### 3. 独立播放页 `/ppt` 根本没有样式 (放大器)

`ppt.tsx` 一直在用 `.hxppt` / `.hxppt__bar` / `.hxppt__body`, 但 `src/` 下**一条 `.hxppt` 规则都不存在**.
后果是页面按 Docusaurus 文档流排版: deck 保持 16:9 + 被导航栏顶下去,
`documentElement.scrollHeight` 实测 926 而视口 757 —— 存在一条**本不该有**的页面滚动条.
上面两个缺陷漏出来的滚轮, 正是落在这条滚动上.

## Evidence

无头 Chromium (CDP) 在弹层里注入 window 阶段的 wheel 探针, 连滚 10 次:

| | 未拦截次数 |
| --- | --- |
| 修复前 | **8 / 10** |
| 修复后 | **0 / 10** |

隔离验证 (证明"漏出来 → 真的会滚走页面"): 手动把 `.hxd-deck` 的
`overscroll-behavior` 从 `contain` 改成 `auto` 以拆掉兜底, 再滚 14 次:

- 修复前: `scrollY 0 → 169`, `deckTop 26 → -143` —— 内容整体上移, 与用户描述一致.
- 修复后: `scrollY 0`, `deckTop 26` —— 兜底拆掉也不再漏.

另: 弹层在本 bug 下"看起来正常", 只是因为 `PptCard` 把 `body.overflow` 设成了 `hidden`
兜住了 —— 这解释了为什么它在笔记页里偶发、而不是必现.

## Fix

1. `Deck.tsx`: 把 `e.preventDefault()` **提到** `lockedRef` 判断之前.
   语义改为"走到这里这次滚轮就归 deck 所有, 先无条件吃掉默认行为, 再决定要不要翻页".
2. `Deck.tsx`: `inScrollable` → `scrollableUnder(target, deltaY)`, 用上 `canScrollFurther`
   做方向判断, 返回真正还能滚的那个元素; 滚到边界就把控制权交回 deck.
3. `Deck.tsx`: `visibleEnough()` 里的 `getBoundingClientRect()` 改为**缓存**(400ms 过期).
   wheel 是最热的事件, 每事件一次强制同步布局会把主线程拖住; 这与"判定偶尔失效"同源.
4. `custom.css`: 补上缺失的 `.hxppt*` 样式, 播放页改 `position: fixed; inset: 0`,
   彻底不产生页面级滚动条; `ppt.tsx` 的 `<Deck>` 加 `fill`.
5. `charts.tsx`: 图表轴标签 12px → 13px, 与全局文字下限一致.

## Consequences

- `preventDefault()` 提到锁判断之前: 滚轮事件一旦进入 deck 就归它所有, 再决定是否翻页。
  代价是 deck 区域内必须自己把"该滚谁"判全, 判漏就会吃掉页面滚动 —— 这正是本条曾经的症状。
- `visibleEnough()` 的 `getBoundingClientRect()` 改为 400ms 缓存。wheel 是最热的事件,
  代价是判定有一个最多 400ms 的陈旧窗口, 换来主线程不被强制同步布局拖住。
- 播放页改 `position: fixed; inset: 0`, 不再产生页面级滚动条; 预览卡片保持"在卡片上滚, 页面正常滚"。
- 图表轴标签 12px → 13px, 与全局文字下限一致。

- 弹层 10 次滚轮: 未拦截 0 次; 翻页仍正常 (page 1 → 3).
- 拆掉 `overscroll-behavior` 兜底后连滚 14 次: `scrollY` 恒为 0.
- 预览卡片 (非交互态) 未被误伤: 在卡片上滚, 页面 `scrollY 0 → 1000`, 正常滚动.
- 弹层侧栏可正常滚动 (`scrollTop 0 → 407`), 滚到底后 `scrollY` 仍为 0 (边界交接正确).
- 独立播放页: `docScrollHeight 757 === innerHeight 757`, 无滚动条;
  deck 尺寸 1600x719, 缩放 **0.797** (修复前 0.45, 画面大了近一倍).
- 逐屏 (page=1..17) 复测: 无溢出 (阈值 12px).
- `npx tsc --noEmit`: `src/hxdeck/` 下 0 错误; `npm run build` SUCCESS.

## Alternatives considered

- **只加重试/防抖**: 治不了根 —— 漏的是默认行为, 不是频率问题.
- **干脆全程 `preventDefault`**: 会让预览态也无法滚动页面, 破坏"卡片就该让页面滚"的既有约定
  (deck.css 顶部那段注释专门讲过这个坑).
- **只在动画锁期间加 CSS `overscroll-behavior: contain`**: CSS 挡不住已经被浏览器接管的滚动,
  且在非交互态会重新引入"在卡片上滚不动页面"的老 bug.
