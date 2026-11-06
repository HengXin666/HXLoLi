# Agent Note: 番剧类页面共用设计语言「HX おすすめ Glass」与原子组件

Status: implemented

Decision-ID: anime-ui-kit

## Code

- `src/components/anime/ui/index.tsx`

## Problem

1. susume 专属页、系列页、即将新增的 kyara 角色配图页要一起重做成现代 bento / 杂志式展示页. 如果每一路各自写 hero、分类切换、卡片悬浮、骨架和空态, 动效参数和断点会再一次各写各的, 视觉上又不连贯, 也无法并行开工.
2. 现有 `--hx-*` 令牌只有颜色 / 圆角 / 阴影 / 曲线, 缺版式 (内容宽、栅格、行宽) 与动效 (悬浮位移、入场间隔、统一 focus) 的令牌.
3. 动效只有零散的 CSS transition, 没有统一的入场、视差、共享布局滑块, 也没有统一处理 `prefers-reduced-motion`.

## Decision

1. `src/css/custom.css` 只在两个 `--hx-*` 块末尾追加令牌: `--hx-container` / `--hx-gutter` / `--hx-glass-blur` / `--hx-mesh` (亮 ≤ .18, 暗 ≤ .12) / `--hx-dots` (带 22px 尺寸的 background 图层) / `--hx-focus` / `--hx-lift` / `--hx-stagger` / `--hx-measure`. 已有行不动.
2. 新目录 `src/components/anime/ui/`: `index.tsx` 导出冻结接口 `useHxReducedMotion`, `hxEase`, `Reveal`, `RevealList`, `HxHero`, `HxPills`, `HxCard`, `HxSectionHead`, `HeartRate`, `BgmScore`, `HxSkeleton`, `HxState`, `NekoPeek`; `ui.module.css` 只取令牌, 派生色只用 `color-mix`; `DESIGN.md` 写令牌表、动效参数和示例.
3. 动效统一用已安装的 `framer-motion`: 入场 0.55s `[0.22,1,0.36,1]` 进视口一次; 列表每项自己观察视口, 延迟 `(序号 % 8) × 40ms` (父级 stagger 在长列表上 20% 阈值可能永远达不到); Hero 光晕视差最多 24px; pill 选中底块 `layoutId` (每个实例用 `useId` 隔离); 气泡与猫娘 lean 用 spring 420 / 26.
4. 每个带动效的组件先读 `useHxReducedMotion()`, 为 true 时直接渲染终态, 不视差不 lean, 骨架不闪; CSS 里另有 `prefers-reduced-motion` 兜底.
5. 猫娘: `HxHero` 内的一只贴 hero 边探进来 (右侧用切口在右的 `neko_left.png`); `NekoPeek` 是舞台外两侧的 sticky 轨道, 几何沿用 susume 页 (切口压进舞台 6%), 不足 1280px 隐藏, 窄屏只留 hero 那一只.
6. `Reveal` 的 `as` 类型按冻结签名写成 `keyof JSX.IntrinsicElements`; React 19 类型已去掉全局 `JSX`, 所以从 `react` 显式 `import type { JSX }`.

## Alternatives considered

- 什么都不做, 各路沿用 `Susume.module.css` 现有写法: 最强理由是 susume 页刚重做过、令牌已有, 零新代码. 否决: 系列页和 kyara 页无处复用 (类名绑死在 susume 模块里), 三路并行时动效和断点必然分叉, 用户要求的就是统一的现代设计语言.
- 复用现有 `Susume.module.css` / `PosterCard` 直接导出给其他页: 代码最少. 否决: 它们属于路 1 的写入范围且正在重构, 把共享接口挂在会被重写的文件上会让其他路被阻塞; 也没有动效组件.
- 纯 CSS 动效 (transition + `@keyframes`), 不用 framer-motion: 不增运行时成本. 否决: 视口入场、滚动视差、`layoutId` 共享布局滑块和 kyara 灯箱放大用 CSS 都要手写 IntersectionObserver / FLIP; `framer-motion` 已是依赖 (`cv.tsx`, `BlogArchivePage` 在用), 不新增包.
- 引入 Tailwind 组件或第三方 UI 库: 现成. 否决: 视觉语汇 (粉色 / 猫娘 / 圆体) 对不上, 且要再维护一套主题.

## Consequences

`src/css/custom.css` 与 `src/components/anime/ui/ui.module.css`, `src/components/anime/ui/DESIGN.md` 是非 AST 资源, 按红线走 resource-review; 后两者是本决策的同目录兄弟, 由 `index.tsx` 代表. 导出签名冻结, 路 1 (susume) / 路 2 (系列简介) / 路 4 (kyara 页面) 只引用不修改; 要改签名必须回到本 note 重新决策并通知各路. `RevealList` 会把每个子节点包进 `li` / `div`, 调用方不要再自己包. `HxSkeleton` 是 `display: contents`, 骨架块直接落进调用方的网格. 本路不启动博客, 实际效果由用户在 `localhost:3000` 上看.
