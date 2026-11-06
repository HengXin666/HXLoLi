# HX おすすめ Glass · 番剧类页面设计语言

susume / 系列 / kyara 等番剧类页面共用的一套视觉与动效. 博客本体是 "粉色 + 猫娘 + 日系圆体", 这几页在此之上做成现代杂志 / 便当格 (bento) 式展示页, 并让猫娘和おすすめ语汇明确出现.

参考: Linear / Vercel 博客的 bento 与光晕, Josh Comeau 博客的弹簧微交互, Pixiv / Bangumi 的画廊密度.

决策 note: `.agents/notes/implemented/feature/2026-11-06-anime-ui-kit.md`

## 1. 令牌

已有 `--hx-*` (accent / gold / 三个 susume 色 / surface / border / text / radius / shadow / ease / dur / `--hx-font-round`) 不变. 本次只在 `src/css/custom.css` 两个 `--hx-*` 块末尾追加:

| 令牌 | 亮 | 暗 | 用途 |
|---|---|---|---|
| `--hx-container` | `1200px` | 同 | 番剧类页面内容最大宽 |
| `--hx-gutter` | `clamp(16px, 3vw, 28px)` | 同 | 栅格间距, 页面左右留白 |
| `--hx-glass-blur` | `blur(14px) saturate(140%)` | 同 | 玻璃面 `backdrop-filter` |
| `--hx-mesh` | 粉 / 金 / 蓝三点 radial 渐变, 透明度 ≤ .18 | ≤ .12 | hero 背景光晕 (`background` 图层) |
| `--hx-dots` | 1px 圆点 22px 网格, 颜色 `--hx-border` | 同 | 页面底纹, 已带 `0 0 / 22px 22px`, 只能放在 `background` 简写里当一层 |
| `--hx-focus` | `0 0 0 3px var(--hx-accent-ring)` | 同 | 统一 focus-visible (`box-shadow`) |
| `--hx-lift` | `translateY(-4px)` | 同 | 卡片悬浮位移 (`transform`) |
| `--hx-stagger` | `40ms` | 同 | 列表入场间隔 (CSS 侧; JS 侧同值 `0.04s`) |
| `--hx-measure` | `68ch` | 同 | 正文行宽, **只限制内层文字, 不限制卡片** |

规则:
- 组件的 CSS 里不写十六进制和 `rgba(`, 颜色全部取令牌. 派生色用 `color-mix(in srgb, <accent> X%, transparent | var(--hx-text) | var(--hx-border))`.
- 页面强调色写在 `--susume-accent` / `--kyara-accent` 上; 原子组件各自暴露 `--hx-hero-accent` / `--hx-pill-accent` / `--hx-card-accent`, 通过 `accent` prop 设置.

```css
.page {
    max-width: var(--hx-container);
    margin: 0 auto;
    padding: 22px var(--hx-gutter) 80px;
    background: var(--hx-dots); /* 可选底纹 */
}
.myGlass { background: var(--hx-surface-glass); backdrop-filter: var(--hx-glass-blur); }
.myLink:focus-visible { outline: none; box-shadow: var(--hx-focus); }
```

## 2. 版式

- 外壳: `max-width: var(--hx-container)`, 左右 `--hx-gutter`.
- 断点: ≥1280 (两侧猫娘轨道) / 997–1279 / 641–996 / ≤640. 996 跟 Docusaurus 移动端断点对齐. 每个页面的 module.css 都要有 `@media (max-width: 996px)` 和 `@media (max-width: 640px)`.
- 结构固定: 面包屑 → `HxHero` → `HxPills` → 主体 (bento / 网格) → 其余.
- 字体: 标题 `--hx-font-round` 700, `clamp(28px, 4vw, 44px)`; kicker 12px, 字距 .18em, 日文 + 罗马音 (例 `HXLoLi のおすすめ · 01 / 03`); 数字 `font-variant-numeric: tabular-nums`.
- 卡片: `--hx-radius-lg`, `--hx-surface-1`, 1px `--hx-border`.

## 3. 动效 (framer-motion, 参数单一来源在 `index.tsx`)

| 场景 | 参数 |
|---|---|
| 入场 | `opacity 0→1, y 16→0`, `0.55s`, `hxEase = [0.22, 1, 0.36, 1]`, `whileInView`, `once: true`, `amount: .2` |
| 列表 | 每一项自己观察视口, 延迟 `(序号 % 8) × 40ms` (长列表整体 20% 进视口可能永远达不到, 所以不用父级 stagger) |
| 卡片悬浮 | `--hx-lift` + `--hx-shadow-md`, `--hx-dur-med` (0.32s); 封面 `scale(1.04)` 由各页自己写 |
| 气泡 / chip / pill 滑块 | spring `stiffness 420, damping 26` |
| Hero 视差 | 光晕随滚动最多偏移 24px (`useScroll` + `useTransform`) |
| 猫娘 | 待机上下浮 6px (4.8s); `lean` 时往外歪头 6° + 外移 6px (spring) |
| 切换 | `HxPills` 选中底块 `layoutId` 滑动 (每个实例独立 id) |

`prefers-reduced-motion: reduce`: `useHxReducedMotion()` 为 true 时所有组件直接渲染终态, 不视差, 不 lean, 骨架不闪; CSS 侧也有同名媒体查询兜底.

## 4. 状态

三态统一: `HxSkeleton` (shimmer, 尺寸跟真实卡一致) / `HxState kind="error"` (标题 + 原因 + 「重试」) / `HxState kind="empty"` (猫娘小图 + 说明怎么填数据). 文案中文, 口吻跟 susume 一致.

## 5. 组件 (接口冻结, 只引用不修改)

```ts
import {
    useHxReducedMotion, hxEase, Reveal, RevealList, HxHero, HxPills, HxCard,
    HxSectionHead, HeartRate, BgmScore, HxSkeleton, HxState, NekoPeek,
} from "@site/src/components/anime/ui";
```

| 组件 | 说明 |
|---|---|
| `useHxReducedMotion()` | 是否减少动效; SSR / 首帧为 false |
| `hxEase` | `[0.22, 1, 0.36, 1]`, 自写 framer 动画时用 |
| `Reveal` | 单个元素进视口淡入上移. `as` 指定标签 |
| `RevealList` | 子项依次入场. **每个子节点自动包进 `<li>` (ol/ul) 或 `<div>`**, 调用方只传内容; 网格类写在 `className` 上, 网格项是 `li` |
| `HxHero` | kicker + 标题 + 副标 + 计数 + 台词气泡 + `children` 槽 (放开关 / 按钮). `neko` 默认 `"right"`, 猫娘贴 hero 边探进来; `false` 不显示 |
| `HxPills` | 有 `href` 渲染 `Link` + `aria-current`, 否则 `button` + `aria-pressed`. `≤640` 横向滚动 |
| `HxCard` | 玻璃卡片; `interactive` 时悬浮 lift |
| `HxSectionHead` | 区块标题 + 计数 + 右侧操作, `id` 给锚点 |
| `HeartRate` / `BgmScore` | 我的评分 (粉心, 整数) / Bangumi 评分 (金星, 一位小数) |
| `HxSkeleton` | `display: contents`, 骨架块直接落进调用方的网格; `shape`: cover 2:3 / card / tile 1:1 / line |
| `HxState` | 错误 (`role="alert"`) / 空态 |
| `NekoPeek` | 两侧探头猫娘轨道, 放进 `position: relative` 的舞台; `sticky` 随滚动; `<1280px` 自动隐藏, 由 hero 那一只代替 |

### 示例

```tsx
<Layout title="おすすめ">
    <main className={styles.page}>
        <nav className={styles.crumbs}>…</nav>
        <div className={styles.stage}>
            <NekoPeek side="left" sticky lean={lean === "left"} />
            <NekoPeek side="right" sticky lean={lean === "right"} />
            <HxHero
                kicker="HXLoLi のおすすめ · 01 / 03"
                title="神作推荐"
                sub="站长亲手递给你的番"
                count="12 部"
                say="这几部是心头好, 真的推荐你看!"
                accent="var(--hx-susume-monosugoi)"
            />
            <HxPills
                aria-label="推荐分类"
                current={key}
                items={defs.map((d) => ({ key: d.key, label: d.display, hint: d.count, href: d.path, accent: d.color }))}
            />
            <HxSectionHead title="书架" count={`${rest.length} 部`} id="shelf" />
            {status === "loading" ? (
                <div className={styles.shelf}><HxSkeleton shape="cover" count={12} /></div>
            ) : status === "error" ? (
                <HxState kind="error" title="番剧记录没拉下来" body={error} onRetry={retry} />
            ) : list.length === 0 ? (
                <HxState kind="empty" title="这里还空着" body="在标注文件里给番加上这个推荐标签就会出现在这里." />
            ) : (
                <RevealList as="ol" className={styles.shelf} aria-label="书架">
                    {rest.map((r) => (
                        <HxCard key={r.id} interactive accent="var(--susume-accent)">
                            …<HeartRate value={8} /> <BgmScore value={7.84} />
                        </HxCard>
                    ))}
                </RevealList>
            )}
        </div>
    </main>
</Layout>
```
