# .tsx 内联演示页 (平台原生形态)

**优先选这个**. 与 `.html` 侧车的区别不是"风格", 而是渲染方式:

| | `.tsx` 内联演示页 | `.html` 侧车 |
|---|---|---|
| 渲染 | React 内联, **融入站点主题** | iframe 独立文档 |
| 换主题 / 跟随明暗 | ✅ 实时切换 | ❌ 自带样式, 不跟随 |
| 引用语法 | `[标题 ##PPT##](x.tsx)` | `[标题 #ppt](x.html)` |
| 注册 | 需 `npm run decks` 生成注册表 | 复制到路由即可 (插件自动) |
| 适用 | **默认选择** | 已存在的外部 HTML / 需要完全自包含单文件时 |

## 怎么建

1. 在笔记同目录建 `<名字>-deck.tsx`, **导出 `slides()`**:

   ```tsx
   import { Slide } from '@site/src/hxdeck/Slide';
   import { Cover, Card } from '@site/src/hxdeck/blocks';
   import { PageHeader, Bullets, Split, Steps, KeyValues, Quote } from '@site/src/hxdeck/ui';
   import { CompareTable, Timeline, Meter, Gauge, CodeDiff } from '@site/src/hxdeck/ui-more';
   import { Stat, BarChartBlock, PieChartBlock, Tree } from '@site/src/hxdeck/charts';
   import { CodeBlock } from '@site/src/hxdeck/code';
   import { Diagram } from '@site/src/hxdeck/diagram';

   export const slides = (): React.ReactNode => (
       <>
           <Slide title="封面" chapter=""><Cover eyebrow="EYEBROW" title="标题" subtitle="副题" /></Slide>
           <Slide title="某页" chapter="01 章节"><PageHeader eyebrow="E" title="T" desc="D" /></Slide>
       </>
   );
   export default function MyDeck() { return <>{slides()}</>; }
   ```

2. `chapter` 相同的屏在左侧栏归为一组; `title` 是左侧栏标题. 导航由 Slide props 自动生成, 不用维护第二份清单.
3. `npm run decks` (build 前会自动执行) 扫描写有 `slides` 的 `.tsx` 并生成注册表.
4. 正文写 `[标题 ##PPT##](名字-deck.tsx)`; 可带页码与主题: `[标题 ##PPT 3 whale##](x.tsx)`.

**不要**导出成只含组件的写法 —— Deck 需要一份铺平的 `<Slide>` 列表来数屏数, 只给组件会导致导航与翻页不渲染.

## 图: 复用 archify, 不要重画

`Diagram` 吃 archify 抽出的 SVG 模块, 配色全走主题变量, 在图内仍是可点/可缩放的:

```bash
node .agents/skills/hx-archify/scripts/extract-diagram.mjs <archify产物.html> src/hxdeck/figures/<名字>.ts
```

然后在 deck 里 `<Diagram asset={myFig} kind="dataflow" pad="sm" caption="..." />`.
**`.html` 侧车与 `.tsx` 演示页可以复用同一张图**, 不必做两遍.

## 让它像 PPT, 而不是像一篇文章

内联演示页最容易做成的失败形态, 是"把正文段落搬进卡片": 每屏只有标题加一段字,
缩到 0.4~0.7 倍投屏之后一片糊. 三条硬要求:

1. **一屏只讲一个判断, 且必须有视觉宾语.**
   文字页最多两栏; 三栏以上就该是 `Columns` / `CompareTable` / `Stat` 排;
   纯文字超过 5 行就该换成表、步骤或图.
2. **每屏至少一个"非文字"元素**: `Diagram` / `BarChartBlock` / `PieChartBlock` /
   `Gauge` / `Meter` / `Timeline` / `CompareTable` / `CodeBlock` / `Stat` 行.
   全是 `Bullets` + `Callout` 的屏, 读起来就是笔记而不是演示.
3. **刻意造大小反差**: `Stat` 的数字 (40px+)、`PageHeader` 标题与正文之间要有量级差,
   不要全屏都是 15~21px 的中等字号.

## 投放前有两件事必须实测, 别靠眼看

画布是固定的 1600x900, 每屏内边距 44/56/60/56, **内容可用高度只有 796px**.
"看起来没几行"的屏照样可能超出去, 而超出部分会被 `overflow: hidden` 静默裁掉.

```bash
# 用 CDP 连 headless chromium, 量每屏真实占用 (含缩放归一化)
chromium --headless=new --no-sandbox --disable-gpu \
  --remote-debugging-port=9333 --window-size=1600,900 about:blank &
```

```js
// 判定: 任一屏 bottom > 840 (或 right > 1600) 就是溢出
// 注意要把子元素坐标除以 stage scale, 否则量到的是缩放后的像素
const s = new DOMMatrixReadOnly(getComputedStyle(el).transform).a;
```

量到溢出后, 按这个顺序改, 不要直接删内容:

1. 去掉图表卡片上的 `hxd-fill` —— `flex: 1 1 0` 会让卡片撑满**剩余**高度,
   而内容高度是固定的, 于是溢出; 给图表设固定 `height` 更可控.
2. 压图高 (`BarChartBlock height`), 或把图从"两栏之一"移到独立通栏并缩高.
3. 删掉与图表重复的 `Callout` —— 图上已经说清的数字不必再用文字复述一遍.
4. 最后才动字号; 字号一旦小于 13px, 投屏基本不可读.

同时核对**最小字号**: 正文类不该低于 17px (画布坐标), 图注/表头不该低于 13px.
同一张屏里字号种类不要超过 4 种, 否则视觉层级会散.

## 常见坑

- 属性里嵌引号: `desc="...要不要知道"当时什么是真的"."` 会编译失败, 用 `「」` 或 `&quot;`.
- 忘记 `npm run decks`: 页面会渲染成"未注册的演示页", 不报错.
- `.tsx` 是**源码**, 与笔记一起版本管理; 它不进 iframe, 所以不存在"自包含单文件"约束.
- `Diagram` 的 `pad="sm"` 内边距每 10px 都从图上扣宽度、图内文字随之等比缩小; 演示页里更该让图占满.
- 图上文字如果读不清, **不要**给 SVG 加 `zoom` 放大整图 —— 那会把节点推出可视区, 点不中
  (`Diagram` 里"初始缩放固定 100%"就是为此). 改用 ui.css 里的字号下限规则, 并按需补 `font-size` 档位.
