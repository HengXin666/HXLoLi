# Agent Note: 架构图放大: 边界错成"整个网页", 内部布局大于父布局

Status: implemented

- 影响: `src/hxdeck/{diagram,Deck,PptCard}.tsx` / `src/hxdeck/{deck-layer.tsx,ui.css}`

## Problem

在笔记里打开 PPT 卡片自带的放大弹层, 再点架构图的"放大"按钮:

1. **它铺满的是整个网页, 不是当前演示页** —— 站点导航栏、正文、其它卡片全被盖住,
   看起来像"整页变成了演示页".
2. **退出路径不明显** —— 浮层盖在弹层工具栏之上, 连"关闭/退出"按钮都点不到;
   底下的演示页侧栏/进度点还透过浮层边缘露出来.
3. **内部布局比父布局还大** —— 弹层里的演示页只有 782px 高, 放大后的浮层却按
   900px 视口排版, 内容被顶出父边界 (用户原话: "内部的布局怎么能放大, 还大于父布局呢?").
4. **宿主在浏览器全屏里时, 点"放大"什么都看不见** —— 全屏元素处于 top layer,
   `document.body` 上的浮层被压在它下面.

## Root cause

`Diagram` 的放大浮层用 `createPortal(inner, document.body)` + `position: fixed; inset: 0`,
再叠加 "全屏高度完全交给 CSS" / 早期版本按 `window.innerHeight` 算 `frameH`:

- `fixed` 的包含块是**视口**, 不是演示页 —— 所以它天然就是"整个网页".
  而卡片预览里演示页被固定在 16:9 (含舞台 `scale()`), 尺寸远小于视口.
- 浮层脱离 `.hxd-deck` 子树后, `--hxd-color-bg` 等 CSS 变量解析不出值,
  `background: var(--hxd-color-bg)` 失效 -> 透明 -> 后面的正文透出来.
- 全屏元素在 top layer, 与 `document.body` 的子节点没有可比性 —— 浮层必然被压住.
- `ui.css` 里的 `body:has(.hxd-diagram[data-full])` 隐藏规则同时失效 (浮层不在 deck 里).

另外还有一个被这次暴露出来的兄弟 bug: `PptCard` 的 `fullscreenchange` 用
`document.fullscreenElement === shellRef.current` 判断"是否被外部撤销". 弹层进入浏览器全屏后,
**再点架构图放大**时 `fullscreenElement` 仍等于 shell (未变), 但一旦有内层元素请求全屏
(iframe 场景), 该等式为假, 弹层会被误判为"全屏被撤销"而直接关掉.

## Decision

1. **新增层端口 `deck-layer.tsx`**: `{ host, setLayerFull }`. `Deck` 把**自己的根节点**
   用回调 ref 提升成 state 后下发; `Diagram` 收到后 `createPortal(inner, host)`.
2. **放大浮层改用 `position: absolute; inset: 0`**, 包含块 = 最近的非 static 祖先 = 演示页根节点.
   边界因此严格等于父布局, "内部大于父布局"在结构上不可能发生.
   没有 Deck 宿主时 (独立使用) 退回 `[data-layer='viewport']` + `fixed` + `100vw/100vh`.
3. **高度不再由 JS 按视口算**: 放大态 `setFrameH(null)`, 改由 CSS `flex: 1 1 auto` 撑满父容器.
4. **`z-index: 60`** 高于 deck 内所有装饰 (最高 7); deck 装饰的隐藏改由
   `.hxd-deck[data-layer-full='true']` 选择, 不依赖 `:has()`, 也不会误伤同页其它卡片.
5. **键盘升级为模态**: 放大期间用**捕获阶段** `keydown` + `stopPropagation`
   抢在 Deck 的翻页监听与 `PptCard` 的 Esc 处理之前 —— 一次 Esc 只退放大, 不连退两层.
6. **指针事件隔离**: 放大态在 `<figure>` 与 `__frame` 上拦 `mousedown/click`,
   避免冒泡到 `modalOverlay` 的"点遮罩关闭"把整个弹层关掉.
7. **FLIP 补间修正两处**: 平移量除以父节点自身的 scale (父节点带 `scale()` 时
   局部 translate 会被放大); 清理计时器改回各自的句柄 (原先误写成 `clearTimeout(timers.current.length)`).
8. **`PptCard` 的全屏撤销判据改为 `!document.fullscreenElement`**: 只有"没有任何元素在
   全屏"才算撤销; 内层元素 (iframe/演示页自己) 请求全屏不再误关弹层.

## Alternatives considered

- **继续 Portal 到 `document.body`, 只把 `fixed` 定位改成"手动算成演示页矩形"**:
  否决. 要在 resize / 翻页 / 弹层缩放时持续跟踪, 任一时刻算错就是又一次"错的全屏";
  而 `absolute` + 原生包含块把这件事交给浏览器, 没有可错的时序.
- **Portal 到`modalFrame` / 卡片外框 (弹层的舞台容器)**:
  否决. 那会让放大只能局限在弹层工具栏下方那块区域, 而且预览卡片里没有 `modalFrame`.
  挂演示页根节点对预览/弹层/独立播放页/浏览器全屏**四种宿主一视同仁**.
- **用 `position: fixed` + `:has()` 把祖先 transform 置 none**:
  否决 (旧注释已记录). `.hxd-strip` 带 0.86s transform 过渡, 强制置 none 会触发一次
  "回到第 1 页"的动画; 而且 `:has()` 在旧浏览器支持面窄.
- **保留 `window.innerHeight` 计算, 只是夹到父容器高度**:
  否决. 那只是把症状盖住 —— 修复前实测 frameH=708 已经大于可用高度,
  真正的病根是"拿视口当演示页".

## Consequences

- 放大的边界由浏览器按包含块算, 不再有"算错视口"的时序可错; 代价是放大浮层必须挂在演示页根节点下,
  独立使用时退回 `fixed` + `100vw/100vh` 这条退化路径。
- `deck-layer.tsx` 成为一个新的层端口: `Deck` 之外的宿主若要支持放大, 必须自己提供这个端口。
- 放大期间键盘改为捕获阶段模态, 一次 `Esc` 只退一层 —— 代价是 Deck 的翻页监听在放大态被完全压制,
  放大态下不能用方向键翻页。
- 节点卡片不再放动作按钮 (分享菜单已提供同一能力), 卡片只剩关闭与上下游跳转。

Playwright (chromium, 1440x900) 实测:

| 场景 | 浮层 rect | 父 (deck) rect | 越界 | 退出按钮可点 |
| --- | --- | --- | --- | --- |
| 弹层内放大 | 1378x780 @ (31,82) | 1380x782 @ (30,81) | 否 | 是 |
| 弹层 + 浏览器全屏 | 1438x854 @ (1,45) | 1440x856 @ (0,44) | 否 | 是 |
| 独立播放页 `/ppt?page=3` | 1438x808 | 1440x810 | 否 | 是 |

- 修复前同一场景: 浮层 `position: fixed` 铺 1440x900 视口, 父 deck 仅 1382x828,
  `document.elementFromPoint` 命中的是 `hxd-card` (浮层被全屏元素压住); 关闭按钮命中 `hxd-diagram__frame`.
- 视口 1440x900 / 1024x700 / 390x844 三档: `fits === true`, 侧栏/进度点/翻页按钮 `opacity: 0`.
- `Esc` 一次: 浮层数 1 -> 0, 弹层与浏览器全屏都保留; `Esc` 两次: 弹层关闭, `body.overflow` 复原.
- 放大期间按 `↓` / 滚轮: 当前页标题保持 "架构图" (不再被 Deck 翻页).
- FLIP 首帧: 浮层从预览图原位置 (1064x425 @ 305,291) 起, ~320ms 后落在 (31,82) 1378x780.
- 预览卡片内工具条按钮数 = 0 (预览态仍按设计不提供放大入口).
- 老 `#ppt` iframe 卡片: 进入浏览器全屏后 `modal === true && fs === true` (内层全屏不再关掉弹层).

## 后续调整 (同日, 按用户要求)

**工具条 (缩放/复位/放大/导出) 从图上方的独立一行, 改为浮在图表内部的右下角.**

- 结构: 新增 `.hxd-diagram__stage` (position: relative) 作为共同定位上下文,
  图框与工具条成为**兄弟节点**. 工具条不能再放进 `__frame` 里 ——
  图框是 `overflow: auto` 的滚动画布, 放里面会跟着缩放后的画布一起滚走
  (放大后按钮"跑出屏幕", 正是"退出路径不明显"的复现).
- 样式: 半透明胶囊 (`color-mix` + `backdrop-filter`), 默认 `opacity: .72`,
  hover/focus-within 变 1; `[data-full='true']` 恒为 1 (放大态它是唯一的退出入口).
- 导出菜单改为**向上**弹出 (`bottom: calc(100% + 6px)`): 工具条已在图的底部, 向下开会顶出图外.
- 滚轮监听从 `__frame` 移到 `__stage`: 否则鼠标停在工具条上滚轮会漏给 Deck 翻页.
- 放大态用到的 `__frame` 高度链同步改成 `浮层 -> __stage -> __frame` (`flex: 1 1 auto`).

实测: 未放大工具条 `[1222,627,139,29]` 落在图框 `[305,291,1064,373]` 内;
放大后 `[972,761,405,40]` 落在图框 `[51,98,1338,715]` 内, 退出按钮 `elementFromPoint` 命中自身;
导出菜单 `[1146,613,90,148]` 向上开且未出图; 悬停工具条滚轮 `187% -> 202%`, 演示页仍停在 "架构图".

## 再调整 (同日, 按用户要求): 对齐 archify 的读者交互

### 1. 删掉 "滚轮缩放 · 拖拽平移" 文案

`.hxd-diagram__hint` 元素与规则一并移除 —— 操作提示属于"说明书", 不该常驻占用图面.

### 2. 导出图标换成分享图标, 并支持复制"这张图在本站的链接"

- 旧图标 `⤓` 既不像导出也不像分享, 且读者真正想要的是**链接**而不是离线文件.
  换成 react-icons 的 `FaShareAlt`; 成功后短暂变 `FaCheck`.
- 菜单第一项 = **复制链接**: 取 `location.href`, 若正聚焦某节点则附上 `#focus=<id>`.
  剪贴板不可用时退回 `textarea + execCommand` (http / 内嵌 iframe 场景).
- 下载格式 (PNG/JPEG/WebP/SVG) 归到 "下载图片" 分组, 仍然只在放大态出现.

### 3. 套用 archify viewer 的交互 (skill: `.agents/skills/hx-archify`)

archify 产物里每个节点/边都带稳定语义钩子, 交互全部建立在其上. 这次把这套机制接进 React:

- **新增 `diagram-focus.tsx`**: `parseFacts` (解析 `data-node-id` / `data-node-label` /
  `data-node-sublabel` / `data-node-context` / `data-node-kind` / `data-edge-from|to|label|key`),
  `neighborhood` (取直接邻域), `focusSvg` (生成聚焦版 SVG 文本),
  `DiagramPassport` (语义护照卡片).
- **点节点 = 聚焦**: 高亮它 + 直接相连的边 + 邻居, 其余降到 `opacity: .13`.
  **再点同一节点 / 点空白 = 取消**. 与 archify `set()` 的 toggle 语义一致.
- **高亮不走 JS**: 只输出 archify 自己的 `data-focus-active` / `data-focus-match` /
  `data-focus-selected` 协议, 视觉规则由 SVG 里那段语义 CSS 负责 —— 与原始产物同一套语言.
- **语义护照**: 卡片贴着被点节点显示, 列 label / sublabel / kind / context / 稳定 ID /
  上游 / 下游; 点上下游条目可直接跳到那个节点; 有 `复制链接` 与关闭按钮.
- **键盘**: 节点本来就带 `tabindex="0" role="button"`, Enter/Space 可聚焦 (并 `stopPropagation`,
  免得被 Deck 当成翻页); `Esc` **分层退出** —— 先收卡片, 再退放大, 与 archify 一致.
- **深链**: 聚焦状态写进 URL `#focus=<id>` (用 `replaceState`, 不塞满回退键), 打开即复原.
- **只在可交互的图上启用** (`canZoom`): 预览卡片是静态缩略图且整卡是"点击打开"热区,
  在那里抢点击会让读者点不开卡片 —— 与缩放/放大的降级规则一致.

### 踩到的两个真坑

1. **SVG 走 `dangerouslySetInnerHTML`, React 每次重渲染都重建这棵子树.**
   先写的 `applyFocus()` 是命令式的 (`svg.setAttribute`), 结果: 点节点 ->
   `setFocusId` 重渲染 (属性生效) -> 紧接着 `setPassportPos` 又一次重渲染 ->
   **属性全被抹掉**, 表现为"点了没反应, 但卡片出来了".
   正解: 把聚焦算进 SVG **字符串** (`useMemo(() => focusSvg(...))`), 属性成为渲染结果的一部分.
2. **"聚焦 -> 写 URL" 的 effect 会把深链清掉.**
   它在首次渲染时看见 `focusId === null`, 于是先执行 `url.hash = ''`,
   后面读 hash 的逻辑就再也看不到 `#focus=` 了 (分享链接打不开).
   正解: 用 `useState` 惰性初始化读 hash, 时序上不可能被覆盖.

### 实测 (生产构建, Playwright)

| 操作 | 结果 |
| --- | --- |
| 点 `cookieFetcher` | `data-focus-active=cookieFetcher`, 7 个 match, 1 个 selected, 非邻域 `opacity .13` |
| 再点同一节点 | 全部复位, hash 清空 |
| 点 `target` | 护照标题 "目标站点", 上游 2 / 下游 0 |
| 键盘 Enter (`learn`) | 聚焦成功, 演示页未被翻走 |
| 点空白 | 取消聚焦 |
| 分享 -> 复制链接 | `...?ppt=cf-gateway-deck#focus=cookieFetcher` |
| 放大态分享菜单 | 复制链接 + "下载图片" (PNG/JPEG/WEBP/SVG), 向上弹出 |
| `Esc` (放大+聚焦) | 第一次收卡片 (保留放大), 第二次退放大 |
| 独立页 `/ppt?...&page=3#focus=learn` | 进来即聚焦 "域名学习" 并显示护照 |
| 预览卡片 | `passports=0, focusAttrs=0, tools=0` (静态如初) |
| `.hxd-diagram__hint` 元素数 | 0 |

## 第三轮 (同日, 按用户三点反馈)

### 1. 同名 PPT 刷新会全部打开

**根因**: 卡片在 URL 里的标识直接用 `title`. deck-embed-test 里有 4 张卡片标题都是
`cf-gateway-deck`, 于是 `?ppt=cf-gateway-deck` 同时命中 4 张, 刷新一次弹 4 个层.

**修法** (两层):

- `PptCard` 新增 `cardId` prop, `PptEmbed` 传"演示页路径 / iframe src" —— 指向具体内容,
  标题只作最后兜底.
- 即便兜底到同一个标识, 模块级 `claimedPptIds` 也只让**第一张**认领, 其余保持关闭.

**踩坑 (重要)**: 认领最早写在 `useState` 的初始化器里, 结果带 `?ppt=` 直接导航时
**一张都打不开**. 原因是初始化器属于渲染阶段: React 会渲染多次而只提交一次,
第一次渲染把标识写进 Set, 提交那次再看到"已被认领"就判定为不打开.
正解: 认领放进 `useEffect` (每次提交只跑一次), 并在卸载时归还名额.

实测: 打开卡片 -> `?ppt=cf-gateway-deck.tsx` -> 刷新, 弹层数 `1` (修前 4);
直接带该参数导航, 弹层数 `1`.

### 2. 导出下拉菜单样式

菜单项是原生 `<button>`, 之前没有重置继承来的站点按钮皮肤 (自带边框/底色/padding),
在紧凑胶囊里显得又重又脏. 现在:

- 条目统一重置为无边框/透明底, 只保留 hover 反馈; 图标固定 12px 并随 hover 提亮.
- 面板: 不透明底 (常悬在复杂图面上), 12px 圆角, 更深阴影, 140ms 入场动画
  (`prefers-reduced-motion` 下关闭).
- 首项"复制链接"加粗为主操作; 下载组有 `下载图片` 小标题与分隔线, 每项带下载图标.
- 顺带把语义护照里的按钮也做了同样的继承重置.

实测放大态菜单: `168x222`, 向上弹出且不出图, 条目 `复制链接 / 下载图片 / PNG / JPEG / WEBP / SVG`.

### 3. 架构图交互动效 —— 之前**没有**, 现在补上

**现状核查**: 动效属于"图带不带"的属性, 由 archify 的 `meta.animation = "trace"` 决定.
原来的 `cf-gateway.ts` 是从**没开 trace** 的 HTML 里抽出来的 ——
SVG 里 `data-animation` / `data-animate` / `--step` 全是 0 个, 所以确实没有动效.
(注意: 抽出来的 CSS **一直**包含 `@keyframes archify-edge-flow / node-pulse` 等规则,
因为它们在 `SVG SEMANTIC CLASSES` 标记之后; 缺的是 SVG 上的钩子.)

**做法**:

- 用 `meta.animation: "trace"` 重新 render 一份 archify 产物并重新 extract,
  `cf-gateway.ts` 现在带 `data-animation="trace"` + 20 个 `data-animate` + `--step`.
- `Diagram` 实现 archify 的 **Motion Governor 协议**: 动效规则全是
  `html[data-ambient-motion="running"] …` 形式, 也就是"由文档根的一个属性统一放行".
  我们照搬, 不自己造动画: 放行 -> 跑一轮 -> 置 `settled` (终态回到作者定义的静态语言).
- 触发条件: **可交互 (canZoom) + 当前这一屏 (useIsSlideActive)**. 预览卡片不动,
  非当前屏也不动 (否则多张图同时往文档根写同一属性会互相打架).
- 尊重 `prefers-reduced-motion` (此时压根不放行).
- 工具条加**重播**按钮 (`▶`, 仅图本身带 trace 时出现).

实测: 预览态 `ambient=null`; 放大态 `running` -> 一轮后 `settled` -> 点重播回到 `running`;
采样动画属性随时间变化: 边 `strokeDashoffset 24.6 -> 9.7 -> 0`、`opacity 0.74 -> 0.90 -> 1`,
节点 `stroke-width 2.4 -> 2.19 -> 1.55` —— 确实在跑, 且跑完停在设计的静态形态.

### 4. 演示页更新

`cf-gateway-deck.tsx` 的架构图那一屏改成"可把玩"的说明:
`点节点看上下游 · 滚轮缩放 · 可放大查看 · 支持复制链接与导出`.

## 第四轮 (同日): 复制链接要"打开即同一个视角"

用户反馈: 复制出来的链接**没有带上第几页**, 而且希望它能**直接打开那个架构图**.
原来只复制 `location.href`, 而卡片打开时地址栏里只有 `?ppt=` ——
页码由 deck 内部维护 (`syncUrl=false`, 不往地址栏写), 放大态也完全没进 URL.

### 链接现在带三样东西

| 参数 | 含义 | 缺失时的体验 |
| --- | --- | --- |
| `?ppt=<卡片标识>` | 直接打开这张卡片的放大弹层 | 落在正文的缩略图上, 还要自己点 |
| `?page=<页码>` | 落在第几屏 | 回到第 1 页, 自己翻 |
| `?zoom=1` | 进入时就是放大态 | 还要自己点"放大" |
| `#focus=<节点>` | 聚焦到哪个节点 (可选) | 无高亮 |

实测复制出的链接:
`?ppt=cf-gateway-deck.tsx&page=3&zoom=1#focus=cookieFetcher`, 打开后:
弹层开、当前屏 "架构图"、放大态、`data-focus-active=cookieFetcher`、7 个 match、
护照标题 "CookieFetcher".

### 实现

- 新增 `DeckPageProvider` / `useDeckPage` (slide-state.tsx): 把**当前页码 + 总屏数**下发给控件,
  否则图不知道自己在第几页 (页码只在 deck 内部 state 里).
- 新增 `PptCardIdProvider` / `usePptCardId` (deck-layer.tsx): 卡片标识下发给内容.
  为什么用 context 而非 prop: 演示页内容是 registry 里**预先渲染好的 React 节点**,
  把 cardId 从 PptCard 传到 Diagram 要穿过一棵不由我们构造的树.
- `PptEmbed` 读 `?page=` (只在被 `?ppt=` 点名的那张卡片上生效, 否则全页卡片都会跳页).
- `Diagram` 读 `?zoom=1` 自动进放大态; 手动点开卡片时清掉 `?page=`/`?zoom=`,
  免得关掉再点开会莫名跳到别人分享的那一页.

### 这一轮踩的坑 (都是真 bug)

1. **hook 放到了提前 return 之后.** `PptEmbed` 在 iframe 分支/主题未就绪时会提前返回,
   我把 `sharedPage` 的 `useMemo` 写在后面 -> "Rendered more hooks than during the previous
   render", 整页崩溃 (`h1` 变成"页面已崩溃"). 所有 hook 必须在任何分支 return 之前.
2. **同一次重排里 hook 用到了 TDZ 变量.** 把 context 读取上移后, 依赖它们的 useCallback
   仍在前面 -> `used before its declaration`. 顺序必须是: context 读取 -> 派生值 -> 回调.
3. **`?zoom=1` 会让每一屏的图都自动放大.** deck 会把所有屏都挂载 (只是隐藏),
   只判 `canZoom` 不够 —— 实测同时出现 2 张铺满的浮层. 必须再加 `slideActive`,
   而且隐藏屏量出来是 0, FLIP 动画也会算错.

## 第五轮: `#ppt` 通用 HTML 侧车在规范 URL 下"整个不显示"

用户反馈: `[CF-Gateway-Pro 过盾链路与降级分支 #ppt ##w100%##](cf-gateway-pipeline.html)`
这种卡片"直接都无法展示了".

### 根因: 相对路径在**无尾斜杠**的规范 URL 下掉了一层

Docusaurus 文档的规范 URL **不带尾斜杠**
(`/knowledge-base/.../001-CF过盾工程-从零实现Turnstile绕过`).
浏览器对这种 URL 解析相对链接时, 会把最后一段当成**文件名**先砍掉再拼接:

```
正确 (带尾斜杠):  .../001-CF过盾.../  + cf-gateway-pipeline.html
                  -> .../001-CF过盾.../cf-gateway-pipeline.html        OK

错误 (无尾斜杠):  .../001-CF过盾...   + cf-gateway-pipeline.html
                  -> .../cf-gateway-pipeline.html                   掉了一层
```

而 `.../cf-gateway-pipeline.html` 并不存在 (侧车只发布在笔记同目录),
静态服务器于是 301 -> 去掉 baseUrl -> 302 -> 首页 -- 最终表现为
**iframe 里加载出的是网站首页**, 看起来就是"演示页整个没显示".

### 为什么是现在才炸

旧组件 `PptHtmlViewer` 里有一份 `resolvePptSrc()`, 专门把相对路径按当前笔记目录
重拼成绝对地址. 这次把它换成 `PptEmbed` 时, **那份解析逻辑丢了** --
`.tsx` 演示页不受影响 (它走 deck 注册表按文件名查找, 与 URL 解析无关),
所以只有通用 `.html` 侧车这一类内容坏掉.

### 修法

`PptEmbed` 新增 `resolveSidecarSrc(src, pathname)`: 把当前 pathname 当**目录**
(末尾补斜杠) 再拼相对路径, 与 URL 是否带尾斜杠无关.
iframe 与"新标签页打开"都改用解析后的地址.

实测 (同一篇笔记):

| URL 形态 | 修前 | 修后 |
| --- | --- | --- |
| 带尾斜杠 | 正常 | 正常 |
| **无尾斜杠 (规范/刷新/侧边栏直达)** | iframe 加载出**网站首页** | 正常, `CF-Gateway-Pro 过盾链路` 20 个节点 |

### 顺带确认的两件事 (都不是本次 bug)

- **`docusaurus serve` 会对 `.html` 请求去掉 baseUrl** (301). 这是本地 serve 命令的行为,
  GitHub Pages 不会这样. 因此验证静态产物要用"带 `/HXLoLi/` 前缀的纯静态服务器"模拟,
  直接 `serve` 会看到假阳性.
- **跨目录引用侧车目前不发布**. `ppt-html-assets` 插件只把 HTML 侧车复制到
  "同目录笔记"的路由下, 所以 `deck-embed-test.md` 里跨目录引用那份侧车仍然取不到.
  这是既有能力边界 (不是本次回归), 需要时可以再扩.

### 验证

用模拟 GitHub Pages 布局 (`/HXLoLi/` 前缀 + 纯静态服务) 逐个跑过所有 `.html` 侧车:

| 笔记 | 修后 iframe 标题 |
| --- | --- |
| CF过盾 | `CF-Gateway-Pro 过盾链路` (20 节点) |
| react-bits | `react-bits 分门别类学习总览` |
| DSH预设 | `DSH 提示词全解 · 四预设步骤链` |
| AgentLoop | `ReactLoopAgent 真实代码路径...` (12 节点) |
| Runtime | `DeepSeek Harness Runtime: 插件树 + 事件日志` (12 节点) |

## 第六轮: 把 archify viewer 的完整交互搬进 .tsx 架构图

用户要求: "我点击之后能有这种详细的说明特效, 以及下面那些选项, 也可以选,
就是所有的交互它都是全面的" —— 即 `.tsx` 里的 `Diagram` 要和 `.html` 侧车
(archify 原生产物) 一样能动.

### 盘点: archify viewer 到底有哪些交互

直接打开侧车枚举控件, 得到 11 个按钮 + 3 个菜单:

| 控件 | 作用 | 我们的实现 |
| --- | --- | --- |
| 主题 | 明/暗 | 由 deck 主题接管 (不重复造) |
| 视觉风格 | classic / signal-flow / blueprint / editorial | 新增"风"按钮 + 面板 |
| 演示模式 | 隐去全部控件只留图 | 新增"▶"按钮 + P/Esc |
| 导出 | 分享卡片 / PNG / JPEG / WebP / SVG / WebM | 已有分享菜单 (下载组) |
| 路径 | 两点间有向路径 | 新增"路"按钮, BFS 最短路 |
| 地图 | 语义雷达 | 未做 (缩略导航, 价值低) |
| 透镜 | 按节点角色统计并高亮 | 新增"透"按钮 + 类型条 |
| 查找 | 搜标签/稳定 ID | 新增"⌕"按钮, `/` 或 F |
| 指南 | 快捷键与操作说明 | 新增"?"按钮 + 面板 |
| 缩放 | -/+/重置 | 已有 |
| 阅读深度 | 随缩放自动升降细节 | **新增** (此前完全没有) |

### 实现要点

**1. 统一语义视图 (`applyView`)**

archify 用一组 `data-*` 属性表达"读者此刻在干什么", 它们的 CSS 是
**互斥且各有优先级**的. 之前只有一个 `focusSvg` 管聚焦; 现在改成 `applyView`,
在一个函数里按 `route > lens > preview > focus` 的优先级算, 避免两个状态
同时挂在 svg 上导致变暗规则互相矛盾.

**2. 阅读深度是"免费"的**

抽出来的 CSS 里本来就有:
```
.diagram-container[data-detail-level="map"]  svg [data-detail="context"] { opacity: 0 }
```
之前没人给容器加这个属性, 所以细节永远全显示. 现在按缩放自动切:
`< 100% -> map`, `>= 100% -> read`, `>= 175% -> full`. 实测 70% 时
context 文本 `opacity: 0`, 100%/190% 时 `1`.

**3. 视觉风格的坑: 属性重复**

archify 的 SVG 根节点**自带** `data-preset="classic"`. 第一版只在末尾追加
`data-preset="blueprint"`, 于是变成两个同名属性, 浏览器取第一个 -> 切了没反应.
修法: `withAttr` 先把旧的 `data-preset` 剥掉再写.

**4. 查找器排序**

搜 `cookie` 时"凭证库"(子标签含 cookie)会排在真正的 CookieFetcher 前面.
加秩: 标签命中 > 稳定 ID > 子标签/上下文命中.

**5. 快捷键必须走捕获阶段**

`/` `r` `t` `p` 这些键要拦在 Deck 的翻页监听之前, 否则会被当成翻页.
输入框内还要额外 `stopPropagation`, 不然打字会翻页.

### 验证 (Playwright, 生产构建)

| 功能 | 结果 |
| --- | --- |
| 语义透镜 | 5 个类型 (backend:4 frontend:3 cloud:1 database:1 external:1), 选中后 6 个 match, 非命中 `opacity .11` |
| 路径探测 | `inject -> fingerprint -> click -> cred -> cookieFetcher -> target`, 6 步 5 跳, 11 个 route-match |
| 查找节点 | `/` 打开, 输入 `cookie` -> `["CookieFetcher", "凭证库"]`, 回车跳转并高亮 |
| 图表指南 | 14 行快捷键说明 |
| 视觉风格 | 4 档; 选"蓝图"后 `data-preset="blueprint"` |
| 演示模式 | 工具条 `display: none`; Esc 退出 |
| 阅读深度 | 70% -> `map` (context 隐藏) / 100% -> `read` / 190% -> `full` |
| 预览卡片 | `tools=0 panels=0 passports=0` (静态如初) |

前五轮的全部回归 (节点聚焦/取消/键盘、分享链接、全屏四条路径、动效、同名卡片、侧车解析) 均通过.

## 第七轮: 修用户报的四个问题 (动画顺序 / 风格 / 路径 / 问号)

### 1 + 3 是**同一个根因**: SVG 被反复重建

用户报: "动画根本没按箭头顺序, 有时候甚至是倒着来的" 和 "路径选择点击根本没用".
实测两者同源 —— `dangerouslySetInnerHTML` 让 React 每次重渲染都重建整棵 SVG 子树:

| 症状 | 机制 |
| --- | --- |
| 动画"倒着跑" | 任何交互触发重渲染 -> SVG 重建 -> 动画被打回起点重新跑 |
| 点击没用 | `mousedown` 里 `setPanning(true)` 是状态变更 -> 重渲染 -> 光标下的节点被销毁 |
| | -> 浏览器**不派发 click** (实测: 只有 mousedown+mouseup, 零个 click) |

**修法 (两处, 缺一不可)**:

1. **SVG 由 effect 一次性写入**, React 不再管理这棵子树 (以 `a.svg` 为身份, 变了才重写).
   视图属性改走新的 `applyViewToDom()` **命令式**设置 —— 只改属性, 不动节点.
2. **拖拽等指针真的移动了才开始**: `mousedown` 只记起点, 超过 3px 阈值才 `setPanning(true)`.
   单纯点击因此不产生任何状态变更, click 正常派发.

顺带把默认缩放**固定为 100%**: 之前自动放大到 187%, 实测 10 个节点里 9 个落在
可视区外 —— 点谁都点不到, 这是"路径点了没用"的第二重原因.

### 2 + 4: 删掉"视觉风格"和"图表指南"

用户判断对, 采纳: 我们的图没有多到需要风格切换, 功能也没多到需要一份说明书.
移除按钮 / 面板 / 快捷键 (T / ?) / 相关 CSS, 工具条从 12 个降到 10 个:

```
缩小 放大 复位 重播动效 语义透镜 路径探测 查找节点 演示模式 放大查看 分享
```

### 验证 (Playwright, 真鼠标点击)

| 项 | 修前 | 修后 |
| --- | --- | --- |
| 点节点后 SVG 节点身份 | `sameNode: false` (被重建) | `sameNode: true` |
| 动画 dashoffset 被交互打断 | 36.9px 跳回 51.0px (倒退) | 保持不动 |
| 真点击节点收到的 click 事件 | **0 个** | 正常派发 |
| 路径探测 (真点击 inject -> target) | 状态不动 | `inject→...→target`, 6 步 5 跳 11 个 match |
| 边到达顺序 | 受交互干扰 | 600/800/1000/1200/1400/1600/1800/2000ms 顺序推进 |
| 可点击节点 (默认) | 1/10 | 10/10 |
| 工具条 | 12 个 (含风格/指南) | 10 个 |

前六轮回归全部通过; 独立播放页 `/ppt` 同样验证 (真点击节点可聚焦).

## 第八轮: 放大后空白 (以及放大后点不动)

用户报: "放大架构图之后, 什么东西都看不到是空白的, 只有放大的时候才会触发".

### 根因: 放大 = **重新挂载**, 而我的注入守卫只认内容

上一轮为了修"动画被打回起点 + 点击失效", 把 SVG 改成注入一次后由 React 不再管理.
但守卫写的是"**内容**变了才写", 漏了一个事实: 放大时浮层 Portal 到 deck 根节点,
**容器 div 是一个全新的节点**.

```
a.svg 没变  ->  守卫判定"不用重写"
但容器换了 ->  新容器里空空如也  ->  放大后一片空白
```

实测: 放大后 `sameContainer: false`, `innerHTML.length === 0`.

**修法**: 守卫的判据改成 **(容器元素, 内容) 对**, 任一变化都重写.

### 连带发现: 放大后点节点没反应

修完空白立刻暴露出第二个 bug: 放大态里点节点, `data-focus-active` 是 null.

同一个"重新挂载"导致的: 事件监听 effect 的 deps 里**没有 full**, 所以
监听还绑在**已卸载的旧 stage** 上; 新 stage 上没有任何监听.
(滚轮那条 effect 早就带了 full, 所以滚轮一直好使, 只有点击/键盘受影响 ——
这个不对称正是排查的突破口.)

**修法**: deps 加 `full`.

### 验证

| 项 | 修前 | 修后 |
| --- | --- | --- |
| 放大后容器 innerHTML | **0** (空白) | 20099 字节, 10 节点 |
| 放大后 SVG 尺寸 | 无 SVG | 1304x681 |
| 放大态点节点 | `focus: null` | `focus: cred`, 非邻域 `opacity .13` |
| 退出放大 | 内联图仍在 | 内联图仍在 (536x183) |
| 二次放大 | - | 正常 (20246 字节) |
| 独立页 /ppt 放大 | - | 1364x709, 点节点正常 |

前七轮回归全部通过 (全屏四条路径 / 动画顺序 / 路径探测 / 透镜 / 阅读深度 / 预览静态).

## 第九轮: 去掉节点卡片上的"复制链接"

用户: "我点击节点它会有一个选项叫做复制链接, 我不要我干嘛要复制链接, 这个简直莫名其妙".

判断: 采纳. 语义护照的职责是"回答这个节点是什么", 不是"对这个节点做点什么".
而且工具条的**分享**菜单第一项本来就是"复制链接"(带 ?ppt= &page= #focus=),
卡片里再放一个是纯重复.

**改动**:

- `DiagramPassport` 去掉 `__actions` 区块与 `onCopy` / `copied` 两个 props
  (这两个 props 现在只剩卡片在用, 一并删掉, 避免留死参数);
- 删除 `.hxd-passport__actions` 的 CSS;
- 组件注释补上"卡片上不放任何动作按钮"的理由.

节点卡片现在只剩: 关闭按钮 + 上下游跳转条目.

### 验证

| 项 | 结果 |
| --- | --- |
| 卡片内按钮 | `["×", "→ 指纹随机化"]` —— 只剩关闭与上游跳转 |
| 卡片是否还有动作区 | `hasActions: false` |
| 分享菜单 | 仍为 `["复制链接"]`, 复制出带 `#focus=` 的完整链接 |
| 节点聚焦 / 取消 / 键盘 / Esc | 全部正常 |
| 全屏四条路径 / 放大空白 / 路径探测 | 全部正常 |
