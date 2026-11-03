# peek-shots: 探头立绘验收

## 最终状态 (2026-11-04 CST, 代码收口)

B14-B21 全部修复. 验证来源: [用户本机] = 用户在工作树实测; [源码] = 按映射地址读到的当前源码核对.

| # | 问题 | 修法 | 验证方式 |
|---|---|---|---|
| B14 | `/search` 找不到墙且无 fallbackWidth, 桌面恒不渲染 | CENTER_WALL 加 `fallbackWidth` (Infima 容器 1140, >= 1440 宽用 1320), 墙位 = 视口中线 ± 宽/2 | [源码] index.tsx `INFIMA_CONTAINER` / CENTER_WALL; 本机 1920 下 `/search` 的 `__nekoDebug().rendered` 应为 true |
| B15 | pocket 求交没豁免页脚, 页底几乎恒 pocket-blocked | 求交时跳过祖先含 `.footer` 的元素, 其余照判 | [源码] NekoPocket 求交; 本机 390 / 768 滚到页底, `pocket.blockedBy` 不含页脚链接 |
| B16 | 隐藏中的 pocket 被算进 slots / overlaps / stages | 只取 `.neko-pocket[data-on]` | [源码] nekoDebug; 本机首屏 (未到页底) `overlaps` 为空 |
| B17 | 放不下时宽度 = room - 16 可为负 | wl / wr / pair w 夹到 >= 0, show = false, reason = no-room | [源码] useWallFit; 本机 no-room 格 `room.w >= 0` |
| B18 (原 N2) | <= 480 档 pocket 抬 64px, 与回顶按钮 (顶 68.8px) 交 4.8px | N 档 `--pk-lift: 76px` (T 档 72 不变) | [源码] BlogWithCats.css `@media (max-width: 480px)`; [用户本机] 已验; 本机 375 / 390 回顶按钮显示时 pocket 仍 data-on |
| B19 (原 N3) | 空 `.neko-sides` 计入 stages | stages 只数 `.neko-sides[data-on], .neko-pocket[data-on]` | [源码] nekoDebug; [用户本机] 已验; no-room 格 stages = 0 |
| B20 | manifest 单侧条目被整条丢掉 | parseManifest 改为 `!left && !right` 才跳过; slotImgs 单侧按锚点进槽不镜像; 宽度只按有图那侧 room 算 | [源码] parseManifest / slotImgs / useWallFit; [用户本机] 已验; `__nekoDebug().assets === 'manifest'`, 单侧条目 (mahiro-02 / milim-02 / unk-blonde-twintail-01) 可被抽到 |
| B21 (原 N1) | `/anime/details` 页内 pair 墙位推算错, 立绘浮在正文上 / v2 时整页无立绘 | 去掉页内 SideCats, 路由表改 gutter, 墙 = `DETAIL_WALL` (`[class*=shell]`, CSS Module 哈希类), 零侵入 + 内侧 10% 淡出 | [源码] ROUTE_RULES `/anime/details`; [用户本机] 已验; 1920 下 `variant === 'gutter'`, `overlaps` 为空 |

附 (非 B 编号):
- N4 (B14 注释称 404 无 `<main>`): 仅注释, 不影响行为.
- 素材: 9 张曾截断的立绘已重传并验证可读; 56 张全过机械校验, manifest 25 entries + 9 excluded 覆盖 56 文件 (见 `catgirl-normalize.md` §4-§5).
- 收口自检 [用户本机]: tsc 10 条无新增, characterPrompt.check 22 项全通.

---

> **v2 更新 (2026-11-03 23:10 CST)**: 源码已修 B14-B17, 复推见 `static-projection-v2.md`. 下方 v1 内容原样保留, 已被修复的条目用 **[已修 Bxx]** 标注.

## v2 摘要

- 源码: index.tsx 40230 -> **41337 B** (diff 即 B14-B17), BlogWithCats.css 16800 B 与 Root.tsx 1571 B 未变.
- 桌面 14 格: 渲染 8 (新增 /search 1920, w 260), no-room 4, off 2 (/anime/details).
- 手机 21 格: pocket 15 格预计都能在页底出现; 768 档 5 格无条件, 375 / 390 档 10 格在回顶按钮显示时被挡 (交 4.8px). 文档 3 格 page-off, 详情 3 格 off.
- 未修问题 4 个: N1 详情页整页无立绘 (off 但 details.tsx 没挂 SideCats); N2 <= 480 档 pocket 与回顶按钮相交; N3 空 .neko-sides 仍计入 stages; N4 B14 注释称 404 无 main (不实).
- 三条判据 v2: hScroll 预计恒过; overlaps 桌面预估空, 手机首屏空 (B16), 页底空 (页脚被 L434 跳过); stages 0 或 1 全过, no-room 格要看 reason.

---

# peek-shots: 探头立绘验收 (静态核验版)

> 2026-11-03 CST. 没有真实截图.

## 为什么没有真实截图

hark 跑在云端沙箱, 本机的浏览器 (/usr/bin/chromium, google-chrome-stable, python playwright) 和本机构建产物分处两地, 互相访问不到; 而且本机 `build/` 是改动前的旧产物 (见 B), 就算在本机截图, 截到的也不是新组件.

本机可用 `npm run build && python3 -m http.server 3001 -d build` 起站 (baseUrl `/HXLoLi/`), 逐路由取 `window.__nekoDebug()` 复验上面每一条.

## A. 读构建产物

经 fs 隧道 curl 了 build/ 下 9 个 html (全部 200):

| 请求的路径 | 实际文件 | 字节 |
|---|---|---|
| index.html | index.html | 71493 |
| anime/index.html | 不存在, 读 anime.html | 12608 |
| anime/details/index.html | 不存在, 读 anime/details.html | 9381 |
| blog/index.html | 不存在, 读 blog.html | 70014 |
| blog/archive/index.html | 不存在, 读 blog/archive.html | 19134 |
| docs/index.html | 不存在 (docs 没有索引页), 读 docs/关于.html | 43382 |
| 404.html | 404.html | 8231 |
| search/index.html | 不存在, 读 search.html | 8258 |
| 博客文章 | blog/2026/10/07/01-AI代码质量.html | 110113 |

这个站的产物是 `<path>.html` 形式, 不是 `<path>/index.html`.

## B. 产物里核验的结果

1. **`.neko-sides` / `.neko-pocket`**: 9 个 html 里都没有 (SSR 首帧本来就不应该有). 但有**旧组件**的 SSR 标记: `class="neko neko-left"` / `"neko neko-right"` 出现在 index / anime/details / blog/archive / 博客文章 4 页, `neko-sticky` 出现在 anime.html.
2. **墙标记**: `.blog-container` + `.blog-content-wrapper` + `.blog-content` 出现在 index / anime/details / blog/archive / 博客文章; anime.html 只有 `.blog-container`; blog / docs / 404 / search 没有. 这是旧版式 (新源码里首页和番剧详情已经不用 BlogWithCats, 博客列表反而改用了).
3. **CSS**: 只有一个样式文件 `assets/css/styles.ea85700b.css` (368632 B). 找到的 neko 选择器 4 个, 全是旧的: `.neko`, `.neko-left`, `.neko-right`, `.neko-sticky`. **`.neko-sides` / `.neko-pocket` / `.neko-slot` / `[data-variant='gutter']` 一个都没有.**
4. **JS**: 下载了 `assets/js/` 全部 1404 个 chunk (43 MB) 逐个 grep: `__nekoDebug` 0 个文件, `neko-sides` 0, `neko-pocket` 0; 只有 `da1adcad.8f84f743.js` 里有旧的 `neko-sticky`. 入口是 `main.4f668f75.js` + `runtime~main.83db2f8a.js`.

结论: `build/` 早于这次改动 (甚至早于 11-02 的 sides 版), 新代码没有进产物. 要在本机验收新组件, 必须先重新 build.

## C. 源码常量推演

读了 index.tsx (40230 B) / BlogWithCats.css (16800 B) / Root.tsx, 加上 BlogListPage / BlogPostPage / BlogLayout / HomePage / anime/details.tsx 判定每页由谁挂舞台, 尺寸取 styles.ea85700b.css 的 Infima 变量. 7 路由 x 5 视口 = 35 行, 见 `static-projection.md`.

- 桌面渲染 7 格: / (1280, 1920), /anime 1920, /docs/关于 1920 (仅右, 宽 134), /blog (1280 宽 196, 1920 宽 320), /404 1920 (宽 260).
- 桌面 no-room 7 格: /anime 1280 (< 1360), /anime/details 两档, /docs/关于 1280, /search 两档, /404 1280.
- 手机 21 格: 3 格文档页 pocket-page-off; 其余 18 格首屏 pocket-wait-footer, 短页 (404, 结果少的 search) 预估 pocket-blocked.

源码层面看出的问题 (详见 static-projection.md §3):
1. **[已修 B15, L505]** pocket 不再豁免页脚, 但只在页脚进视口时才实测; 页脚 li 整列宽, 手机端 pocket 很可能大多数时候被判 pocket-blocked 而不显示.
2. **[已修 B16, L419-421]** nekoDebug 的 overlaps 把 visibility: hidden 的 pocket 也算进去, 手机端可能误报.
3. /search 和 /anime/details 没有 `<main>` **[已修 B14, L705/L719: 搜索走 fallback, 1920 改为渲染; 详情页改 off, 引出新问题 N1]**, CENTER_WALL 找不到墙, 桌面端永远不渲染 (与 peek-layout.md §2 不符).
4. (另 v1 发现 room.w 可为负 **[已修 B17, L334/L339-340]**.) 博客文章页在五个视口下都不出立绘 (1920 room 只有 83, 手机端 pocketOff).

## D. 三条机械判据的静态结论

| 判据 | 静态结论 | 必须本机实测才能确认的 |
|---|---|---|
| scrollWidth <= innerWidth | 立绘这边恒过: 槽与 pocket 都是 position: fixed, `.neko-sides` width/height 0 | 页面自身是否本来就横向溢出 (与立绘无关) |
| overlaps 为空 | 桌面 7 个渲染格按坐标推算都落在留白里, 预估空; 不渲染的格子没有槽, 自然空 | 手机 18 个 pocket 格 (取决于真实 DOM 与滚动位置, 且有隐藏 pocket 误报风险); /anime 1920 的筛选控件位置; 首页 edges 角落里有没有可点元素 |
| stages 为 0 或 1 | 35 格全过: 渲染与 no-room 都是 1 个 `.neko-sides` 或 1 个 `.neko-pocket`, 文档页手机端为 0; RouteCats 在页面挂了 SideCats 时让位 | 站内路由切换后旧舞台是否及时卸载 |

另外必须本机实测: 立绘与 pocket 的实际观感 (图像本身是否压到内容), 滚动条是否真占宽 (影响 /docs 1920 的宽度 130 还是 134), 以及 pocket 在滚动过程中是否真能出现.

本机复验步骤: `cd HXLoLi && npm run build && python3 -m http.server 3001 -d build`, 然后访问 `http://localhost:3001/HXLoLi/...` (baseUrl 是 `/HXLoLi/`), 每页等 1.5s 后取 `window.__nekoDebug()`.

## 附注

- 写盘时用 `_probe.txt` 试过一次 PUT (5 字节, 内容 "test"), 隧道不支持 DELETE, 留在本目录, 可以手工删.
