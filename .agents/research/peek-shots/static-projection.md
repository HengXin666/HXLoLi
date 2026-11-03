未截图, 由源码常量静态推演 (本机浏览器仅在本机可用; build/ 是改动前的旧产物, 不含新组件)

> 2026-11-03 22:55 CST. 源码取自隧道当时的版本: `src/components/BlogWithCats/index.tsx` (40230 B), `BlogWithCats.css` (16800 B), `src/theme/Root.tsx` (1571 B).
> 版面尺寸取自 `build/assets/css/styles.ea85700b.css` 的 Infima 变量 (主题布局与立绘改动无关, 旧产物可用) 和 build/*.html 的 DOM 结构.

## 0. 推演用到的常量 (全部出自源码)

| 常量 | 值 | 出处 |
|---|---|---|
| DESKTOP_QUERY | `(min-width: 997px) and (min-height: 501px)` | index.tsx |
| POCKET_QUERY | `(max-width: 768px)`; 769-996 不出 pocket | index.tsx |
| NEKO_MIN / NEKO_MAX / NEKO_GAP / NEKO_IN | 130 / 320 / 16 / 0.06 | index.tsx |
| GUTTER_MIN / GUTTER_MAX | 120 / 260 | index.tsx |
| pair 宽 | `min(320, (min(roomL,roomR) - 16) / 0.94) x short`, 渲染条件 `w >= 130 x short` | useWallFit |
| gutter 宽 | `wr = min(260, roomR - 16) x short`, 渲染 `wr >= 120 x short`; 左侧还要 `sides=both` 且 `wl >= 120` | useWallFit |
| short | innerHeight <= 640 ? 0.85 : 1 (本表 5 个视口都 > 640, 恒为 1) | useWallFit |
| VIEWPORT_QUERY | edges >= 1100, screen >= 1360, side >= 1200, corner >= 1400 | index.tsx |
| --neko-lift | `clamp(165px, 8vh + 125px, 245px)`: 800 高 = 189px, 1080 高 = 211.4px | CSS |
| 槽 max-height | `100vh - 60px - lift`: 800 高 = 551px, 1080 高 = 808.6px | CSS |
| 人物区 | 槽矩形下 55% (nekoDebug 按 top + 0.45h 算) | nekoDebug |
| edges | 宽 `clamp(160, 12vw, 220)`, bottom 0, translateY(30%), 无 max-height | CSS |
| screen | 宽 `clamp(180, (100vw-1200)x0.5+130, 270)`, 推出 14% | CSS |
| pocket | <= 480: 36px x 14vh, 底抬 64; 481-768: 44px x 18vh, 底抬 72; fixed 贴视口右边; 页脚进视口才实测, 合规才 data-on | CSS + NekoPocket |
| pocketOff | 文档 (DOC_WALL) 与博客文章页 (isBlogPost) 为 true | RouteCats / SideCats |
| Infima | container 1140 (>= 1440 宽为 1320), --doc-sidebar-width 300, navbar 60 | styles.ea85700b.css |
| 滚动条 | 按 clientWidth = innerWidth 算 (站点 CSS 有 `::-webkit-scrollbar{display:none}`); 若根元素实际有 9px 滚动条, 受影响的格子在备注里给了第二个值 | 假设 |

## 1. 路由 -> 舞台 (按源码)

| 路由 | 谁挂舞台 | 变体 | 墙 | 依据 |
|---|---|---|---|---|
| `/` | RouteCats | edges | 视口边 | ROUTE_RULES `'/'`; HomePage.tsx 不再用 BlogWithCats |
| `/anime` | RouteCats | screen | 视口边 | ROUTE_RULES `'/anime'` (精确匹配) |
| `/anime/details?id=443831` | RouteCats | gutter (FALLBACK, CENTER_WALL) | `main .container` / `main.container` | `'/anime'` 是精确匹配不覆盖子路径; details.tsx 当前源码**没有** BlogWithCats / SideCats |
| `/docs/关于` | RouteCats | gutter, sides r, pocketOff | `[class*=docMainContainer] > .container` | ROUTE_RULES `'/docs/'` |
| `/blog` | 页内 SideCats (BlogListPage, contentWidth 880) | pair | `.blog-content` | BlogListPage 用 Layout (无博客侧栏 / TOC) |
| `/search?q=test` | RouteCats | gutter, CENTER_WALL | `main .container` / `main.container` | ROUTE_RULES `'/search'` |
| `/404` | RouteCats | gutter (FALLBACK, CENTER_WALL) | 同上 | 表外路由 |

DOM 事实 (build/*.html, 主题结构): 404 页是 `<main class="container margin-vert--xl">` -> 命中 `main.container`; 搜索页是 `<div class="container margin-vert--lg">`, 页面里**没有 `<main>`**; 番剧详情页也**没有 `<main>`**, 只有导航栏里的 `.container-fluid`. 所以这两页的 CENTER_WALL 两个选择器都找不到墙.

## 2. 推演表 (7 路由 x 5 视口 = 35 行)

列: 舞台 = .neko-sides 或 .neko-pocket 或无; stages = nekoDebug 会数到的 `.neko-sides, .neko-pocket` 个数 (首屏, 未滚动); hScroll 只计立绘贡献; overlaps 是首屏预估.

| # | 路由 | 视口 | 舞台 | 变体 | 渲染 | reason (首屏) | 关键数字 | stages | hScroll | overlaps |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | / | 375x667 | .neko-pocket | edges -> pocket | 首屏否 | pocket-wait-footer | 36x93, 底抬 64, x 339-375, y 510-603 | 1 | 否 | 需实测 (见 §3-1) |
| 2 | / | 390x844 | .neko-pocket | edges -> pocket | 首屏否 | pocket-wait-footer | 36x118, 底抬 64, x 354-390, y 662-780 | 1 | 否 | 需实测 |
| 3 | / | 768x1024 | .neko-pocket | edges -> pocket | 首屏否 | pocket-wait-footer | 44x184, 底抬 72, x 724-768, y 768-952 | 1 | 否 | 需实测 |
| 4 | / | 1280x800 | .neko-sides | edges | 是 (两侧) | rendered | 宽 160 (12vw=153.6 被 clamp 到 160), 高 384, 人物区 y 704-800(可见), x 0-160 / 1120-1280; room null | 1 | 否 | 预估空 |
| 5 | / | 1920x1080 | .neko-sides | edges | 是 (两侧) | rendered | 宽 220 (12vw=230.4 封顶), 高 528, 人物区 y 948-1080, x 0-220 / 1700-1920 | 1 | 否 | 预估空 |
| 6 | /anime | 375x667 | .neko-pocket | screen -> pocket | 首屏否 | pocket-wait-footer | 同 #1 | 1 | 否 | 需实测 |
| 7 | /anime | 390x844 | .neko-pocket | screen -> pocket | 首屏否 | pocket-wait-footer | 同 #2 | 1 | 否 | 需实测 |
| 8 | /anime | 768x1024 | .neko-pocket | screen -> pocket | 首屏否 | pocket-wait-footer | 同 #3 | 1 | 否 | 需实测 |
| 9 | /anime | 1280x800 | .neko-sides (空) | screen | 否 | no-room | 1280 < 1360 (VIEWPORT_QUERY.screen) | 1 | 否 | 空 |
| 10 | /anime | 1920x1080 | .neko-sides | screen | 是 (两侧) | rendered | 宽 270 ((1920-1200)x0.5+130=490 封顶 270), 高 648 (< max 808.6), 推出 37.8px; 左槽 x -38-232, 右槽 1688-1958; 人物区 y 512-869; 卡片墙 360-1560, 与立绘可见部分隔 128px | 1 | 否 (fixed) | 预估空 |
| 11 | /anime/details | 375x667 | .neko-pocket | gutter -> pocket | 首屏否 | pocket-wait-footer | 同 #1 | 1 | 否 | 需实测 |
| 12 | /anime/details | 390x844 | .neko-pocket | gutter -> pocket | 首屏否 | pocket-wait-footer | 同 #2 | 1 | 否 | 需实测 |
| 13 | /anime/details | 768x1024 | .neko-pocket | gutter -> pocket | 首屏否 | pocket-wait-footer | 同 #3 | 1 | 否 | 需实测 |
| 14 | /anime/details | 1280x800 | .neko-sides (空) | gutter | 否 | no-room | 页面无 `<main>`, 墙找不到, 无 fallbackWidth -> setFit(null), 重试 12 次后放弃; room null | 1 | 否 | 空 |
| 15 | /anime/details | 1920x1080 | .neko-sides (空) | gutter | 否 | no-room | 同 #14 (与视口无关) | 1 | 否 | 空 |
| 16 | /docs/关于 | 375x667 | 无 | gutter -> pocket | 否 | pocket-page-off | pocketOff (DOC_WALL), NekoPocket 返回 null | 0 | 否 | 空 |
| 17 | /docs/关于 | 390x844 | 无 | gutter -> pocket | 否 | pocket-page-off | 同上 | 0 | 否 | 空 |
| 18 | /docs/关于 | 768x1024 | 无 | gutter -> pocket | 否 | pocket-page-off | 同上 | 0 | 否 | 空 |
| 19 | /docs/关于 | 1280x800 | .neko-sides (空) | gutter r | 否 | no-room | docMainContainer = 1280-300 = 980 < 1140, 容器铺满到视口右边: roomL 0 (侧边栏右沿 300) / roomR 0, w = min(260, 0-16) = -16 -> room {l:0, r:0, w:-16} | 1 | 否 | 空 |
| 20 | /docs/关于 | 1920x1080 | .neko-sides | gutter r | 是 (仅右) | rendered | 主区 1620, 容器 1320 居中: 墙 450-1770; roomL 150 / roomR 150; w = min(260, 134) = 134 >= 120 (9px 滚动条时 roomR 145.5, w 130, 仍渲染); 高 321.6, 槽 y 547-869, 人物区 y 692-869, x 1770-1904 | 1 | 否 | 预估空 (返回顶部按钮 y 1011-1059 在人物区下方) |
| 21 | /blog | 375x667 | .neko-pocket | pair -> pocket | 首屏否 | pocket-wait-footer | 同 #1 (列表页不是 isBlogPost, 照出) | 1 | 否 | 需实测 |
| 22 | /blog | 390x844 | .neko-pocket | pair -> pocket | 首屏否 | pocket-wait-footer | 同 #2 | 1 | 否 | 需实测 |
| 23 | /blog | 768x1024 | .neko-pocket | pair -> pocket | 首屏否 | pocket-wait-footer | 同 #3 | 1 | 否 | 需实测 |
| 24 | /blog | 1280x800 | .neko-sides | pair | 是 (两侧) | rendered | 墙 200-1080 (880 居中); room 200/200; w = (200-16)/0.94 = 195.7 -> 196; 高 470 (< max 551); 左槽 x 16-212, 右槽 1068-1264; 人物区 y 353-611 | 1 | 否 | 预估空 (blog-content 内的链接按规则豁免) |
| 25 | /blog | 1920x1080 | .neko-sides | pair | 是 (两侧) | rendered | 墙 520-1400; room 520/520; w = min(320, 536) = 320; 高 768 (< max 808.6); 左槽 x 219-539, 右槽 1381-1701; 人物区 y 446-869 | 1 | 否 | 预估空 |
| 26 | /search?q=test | 375x667 | .neko-pocket | gutter -> pocket | 视页面高度 | pocket-wait-footer 或 pocket-blocked | 同 #1; 结果少时页脚首屏可见, 会立即实测 (见 §3-1) | 1 | 否 | 需实测 |
| 27 | /search?q=test | 390x844 | .neko-pocket | gutter -> pocket | 同上 | 同上 | 同 #2 | 1 | 否 | 需实测 |
| 28 | /search?q=test | 768x1024 | .neko-pocket | gutter -> pocket | 同上 | 同上 | 同 #3 | 1 | 否 | 需实测 |
| 29 | /search?q=test | 1280x800 | .neko-sides (空) | gutter | 否 | no-room | 搜索页无 `<main>`, CENTER_WALL 两个选择器都落空 -> 墙找不到; room null | 1 | 否 | 空 |
| 30 | /search?q=test | 1920x1080 | .neko-sides (空) | gutter | 否 | no-room | 同 #29 (设计文档预期 >= 1440 渲染, 实际永不渲染) | 1 | 否 | 空 |
| 31 | /404 | 375x667 | .neko-pocket | gutter -> pocket | 大概率否 | pocket-blocked (预估) | 404 很短, 页脚首屏可见 -> 立即实测; pocket y 510-603 落在页脚区, 页脚 li 是整列宽, 横向必交 (见 §3-1) | 1 | 否 | 需实测 |
| 32 | /404 | 390x844 | .neko-pocket | gutter -> pocket | 大概率否 | pocket-blocked (预估) | 同上, y 662-780 | 1 | 否 | 需实测 |
| 33 | /404 | 768x1024 | .neko-pocket | gutter -> pocket | 大概率否 | pocket-blocked (预估) | 同上, y 768-952 | 1 | 否 | 需实测 |
| 34 | /404 | 1280x800 | .neko-sides (空) | gutter | 否 | no-room | 墙 main.container 70-1210 (1140); room 70/70; wl = wr = 54 < 120 -> room {l:70, r:70, w:54} | 1 | 否 | 空 |
| 35 | /404 | 1920x1080 | .neko-sides | gutter both | 是 (两侧) | rendered | 墙 300-1620 (1320); room 300/300; w = min(260, 284) = 260; 高 624; 左槽 x 40-300, 右槽 1620-1880; 人物区 y 525-869 | 1 | 否 | 预估空 |

汇总: 渲染 7 格 (#4 #5 #10 #20 #24 #25 #35, 全部是桌面); 桌面 no-room 7 格 (#9 #14 #15 #19 #29 #30 #34); 手机 21 格里 3 格 pocket-page-off (文档), 其余 18 格首屏都是 pocket-wait-footer 或 (短页) 预估 pocket-blocked.

## 3. 推演里发现的问题 (源码层面, 不需要浏览器就能确认)

1. **pocket 与页脚**: 新版 pocketBlockers 不再跳过 `.footer` (旧版有 `n.closest('.footer')` 豁免), 而 pocket 又只在页脚进视口时才实测. 页脚的 `li.footer__item` 是整列宽的块元素 (404.html 里有 4 个), 只要 pocket 的竖直范围落在页脚上, 横向一定相交 -> pocket-blocked. 滚到页底时 pocket (底抬 64/72px) 几乎必然落在页脚区, 所以手机端很可能在多数页面都显示不出来. 只有页脚刚露头、pocket 对着正文空白的那一小段滚动区间可能合规.
2. **nekoDebug 的 overlaps 也数隐藏的 pocket**: slots 取 `.neko-pocket` 时不看 data-on, 首屏 visibility: hidden 的 pocket 矩形也会参与 overlaps 计算, 手机端的 overlaps 判据可能误报.
3. **/search 与 /anime/details 桌面端永远不渲染**: 两页都没有 `<main>`, CENTER_WALL (`main .container`, `main.container`) 找不到墙, 又没有 fallbackWidth. 与 peek-layout.md §2 "搜索约 >= 1440 渲染" 不符; details 页的设计文档写的是页内 pair, 当前 details.tsx 已不挂 BlogWithCats.
4. **room.w 会是负数**: gutter 放不下时 w = min(260, room-16) 可能为负 (#19 给 -16), 只影响自检报告的可读性.
5. **空舞台也算 stages=1**: no-room 时 NekoStage 仍渲染空的 `.neko-sides`, stages 判据仍过 (0 或 1), 但不能用 stages 区分渲染与否, 要看 reason.

## 4. 附: 两个表外页面 (未计入 35 行)

- 博客文章页 1920x1080: BlogLayout 侧栏 col--3 (480) + 正文列 78.25% (1126.8) + TOC 列; 墙 563-1523, roomL 83 (侧栏) / roomR 83 (TOC) -> w = (83-16)/0.94 = 72 < 130 -> no-room. 1280 更窄, 同样 no-room. 手机端 pocketOff -> pocket-page-off. 也就是文章页在这五个视口下都不出立绘.
- 博客归档页: 页内 pair, 版式同 /blog 的 960 列, 1280 room 160 -> w 153 渲染, 1920 w 320 渲染 (未核对归档页是否有侧栏).
