未截图, 由源码常量静态推演 (v2, 针对 B14-B17 修复后的源码复推)

> 2026-11-03 23:10 CST. v1 结论见 static-projection.md (保留不改).

## 1. 读到的是修复后版本 (要求 1)

| 文件 | v1 字节 | v2 字节 | v2 md5 | 结论 |
|---|---|---|---|---|
| src/components/BlogWithCats/index.tsx | 40230 | **41337** | a3e86f707fc2896e957ae43a05a854f6 | 已变, diff 恰好是 B14-B17 + 一段注释 |
| src/components/BlogWithCats/BlogWithCats.css | 16800 | 16800 | 06af2d7b9fc9d37d8862cdce659f75ba | 未变 (与用户所说一致) |
| src/theme/Root.tsx | 1571 | 1571 | 9d11184701d5f928d5269b9f5f66a261 | 未变 |
| src/pages/anime/details.tsx (补读) | 30186 | 30186 | - | 未变, 仍无 SideCats / BlogWithCats (0 处) |

v1 -> v2 的 index.tsx diff 只有这些块: L306-308 (fallbackWidth 1140/1320), L334 / L339-340 (夹到 >= 0), L419-421 (pocket 只取 data-on), L504-505 (footer 豁免), L698 / L705 (INFIMA_CONTAINER, CENTER_WALL), L718-719 (details off), L765-768 (注释). 其余常量 (DESKTOP_QUERY, POCKET_QUERY, NEKO_*, GUTTER_*, VIEWPORT_QUERY) 与 v1 相同, 所以 v1 §0 常量表照用.

## 2. 推演表 (7 路由 x 5 视口 = 35 行, 要求 2)

列: 舞台 = 会挂到 DOM 的元素; stages = `__nekoDebug().stages` (v2 口径: `.neko-sides` + `.neko-pocket[data-on]`).

| # | 路由 | 视口 | 舞台 | 变体 | 渲染 | reason | 关键数字 | stages |
|---|---|---|---|---|---|---|---|---|
| 1 | / | 375x667 | .neko-pocket | edges -> pocket | 条件出 (回顶按钮隐藏时) | 首屏 pocket-wait-footer; 页底 rendered | 36x93, 底抬 64, x 339-375, y 510-603; 滚到页脚后 data-on; 回顶按钮显示时与之竖向交 4.8px -> pocket-blocked (§4) | 首屏 0, 出现后 1 |
| 2 | / | 390x844 | .neko-pocket | edges -> pocket | 条件出 (回顶按钮隐藏时) | 首屏 pocket-wait-footer; 页底 rendered | 36x118, 底抬 64, x 354-390, y 662-780; 滚到页脚后 data-on; 回顶按钮显示时与之竖向交 4.8px -> pocket-blocked (§4) | 首屏 0, 出现后 1 |
| 3 | / | 768x1024 | .neko-pocket | edges -> pocket | 能出 | 首屏 pocket-wait-footer; 页底 rendered | 44x184, 底抬 72, x 724-768, y 768-952; 滚到页脚后 data-on (pocket 竖向完全落在页脚内, 页脚已豁免) | 首屏 0, 出现后 1 |
| 4 | / | 1280x800 | .neko-sides | edges | 是 (两侧) | rendered | 宽 160, 高 384, x 0-160 / 1120-1280 (与 v1 同) | 1 |
| 5 | / | 1920x1080 | .neko-sides | edges | 是 (两侧) | rendered | 宽 220, 高 528, x 0-220 / 1700-1920 (与 v1 同) | 1 |
| 6 | /anime | 375x667 | .neko-pocket | screen -> pocket | 条件出 (回顶按钮隐藏时) | 首屏 pocket-wait-footer; 页底 rendered | 36x93, 底抬 64, x 339-375, y 510-603; 滚到页脚后 data-on; 回顶按钮显示时与之竖向交 4.8px -> pocket-blocked (§4) | 首屏 0, 出现后 1 |
| 7 | /anime | 390x844 | .neko-pocket | screen -> pocket | 条件出 (回顶按钮隐藏时) | 首屏 pocket-wait-footer; 页底 rendered | 36x118, 底抬 64, x 354-390, y 662-780; 滚到页脚后 data-on; 回顶按钮显示时与之竖向交 4.8px -> pocket-blocked (§4) | 首屏 0, 出现后 1 |
| 8 | /anime | 768x1024 | .neko-pocket | screen -> pocket | 能出 | 首屏 pocket-wait-footer; 页底 rendered | 44x184, 底抬 72, x 724-768, y 768-952; 滚到页脚后 data-on (pocket 竖向完全落在页脚内, 页脚已豁免) | 首屏 0, 出现后 1 |
| 9 | /anime | 1280x800 | .neko-sides (空) | screen | 否 | no-room | 1280 < 1360 (与 v1 同) | 1 |
| 10 | /anime | 1920x1080 | .neko-sides | screen | 是 (两侧) | rendered | 宽 270, 高 648, 推出 37.8px (与 v1 同) | 1 |
| 11 | /anime/details?id=443831 | 375x667 | 无 | off | 否 | off | **v1 -> v2 变化**: ROUTE_RULES L719 显式 off, RouteCats 返回 null; details.tsx 不挂 SideCats, 所以整页无舞台 (新问题 N1) | 0 |
| 12 | /anime/details?id=443831 | 390x844 | 无 | off | 否 | off | **v1 -> v2 变化**: ROUTE_RULES L719 显式 off, RouteCats 返回 null; details.tsx 不挂 SideCats, 所以整页无舞台 (新问题 N1) | 0 |
| 13 | /anime/details?id=443831 | 768x1024 | 无 | off | 否 | off | **v1 -> v2 变化**: ROUTE_RULES L719 显式 off, RouteCats 返回 null; details.tsx 不挂 SideCats, 所以整页无舞台 (新问题 N1) | 0 |
| 14 | /anime/details?id=443831 | 1280x800 | 无 | off | 否 | off | **v1 -> v2 变化**: ROUTE_RULES L719 显式 off, RouteCats 返回 null; details.tsx 不挂 SideCats, 所以整页无舞台 (新问题 N1) | 0 |
| 15 | /anime/details?id=443831 | 1920x1080 | 无 | off | 否 | off | **v1 -> v2 变化**: ROUTE_RULES L719 显式 off, RouteCats 返回 null; details.tsx 不挂 SideCats, 所以整页无舞台 (新问题 N1) | 0 |
| 16 | /docs/关于 | 375x667 | 无 | gutter r -> pocket | 否 | pocket-page-off | pocketOff (DOC_WALL) (与 v1 同) | 0 |
| 17 | /docs/关于 | 390x844 | 无 | gutter r -> pocket | 否 | pocket-page-off | pocketOff (DOC_WALL) (与 v1 同) | 0 |
| 18 | /docs/关于 | 768x1024 | 无 | gutter r -> pocket | 否 | pocket-page-off | pocketOff (DOC_WALL) (与 v1 同) | 0 |
| 19 | /docs/关于 | 1280x800 | .neko-sides (空) | gutter r | 否 | no-room | roomL 0 / roomR 0; **w = max(0, -16) = 0** (v1 为 -16, B17) | 1 |
| 20 | /docs/关于 | 1920x1080 | .neko-sides | gutter r | 是 (仅右) | rendered | 墙 450-1770, roomR 150, w 134, x 1770-1904 (与 v1 同) | 1 |
| 21 | /blog | 375x667 | .neko-pocket | pair -> pocket | 条件出 (回顶按钮隐藏时) | 首屏 pocket-wait-footer; 页底 rendered | 36x93, 底抬 64, x 339-375, y 510-603; 滚到页脚后 data-on; 回顶按钮显示时与之竖向交 4.8px -> pocket-blocked (§4) | 首屏 0, 出现后 1 |
| 22 | /blog | 390x844 | .neko-pocket | pair -> pocket | 条件出 (回顶按钮隐藏时) | 首屏 pocket-wait-footer; 页底 rendered | 36x118, 底抬 64, x 354-390, y 662-780; 滚到页脚后 data-on; 回顶按钮显示时与之竖向交 4.8px -> pocket-blocked (§4) | 首屏 0, 出现后 1 |
| 23 | /blog | 768x1024 | .neko-pocket | pair -> pocket | 能出 | 首屏 pocket-wait-footer; 页底 rendered | 44x184, 底抬 72, x 724-768, y 768-952; 滚到页脚后 data-on (pocket 竖向完全落在页脚内, 页脚已豁免) | 首屏 0, 出现后 1 |
| 24 | /blog | 1280x800 | .neko-sides | pair | 是 (两侧) | rendered | 墙 200-1080, w 196, 高 470 (与 v1 同) | 1 |
| 25 | /blog | 1920x1080 | .neko-sides | pair | 是 (两侧) | rendered | 墙 520-1400, w 320, 高 768 (与 v1 同) | 1 |
| 26 | /search?q=test | 375x667 | .neko-pocket | gutter -> pocket | 条件出 (回顶按钮隐藏时) | 首屏 pocket-wait-footer; 页底 rendered | 36x93, 底抬 64, x 339-375, y 510-603; 滚到页脚后 data-on; 回顶按钮显示时与之竖向交 4.8px -> pocket-blocked (§4) (结果少时页脚首屏即可见, 打开就出) | 首屏 0, 出现后 1 |
| 27 | /search?q=test | 390x844 | .neko-pocket | gutter -> pocket | 条件出 (回顶按钮隐藏时) | 首屏 pocket-wait-footer; 页底 rendered | 36x118, 底抬 64, x 354-390, y 662-780; 滚到页脚后 data-on; 回顶按钮显示时与之竖向交 4.8px -> pocket-blocked (§4) (结果少时页脚首屏即可见, 打开就出) | 首屏 0, 出现后 1 |
| 28 | /search?q=test | 768x1024 | .neko-pocket | gutter -> pocket | 能出 | 首屏 pocket-wait-footer; 页底 rendered | 44x184, 底抬 72, x 724-768, y 768-952; 滚到页脚后 data-on (pocket 竖向完全落在页脚内, 页脚已豁免) (结果少时页脚首屏即可见, 打开就出) | 首屏 0, 出现后 1 |
| 29 | /search?q=test | 1280x800 | .neko-sides (空) | gutter both | 否 | no-room | **v2 变化**: 墙找不到 -> fallback 1140 (vw < 1440), 墙 70-1210, room 70/70, wl = wr = 54 < 120 | 1 |
| 30 | /search?q=test | 1920x1080 | .neko-sides | gutter both | **是 (两侧)** | rendered | **v2 变化**: fallback 1320 (vw >= 1440), 墙 300-1620, room 300/300, w = min(260, 284) = 260, 左槽 x 40-300, 右槽 1620-1880 (v1 为 no-room) | 1 |
| 31 | /404 | 375x667 | .neko-pocket | gutter -> pocket | 条件出 (回顶按钮隐藏时) | 首屏 pocket-wait-footer; 页底 rendered | 36x93, 底抬 64, x 339-375, y 510-603; 滚到页脚后 data-on; 回顶按钮显示时与之竖向交 4.8px -> pocket-blocked (§4) (v1 预估 pocket-blocked, 已因 B15 翻转) | 首屏 0, 出现后 1 |
| 32 | /404 | 390x844 | .neko-pocket | gutter -> pocket | 条件出 (回顶按钮隐藏时) | 首屏 pocket-wait-footer; 页底 rendered | 36x118, 底抬 64, x 354-390, y 662-780; 滚到页脚后 data-on; 回顶按钮显示时与之竖向交 4.8px -> pocket-blocked (§4) (v1 预估 pocket-blocked, 已因 B15 翻转) | 首屏 0, 出现后 1 |
| 33 | /404 | 768x1024 | .neko-pocket | gutter -> pocket | 能出 | 首屏 pocket-wait-footer; 页底 rendered | 44x184, 底抬 72, x 724-768, y 768-952; 滚到页脚后 data-on (pocket 竖向完全落在页脚内, 页脚已豁免) (v1 预估 pocket-blocked, 已因 B15 翻转) | 首屏 0, 出现后 1 |
| 34 | /404 | 1280x800 | .neko-sides (空) | gutter both | 否 | no-room | 404 有 main.container, 墙 70-1210, w 54 < 120 (与 v1 同; 不走 fallback) | 1 |
| 35 | /404 | 1920x1080 | .neko-sides | gutter both | 是 (两侧) | rendered | 墙 300-1620, w 260 (与 v1 同) | 1 |

汇总:
- 桌面 14 格: 渲染 **8** (#4 #5 #10 #20 #24 #25 #30 #35; v1 为 7, 新增 #30 搜索 1920); no-room 4 (#9 #19 #29 #34); off 2 (#14 #15 番剧详情, v1 为 no-room).
- 手机 21 格: pocket-page-off 3 (文档); off 3 (番剧详情, 无舞台); **pocket 15 格** (v1 为 18 格, 详情页 3 格因 off 退出). 15 格首屏都是 pocket-wait-footer, 滚到页底后: 768 档 5 格能出; 375 / 390 档 10 格在回顶按钮隐藏时能出.

## 3. B14-B17 逐条确认 (要求 3)

| 编号 | 证明修了的代码行 (v2 index.tsx) | 修复后推演变化 |
|---|---|---|
| B14 | L698 `const INFIMA_CONTAINER = 1140;` L705 `CENTER_WALL = { ..., fallbackWidth: INFIMA_CONTAINER, sides: 'both' }`; L307 `const fw = cfg.fallbackWidth === INFIMA_CONTAINER && vw >= 1440 ? 1320 : ...`; L719 `{ match: '/anime/details', variant: 'off' }` | /search 1920: no-room -> **rendered** (w 260, 两侧); /search 1280: 仍 no-room (54 < 120), 但 room 从 null 变成 70/70, 报告可读; /404 不变 (有 main.container, 本来就量得到墙); /anime/details: no-room -> **off** (桌面和手机都无舞台, stages 0) |
| B15 | L505 `if (n.closest('.footer')) continue;` (在 pocketBlockers 循环里, 位于 CLICKABLE / POCKET_TEXT 判定之前) | 页底时 pocket 竖向完全落在页脚内 (见 §4), 页脚 li / a / img 不再算障碍 -> v1 预估的 pocket-blocked 翻为能出; 404 三格 (#31-33) 由 "大概率否" 变为能出 |
| B16 | L419 `querySelectorAll('.neko-sides, .neko-pocket[data-on]')`; L421 slots 取 `.neko-slot, .neko-pocket[data-on]` | 手机首屏 (pocket 隐藏) stages = 0, overlaps 不再含隐藏 pocket, 误报消失; 出现后 stages = 1. 另: L434 overlaps 本身也跳过 `.footer`, 所以页底的 pocket 不会因压在页脚上被报 overlap |
| B17 | L334 pair `w = Math.max(0, ...)`; L339-340 gutter `wl / wr = Math.max(0, ...)` | #19 docs 1280: room.w -16 -> **0**; reason 仍 no-room (L646 `!show`). 其余格子 w 本来就 >= 0, 无变化 |

## 4. pocket 重判 (要求 4)

v1 的 "大多数时候 pocket-blocked" 依据是页脚 li 整列宽. B15 后重判:

1. **页脚高度 vs pocket 高度**: 手机端 Infima `.col` 在 <= 996 宽堆成整列. 页脚 = 上下 padding 32+32 + 社区列 (标题 + 1 链接 + 列间距 48) + 空列 (仅间距) + 友情链接列 (标题 + cppreference 36px + 10px + BanGuMi 50px + buildwith 50px + 48) + footer__links 16 + 版权 1-2 行. 粗估 **约 450-550px**. pocket 顶边离视口底: 375 档 64+93 = 157, 390 档 64+118 = 182, 768 档 72+184 = 256. 页底时 pocket 整条落在页脚内, POCKET_TEXT / CLICKABLE 里剩下的只有非页脚元素.
2. **Docusaurus 主题页脚贴底** (`main-wrapper` flex 撑满), 所以即使短页 (404, 结果少的搜索) 不能滚动, 页脚也在视口底部, 同样整条覆盖 pocket 区.
3. **唯一剩余障碍: 回顶按钮** `.theme-back-to-top-button` (在 CLICKABLE 里, 不在 .footer 内): fixed, bottom 1.3rem = 20.8px, right 20.8px, 48x48 -> 占 y [h-68.8, h-20.8], x [vw-68.8, vw-20.8].
   - 375 / 390 档 (底抬 64): pocket 底边 h-64 < 按钮顶 h-68.8 不成立 -> **竖向交 4.8px**, 横向 [vw-36, vw-20.8] 相交 15.2px -> 按钮可见时 pocket-blocked (`button.backToTopButton_xxx`).
   - 768 档 (底抬 72): pocket 底边 h-72, 在按钮顶 h-68.8 之上 3.2px -> 不交.
   - 按钮可见性: Docusaurus 回顶按钮只在 "向上滚且超过 300px 且未到页底" 时显示, 向下滚即隐藏 (visibleRect 检查 visibility, 隐藏时不算). 所以顺着往下滚到页脚 -> pocket 能出; 在页脚可见时往回滑一点 -> 按钮出现 -> pocket 消失. (按钮行为按 Docusaurus 主题源码的记忆, 需实测.)
4. 页脚刚露头 (露出 < 157/182/256px) 时 pocket 还对着正文: 正文 p/li 是整列宽到 vw-16, 与 pocket x 区间相交 -> 这段滚动区间 pocket-blocked, 属预期 (设计就是 "页脚上方" 出现).

**结论: 15 个 pocket 格子预计全部能出** (滚到页底时). 其中 768 档 5 格 (/, /anime, /blog, /search, /404) 无条件; 375 / 390 档 10 格条件出, 回顶按钮显示时被挡. 仍会 blocked 的只有这个场景. 详情页 3 格因 off 不进入这个统计.

## 5. 新发现的问题 (要求 5)

| 编号 | 问题 | 依据 | 影响 |
|---|---|---|---|
| N1 | **番剧详情页整页没有立绘** | L719 设 off, 注释和 Root.tsx 都说 "详情页自己挂 SideCats", 但 details.tsx (30186 B) 里 SideCats / BlogWithCats 出现 0 次 | 桌面 2 格、手机 3 格都是 off, stages 0. 设计文档的 "页内 pair" 没有实现. 要么 details.tsx 挂 `<SideCats />`, 要么去掉 off 让 gutter fallback 接管 |
| N2 | **<= 480 档 pocket 与回顶按钮交 4.8px** | CSS `--pk-lift: 64px` vs 按钮 bottom 20.8 + 48 = 68.8 | 按钮可见时 pocket 消失, 用户回滑时立绘闪退. 把 N 档 lift 调到 >= 72 (与 T 档一致) 即可 |
| N3 | 空 `.neko-sides` 仍计入 stages (v1 §3-5, 未改) | L419 仍数 `.neko-sides` (no-room 时无 data-on 也数) | 只影响判据解读: no-room 格 stages = 1, 要看 reason. 可改成 `.neko-sides[data-on]` |
| N4 | B14 注释 "搜索 / 404 没有 <main>" 不准确 | 404.html 是 `<main class="container margin-vert--xl">` | 仅注释; 404 本来就量到墙, 只有搜索走 fallback |

## 6. 必须本机浏览器 + 重新 build 才能确认 (要求 6)

1. build/ 仍是改动前的旧产物 (无 `.neko-sides` / `.neko-pocket`, `__nekoDebug` 0 个 chunk). 先 `npm run build`, 再对 7 路由 x 5 视口取 `window.__nekoDebug()`.
2. 搜索页 1920 的 fallback 墙 (300-1620) 是否与真实 `.container` 对齐; 搜索框 / 结果列表是否有元素超出 1320 列.
3. 手机端页脚真实高度 (估 450-550px), 以及页底时 pocket 是否完全落在页脚内.
4. 回顶按钮的显隐时机, 以及 375 / 390 档 pocket 是否如推演在回滑时被挡 (N2).
5. 站点是否还有其他 fixed 右下角元素 (Root 挂的 AssLyrics 歌词悬浮窗, 有播放列表时出现) 与 pocket 相交.
6. 滚动条是否占宽 (影响 #20 roomR 150 -> 145.5).
7. 观感: 立绘是否压到内容、路由切换后旧舞台是否卸载、/anime/details 是否接受整页无立绘 (N1).
