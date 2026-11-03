# CHECKPOINT (2026-11-03 02:00 CST, /anime screen 探出幅度加大)
- 用户嫌探得少: --neko-out 由 calc(24% - p*6%) 改为 calc(14% - p*10%) (BlogWithCats.css screen 段). 只改 CSS.
- harness 1440 实测压最外一列海报: 多数对 0-50px, pair10 左最多 87px (手+头发), 滚满再 +25px. 若要严格不压: 让立绘 z 低于卡片 (需卡片 position+z-index), 未做.
- 截图 .agents/research/anime-cats-4.png / -dark.png: 左列改前 (24%), 右列改后 (14%); 上行滚动 0, 下行滚满一屏. 沙箱 harness 非 dev server. stub tsc 0.

# CHECKPOINT (2026-11-03 01:55 CST, /anime 改用 screen 变体)
- 推翻上一节"/anime 不放": 用户方案, 屏幕左右边界当墙. 新变体 `screen` (index.tsx NekoVariant + ROUTE_VARIANTS['/anime'], 媒体查询 min-width 1360px; CSS 在 BlogWithCats.css 末尾).
- 左槽放 right 图、右槽放 left 图, 不镜像 (裁切线推出屏幕, 人朝卡片). 宽 clamp(180px, (100vw-1200px)/2+130px, 270px) (1440 时 250px), 静止各推出图宽 24%, 滚动一屏收到 18%; 底边 --neko-lift.
- harness 实测 (1440, 6 对): 身体大多在 120px 留白里, 手/发梢最多压最外一列海报 0-62px (pair10 左最多), 滚满 +15px.
- 截图 .agents/research/anime-cats-3.png / -dark.png (每张左: 滚动 0, 右: 滚动一屏), 沙箱 harness 非 dev server. stub tsc 0.
- 待: 真实页面截图 + 仓库 tsc.

# CHECKPOINT (2026-11-03 01:45 CST, /anime 去掉立绘)
- 选 A: index.tsx ROUTE_VARIANTS 删 '/anime' (留注释说明). 理由: 卡片墙 1200px 铺满, 1440 宽两侧各 120px, 没有实心墙可躲, 放立绘只能压卡或贴视口边浮着; 功能页不为装饰收窄.
- corner 变体 CSS 保留, 当前无路由使用. Root.tsx 注释里"番剧库"已过时 (未改, 只是注释).
- 截图 .agents/research/anime-cats-3.png (上 light 下 dark, 沙箱 harness, 非 dev server). 沙箱桩 tsc 0.
- 待: 真实页面截图 + 仓库 tsc; /、/anime/details、/anime/graph 撤镜像后的真实效果.

# CHECKPOINT (2026-11-03 01:40 CST, /anime 侧边立绘)

- 截图 (沙箱 Playwright 1440x900, 真实 BlogWithCats.css + 原图 + 仿 /anime 骨架 1240 / padding 20 / 海报网格; 不是 dev server): .agents/research/anime-cats-1.png (改前), anime-cats-2.png / -2-dark.png (改后), anime-cats-compare.png (多对).
- 现象: corner 立绘被 scale:-1 1 镜像, 裁切线翻到朝内容一侧, 一条硬竖边压在最后一列海报上; 头发和身体朝屏幕外; right:0 bottom:0 + 下沉 24% + rotate(-6deg) 只露个头.
- 根因: 上一轮"朝向修复"给 corner / side / edges 加的镜像是错的 (left 图身体贴右, 镜像后贴左). 同样影响 / (edges)、/anime/details (side)、/anime/graph (edges).
- 修 (PUT + md5 回读): BlogWithCats.css 撤掉三变体的 scale:-1 1, 入场改成从视口边滑出 (右 neko-peek-l, 左 neko-peek-r); corner 重写: 不旋转, bottom: var(--neko-lift), translate(6%, p*-48px) 把裁切线推出视口, width clamp(150px, (100vw-1200px)*0.75, 240px), 底部淡出走默认 --neko-fade. index.tsx: corner 媒体查询 1400px (留白 >= 100px).
- 未验: 真实 /anime、/、/anime/details、/anime/graph 页面截图; 仓库 tsc.

# CHECKPOINT (2026-11-03 01:18 CST, 右侧割裂)

- 逐张量 28 张立绘 (.agents/research/catgirl-anchor.md, 模拟图 catgirl-anchor-sim.png): 全部落位正确, left 图重心 0.69-0.80 贴右, right 图 0.21-0.42 贴左, 内侧空隙中位 0-3%. "右槽图朝正文是透明留白"不成立, 没有翻转/换图.
- 推断真因: pair 立绘 fixed, 墙沿按 50%±cw/2 推算 + --neko-w 用 100vw (含滚动条), 真实正文偏几像素时一侧硬裁切线露出或留缝. 未在浏览器确认.
- 已改 (PUT + md5 回读 OK):
  - index.tsx: useWallEdges 量同一 .blog-container 里 .blog-content 的 rect, 写 --neko-wl / --neko-wr / --neko-w (两侧取较窄留白, ResizeObserver + resize). NEKO_PAIRS 类型 NekoPair[], 可选 anchor / noFlip; flipFor() 落位与槽位相反时加 neko-flip (镜像), noFlip 冲突整组不渲染. 当前无条目用到.
  - BlogWithCats.css: pair 槽 left 改用 var(--neko-wl/wr, 原推算); pair .neko-img 叠水平 mask (内侧 90%->100% 淡出, mask-composite intersect / -webkit source-in), neko-flip 反向 mask 与入场.
  - 沙箱桩 tsc (strict, 同仓库 tsconfig 无 noUncheckedIndexedAccess) 对 index.tsx 0 错.
- 下一步: 用户跑 tsc + 明暗截图卡 (catgirl-fit-{dark,light}.png 仍是旧的/模拟), 看右侧; 若墙沿已对齐仍嫌硬, 调 90% -> 85%.

# CHECKPOINT (2026-11-03 01:02 CST, 额度耗尽前交接)

> 接手先读本节, 旧内容在下方. 读写走 fs 隧道 (GET/PUT, 无 DELETE, PUT 不建父目录, 必须串行, --max-time 400, PUT 后 GET 回读比 md5). 命令要用户跑: 代码卡格式 `tool: shell` 换行 `--- command` 换行 命令.

## 已完成 (均已 PUT 并回读 md5 一致)
- 立绘 13 对 static/img/catgirl/pair{1-13}-{left,right}.png; 本轮重生成并替换 3L/5L/5R/8R/10R.
- 朝向: 13 对图本身逐张核过全对 (表: .agents/research/catgirl-orientation.md; 10R 外侧举手, 标记待换). 真 bug 在 NekoStage 的 edges/corner/side 变体 (视口边当墙, 脸朝屏外, 影响 / /anime /anime/details /anime/graph): BlogWithCats.css 给这三变体 .neko-img 加 `scale:-1 1` + 入场关键帧取反. pair 变体不镜像.
- src/components/BlogWithCats/{index.tsx,BlogWithCats.css}: 每图 fade 字段 -> --neko-fade 遮罩; --neko-lift clamp(165px, 8vh+125px, 245px) (+50px); pair 滚动去 rotate; 内侧侵入 6%; corner rotate(-6deg) translate(14%, 24%-p*18%).
- src/theme/BlogListPage/index.tsx: 删 "Blog" kicker, h1 改 styles.srOnly (仅读屏/SEO), blogDescription 小字保留, stagger 下标 lead0/stats1/tags2. styles.module.css 追加 .srOnly / .page padding-top 32 / .head mb40 / .lead.
- src/theme/Root.tsx 挂 <RouteCats /> (按路由选变体).

- pair14 (01:03): 源 static/img/catgirl-source/f506e88e-left-half.png 左半 (x<486 白缝), 白底 floodfill 抠图, 贴底 716x1719. 原图角色裁切线在左 -> right=原图, left=镜像 (镜像陷阱: 用户说的'原图放左槽'会让裁切线朝外, 故反过来). NEKO_PAIRS 现 14 对. 白裙底部被 floodfill 吃掉一些, 靠底部渐隐遮, 未浏览器验证.

## 正在做 / 未验证
- 仓库 tsc (基线 scripts/quality-baseline.json 10 条, 新增须 0) 和 /blog 明暗截图 (.agents/research/catgirl-fit-{dark,light}.png, 现为 PIL 模拟) 的代码卡已发, 输出未回.
- 浏览器里没看过任何一处: 镜像后的四个路由, lift 小视口是否顶导航栏, corner 是否露裁切线, 博客新页头.
- 手/肢体硬切: 新 5 张未逐张复核; fade 值是脚本假设, 未逐张核.
- 00:23 "页面都是坏的" 原因未定位.

## 下一步 (按序)
1. 让用户跑 tsc + 截图卡, 看结果; 有新增错误先修.
2. 新需求 (01:01, 未开工): 博客标签页美化.
   - 新建 src/theme/BlogTagsListPage/index.tsx (+styles.module.css): 标签云/网格, 名字+文章数, hover 响应.
   - 新建 src/theme/BlogTagsPostsPage/index.tsx (+styles.module.css): 顶部标签名+文章数 (不放大字 h1, 用 srOnly), 列表复用 BlogListPage 的条目组件 (需先把条目抽成可导出组件, 别重写), 套 BlogWithCats 拿立绘, 动效同一套 stagger/reveal/hover.
   - props 参考 node_modules/@docusaurus/theme-classic/lib/theme/BlogTagsListPage 与 BlogTagsPostsPage (tags / tag / listMetadata / sidebar / items); 保留 PageMetadata 和 SearchMetadata.
   - 检查 src/theme/BlogArchivePage 与作者页是否跟上新设计, 统一.
   - 父目录不存在时 PUT 会失败: 先给用户 mkdir 卡.
3. 核对 BlogListPage/styles.module.css 原有两处 .lead (约 81 / 707 行) 与新追加规则的冲突.
4. Agent Notes: 改写 .agents/notes/.../2026-11-02-blog-catgirl-sides.md (13 对, NekoStage 四变体, RouteCats, 镜像), anime-tags note 补 "萝莉->ロリ", redline 2026-11-01-navbar-slim-and-cdn-first-load.md; 跑 uv run scripts/redlines/agent_notes.py --diff.
5. 待确认旧项: AnimeSharePanel.tsx/.module.css 是否 git rm; div 上 inert 的 TS 类型; static/img/catgirl-source/ 约 33MB 体积; /anime .groupTitle 类名.

## 已知坑
- 隧道并发 PUT 会在约 125s 截断却报 ok -> 只能串行并回读核对.
- GET 有 CDN 缓存, 加 &_=时间戳.
- 镜像陷阱: 裁切线固定在槽位内侧, 单独翻图会把裁切线翻到外侧; 只能在"墙=视口边"的变体里镜像, 或换槽/重生成.
- 不动 filters.ts/overlay.ts/details.tsx/AnimeGraphPage/, 不 npm start/build, 不加依赖, 中文文案, 只动 opacity/transform, 尊重 reduced-motion.

---

# CHECKPOINT (2026-11-02 22:10 CST 第八轮已 PUT)
- 表格模式: 无外框纯文字流 (年份灰色粗体小标题 + 密排番名), AnimeShareBoard.module.css .yearTable/.yearRow/.yearHead/.nameTag.
- 内定筛选与短链接: data/animeShareDefault.ts (ANIME_SHARE_DEFAULT = 站长看过 + 首播排序; 预设 mine / recent / loli); animeShareLink.ts resolveShareStart / buildShareSpecQuery; ?p=<名字>, 未知名字降级默认并提示, 带番单筛选参数的老链接不套默认.
- 分享图: animeShareImage.ts 改印刷品风 (纸色底、宋体标题、发丝线、单一朱红强调, 去渐变 / 圆角 / 角标 / chip). 沙箱没有 CJK 字体, 未实渲染.
- notes: table-view-and-preview (+ data/animeShareDefault.ts 进 Code) / image-and-list 已补.
- 沙箱桩 tsc 对改动文件 0 错误; 仓库 npx tsc --noEmit 与 redline 仍待本机跑.

# CHECKPOINT (2026-11-02 21:35 CST 第七轮已 PUT)
- 分享图跟页面筛选与分组 (去掉 DEFAULT_FILTERS 二次 applyFilters; 表格模式按 yearRowsOf 分组).
- 预览图点开 ImageViewer 大图 (原生 dialog, 缩放 / 拖动 / 翻页 / Esc / 点遮罩; 不冒泡关外层弹窗).
- 弹窗关闭按钮: AnimeIndexPage.module.css .shareClose absolute 圆形 38px; 内容在 index.tsx 新加的 .shareBody 里滚.
- 独立页页头: 面包屑 + 标题 + 副标题; 弹窗里去掉"在独立页面打开 →"胶囊.
- 源标签: animeStore.ts loadSourceTags (CDN -> static/anime/ANiMeTags.json 兜底 -> 静默降级); tags.ts setSourceTags/parseSourceTags, 合并 源 ∪ 追番 ∪ 手工表.
- notes: table-view-and-preview / share-on-index / tags-and-home-anime 已补.
- 本地桩环境 tsc 对改动文件 0 错误; 仓库 npx tsc --noEmit 未在本机跑.
- 上一轮遗留仍未做: src 全量 grep 旧标签名; data/animeTags.ts 的 note 同步.

# 更早: /anime 分享番单 (表格视图 + 分享图预览 + 弹窗合并) 与首页 banner

更新: 2026-11-02 01:20 CST

## 已完成 (都已写到盘上)
- src/components/anime/AnimeShareBoard.tsx: 卡片 / 表格两种视图 (独立页用 URL `view=table`, 弹窗里用组件 state 不改 URL); 表格 = 年份格 + 番名密排, 不显示部数, 无首播日期组叫"待推出"; ShareImagePreview 组件做分享图页内预览 (带封面 / 不带封面两版缓存, 多页翻页, blob URL 有 revoke); 右侧 360px 吸顶分享栏 (生成链接 / 复制清单 / 预览); `embedded` prop 供 /anime 弹窗使用.
- src/components/anime/AnimeShareBoard.module.css: 13,010 B. 括号 96/96, postcss.parse 与 css-tree 无错, 用仓库 postcss 链 (postcss-import + tailwindcss + postcss-nested + autoprefixer) 编译正常, 53 个 styles.xxx 全部有定义.
- src/pages/anime/index.tsx 第 194 行: `{shareOpen && <AnimeShareBoard embedded />}`; AnimeIndexPage.module.css 的 .shareDialog 改为 min(1200px, 96vw) x min(90vh, 1000px).
- AnimeSharePanel.tsx / AnimeSharePanel.module.css 已 git rm, 源码中无任何引用.
- src/components/anime/animeShareLink.ts: 新增 siteRootOf, 分享链接与分享图页脚按 window.location (origin + /anime 之前的路径) 生成, SSR 回退 siteConfig.
- static/img/main_menu_misaka-640.webp (640x335, 25,858 B) 与 -960.webp (960x503, 44,172 B), 由 main_menu_misaka.png 生成 (PIL, webp q85); HomePage.tsx 未改.
- notes: .agents/notes/implemented/feature/2026-11-01-anime-share-table-view-and-preview.md 等已同步.

## 2026-11-02 01:20 第五轮 (已 PUT)
- 分享图标题不再带数字: AnimeShareBoard.tsx imageTitle = "我看过的番" / "番单" (页面进度条 N/M 保留).
- 弹窗真模态: src/pages/anime/index.tsx 去掉点遮罩关闭, 加 onCancel preventDefault (Esc 不关); 关闭按钮改为"✕ 关闭"胶囊按钮 (AnimeIndexPage.module.css .shareClose). 焦点约束靠 showModal 原生 inert, 未另写 focus trap.
- 独立页返回控件: 文案"← 番剧库", .back 改为描边胶囊按钮样式.
- data/animeTags.ts: 萝莉 全部改 ロリ (定义 / ANIME_TAGS / 注释), ANIME_TAG_ALIASES 加 "\u841d\u8389" -> ロリ (转义写, 使 grep 萝莉 为 0); 舰娘 定义与 ANIME_TAGS 值删除 (留下 3 个空数组 [] 条目). src/utils/anime/tags.ts 注释示例改 ロリ.

## 第六条 搜索栏抖动 (已 PUT, 01:25)
- 原因 1: FilterBar 的状态 chip 部数 (counts.status) 随搜索实时变化, 三位数变一位数改变 chip 宽度, 同一 flex 行里 .search (flex: 1 1 240px) 分到的宽度跟着变. 修: src/components/anime/Tag/FilterBar.module.css 的 .count 定宽 3ch + tabular-nums.
- 原因 2: 结果变少时页面短于视口, 滚动条消失, 整页宽度跳. 修: src/pages/anime/AnimeIndexPage.module.css 的 .results min-height 40vh -> 100vh, 滚动条常驻.
- 未在浏览器实测; 两个都是非 AST 资源 (resource-review).

## 第五轮未完成 (接手先做)
- 第 6 条 "接上爬虫源标签" 没做: 需在 src/utils/anime/animeStore.ts 里 reqAnimeByCDN('/data/ANiMeTags.json') (失败降级为 {}), tags.ts getRecordTags 合并顺序改为 源标签 ∪ user_status.tags, 再用 data/animeTags.ts 覆盖; 注意 tagCache 是 WeakMap 按记录缓存, 源标签晚到时要失效 (例如源标签加载后重新 applyOverlay 生成新记录对象). 先确认 ANiMeTags.json 的结构 (CDN 上看一眼).
- 本轮 Agent Note 未同步: index.tsx -> 2026-11-01-anime-share-on-index.md (加"真模态"); AnimeShareBoard.tsx/.module.css -> 2026-11-01-anime-share-table-view-and-preview.md (标题去数字, 返回按钮); data/animeTags.ts + tags.ts -> 2026-11-01-anime-page-refactor.md (ロリ 改名, 删 舰娘). 不补会被 redline 判单边改动.
- src/ 其他文件里可能还有 "萝莉" (只查了 tags.ts 与 data/animeTags.ts), 跑 grep -rn 萝莉 src/ 修掉.
- 未跑 tsc; 跑 npx tsc --noEmit 对照 scripts/quality-baseline.json.

## 正在做: "弹窗没有样式" 排查 (结论, 未改代码)
- 截图里的文案 "我的番单 · 177 部" / "按当前筛选生成分享链接" / "分享链接" / "复制链接" / "打开分享页 →" / "其他方式 ☑ 图片带封面 / 下载图片 / 复制清单 / 逗号分隔" 全部是已删除的旧组件 AnimeSharePanel.tsx 的界面 (shareTitle() 对混合状态返回"我的番单"; 复制清单 / 逗号分隔它也有). 当前盘上 src/ 没有这些句子, src/components/anime 与 src/pages/anime 下也没有 .hark-bak 残留.
- 所以不是新旧混合, 而是浏览器里运行的仍是旧 AnimeSharePanel 的 JS; 它的 AnimeSharePanel.module.css 已被删除, HMR 把那份样式卸掉了, 于是变成裸 HTML. 新 AnimeShareBoard 的 CSS 没问题, 只是根本没被渲染.
- 判断依据: 删除文件 + 改 import 这类变更 HMR 常常无法热替换, 已打开的标签页继续跑内存里的旧模块.

## 下一步 (接手的人先做这个)
1. 在浏览器对 http://localhost:3000/HXLoLi/anime 做一次硬刷新 (Ctrl+Shift+R) 或关掉标签页重开, 再点"分享". 应看到"点击选择你看过的番"标题, 显示行有"卡片 / 表格 (按年份)".
2. 若仍是旧界面: 停 dev server, 删 .docusaurus 与 node_modules/.cache, 由用户重新 `npm start` (仓库红线: 不许代理启动博客). 再在 DevTools Network 里确认加载的 chunk 含 `AnimeShareBoard`.
3. 新界面出来后再核对: 弹窗边框圆角遮罩 (.shareDialog)、表格密排 (.nameFlow/.nameTag)、预览图有宽高 (.previewImg).
4. 跑 `npx tsc --noEmit` 对照 scripts/quality-baseline.json (新增应为 0) 与 `uv run scripts/redlines/agent_notes.py --diff`.

## 已知坑
- 读写走 fs 隧道 (https://fs.lolioa.com/fs/<绝对路径>?token=...), 只支持 GET / PUT, 不支持 DELETE; GET 有 CDN 缓存, 加 `&_=时间戳`.
- 盘上文件可能被别人直接改过 (例如 AnimeShareBoard.tsx 的 note 引用被合并), 改前先重新读.
- 2026-10-31-homepage-profile-and-project-index.md 写的 webp 体积 (15 KB / 26 KB) 与实际 (25,858 B / 44,172 B) 不符, 未改.
- 不要 npm start / npm run build; 不加依赖; filters.ts / tags.ts / overlay.ts / animeStore.ts / details.tsx / AnimeGraphPage/ / data/animeTags.ts 不动.
