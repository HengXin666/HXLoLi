# Agent Note: 首页改成"左栏个人卡 + 右栏数据流", 图表与关系图懒加载

Status: implemented

Decision-ID: homepage-profile-and-project-index

## Problem

提交 926b31ba76 把首页整页换成项目生态图, 没有自我介绍. 上一轮改回"自我介绍 + 数据 + 项目"后, 实际渲染 (沙箱 Chromium, 1440 / 1280 / 390 宽) 仍有三类问题:

- 重叠: 生态图的连线层 `.edges` 是绝对定位的 SVG, 节点是普通文档流, 没给层级时 SVG 画在节点之上, 连线从节点名上穿过; 项目卡的标签行 `position: absolute; bottom: 22px` 叠在 5 行描述上; 字数图 docs/blog 与总量两条线在 ai-docs 出现前完全重合, ai-docs 线又与增量线的 0 段重合.
- 首屏太长: 三段自我介绍约 190 字, 头图 1441x755 的 PNG 按容器宽 (约 1310px) 铺开, 显示高约 690px, 文件 1.75 MB 且被 `<link rel=preload>` 抢先下载.
- 首页入口包太重: 首页直接引用 recharts、swiper、framer-motion (经 LatestDocsSection) 与 `data/ecosystem.ts`、`data/wordStats.ts`, 关系图默认不显示也照样下载.

第三轮用户反馈又指出五处:

- "最近笔记"只有 docs/ 一栏, 看不到博客 (blog/) 与 AI 文章 (ai-docs/, 路由 /knowledge-base); docs-rss-plugin 只扫 docs/.
- WakaTime 把 system 之和当"编码时长": 实测一天的系统在线时长是 languages 之和的 5 到 20 倍, 含挂机, 两个维度混在一起. 展开图是两张各占半宽的 400px 折线图, 太挤; "时间精度"是原生 select (分钟 / 秒 / 毫秒), 只影响 tooltip, 纵轴始终是小时; "更多"是条件渲染瞬切.
- 近一年项目里有私有仓库 HX-Jungle (匿名访问 404).
- 关系图: 层级做行、7 条产品线做列塞进约 760px 的右栏, 每格只放得下一个节点, 只能往下堆, 又窄又高; 悬浮是原生 title 与连线上的 SVG 的 title 子元素.
- 首页 9 处原生 title 悬浮 (个人卡数字、找到我链接、项目卡与标签、热力格、关系图图例 2 处、连线、节点), 样式不可控, 键盘与触屏都看不到.

## Decision

布局参考 brittanychiang.com: 桌面端 (>996px) 左栏 320px 粘性个人卡, 右栏是内容流; 窄屏退化为单栏. 个人卡从上到下: 头图横幅、头像与站名、一句话自我介绍、"在做"的项目胶囊 (取 `pushedAt` 最近的 5 个, 做法参考 antfu.me 首页的项目胶囊行)、四个数字、两个按钮, 卡下是分节导航, 当前节的短线由 24px 拉长到 48px.

头图换成 `static/img/main_menu_misaka-640.webp` (15 KB) 与 `-960.webp` (26 KB), `srcset` 按 320px 宽选 640w; 显示高度 168px (320 x 755 / 1441), 窄屏固定 160px 并 `object-fit: cover`. 原 PNG 不删, 首页不再引用.

自我介绍从三段压成一句, 存在 `data/profile.ts` 的 `profileLede`. 第二段的"笔记 -> 库 -> 应用"并进这一句, 第三段讲站点迁移历史, 与首屏无关, 删掉.

右栏六节: 写作 (字数图) / 最近更新 / 时间 / 近一年在做 / 在追的番 / 关于. 在追的番由 `HomeAnime` 渲染, 标签与数据口径见 `.agents/notes/implemented/feature/2026-11-01-anime-tags-and-home-anime.md`, 这里只负责放进分节与懒加载. 最近更新是三个 tab: 博客 / AI 文章 / 笔记, 各 6 条, 一行一条、日期右对齐、1px 分隔线 (参考 leerob.com 的列表), 不再走带 framer-motion 入场动画的 LatestDocsSection. 数据在生成侧拆: `plugins/docs-rss-plugin.mjs` 的 `loadContent` 照旧写 docs 一栏, 新增的 `allContentLoaded` 从博客插件与 ai-docs 文档插件已加载的元数据里取 permalink、标题、日期, 写成 `data/latestDocs.ts` 的 `latestBlogs` 与 `latestAiDocs` 两个数组; 不在插件里重算 slug, 链接与站点路由一致. 排除 draft / unlisted / hx_protected. ai-docs 的日期取 front matter `created_at`. `loadContent` 先写时保留文件里已有的两栏, 别的插件加载失败也不会让首页的导入落空.

时间一节分两个维度, 数据在 `WakaTimeChart/wakaData.ts` 统一解析、两个组件共用一次下载: 编码时长 = languages 之和, 使用设备 = system 之和并按 Linux / Windows / Mac 拆开. 编码用主色 #ff88ff, 设备用青色 #5ee7ff. 数据地址改为 jsDelivr (`cdn.jsdelivr.net/gh/HengXin666/HengXin666@main/data/history.json`): 原 raw.githubusercontent.com 路径 2026-11-01 实测 404, jsDelivr 同文件 200 且带 CORS; 它回 max-age 7 天, 所以拉取时加 `?d=当天日期`, 每天最多重新拉一次.

默认是 `Pulse.tsx`: 四个数字 (近一年编码、设备在线、编码占在线、连续编码天数) + 一年 53 周热力格 + 所选区间 (30 / 90 / 365 天) 的占比条. 热力格有"编码时长 / 使用设备 / 叠加"三档: 单看时格子按该维度分位数分四档上色; 叠加时格子底色是设备档位, 中间方块的边长是编码档位, 两种颜色不在同一层. 热力格整体可聚焦, 方向键逐天移动, 每格的日期、编码、设备 (分系统)、前三语言放在统一悬浮卡片里.

"更多 · 展开趋势图"打开 `WakaTimeChart/index.tsx` (重写): 一张满宽 340px 高的图, 维度是两个可同时打开的开关, 两个都开即叠加 (设备按系统堆叠压暗做底, 编码时长是一条粉色粗线, 同单位同一根轴); 只开编码时按前五语言堆叠, 只开设备时按系统堆叠. 原生 select 换成两组分段按钮: 粒度 日 / 周 / 月 (周、月取日均, 数值可比), 单位 小时 / 分钟 (纵轴与提示一起换). 2026-11-01 再加"数值"切换 绝对时长 / 语言占比: 占比模式只画前五语言 + 其他, 每期各语言占当期编码时长的百分比, 用原始秒数按最大余数法取到 0.1%, 合计恰好 100.0 (直接四舍五入会出现 99.9 / 100.1); 纵轴固定 0-100%, 份额最大的语言 (近 365 天第一) 在堆叠最底层, 色带顶边对着纵轴即其份额; 提示按份额降序列出各语言百分比, 第一名高亮, 并给该期编码总时长 (周 / 月是日均); 当期无编码时提示"没有编码记录", 该点堆叠为 0. 占比只对编码有意义 (设备在线不分语言), 所以占比模式下维度与单位按钮置灰, 状态保留, 切回绝对时长恢复. 展开是"充能"动画: 外层 `grid-template-rows` 0fr -> 1fr, 内层 `clip-path` 自上而下展开并由模糊变清晰, 一道主色光带扫过一次, 折线再由 recharts 画出; 约 700ms. `prefers-reduced-motion` 下过渡、光带和 recharts 动画全部关闭, 直接显示. 第一次展开才挂载 (才下载 recharts), 之后收起只是折叠.

项目条带改成纯 CSS 跑马灯: 两份列表 `translateX(calc(-50% - gap / 2))`, 每张卡 4 秒, 悬停与键盘聚焦暂停, `prefers-reduced-motion` 下改为可横滑的静态列表; 卡片是纵向 flex, 标签与日期在文档流里, 不会压住描述. 条带只放公开仓库 (2026-11-01 逐个匿名访问 github.com 核对, 去掉 HX-Jungle); 这份数据没有流水线自动更新, `.github/workflows/` 里没有 schedule/cron, scripts/ 里也没有脚本写 `data/projectData.ts`, 手工维护. 卡片悬停 / 聚焦出统一悬浮卡片 (完整简介、标签含义、徽章与最近推送), 复制出来做无缝循环的那份不挂卡片也不进 Tab 顺序.

条带下方常驻"更多 · 项目关系图"横条, 点开才懒加载 `EcosystemPanel`. 第三轮把关系图重做成浮层: 宽 min(1320px, 100vw - 48px)、高不超过 92vh 的对话框, 行 = 7 条产品线, 列 = 5 个层级 (源头 -> 门户), 列宽按每层单格最多节点数加权 (源头 1.6 : 应用 1.45 : 轮子 0.95 : 平台 0.9 : 门户 0.6). 节点是 24px 高、最宽 168px 的胶囊 (超长名省略, 全名在悬浮卡片), 同一格横向换行; 1440x900 下 7 行全部放进浮层不用滚动, 实测最多的一格 (其他 x 应用, 10 个) 也只占三四行; 旧版 7 列挤在约 760px 的右栏里, 每格只放得下一个节点, 只能竖着堆. 连线默认淡色细线, 推断关系是虚线; 悬停 / 聚焦节点时与它相连的线点亮并流动, 相邻节点描青边, 其余变暗. 图例四种连线是开关按钮. 节点悬浮卡片给简介、技术栈、最近推送和前三条关系及依据 (原连线上的 SVG 的 title 子元素). 点节点再叠详情卡 (`ProjectDetail`, z-index 400 / 401 高于浮层 300 / 301), Esc 先关详情卡再关浮层. 窄屏浮层贴底, 图可横滑 (最小 880px 宽).

悬浮统一走 `src/components/common/Tooltip` (原番剧页的组件就地升级, 旧用法 `selectable` / `style` 不变): 悬停、键盘聚焦、触屏轻点 (非链接) 都能打开, Esc 关闭, 触发元素带 `aria-describedby`, 卡片 `role="tooltip"`; `position: fixed`, 上方放不下翻到下方, 左右贴边时夹在视口内 8px, 箭头跟触发点走; 另有 `useHoverCard` 给热力格这种成百上千个格子共用一张卡片. 首页 9 处原生 title: 6 处换成悬浮卡片 (个人卡数字、项目卡、热力格、图例 2 处、关系图节点), 3 处删掉 (项目卡标签: 含义并进项目卡的悬浮卡片; 连线的 SVG 的 title 子元素: 依据挪进节点卡片与详情卡; 找到我链接: 说明挪进正文, 每个链接下方直接显示). "找到我"每个链接带 react-icons 的 SVG 图标 (FaGithub、SiBilibili、SiLeetcode、FaEnvelope; react-icons 5.5.0 已是依赖).

字数图、WakaTime、关系图都用 `React.lazy` 拆 chunk, 由 `LazyMount` 在进入视口附近才挂载, 占位高度与真实组件一致, 加载失败只在该块显示提示. 字数数据 `wordStats` 与字数图同一个懒加载块. 所有 `import()` 集中在同目录的 `lazyChunks.js`, 类型由 `lazyChunks.d.ts` 提供: 仓库 tsconfig 是 nodenext, `.tsx` 里的 `import()` 按 ESM 解析, 相对路径缺扩展名报 TS2834/TS2835, `@site/*` 映射失效报 TS2307, 而门禁 (`npm run typecheck` / `quality-gate.mjs`) 对比 `scripts/quality-baseline.json`, 新增即失败; `.js` 不在 tsc 检查范围 (未开 allowJs), webpack 照常拆块.

视觉令牌只用站点已有变量: `--ifm-color-primary` / `--ifm-background-color` / `--ifm-color-emphasis-*` / `--ifm-font-family-monospace` / `--ifm-navbar-height`; 字号 11 / 12 / 13 / 15 / 20 / 28 px; 间距取 4 的倍数; 悬停 150ms、展开 240ms, 缓动 `cubic-bezier(0.2, 0.8, 0.2, 1)`; 背景点阵用 CSS 渐变, 不加图片与网络字体.

回档范围不变: 撤掉 926b31ba76 的整页生态图布局, 它的 note `2026-10-31-homepage-ecosystem-graph.md` 已删除; `ProjectDetail.tsx`、`data/ecosystem.ts` 保留 926b31ba76 的数据与结构; `EcosystemGraph.tsx` 第三轮按上文重写, 样式移到 `graph.module.css`, `home.module.css` 只剩详情卡.

## Alternatives considered

- 什么都不做 / 只修三处重叠: 改动最小, 风险最低. 否决是因为用户要求整体重设计, 且只修重叠不解决首页入口包里 recharts、swiper、framer-motion 与两份大数据文件同步下载的问题.
- 复用现有 swiper 轮播, 只把标签改成文档流: 交互现成. 否决是因为跑马灯用 CSS 就能做, swiper 在首页入口里约 87 KB (压缩前), 换掉后功能不变.
- 复用 LatestDocsSection: 少一个列表实现. 否决是因为它为五条链接引入 framer-motion 入场动画, 首页入口因此多约 128 KB (motion-dom + framer-motion, 压缩前).
- 头图保留整幅, 只换 WebP: 体积已降到 26 KB. 否决是因为用户明确嫌它太大, 首屏高度问题不在字节数而在 690px 的显示高度.
- 首屏做成 bento 网格 (个人卡与字数图并排平铺): 一屏信息密度最高. 否决是因为字数图需要较宽的横轴才读得清, 与个人卡并排时两边都挤; 粘性左栏让个人卡一直可见, 右栏图表可以用满宽.
- 关系图做成独立 `/projects` 页面: 首页更短. 否决是因为需求要求关系图收在项目条带的"更多"里.
- 动态 import 补 `.js` 扩展名 / `@site` 换相对路径: 改动最小, tsc 能过. 否决是因为 Docusaurus 3.7 的 webpack 没配 `resolve.extensionAlias`, `../chart/WordCountChart/index.js` 在构建时解析不到真实的 `index.tsx`.
- 把 5 条错误登记进 quality-baseline: 一条命令. 否决是因为这是新引入的错误, 基线只放行存量.
- 最近更新在渲染侧拆 (按 category 过滤 latestDocs): 不动插件. 否决是因为 latestDocs 只来自 docs/, 里面根本没有博客与 ai-docs 的条目, 过滤不出来.
- 在插件里自己扫 blog/、ai-docs/ 并拼 URL: 与现有 docs 一栏的写法一致. 否决是因为博客的 URL 取决于文件名日期、front matter slug 等规则, 自己拼容易与站点路由不一致; allContentLoaded 直接拿插件算好的 permalink.
- WakaTime 保留两张半宽图, 只把 select 换样式: 改动小. 否决是因为两张图各自一根纵轴、各自一个刷选条, 看不出设备与编码的关系, 宽度也还是挤.
- 语言占比用 recharts 的 `stackOffset="expand"`: 一行配置即 0-1 堆叠. 否决是因为它在渲染时归一化, 提示里拿不到百分比, 还得再算一遍; 而且无编码的日子会被除成 NaN. 自己算百分比, 堆叠与提示用同一份数.
- 语言占比单独做一张百分比面积图常驻: 不用切换. 否决是因为展开区已有 340px 的图, 再加一张就超出一屏; 用户也偏好切换.
- 编码与设备画双纵轴: 两条线都"撑满". 否决是因为两者同为时长, 双轴会让编码看起来和设备一样多, 正是第一轮字数图里出过的问题.
- 关系图原地加宽 (挤掉左栏): 不用浮层. 否决是因为左栏是粘性个人卡, 被挤掉后整页布局跳动; 浮层宽度不受两栏限制.
- 引入第三方 tooltip 库 (floating-ui 等): 定位更全. 否决是因为需求不许, 且翻转 + 贴边夹取几十行就够.
- 用 react-force-graph 或三维库重画关系图: 视觉更"科技". 否决是因为力导向布局停不稳分层结构, 且属于重量级新依赖; 本轮不新增任何依赖.

## Consequences

`data/latestDocs.ts` 现在导出三个数组, 由 docs-rss-plugin 在 start / build 时重写. 首页不再引用 swiper 与 LatestDocsSection, 这两者若在其他页面也不用, 可以另行评估移除依赖 (本轮未动 package.json). 换项目只改 `data/projectData.ts`, 条带与"在做"胶囊都按 `pushedAt` 自动排序; 换头图要重新生成两张 WebP. WakaTime 数据仍从 raw.githubusercontent.com 拉取, 热力格与趋势图经 `wakaData.ts` 共用一次下载. `Tooltip` 升级后番剧页 (`src/pages/anime/cv.tsx`、`details.tsx`) 的悬浮卡片改为 fixed 定位并会自动翻转, 外观换成首页同款. `page.module.css`、`home.module.css`、`graph.module.css`、`ProjectCarousel/css.css`、`WakaTimeChart/pulse.module.css`、`WakaTimeChart/trend.module.css`、`common/Tooltip/Tooltip.module.css` 是非 AST 资源, 需人工核对; 两张 WebP 是新增静态资源.

## Code

- `src/pages/index.tsx`
- `src/components/home/HomePage.tsx`
- `src/components/ProjectCarousel/index.tsx`
- `src/components/chart/WakaTimeChart/index.tsx`
- `src/components/common/Tooltip/index.tsx`
- `plugins/docs-rss-plugin.mjs`
