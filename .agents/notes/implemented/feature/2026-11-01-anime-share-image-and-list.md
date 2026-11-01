# Agent Note: 番单分享 (按当前筛选出 PNG 分享图 + 文本清单)

Status: implemented

Decision-ID: anime-share-image-and-list

## Problem

想把"我看过 / 在追哪些番"推荐给别人, 站点只有番剧库页面本身, 没有能直接发出去的图片或文字清单. 要求: 按番剧库当前的筛选结果出 (不是全库), 图可选带不带番剧封面, 另能一键复制番名清单; 站点纯静态 (Cloudflare Pages), 生成只能在浏览器端; 不引入 html2canvas / dom-to-image 这类重库; 不改番剧库页面与筛选逻辑, 不读 localStorage.

封面来源: HXLoLi-ANiMe 的 `data/ANiMeRecord.json` 里是 `anime_data.image_url`, 完整 URL, 全部在 `lain.bgm.tv/pic/cover/l/...` (原图约 2000x2800, 0.9 MB), 该地址不返回 `Access-Control-Allow-Origin`, 画进 Canvas 会污染画布, `toBlob` 失败.

## Decision

新页面 `/anime/share?<与番剧库相同的查询串>`, 只读 `location.search`, 用 `src/utils/anime/filters.ts` 的 `parseFilters` -> `applyFilters` -> `groupRecords` (与番剧库同一套纯函数), 所以部数与顺序天然与番剧库一致; 条件 chips 用 `activeConditions` 的文案. 标题按状态起: 只看过 "我看过的番", 只在看 "我在追的番", 两者 "我看过和在追的番", 其余 "我的番单".

分享图 (`src/components/anime/animeShareImage.ts`) 用 Canvas 2D 自己画, 逻辑宽 1080:

- 带封面 = 封面墙: 列数随部数 4 / 5 / 6 / 8 / 10, 2:3 居中裁切 (object-fit: cover), 左上状态角标、右上序号, 下方中文名 (截断) 与 "季度 · ★评分"; 按首播 / 更新排序时插入与番剧库相同的分段标题.
- 不带封面 = 纯文字版: 不发任何图片请求, 每部一条色块 (序号、中文名、原名、状态 / 季度 / 评分, 左侧色条按状态), 超过 20 部双栏.
- 封面按顺序试: `lain.bgm.tv/r/400/pic/...` (Bangumi 缩图服务, 带 `ACAO: *`, 约 50 KB) -> HXLoLi-ANiMe 镜像 `data/anime/<id>.jpg` (经 `toAnimeCdnUrl`, jsDelivr 与 CF Worker 均带 CORS) -> 带标题的占位块; `crossOrigin = "anonymous"`, 并发 6, 单张 8 秒超时. 失败数在页面提示, 不影响部数与顺序.
- 优先 2x 输出; 画布面积超出约 16M 像素时降到 1.5x / 1x, 仍超就按行切成多张, 序号连续.
- 下载 PNG; 浏览器支持 `navigator.canShare({ files })` 时多一个"分享".

文本清单 (`src/components/anime/animeShareText.ts`): "一行一部" (首行标题与部数, 每行一个名字, 末行附带当前筛选的分享页链接) 与 "逗号分隔" (全角逗号, 中文名里常有半角逗号); 名字为空回退原名. Clipboard API 不可用时退回 textarea + `execCommand("copy")`.

详情页加"分享番单"入口 (默认 `status=watched,watching`). 番剧库页面上的入口需要在 `src/pages/anime/index.tsx` 加一个指向 `/anime/share${location.search}` 的链接, 该文件不在本决策范围, 留给维护它的一方.

## Alternatives considered

- 什么都不做 / 让用户自己截图番剧库: 零成本. 否决: 截图带着工具栏和交互控件, 长列表截不全, 也没法出纯文字版和可复制清单.
- 复用 anime-sedai 的做法 (DOM 截图库 modern-screenshot / html2canvas): 排版直接用 CSS. 否决: 要新增依赖 (html2canvas 约 45 KB gzip), 用户要求先批; DOM 截图同样受跨域图片限制, 并没有省掉 CORS 处理.
- 运行时调 Bangumi API 取封面: 能拿到最新图. 否决: 封面 URL 已在 ANiMeRecord.json 里; API 返回的仍是不带 CORS 的原图; token 只能放在 GitHub Actions secrets, 进前端就是泄露.
- 只用镜像 `data/anime/<id>.jpg`: 已有 CORS, 地址稳定. 否决为首选: 单张平均 343 KB、最大 3 MB, 全库一张图要拉约 55 MB; 保留为第二候选.
- 在番剧库页面上直接加分享按钮与弹层: 入口最顺. 否决: 列表页与筛选逻辑在另一条线手里, 本轮不能动; 独立页面只读 URL, 两边互不干扰.
- SVG -> PNG: 文字排版更省事. 否决: `<image>` 里的跨域封面同样需要先转成 data URL, 还要处理字体嵌入, 比直接 Canvas 2D 更绕.

## Consequences

2026-11-01 更新: /anime/share 已改为可交互番单 (勾选 + 生成链接), 见 `.agents/notes/implemented/feature/2026-11-01-anime-share-link.md`; 本决策的出图与文本清单作为附带功能保留. 2026-11-02 起 AnimeSharePanel 已删除, 出图 (含页内预览, 见 `.agents/notes/implemented/feature/2026-11-01-anime-share-table-view-and-preview.md`) 与复制清单都在 AnimeShareBoard 的分享区, 番剧库弹窗与分享页共用. 新增路由 `/anime/share`. 分享图的部数与顺序完全跟随 `filters.ts`, 那边的排序如果有问题会一起体现 (例如 "最近更新" 按 `last_update` 字符串比较, 而数据格式以星期开头 `Mon 2026-03-30 ...`, 实际按星期排; 需要维护 `filters.ts` 的一方修). `lain.bgm.tv/r/400/` 不是 Bangumi 文档化的接口, 失效时自动落到镜像, 只是更慢. 字体用系统中文字体栈, 不同系统出图字形略有差异. `src/components/anime/AnimeShareBoard.module.css` 是非 AST 资源, 需人工核对. 本地没跑 `tsc --noEmit`, 在同样严格模式的桩环境里零错误, 并用 node-canvas 实际出图核对过 12 部封面墙 / 纯文字版 / 160 部全库三种情况.

## Code

- `src/components/anime/animeShareImage.ts`
