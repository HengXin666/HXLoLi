# Agent Note: 番剧手工标签表 (萝莉为重点) + 按标签筛选 + 首页追番块

Status: implemented

Decision-ID: anime-tags-and-home-anime

## Code

- `src/components/anime/AnimeCard/index.tsx`
- `src/components/home/HomeAnime.tsx`

## Problem

番剧数据在 HXLoLi-ANiMe 仓库, 本站运行时经 CDN (按 `data/animeCdnVersion.ts` 的 tag) 拉 `data/ANiMeRecord.json` (2026-11-01 约 12.5 MB, 160 条). 每条是 `anime_data` (id 即 BanGuMi subject id、name、name_cn、short_summary、score、image_url、eps、date、summary、total_episodes、characters、relations、episodes) 加 `user_status` (watch_status、watched_eps、last_update、comment、tags). 唯一的分类来源是 `user_status.tags`, 160 部里只有 9 部打了标签 (异世界 5、萝莉 3、日常 / 百合 / 舰娘 / 妹控 / 纯爱 各 1), 番剧页也不能按标签筛. 角色的 `tags` 是 BanGuMi 资料框 (性别、身高、生日等键值), 不是萌属性, 推不出"萝莉". 首页没有任何番剧内容.

## Decision

标签落成本站的一张表 `data/animeTags.ts`: `ANIME_TAG_DEFS` 定义 8 个分类及判定口径 (萝莉 / 异世界 / 日常 / 恋爱 / 搞笑 / 战斗 / 催泪 / 悬疑, 萝莉排第一), `ANIME_TAGS` 以 bgm subject id 为键列出每部番的分类, 行尾注释写中文名只为人读. 2026-11-01 按番剧数据里的名字逐部人工判定, 157 部有表内标签: 战斗 41、异世界 35、萝莉 34、日常 30、搞笑 29、恋爱 17、悬疑 11、催泪 6. 萝莉口径是"主角或核心角色是小学生年纪 / 幼女外形 (含合法萝莉), 且是作品卖点之一; 只在回忆或 Q 版里出现的不算", 所以芙莉莲、我推的孩子、异世界四重奏、碧蓝航线微速前行这类没打.

渲染统一走 `src/utils/anime/tags.ts` 的 `getRecordTags`: 表内分类 (按定义顺序) + `user_status.tags` (原顺序) 合并去重, 两个来源都只读. 番剧卡、详情页、首页都用它. `countTags` 给筛选条与首页计数.

番剧页 (`src/pages/anime/index.tsx`) 在两组页签下加 `TagFilter` 标签条 (与 `Tag` 同目录) (单选, 带部数, 选中本站分类时显示口径); 筛选写在 URL `?tag=` 上, 首页与详情页的标签都链到 `/anime?tag=...`. 观看状态页签末尾加"全部" (`ALL_WATCH_STATUS = 0`, 不进番剧仓库的 WatchStatus 枚举), 带 tag 进来时切到全部, 否则从首页点"萝莉"只能看到在看的那几部. 筛选后为空时显示一行说明, 不再落到占位卡.

首页新增"在追的番"一节 (`HomeAnime`): 一行状态计数 (记录总数 + 看过 / 在看 / 搁置 / 想看 / 抛弃) + 最近更新进度的 6 部在看封面 (名字、进度、评分, 点进详情页) + 标签入口 (按 countTags 前 10 个, 萝莉高亮). 数据与番剧页同源 (`useAnimeRecords`), 没有任何硬编码数字. 组件经 `lazyChunks.js` 的 `loadHomeAnime` 拆块, 由 `LazyMount` 滚到附近 (rootMargin 300px) 才挂载, 首屏不下载那份 JSON.

## Alternatives considered

- 什么都不做 / 复用现有 `user_status.tags`: 零改动, 数据本来就在番剧仓库. 否决是因为只有 9 部有标签, 萝莉只有 3 部, 筛出来不成一类; 而且那边是追番时手打的, 本站补分类不该去改别的仓库的数据.
- 把标签写回 HXLoLi-ANiMe 的 ANiMeRecord.json: 数据只有一处. 否决是因为那份 JSON 由那边的脚本从 BanGuMi 同步生成, 手改会被覆盖, 本站还要等 CDN tag 发版才看得到.
- 从角色资料 (年龄 / 身高) 自动推萝莉: 不用人工. 否决是因为资料框字段稀疏 (年龄只有 214 条), 且设定年龄与外形 / 卖点经常不符, 判不准.
- 首页画状态或评分分布图: 一眼有图. 否决是因为 160 部里看过 134 部, 状态分布就是一根长条, 一行数字更清楚; 评分集中在 6 到 8 分, 图也说不出东西; 还要在首页再拉一份 recharts.
- 首页做构建期摘要 (只放计数与 6 张封面的小 JSON): 不用拉 12 MB. 否决是因为数据在另一个仓库、经 CDN tag 更新, 本仓库没有生成它的流水线, 手工快照就是硬编码数字; 懒挂载已让首屏不受影响.

## Consequences

新番入库后不在表里只是没有分类, 不报错; 补标签就在 `data/animeTags.ts` 加一行. 首页番剧块要在滚到附近时下载约 2.4 MB (gzip) 的 ANiMeRecord.json, 与番剧页共用模块级缓存, 进番剧页不再重复下载; CDN 拉取失败时该块一直是骨架 (animeStore 没有错误回调, 本轮未改). 观看状态的 localStorage 现在可能存 0 (全部). 萝莉判定是人工口径, 有争议的几部 (间谍过家家、GJ部、请问您今天要来点兔子吗、熊熊勇闯异世界、打了300年的史莱姆) 按"核心角色 + 卖点"收入, 改口径时直接改表. `src/pages/anime/details.tsx` 的标签改用 getRecordTags 并链到筛选, 与番剧页同目录. `src/components/home/page.module.css` 新增 anime* 样式, 是非 AST 资源, 需人工核对.

2026-11-01 晚: 番剧页的单选标签条、"全部"页签与带 tag 切状态的 effect 已被 `2026-11-01-anime-page-refactor.md` (URL 化组合筛选) 取代; 萝莉复核后为 35 部 (兔子两季移除, 学生会的一己之见两季与 NEKOPARA OVA Extra 补上), 新增"校园".

2026-11-02 晚: 接上爬虫源标签. `src/utils/anime/animeStore.ts` 拉 `/data/ANiMeTags.json` (与 ANiMeRecord.json 同一个 ANIME_CDN_TAG); 当前 tag HX-20261101141415-poqRvw 早于该文件, CDN 404, 所以兜底读站点自带的 `static/anime/ANiMeTags.json` (HXLoLi-ANiMe data/ANiMeTags.json 的快照, 160 条, 2026-11-02 00:24 生成), 两处都拉不到就静默降级. `src/utils/anime/tags.ts` 的 getRecordTags 合并为 源标签 ∪ user_status.tags ∪ data/animeTags.ts 手工表 (别名归并、去重, overlay 仍是最终结果), 源标签晚到时 setSourceTags 清缓存并推一份新数组让页面重算. 下次 animeCdnVersion 更新到含 ANiMeTags.json 的 tag 后, static 快照可删.
