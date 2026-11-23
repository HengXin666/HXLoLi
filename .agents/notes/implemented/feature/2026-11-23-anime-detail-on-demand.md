# Agent Note: 番剧页按需读数据, 并按 tag 持久缓存

Status: implemented

Decision-ID: anime-detail-on-demand

## Code

- `src/utils/anime/animeStore.ts`
- `src/pages/anime/details.tsx`

## Problem

番剧详情页只展示一部番, 却要先把 HXLoLi-ANiMe 的整份 `data/ANiMeRecord.json` 拉下来。这份文件解压后约 15.5 MB, CDN 压缩传输约 1.9 MB。在用户的机器上实测, 冷加载要 6 到 15 秒, 第二次打开也要 3 秒以上。

第二次也慢, 是因为 CDN 节点会被自动优选切换 (cdn.jsdelivr 和 fastly 之间), URL 一变, HTTP 缓存就不命中, 整份文件重新下载。

用户原话: 「为什么每次我去加载都感觉很慢呢? 至少要等 10 秒, 即便在我的网络很好, 而且接入了代理的情况下」「尽量优化它」。

## Decision

**三层改动: 数据按需拆分, 页面按需读取, 读过的按 tag 存本地。**

- 数据仓库在爬虫落盘时生成 `data/record/<id>.json` (单部完整记录) 和 `data/ANiMeIndex.json` (全部番的轻量列表, 没有角色, 每集只有类型和时长等字段)。决策记在 HXLoLi-ANiMe 仓库的 anime-record-split 记录里 (2026-11-23)。
- `useAnimeRecord(id)` 只拉这一部; 拿不到时先用轻量列表判断库里有没有, 没有就直接 404, 有 (旧 tag 没有单部文件) 才退回全量。
- `useAnimeRecordsState(baseUrl, "lite")` 读轻量列表。列表页, 系列页, 推荐分类页, 分享板, 首页番剧块和详情页的相似推荐用它; 声优页和关系图要角色, 继续用默认的 `"full"`。全量已在内存时, 轻量范围直接复用全量。旧 tag 没有索引文件时退回全量。
- `src/utils/anime/dataCache.ts` 把读过的数据文件存进 Cache Storage, 桶名带 `ANIME_CDN_TAG`。同一 tag 下文件不变, 所以命中就不走网络; 换 tag 时删掉旧桶。拿不到 Cache Storage 时直接走网络。
- 详情页相关条目和相似推荐的海报从 CSS 背景图改为 `<img loading="lazy">`, 首屏封面加 `fetchPriority="high"`, 不再让十几张 200 到 500 KB 的封面和首屏抢带宽。

实测 (用户机器, 本地 dev server):

- 只加缓存时: 冷加载 4 到 10 秒, 第二次 0.5 秒。
- 单部文件 + 轻量列表 (用本地静态服务模拟新 tag): 详情页冷加载约 0.6 到 0.7 秒, 不再请求 ANiMeRecord.json; 不存在的 id 直接 404; 列表页和系列页只读 ANiMeIndex.json; 声优页仍读全量。

## Consequences

- 新 tag 发布前, 详情页会先请求两个不存在的文件再退回全量, 冷加载比以前多 3 到 4 秒。数据仓库流水线打出新 tag, HXLoLi 合入 cdnVersion PR 之后就走快路径。
- 轻量列表没有角色和每集标题 / 简介。列表类页面以后要用这些字段, 要么在数据仓库 `split.py` 里放开, 要么改用 `"full"`。
- Cache Storage 每个 tag 多占约 15 MB (读过全量时) 或几百 KB (只读过轻量和单部)。
- overlay (dev 编辑器) 和标注到达都会刷新全量, 轻量和单部三份内存数据, 页面通过同一个通知重读。

## Alternatives considered

- **什么都不做 / 复用现有全量文件**: 不用改数据仓库, 最稳。但首次打开和每次换 tag 后仍要下载 15 MB, 用户的抱怨正是这个场景, 所以不够。
- **只加前端缓存**: 已经做了, 第二次打开从 3 到 6 秒降到 0.5 秒。但冷加载不变, 每次数据更新换 tag 后又回到慢路径。
- **把 HTTP 缓存交给浏览器, 固定 CDN 节点**: 节点优选是为了在不同网络下选最快的, 固定它会伤到其他用户; 而且首次加载仍是全量。
- **详情页先显示列表里的摘要再补全**: 仍要先拿到全量或轻量列表才能显示任何东西, 不如单部文件直接。
