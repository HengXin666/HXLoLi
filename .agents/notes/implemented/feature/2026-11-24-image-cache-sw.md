# Agent Note: 图片缓存 Service Worker, 立绘改用 WebP

Status: implemented

Decision-ID: image-cache-sw

## Code

- `src/theme/Root.tsx`
- `src/components/BlogWithCats/index.tsx`

## Problem

番剧海报和两侧探头的立绘每次都要重新加载。用户原话: 「它每次都加载半天, 也不缓存嘞。特别是番剧页面, 以及左右那两个探出头来的二次元角色」。

海报 URL 形如 `<节点>/gh/HengXin666/HXLoLi-ANiMe@<tag>/data/anime/<id>.jpg`。数据仓库几乎每天打新 tag, 节点又会在 cdn.jsdelivr 和 fastly 之间自动切换, 同一张图的 URL 一变, 浏览器 HTTP 缓存就不命中。单张海报约 176 KB (706 px 宽)。

立绘是 RGBA 无损 PNG, 928 x 2227, 单张 470 到 740 KB, 整个目录 27 MB。展示槽最宽 380 px (`NEKO_MAX`)。GitHub Pages 只给 `max-age=600`, 十分钟后又要重新验证。

## Decision

**一个只管图片的 Service Worker, 加上立绘换成展示尺寸的 WebP。**

- `static/sw.js` 拦两类图片, 其它请求不碰。番剧海报和角色图 (`HXLoLi-ANiMe@<tag>/data/(anime|kyara)/...`) 用 `data/` 之后的路径做缓存键, 与节点和 tag 无关, 缓存优先, 30 天后重拉, 最多留 2000 张。立绘 (同源 `/img/catgirl/`) 先给缓存, 后台再拉一次更新。
- 拉海报时用 cors 模式 (jsDelivr 给 `access-control-allow-origin: *`), 拿到可读响应才进缓存; 拿不到就退回原请求, 只是不缓存。
- `Root.tsx` 只在生产构建注册, 本地 dev server 不注册。
- 立绘母版 PNG 不动, `scripts/catgirl-webp.sh` 生成同名 WebP (宽 760, 即 380 px 的 2 倍屏), 平均约 52 KB, 目录合计 2.9 MB。`BlogWithCats` 的 `shownFile` 把 manifest 里的 `.png` 换成 `.webp`, 宽高比不变。

验证 (用户机器的 headless chromium, 本地静态服务): 在线加载一次海报后断网, 换另一个 tag 和另一个节点的同一张海报仍能显示; 立绘断网后从缓存显示。

## Alternatives considered

- 什么都不做: 每天换 tag, 海报缓存每天作废一次; 立绘每张仍要几百 KB。
- 复用现有方案 (`src/utils/anime/dataCache.ts` 的 Cache Storage): 它按 tag 分桶, 换 tag 就清空, 而且只管 fetch 的数据文件, `<img>` 的请求进不去。
- 海报 URL 固定到某个 tag 或节点: 新番海报只在新 tag 里, 固定后拿不到; 固定节点又失去自动切换。
- 立绘用 `<picture>` 同时给 PNG 兜底: 目标浏览器都支持 WebP, 多一层结构没有收益。

## Consequences

- 新增或替换立绘后要重跑 `bash scripts/catgirl-webp.sh`, 否则页面请求的 WebP 不存在, 立绘不显示。
- 数据仓库若替换某部番的海报但文件名不变, 最多 30 天后才在已缓存的浏览器上更新。
- 改 `sw.js` 的缓存规则时把 `VERSION` 加一, 旧桶在 activate 时清掉。
- 海报本身仍是 700 px 左右的原图。要再省流量, 得在数据仓库生成缩略图, 那是跨仓库改动。
