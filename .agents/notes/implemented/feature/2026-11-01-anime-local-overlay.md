# Agent Note: 番剧本人标注: 本地 overlay 模式

Status: implemented

Decision-ID: anime-local-overlay

## Problem
标签 / 观看状态 / 进度 / 评分 / 评论只能去 HXLoLi-ANiMe 仓库或 data/animeTags.ts 手改, 站里看到问题不能顺手改. 只需要站长本人能改, 不做访客标注; 生产站不能有任何写入口.

## Decision
- 数据: `data/animeOverlay.json`, 键是 BanGuMi subject id, 字段 `watch_status / watched_eps / rate / tags / comment / updated_at`. 写入时键按数字升序、字段顺序固定、2 空格缩进, 一次改动只动一块, git diff 可读.
- 读: `src/utils/anime/overlay.ts` 静态 import 这份 JSON, `animeStore.ts` 拉到 CDN 的 ANiMeRecord.json 后逐条盖上 (`applyOverlay`). 生产与 dev 都读, 只读.
- 写: `plugins/anime-overlay-plugin.mjs` 只在 `docusaurus start` 的客户端配置 (`config.mode === 'development'`) 里经 `devServer.onListening` 挂 `PUT/DELETE /__hx/anime-overlay/<id>`, 校验后原子写 (tmp + rename). 要求本机 Host、`application/json` 与 `X-HX-Overlay: 1` 头 (跨站请求会被 CORS 预检挡住).
- 编辑器: `src/components/anime/Tag/OverlayEditor.tsx`, 详情页"我的状态"下方. 由 `lazyChunks.js` 的 `loadAnimeOverlayEditor = process.env.NODE_ENV === 'development' ? () => import(...) : null` 提供; 生产构建里这个 `import()` 在死分支里, webpack ConstPlugin 不解析它, 不产生 chunk. 保存后 `setOverlayEntry` 就地重算记录, 不用刷新; 文件变化也会触发 HMR.
- 只提交"已有 overlay 的字段"与"这次改过的字段", 没碰的字段继续跟随 CDN / animeTags.ts.

### 优先级
1. watch_status / watched_eps / comment / rate: overlay > CDN `user_status`. rate 只有 overlay 有.
2. tags: overlay 写了 `tags` 就是这部番的**最终列表**, `data/animeTags.ts` 和 CDN `user_status.tags` 对这部番都不再参与 (这样才能删掉来自 animeTags.ts 的标签). 没写 `tags` 时照旧 = animeTags.ts ∪ CDN tags.
3. 删除 overlay 条目 = 回到 CDN + animeTags.ts.
4. animeTags.ts 仍是分类定义 (`ANIME_TAG_DEFS` 的分组与口径) 的唯一来源, overlay 不改定义.

## Alternatives considered
- 什么都不做: 最强理由与结论 —— 标注本来就能改 data/animeTags.ts 再提交, 不加代码就不用引入写入通道与生产隔离的复杂度. 否决: 用户要的是本地启动时在前端直接增删改查, 改源码再提交不是他要的操作方式.
- 复用 dev server 既有中间件钩子 (setupMiddlewares): 最强理由与结论 —— 最强理由是不用自己管监听生命周期. 否决: Docusaurus 已定义同名函数, webpack-merge 6 会把两个函数的返回数组拼接, 中间件栈重复注册, 实测不可用.
- 直接改写 data/animeTags.ts: 只能管标签, 而且要在 TS 源码里做字符串手术, 否决.
- localStorage: 刷新还在但不进仓库, 换机器就丢, 也不可 diff, 否决.
- 写回 HXLoLi-ANiMe 仓库: 那边由 GitHub Actions 从 Bangumi 同步, 会被覆盖; 且是只读来源.
- dev server 用 `setupMiddlewares`: Docusaurus 已定义同名函数, webpack-merge 6 会把两个函数的返回数组拼起来, 中间件栈重复, 改用 `onListening`.

## Consequences
- 验收: ① dev 下改一个标签刷新还在 (写进 data/animeOverlay.json, 构建时打包). ② 改动落在 `data/animeOverlay.json`. ③ 生产构建后 `grep -r "__hx/anime-overlay\|X-HX-Overlay" build/` 应为空.
- overlay 接管 tags 后, 之后再改 animeTags.ts 对这部番不生效, 要删 overlay 的 tags 才回到合并规则.
- CDN 更新后 overlay 仍然覆盖, 想让 Bangumi 的新进度生效需要删掉 overlay 里对应字段.

## Code

- `plugins/anime-overlay-plugin.mjs`
- `src/utils/anime/overlay.ts`
- `src/components/anime/Tag/OverlayEditor.tsx`
- `src/components/home/lazyChunks.js`
- `src/pages/anime/details.tsx`
