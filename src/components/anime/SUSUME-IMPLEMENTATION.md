# 路5 实施记录: 主站只读人工标注 + susume 专属页与入口卡

依据: `/home/hx/Loli/code/HXLoLis/hx-anime-tag-refactor-design.md` 第 3.1 / 4 / 10 节. 日期 2026-11-04.

## 仓库实际结构与落点

HXLoLi 是 Docusaurus 3.7 (不是 Next.js, 没有 `app/` 目录), 路由由 `src/pages/**` 文件生成, 不支持 `[slug]` 动态段;
所以三个专属页是三个路由文件 + 一个共用组件.

**⚠ 映射接口限制**: fs.lolioa.com 读写接口**不能建目录, 也不能删除** (POST/PUT 只写文件, DELETE/PATCH 返回 404).
所以 `src/pages/anime/susume/` 目录建不出来. 三个路由文件暂时放在 `src/pages/anime/` 下, 文件名以 `_` 开头;
Docusaurus 默认排除 `_*.tsx`, 不生成路由, 也不影响构建. 另外, 试建目录时留下了两个 **0 字节的空文件** (没有扩展名,
Docusaurus 和 webpack 都不会读到), 需要删掉:
`HXLoLi/src/pages/anime/susume`, `HXLoLi/src/components/anime/Susume`.

在本机执行下面这段即可完成落位 (路由文件里全是 `@site/` 绝对导入, 搬动后不用改代码):

```bash
cd /home/hx/Loli/code/HXLoLis/HXLoLi
rm src/pages/anime/susume src/components/anime/Susume   # 两个 0 字节空文件
mkdir -p src/pages/anime/susume
for s in monosugoi loli imouto; do mv "src/pages/anime/_susume.$s.tsx" "src/pages/anime/susume/$s.tsx"; done
```

| 路由 | 最终文件 (绝对路径) | 当前暂存位置 |
|------|-------------------|-------------|
| `/anime/susume/monosugoi` (物凄い) | `/home/hx/Loli/code/HXLoLis/HXLoLi/src/pages/anime/susume/monosugoi.tsx` | `src/pages/anime/_susume.monosugoi.tsx` |
| `/anime/susume/loli` (ロリ) | `/home/hx/Loli/code/HXLoLis/HXLoLi/src/pages/anime/susume/loli.tsx` | `src/pages/anime/_susume.loli.tsx` |
| `/anime/susume/imouto` (シスコン) | `/home/hx/Loli/code/HXLoLis/HXLoLi/src/pages/anime/susume/imouto.tsx` | `src/pages/anime/_susume.imouto.tsx` |
| 共用页面组件 | `/home/hx/Loli/code/HXLoLis/HXLoLi/src/components/anime/SusumePage.tsx` | (已就位) |
| 首页入口卡 | `/home/hx/Loli/code/HXLoLis/HXLoLi/src/components/anime/SusumeEntryCards.tsx` | (已就位) |
| 样式 | `/home/hx/Loli/code/HXLoLis/HXLoLi/src/components/anime/Susume.module.css` | (已就位) |

没执行上面这段之前: 首页入口卡、详情页徽章, 以及旧链接 `?tag=物凄い` 的跳转都会指向还不存在的路由 (404).
其余改动 (只读 labels、九项筛选) 已经生效.

## 改了什么

1. **只读 labels**
   - 新增 `src/utils/anime/labels.ts`: 读 `HXLoLi-ANiMe/data/labels/<id>.json` (同 `ANIME_CDN_TAG`).
     先读构建产物 `data/labels/index.json`; 没有就用 jsDelivr data API 列出 `data/labels/*.json` 再逐个拉 (并发 6).
     目录不存在 = 没有标注 = 空表. 只保留词表内的标签.
   - `src/utils/anime/tags.ts`: `getRecordTags` 只读 labels. 删掉了与 `ANiMeTags.json` (`parseSourceTags` / `setSourceTags`)、
     `animeTags.ts` 手工分配表、`animeOverlay.json.tags`、`user_status.tags` 的合并. 新增 `getRecordLabel / getRecordRate /
     getRecordReview / recordsOfSusume / susumeDefOfTag / susumePath`. `countTags` 只数普通九项.
   - `src/utils/anime/animeStore.ts`: `loadSourceTags` (拉 ANiMeTags.json) 换成 `loadLabels`, 新增 `useAnimeLabelsStatus`.
   - `src/utils/anime/overlay.ts`: overlay 的 `tags` 标记 `@deprecated`, 不再读取. 它不再接管标签, 也不再覆盖 `user_status.tags`.
     `overlayTagsOf` 已删除.
   - `data/animeTags.ts`: 只保留词表渲染部分, 镜像 `HXLoLi-ANiMe/data/labelVocab.json` v1 (`ANIME_SUSUME_DEFS` + 九项 `ANIME_TAG_DEFS`).
     手工分配表 `ANIME_TAGS` 已移除 (历史见 git). 别名 萝莉→ロリ、妹控→シスコン 只用于解析旧链接, 不是数据迁移.
   - `components/anime/Tag/OverlayEditor.tsx` (仅 dev): 不再编辑标签, 只读展示 labels; 保存时不再写回旧 tags.
2. **三个专属页**: 按本人评分降序 (labels.rate, 其次 overlay.rate; 未评分排最后; 同分看 Bangumi 评分),
   每部附一句话观后感 (labels.review, 其次追番短评). 每部都有封面、中日文名、状态和详情页链接,
   页面有骨架 / 错误重试 / 空态三种状态, 并能互相切换三个分类.
   `/anime/susume` 本身没有页面 (入口卡在 `/anime` 顶部).
3. **首页入口卡**: `/anime` 标题下面放三张大卡 (封面拼贴取前 4 部, 加数量). 详情页上的推荐分类是金色徽章
   (`AnimeDetailPage.module.css .susumeBadge`), 点了进专属页. 海报卡上也一样 (`PosterCard.module.css .susumeTag`).
4. **旧链接**: `/anime?tag=物凄い|ロリ|シスコン|萝莉|妹控` (多选里带上任意一个也算) 在 `src/pages/anime/index.tsx` 的
   `legacySusumeTarget` 里, 客户端 `history.replace` 到专属页 (静态站点做不了按查询串的服务端 301).
5. **普通九项** (异世界/热血/战斗/推理/校园/日常/搞笑/恋爱/致郁) 仍走 `?tag=` 筛选, `filters.ts` 没有改.

## 没动的 (按范围)

- HXLoLi-ANiMe 子仓库、workflows、`labelVocab.json`、`static/anime/ANiMeTags.json` (现在没人读, 是废弃文件, 未删除).
- `data/animeOverlay.json` 数据原样保留. 里面残留的 `tags` 已在代码层废弃不读; 按设计 10.3 移除这个字段需另开一步.
- `src/components/home/HomeAnime.tsx` 不在写入范围. 它用的 `countTags` 签名没变, 现在只列普通九项, 点击仍走 `?tag=`.
  其中写死的 `'萝莉'` 高亮已经失效, 不影响功能.
- `worker.js`: 不在写入范围. 如果要真正的服务端 301, 可在 worker 里对 `/anime?tag=` 做同样的映射.

## 依赖与待办

- **标注流需产出 `data/labels/index.json`** (推荐, 一次请求拿全). 没有它也能工作, 但要多一次 jsDelivr 列目录 + N 次请求.
- `labelVocab.json` 改动时同步 `data/animeTags.ts` (`ANIME_LABEL_VOCAB_VERSION`).
- **Agent Notes 红线**: 本轮写入范围不含 `.agents/notes/`, 没有新建配对 note, `uv run scripts/redlines/agent_notes.py --diff`
  预计会报单边改动. 需要补一篇 note, 建议路径 `.agents/notes/implemented/feature/2026-11-04-anime-labels-readonly-susume.md`,
  `## Code` 列出 `HXLoLi/src/utils/anime/tags.ts`、`HXLoLi/src/components/anime/SusumePage.tsx`、
  `HXLoLi/src/pages/anime/susume/monosugoi.tsx` (搬位后)、`HXLoLi/data/animeTags.ts`, 并把这些文件头的引用改成指向它.
- 校验: 本地用 tsc 5.6 + 存根检查了改动文件, 0 个类型错误. 没有跑 `docusaurus build` (按 AGENTS.md, 不主动启动博客程序).
