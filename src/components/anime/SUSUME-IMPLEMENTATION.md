# おすすめ (susume) 实施记录

决策见 `.agents/notes/implemented/feature/2026-11-06-anime-susume-neko-redesign.md`.

| 路由 | 路由文件 | 页面组件 |
|------|----------|----------|
| `/anime/susume` | `src/pages/anime/susume/index.tsx` | `src/components/anime/susume/SusumeHubPage.tsx` |
| `/anime/susume/monosugoi` · `loli` · `imouto` | `src/pages/anime/susume/<slug>.tsx` | `src/components/anime/susume/SusumeCategoryPage.tsx` |
| `/anime` 顶部入口带 | `src/pages/anime/index.tsx` | `src/components/anime/susume/SusumeEntry.tsx` |

- 数据: `src/components/anime/susume/useSusume.ts`. 只读人工标注 (`utils/anime/tags.ts` 的 `recordsOfSusume`), 按本人评分降序.
- 共用小件: `src/components/anime/susume/parts.tsx` (分类切换条 / 元信息 chips / 封面扇 / 空推荐板).
- 舞台: `src/components/anime/stage/` (Hero 封面墙 / 逐字标题 / 统计滚动 / 名次数字 / 区块标题), kyara 页共用.
- 样式: `src/components/anime/susume/susume.module.css` 和 `src/components/anime/stage/stage.module.css`, 只用 `--hx-*` 令牌.
- 已删除: `SusumePage.tsx`, `SusumeEntryCards.tsx`, `Susume.module.css`.
