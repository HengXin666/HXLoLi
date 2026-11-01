# Agent Note: 番剧详情页重做 (Hero + 进度 + 分组剧集 + 相似推荐, 明暗主题)

Status: implemented

Decision-ID: anime-detail-redesign

## Code

- `src/pages/anime/details.tsx`

## Problem

详情页 (`/anime/details?id=<id>`) 信息架构是早期拼出来的: 标题居中一行, 下面封面 + 两个灰盒 ("基本信息" / "我的状态") 里逐行 `<strong>评分:</strong> 7.5 / 10`, 观看进度只是 `12 / 13` 文字; 剧集是正片 / SP / OP / ED 混在一起的数字方块, 看不出看到第几集, 单集详情只能靠 500px 宽的悬停卡 (触屏不友好); 简介整段铺开; 角色是横向滚动条; 页尾没有任何"接着看什么". 颜色全部写死深色 (`#2b2b2b` / `#3c3c3c` / `#e0e0e0`), 亮色主题下是一块黑板. 大量内联 style.

另有几处缺陷: `record.anime_data.episodes.sort(...)` 原地排序, 而 `animeStore` 把记录 `Object.freeze` 了 (数组本身不冻结时也会改写全站共享的数据); 关联条目里 Tooltip 的 `key={relation}` 同组重复; CV 列表 Fragment 无 key; 外链 `target=_blank` 没有 `rel`.

## Decision

按"这是什么 -> 我看到哪 -> 我怎么评 -> 深挖"重排, 页面拆成文件内的小组件 (`Hero` / `SectionNav` / `Summary` / `Episodes` / `Characters` / `Relations` / `Section`), 不新建目录:

- Hero: 主题色卡片, 虚化封面只作左上角淡光晕 (细节见 `2026-11-01-anime-detail-sidebar-and-hero`), 左 2:3 封面 (定宽 220px, 可放大), 右侧中文标题 / 原名, 元信息行 (番剧季度 "2024 秋 · 10 月番"、首播日期、正片话数、Bangumi 评分), 状态卡 (状态徽章按状态着色、"已看 N / M 话"、更新日期、进度条), 标签 chips (链回番剧库按该标签筛), 操作 (Bangumi、关系图、分享番单).
- 短评 (`user_status.comment`) 做成引用卡放在 Hero 下, 第一眼可见.
- 页内目录: 简介 / 剧集 / 角色 / 关联 / 相似 / 讨论; 宽屏为左侧竖排侧栏并高亮当前段, 窄屏为不吸顶的横滑细条 (细节见 `2026-11-01-anime-detail-sidebar-and-hero`).
- 简介超过 220 字默认折叠 6 行.
- 剧集按类型分组 (正片 / SP / OP·ED / PV 与其他), 组内复制后按 `sort` 排; 正片前 `watched_eps` 集高亮为已看; 方块是 button, 悬停 / 聚焦 / 轻点出单集卡.
- 角色网格, 主角 -> 配角 -> 客串, 默认 12 个可展开; CV 名字保留原来的人物卡 (头像、bgm、出演番剧).
- 关联条目按关系分组横排海报, 在我库里的走站内详情, 不在的去 bgm 并标 "bgm ↗".
- 新增相似推荐: 库里与本番共享标签最多的 6 部 (同分按评分), 用 `getRecordTags` 只读计算.
- Giscus 放最后, term 不变, 旧讨论不丢.
- 样式改用 Infima 变量 (`--ifm-color-emphasis-*`, `--ifm-background-surface-color`, `--ifm-color-primary`), Hero 文字同样走主题色, 明暗都可读; 窄屏 Hero 封面缩到左上、状态与操作占满整行; `prefers-reduced-motion` 关动效.
- 保留冷启动三态 (骨架 / 错误重试 / 404) 与另一条线接入的 dev-only 本人标注编辑器 (`OverlayEditor`, 放在短评下方).

## Alternatives considered

- 什么都不做 / 只换配色: 改动最小, 不碰另一条线. 否决: 用户要的是信息架构重做, 换色解决不了进度不可见、剧集混排、亮色主题黑板、冻结数组原地 sort 这些问题.
- 复用番剧库的 `PosterCard` 做关联 / 相似海报: 风格统一. 否决: `PosterCard` 带状态角标、进度条、悬浮短评, 依赖完整 `ANiMeRecord`, 关联条目只有 `Relation` (id / 名字 / 封面), 喂不进去; 文件内 20 行的 `PosterLink` 够用.
- 拆到新目录 `src/components/anime/AnimeDetail/`: 每个组件独立文件更好维护. 本轮否决: 用户限定只动 `details.tsx` 及其专属样式, 新目录未获批; 组件都在文件内, 以后要拆是纯移动.
- 剧集详情改成点击展开的内联面板: 移动端更稳. 否决: 全站已有 `Tooltip` (悬停 / 聚焦 / 轻点三种触发, selectable 常驻), 直接复用, 不再造一套弹层.

## Consequences

`src/pages/anime/AnimeDetailPage.module.css` 整体重写 (非 AST 资源, 需人工核对), 旧类名 (`infoBox` / `episodeList` / `characterScrollContainer` 等) 不再存在, 只有本页引用. 页面不再使用 `MDXA` 与 `Tag` 组件. 相似推荐依赖 `getRecordTags`, 本人标注 overlay 改了标签后会自动反映. 修改与 `src/pages/anime/index.tsx` 同目录, 红线检查会把 `anime-page-refactor` 决策标为待 review (同目录兄弟代码变化), 那篇 note 的事实没有改变. 本机 `npx tsc --noEmit` 已复核 (2026-11-01, 含后续 sidebar-and-hero 改动): 错误全在 `scripts/quality-baseline.json` 基线内, `details.tsx` 零错误.
