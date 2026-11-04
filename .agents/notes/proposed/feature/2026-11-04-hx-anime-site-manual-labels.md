# Agent Note: 番剧标签主站只读人工标注 + 三个推荐分类专属页

Status: proposed

Decision-ID: hx-anime-site-manual-labels

## Code

- `src/pages/anime/susume/monosugoi.tsx`

## Problem

主站番剧 tag 曾由四处来源混合渲染: `HXLoLi-ANiMe/data/ANiMeTags.json` 的自动上墙分配, `data/animeTags.ts` 的手工分配表, `data/animeOverlay.json` 的 tags, bgm 同步回来的 user_status.tags. 无法追溯谁标的, 自动推断误标多. 推荐分类 (物凄い / ロリ / シスコン) 原本只是 `?tag=` 筛选结果, 没有独立入口页面.

## Decision

1. 主站渲染只认人工标注 `HXLoLi-ANiMe/data/labels/<bgm_subject_id>.json` (经 `src/utils/anime/labels.ts` 加载), 未标注 = 没有文件 = 不展示任何 tag. 不再读 ANiMeTags.json, animeTags.ts 分配表, animeOverlay.json 的 tags, user_status.tags. 不读同步回来的 user_status.tags 即防回环.
2. `data/animeTags.ts` 只保留词表渲染部分, 是 `HXLoLi-ANiMe/data/labelVocab.json` 的镜像 (两个仓库, 构建时拿不到子仓文件所以抄一份). 词表唯一来源仍是 labelVocab.json, display / slug 必须逐字一致. 普通标注九项仍走 `?tag=` 筛选.
3. 三个推荐分类是独立路由专属页, 不是 `?tag=` 筛选结果: 物凄い -> `/anime/susume/monosugoi`, ロリ -> `/anime/susume/loli`, シスコン -> `/anime/susume/imouto`. 页面按本人评分降序并附一句话观后感.
4. `/anime` 首页顶部三张大入口卡 (封面拼贴 + 数量), 详情页这三个标签渲染为特殊徽章点击进入对应 susume 页. 旧式 `/anime?tag=物凄い` 等链接客户端重定向到对应 susume 页, 避免两套入口.
5. 标注 PR 与 data 流的产出由 HXLoLi-ANiMe 子仓的流水线负责, 本仓只消费 `data/labels/` 与词表镜像.

## Alternatives considered

- **什么都不做 / 复用现有四源合并渲染**: 最强理由是覆盖率高且不用改主站. 否决: 误标正是重构起因, 四源并存无法追溯, 自动推断无法反映个人口味.
- **复用 animeTags.ts 手工表, 只关自动推断**: 最强理由是改动最小. 否决: 手工表是大对象, 多 PR 并行必冲突, 没有封面化勾选入口和过期机制, 标注成本仍是打开文件打字.
- **推荐分类继续用 ?tag= 筛选, 不建专属页**: 最强理由是不用新增路由. 否决: 用户明确要求这三个是有明显入口的专门页面, 与 ?tag= 不同; 筛选页按数量排序, 无法按本人评分和观后感组织.
- **主站直接读子仓 labelVocab.json**: 最强理由是单一来源. 否决: 两个仓库, 主站构建时拿不到子仓文件, 只能镜像一份并靠人工同步.

## Consequences

- 切到只读 data/labels 后站内分类视觉上清空, 之后分类只随人工标注增长.
- 词表改动要同时改两个仓库 (labelVocab.json 与 animeTags.ts 镜像), 漏改会显示不一致.
- 三个 susume 页依赖评分与观后感数据, 未评分的番排在末尾.
- bgm 上已有旧随手标签不自动删除, 一次性清理需单独确认.

## Verification

- `uv run scripts/redlines/agent_notes.py --all` 在本子仓运行, 结果见本仓 `.git/reports/agent-notes-all.json`.
- 静态可查: `src/pages/anime/susume/{monosugoi,loli,imouto}.tsx` 存在且各 import SusumePage; `src/pages/anime/index.tsx` 含 SusumeEntryCards 与 legacySusumeTarget; `src/utils/anime/tags.ts` 含 susumePath / susumeDefOfTag; `data/animeTags.ts` 含 ANIME_SUSUME_DEFS 三项 slug.
- 页面真实渲染与重定向未做浏览器实测.
