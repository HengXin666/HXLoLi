# HXLoLi-ANiMe 番剧数据"过期不回写"诊断

> 诊断日期: 2026-11-01. 只读诊断, 未改任何代码.
> 对象: `HXLoLi-ANiMe` (数据仓库, 本地 HEAD `53f2ad4`) + `HXLoLi` (博客, 只读).

## 结论速览

- **漏洞类型: c** (有 update, 但被 `if` 挡住): 已有番剧的条目级信息, 只有在 **用户自己的收藏 `updated_at` 变化** 时才会刷新; 番剧本身在 Bangumi 上更新 (集数、评分、剧集表、角色等) 不会触发回写.
- 叠加一个 **a 型的半截**: `score` / `image_url` / `short_summary` 只在新增时写一次, 之后任何路径都不更新; 刷新路径 `_refresh_anime_full` 也不调用 `_get_subjects_info`, 所以 `eps` / `total_episodes` / `date` / `summary` / `name_cn` 同样永远停在入库那一刻.
- d (接口缓存) 与 e (合并方向反) 排除; f 有两个**次要**问题 (CF Worker 不会被 Bot push 触发重新部署; 旧 Tag 清理可能删掉博客正钉着的 Tag), 但不是本 bug 的主因.

---

## 一. 数据怎么生成、存在哪

### 工作流清单 (`.github/workflows/`)

| 文件 | 作用 | 触发 | 与番剧数据关系 |
|---|---|---|---|
| `anime-pipeline.yaml` | **爬取 → commit/push → 打 CDN Tag → 向 HXLoLi 发 PR** | `schedule` 两次/天: `0 16 * * *` (北京 00:00) 与 `0 7 * * *` (北京 15:00); `push` 到 main 且改了 `scripts/**` 或 `data/**`; `workflow_dispatch` 手动 | **负责抓番剧数据的那个** |
| `deploy-cf-worker.yaml` | 把 `data/` 部署到 Cloudflare Worker `hxloli-anime` | `push` 到 main 且改了 `data/**`; 手动 | 博客 CF Worker 节点的数据源 |
| `agent-notes.yml` / `agent-notes-full.yml` / `agent-notes-report.yml` | Agent Notes 红线检查与报告 | push / PR / workflow_run | 无关 |

> README 里写的 `crawl-anime.yml` / `cdn-tag.yml` 已经合并成 `anime-pipeline.yaml`, README 过时.

### `anime-pipeline.yaml` 流程

1. `uv sync` (scripts/crawler)
2. 跑 `uv run Api.py -u "${BGM_USERNAME:-heng_xin}" -o "${BGM_TOKEN}"` (第 64–71 行)
3. `git add -A && git diff --cached --quiet` 判断有没有变化 (第 77–95 行)
4. 有变化才: commit `[LoLi-Bot]: ...` 并 `git push` (第 97–103 行) → 打 `HX-{时间戳}-{随机6位}` Tag 并 push (第 109–121 行) → 只保留最新 2 个 `HX-*` Tag (第 123–134 行) → 向 `HengXin666/HXLoLi` 开 PR, 改 `data/animeCdnVersion.ts` 里的 `ANIME_CDN_TAG` (第 281–340 行)

### 输出文件

`Api.py` 的 `save_to_json` (第 419–443 行) 写:
- `data/ANiMeRecord.json` (番剧记录, 当前 160 条, ~12.5 MB)
- `data/Actor.json` (声优)
- `download_all_img` 下载到 `data/anime|kyara|relation|cv/*.jpg`

频率: 每天 2 次. 但**只有 diff 非空才会出新 Tag**.

---

## 二. 漏洞在哪 (证据)

### 2.1 主因: 已有记录的刷新被 `last_update` 挡住 (类型 c)

`scripts/crawler/Api.py` → `Api._get_user_watching_anime`, 第 172–202 行:

```python
for it in data:
    if int(it["subject_id"]) in self._record_map:
        ref = self._record_map[int(it["subject_id"])]
        if ref.user_status.watch_status != WatchStatus(it["type"]):
            ref.user_status.watch_status = WatchStatus(it["type"])
        if ref.user_status.watched_eps != it["ep_status"]:
            ref.user_status.watched_eps = it["ep_status"]
        if ref.user_status.comment != it["comment"]:
            ref.user_status.comment = it["comment"]
        if ref.user_status.tags != it["tags"]:
            ref.user_status.tags = it["tags"]

        if ref.user_status.last_update != it["updated_at"]:      # ← 唯一的刷新闸门
            ref.user_status.last_update = it["updated_at"]
            ...
            lambda_func(idx)          # → requires._task → _refresh_anime_full
        continue                      # ← 其余情况: 条目级数据完全不碰
```

`it["updated_at"]` 是 **用户收藏条目** 的最后修改时间 (改状态/点格子/写评论), 不是番剧条目本身的更新时间. 所以:
- 番剧在播、Bangumi 补了剧集/简介/角色、评分变化 → 用户没动收藏 → 不刷新.
- 用户已"看过"的老番 → 永远不刷新.
- 注意: 这里 `it["subject"]` 里本来就带着最新的 `score` / `eps` / `name_cn` / `date` / `images`, 但对已有记录完全没读.

### 2.2 加重: 刷新路径本身也不全 (a 型的半截)

`Api._refresh_anime_full`, 第 91–98 行:

```python
def _refresh_anime_full(self, idx: int) -> None:
    record = self._anime_record[idx]
    record.anime_data.characters.clear()
    record.anime_data.relations.clear()
    record.anime_data.episodes.clear()
    self._get_anime_characters(idx)
    self._get_relations(idx)
    self._get_episodes(idx)
```

- 不调用 `_get_subjects_info` (第 134–154 行). 而 `eps` / `total_episodes` / `date` / `summary` / `name_cn` / `name` **只在** 新增分支 (第 226 行) 通过 `_get_subjects_info` 写过一次.
- `score` / `image_url` / `short_summary` 只在新增分支构造 `ANiMeData` 时写 (第 205–217 行), **全文件没有任何地方再更新它们** (`_get_subjects_info` 也不写 score).
- 新增时 `eps` / `total_episodes` 来自收藏列表的 `subject.eps`, 新番入库时常为 0, 之后就一直是 0.

### 2.3 实测佐证 (本地 `data/ANiMeRecord.json` vs `api.bgm.tv/v0/subjects/{id}` 2026-11-01 实时值)

| id | 番剧 | 本地 score / eps / total | 线上 score / eps / total |
|---|---|---|---|
| 569116 | 碧蓝之海 第三季 | 7.2 / **0** / 12 | 7.0 / 12 / 12 |
| 501963 | 无职转生 第三季 | 7.9 / **0** / **0** (date 07-05) | 8.0 / 14 / 14 (date 07-04) |
| 541285 | 与你相恋到生命尽头 | 7.2 / 13 / **12** | 6.6 / 13 / 17 |
| 633836 | Re:从零 第四季 夺还 | **7.9** / 8 / 8, name_cn 少"篇" | 7.2 / 8 / 8 |
| 623854 | 女神"异世界转生…" | 7.0 / **0** / 12 | 7.1 / 12 / 12 |

另外: 远端最新 Tag 是 `HX-20260913123905-V2PJj3` (jsDelivr 包索引), 本地记录里最晚的 `last_update` 也是 2026-09-13 — 说明从 9/13 起每天两次跑, `git diff` 一直为空, 一个新 Tag 都没出. 正好符合"用户没动收藏 → 什么都不写"的行为.

### 2.4 逐项核对 a~f

| 类型 | 结论 | 依据 |
|---|---|---|
| a 已有记录直接跳过 | **部分是** (score/image/short_summary 永不更新; 未改收藏时整条跳过) | 第 202 行 `continue`; 2.2 |
| b 只 insert 不 update | 番剧记录: 否 (有 update 分支). 声优: **是** — `_get_anime_characters` 第 294 行 `if actor["id"] not in self._actor_map` 只补新声优; 图片: **是** — `_get_img_and_save` 第 68 行文件存在即跳过, 封面换图不会更新 | 次要 |
| **c 被 if 挡住** | **是, 主因** | 第 187 行 `if ref.user_status.last_update != it["updated_at"]` |
| d 接口返回缓存 | 否. `api.bgm.tv/v0/subjects/{id}` 实时返回新值 (见 2.3); 爬虫不读任何本地缓存 | |
| e 合并方向反了 | 否. 先 `load_from_json` 读旧, 再用 API 值覆盖, 最后整文件写回 | 第 391–443、464–468 行 |
| f git / CDN | 主路径 commit+push+tag 正常. 次要问题见第三节 3.3 | |

---

## 三. 博客侧怎么读到这份数据 (HXLoLi, 只读)

### 3.1 链路

1. `src/utils/anime/animeStore.ts` `loadAnimeRecords()` (第 12–33 行) → `reqAnimeByCDN('/data/ANiMeRecord.json')`, 结果存在模块级 `cachedRecords` (只在当前页面会话内, 刷新页面即失效).
2. `src/utils/cdn/linkJsDelivr.ts`:
   - `animeEngine` 用 `ref: ANIME_CDN_TAG` (第 116–118 行), 即 **按 Tag 钉版本**.
   - `ANIME_CDN_TAG` 来自 `data/animeCdnVersion.ts`, **当前值 `HX-20260906113630-E1EgjV`**.
   - 节点 = CF Worker 直连 `hxloli-anime.woa.qzz.io` + hx-cdn-forge 自带 jsDelivr 系节点 (第 92–96 行); 默认节点 `jsd-mirror` (第 46 行), 用户测速/选择的节点存 `localStorage['hxloli-cdn-node']`.
   - SSR 兜底 URL: `https://cdn.jsdmirror.com/gh/HengXin666/HXLoLi-ANiMe@{TAG}/...` (第 218–221 行).
3. 所以: **走 jsDelivr (镜像 jsdmirror) + 自己的 CF Worker 两条**, 主路径是 jsDelivr@tag.

### 3.2 缓存与可见延迟

- jsDelivr `@tag` 实测响应头: `cache-control: public, max-age=604800, s-maxage=43200` (浏览器 7 天, 边缘 12 小时). 但每次数据更新都会生成**新 Tag = 新 URL**, 所以 CDN TTL 不会造成"看到旧数据".
- 博客真正的延迟 = **爬虫出新 Tag (每天 00:00 / 15:00) → HXLoLi 的 PR 被合并 → 博客重新构建部署**. PR 合并是人工闸门.
- 现状: 远端已有 `HX-20260913…` 但博客仍钉在 `HX-20260906…`, 9/13 那次的 PR 看起来没合.

### 3.3 次要的 f 型风险 (不是本 bug 主因, 修的时候顺手)

1. **CF Worker 不会因 Bot 的 push 重新部署**: `anime-pipeline.yaml` 用默认 `GITHUB_TOKEN` checkout 并 `git push`, GitHub 规定 `GITHUB_TOKEN` 产生的 push 不触发其他工作流, 所以 `deploy-cf-worker.yaml` 的 `push: paths: data/**` 对 Bot 提交不生效. 且 CF Worker 节点不看 Tag, 选到 `cf-worker` 节点的访客看到的是"上一次手动/人工 push 部署的版本", 和 jsDelivr@tag 节点不一致.
2. **旧 Tag 清理可能删掉博客正在用的 Tag**: 第 123–134 行只保留最新 2 个 `HX-*`. 现在是 [09-13, 09-06], 博客钉 09-06; 修好后下一次出新 Tag, 09-06 就会被删. 若对应 PR 还没合, 博客钉住的 Tag 在 GitHub 上消失, jsDelivr 只能靠已缓存的文件撑着, 未缓存的文件 (图片等) 会 404.

---

## 四. 修法 (下一轮执行, 本轮未动)

### 4.1 爬虫 (`scripts/crawler/Api.py`)

1. **已有记录每次都从收藏列表的 `it["subject"]` 同步轻量字段** (零额外请求): `name` / `name_cn` / `short_summary` / `score` / `eps` / `date` / `images.large`.
2. **改刷新闸门**: 把第 187 行的单一条件改为"需要刷新"判定, 满足任一就走完整刷新:
   - 用户 `updated_at` 变了 (保留原逻辑);
   - 未完结/近期: `watch_status` 为 在看/想看/搁置, 或 `date` 在最近 N 天 (建议 180) 内;
   - 数据明显不全: `eps == 0`, `total_episodes == 0`, `len(episodes) < eps`, 或与 `it["subject"]["eps"]` 不一致.
   - 老番兜底: 可选按 `id % 7 == 当天星期` 轮转全量刷新, 每周把所有条目都过一遍, 控制请求量.
3. **`_refresh_anime_full` 补上 `_get_subjects_info`**, 并让 `_get_subjects_info` 也写 `score` (`data["rating"]["score"]`) 与 `image_url`.
4. **先拉后换**: `_refresh_anime_full` 现在先 `clear()` 再请求, 请求失败会把角色/剧集清空写回; 改成拉到临时列表成功后再替换.
5. 顺手: `_get_subjects_info` 的 `except` 里 `response` 可能未定义 (第 147–148 行); 声优已存在时也更新 `name/short_summary/image_url`.
6. 可选: 封面图按 URL 变化重下 (目前文件存在就跳过).

### 4.2 工作流 (`anime-pipeline.yaml` / `deploy-cf-worker.yaml`)

1. 让 CF Worker 跟着数据更新: `deploy-cf-worker.yaml` 加 `workflow_run: workflows: ["[LoLi-ANiMe]: 爬取番剧数据 & CDN Tag"] types: [completed]`, 或在 pipeline 末尾直接调用部署 job.
2. Tag 清理改为: 保留最新 N 个 (建议 5) **且**永远保留 HXLoLi `main` 上 `animeCdnVersion.ts` 当前钉着的 Tag.
3. 注意评分每天浮动会让几乎每次跑都有 diff → 每天最多 2 个 PR. 若嫌吵, 可把 PR 改为"更新已存在的同一个 `auto/anime-cdn-*` PR"而不是每次新开.

### 4.3 验证方式

- 本地跑爬虫后对 2.3 表里的 5 个 id 比对 `score/eps/total_episodes/episodes 数`, 应与线上一致.
- 对一部"看过"且多月未动收藏的老番, 确认轮转刷新能更新它.
- 跑完 `git diff --stat data/ANiMeRecord.json` 应非空, 流水线能出新 Tag.

### 4.4 Agent Notes

按仓库红线, 修代码时需在同一 diff 中新增/更新对应 `.agents/notes/...md` 并在 `Api.py` 相关声明顶部反向引用, 完成后跑 `uv run scripts/redlines/agent_notes.py --diff`.
