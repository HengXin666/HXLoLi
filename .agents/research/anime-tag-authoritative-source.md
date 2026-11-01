# 番剧萌属性标签的权威数据源调研

调研日期: 2026-11-02 (Asia/Shanghai)
样本: HXLoLi-ANiMe `data/ANiMeRecord.json` 共 177 部, 取自 2026-11-01 修复后爬虫产出的镜像; 手工表 `data/animeTags.ts` 共 157 条, 其中 35 条为「萝莉」
范围: 只调研, 未改任何代码. 所有数字均为本次实测, 中间产物在 Hark 沙箱 `/workspace/work/tags/` (hit.json / al.json / kitsu.json)

---

## 1. 结论

**推荐用 manami-project/anime-offline-database (AOD) 作为标签主源, 用 bangumi-data 把 bgm id 映射过去.**
理由: 它是唯一一个"整库可下载 + 带来源 URL + 授权明确 (ODbL/DbCL)"的数据集. 它把 AniDB / AniList / anime-planet / Kitsu / aniSearch 等十个站点的标签合并在一起, 177 部里能覆盖 169 部. 对于除「萝莉」之外的 8 类标签, 它对手工标注的召回率接近 100%. 

**需要说清的坏消息: 「萝莉」这一维度没有任何公开源能做到权威.**
你手工表的口径是"主角或核心角色是小学生年纪 / 幼女外形, 且是卖点". 实测下来:

| 信号 | 标出 | 与手工 33 部重合 (召回) | 精度 |
|---|---|---|---|
| AOD `tags` 含 `loli` / `lolicon` | 19 | 10/33 | 10/19 |
| Kitsu category `Loli` | 4 | 2/33 | 2/4 |
| AniList tag `Primarily Child Cast` rank≥50 | 13 | 10/33 | 10/13 |
| AniList 主角色 (role MAIN) 年龄≤12 | 27 | 14/33 | 14/27 |
| 以上四个取并集 | 46 | 23/33 | 23/46 |

注: 手工标萝莉的 35 部里, 有 2 部 (少女波子汽水、晚安sex) 不在 AOD 映射内, 所以分母是 33.

因此本文的建议是: **萝莉改成"有证据的半自动"**. 由多源信号给出候选与证据, 手工表降级为"覆盖层 (override)". 不建议把任何单一源当作萝莉的真值.
其它 11 类中, 舰娘与妹控在源里几乎没有对应, 维持 `userOnly`.

---

## 2. 候选源逐个实测

符号: ✅ 可用 / ⚠️ 可用但有硬伤 / ❌ 否决

### 2.1 ✅ manami-project/anime-offline-database (AOD), 推荐的主源

- **仓库没有失效.** 你之前测的应该是 GitHub API, 它对 Hark 沙箱出口返回 `{"message":"Sorry. Your account was suspended","status":"403"}`. 对 `torvalds/linux` 同样是这个返回, 所以是出口 IP 的问题, 与仓库无关. 网页和 release 下载都正常:
  ```
  $ curl -sL -o /dev/null -w "%{http_code} %{url_effective}" https://github.com/manami-project/anime-offline-database/releases/latest
  200 https://github.com/manami-project/anime-offline-database/releases/tag/2026-27
  $ curl -sL -o aod.json -w "%{http_code} %{size_download}" https://github.com/manami-project/anime-offline-database/releases/latest/download/anime-offline-database-minified.json
  200 62331136
  $ curl -sL -o aod.json.zst ... /anime-offline-database-minified.json.zst
  200 6033918
  ```
- **顶层结构**: `{"$schema","license","repository","scoreRange","lastUpdate","data":[...]}`, 其中 `lastUpdate = "2026-07-04"`, `data` 共 41537 条.
- **License 字段原文**: `Open Data Commons Open Database License (ODbL) v1.0 + Database Contents License (DbCL) v1.0`. 自用以及在博客展示都允许, 条件是署名; 如果再分发衍生数据库, 须同样以 ODbL 开放. 博客侧放一行署名即可.
- **真实片段**, 龙王的工作 (bgm 219658 → MAL 35905):
  ```json
  "sources": ["https://anidb.net/anime/13281","https://anilist.co/anime/99507","https://anime-planet.com/anime/the-ryuos-work-is-never-done","https://animecountdown.com/701123","https://animenewsnetwork.com/encyclopedia/anime.php?id=19872","https://anisearch.com/anime/12628","https://kitsu.app/anime/13661","https://livechart.me/anime/2789","https://myanimelist.net/anime/35905","https://simkl.com/anime/701123"],
  "tags": ["age gap", ..., "comedy", "female harem", "harem", "kuudere", "loli", "lolicon", "male protagonist", ..., "romance", "seinen", "shogi", "slice of life", "tsundere", "yandere"]
  ```
- **萝莉类字段路径**: `data[].tags[]`. 取值为小写字符串 `"loli"` (全库 217 部) 和 `"lolicon"` (37 部). 相邻的还有 `"primarily child cast"` (624)、`"child protagonists"` (334)、`"kids"` (8134, 指儿童向, 不能当萝莉用).
- **投票 / 置信度**: **没有.** 标签是各源的并集, 不带出处, 也不带权重.
- **更新频率**: 设计上每周发一个 release (tag 名是 `YYYY-WW`). 但实测 latest 仍是 `2026-27`, `lastUpdate` 为 2026-07-04, 距今约 4 个月没有新版本 (见风险 4.1).
- **覆盖 (177 部)**: 通过 bangumi-data 映射命中 171 部, 能在 AOD 找到条目的有 **169 部**.
  - 不在 bangumi-data 的 6 部: 某科学的超电磁炮 第四季 (537743)、少女波子汽水 (184740)、晚安sex (249681)、咔嗒咔嗒 第二季 (614606)、为美好的世界献上祝福！第四季 (544024)、剧场版 想要成为影之实力者！残响篇 (471345).
  - 映射到了但 AOD 里没有的 2 部: 26804、296195.
- **其它类与手工表的对照** (AOD 映射规则见第 3.2 节; 括号为手工数 / AOD 数 / 重合数):
  - 异世界 (34 / 52 / 34)
  - 校园 (28 / 92 / 27)
  - 日常 (29 / 90 / 29)
  - 恋爱 (16 / 72 / 16)
  - 搞笑 (29 / 136 / 29)
  - 战斗 (38 / 96 / 37)
  - 催泪 (6 / 69 / 6)
  - 悬疑 (11 / 57 / 10)
  - 百合 (0 / 25 / 0)

  结论: AOD 对手工标注几乎全覆盖, 但宽得多. 比如「搞笑」AOD 给了 136 部, 原因是 comedy 是很多源的默认 genre. 所以必须配合"主标签"截断规则 (第 3 节), 不能把 genre 全部原样上墙.

### 2.2 ✅ bangumi-data, 作为映射层 (不是标签源)

- `curl -sL https://cdn.jsdelivr.net/npm/bangumi-data@latest/dist/data.json` 返回 200, 7906809 B. 版本 0.3.234, **License 为 CC-BY-4.0**.
- 共 8925 条. 各站点覆盖条数: bangumi 8862, mal 8405, aniList 8328, anidb 8199, bilibili 3207.
- 真实片段:
  ```json
  {"title":"Cyberpunk: Edgerunners 2", ..., "sites":[{"site":"bangumi","id":"513878"}, ..., {"site":"mal","id":"61990"},{"site":"anidb","id":"19445"},{"site":"aniList","id":"195539"}]}
  ```
- 注意: `sites[]` 里有的条目没有 `id` 字段 (只有 `url`, 例如 iqiyi), 解析时要容错.
- 更新频率: npm 包持续发版, 新番季会更新.

### 2.3 ⚠️ Kitsu categories: endpoint 没错, 之前的空数组应该是调用方式的问题

- **结论先说**: `GET https://kitsu.app/api/edge/anime/{kitsu_id}/categories` 是正确的 endpoint. 龙王的工作 (Kitsu 13661) 的返回非空, 而且第一条就是 Loli:
  ```
  $ curl -s "https://kitsu.app/api/edge/anime/13661/categories?page%5Blimit%5D=20&fields%5Bcategories%5D=title"
  {"data":[{"id":"40","type":"categories","links":{"self":"https://kitsu.app/api/edge/categories/40"},"attributes":{"title":"Loli"}},{"id":"160","type":"categories",...,"attributes":{"title":"Comedy"}},...
  ```
  完整标题为 `['Loli','Comedy','Harem','Sports']`.
- **之前为什么拿到空数组**, 两个常见原因 (第一个我本次亲自踩到了):
  1. curl 不加 `-g` 就会把 `page[limit]` 里的 `[]` 当成 glob. 我本次第一次跑批时 165 条全部返回空 body, 加上 `-g` 或把方括号写成 `%5B` `%5D` 后才正常.
  2. 用了错误的 id. Kitsu id 不等于 MAL id, 也不等于 bgm id. 龙王的工作是 13661, 应当从 AOD 的 `sources` 中取 `kitsu.app/anime/<id>`.

  另外 `/anime/13661/media-categories` 和 `/categories?filter[slug]=loli` 都返回 `404 Route Not Found`, 不要用这两条.
- **Loli 类目本身**: `GET /api/edge/categories/40` 返回 `{'title':'Loli','slug':'loli','totalMediaCount':684,'nsfw':False}`.
- **覆盖**: 169 部中, 165 部拿到非空 categories, 4 部没有 Kitsu 映射.
- **萝莉信号很弱**: 只有 4 部带 Loli, 其中 2 部与手工表重合. 间谍过家家拿到的是 `['Assassin','Action','Comedy','Slice of Life','Supernatural','Family','Shounen']`; 天使降临到了我身边 拿到的是 `['Shoujo Ai','Comedy','Slice of Life']`.
- **投票 / 置信度**: 没有.
- **授权**: `robots.txt` 写着 `User-agent: * / Disallow: /api/`, 但这条是针对搜索引擎爬虫的; Kitsu 的 API 本身是公开且有文档的 JSON:API. 由于 AOD 已经合并了 Kitsu 的 categories (龙王的工作在 AOD 里也有 `loli`), **单独再爬 Kitsu 没有增量**, 所以否决为单独数据源.

### 2.4 ⚠️ AniList GraphQL: 不用标签集合, 改用"主角色年龄"和 `Primarily Child Cast` 的 rank 作为萝莉辅助信号

你否掉的是 `MediaTagCollection` (只看有没有萝莉维度). 这里改用两个它确实有的字段, 而且都带强度:

```
POST https://graphql.anilist.co
query { a0: Media(id:140960,type:ANIME){ id tags{name rank} characters(role:MAIN,perPage:10){nodes{name{full} age}} } }
```

真实返回 (摘录):
- 间谍过家家: `('Anya Forger','6 (self asserted)'), ('Yor Forger','27')`
- 魔法少女伊莉雅: `'Primarily Child Cast': 79`, `('Miyu Edelfelt','10')`
- 萝球社: `'Primarily Child Cast': 80`
- 天使的3P: `'Primarily Child Cast': 97`

- **字段路径**:
  - `Media.tags[].{name,rank}`, 其中 rank 取 0–100, 是社区投票得出的相关度百分比.
  - `Media.characters(role:MAIN).nodes[].age`, 是自由文本, 例如 `"9-13"`、`"10 (mentally), 18 (biologically)"`、`"300+ (Actual)"`, 需要解析首个数字.
- **覆盖**: 169/169 部都查到 (每批 20 个 alias 合并成一次请求, 间隔 2.5s, 共 9 次请求, 没有被限流).
- **主角色年龄≤12 的噪音来源**:
  - 合集类主角: 异世界四重奏里的 Tanya, 9-13 岁.
  - 心智年龄的写法: 盾之勇者的 Raphtalia, "10 (mentally)".
  - 13 岁擦边: 超电磁炮里的黑子, 12-13 岁.
  - 已知漏判: 龙王的工作 (AniList 主角色无年龄)、天使降临 (无)、龙女仆 (主角色只有 25 岁的 Kobayashi, 康娜是配角).
- **授权**: AniList API ToS 允许非商业使用, 限流 90 req/min.

### 2.5 ⚠️ AniDB HTTP API: 有带权重的 loli 标签, 但必须注册 client

```
$ curl -s --compressed "http://api.anidb.net:9001/httpapi?request=anime&client=test&clientver=1&protover=1&aid=13281"
<error code="302">client version missing or invalid</error>
```

- 需要用户在 AniDB 注册一个 HTTP client 名. 注册后 `<tags><tag weight="…">` 带 0–600 的权重, 是所有源里唯一有"程度"的萝莉标签.
- 限流非常严: 官方要求每 2 秒最多 1 个请求, 而且同一 aid 一天内只能拉一次, 违反会被封 IP.
- 另外, AOD 已经合并了 AniDB 的标签名 (只是丢掉了权重).
- 结论: **作为可选增强保留, 不作为主源.** 如果以后萝莉要按权重排序, 再去注册 client.
- 匿名可下的 `https://anidb.net/api/anime-titles.xml.gz` 返回 200, 1884819 B, 但里面只有标题, 没有标签.

### 2.6 ❌ Bilibili 番剧 styles

```
$ curl -s "https://api.bilibili.com/pgc/review/user?media_id=8932"
{"code":0,...,"media":{...,"media_id":8932,"season_id":21554,...,"title":"龙王的工作！","type":1,...
$ curl -s "https://api.bilibili.com/pgc/view/web/season?season_id=21554"   → result.styles
['小说改', '职场']
```

- bangumi-data 里的 `bilibili` id 是 media_id; 要先换成 season_id, 才能拿到 `result.styles`.
- 否决理由:
  - 标签非常粗, 龙王的工作只有"小说改 / 职场", 没有萝莉、百合这一层.
  - bangumi-data 的 bilibili 映射只有 3207/8925 条, 很多番不在 B 站上.
  - 接口没有公开授权, 而且有风控.

### 2.7 ❌ 萌娘百科 (含 raw 导出)

```
$ curl -s "https://zh.moegirl.org.cn/index.php?title=龙王的工作！&action=raw"
HTTP 403  <title>Just a moment...</title> ... challenges.cloudflare.com
```

- `action=raw` 被 Cloudflare 质询拦下. 加上你测过的 `api.php` 返回 action-notallowed, 两条路都不通.
- 第三方镜像本次没有找到稳定可用的: 公开镜像大多是过期快照, 授权也不清楚, 萌百内容是 CC BY-NC-SA.
- "萌点" 写在角色模板里, 是角色级而不是番剧级. 即使能抓, 也要自己做聚合.

### 2.8 ❌ 豆瓣

`https://movie.douban.com/subject/26934346/` 返回 200, 但只有 2965 B, 内容是 `<title>豆瓣</title>` 加上 loading 动画的 JS 挑战页. 拿不到标签, 而且 ToS 禁止抓取.

### 2.9 ❌ Annict / しょぼいカレンダー

- Annict: `GET https://api.annict.com/v1/works?filter_title=りゅうおうのおしごと` 返回 `HTTP 400`, 因为必须带 access_token. 而且 Annict 的作品数据没有标签字段 (只有 media / season / 外部 id), 没有萌属性维度.
- syobocal: `db.php?Command=TitleLookup&TID=4811` 返回 200 的 XML:
  ```xml
  <TitleItem id="4811"><TID>4811</TID><LastUpdate>2025-03-30 22:24:54</LastUpdate><Title>石原夏織のCarry up!?</Title>...
  ```
  它只有 `Cat` 这类大类 (アニメ / ラジオ 等) 和放送信息, 没有标签.

### 2.10 ❌ anime-planet / aniSearch / LiveChart (网页抓取)

- anime-planet: 页面 200 (99450 B). 龙王的工作的 tag 链接实测为 `apprenticeship, based-on-a-light-novel, board-games, comedy, seinen, shogi, slice-of-life`, 其中没有 loli. 它已经被 AOD 合并.
- aniSearch: 页面 200, 但 `robots.txt` 对多个 UA 写着 `Disallow: /`. 它也已被 AOD 合并.
- LiveChart: 页面 200, 但没有萌属性标签. 它也是 AOD 的来源之一.

### 2.11 ❌ Jikan (MAL 非官方) / Shikimori

- `https://api.jikan.moe/v4/anime/35905` 两次都在 30s 超时, 返回空 body.
- `https://shikimori.one/api/animes/35905` 第一次返回 200 (`"name":"Ryuuou no Oshigoto!","score":"6.82"...`), 第二次返回空.
- 两者都只有 MAL 的 genres / themes, 没有 loli 维度, 不值得再投入.

---

## 3. 落地方案

### 3.1 总体: 爬虫产出 `data/ANiMeTags.json` (生成物), 博客侧"源标签 + 手工覆盖"合并

```
HXLoLi-ANiMe (每周一次)
  bangumi-data (npm/jsDelivr) ─┐
  AOD release (GitHub)      ───┼─> scripts/crawler/Tags.py ─> data/ANiMeTags.json ─> 打 Tag / CDN
  AniList GraphQL (仅萝莉辅助)─┘
HXLoLi 博客
  animeStore.ts 再拉一份 /data/ANiMeTags.json
  src/utils/anime/tags.ts: 源标签 ∪ user_status.tags, 再用 data/animeTags.ts 做 add/remove 覆盖
```

### 3.2 爬虫: 新增 `scripts/crawler/Tags.py`, 不往 Api.py 里塞

`Api.py` 负责 Bangumi 数据, 体量已经有 23 KB, 标签是另一条管线, 所以单独开一个模块.

实现步骤:

1. 读取 `data/ANiMeRecord.json`, 取全部 `anime_data.id`.
2. 下载 bangumi-data 的 `dist/data.json`, 建 `bgm_id -> {mal, aniList, anidb, kitsu?}` 映射. 跳过没有 `id` 的 site.
3. 下载 AOD minified json (推荐 `.zst`, 6 MB; 需要 `zstandard` 依赖, 或者直接下 62 MB 的 json). 建 `source_url -> entry` 索引. 按 mal、aniList、anidb 的顺序找条目.
4. 用 AniList GraphQL 查询, 每批 20 个 alias, 批间 sleep 2.5s, 取:
   - `tags{name rank}`
   - `characters(role:MAIN){nodes{age}}`
5. 用下面的规则表把英文标签映射成站内中文标签, 并写入 `evidence`:

   | 站内 | AOD tags 命中任一 | 额外条件 |
   |---|---|---|
   | 异世界 | isekai, reincarnation, person in a strange world | — |
   | 校园 | school life, school club(s), high school, elementary school | 不用裸 `school` (太宽) |
   | 日常 | slice of life, iyashikei, cgdct | — |
   | 恋爱 | romance, romantic comedy | AniList `Romance` rank≥60 时才算主线 (可选) |
   | 搞笑 | comedy | 只在 AniList genres 含 Comedy 时才写, 防止 136 部全上 |
   | 战斗 | action, fighting, battle | — |
   | 催泪 | tragedy, tearjerker | — |
   | 悬疑 | mystery, psychological, thriller | — |
   | 百合 | yuri, shoujo ai, girls love | — |
   | 萝莉 | 见下方 | — |

   萝莉的规则:
   - **score** = 2×[AOD loli/lolicon] + 2×[Kitsu/AOD 已含] + [AniList Primarily Child Cast rank≥50] + [主角色解析年龄≤12].
   - score≥2 输出 `"萝莉"`, 置信度为 high; score==1 输出到 `candidates`, 不上墙.
   - 实测 46 部至少有 1 个信号, 召回 23/33.
6. 写出 `data/ANiMeTags.json`. 结构如下:

   ```json
   {
     "generatedAt": "2026-11-02T00:00:00+08:00",
     "sources": {
       "aod": {"release": "2026-27", "lastUpdate": "2026-07-04", "license": "ODbL-1.0 + DbCL-1.0"},
       "bangumiData": {"version": "0.3.234", "license": "CC-BY-4.0"},
       "anilist": {}
     },
     "items": {
       "219658": {
         "tags": ["萝莉", "日常", "搞笑", "恋爱"],
         "candidates": [],
         "evidence": {
           "萝莉": ["aod:loli", "aod:lolicon"],
           "日常": ["aod:slice of life"]
         },
         "ids": {"mal": 35905, "anilist": 99507, "anidb": 13281, "kitsu": 13661}
       }
     },
     "unmapped": [537743, 184740, 249681, 614606, 544024, 471345, 26804, 296195]
   }
   ```

7. 先拉取到临时结构, 全部成功后再写文件 (与 2026-11-01 的修复原则一致). 任何一个源失败时, 保留上一版的 `ANiMeTags.json` 不覆盖, 并让 workflow 以失败退出.
8. Agent Notes: 新增 `.agents/notes/implemented/feature/<date>-anime-tags-source.md`, 与代码放在同一个 diff 里.

### 3.3 Workflow

`anime-pipeline.yaml` 增加一个 `weekly` 的 cron job, 每周一 04:00 CST 跑 `Tags.py`, 产物跟随现有的 Tag / CDN 流程. 每天跑没有意义: AOD 最快也是一周一版, bangumi-data 也是按新番季更新.

### 3.4 博客侧 (HXLoLi)

- `animeStore.ts`: 与 `ANiMeRecord.json` 用同一个 `animeCdnVersion` Tag, 再 `reqAnimeByCDN('/data/ANiMeTags.json')`.
- `src/utils/anime/tags.ts`: 合并顺序为 `源 tags ∪ user_status.tags`, 然后套用 `data/animeTags.ts`.
- `data/animeTags.ts` 改成覆盖层, 格式为 `{ add?: string[], remove?: string[] }`:
  - 只记录与源不一致的条目, 例如学生会的一己之见要 add 萝莉、魔法禁书目录要 remove 萝莉.
  - 预计会从 157 条缩到几十条.
- `ANIME_TAG_DEFS` 中萝莉的 `rule` 改写成"来自 AOD / AniList 的证据 + 本站覆盖", 并在筛选面板 hover 时展示 `evidence`.
- 页脚加署名: "标签数据: anime-offline-database (ODbL), bangumi-data (CC-BY-4.0), AniList".

### 3.5 首次切换建议

先只生成 `ANiMeTags.json`, 博客侧不切换, 产出一份"源 vs 手工"的差异表给你逐条拍板. 本次已知的差异如下:

- **手工有、源无** (10 部): 学生会的一己之见 (含 Lv.2)、打了300年的史莱姆 (含第二季)、NEKOPARA OVA、GJ部、某科学的一方通行、小林家的龙女仆 怕寂寞的龙、无神世界的神明活动、邻家的吸血鬼小妹.
- **源有、手工无** (23 部), 例如: 魔法禁书目录 1–3、超电磁炮 / S / T、请问您今天要来点兔子吗 (含 BLOOM)、盾之勇者 (含第四季)、平凡职业 (含第二季)、拔作岛、日常、章鱼哔的原罪、无职转生、异世界四重奏 1–3、夏洛特、赛博朋克：边缘行者 2、某暗部的少女共栖、乱世千金倪亚‧利斯顿.

  其中不少只是"有 12 岁上下的配角", 按你的口径 ("核心角色 + 卖点") 应该 remove. 这正是需要覆盖层的原因.

---

## 4. 风险

### 4.1 数据源消失 / 停更

- AOD 最新版仍是 2026-27 (lastUpdate 2026-07-04), 约 4 个月没有新 release. 这期间的新番在 AOD 里会缺失, 本次 177 部里已有 2 部映射不到. 另外 GitHub API 对本出口返回 "account suspended", 已确认是出口侧的问题, 但说明从 Actions 拉取也要准备好重试.
- 对策:
  - 在仓库里缓存上一版 `ANiMeTags.json`.
  - 新番找不到条目时进 `unmapped`, 不报错.
  - 若 AOD 超过 8 周没有新 release, workflow 打 warning.
  - 必要时可退回 "AniList + Kitsu 直连" (本次两者都实测可用), 代价是请求数增加约 340 次.
- bangumi-data 是新番季更新; 有 6 部当前不在其中, 需要手工 `ids` 覆盖 (在覆盖层里允许写 `mal` id).

### 4.2 标签口径与用户预期不符 (最大风险)

- 英文社区的 `loli` 偏向"萝莉角色 / 萝莉控要素", 与你的"核心角色小学生 + 卖点"并不等价.
- 实测并集的精度只有 50%, 召回 70%. 典型偏差:
  - 魔法禁书目录被标 loli (茵蒂克丝 / 打止), 而你没标.
  - 学生会的一己之见 (合法萝莉会长) 你标了, 源里没有.
- AniList 的年龄是自由文本, 解析规则会误判 "10 (mentally)" 这种写法.
- `comedy` / `school` 等 genre 太宽. 不加截断规则的话, 「搞笑」会从 29 部膨胀到 136 部, 页面分类会失去区分度.
- 结论: **不能做到完全自动替代手工判断**. 方案的价值在于"每个标签都有可追溯证据 + 覆盖层很薄", 而不是消灭人工.

### 4.3 被封 / 限流

- AOD 和 bangumi-data 都是静态文件下载 (GitHub release / jsDelivr), 基本没有封禁风险.
- AniList 限流 90 req/min. 按 20 个 alias 一批, 每周约 9 个请求, 远低于限制.
- 不要引入 AniDB HTTP API, 除非注册了 client 并严格遵守 2 秒 1 请求、同 aid 每日一次; 违规会封 IP, 而 GitHub Actions 的出口是共享的.
- Kitsu 不需要直连 (AOD 已合并). 如果直连, 记得 curl 加 `-g` 或对 `[]` 做 URL 编码.

### 4.4 授权

- 博客展示需要署名:
  - AOD: ODbL / DbCL.
  - bangumi-data: CC-BY-4.0.
  - AniList: 按其 ToS, 非商业.
- 若把 `ANiMeTags.json` 作为数据库公开再分发, 衍生库需保持 ODbL. 博客仓库本身是公开的, 文件里已带 `sources.license`, 满足这一要求.
