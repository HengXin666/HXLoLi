# Tags.py 实测运行统计 (2026-11-02)

临时文件, 给 Step 2 差异表之前过目用. 产物: HXLoLi-ANiMe `data/ANiMeTags.json`; 脚本: `scripts/crawler/Tags.py`.

## 运行环境

- 输入: 仓库当前 `data/ANiMeRecord.json` (160 部). 注意: 2026-11-01 修复后的爬虫尚未在本机跑过, 那次沙箱实测是 177 部, 合入后再跑会变化
- AOD release `2026-27` (lastUpdate 2026-07-04), 联网下载 62 MB minified json, 未用 zst
- bangumi-data `0.3.234`; AniList GraphQL 每批 20 部, 间隔 2.5s
- 生成时间 2026-11-02T00:22:03+08:00, 耗时约 30 s

## 判据核对

| 判据 | 结果 |
|---|---|
| 1 点兔系列在ロリ上墙 | 88287 请问您今天要来点兔子吗？ 与 260770 BLOOM 均上墙; 规则本身已命中 (score 3 / 4), 另带 judgement 兜底. **库里没有第二季条目** (bgm 名称前缀兜底已写, 第二季入库后自动命中) |
| 2 舰娘不出现 | `grep -c 舰娘 ANiMeTags.json` = 0; 源码 DROPPED_TAGS 兜底 + assert |
| 3 妹控有源标签支撑 | 见下节 |
| 4 每个标签 evidence 非空 | 脚本内 assert, 0 个空 evidence |
| 5 统计 | 见下节 |

## 统计

- 总部数 160, 覆盖 154 (至少命中 AOD 或 AniList), unmapped 6: [537743, 26804, 184740, 296195, 249681, 614606]
- 在 bangumi-data 156, 在 AOD 154, 在 AniList 154
- 上墙: ロリ 26, 百合 24, 妹控 4, 异世界 48, 校园 75, 日常 85, 恋爱 32, 搞笑 89, 战斗 85, 催泪 63, 悬疑 52
- candidates (score==1): ロリ 18, 妹控 9
- 沙箱 177 部版本 (修复后爬虫产物) 对照: 覆盖 169, unmapped 8, ロリ上墙 29 / candidates 20, 妹控 4 / 9

## 妹控的源标签 (实测)

| 源 | 字段路径 | 标签名 | 全库条数 | 本库命中 | 分值 |
|---|---|---|---|---|---|
| AOD | `data[].tags[]` | `imouto` | 10 | 2 | +2 |
| AOD | `data[].tags[]` | `sister complex` | 10 | 0 | +2 |
| AOD | `data[].tags[]` | `brother-sister incest` | 35 | 0 | +2 |
| AOD | `data[].tags[]` | `siblings` | 237 | 8 | +1 |
| AniList | `Media.tags[].{name,rank}` | `Incest` (Theme-Romance) | — | 见 evidence | rank≥20 +1 |
| Kitsu | `/anime/{id}/categories` | 无. slug `sister-complex` / `siscon` / `imouto` / `incest` 均 `meta.count=0` | 0 | 0 | 不用 |

AniList `MediaTagCollection` 中与兄妹相关的只有 Incest / Twins / Shimaidon, 没有 imouto 或 siscon.

妹控上墙与候选:

- 上墙 517057 【我推的孩子】 第三季: ['aod:siblings', 'anilist:Incest rank=54', 'score=2']
- 上墙 213076 如果有妹妹就好了。: ['aod:imouto', 'score=2']
- 上墙 79227 游戏人生: ['aod:siblings', 'anilist:Incest rank=30', 'score=2']
- 上墙 172498 埃罗芒阿老师: ['aod:imouto', 'aod:siblings', 'score=3']
- 候选 277554 无职转生～到了异世界就拿出真本事～: ['anilist:Incest rank=20', 'score=1']
- 候选 235130 碧蓝之海: ['anilist:Incest rank=46', 'score=1']
- 候选 328313 世界顶尖的暗杀者转生为异世界贵族: ['anilist:Incest rank=60', 'score=1']
- 候选 212775 干物妹！小埋R: ['aod:siblings', 'score=1']
- 候选 120187 干物妹！小埋: ['aod:siblings', 'score=1']
- 候选 443428 【我推的孩子】 第二季: ['aod:siblings', 'score=1']
- 候选 386809 【我推的孩子】: ['aod:siblings', 'score=1']
- 候选 378862 别当欧尼酱了！: ['aod:siblings', 'score=1']
- 候选 569116 碧蓝之海 第三季: ['anilist:Incest rank=50', 'score=1']

## ロリ (原 萝莉) 规则 (按用户三条判定修订)

```
aod loli|lolicon +2; Bangumi 主角+配角中 >=2 个女性 小学生年纪(年龄最大值<=12/小学N年级)/幼女外形(身高<=145cm 且首个年龄<=14) +2, 仅 1 个且为主角 +1; AniList Primarily Child Cast rank>=50 +1; AniList 主角色年龄<=12 +1
```
核心角色群 = Bangumi 角色 relation 为 主角 或 配角 (不含闲角), 只计女性. 点兔的智乃 / 奈津惠 / 麻耶 Bangumi 年龄为 "13岁初二生", 身高 144 / 145 / 140cm, 走的是 "幼女外形" 分支. 门槛: 身高 <= 145cm 且首个年龄 <= 14. 这是让点兔按规则命中的最低门槛, 也会让同类 (初中生 + 矮个, 2 人以上) 番剧进入, Step 2 差异表里会逐条列出.

## 第 4 条判定: 萝莉 改名 ロリ

- `Tags.py` 输出一律写 `ロリ`, `grep -c 萝莉 data/ANiMeTags.json` = 0; 旧名 `萝莉` 只在 `LEGACY_TAG_ALIASES` 里出现
- 博客侧需改的点 (HXLoLi 只读, 本轮未改, 见 `.agents/research/anime-tag-rename-loli.md`)

## ANiMeTags.json 前 20 条

```json
{"443831": {"name_cn": "Fate/strange Fake", "tags": ["战斗", "催泪", "悬疑"], "candidates": [], "evidence": {"战斗": ["aod:action", "aod:fighting"], "催泪": ["aod:tragedy"], "悬疑": ["aod:mystery", "aod:psychological", "aod:thriller"]}, "candidateEvidence": {}, "ids": {"anidb": 18093, "anilist": 166617, "kitsu": 47682, "mal": 55830}}}
{"517057": {"name_cn": "【我推的孩子】 第三季", "tags": ["妹控", "异世界", "日常", "催泪", "悬疑"], "candidates": [], "evidence": {"妹控": ["aod:siblings", "anilist:Incest rank=54", "score=2"], "异世界": ["aod:reincarnation"], "日常": ["aod:slice of life"], "催泪": ["aod:tragedy"], "悬疑": ["aod:mystery", "aod:psychological"]}, "candidateEvidence": {}, "ids": {"anidb": 18901, "anilist": 182587, "kitsu": 49264, "mal": 60058}}}
{"255526": {"name_cn": "Fate/Grand Order -绝对魔兽战线巴比伦尼亚-", "tags": ["异世界", "战斗", "催泪"], "candidates": [], "evidence": {"异世界": ["aod:isekai", "aod:person in a strange world", "aod:reincarnation"], "战斗": ["aod:action"], "催泪": ["aod:tragedy"]}, "candidateEvidence": {}, "ids": {"anidb": 14238, "anilist": 103275, "kitsu": 42269, "mal": 38084}}}
{"515759": {"name_cn": "葬送的芙莉莲 第二季", "tags": ["日常", "催泪"], "candidates": [], "evidence": {"日常": ["aod:iyashikei", "aod:slice of life"], "催泪": ["aod:tragedy"]}, "candidateEvidence": {}, "ids": {"anidb": 18886, "anilist": 182255, "kitsu": 49240, "mal": 59978}}}
{"564421": {"name_cn": "异世界四重奏 第三季", "tags": ["异世界", "校园", "日常", "搞笑"], "candidates": ["ロリ"], "evidence": {"异世界": ["aod:isekai", "aod:person in a strange world"], "校园": ["aod:school life"], "日常": ["aod:slice of life"], "搞笑": ["aod:comedy", "anilist:genre Comedy"]}, "candidateEvidence": {"ロリ": ["anilist:主角色年龄<=12 Raphtalia (10 (mentally), 18 (biologically))", "score=1"]}, "ids": {"anidb": 19385, "anilist": 194447, "mal": 61851}}}
{"445905": {"name_cn": "判处勇者刑 惩罚勇者9004队刑务纪录", "tags": ["异世界", "战斗", "催泪", "悬疑"], "candidates": [], "evidence": {"异世界": ["aod:isekai"], "战斗": ["aod:action"], "催泪": ["aod:tragedy"], "悬疑": ["aod:mystery", "aod:psychological", "aod:thriller"]}, "candidateEvidence": {}, "ids": {"anidb": 18124, "anilist": 167152, "kitsu": 47759, "mal": 56009}}}
{"262865": {"name_cn": "异世界四重奏", "tags": ["异世界", "校园", "日常", "搞笑"], "candidates": ["ロリ"], "evidence": {"异世界": ["aod:isekai", "aod:person in a strange world"], "校园": ["aod:high school", "aod:school life"], "日常": ["aod:slice of life"], "搞笑": ["aod:comedy", "anilist:genre Comedy"]}, "candidateEvidence": {"ロリ": ["anilist:主角色年龄<=12 Tanya Degurechaff (9-13)", "score=1"]}, "ids": {"anidb": 14435, "anilist": 104454, "kitsu": 42032, "mal": 38472}}}
{"285422": {"name_cn": "异世界四重奏 第二季", "tags": ["异世界", "校园", "日常", "搞笑"], "candidates": ["ロリ"], "evidence": {"异世界": ["aod:isekai", "aod:person in a strange world"], "校园": ["aod:school life"], "日常": ["aod:slice of life"], "搞笑": ["aod:comedy", "anilist:genre Comedy"]}, "candidateEvidence": {"ロリ": ["anilist:主角色年龄<=12 Tanya Degurechaff (9-13)", "score=1"]}, "ids": {"anidb": 14964, "anilist": 110178, "kitsu": 42410, "mal": 39988}}}
{"510753": {"name_cn": "靠死亡游戏混饭吃。", "tags": ["百合", "战斗", "催泪", "悬疑"], "candidates": [], "evidence": {"百合": ["aod:yuri"], "战斗": ["aod:action"], "催泪": ["aod:tragedy"], "悬疑": ["aod:mystery", "aod:psychological", "aod:thriller"]}, "candidateEvidence": {}, "ids": {"anidb": 18845, "anilist": 180746, "kitsu": 50337, "mal": 59711}}}
{"333176": {"name_cn": "这个医师超麻烦", "tags": ["日常", "搞笑", "战斗"], "candidates": [], "evidence": {"日常": ["aod:slice of life"], "搞笑": ["aod:comedy", "anilist:genre Comedy"], "战斗": ["aod:action"]}, "candidateEvidence": {}, "ids": {"anidb": 16249, "anilist": 132351, "kitsu": 44383, "mal": 48742}}}
{"514353": {"name_cn": "小林家的龙女仆 怕寂寞的龙", "tags": ["ロリ", "百合", "异世界", "日常", "战斗"], "candidates": [], "evidence": {"ロリ": ["bgm:核心角色群 康娜卡姆依[主角|年龄 9岁（人类）], 才川莉子[配角|年龄 9岁]", "score=2"], "百合": ["aod:yuri"], "异世界": ["aod:isekai", "aod:person in a strange world"], "日常": ["aod:slice of life"], "战斗": ["aod:action"]}, "candidateEvidence": {}, "ids": {"anidb": 18873, "anilist": 181839, "kitsu": 49215, "mal": 59897}}}
{"193373": {"name_cn": "剧场版 Fate/kaleid liner 魔法少女☆伊莉雅 雪下的誓言", "tags": ["异世界", "校园", "战斗", "催泪", "悬疑"], "candidates": ["ロリ"], "evidence": {"异世界": ["aod:isekai"], "校园": ["aod:school life"], "战斗": ["aod:action"], "催泪": ["aod:tragedy"], "悬疑": ["aod:psychological"]}, "candidateEvidence": {"ロリ": ["anilist:主角色年龄<=12 Miyu Edelfelt (10)", "score=1"]}, "ids": {"anidb": 12420, "anilist": 97757, "kitsu": 12555, "mal": 34100}}}
{"147466": {"name_cn": "Fate/kaleid liner 魔法少女☆伊莉雅 3rei!!", "tags": ["ロリ", "百合", "异世界", "搞笑", "战斗"], "candidates": [], "evidence": {"ロリ": ["bgm:主角 伊莉雅斯菲尔·冯·爱因兹贝伦 (Prisma☆Illya)[简介 小学五年级]", "anilist:Primarily Child Cast rank=50", "anilist:主角色年龄<=12 Illyasviel von Einzbern (8 (Fate/Zero), 18 (Fate/stay night)), Miyu Edelfelt (10)", "score=3"], "百合": ["aod:yuri"], "异世界": ["aod:isekai"], "搞笑": ["aod:comedy", "anilist:genre Comedy"], "战斗": ["aod:action"]}, "candidateEvidence": {}, "ids": {"anidb": 11639, "anilist": 21379, "kitsu": 11438, "mal": 31706}}}
{"113134": {"name_cn": "Fate/kaleid liner 魔法少女☆伊莉雅 2wei Herz!", "tags": ["ロリ", "百合", "异世界", "日常", "搞笑", "战斗"], "candidates": [], "evidence": {"ロリ": ["bgm:主角 伊莉雅斯菲尔·冯·爱因兹贝伦 (Prisma☆Illya)[简介 小学五年级]", "anilist:主角色年龄<=12 Illyasviel von Einzbern (8 (Fate/Zero), 18 (Fate/stay night)), Miyu Edelfelt (10)", "score=2"], "百合": ["aod:yuri"], "异世界": ["aod:isekai"], "日常": ["aod:slice of life"], "搞笑": ["aod:comedy", "anilist:genre Comedy"], "战斗": ["aod:action"]}, "candidateEvidence": {}, "ids": {"anidb": 10833, "anilist": 20845, "kitsu": 9095, "mal": 27525}}}
{"565802": {"name_cn": "辉夜大小姐想让我告白 通往大人的阶梯", "tags": ["校园", "日常", "恋爱", "搞笑", "悬疑"], "candidates": [], "evidence": {"校园": ["aod:school life"], "日常": ["aod:slice of life"], "恋爱": ["aod:romance", "aod:romantic comedy", "anilist:genre Romance"], "搞笑": ["aod:comedy", "anilist:genre Comedy"], "悬疑": ["aod:psychological"]}, "candidateEvidence": {}, "ids": {"anidb": 19402, "anilist": 194884, "kitsu": 49998, "mal": 61903}}}
{"83402": {"name_cn": "Fate/kaleid liner 魔法少女☆伊莉雅 2wei!", "tags": ["ロリ", "百合", "异世界", "校园", "日常", "搞笑", "战斗"], "candidates": [], "evidence": {"ロリ": ["bgm:主角 伊莉雅斯菲尔·冯·爱因兹贝伦 (Prisma☆Illya)[简介 小学五年级]", "anilist:Primarily Child Cast rank=60", "anilist:主角色年龄<=12 Illyasviel von Einzbern (8 (Fate/Zero), 18 (Fate/stay night)), Miyu Edelfelt (10)", "score=3"], "百合": ["aod:yuri"], "异世界": ["aod:isekai"], "校园": ["aod:elementary school", "aod:school life"], "日常": ["aod:slice of life"], "搞笑": ["aod:comedy", "anilist:genre Comedy"], "战斗": ["aod:action"]}, "candidateEvidence": {}, "ids": {"anidb": 10091, "anilist": 20467, "kitsu": 8003, "mal": 20509}}}
{"377607": {"name_cn": "无神世界的神明活动", "tags": ["异世界", "日常", "搞笑", "战斗", "悬疑"], "candidates": [], "evidence": {"异世界": ["aod:isekai", "aod:person in a strange world", "aod:reincarnation"], "日常": ["aod:slice of life"], "搞笑": ["aod:comedy", "anilist:genre Comedy"], "战斗": ["aod:action", "aod:fighting"], "悬疑": ["aod:mystery"]}, "candidateEvidence": {}, "ids": {"anidb": 17337, "anilist": 148048, "kitsu": 46089, "mal": 51693}}}
{"498378": {"name_cn": "间谍过家家 第三季", "tags": ["日常", "搞笑", "战斗", "催泪"], "candidates": ["ロリ"], "evidence": {"日常": ["aod:slice of life"], "搞笑": ["aod:comedy", "anilist:genre Comedy"], "战斗": ["aod:action"], "催泪": ["aod:tragedy"]}, "candidateEvidence": {"ロリ": ["anilist:主角色年龄<=12 Anya Forger (6 (self asserted))", "score=1"]}, "ids": {"anidb": 18677, "anilist": 177937, "kitsu": 48939, "mal": 59027}}}
{"285757": {"name_cn": "一拳超人 第三季", "tags": ["搞笑", "战斗"], "candidates": [], "evidence": {"搞笑": ["aod:comedy", "anilist:genre Comedy"], "战斗": ["aod:action", "aod:fighting"]}, "candidateEvidence": {}, "ids": {"anidb": 17576, "anilist": 153800, "kitsu": 46360, "mal": 52807}}}
{"537743": {"name_cn": "某科学的超电磁炮 第四季", "tags": [], "candidates": [], "evidence": {}, "candidateEvidence": {}, "ids": {}}}
```
