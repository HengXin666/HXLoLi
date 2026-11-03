# 第 4 条判定落地清单: 萝莉 显示名改为 ロリ (博客侧)

2026-11-02. HXLoLi-ANiMe 侧 `Tags.py` 已完成: `data/ANiMeTags.json` 只写 `ロリ`. 博客仓库按约定只读, 以下改动本轮未做, 交给下一步实现.

## 实测结论: URL 无 ASCII 假设

- `src/utils/anime/filters.ts` 的 `parseFilters` 用 `new URLSearchParams(search)` + `splitList` (按逗号切) + `normalizeTag`. `serializeFilters` 用 `URLSearchParams.set` 后只把 `%2C` 还原成逗号. 没有 ASCII 正则或 charCode 判断
- node 实测: `p.set("tag","ロリ,百合")` 序列化为 `tag=%E3%83%AD%E3%83%AA,%E7%99%BE%E5%90%88`, 再 `new URLSearchParams(s).get("tag")` 读回 `ロリ,百合`. 所以 `/anime?tag=ロリ` 与已编码形式都能工作
- 旧链接 `/anime?tag=萝莉`: `parseFilters` 已对 URL 标签做 `normalizeTag`, 加上别名后自动变成 `ロリ`. `serializeFilters` 下一次写 URL 时会输出 `ロリ`

## 必改

1. `data/animeTags.ts`
   - `ANIME_TAG_DEFS` 中 `{ tag: "萝莉", ... }` 改为 `{ tag: "ロリ", ... }`
   - `ANIME_TAG_ALIASES` 增加 `萝莉: "ロリ"` (现有只有 `纯爱: "恋爱"`)
   - 删除 `{ tag: "舰娘", ... }` 定义 (第 2 条判定)
   - 手工表 35 条 `"萝莉"`: 写入时统一改成 `"ロリ"`. 即使暂时不改, 有第 2 点也能读对
2. `src/utils/anime/tags.ts`, `getRecordTags`
   - 现状 `const own = override ? [] : ANIME_TAGS[id] ?? [];` **没有过 `normalizeTag`**, 只有 user / overlay 标签做了归并. 所以手工表里残留的 `"萝莉"` 不会被认成 `ロリ`
   - 改为 `(... ?? []).map(normalizeTag)`
   - 顶部注释里的示例 `/anime?tag=萝莉` 改为 `ロリ`
3. 舰娘清理: `user_status.tags` 里仍有 `舰娘` (297168 碧蓝航线：微速前行！). 删掉定义后, 它会落入"其他"组. 若要完全不出现, 需要在 `getRecordTags` 中过滤 (例如 `DROPPED_TAGS = new Set(["舰娘"])`). 或者在 Bangumi 收藏里删掉这个随手标签, 那是数据源头, 爬虫会同步
4. overlay (`data/animeOverlay.json`) 若写过 `萝莉`, 已经走 `normalizeTag`, 无需改

## 验证建议

- `/anime?tag=ロリ`, `/anime?tag=%E3%83%AD%E3%83%AA` 与 `/anime?tag=萝莉` 三者结果一致, 且地址栏最终都显示 `tag=ロリ`
- 筛选面板只出现 `ロリ`, 不出现 `萝莉` 与 `舰娘`
