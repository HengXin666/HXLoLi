import { ANIME_TAG_ALIASES, ANIME_TAG_DEFS, ANIME_TAGS, AnimeTagGroup } from "@site/data/animeTags";
import { ANiMeRecord } from "@site/src/utils/anime/types";
import { overlayTagsOf } from "@site/src/utils/anime/overlay";

/** 番剧页 URL 上的标签筛选参数: /anime?tag=萝莉 或多选 /anime?tag=萝莉,日常 (见 utils/anime/filters.ts) */
export const ANIME_TAG_QUERY = "tag";

const DEF_ORDER = new Map(ANIME_TAG_DEFS.map((d, i) => [d.tag, i]));
const DEF_BY_TAG = new Map(ANIME_TAG_DEFS.map((d) => [d.tag, d]));

/** 表外随手标签的分组名 */
export const OTHER_TAG_GROUP = "其他";
export type TagGroupName = AnimeTagGroup | typeof OTHER_TAG_GROUP;

/** 随手标签按 ANIME_TAG_ALIASES 归并 (纯爱 -> 恋爱) */
export function normalizeTag (tag: string): string {
    return ANIME_TAG_ALIASES[tag] ?? tag;
}

const tagCache = new WeakMap<ANiMeRecord, string[]>();

/**
 * 一部番的全部标签 = data/animeTags.ts 里本站补的分类 + user_status.tags 里追番时随手打的标签, 别名归并后去重.
 * 表里定义过的分类按 ANIME_TAG_DEFS 的顺序排前面, 其余随手标签保持原顺序跟在后面.
 * 两个来源都只读: 番剧数据仓库 (HXLoLi-ANiMe) 与本站标签表互不改写. 结果按记录对象缓存 (记录是冻结的, 筛选时每次按键都要算一遍).
 * 本地 overlay (data/animeOverlay.json) 给这部番写了 tags 时, 那份列表就是最终结果, 上面两个来源都不再参与.
 * 标签合并口径的决策见 src/utils/anime/filters.ts 与 src/utils/anime/overlay.ts 顶部注释
 */
export function getRecordTags (record: ANiMeRecord): string[] {
    const hit = tagCache.get(record);
    if (hit) return hit;
    const override = overlayTagsOf(record);
    const own = override ? [] : ANIME_TAGS[record.anime_data.id] ?? [];
    const user = (override ?? record.user_status.tags ?? []).map(normalizeTag);
    const all = [...new Set([...own, ...user])];
    const defined = all.filter((t) => DEF_ORDER.has(t)).sort((a, b) => DEF_ORDER.get(a)! - DEF_ORDER.get(b)!);
    const extra = all.filter((t) => !DEF_ORDER.has(t));
    const out = [...defined, ...extra];
    tagCache.set(record, out);
    return out;
}

export type TagCount = { tag: string; count: number; rule?: string; group: TagGroupName };

/** 标签所属的组; 表外随手标签归"其他" */
export function getTagGroup (tag: string): TagGroupName {
    return DEF_BY_TAG.get(tag)?.group ?? OTHER_TAG_GROUP;
}

/**
 * 按标签计数. 本站补的分类按定义顺序列出 (即使为 0); 只来自随手标签的定义 (userOnly) 与表外随手标签只在出现过时列出,
 * 表外的按数量降序跟在后面.
 */
export function countTags (records: readonly ANiMeRecord[]): TagCount[] {
    const counts = new Map<string, number>();
    for (const r of records) for (const t of getRecordTags(r)) counts.set(t, (counts.get(t) ?? 0) + 1);
    const defined: TagCount[] = ANIME_TAG_DEFS
        .filter((d) => !d.userOnly || counts.has(d.tag))
        .map((d) => ({ tag: d.tag, count: counts.get(d.tag) ?? 0, rule: d.rule, group: d.group }));
    const extra: TagCount[] = [...counts.entries()]
        .filter(([t]) => !DEF_ORDER.has(t))
        .sort((a, b) => b[1] - a[1])
        .map(([tag, count]) => ({ tag, count, group: OTHER_TAG_GROUP }));
    return [...defined, ...extra];
}
