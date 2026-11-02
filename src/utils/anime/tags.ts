import { ANIME_TAG_ALIASES, ANIME_TAG_DEFS, ANIME_TAGS, AnimeTagGroup } from "@site/data/animeTags";
import { ANiMeRecord } from "@site/src/utils/anime/types";
import { overlayTagsOf } from "@site/src/utils/anime/overlay";

/** 番剧页 URL 上的标签筛选参数: /anime?tag=ロリ 或多选 /anime?tag=ロリ,日常 (见 utils/anime/filters.ts) */
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

let tagCache = new WeakMap<ANiMeRecord, string[]>();

/** 爬虫产出的源标签 (HXLoLi-ANiMe data/ANiMeTags.json 的 items[id].tags), 由 animeStore 加载后经 setSourceTags 注入; 没拉到时为空 */
let sourceTags: ReadonlyMap<number, readonly string[]> = new Map();

/** ANiMeTags.json 里本站用得到的部分 */
export interface ANiMeTagsFile {
    items?: Record<string, { tags?: readonly string[] } | undefined>;
}

/** 解析 ANiMeTags.json; 结构不对时返回空表, 不抛错 */
export function parseSourceTags (file: unknown): Map<number, string[]> {
    const out = new Map<number, string[]>();
    const items = (file as ANiMeTagsFile | null)?.items;
    if (!items || typeof items !== "object") return out;
    for (const [k, v] of Object.entries(items)) {
        const id = Number(k);
        const tags = Array.isArray(v?.tags) ? v!.tags!.filter((x): x is string => typeof x === "string" && x.length > 0) : [];
        if (Number.isSafeInteger(id) && tags.length) out.set(id, tags);
    }
    return out;
}

/** 注入源标签并清掉按记录的缓存 (源标签比番剧记录晚到, 早算出的结果要作废) */
export function setSourceTags (map: ReadonlyMap<number, readonly string[]>): void {
    sourceTags = map;
    tagCache = new WeakMap();
}

/**
 * 一部番的全部标签 = 爬虫源标签 (ANiMeTags.json) ∪ user_status.tags 里追番时随手打的标签 ∪ data/animeTags.ts 手工表 (覆盖层, 补源标签漏判的), 别名归并后去重.
 * ANiMeTags.json 拉不到时源标签为空, 退化为 追番标签 ∪ 手工表.
 * 表里定义过的分类按 ANIME_TAG_DEFS 的顺序排前面, 其余随手标签保持原顺序跟在后面.
 * 两个来源都只读: 番剧数据仓库 (HXLoLi-ANiMe) 与本站标签表互不改写. 结果按记录对象缓存 (记录是冻结的, 筛选时每次按键都要算一遍).
 * 本地 overlay (data/animeOverlay.json) 给这部番写了 tags 时, 那份列表就是最终结果, 上面两个来源都不再参与.
 * 标签合并口径的决策见 src/utils/anime/filters.ts 与 src/utils/anime/overlay.ts 顶部注释
 */
export function getRecordTags (record: ANiMeRecord): string[] {
    const hit = tagCache.get(record);
    if (hit) return hit;
    const override = overlayTagsOf(record);
    const id = record.anime_data.id;
    const source = override ? [] : (sourceTags.get(id) ?? []).map(normalizeTag);
    const own = override ? [] : ANIME_TAGS[id] ?? [];
    const user = (override ?? record.user_status.tags ?? []).map(normalizeTag);
    const all = [...new Set([...source, ...user, ...own])];
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
