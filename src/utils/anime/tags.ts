import { ANIME_SUSUME_DEFS, ANIME_TAG_ALIASES, ANIME_TAG_DEFS, AnimeSusumeDef, AnimeTagGroup } from "@site/data/animeTags";
import { ANiMeRecord } from "@site/src/utils/anime/types";
import type { AnimeLabel } from "@site/src/utils/anime/labels";

/**
 * 番剧标签 = **只读人工标注** HXLoLi-ANiMe/data/labels/<id>.json (加载见 utils/anime/labels.ts, 注入见 animeStore.ts).
 * 2026-11-04 tag 重构 (hx-anime-tag-refactor-design.md 第 4 / 10 节): 不再合并
 *   - HXLoLi-ANiMe/data/ANiMeTags.json 的自动上墙分配 (原 parseSourceTags / setSourceTags 已删除),
 *   - data/animeTags.ts 的手工分配表 (已删除, 那边只剩词表),
 *   - data/animeOverlay.json 的 tags (overlay 不再接管标签, 见 utils/anime/overlay.ts),
 *   - user_status.tags (追番随手标签, bgm 同步回来的也在这里; 不读它即防回环).
 * 未标注 = 没有文件 = 空数组, 页面不展示任何 tag.
 *
 * 两类标签:
 *   - 推荐分类 (物凄い / ロリ / シスコン): 有独立专属页 /anime/susume/<slug>, 渲染成特殊徽章, 不进 ?tag= 筛选.
 *   - 普通标注 (九项): 走 /anime?tag= 筛选.
 */

/** 番剧页 URL 上的普通标注筛选参数: /anime?tag=推理 或多选 /anime?tag=推理,致郁 (见 utils/anime/filters.ts) */
export const ANIME_TAG_QUERY = "tag";

/** 推荐分类排前, 普通标注按词表顺序 */
const DEF_ORDER = new Map<string, number>([
    ...ANIME_SUSUME_DEFS.map((d, i) => [d.display, i] as const),
    ...ANIME_TAG_DEFS.map((d, i) => [d.tag, ANIME_SUSUME_DEFS.length + i] as const),
]);
const DEF_BY_TAG = new Map(ANIME_TAG_DEFS.map((d) => [d.tag, d]));
const SUSUME_BY_TAG = new Map(ANIME_SUSUME_DEFS.map((d) => [d.display, d]));
const SUSUME_BY_SLUG = new Map(ANIME_SUSUME_DEFS.map((d) => [d.slug, d]));

/** 词表外标签的分组名 (标注已按词表过滤, 正常不会出现; 筛选面板保留这一栏只为兼容) */
export const OTHER_TAG_GROUP = "其他";
export type TagGroupName = AnimeTagGroup | typeof OTHER_TAG_GROUP;

/** 旧链接里的标签名按 ANIME_TAG_ALIASES 归并 (萝莉 -> ロリ, 妹控 -> シスコン) */
export function normalizeTag (tag: string): string {
    return ANIME_TAG_ALIASES[tag] ?? tag;
}

// ---------------------------------------------------------------- 推荐分类

/** 这个标签是不是推荐分类; 是则返回它的定义 (专属页 slug 等) */
export function susumeDefOfTag (tag: string): AnimeSusumeDef | undefined {
    return SUSUME_BY_TAG.get(normalizeTag(tag));
}

export function susumeDefOfSlug (slug: string): AnimeSusumeDef | undefined {
    return SUSUME_BY_SLUG.get(slug);
}

/** 推荐分类专属页路径 (不含 baseUrl; Link 会自己补) */
export function susumePath (def: AnimeSusumeDef): string {
    return `/anime/susume/${def.slug}`;
}

// ---------------------------------------------------------------- 标注表

let labels: ReadonlyMap<number, AnimeLabel> = new Map();
let tagCache = new WeakMap<ANiMeRecord, string[]>();

/** 注入人工标注并清掉按记录的缓存 (标注比番剧记录晚到, 早算出的结果要作废) */
export function setAnimeLabels (map: ReadonlyMap<number, AnimeLabel>): void {
    labels = map;
    tagCache = new WeakMap();
}

/** 这部番的人工标注; 未标注返回 undefined */
export function getRecordLabel (record: ANiMeRecord): AnimeLabel | undefined {
    return labels.get(record.anime_data.id);
}

/**
 * 一部番的全部标签 = 它的标注文件里的 tags (已按词表过滤), 推荐分类在前、普通标注按词表顺序.
 * 只读 data/labels; 不读 user_status.tags / animeOverlay.json.tags / ANiMeTags.json / animeTags.ts 分配表.
 */
export function getRecordTags (record: ANiMeRecord): string[] {
    const hit = tagCache.get(record);
    if (hit) return hit;
    const own = getRecordLabel(record)?.tags ?? [];
    const out = [...new Set(own)]
        .filter((t) => DEF_ORDER.has(t))
        .sort((a, b) => DEF_ORDER.get(a)! - DEF_ORDER.get(b)!);
    tagCache.set(record, out);
    return out;
}

/** 只要普通标注 (走 ?tag= 筛选的九项) */
export function getRecordGenreTags (record: ANiMeRecord): string[] {
    return getRecordTags(record).filter((t) => DEF_BY_TAG.has(t));
}

/** 只要推荐分类 */
export function getRecordSusume (record: ANiMeRecord): AnimeSusumeDef[] {
    return getRecordTags(record).map((t) => SUSUME_BY_TAG.get(t)).filter((d): d is AnimeSusumeDef => !!d);
}

/** 本人评分: 标注文件的 rate 优先, 其次 overlay 的 rate; 没有返回 undefined */
export function getRecordRate (record: ANiMeRecord): number | undefined {
    return getRecordLabel(record)?.rate ?? record.user_status.rate;
}

/** 一句话观后感: 标注文件的 review 优先, 其次追番短评 */
export function getRecordReview (record: ANiMeRecord): string {
    return (getRecordLabel(record)?.review ?? record.user_status.comment ?? "").trim();
}

/** 进某个推荐分类的番, 按本人评分降序 (未评分垫底), 同分按 Bangumi 评分, 再按最近更新 */
export function recordsOfSusume (records: readonly ANiMeRecord[], def: AnimeSusumeDef): ANiMeRecord[] {
    return records
        .filter((r) => getRecordTags(r).includes(def.display))
        .sort((a, b) =>
            (getRecordRate(b) ?? 0) - (getRecordRate(a) ?? 0)
            || (b.anime_data.score || 0) - (a.anime_data.score || 0)
            || (b.user_status.last_update ?? "").localeCompare(a.user_status.last_update ?? ""));
}

// ---------------------------------------------------------------- 普通标注计数 (筛选面板 / 首页)

export type TagCount = { tag: string; count: number; rule?: string; group: TagGroupName };

/** 标签所属的组 */
export function getTagGroup (tag: string): TagGroupName {
    return DEF_BY_TAG.get(tag)?.group ?? OTHER_TAG_GROUP;
}

/**
 * 普通标注按词表顺序计数 (即使为 0 也列出). 推荐分类不在这里: 它们有专属页, 不进 ?tag= 筛选.
 */
export function countTags (records: readonly ANiMeRecord[]): TagCount[] {
    const counts = new Map<string, number>();
    for (const r of records) for (const t of getRecordGenreTags(r)) counts.set(t, (counts.get(t) ?? 0) + 1);
    return ANIME_TAG_DEFS.map((d) => ({ tag: d.tag, count: counts.get(d.tag) ?? 0, rule: d.rule, group: d.group }));
}
