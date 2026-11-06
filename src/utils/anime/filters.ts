import { ANiMeRecord, EpisodeType, WatchStatus } from "@site/src/utils/anime/types";
import { ANIME_TAG_QUERY, countTags, getRecordTags, normalizeTag, OTHER_TAG_GROUP, TagCount } from "@site/src/utils/anime/tags";

/**
 * 番剧页筛选的全部状态与纯函数: URL <-> 状态、组合匹配、分面计数、排序分组、季度归属.
 * 状态完整写在 URL 上 (?q=&status=watching,watched&tag=萝莉,日常&match=any&year=2024&score=8&sort=air),
 * 默认值不写, 所以 /anime 就是默认视图, 分享链接能还原全部筛选. 只有视图密度 (海报 / 列表) 存本地.
 * 组合语义: 维度之间"且"; 观看状态内"或"; 标签默认"全部命中"(且), 可切"任一命中"(或).
 * ?fold=series 切到「系列」视图: 筛选仍按单季判断, 系列只要有一季命中就显示 (分组 / 排序见 utils/anime/series.ts 的 groupSeries).
 * .agents/notes/implemented/feature/2026-11-01-anime-page-refactor.md
 */
export type SortKey = "update" | "air" | "score" | "name";
export type TagMatch = "all" | "any";
/** 卡片单位: none = 一部一张 (默认); series = 同系列各季聚成一张 (「系列」视图, 见 utils/anime/series.ts). 与排序正交, 不是筛选条件 */
export type FoldMode = "none" | "series";

export interface AnimeFilters {
    q: string;
    status: WatchStatus[];
    tags: string[];
    match: TagMatch;
    year: number | null;
    minScore: number | null;
    sort: SortKey;
    fold: FoldMode;
}

export const DEFAULT_FILTERS: AnimeFilters = {
    q: "",
    status: [],
    tags: [],
    match: "all",
    year: null,
    minScore: null,
    sort: "update",
    fold: "none",
};

export const STATUS_OPTIONS: readonly { value: WatchStatus; key: string; label: string }[] = [
    { value: WatchStatus.WATCHING, key: "watching", label: "在看" },
    { value: WatchStatus.WATCHED, key: "watched", label: "看过" },
    { value: WatchStatus.WANT_TO_WATCH, key: "wish", label: "想看" },
    { value: WatchStatus.ON_HOLD, key: "onhold", label: "搁置" },
    { value: WatchStatus.DROPPED, key: "dropped", label: "抛弃" },
];

export const SORT_OPTIONS: readonly { value: SortKey; label: string }[] = [
    { value: "update", label: "最近更新" },
    { value: "air", label: "首播日期" },
    { value: "score", label: "评分" },
    { value: "name", label: "名称" },
];

export const SCORE_OPTIONS: readonly number[] = [6, 7, 8];

export function statusLabel (s: WatchStatus): string {
    return STATUS_OPTIONS.find((o) => o.value === s)?.label ?? "未知";
}

const STATUS_BY_KEY = new Map(STATUS_OPTIONS.map((o) => [o.key, o.value]));
const KEY_BY_STATUS = new Map(STATUS_OPTIONS.map((o) => [o.value, o.key]));
const SORT_KEYS = new Set<string>(SORT_OPTIONS.map((o) => o.value));

const splitList = (raw: string | null): string[] =>
    (raw ?? "").split(",").map((s) => s.trim()).filter(Boolean);

/** URL 查询串 -> 筛选状态. 认不出的值丢弃, 不报错; 也兼容旧链接的单个 ?tag=萝莉 */
export function parseFilters (search: string): AnimeFilters {
    const p = new URLSearchParams(search);
    const status = splitList(p.get("status"))
        .map((k) => STATUS_BY_KEY.get(k))
        .filter((v): v is WatchStatus => v !== undefined);
    const year = Number(p.get("year"));
    const score = Number(p.get("score"));
    const sort = p.get("sort") ?? "";
    return {
        q: (p.get("q") ?? "").trim(),
        status: [...new Set(status)],
        tags: [...new Set(splitList(p.get(ANIME_TAG_QUERY)).map(normalizeTag))],
        match: p.get("match") === "any" ? "any" : "all",
        year: Number.isInteger(year) && year > 1900 ? year : null,
        minScore: Number.isFinite(score) && score > 0 ? score : null,
        sort: SORT_KEYS.has(sort) ? (sort as SortKey) : DEFAULT_FILTERS.sort,
        fold: p.get("fold") === "series" ? "series" : DEFAULT_FILTERS.fold,
    };
}

/** 筛选状态 -> 查询串 (不含"?"); 默认值不写 */
export function serializeFilters (f: AnimeFilters): string {
    const p = new URLSearchParams();
    if (f.q) p.set("q", f.q);
    if (f.status.length) p.set("status", f.status.map((s) => KEY_BY_STATUS.get(s)).join(","));
    if (f.tags.length) p.set(ANIME_TAG_QUERY, f.tags.join(","));
    if (f.tags.length > 1 && f.match === "any") p.set("match", "any");
    if (f.year !== null) p.set("year", String(f.year));
    if (f.minScore !== null) p.set("score", String(f.minScore));
    if (f.sort !== DEFAULT_FILTERS.sort) p.set("sort", f.sort);
    if (f.fold !== DEFAULT_FILTERS.fold) p.set("fold", f.fold);
    // URLSearchParams 会把逗号编码成 %2C, 还原成逗号, 链接可读
    return p.toString().replace(/%2C/g, ",");
}

/** 一个生效中的条件: 显示成可单独去掉的 chip */
export interface ActiveCondition {
    key: string;
    label: string;
    without: AnimeFilters;
}

export function activeConditions (f: AnimeFilters): ActiveCondition[] {
    const out: ActiveCondition[] = [];
    if (f.q) out.push({ key: "q", label: `搜索「${f.q}」`, without: { ...f, q: "" } });
    for (const s of f.status) {
        out.push({ key: `status-${s}`, label: statusLabel(s), without: { ...f, status: f.status.filter((x) => x !== s) } });
    }
    for (const t of f.tags) {
        out.push({ key: `tag-${t}`, label: `#${t}`, without: { ...f, tags: f.tags.filter((x) => x !== t) } });
    }
    if (f.year !== null) out.push({ key: "year", label: `${f.year} 年首播`, without: { ...f, year: null } });
    if (f.minScore !== null) out.push({ key: "score", label: `评分 ≥ ${f.minScore}`, without: { ...f, minScore: null } });
    return out;
}

/** 清空所有条件, 保留排序与聚合方式 (都不是"条件") */
export function clearConditions (f: AnimeFilters): AnimeFilters {
    return { ...DEFAULT_FILTERS, sort: f.sort, fold: f.fold };
}

type Dimension = "q" | "status" | "tags" | "year" | "score";

function airYear (r: ANiMeRecord): number | null {
    const y = Number((r.anime_data.date ?? "").slice(0, 4));
    return Number.isInteger(y) && y > 1900 ? y : null;
}

function matchesQuery (r: ANiMeRecord, q: string): boolean {
    const needle = q.toLowerCase();
    const { name, name_cn } = r.anime_data;
    return [name_cn, name, r.user_status.comment]
        .some((s) => (s ?? "").toLowerCase().includes(needle));
}

function matchesTags (r: ANiMeRecord, tags: readonly string[], match: TagMatch): boolean {
    if (tags.length === 0) return true;
    const own = getRecordTags(r);
    return match === "all" ? tags.every((t) => own.includes(t)) : tags.some((t) => own.includes(t));
}

/** 记录是否满足筛选; skip 指定的维度不参与 (分面计数用) */
function matches (r: ANiMeRecord, f: AnimeFilters, skip?: Dimension): boolean {
    if (skip !== "q" && f.q && !matchesQuery(r, f.q)) return false;
    if (skip !== "status" && f.status.length && !f.status.includes(r.user_status.watch_status)) return false;
    if (skip !== "tags" && !matchesTags(r, f.tags, f.match)) return false;
    if (skip !== "year" && f.year !== null && airYear(r) !== f.year) return false;
    if (skip !== "score" && f.minScore !== null && !(r.anime_data.score >= f.minScore)) return false;
    return true;
}

function compareBy (sort: SortKey): (a: ANiMeRecord, b: ANiMeRecord) => number {
    switch (sort) {
        case "air":
            // 没有首播日期的排最后
            return (a, b) => (b.anime_data.date || "0000").localeCompare(a.anime_data.date || "0000");
        case "score":
            return (a, b) => (b.anime_data.score || 0) - (a.anime_data.score || 0);
        case "name":
            return (a, b) => (a.anime_data.name_cn || a.anime_data.name).localeCompare(b.anime_data.name_cn || b.anime_data.name, "zh-Hans-CN");
        case "update":
        default:
            return (a, b) => (b.user_status.last_update ?? "").localeCompare(a.user_status.last_update ?? "");
    }
}

export function applyFilters (records: readonly ANiMeRecord[], f: AnimeFilters): ANiMeRecord[] {
    return records.filter((r) => matches(r, f)).sort(compareBy(f.sort));
}

export interface FacetCounts {
    status: Map<WatchStatus, number>;
    /** 每个标签: 在其他条件不变时勾上它会剩几部 (已勾上的: 当前结果里有它的部数) */
    tags: TagCount[];
    years: { year: number; count: number }[];
}

/**
 * 分面计数: 每个选项旁的数字 = 其他维度条件不变时, 选上它会剩几部. 用户点之前就知道会不会点空.
 */
export function facetCounts (records: readonly ANiMeRecord[], f: AnimeFilters): FacetCounts {
    const status = new Map<WatchStatus, number>();
    const byOthersForStatus = records.filter((r) => matches(r, f, "status"));
    for (const r of byOthersForStatus) {
        status.set(r.user_status.watch_status, (status.get(r.user_status.watch_status) ?? 0) + 1);
    }

    // 标签: "全部命中"时以当前已选标签为前提再加一个; "任一命中"时只看其他维度
    const base = records.filter((r) => matches(r, f, "tags"));
    const pool = f.match === "all" ? base.filter((r) => matchesTags(r, f.tags, "all")) : base;
    const tags = countTags(pool);
    // 表外随手标签若因当前条件计数为 0 而消失, 已选中的仍要列出, 以便取消
    for (const t of f.tags) {
        if (!tags.some((c) => c.tag === t)) tags.push({ tag: t, count: 0, group: OTHER_TAG_GROUP });
    }

    const yearCount = new Map<number, number>();
    for (const r of records) {
        if (!matches(r, f, "year")) continue;
        const y = airYear(r);
        if (y !== null) yearCount.set(y, (yearCount.get(y) ?? 0) + 1);
    }
    const years = [...yearCount.entries()].sort((a, b) => b[0] - a[0]).map(([year, count]) => ({ year, count }));

    return { status, tags, years };
}

/**
 * 番剧季度. 日本动画按开播月份归季: 1 月冬、4 月春、7 月夏、10 月秋, 各管三个月.
 * 旧实现把 1-3 月叫"春"、4-6 月"夏"、7-9 月"秋"、10-12 月"冬", 季名整体后移了一季; 且用 new Date() 解析,
 * "YYYY-MM-DD" 按 UTC 解析后在西半球时区会退回前一天, 月初的番可能被分到上一季. 这里直接切字符串.
 */
export type AnimeSeason = "冬" | "春" | "夏" | "秋";
const SEASONS: readonly AnimeSeason[] = ["冬", "春", "夏", "秋"];

export function animeSeasonOf (date: string | null | undefined): { year: number; season: AnimeSeason; index: number } | null {
    const y = Number((date ?? "").slice(0, 4));
    const m = Number((date ?? "").slice(5, 7));
    if (!Number.isInteger(y) || y < 1900 || !(m >= 1 && m <= 12)) return null;
    const index = Math.floor((m - 1) / 3);
    return { year: y, season: SEASONS[index], index };
}

export interface RecordGroup {
    key: string;
    label: string;
    records: ANiMeRecord[];
}

/**
 * 按排序方式分组 (输入须已排好序): 首播日期 -> 番剧季度 ("2026 秋 · 10 月番"); 最近更新 -> 年月;
 * 评分 / 名称 -> 不分组. 保持输入顺序, 组按首次出现排列.
 */
export function groupRecords (sorted: readonly ANiMeRecord[], sort: SortKey): RecordGroup[] {
    if (sort === "score" || sort === "name") {
        return sorted.length ? [{ key: "all", label: "", records: [...sorted] }] : [];
    }
    const groups: RecordGroup[] = [];
    const byKey = new Map<string, RecordGroup>();
    for (const r of sorted) {
        let key: string;
        let label: string;
        if (sort === "air") {
            const s = animeSeasonOf(r.anime_data.date);
            key = s ? `${s.year}-${s.index}` : "unknown";
            label = s ? `${s.year} ${s.season} · ${s.index * 3 + 1} 月番` : "未知日期";
        } else {
            const d = r.user_status.last_update ?? "";
            key = d.slice(0, 7) || "unknown";
            label = key === "unknown" ? "未知日期" : `${d.slice(0, 4)} 年 ${Number(d.slice(5, 7))} 月更新`;
        }
        let g = byKey.get(key);
        if (!g) {
            g = { key, label, records: [] };
            byKey.set(key, g);
            groups.push(g);
        }
        g.records.push(r);
    }
    return groups;
}

/** 正片集数: 优先 eps, 没有就数 episodes 里的正片 */
export function normalEpisodeCount (r: ANiMeRecord): number {
    if (r.anime_data.eps > 0) return r.anime_data.eps;
    return r.anime_data.episodes.filter((e) => e.type === EpisodeType.NORMAL).length;
}
