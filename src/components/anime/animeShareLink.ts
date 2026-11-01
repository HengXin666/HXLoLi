import { ANiMeRecord } from "@site/src/utils/anime/types";
import { AnimeFilters, DEFAULT_FILTERS, parseFilters, serializeFilters } from "@site/src/utils/anime/filters";

/*
 * 分享番单的 URL 编解码 (纯函数, 不碰 DOM).
 * 链接形如 /anime/share?status=watched&tag=日常&pick=1kC8x...
 *   - 筛选部分与番剧库 /anime 的查询串完全同一套 (parseFilters / serializeFilters), 定义"这份番单是哪些番";
 *   - pick = 分享者勾了"我看过"的 Bangumi subject id 集合, 用 encodeIds 压缩.
 * 决策见 src/components/anime/AnimeShareBoard.tsx 顶部引用的 note.
 */

export const PICK_PARAM = "pick";
export const SHARE_PATH = "/anime/share";

/** 64 个 URL 无需转义的字符 (RFC 3986 unreserved): 每个字符 6 bit = 5 bit 数据 + 1 bit "后面还有" */
const ALPHABET = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ-_";
const INDEX = new Map([...ALPHABET].map((c, i) => [c, i]));
/** 编码版本前缀; 以后换编码时据此兼容旧链接 */
const CODEC_V1 = "1";
/** 解码上限, 防止恶意超长参数卡死页面 */
const MAX_IDS = 20000;

/**
 * id 集合 -> 紧凑字符串. 去重升序后取相邻差, 每个差值写成小端 5 bit 一组的变长整数 (base64url 字符).
 * 差 < 32 占 1 字符, < 1024 占 2, < 32768 占 3, < 1048576 占 4. 不需要分隔符, 也不会被 URL 转义.
 * 空集合返回空串.
 */
export function encodeIds (ids: Iterable<number>): string {
    const sorted = [...new Set(ids)].filter((n) => Number.isSafeInteger(n) && n > 0).sort((a, b) => a - b);
    if (sorted.length === 0) return "";
    let out = CODEC_V1;
    let prev = 0;
    for (const id of sorted) {
        let d = id - prev;
        prev = id;
        do {
            let g = d % 32;
            d = Math.floor(d / 32);
            if (d > 0) g += 32;
            out += ALPHABET[g];
        } while (d > 0);
    }
    return out;
}

/**
 * encodeIds 的逆. 认不出的输入返回空数组, 不抛错.
 * 另兼容手写的十进制逗号列表 (pick=12,345,6789), 方便调试与手工拼链接.
 */
export function decodeIds (raw: string | null | undefined): number[] {
    const s = (raw ?? "").trim();
    if (!s) return [];
    if (/^\d+(,\d+)*$/.test(s)) {
        return [...new Set(s.split(",").map(Number).filter((n) => Number.isSafeInteger(n) && n > 0))].slice(0, MAX_IDS);
    }
    if (s[0] !== CODEC_V1) return [];
    const out: number[] = [];
    let prev = 0;
    let value = 0;
    let mult = 1;
    for (let i = 1; i < s.length; i++) {
        const g = INDEX.get(s[i]);
        if (g === undefined) return [];
        value += (g % 32) * mult;
        if (g >= 32) {
            mult *= 32;
            if (mult > 2 ** 40) return [];
            continue;
        }
        prev += value;
        out.push(prev);
        if (out.length >= MAX_IDS) break;
        value = 0;
        mult = 1;
    }
    return out;
}

export interface ShareLinkState {
    /** 番单定义: 与番剧库同一套筛选 */
    filters: AnimeFilters;
    /** 链接里携带的"分享者看过"的 id */
    pick: number[];
}

export function parseShareQuery (search: string): ShareLinkState {
    const p = new URLSearchParams(search);
    return { filters: parseFilters(search), pick: decodeIds(p.get(PICK_PARAM)) };
}

/** 生成查询串 (不含 "?"). pick 为空时不写 */
export function buildShareQuery (filters: AnimeFilters, pick: Iterable<number> = []): string {
    const base = serializeFilters(filters);
    const code = encodeIds(pick);
    const parts = [base, code ? `${PICK_PARAM}=${code}` : ""].filter(Boolean);
    return parts.join("&");
}

/** 站内路径 (给 <Link>) */
export function sharePath (filters: AnimeFilters, pick: Iterable<number> = []): string {
    const q = buildShareQuery(filters, pick);
    return `${SHARE_PATH}${q ? `?${q}` : ""}`;
}

/** 完整可复制的绝对地址. siteUrl 如 "https://hxloli.example", baseUrl 如 "/" */
export function shareUrl (siteUrl: string, baseUrl: string, filters: AnimeFilters, pick: Iterable<number> = []): string {
    const root = `${siteUrl.replace(/\/$/, "")}${baseUrl.replace(/\/$/, "")}`;
    return `${root}${sharePath(filters, pick)}`;
}

/** FNV-1a 32 bit, 输出 base36. 只用来给 localStorage 键做短标识, 不做安全用途 */
function hash36 (text: string): string {
    let h = 0x811c9dc5;
    for (let i = 0; i < text.length; i++) {
        h ^= text.charCodeAt(i);
        h = Math.imul(h, 0x01000193) >>> 0;
    }
    return h.toString(36);
}

/**
 * 番单标识: 只由"哪些番"决定 (筛选条件去掉排序), 与分享者的 pick 无关.
 * 所以同一份番单无论谁转发、勾了多少, 打开的人看到的都是自己同一份本地勾选; 不同番单互不污染.
 */
export function listKeyOf (filters: AnimeFilters): string {
    const q = serializeFilters({ ...filters, sort: DEFAULT_FILTERS.sort });
    return q ? hash36(q) : "all";
}

export function storageKeyOf (baseUrl: string, filters: AnimeFilters): string {
    // baseUrl 在 Cloudflare 为 "/", GitHub Pages 为 "/HXLoLi" (无尾斜杠), 统一补成以 "/" 结尾
    return `${baseUrl.replace(/\/?$/, "/")}anime/share:v1:${listKeyOf(filters)}`;
}

/** 本地勾选读写 (存的也是 encodeIds 的串, 省空间). 隐私模式 / 配额满时静默失败 */
export function readPicks (key: string): Set<number> {
    try {
        return new Set(decodeIds(localStorage.getItem(key)));
    } catch {
        return new Set();
    }
}

export function writePicks (key: string, ids: ReadonlySet<number>): void {
    try {
        if (ids.size === 0) localStorage.removeItem(key);
        else localStorage.setItem(key, encodeIds(ids));
    } catch {
        // 忽略写入失败
    }
}

// ---------------------------------------------------------------- 分享页专属筛选 (不进 filters.ts)

export type YearRange = "5" | "10" | "15" | "all";

export const YEAR_RANGE_OPTIONS: readonly { value: YearRange; label: string }[] = [
    { value: "5", label: "近 5 年" },
    { value: "10", label: "近 10 年" },
    { value: "15", label: "近 15 年" },
    { value: "all", label: "全部" },
];

export function airYearOf (r: ANiMeRecord): number | null {
    const y = Number((r.anime_data.date ?? "").slice(0, 4));
    return Number.isInteger(y) && y > 1900 ? y : null;
}

/** 首播年份是否落在"近 N 年" (含今年). 没有日期的番只在"全部"里出现 */
export function inYearRange (r: ANiMeRecord, range: YearRange, thisYear: number): boolean {
    if (range === "all") return true;
    const y = airYearOf(r);
    return y !== null && y > thisYear - Number(range);
}

export type MarkFilter = "all" | "mine" | "notMine" | "theirs" | "both";

export const MARK_OPTIONS: readonly { value: MarkFilter; label: string; needsPick?: boolean }[] = [
    { value: "all", label: "全部" },
    { value: "mine", label: "我看过" },
    { value: "notMine", label: "我没看过" },
    { value: "theirs", label: "TA 看过", needsPick: true },
    { value: "both", label: "我们都看过", needsPick: true },
];

export function matchesMark (id: number, mark: MarkFilter, mine: ReadonlySet<number>, theirs: ReadonlySet<number>): boolean {
    switch (mark) {
        case "mine": return mine.has(id);
        case "notMine": return !mine.has(id);
        case "theirs": return theirs.has(id);
        case "both": return mine.has(id) && theirs.has(id);
        case "all":
        default: return true;
    }
}
