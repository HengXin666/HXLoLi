import { ANiMeRecord } from "@site/src/utils/anime/types";
import { AnimeFilters, DEFAULT_FILTERS, parseFilters, serializeFilters } from "@site/src/utils/anime/filters";
import { normalizeTag } from "@site/src/utils/anime/tags";
import { ANIME_SHARE_DEFAULT, ANIME_SHARE_PRESETS } from "@site/data/animeShareDefault";
import type { AnimeShareStart } from "@site/data/animeShareDefault";

/*
 * 分享番单的 URL 编解码 (纯函数, 不碰 DOM).
 * 链接形如 /anime/share?status=watched&tag=日常&pick=1kC8x...
 *   - 筛选部分与番剧库 /anime 的查询串完全同一套 (parseFilters / serializeFilters), 定义"这份番单是哪些番";
 *   - pick = 分享者勾了"我看过"的 Bangumi subject id 集合, 用 encodeIds 压缩;
 *   - p = 预设名 (data/animeShareDefault.ts), 短链接 /anime/share?p=mine 展开成预设的页内筛选, 见文件末尾 resolveShareStart.
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
    // p 不是番单筛选, parseFilters 本来就不认它, 这里无需剔除
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

/** 匹配路径里番剧区的起点 (/anime, /anime/, /anime/share ...), 它前面的部分就是站点 baseUrl */
const ANIME_SEGMENT = /\/anime(?:\/|$)/;

/**
 * 当前访问的站点根 (协议 + 域名 + baseUrl, 无尾斜杠), 如 "https://km.woa.qzz.io" 或 "https://hengxin666.github.io/HXLoLi".
 * 站点同时部署在多个地址 (Cloudflare / GitHub Pages ...), docusaurus.config.ts 的 url / baseUrl 是构建时写死的,
 * 所以浏览器里以 window.location 为准: origin + pathname 中 /anime 之前的部分; 路径里找不到 /anime 时 baseUrl 退回配置值.
 * SSR / 非浏览器环境没有 window, 整体退回配置的 siteUrl + baseUrl.
 */
export function siteRootOf (siteUrl: string, baseUrl: string): string {
    const configBase = baseUrl.replace(/\/$/, "");
    if (typeof window !== "undefined" && /^https?:$/.test(window.location.protocol)) {
        const path = window.location.pathname;
        const i = path.search(ANIME_SEGMENT);
        const base = i >= 0 ? path.slice(0, i).replace(/\/$/, "") : configBase;
        return `${window.location.origin}${base}`;
    }
    return `${siteUrl.replace(/\/$/, "")}${configBase}`;
}

/** 完整可复制的绝对地址, 域名与 baseUrl 取当前访问来源 (见 siteRootOf); siteUrl / baseUrl 只在 SSR 时兜底 */
export function shareUrl (siteUrl: string, baseUrl: string, filters: AnimeFilters, pick: Iterable<number> = []): string {
    return `${siteRootOf(siteUrl, baseUrl)}${sharePath(filters, pick)}`;
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

// ---------------------------------------------------------------- 内定筛选: 默认值与命名预设 (定义在 data/animeShareDefault.ts)

export const PRESET_PARAM = "p";
/** 预设名的合法形状; 链接 /anime/share?p=<名字> 因此最长约 32 字符 */
export const PRESET_NAME = /^[a-z0-9-]{1,16}$/;
/** 保留名: 指向 ANIME_SHARE_DEFAULT. 番单本身带筛选参数时 (从番剧库弹窗生成), 链接靠它告诉对方"用默认的页内筛选" */
export const DEFAULT_PRESET = "default";

export type ShareViewMode = "card" | "table";

/** 预设 / 默认值展开后的页内状态 */
export interface ShareStartState {
    view: AnimeFilters;
    range: YearRange;
    mark: MarkFilter;
    mode: ShareViewMode;
    /** 预设里预置的"TA 看过"; 链接自带 pick 时不用 */
    pick: number[];
}

export interface ResolvedShareStart {
    /** preset = ?p=<名字> 命中; default = 用了默认值; none = 老长链接, 不套任何内定状态 */
    source: "preset" | "default" | "none";
    /** source 为 preset 时的名字 */
    name: string | null;
    label: string | null;
    /** ?p= 给了但不存在 / 不合规时的原值, 页面据此提示 */
    missing: string | null;
    state: ShareStartState;
}

const NEUTRAL_START: ShareStartState = { view: { ...DEFAULT_FILTERS, sort: "air" }, range: "all", mark: "all", mode: "card", pick: [] };

export function expandShareStart (s: AnimeShareStart): ShareStartState {
    return {
        view: {
            ...DEFAULT_FILTERS,
            q: (s.q ?? "").trim(),
            status: [...new Set(s.status ?? [])],
            tags: [...new Set((s.tags ?? []).map(normalizeTag))],
            match: s.match === "any" ? "any" : "all",
            sort: s.sort ?? "air",
        },
        range: s.year ?? "all",
        mark: s.mark ?? "all",
        mode: s.view === "table" ? "table" : "card",
        pick: decodeIds(s.pick),
    };
}

export function presetOf (name: string): AnimeShareStart | null {
    if (name === DEFAULT_PRESET) return ANIME_SHARE_DEFAULT;
    if (!PRESET_NAME.test(name) || !Object.prototype.hasOwnProperty.call(ANIME_SHARE_PRESETS, name)) return null;
    return ANIME_SHARE_PRESETS[name] ?? null;
}

/** 查询串里有没有番单筛选参数 (与 parseFilters 同一套键; 解析后全为默认值的视作没有) */
export function hasListParams (search: string): boolean {
    return serializeFilters(parseFilters(search)) !== "";
}

/**
 * 决定页内筛选的初始状态:
 * 1. ?p=<名字> 命中预设 -> 展开预设 (p=default 即默认值);
 * 2. ?p= 未命中 -> 默认值, 并带回 missing 供页面提示;
 * 3. 没有 p: 弹窗 (embedded) 一律默认值; 独立页没有番单筛选参数时默认值, 有则是老长链接, 不套 (NEUTRAL: 全部, 首播日期排序).
 */
export function resolveShareStart (search: string, embedded: boolean): ResolvedShareStart {
    const raw = new URLSearchParams(search).get(PRESET_PARAM);
    const name = raw === null ? null : raw.trim().toLowerCase();
    if (name) {
        const preset = presetOf(name);
        if (preset) {
            const isDefault = name === DEFAULT_PRESET;
            return { source: isDefault ? "default" : "preset", name: isDefault ? null : name, label: preset.label, missing: null, state: expandShareStart(preset) };
        }
        return { source: "default", name: null, label: ANIME_SHARE_DEFAULT.label, missing: raw, state: expandShareStart(ANIME_SHARE_DEFAULT) };
    }
    if (embedded || !hasListParams(search)) {
        return { source: "default", name: null, label: ANIME_SHARE_DEFAULT.label, missing: null, state: expandShareStart(ANIME_SHARE_DEFAULT) };
    }
    return { source: "none", name: null, label: null, missing: null, state: NEUTRAL_START };
}

/** 页内状态是否仍与初始状态一致 (一致才能用 ?p= 短链接; 改过就是自定义) */
export function sameShareState (a: Omit<ShareStartState, "pick">, b: Omit<ShareStartState, "pick">): boolean {
    return a.range === b.range && a.mark === b.mark && a.mode === b.mode && serializeFilters(a.view) === serializeFilters(b.view);
}

export interface ShareLinkSpec {
    /** 番单定义 (URL 上的筛选) */
    filters: AnimeFilters;
    /** 勾选 */
    pick: Iterable<number>;
    /** 当前视图模式 */
    mode: ShareViewMode;
    /** 当前仍是预设 / 默认状态时的预设名 (默认值传 DEFAULT_PRESET), 自定义传 null */
    preset: string | null;
}

/**
 * 分享链接的查询串 (不含 "?"):
 * - 仍是预设: [番单筛选] + p=<名字> + [pick]; 番单无筛选且是默认值时省掉 p (对方打开本来就是默认值);
 * - 自定义: 老格式 [番单筛选] + [pick], 页内筛选不进链接.
 * 视图模式只在与对方打开时的初始模式不同时写 view=<模式>.
 */
export function buildShareSpecQuery (spec: ShareLinkSpec): string {
    const base = serializeFilters(spec.filters);
    const code = encodeIds(spec.pick);
    const preset = spec.preset === DEFAULT_PRESET && !base ? null : spec.preset;
    const parts = [base, preset ? `${PRESET_PARAM}=${preset}` : "", code ? `${PICK_PARAM}=${code}` : ""].filter(Boolean);
    const q = parts.join("&");
    const target = resolveShareStart(q ? `?${q}` : "", false).state.mode;
    if (spec.mode !== target) parts.push(`view=${spec.mode}`);
    return parts.join("&");
}

export function shareSpecPath (spec: ShareLinkSpec): string {
    const q = buildShareSpecQuery(spec);
    return `${SHARE_PATH}${q ? `?${q}` : ""}`;
}

export function shareSpecUrl (siteUrl: string, baseUrl: string, spec: ShareLinkSpec): string {
    return `${siteRootOf(siteUrl, baseUrl)}${shareSpecPath(spec)}`;
}
