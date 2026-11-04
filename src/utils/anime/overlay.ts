import overlayJson from "@site/data/animeOverlay.json";
import { ANiMeRecord, WatchStatus } from "@site/src/utils/anime/types";

/**
 * 本人标注 overlay: data/animeOverlay.json, 键是 BanGuMi subject id. 运行时盖在 CDN 拉来的 ANiMeRecord 上.
 *
 * 优先级 (逐字段, 只看 overlay 里出现了的字段):
 *   - watch_status / watched_eps / comment / rate: overlay > CDN user_status.
 *   - tags: **已废弃, 不再读取** (2026-11-04 tag 重构, hx-anime-tag-refactor-design.md 第 4 / 10 节).
 *           标签只来自人工标注 HXLoLi-ANiMe/data/labels/<id>.json (见 utils/anime/labels.ts 与 tags.ts);
 *           animeOverlay.json 里残留的 tags 字段原样保留在文件里但不参与渲染, 也不再覆盖 user_status.tags.
 *   - 删掉某部番的 overlay 条目 = 回到 CDN 数据.
 * 写入只在 dev 下经 plugins/anime-overlay-plugin.mjs 完成; 本模块只读, 生产构建也只读这份文件.
 * .agents/notes/implemented/feature/2026-11-01-anime-local-overlay.md
 */
export interface AnimeOverlayEntry {
    /** @deprecated 2026-11-04 起不再读取 (标签只认 data/labels 人工标注); 只为读旧文件不报错而保留 */
    tags?: string[];
    watch_status?: WatchStatus;
    watched_eps?: number;
    /** 本人评分 1-10, CDN 数据里没有这一项 */
    rate?: number;
    comment?: string;
    /** 写入时间 (ISO), 由 dev 写入通道填 */
    updated_at?: string;
}

export type AnimeOverlay = Record<string, AnimeOverlayEntry>;

const WATCH_STATUSES = new Set<number>([1, 2, 3, 4, 5]);

/** 只保留形状正确的字段; 手改坏了的 JSON 不至于让整页崩掉 */
export function sanitizeOverlayEntry (raw: unknown): AnimeOverlayEntry | null {
    if (!raw || typeof raw !== "object") return null;
    const r = raw as Record<string, unknown>;
    const out: AnimeOverlayEntry = {};
    if (Array.isArray(r.tags)) out.tags = r.tags.filter((t): t is string => typeof t === "string" && t.trim() !== "").map((t) => t.trim());
    if (typeof r.watch_status === "number" && WATCH_STATUSES.has(r.watch_status)) out.watch_status = r.watch_status as WatchStatus;
    if (typeof r.watched_eps === "number" && Number.isFinite(r.watched_eps) && r.watched_eps >= 0) out.watched_eps = Math.floor(r.watched_eps);
    if (typeof r.rate === "number" && r.rate >= 1 && r.rate <= 10) out.rate = Math.round(r.rate);
    if (typeof r.comment === "string") out.comment = r.comment;
    if (typeof r.updated_at === "string") out.updated_at = r.updated_at;
    return Object.keys(out).length ? out : null;
}

function sanitizeOverlay (raw: unknown): AnimeOverlay {
    const out: AnimeOverlay = {};
    if (!raw || typeof raw !== "object") return out;
    for (const [id, v] of Object.entries(raw as Record<string, unknown>)) {
        if (!/^\d+$/.test(id)) continue;
        const e = sanitizeOverlayEntry(v);
        if (e) out[id] = e;
    }
    return out;
}

/** 构建时打进包里的 overlay (dev 下改文件后由 HMR 更新) */
export const ANIME_OVERLAY: Readonly<AnimeOverlay> = Object.freeze(sanitizeOverlay(overlayJson));

/** overlay 盖到一条记录上; 没有条目时原样返回 (同一个对象, 缓存不失效) */
export function applyOverlayEntry (record: ANiMeRecord, entry: AnimeOverlayEntry | undefined | null): ANiMeRecord {
    if (!entry) return record;
    // tags 已废弃: 解构出来丢掉, 既不接管标签也不覆盖 user_status.tags (标签只读 data/labels)
    const { tags: _deprecatedTags, rate, updated_at: _u, ...status } = entry;
    if (Object.keys(status).length === 0 && rate === undefined) return record;
    return Object.freeze({
        anime_data: record.anime_data,
        user_status: Object.freeze({
            ...record.user_status,
            ...status,
            ...(rate !== undefined ? { rate } : {}),
        }),
    });
}

export function applyOverlay (records: readonly ANiMeRecord[], overlay: Readonly<AnimeOverlay> = ANIME_OVERLAY): ANiMeRecord[] {
    if (Object.keys(overlay).length === 0) return [...records];
    return records.map((r) => applyOverlayEntry(r, overlay[String(r.anime_data.id)]));
}
