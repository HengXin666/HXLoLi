import { ANIME_SUSUME_DEFS, ANIME_TAG_ALIASES, ANIME_TAG_DEFS } from "@site/data/animeTags";
import { ANIME_CDN_TAG } from "@site/data/animeCdnVersion";
import { GITHUB_CONFIG } from "@site/src/config/github";
import { reqAnimeByCDN } from "@site/src/utils/cdn/linkJsDelivr";

/**
 * 人工标注 (唯一的 tag 来源): HXLoLi-ANiMe/data/labels/<bgm_subject_id>.json, 一部番一个文件, 未标注 = 没有文件.
 * 形状见 hx-anime-tag-refactor-design.md 第 4 节:
 *   { "id": 517057, "tags": ["推理", "致郁"], "rate": 9, "review": "一句话观后感", "mode": "new|review",
 *     "by": "pr#123", "labeledAt": "2026-11-04T20:00+08:00", "bgmSynced": true }
 *
 * 读取顺序 (与 ANiMeRecord.json 同一个 ANIME_CDN_TAG, 数据与标注同版本):
 *   1. 构建产物 data/labels/index.json (标注流合入时汇总, 一次请求拿全; 形状接受 数组 / { items: {id: label} } / { labels: [...] }).
 *   2. 没有汇总文件时: jsDelivr data API 列出该 tag 下 data/labels/*.json, 再逐个拉 (并发 6). 目录不存在 = 一个都没标, 返回空表.
 * 两处都失败才算加载失败; 页面据此显示"标注没拉到", 不会把"失败"误当成"没有标注".
 *
 * 渲染只认词表内的标签 (data/animeTags.ts, 镜像 labelVocab.json): 文件里写了词表外的词, 丢弃不显示.
 * 不再读: HXLoLi-ANiMe/data/ANiMeTags.json 的分配表, data/animeTags.ts 的手工分配, data/animeOverlay.json 的 tags, user_status.tags.
 */
export interface AnimeLabel {
    id: number;
    /** 词表内的标签, 已按别名归并、去重 */
    tags: string[];
    /** 本人评分 1-10 (观后感模式) */
    rate?: number;
    /** 一句话观后感 */
    review?: string;
    mode?: "new" | "review";
    by?: string;
    labeledAt?: string;
    bgmSynced?: boolean;
}

export const LABELS_DIR = "/data/labels";
export const LABEL_INDEX_PATH = `${LABELS_DIR}/index.json`;
const LABEL_FILE_RE = /^\/data\/labels\/(\d+)\.json$/;

const KNOWN_TAGS = new Set<string>([...ANIME_SUSUME_DEFS.map((d) => d.display), ...ANIME_TAG_DEFS.map((d) => d.tag)]);

/** 只保留形状正确的字段; 手改坏了的 JSON 不至于让整页崩掉. 没有合法 id 返回 null */
export function sanitizeLabel (raw: unknown, fallbackId?: number): AnimeLabel | null {
    if (!raw || typeof raw !== "object" || Array.isArray(raw)) return null;
    const r = raw as Record<string, unknown>;
    const id = typeof r.id === "number" && Number.isSafeInteger(r.id) && r.id > 0 ? r.id : fallbackId;
    if (!id) return null;
    const tags = Array.isArray(r.tags)
        ? [...new Set(r.tags
            .filter((t): t is string => typeof t === "string")
            .map((t) => t.trim())
            .map((t) => ANIME_TAG_ALIASES[t] ?? t)
            .filter((t) => KNOWN_TAGS.has(t)))]
        : [];
    const out: AnimeLabel = { id, tags };
    if (typeof r.rate === "number" && r.rate >= 1 && r.rate <= 10) out.rate = Math.round(r.rate);
    if (typeof r.review === "string" && r.review.trim()) out.review = r.review.trim().slice(0, 500);
    if (r.mode === "new" || r.mode === "review") out.mode = r.mode;
    if (typeof r.by === "string") out.by = r.by;
    if (typeof r.labeledAt === "string") out.labeledAt = r.labeledAt;
    if (typeof r.bgmSynced === "boolean") out.bgmSynced = r.bgmSynced;
    return out;
}

/** 汇总文件 -> 标注表. 接受 数组 / { items: {id: label} } / { labels: [...] } */
export function parseLabelIndex (file: unknown): Map<number, AnimeLabel> {
    const out = new Map<number, AnimeLabel>();
    const put = (raw: unknown, key?: string) => {
        const l = sanitizeLabel(raw, key && /^\d+$/.test(key) ? Number(key) : undefined);
        if (l) out.set(l.id, l);
    };
    if (Array.isArray(file)) file.forEach((x) => put(x));
    else if (file && typeof file === "object") {
        const f = file as { items?: unknown; labels?: unknown };
        if (Array.isArray(f.labels)) f.labels.forEach((x) => put(x));
        else if (f.items && typeof f.items === "object") for (const [k, v] of Object.entries(f.items)) put(v, k);
    }
    return out;
}

async function readJson (path: string): Promise<unknown> {
    const res = await reqAnimeByCDN(path);
    return JSON.parse(await res.blob.text());
}

async function listLabelFiles (): Promise<number[]> {
    const url = `https://data.jsdelivr.com/v1/packages/gh/${GITHUB_CONFIG.USER}/HXLoLi-ANiMe@${ANIME_CDN_TAG}?structure=flat`;
    const res = await fetch(url);
    if (!res.ok) throw new Error(`labels 目录列表 ${res.status}`);
    const json = (await res.json()) as { files?: { name?: string }[] };
    const ids: number[] = [];
    for (const f of json.files ?? []) {
        const m = LABEL_FILE_RE.exec(f.name ?? "");
        if (m) ids.push(Number(m[1]));
    }
    return ids;
}

/** 加载全部标注. 汇总文件优先, 其次逐个文件; 都失败才 reject */
export async function fetchAnimeLabels (): Promise<Map<number, AnimeLabel>> {
    try {
        return parseLabelIndex(await readJson(LABEL_INDEX_PATH));
    } catch {
        /* 还没有汇总产物, 走逐个文件 */
    }
    const ids = await listLabelFiles();
    const out = new Map<number, AnimeLabel>();
    let next = 0;
    const worker = async () => {
        while (next < ids.length) {
            const id = ids[next++];
            try {
                const l = sanitizeLabel(await readJson(`${LABELS_DIR}/${id}.json`), id);
                if (l) out.set(l.id, l);
            } catch {
                /* 单个文件坏了只少这一部, 不影响其他 */
            }
        }
    };
    await Promise.all(Array.from({ length: Math.min(6, ids.length) }, worker));
    return out;
}
