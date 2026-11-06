import React from "react";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import useIsBrowser from "@docusaurus/useIsBrowser";
import { useHistory } from "@docusaurus/router";
import { useBaseUrlUtils } from "@docusaurus/useBaseUrl";
import { PageMetadata } from "@docusaurus/theme-common";
import { FaCodeBranch, FaExternalLinkAlt, FaPlay, FaStar } from "react-icons/fa";
import { useAnimeRecordsState } from "@site/src/utils/anime/animeStore";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";
import { ANiMeRecord, EpisodeType, Relation, WatchStatus } from "@site/src/utils/anime/types";
import { ANIME_TAG_QUERY, susumeDefOfTag, susumePath } from "@site/src/utils/anime/tags";
import { animeSeasonOf, normalEpisodeCount, statusLabel } from "@site/src/utils/anime/filters";
import {
    AnimeSeason,
    AnimeSeries,
    missingLabel,
    parseSeasonMarker,
    seriesCoverId,
    seriesScoreRange,
    seriesTags,
    seriesYearRange,
    stripSeasonSuffix,
    useAnimeSeries,
} from "@site/src/utils/anime/series";
import useQuery from "@site/src/utils/url/useQuery";
import BlogWithCats from "@site/src/components/BlogWithCats";
import detail from "./AnimeDetailPage.module.css";
import styles from "./SeriesPage.module.css";

/*
 * 番剧系列页 /anime/series?id=<系列内任一季的 bgm id>   (2026-11-06 重构)
 *
 * 旧版问题:
 *  1. 系列 = bgm 关系图的整个连通分量, 禁书目录 / 超电磁炮 / 一方通行 / 暗部少女共栖 被压成一条 "正篇", 季号互相串;
 *  2. 剧场版一律丢进 "特别篇", 但 番剧A -> 剧场版 -> 番剧B 这种剧场版是正片剧情, 观看顺序被打断;
 *  3. 首播时间线只有点 + YYYY-MM, 看不出是哪一部.
 *
 * 新版 (只动本页, 不改 utils/anime/series.ts 的系列聚合, 番剧库卡片不受影响):
 *  - 作品线: 在系列内部只沿 前传 / 续集 边再做一次并查集 -> 每条线是一部 "番剧" (禁书 / 超电磁炮 / 一方通行 ...);
 *    没有 前传 / 续集 的剧场版 / OVA 按 主线故事 / 番外篇 / 总集篇 / 剧场版 关系挂回所属线; 完全没有库内关系的条目按名字前缀挂靠.
 *    衍生 / 相同世界观 / 不同世界观 不连线 (这正是不同番剧的边界).
 *  - 观看顺序: 每条线内 TV 与剧场版混排按首播排序, 剧场版算正片; OVA / 番外 / 总集篇 作为可选条目插在原位, 可一键隐藏.
 *    季号按线内重编 (标题写明的季号优先, Part.2 / 后半 并入上一季).
 *  - 放送时间线: 横向滚动的封面卡轨道, 卡片带全名 / 原名 / 类型 / 日期, 间距随放送间隔伸缩, 跨年插年份刻度,
 *    长间隔标 "隔 N 年 M 个月", 未定档单列; 打开时自动滚到 命中 / 继续看 的那一部.
 * 加载 / 错误 / 404 三态与详情页一致. 讨论区不放这里 (挂在每季详情页).
 */

/** 1-3 集且标题没写季号的视为短篇 (剧场版 / OVA / SP) */
const SHORT_MAX_EPS = 3;
/** 作品线之间只认这两种边 */
const LINE_RELATIONS: ReadonlySet<string> = new Set(["前传", "续集"]);
/** 剧场版 / OVA 挂回作品线时认的边 (衍生 / 相同世界观 不算, 那是另一部番) */
const ATTACH_RELATIONS: ReadonlySet<string> = new Set(["前传", "续集", "主线故事", "番外篇", "总集篇", "剧场版", "外传"]);
/** 作品线配色 (色相), 依次轮换 */
const LINE_HUES = [318, 205, 150, 32, 265, 0, 180, 90];

type EntryKind = "tv" | "movie" | "ova" | "recap" | "extra";

const KIND_LABEL: Record<EntryKind, string> = { tv: "TV", movie: "剧场版", ova: "OVA", recap: "总集篇", extra: "番外" };

interface Entry {
    record: ANiMeRecord;
    id: number;
    /** YYYY-MM-DD, 未定档为 "" */
    date: string;
    kind: EntryKind;
    /** 正片 (TV + 剧场版): 计入进度与观看顺序编号; 其余为可选 */
    canon: boolean;
    /** 线内完整标签: 第 2 季 / 第 2 季 · 第 2 部分 / 剧场版 / OVA ... */
    label: string;
    /** 短标签: S2 / S2-2 / 剧场版 / OVA */
    short: string;
    /** 已播出 (有日期且不晚于今天) */
    aired: boolean;
}

interface MissingEntry {
    rel: Relation;
    before: boolean;
}

interface WorkLine {
    index: number;
    title: string;
    originalTitle: string;
    entries: Entry[];
    missing: MissingEntry[];
    /** 衍生作品线 (有 主线故事 指向别的线, 或被别的线标为 衍生) */
    spinoff: boolean;
    hue: number;
}

// ---------------------------------------------------------------- 数据整理

const dateOf = (r: ANiMeRecord): string => (typeof r.anime_data.date === "string" ? r.anime_data.date : "") || "";

const byDate = (a: Entry, b: Entry): number =>
    (a.date || "9999").localeCompare(b.date || "9999") || a.id - b.id;

function todayIso (): string {
    return new Date().toISOString().slice(0, 10);
}

function kindOf (season: AnimeSeason): EntryKind {
    const r = season.record;
    const mk = parseSeasonMarker(r.anime_data.name_cn, r.anime_data.name);
    const eps = normalEpisodeCount(r);
    if (season.special === "剧场版" || mk.movie) return "movie";
    if (season.special === "总集篇") return "recap";
    if (season.kind === "main") return "tv";
    if (season.special === "OVA" || /\b(?:OVA|OAD)\b/i.test(`${r.anime_data.name} ${r.anime_data.name_cn}`.normalize("NFKC"))) return "ova";
    // bgm 不给条目类型: 单集且正片时长 >= 1 小时 (或时长缺失) 的短篇按剧场版算 (禁书 恩底弥翁的奇迹 被 bgm 挂成 番外篇)
    if (eps === 1) {
        const dur = (r.anime_data.episodes || []).find((x) => x.type === EpisodeType.NORMAL)?.duration_seconds ?? 0;
        if (dur === 0 || dur >= 3600) return "movie";
    }
    if (eps > SHORT_MAX_EPS) return "tv";
    if (eps === 0 && (mk.no !== null || mk.final)) return "tv"; // 未放送的 第四季
    return "extra";
}

function find (parent: Map<number, number>, x: number): number {
    let r = x;
    while (parent.get(r) !== r) r = parent.get(r)!;
    let c = x;
    while (parent.get(c) !== r) { const n = parent.get(c)!; parent.set(c, r); c = n; }
    return r;
}

function union (parent: Map<number, number>, a: number, b: number): void {
    const ra = find(parent, a);
    const rb = find(parent, b);
    if (ra !== rb) parent.set(rb, ra);
}

/** 线内给 TV 编季号, 给每条生成标签 */
function labelEntries (entries: Entry[]): void {
    const tvs = entries.filter((e) => e.kind === "tv");
    let prevNo = 0;
    let prev: Entry | null = null;
    const parts = new Map<Entry, { no: number; part: number | null; final: boolean }>();
    for (const e of tvs) {
        const mk = parseSeasonMarker(e.record.anime_data.name_cn, e.record.anime_data.name);
        let no: number;
        if (mk.no !== null) no = mk.no;
        else if (mk.part !== null && mk.part > 1 && prev) no = prevNo;
        else no = prevNo + 1;
        let part: number | null = mk.part;
        if (prev && no === prevNo) {
            const p = parts.get(prev)!;
            if (p.part === null) p.part = 1;
            if (part === null || part <= p.part) part = p.part + 1;
        } else if (part === 1) {
            part = null;
        }
        parts.set(e, { no, part, final: mk.final });
        prevNo = no;
        prev = e;
    }
    const singleTv = tvs.length === 1;
    for (const e of entries) {
        if (e.kind !== "tv") {
            e.label = e.kind === "extra" ? "番外" : KIND_LABEL[e.kind];
            e.short = e.kind === "extra" ? "SP" : KIND_LABEL[e.kind];
            continue;
        }
        const p = parts.get(e)!;
        if (singleTv && !p.part && !p.final && p.no === 1) {
            e.label = "TV";
            e.short = "TV";
            continue;
        }
        const head = p.final ? "最终季" : `第 ${p.no} 季`;
        e.label = p.part ? `${head} · 第 ${p.part} 部分` : head;
        const sh = p.final ? "FS" : `S${p.no}`;
        e.short = p.part ? `${sh}-${p.part}` : sh;
    }
}

/** 系列 -> 作品线 */
function buildLines (series: AnimeSeries): WorkLine[] {
    const today = todayIso();
    const entries: Entry[] = series.seasons.map((season) => {
        const kind = kindOf(season);
        const date = dateOf(season.record);
        return {
            record: season.record,
            id: season.record.anime_data.id,
            date,
            kind,
            canon: kind === "tv" || kind === "movie",
            label: "",
            short: "",
            aired: !!date && date <= today,
        };
    });
    const byId = new Map(entries.map((e) => [e.id, e]));
    const parent = new Map(entries.map((e) => [e.id, e.id]));
    const related = new Set<number>(); // 有任何库内关系的条目
    const attachTo = new Map<number, Set<number>>(); // 挂靠候选 (双向)
    const spinoffIds = new Set<number>(); // 自己是衍生 (有 主线故事 指向系列内条目, 或被标为 衍生)
    const addAttach = (a: number, b: number): void => {
        if (!attachTo.has(a)) attachTo.set(a, new Set());
        if (!attachTo.has(b)) attachTo.set(b, new Set());
        attachTo.get(a)!.add(b);
        attachTo.get(b)!.add(a);
    };

    for (const e of entries) {
        for (const rel of e.record.anime_data.relations || []) {
            if (!byId.has(rel.id) || rel.id === e.id) continue;
            related.add(e.id);
            related.add(rel.id);
            if (LINE_RELATIONS.has(rel.relation)) union(parent, e.id, rel.id);
            if (ATTACH_RELATIONS.has(rel.relation)) addAttach(e.id, rel.id);
            if (rel.relation === "主线故事") spinoffIds.add(e.id);
            if (rel.relation === "衍生") spinoffIds.add(rel.id);
        }
    }

    // 只有短篇 (无 TV) 的分量: 挂回关系邻居所在的线
    const groups = (): Map<number, Entry[]> => {
        const m = new Map<number, Entry[]>();
        for (const e of entries) {
            const r = find(parent, e.id);
            if (!m.has(r)) m.set(r, []);
            m.get(r)!.push(e);
        }
        return m;
    };
    for (const [root, members] of groups()) {
        if (members.some((e) => e.kind === "tv")) continue;
        if (!members.some((e) => related.has(e.id))) continue;
        const target = members
            .flatMap((e) => [...(attachTo.get(e.id) ?? [])])
            .find((id) => find(parent, id) !== root && (groups().get(find(parent, id)) ?? []).some((x) => x.kind === "tv"));
        if (target !== undefined) union(parent, target, root);
    }

    // 完全没有库内关系的条目: 按名字前缀挂到最像的线, 都不像就合成一条
    const lineKey = (e: Entry): string => stripSeasonSuffix(e.record.anime_data.name_cn || e.record.anime_data.name).normalize("NFKC").replace(/^(剧场版|劇場版)\s*/, "").toLowerCase();
    const lonely = entries.filter((e) => !related.has(e.id));
    const anchors = entries.filter((e) => related.has(e.id) && e.kind === "tv");
    let pool: number | null = null;
    for (const e of lonely) {
        const k = lineKey(e);
        const hit = anchors
            .map((a) => ({ a, k: lineKey(a) }))
            .filter(({ k: ak }) => ak && (k.startsWith(ak) || ak.startsWith(k)))
            .sort((x, y) => y.k.length - x.k.length)[0];
        if (hit) union(parent, hit.a.id, e.id);
        else if (pool === null) pool = e.id;
        else union(parent, pool, e.id);
    }

    // 未收录的 前传 / 续集: 挂到引用它的条目所在线
    const missingIds = new Set(series.missing.map((m) => m.id));
    const missingByRoot = new Map<number, MissingEntry[]>();
    const seenMissing = new Set<number>();
    for (const e of entries) {
        for (const rel of e.record.anime_data.relations || []) {
            if (!missingIds.has(rel.id) || seenMissing.has(rel.id)) continue;
            seenMissing.add(rel.id);
            const root = find(parent, e.id);
            if (!missingByRoot.has(root)) missingByRoot.set(root, []);
            missingByRoot.get(root)!.push({ rel, before: rel.relation === "前传" });
        }
    }

    const lines: WorkLine[] = [];
    for (const [root, members] of groups()) {
        members.sort(byDate);
        labelEntries(members);
        const head = members.find((e) => e.kind === "tv") ?? members[0];
        const a = head.record.anime_data;
        lines.push({
            index: 0,
            title: stripSeasonSuffix(a.name_cn || a.name),
            originalTitle: stripSeasonSuffix(a.name),
            entries: members,
            missing: missingByRoot.get(root) ?? [],
            spinoff: members.some((e) => spinoffIds.has(e.id)),
            hue: 0,
        });
    }
    const firstDate = (l: WorkLine): string => (l.entries.find((e) => e.canon && e.date) ?? l.entries.find((e) => e.date))?.date || "9999";
    // 系列 key 所在的线 (首播最早的正篇) 排第一, 其余按首播
    lines.sort((x, y) => {
        const kx = x.entries.some((e) => e.id === series.key) ? 0 : 1;
        const ky = y.entries.some((e) => e.id === series.key) ? 0 : 1;
        return kx - ky || firstDate(x).localeCompare(firstDate(y));
    });
    lines.forEach((l, i) => {
        l.index = i;
        l.hue = LINE_HUES[i % LINE_HUES.length];
        if (i === 0) l.spinoff = false;
    });
    return lines;
}

function entryProgress (e: Entry): { ratio: number; text: string } {
    const r = e.record;
    const total = normalEpisodeCount(r);
    const watched = Math.max(0, r.user_status.watched_eps || 0);
    const st = r.user_status.watch_status;
    if (!e.aired) return { ratio: 0, text: e.date ? `${e.date} 开播` : "未定档" };
    if (e.kind === "movie" && total <= 1) return { ratio: st === WatchStatus.WATCHED ? 1 : 0, text: st === WatchStatus.WATCHED ? "已看" : "未看" };
    if (st === WatchStatus.WATCHED) return { ratio: 1, text: total ? `全 ${total} 话` : "看完" };
    if (st === WatchStatus.WANT_TO_WATCH) return { ratio: 0, text: total ? `共 ${total} 话` : "未开始" };
    if (!total) return { ratio: 0, text: watched ? `看到第 ${watched} 话` : "未开始" };
    return { ratio: Math.min(1, watched / total), text: `${watched} / ${total} 话` };
}

function entryEpisodes (e: Entry): { watched: number; total: number } {
    const r = e.record;
    const total = normalEpisodeCount(r);
    if (r.user_status.watch_status === WatchStatus.WATCHED) return { watched: total, total };
    return { watched: Math.min(total, Math.max(0, r.user_status.watched_eps || 0)), total };
}

function whenText (date: string): string {
    const s = animeSeasonOf(date);
    return s ? `${date.slice(0, 7)} · ${s.year} ${s.season}` : "未定档";
}

function monthsBetween (a: string, b: string): number {
    const ya = Number(a.slice(0, 4));
    const ma = Number(a.slice(5, 7));
    const yb = Number(b.slice(0, 4));
    const mb = Number(b.slice(5, 7));
    return (yb - ya) * 12 + (mb - ma);
}

function gapText (months: number): string {
    const y = Math.floor(months / 12);
    const m = months % 12;
    return `隔 ${y ? `${y} 年` : ""}${m ? ` ${m} 个月` : ""}`.replace(/\s+/g, " ").trim();
}

/** 系列聚合: 正片 (已播) 进度与状态 */
function aggregate (lines: WorkLine[]): {
    watched: number; total: number; ratio: number; status: WatchStatus; label: string; canonCount: number; optionalCount: number;
} {
    const canon = lines.flatMap((l) => l.entries).filter((e) => e.canon);
    const aired = canon.filter((e) => e.aired);
    let watched = 0;
    let total = 0;
    for (const e of aired) { const p = entryEpisodes(e); watched += p.watched; total += p.total; }
    const sts = aired.map((e) => e.record.user_status.watch_status);
    const done = sts.filter((s) => s === WatchStatus.WATCHED).length;
    let status: WatchStatus;
    let label: string;
    if (sts.includes(WatchStatus.WATCHING)) { status = WatchStatus.WATCHING; label = "在看"; }
    else if (sts.length && done === sts.length) { status = WatchStatus.WATCHED; label = "全部看过"; }
    else if (sts.length && sts.every((s) => s === WatchStatus.WANT_TO_WATCH)) { status = WatchStatus.WANT_TO_WATCH; label = "想看"; }
    else if (done > 0) { status = WatchStatus.WATCHING; label = `看过 ${done} / ${sts.length} 部`; }
    else { status = sts[sts.length - 1] ?? WatchStatus.WANT_TO_WATCH; label = statusLabel(status); }
    return {
        watched, total, ratio: total > 0 ? Math.min(1, watched / total) : 0, status, label,
        canonCount: canon.length,
        optionalCount: lines.reduce((n, l) => n + l.entries.filter((e) => !e.canon).length, 0),
    };
}

/** 继续看: 任何线里在看的正片; 否则第一条线里第一部已播没看完的正片 */
function nextUpEntry (lines: WorkLine[]): Entry | null {
    const all = lines.flatMap((l) => l.entries).filter((e) => e.canon);
    return all.find((e) => e.record.user_status.watch_status === WatchStatus.WATCHING)
        ?? (lines[0]?.entries ?? []).find((e) => e.canon && e.aired && e.record.user_status.watch_status !== WatchStatus.WATCHED)
        ?? null;
}

const coverOf = (id: number): string => toAnimeCdnUrl(`/data/anime/${id}.jpg`);
const lineStyle = (hue: number): React.CSSProperties => ({ ["--line-hue" as string]: hue } as React.CSSProperties);

// ---------------------------------------------------------------- 组件

/*
 * 系列头部简介 (2026-11-06 第三版)
 * 旧版是单独一节「简介」: 渐变光晕 + 大引号 + 「あらすじ」字距小标 + 右侧玻璃「系列速览」, 和头部之间隔一大段;
 * 头部标题下面反而空着一大块 (heroGrid 的 auto / 1fr 行被封面撑开).
 * 现在简介直接排在头部标题和元信息下面 (同 Letterboxd / AniList 条目页: 海报左, 标题 + 简介 + 数据右),
 * 默认 4 行 line-clamp, 文字放不下才出「展开」文字按钮; 速览数据改成头部里一行发丝线分隔的数字.
 */
const SYNOPSIS_LINES = 4;

function Synopsis ({ text }: { text: string }): React.ReactElement | null {
    const body = (text || "").trim();
    const ref = React.useRef<HTMLParagraphElement>(null);
    const [open, setOpen] = React.useState(false);
    const [overflow, setOverflow] = React.useState(false);

    React.useEffect(() => {
        const el = ref.current;
        if (!el) return undefined;
        const measure = (): void => {
            if (open) return;
            setOverflow(el.scrollHeight > el.clientHeight + 2);
        };
        measure();
        const ro = typeof ResizeObserver !== "undefined" ? new ResizeObserver(measure) : null;
        ro?.observe(el);
        return () => ro?.disconnect();
    }, [body, open]);

    if (!body) return null;
    return (
        <div className={styles.synopsis}>
            <p
                ref={ref}
                className={`${styles.synopsisText} ${open ? "" : styles.synopsisClamp}`}
                style={{ ["--synopsis-lines" as string]: SYNOPSIS_LINES } as React.CSSProperties}
            >
                {body}
            </p>
            {(overflow || open) && (
                <button type="button" className={styles.synopsisMore} onClick={() => setOpen((v) => !v)} aria-expanded={open}>
                    {open ? "收起" : "展开全文"}
                </button>
            )}
        </div>
    );
}

function KindChip ({ e }: { e: Entry }): React.ReactElement {
    const cls = e.kind === "movie" ? styles.chipMovie : e.canon ? styles.chipTv : styles.chipExtra;
    return <span className={`${styles.chip} ${cls}`}>{e.label}</span>;
}

function StepCard ({ e, step, hit, nextUp }: { e: Entry; step: number | null; hit: boolean; nextUp: boolean }): React.ReactElement {
    const a = e.record.anime_data;
    const { ratio, text } = entryProgress(e);
    const comment = (e.record.user_status.comment ?? "").trim();
    const eps = normalEpisodeCount(e.record);
    const st = e.record.user_status.watch_status;
    const done = st === WatchStatus.WATCHED;
    return (
        <li
            className={[styles.step, !e.canon && styles.stepOptional, hit && styles.stepHit, nextUp && styles.stepNext, done && styles.stepDone, !e.aired && styles.stepUpcoming].filter(Boolean).join(" ")}
            id={`a-${e.id}`}
        >
            <span className={styles.stepIndex} aria-hidden>{step ?? ""}</span>
            <Link to={`/anime/details?id=${a.id}`} className={styles.stepCard}>
                <span className={styles.stepCover}>
                    <img src={coverOf(a.id)} alt="" loading="lazy" decoding="async" />
                    {e.aired && <span className={styles.stepBar} aria-hidden><span style={{ width: `${ratio * 100}%` }} /></span>}
                </span>
                <span className={styles.stepBody}>
                    <span className={styles.stepTop}>
                        <KindChip e={e} />
                        {e.kind === "movie" && e.canon && <span className={styles.canonHint}>正片剧情</span>}
                        {nextUp && <span className={styles.nextBadge}>继续看</span>}
                        <span className={styles.stepWhen}>{whenText(e.date)}</span>
                        {eps > 0 && e.kind !== "movie" && <span className={styles.stepWhen}>{eps} 话</span>}
                        {a.score > 0 && <span className={styles.stepScore}><FaStar aria-hidden /> {a.score.toFixed(1)}</span>}
                    </span>
                    <span className={styles.stepTitle}>{a.name_cn || a.name}</span>
                    {a.name_cn && a.name && a.name !== a.name_cn && <span className={styles.stepOriginal}>{a.name}</span>}
                    <span className={styles.stepStatus}>
                        {e.aired && <span className={`${styles.stepState} ${st === WatchStatus.WATCHING ? styles.stepStateOn : ""}`}>{statusLabel(st)}</span>}
                        <span>{text}</span>
                    </span>
                    {comment && <span className={styles.stepComment}>「{comment}」</span>}
                </span>
            </Link>
        </li>
    );
}

function MissingStep ({ m }: { m: MissingEntry }): React.ReactElement {
    const { rel } = m;
    return (
        <li className={`${styles.step} ${styles.stepMissing}`}>
            <span className={styles.stepIndex} aria-hidden>?</span>
            <a href={`https://bgm.tv/subject/${rel.id}`} target="_blank" rel="noopener noreferrer" className={styles.stepCard}>
                <span className={styles.stepCover}>
                    <img src={toAnimeCdnUrl(`/data/relation/${rel.id}.jpg`)} alt="" loading="lazy" decoding="async" />
                </span>
                <span className={styles.stepBody}>
                    <span className={styles.stepTop}>
                        <span className={`${styles.chip} ${styles.chipExtra}`}>未收录 · {missingLabel(rel)}</span>
                    </span>
                    <span className={styles.stepTitle}>{rel.name_cn || rel.name}</span>
                    {rel.name_cn && rel.name && rel.name !== rel.name_cn && <span className={styles.stepOriginal}>{rel.name}</span>}
                    <span className={styles.stepStatus}>去 Bangumi 看看 <FaExternalLinkAlt aria-hidden /></span>
                </span>
            </a>
        </li>
    );
}

function LineSection ({ line, showExtras, hits, next }: { line: WorkLine; showExtras: boolean; hits: Set<number>; next: Entry | null }): React.ReactElement {
    const shown = line.entries.filter((e) => showExtras || e.canon);
    const canon = line.entries.filter((e) => e.canon && e.aired);
    let watched = 0;
    let total = 0;
    for (const e of canon) { const p = entryEpisodes(e); watched += p.watched; total += p.total; }
    const doneCount = canon.filter((e) => e.record.user_status.watch_status === WatchStatus.WATCHED).length;
    const dates = line.entries.map((e) => e.date).filter(Boolean).sort();
    const years = dates.length ? (dates[0].slice(0, 4) === dates[dates.length - 1].slice(0, 4) ? dates[0].slice(0, 4) : `${dates[0].slice(0, 4)}–${dates[dates.length - 1].slice(0, 4)}`) : "";
    const tv = line.entries.filter((e) => e.kind === "tv").length;
    const movies = line.entries.filter((e) => e.kind === "movie").length;
    let step = 0;
    return (
        <section className={styles.line} style={lineStyle(line.hue)} aria-labelledby={`line-${line.index}`} id={`line-${line.index}-anchor`}>
            <header className={styles.lineHead}>
                <span className={styles.lineSwatch} aria-hidden />
                <div className={styles.lineTitles}>
                    <Heading as="h3" id={`line-${line.index}`} className={styles.lineTitle}>
                        {line.title}
                        {line.spinoff ? <span className={styles.lineTag}>衍生</span> : line.index === 0 ? <span className={styles.lineTag}>本篇</span> : null}
                    </Heading>
                    {line.originalTitle && line.originalTitle !== line.title && <p className={styles.lineOriginal}>{line.originalTitle}</p>}
                </div>
                <div className={styles.lineMeta}>
                    <span>{[tv ? `TV ${tv} 部` : "", movies ? `剧场版 ${movies} 部` : "", years].filter(Boolean).join(" · ")}</span>
                    {canon.length > 0 && (
                        <span className={styles.lineProgress} title={`已看 ${watched} / ${total} 话`}>
                            <span className={styles.lineProgressBar}><span style={{ width: `${total ? Math.round((watched / total) * 100) : 0}%` }} /></span>
                            {doneCount} / {canon.length} 部
                        </span>
                    )}
                </div>
            </header>
            <ol className={styles.route}>
                {line.missing.filter((m) => m.before).map((m) => <MissingStep key={m.rel.id} m={m} />)}
                {shown.map((e) => {
                    if (e.canon) step += 1;
                    return <StepCard key={e.id} e={e} step={e.canon ? step : null} hit={hits.has(e.id)} nextUp={next?.id === e.id} />;
                })}
                {line.missing.filter((m) => !m.before).map((m) => <MissingStep key={m.rel.id} m={m} />)}
            </ol>
        </section>
    );
}

/**
 * 放送时间线: 横向滚动的封面卡轨道. 卡片按首播排序, 两卡之间的空隙随放送间隔伸缩 (对数压缩, 封顶),
 * 跨年处插年份刻度, 间隔 >= 1 年标注; 未定档的放在末尾单独一段. 卡片顶部色条 = 所属作品线.
 */
function Timeline ({ lines, showExtras, focusId }: { lines: WorkLine[]; showExtras: boolean; focusId: number | null }): React.ReactElement | null {
    const trackRef = React.useRef<HTMLDivElement>(null);
    const hueOf = new Map<number, WorkLine>();
    for (const l of lines) for (const e of l.entries) hueOf.set(e.id, l);
    const all = lines.flatMap((l) => l.entries).filter((e) => showExtras || e.canon);
    const dated = all.filter((e) => e.date).sort(byDate);
    const undated = all.filter((e) => !e.date);
    const today = todayIso();

    React.useEffect(() => {
        const track = trackRef.current;
        if (!track || focusId === null) return;
        const card = track.querySelector<HTMLElement>(`[data-id="${focusId}"]`);
        if (card) track.scrollLeft = Math.max(0, card.offsetLeft - track.clientWidth / 2 + card.clientWidth / 2);
    }, [focusId, showExtras]);

    const scrollBy = (dir: number): void => {
        const track = trackRef.current;
        if (track) track.scrollBy({ left: dir * track.clientWidth * 0.8, behavior: "smooth" });
    };

    if (all.length < 2) return null;

    const nodes: React.ReactNode[] = [];
    let prevDate = "";
    let nowPlaced = false;
    dated.forEach((e, i) => {
        const year = e.date.slice(0, 4);
        if (i > 0) {
            const months = Math.max(0, monthsBetween(prevDate, e.date));
            const width = Math.round(Math.min(160, 12 + Math.log2(1 + months) * 26));
            nodes.push(
                <span key={`gap-${e.id}`} className={styles.tlGap} style={{ width }} aria-hidden>
                    {months >= 12 && <span className={styles.tlGapText}>{gapText(months)}</span>}
                </span>,
            );
        }
        if (!nowPlaced && e.date > today && prevDate && prevDate <= today) {
            nodes.push(<span key="now" className={styles.tlNow}><span>今天</span></span>);
            nowPlaced = true;
        }
        if (i === 0 || year !== prevDate.slice(0, 4)) {
            nodes.push(<span key={`y-${e.id}`} className={styles.tlYear}><span>{year}</span></span>);
        }
        nodes.push(<TimelineCard key={e.id} e={e} line={hueOf.get(e.id)!} focus={focusId === e.id} multiLine={lines.length > 1} />);
        prevDate = e.date;
    });
    if (undated.length) {
        nodes.push(<span key="tbd" className={styles.tlYear}><span>未定档</span></span>);
        undated.forEach((e) => nodes.push(<TimelineCard key={e.id} e={e} line={hueOf.get(e.id)!} focus={focusId === e.id} multiLine={lines.length > 1} />));
    }

    return (
        <div className={styles.tlWrap}>
            <button type="button" className={`${styles.tlNav} ${styles.tlNavPrev}`} onClick={() => scrollBy(-1)} aria-label="向前滚动">‹</button>
            <div ref={trackRef} className={styles.tlTrack} aria-label="放送时间线" role="list">
                {nodes}
            </div>
            <button type="button" className={`${styles.tlNav} ${styles.tlNavNext}`} onClick={() => scrollBy(1)} aria-label="向后滚动">›</button>
        </div>
    );
}

function TimelineCard ({ e, line, focus, multiLine }: { e: Entry; line: WorkLine; focus: boolean; multiLine: boolean }): React.ReactElement {
    const a = e.record.anime_data;
    const st = e.record.user_status.watch_status;
    const { ratio } = entryProgress(e);
    return (
        <Link
            to={`/anime/details?id=${a.id}`}
            className={[styles.tlCard, !e.canon && styles.tlCardOptional, focus && styles.tlCardFocus, st === WatchStatus.WATCHED && styles.tlCardDone].filter(Boolean).join(" ")}
            style={lineStyle(line.hue)}
            data-id={a.id}
            role="listitem"
            title={`${a.name_cn || a.name}${a.name && a.name !== a.name_cn ? `\n${a.name}` : ""}\n${e.label} · ${e.date || "未定档"}`}
        >
            <span className={styles.tlDot} aria-hidden />
            <span className={styles.tlDate}>{e.date ? e.date.slice(0, 7).replace("-", ".") : "TBD"}</span>
            <span className={styles.tlCover}>
                <img src={coverOf(a.id)} alt="" loading="lazy" decoding="async" />
                <span className={styles.tlKind}>{e.short}</span>
                {e.aired && ratio > 0 && <span className={styles.tlBar} aria-hidden><span style={{ width: `${ratio * 100}%` }} /></span>}
            </span>
            <span className={styles.tlName}>{a.name_cn || a.name}</span>
            {a.name && a.name !== a.name_cn && <span className={styles.tlOriginal}>{a.name}</span>}
            {multiLine && <span className={styles.tlLine}>{line.title}</span>}
        </Link>
    );
}

// ---------------------------------------------------------------- 页面

/**
 * 系列过渡页 /anime/series?id= : 同一番剧各季按顺序列出 (Hero / 继续观看 / 简介卡 / 季卡 / 特别篇 / 时间线).
 * 同目录 details.tsx 的 SeriesNav 与 index.tsx 的系列视图也由本锚点代表.
 * .agents/notes/implemented/feature/2026-11-05-anime-series-fold.md
 */
export default function AnimeSeriesPage (): React.ReactElement {
    const query = useQuery();
    const id = query.get("id");
    const hitParam = query.get("hit") ?? "";
    const { siteConfig } = useDocusaurusContext();
    const isBrowser = useIsBrowser();
    const history = useHistory();
    const { withBaseUrl } = useBaseUrlUtils();
    const { records, status, error, retry } = useAnimeRecordsState(siteConfig.baseUrl);
    const { byId } = useAnimeSeries(records);
    const [showExtras, setShowExtras] = React.useState(true);

    const validId = !!id && /^\d+$/.test(id);
    const series = validId ? byId.get(Number(id)) : undefined;
    const hits = React.useMemo(() => new Set(hitParam.split(",").map(Number).filter((n) => n > 0)), [hitParam]);
    const lines = React.useMemo(() => (series ? buildLines(series) : []), [series]);

    React.useEffect(() => {
        if (!series) return;
        if (series.seasons.length === 1) {
            history.replace(withBaseUrl(`/anime/details?id=${series.key}`));
        } else if (String(series.key) !== id) {
            history.replace(withBaseUrl(`/anime/series?id=${series.key}${hitParam ? `&hit=${hitParam}` : ""}`));
        }
    }, [series, id, hitParam, history, withBaseUrl]);

    if (!series && (!isBrowser || (validId && status === "loading"))) {
        return (
            <Layout title="アニメ">
                <div className={detail.shell} aria-busy="true" aria-label="正在加载番剧系列">
                    <div className={detail.skeletonHero}>
                        <div className={detail.skeletonCover} />
                        <div className={detail.skeletonLines}>
                            <div className={`${detail.skeletonLine} ${detail.skeletonTitle}`} />
                            <div className={`${detail.skeletonLine} ${detail.skeletonW40}`} />
                            <div className={`${detail.skeletonLine} ${detail.skeletonW75}`} />
                        </div>
                    </div>
                </div>
            </Layout>
        );
    }

    if (!series && validId && status === "error") {
        return (
            <Layout title="アニメ">
                <div className={`${detail.shell} ${detail.center}`} role="alert">
                    <Heading as="h2" id="load-error">追番记录没能加载出来</Heading>
                    <p>数据从 HXLoLi-ANiMe 的 CDN 拉取, 可能是网络或 CDN 节点问题{error ? `: ${error}` : ""}.</p>
                    <button type="button" className="button button--primary" onClick={retry}>重试</button>
                </div>
            </Layout>
        );
    }

    if (!series) {
        return (
            <Layout title="アニメ 404 NOT FOUND">
                <div className={`${detail.shell} ${detail.center}`}>
                    <Heading as="h2" id="404">404 NOT FOUND</Heading>
                    <p>追番记录里没有这个系列</p>
                    <p><Link to="/anime?fold=series">← 回番剧库</Link></p>
                </div>
            </Layout>
        );
    }

    const mainLine = lines[0];
    const first = (mainLine?.entries.find((e) => e.kind === "tv") ?? mainLine?.entries[0])?.record.anime_data ?? series.seasons[0].record.anime_data;
    const coverUrl = coverOf(seriesCoverId(series));
    const agg = aggregate(lines);
    const range = seriesScoreRange(series);
    const years = seriesYearRange(series);
    const tags = seriesTags(series);
    const next = nextUpEntry(lines);
    const firstCanon = mainLine?.entries.find((e) => e.canon) ?? mainLine?.entries[0];
    const title = mainLine?.title || series.title;
    const original = mainLine?.originalTitle || series.originalTitle;
    const focusId = [...hits].find((h) => lines.some((l) => l.entries.some((e) => e.id === h))) ?? next?.id ?? null;
    return (
        <>
            <PageMetadata title={title} description={first.short_summary || first.summary} />
            <Layout title={title}>
                {/* 两侧立绘藏在正文墙后 (同博客文章页), 墙宽与 .shell 一致 */}
                <BlogWithCats contentWidth={1200}>
                <div className={`${detail.shell} ${styles.shellFit}`}>
                    <Link to="/anime?fold=series" className={styles.back}>番剧库 / 系列</Link>

                    <header className={styles.head}>
                        <div className={styles.cover}>
                            <span className={styles.coverFallback} aria-hidden>{title}</span>
                            <img
                                src={coverUrl}
                                alt={title}
                                className={styles.coverImg}
                                onError={(ev) => { ev.currentTarget.style.visibility = "hidden"; }}
                            />
                        </div>

                        <div className={styles.headMain}>
                            <p className={styles.kicker}>
                                系列{lines.length > 1 ? ` · ${lines.length} 条作品线` : ""}
                            </p>
                            <h1 className={styles.title}>{title}</h1>
                            {original && original !== title && <p className={styles.original}>{original}</p>}

                            <ul className={styles.meta} aria-label="系列信息">
                                {range && (
                                    <li className={styles.metaScore}>
                                        <FaStar aria-hidden />
                                        <strong>{range.min === range.max ? range.max.toFixed(1) : `${range.min.toFixed(1)}–${range.max.toFixed(1)}`}</strong>
                                        <span>Bangumi</span>
                                    </li>
                                )}
                                {years && <li>{years}</li>}
                                {tags.map((tag) => {
                                    const susume = susumeDefOfTag(tag);
                                    return (
                                        <li key={tag}>
                                            {susume ? (
                                                <Link to={susumePath(susume)} className={styles.metaLink}>{tag}</Link>
                                            ) : (
                                                <Link to={`/anime?fold=series&${ANIME_TAG_QUERY}=${encodeURIComponent(tag)}`} className={styles.metaLink}>#{tag}</Link>
                                            )}
                                        </li>
                                    );
                                })}
                            </ul>

                            <Synopsis text={first.summary || first.short_summary} />

                            <dl className={styles.facts}>
                                <div className={styles.fact}>
                                    <dt>正片</dt>
                                    <dd>{agg.canonCount}<small> 部</small>{agg.optionalCount ? <small> + {agg.optionalCount} 番外</small> : null}</dd>
                                </div>
                                {agg.total > 0 && (
                                    <div className={styles.fact}>
                                        <dt>话数</dt>
                                        <dd>{agg.total}<small> 话</small></dd>
                                    </div>
                                )}
                                <div className={styles.fact}>
                                    <dt>{agg.label}</dt>
                                    <dd>
                                        {agg.watched}<small>{agg.total > 0 ? ` / ${agg.total}` : ""}</small>
                                    </dd>
                                </div>
                            </dl>
                            <div className={styles.progress} role="progressbar" aria-label="系列观看进度" aria-valuemin={0} aria-valuemax={agg.total || 1} aria-valuenow={Math.min(agg.watched, agg.total || 1)}>
                                <span style={{ width: `${Math.round(agg.ratio * 100)}%` }} />
                            </div>

                            {lines.length > 1 && (
                                <nav className={styles.lineNav} aria-label="作品线">
                                    {lines.map((l) => (
                                        <a key={l.index} href={`#line-${l.index}-anchor`} className={styles.lineNavItem} style={lineStyle(l.hue)}>
                                            <span className={styles.lineSwatch} aria-hidden />
                                            {l.title}
                                            <small>{l.entries.filter((e) => e.canon).length}</small>
                                        </a>
                                    ))}
                                </nav>
                            )}

                            <div className={styles.actions}>
                                {next ? (
                                    <Link className={`${styles.action} ${styles.primary}`} to={`/anime/details?id=${next.id}`}>
                                        <FaPlay aria-hidden /> 继续看 · {next.record.anime_data.name_cn || next.record.anime_data.name}
                                    </Link>
                                ) : firstCanon ? (
                                    <Link className={`${styles.action} ${styles.primary}`} to={`/anime/details?id=${firstCanon.id}`}>
                                        <FaPlay aria-hidden /> 从头重温
                                    </Link>
                                ) : null}
                                <Link className={styles.action} to={`/anime/graph?id=${series.key}`}>
                                    <FaCodeBranch aria-hidden /> 关系图
                                </Link>
                                <a className={styles.action} href={`https://bgm.tv/subject/${series.key}`} target="_blank" rel="noopener noreferrer">
                                    <FaExternalLinkAlt aria-hidden /> Bangumi
                                </a>
                            </div>
                        </div>
                    </header>

                    <section className={styles.block} aria-labelledby="timeline">
                        <div className={styles.blockHead}>
                            <Heading as="h2" id="timeline" className={detail.sectionTitle}>放送时间线</Heading>
                            <label className={styles.toggle}>
                                <input type="checkbox" checked={showExtras} onChange={(ev) => setShowExtras(ev.target.checked)} />
                                <span>显示 OVA / 番外 / 总集篇</span>
                            </label>
                        </div>
                        <Timeline lines={lines} showExtras={showExtras} focusId={focusId} />
                    </section>

                    <section className={styles.block} aria-labelledby="order">
                        <div className={styles.blockHead}>
                            <Heading as="h2" id="order" className={detail.sectionTitle}>
                                观看顺序 <span className={detail.muted}>剧场版按首播插入正片</span>
                            </Heading>
                        </div>
                        <div className={styles.lines}>
                            {lines.map((l) => <LineSection key={l.index} line={l} showExtras={showExtras} hits={hits} next={next} />)}
                        </div>
                    </section>
                </div>
                </BlogWithCats>
            </Layout>
        </>
    );
}
