import React from "react";
import Link from "@docusaurus/Link";
import { useHistory, useLocation } from "@docusaurus/router";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import useIsBrowser from "@docusaurus/useIsBrowser";
import { useAnimeRecordsState } from "@site/src/utils/anime/animeStore";
import {
    activeConditions,
    AnimeFilters,
    applyFilters,
    DEFAULT_FILTERS,
    facetCounts,
    groupRecords,
    serializeFilters,
    SORT_OPTIONS,
    SortKey,
    statusLabel,
    STATUS_OPTIONS,
} from "@site/src/utils/anime/filters";
import type { RecordGroup } from "@site/src/utils/anime/filters";
import { ANiMeRecord, WatchStatus } from "@site/src/utils/anime/types";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";
import {
    airYearOf,
    inYearRange,
    MARK_OPTIONS,
    MarkFilter,
    matchesMark,
    parseShareQuery,
    readPicks,
    sharePath,
    shareUrl,
    storageKeyOf,
    writePicks,
    YEAR_RANGE_OPTIONS,
    YearRange,
} from "./animeShareLink";
import { canCopyImage, copyImage, downloadImages, makeShareImage, todayText } from "./animeShareExport";
import { copyText, toCommaList, toLineList } from "./animeShareText";
import styles from "./AnimeShareBoard.module.css";

const TAG_PREVIEW = 18;

const displayName = (r: ANiMeRecord): string => r.anime_data.name_cn || r.anime_data.name;

/** 分享图里每部都标成"看过" (图的标题就是"我看过的番"), 不沿用站长自己的观看状态 */
const asWatched = (r: ANiMeRecord): ANiMeRecord => ({ ...r, user_status: { ...r.user_status, watch_status: WatchStatus.WATCHED } });

type Flash = "link" | "line" | "comma" | null;

type ViewMode = "card" | "table";
const VIEW_PARAM = "view";
/**
 * 独立页的视图模式只认 URL 上的 view=table, 其余一律卡片; 卡片是默认, 不写参数, 老链接原样可用. 弹窗里 (embedded) 改用组件 state.
 * 筛选结果 (visible) 与勾选 (mine) 不看这个参数, 两种模式共用, 切换不丢状态; 表格模式的决策见文件里主组件的注释.
 */
function viewModeOf (search: string): ViewMode {
    return new URLSearchParams(search).get(VIEW_PARAM) === "table" ? "table" : "card";
}

/** 去掉 view 后的查询串: 番单定义与分享者勾选只随它变, 切视图不触发重算 */
function withoutView (search: string): string {
    const p = new URLSearchParams(search);
    p.delete(VIEW_PARAM);
    const q = p.toString();
    return q ? `?${q}` : "";
}

/** 给链接 / 站内路径追加视图参数; 卡片模式原样返回 */
function withView (url: string, mode: ViewMode): string {
    return mode === "table" ? `${url}${url.includes("?") ? "&" : "?"}${VIEW_PARAM}=table` : url;
}

interface YearRow {
    key: string;
    label: string;
    records: ANiMeRecord[];
}

/** 表格模式按首播年份分组: 年份升序 (同参考站从早到晚), 组内保持当前排序; 没有首播日期的是还没播的, 归"待推出"放最后 */
function yearRowsOf (list: readonly ANiMeRecord[]): YearRow[] {
    const byYear = new Map<number, ANiMeRecord[]>();
    const unknown: ANiMeRecord[] = [];
    for (const r of list) {
        const y = airYearOf(r);
        if (y === null) {
            unknown.push(r);
            continue;
        }
        const bucket = byYear.get(y);
        if (bucket) bucket.push(r);
        else byYear.set(y, [r]);
    }
    const rows: YearRow[] = [...byYear.keys()].sort((a, b) => a - b).map((y) => ({ key: String(y), label: String(y), records: byYear.get(y) ?? [] }));
    if (unknown.length) rows.push({ key: "upcoming", label: "待推出", records: unknown });
    return rows;
}

const itemTitle = (r: ANiMeRecord): string =>
    `${displayName(r)}${r.anime_data.name !== displayName(r) ? `「${r.anime_data.name}」` : ""} · 站长${statusLabel(r.user_status.watch_status)}`;

type CoverKey = "cover" | "plain";

const COVER_OPTIONS: readonly { value: CoverKey; label: string }[] = [
    { value: "cover", label: "带封面" },
    { value: "plain", label: "不带封面" },
];

/** 一个版本 (带封面 / 不带封面) 的生成结果: PNG 分页与对应的 blob: URL, 两者下标一一对应 */
interface PreviewImage {
    pages: Blob[];
    urls: string[];
    failedCovers: number;
}

function revokePreview (img: PreviewImage | undefined): void {
    img?.urls.forEach((u) => URL.revokeObjectURL(u));
}

interface ShareImagePreviewProps {
    title: string;
    conditions: readonly string[];
    groups: readonly RecordGroup[];
    siteUrl: string;
    /** 图片内容的指纹; 变了说明已生成的图过期, 两个版本都丢弃 */
    contentKey: string;
    summary: string;
    empty: boolean;
}
/**
 * 分享图预览: 画图仍走 animeShareExport.makeShareImage (内部是 animeShareImage.renderShareImage), 这里只负责展示.
 * - 两个版本各自缓存 (Blob + createObjectURL), 切到没生成过的版本时自动生成; 同一版本重新生成或内容变了时先 revoke 旧 URL, 卸载时全部 revoke.
 * - 多页 (画布超高被切开) 用 page 下标翻页; 复制只复制当前页, 下载走 downloadImages 一次下全部页.
 * - jobRef 递增作废进行中的生成, 防止旧结果覆盖新内容.
 * 出图与文本清单的决策见 animeShareExport.ts 顶部注释
 */
function ShareImagePreview ({ title, conditions, groups, siteUrl, contentKey, summary, empty }: ShareImagePreviewProps): React.ReactElement {
    const [version, setVersion] = React.useState<CoverKey>("cover");
    const [images, setImages] = React.useState<Partial<Record<CoverKey, PreviewImage>>>({});
    const [page, setPage] = React.useState(0);
    const [size, setSize] = React.useState<{ w: number; h: number } | null>(null);
    const [busy, setBusy] = React.useState<string | null>(null);
    const [message, setMessage] = React.useState<string | null>(null);
    const [copied, setCopied] = React.useState(false);
    const imagesRef = React.useRef(images);
    const jobRef = React.useRef(0);

    const replaceImages = (next: Partial<Record<CoverKey, PreviewImage>>): void => {
        imagesRef.current = next;
        setImages(next);
    };

    // 内容变了: 作废进行中的生成, revoke 两个版本的旧 URL
    React.useEffect(() => {
        jobRef.current += 1;
        revokePreview(imagesRef.current.cover);
        revokePreview(imagesRef.current.plain);
        replaceImages({});
        setPage(0);
        setSize(null);
        setBusy(null);
        setMessage(null);
    }, [contentKey]);

    // 卸载时释放全部 blob: URL
    React.useEffect(() => () => {
        jobRef.current += 1;
        revokePreview(imagesRef.current.cover);
        revokePreview(imagesRef.current.plain);
    }, []);

    const generate = async (v: CoverKey): Promise<void> => {
        const job = ++jobRef.current;
        setBusy("生成中…");
        setMessage(null);
        try {
            const res = await makeShareImage({
                title,
                conditions,
                groups,
                withCovers: v === "cover",
                siteUrl,
                onProgress: (done, n) => { if (job === jobRef.current) setBusy(`加载封面 ${done} / ${n}…`); },
            });
            if (job !== jobRef.current) return;
            const next: PreviewImage = { pages: res.pages, urls: res.pages.map((b) => URL.createObjectURL(b)), failedCovers: res.failedCovers };
            revokePreview(imagesRef.current[v]);
            replaceImages({ ...imagesRef.current, [v]: next });
            setVersion(v);
            setPage(0);
            setSize(null);
            if (v === "cover" && res.failedCovers > 0) setMessage(`${res.failedCovers} 张封面没加载上, 画成了占位块.`);
        } catch (e) {
            if (job === jobRef.current) setMessage(`图片生成失败: ${e instanceof Error ? e.message : String(e)}`);
        } finally {
            if (job === jobRef.current) setBusy(null);
        }
    };

    const pickVersion = (v: CoverKey): void => {
        setVersion(v);
        setPage(0);
        setSize(null);
        setMessage(null);
        if (!imagesRef.current[v] && !empty) void generate(v);
    };

    const current = images[version];
    const total = current?.urls.length ?? 0;
    const cur = Math.min(page, Math.max(total - 1, 0));
    const label = COVER_OPTIONS.find((o) => o.value === version)?.label ?? "";
    const goPage = (p: number): void => {
        setPage(p);
        setSize(null);
    };

    const download = (): void => {
        if (current) downloadImages(current.pages, `hxloli-anime-share-${todayText()}${version === "plain" ? "-plain" : ""}`);
    };
    const copy = async (): Promise<void> => {
        if (!current) return;
        try {
            await copyImage(current.pages[cur]);
            setCopied(true);
            window.setTimeout(() => setCopied(false), 1800);
        } catch (e) {
            setMessage(`复制失败: ${e instanceof Error ? e.message : String(e)}`);
        }
    };

    return (
        <div className={styles.previewBox}>
            <div className={styles.shareRow}>
                <span className={styles.filterLabel}>分享图</span>
                {COVER_OPTIONS.map((o) => (
                    <button key={o.value} type="button" className={styles.pill} aria-pressed={version === o.value} disabled={busy !== null} onClick={() => pickVersion(o.value)}>
                        {o.label}
                    </button>
                ))}
                <span className={styles.spacer} />
                <button type="button" className="button button--primary" disabled={busy !== null || empty} onClick={() => { void generate(version); }}>
                    {current ? "重新生成" : "生成图片"}
                </button>
            </div>
            <p className={styles.muted}>{busy ?? summary}</p>
            {current && total > 0 && (
                <div className={styles.preview}>
                    <figure className={styles.previewFrame}>
                        <img
                            className={styles.previewImg}
                            src={current.urls[cur]}
                            alt={`分享图预览 (${label}) 第 ${cur + 1} 页`}
                            onLoad={(e) => setSize({ w: e.currentTarget.naturalWidth, h: e.currentTarget.naturalHeight })}
                        />
                    </figure>
                    <div className={styles.previewSide}>
                        <div className={styles.pager}>
                            <button type="button" className={styles.pill} disabled={cur === 0} onClick={() => goPage(cur - 1)}>上一页</button>
                            <span className={styles.pageNo} aria-live="polite">第 {cur + 1} / {total} 页</span>
                            <button type="button" className={styles.pill} disabled={cur >= total - 1} onClick={() => goPage(cur + 1)}>下一页</button>
                        </div>
                        <span className={styles.muted}>{label}{size ? ` · ${size.w} × ${size.h} px` : ""}</span>
                        <button type="button" className="button button--outline button--primary" disabled={busy !== null} onClick={download}>
                            {total > 1 ? `下载全部 ${total} 张` : "下载图片"}
                        </button>
                        {canCopyImage() && (
                            <button type="button" className="button button--outline button--primary" disabled={busy !== null} onClick={() => { void copy(); }}>
                                {copied ? "已复制 ✓" : total > 1 ? `复制第 ${cur + 1} 页` : "复制图片"}
                            </button>
                        )}
                    </div>
                </div>
            )}
            {message && <p className={styles.warn} role="status">{message}</p>}
        </div>
    );
}
/**
 * embedded: 挂在番剧库 /anime 的分享 <dialog> 里时为 true. 此时不读写地址栏的 view (视图模式是组件内 state, 打开弹窗不改 URL),
 * 不显示"回番剧库", 改为"在独立页面打开"; 番单定义仍读当前 location.search (即番剧库的筛选), 与 /anime/share 共用本组件与本地勾选.
 */
export interface AnimeShareBoardProps {
    embedded?: boolean;
}
/**
 * /anime/share 的可交互番单 (对标 anime-sedai: 点番标记"我看过", 顶部实时"我看过 N/M 部", 生成自己的链接再传播).
 * - 番单 = URL 上的筛选 (与番剧库同一套 parseFilters / applyFilters) 作用在 CDN 的 ANiMeRecord.json 上, M 固定为这份番单的部数;
 *   URL 里的 pick 是分享者勾的"TA 看过", 只读, 用来对照; 打开的人自己的勾选存 localStorage, 键带番单标识 (listKeyOf).
 * - 页内筛选 (观看状态 / 标签 / 搜索 / 排序 用 AnimeFilters + applyFilters + facetCounts; 年份范围与"勾选状态"是分享页专属)
 *   只改本页显示, 不改番单本身, 所以 M 与本地勾选的键不随筛选变化.
 * - "生成我的链接" = 原番单筛选 + 我的勾选 (encodeIds) 重新编码; 图片导出复用 animeShareImage.ts.
 * 条目用自画的轻量按钮而不是 PosterCard: PosterCard 整卡是详情链接 + 标签按钮, 套不进"点一下切换勾选"的语义.
 * 纯静态, 不调 Bangumi API.
 * .agents/notes/implemented/feature/2026-11-01-anime-share-table-view-and-preview.md
 */
export default function AnimeShareBoard ({ embedded = false }: AnimeShareBoardProps = {}): React.ReactElement {
    const location = useLocation();
    const history = useHistory();
    const isBrowser = useIsBrowser();
    const { siteConfig } = useDocusaurusContext();
    const { records, status, error, retry } = useAnimeRecordsState(siteConfig.baseUrl);

    // 番单定义与分享者勾选: 只在 URL 变化时重读
    const listSearch = React.useMemo(() => withoutView(location.search), [location.search]);
    const { filters: base, pick } = React.useMemo(() => parseShareQuery(listSearch), [listSearch]);

    // 视图模式 (卡片 / 表格): 独立页存在 URL 的 view 参数里, 复制地址栏或"生成我的链接"都会带上; 弹窗里只存 state, 不动地址栏
    const [embeddedMode, setEmbeddedMode] = React.useState<ViewMode>("card");
    const mode = embedded ? embeddedMode : viewModeOf(location.search);
    const setMode = (next: ViewMode): void => {
        if (next === mode) return;
        if (embedded) {
            setEmbeddedMode(next);
            return;
        }
        const p = new URLSearchParams(location.search);
        if (next === "table") p.set(VIEW_PARAM, "table");
        else p.delete(VIEW_PARAM);
        const q = p.toString();
        history.replace({ pathname: location.pathname, search: q ? `?${q}` : "", hash: location.hash });
    };
    const theirs = React.useMemo(() => new Set(pick), [pick]);
    const baseList = React.useMemo(() => applyFilters(records, base), [records, base]);
    const baseIds = React.useMemo(() => new Set(baseList.map((r) => r.anime_data.id)), [baseList]);
    const baseLabels = React.useMemo(() => activeConditions(base).map((c) => c.label), [base]);

    // 本地勾选: 键带番单标识; 键变了先读再写, 免得把上一份番单的勾选写进新键
    const storageKey = storageKeyOf(siteConfig.baseUrl, base);
    const [mine, setMine] = React.useState<Set<number>>(() => new Set());
    const [loadedKey, setLoadedKey] = React.useState<string | null>(null);
    React.useEffect(() => {
        setMine(readPicks(storageKey));
        setLoadedKey(storageKey);
    }, [storageKey]);
    React.useEffect(() => {
        if (loadedKey === storageKey) writePicks(storageKey, mine);
    }, [loadedKey, storageKey, mine]);

    // 页内筛选
    const [view, setView] = React.useState<AnimeFilters>(() => ({ ...DEFAULT_FILTERS, sort: "air" }));
    const [range, setRange] = React.useState<YearRange>("all");
    const [mark, setMark] = React.useState<MarkFilter>("all");
    const [showAllTags, setShowAllTags] = React.useState(false);
    const thisYear = React.useMemo(() => new Date().getFullYear(), []);

    const ranged = React.useMemo(() => baseList.filter((r) => inYearRange(r, range, thisYear)), [baseList, range, thisYear]);
    const counts = React.useMemo(() => facetCounts(ranged, view), [ranged, view]);
    const visible = React.useMemo(
        () => applyFilters(ranged, view).filter((r) => matchesMark(r.anime_data.id, mark, mine, theirs)),
        [ranged, view, mark, mine, theirs],
    );
    const groups = React.useMemo(() => groupRecords(visible, view.sort), [visible, view.sort]);
    const yearRows = React.useMemo(() => (mode === "table" ? yearRowsOf(visible) : []), [mode, visible]);

    const mineInList = React.useMemo(() => baseList.filter((r) => mine.has(r.anime_data.id)), [baseList, mine]);
    const theirsInList = React.useMemo(() => baseList.filter((r) => theirs.has(r.anime_data.id)).length, [baseList, theirs]);
    const bothInList = React.useMemo(() => mineInList.filter((r) => theirs.has(r.anime_data.id)).length, [mineInList, theirs]);
    const visibleMine = React.useMemo(() => visible.filter((r) => mine.has(r.anime_data.id)).length, [visible, mine]);
    const total = baseList.length;
    const percent = total ? Math.round((mineInList.length / total) * 100) : 0;

    const toggle = React.useCallback((id: number) => {
        setMine((prev) => {
            const next = new Set(prev);
            if (next.has(id)) next.delete(id);
            else next.add(id);
            return next;
        });
    }, []);
    const setMany = (ids: readonly number[], on: boolean): void => {
        setMine((prev) => {
            const next = new Set(prev);
            for (const id of ids) {
                if (on) next.add(id);
                else next.delete(id);
            }
            return next;
        });
    };
    const copyTheirs = (): void => setMine(new Set([...theirs].filter((id) => baseIds.has(id))));
    const clearMine = (): void => {
        if (mine.size === 0 || window.confirm(`清空这份番单里你勾的 ${mine.size} 部?`)) setMine(new Set());
    };

    const toggleStatus = (s: WatchStatus): void =>
        setView((v) => ({ ...v, status: v.status.includes(s) ? v.status.filter((x) => x !== s) : [...v.status, s] }));
    const toggleTag = (t: string): void =>
        setView((v) => ({ ...v, tags: v.tags.includes(t) ? v.tags.filter((x) => x !== t) : [...v.tags, t] }));
    const resetView = (): void => {
        setView((v) => ({ ...DEFAULT_FILTERS, sort: v.sort }));
        setRange("all");
        setMark("all");
    };
    const viewDirty = range !== "all" || mark !== "all" || serializeFilters({ ...view, sort: DEFAULT_FILTERS.sort }) !== "";

    // 生成我的链接: 原番单筛选 + 我的勾选 (只编码番单内的 id)
    const myPick = React.useMemo(() => mineInList.map((r) => r.anime_data.id), [mineInList]);
    const [myLink, setMyLink] = React.useState<string | null>(null);
    const [flash, setFlash] = React.useState<Flash>(null);
    const [message, setMessage] = React.useState<string | null>(null);
    const flashOn = (kind: Flash): void => {
        setFlash(kind);
        window.setTimeout(() => setFlash((f) => (f === kind ? null : f)), 1800);
    };
    const makeLink = async (): Promise<void> => {
        const url = withView(shareUrl(siteConfig.url, siteConfig.baseUrl, base, myPick), mode);
        setMyLink(url);
        setMessage(null);
        try {
            await copyText(url);
            flashOn("link");
        } catch (e) {
            setMessage(`没能自动复制, 请手动复制上面的链接 (${e instanceof Error ? e.message : String(e)})`);
        }
    };
    // 勾选或视图变了, 旧链接作废, 避免复制到过期的
    React.useEffect(() => { setMyLink(null); }, [myPick, mode]);

    // 图片: 有勾选就画"我看过的", 否则画当前页内筛选结果; 生成与预览在 ShareImagePreview
    const siteHost = `${siteConfig.url.replace(/^https?:\/\//, "").replace(/\/$/, "")}${siteConfig.baseUrl.replace(/\/$/, "")}`;
    const imageSource = mineInList.length ? mineInList : visible;
    const imageTitle = mineInList.length ? `我看过的番 · ${mineInList.length}/${total}` : `番单 · ${visible.length} 部`;
    const imageGroups = React.useMemo(
        () => groupRecords(applyFilters(imageSource.map(asWatched), { ...DEFAULT_FILTERS, sort: view.sort }), view.sort),
        [imageSource, view.sort],
    );
    const imageKey = React.useMemo(
        () => [imageTitle, view.sort, baseLabels.join(","), imageSource.map((r) => r.anime_data.id).join(",")].join("|"),
        [imageTitle, view.sort, baseLabels, imageSource],
    );
    const imageSorted = React.useMemo(
        () => applyFilters(imageSource, { ...DEFAULT_FILTERS, sort: view.sort }),
        [imageSource, view.sort],
    );
    const copyList = async (kind: "line" | "comma"): Promise<void> => {
        setMessage(null);
        const link = withView(shareUrl(siteConfig.url, siteConfig.baseUrl, base, myPick), mode);
        try {
            await copyText(kind === "line" ? toLineList(imageTitle, imageSorted, link) : toCommaList(imageSorted));
            flashOn(kind);
        } catch (e) {
            setMessage(`复制失败: ${e instanceof Error ? e.message : String(e)}`);
        }
    };
    const imageSummary = mineInList.length ? `图片内容: 我看过的 ${mineInList.length} 部` : `还没勾选, 图片内容为当前显示的 ${visible.length} 部`;

    if (!isBrowser || (status === "loading" && records.length === 0)) {
        return <div className={styles.shell} aria-busy="true"><p className={styles.muted}>正在加载番单…</p></div>;
    }
    if (status === "error" && records.length === 0) {
        return (
            <div className={styles.shell} role="alert">
                <p>追番记录没能加载出来{error ? `: ${error}` : ""}.</p>
                <button type="button" className="button button--primary" onClick={retry}>重试</button>
            </div>
        );
    }

    const tagList = counts.tags.filter((t) => t.count > 0 || view.tags.includes(t.tag)).sort((a, b) => b.count - a.count);
    const shownTags = showAllTags ? tagList : tagList.slice(0, TAG_PREVIEW);
    const visibleIds = visible.map((r) => r.anime_data.id);
    const hasPick = theirs.size > 0;

    return (
        <div className={`${styles.shell}${embedded ? ` ${styles.embedded}` : ""}`}>
            <header className={styles.head}>
                {embedded
                    ? <Link to={withView(sharePath(base, myPick), mode)} className={styles.back}>在独立页面打开 →</Link>
                    : <Link to={`/anime${serializeFilters(base) ? `?${serializeFilters(base)}` : ""}`} className={styles.back}>← 回番剧库</Link>}
                <h1 className={styles.title}>点击选择你看过的番</h1>
                {baseLabels.length > 0 && (
                    <div className={styles.chips} aria-label="这份番单的条件">
                        <span className={styles.muted}>番单条件</span>
                        {baseLabels.map((c) => <span key={c} className={styles.chip}>{c}</span>)}
                    </div>
                )}
            </header>

            <section className={styles.progressBar} aria-live="polite">
                <p className={styles.progressText}>
                    我看过 <b>{mineInList.length}</b>/{total} 部
                    {hasPick && <span className={styles.compare}> · TA 看过 {theirsInList} 部 · 我们都看过 {bothInList} 部</span>}
                </p>
                <div className={styles.meter} aria-hidden><span style={{ width: `${percent}%` }} /></div>
            </section>

            <div className={styles.layout}>
            <section className={styles.filters} aria-label="页内筛选">
                <div className={styles.filterRow} role="group" aria-label="显示方式">
                    <span className={styles.filterLabel}>显示</span>
                    <button type="button" className={styles.pill} aria-pressed={mode === "card"} onClick={() => setMode("card")}>卡片</button>
                    <button type="button" className={styles.pill} aria-pressed={mode === "table"} onClick={() => setMode("table")}>表格 (按年份)</button>
                </div>
                <div className={styles.filterRow}>
                    <span className={styles.filterLabel}>年份</span>
                    {YEAR_RANGE_OPTIONS.map((o) => (
                        <button key={o.value} type="button" className={styles.pill} aria-pressed={range === o.value} onClick={() => setRange(o.value)}>
                            {o.label}
                        </button>
                    ))}
                </div>
                <div className={styles.filterRow}>
                    <span className={styles.filterLabel}>勾选</span>
                    {MARK_OPTIONS.filter((o) => !o.needsPick || hasPick).map((o) => (
                        <button key={o.value} type="button" className={styles.pill} aria-pressed={mark === o.value} onClick={() => setMark(o.value)}>
                            {o.label}
                        </button>
                    ))}
                </div>
                <div className={styles.filterRow}>
                    <span className={styles.filterLabel}>站长状态</span>
                    {STATUS_OPTIONS.map((o) => {
                        const on = view.status.includes(o.value);
                        const n = counts.status.get(o.value) ?? 0;
                        return (
                            <button key={o.value} type="button" className={styles.pill} aria-pressed={on} disabled={!on && n === 0} onClick={() => toggleStatus(o.value)}>
                                {o.label}<span className={styles.count}>{n}</span>
                            </button>
                        );
                    })}
                </div>
                <div className={styles.filterRow}>
                    <span className={styles.filterLabel}>排序</span>
                    {SORT_OPTIONS.map((o) => (
                        <button key={o.value} type="button" className={styles.pill} aria-pressed={view.sort === o.value} onClick={() => setView((v) => ({ ...v, sort: o.value as SortKey }))}>
                            {o.label}
                        </button>
                    ))}
                    <label className={styles.search}>
                        <span className={styles.srOnly}>在番单里搜索</span>
                        <input
                            type="search"
                            value={view.q}
                            placeholder="搜索番名"
                            onChange={(e) => { const q = e.target.value; setView((v) => ({ ...v, q })); }}
                        />
                    </label>
                </div>
                {tagList.length > 0 && (
                    <div className={styles.filterRow}>
                        <span className={styles.filterLabel}>标签</span>
                        {shownTags.map((t) => (
                            <button key={t.tag} type="button" className={styles.pill} aria-pressed={view.tags.includes(t.tag)} onClick={() => toggleTag(t.tag)}>
                                #{t.tag}<span className={styles.count}>{t.count}</span>
                            </button>
                        ))}
                        {tagList.length > TAG_PREVIEW && (
                            <button type="button" className={styles.linkBtn} onClick={() => setShowAllTags((s) => !s)}>
                                {showAllTags ? "收起" : `更多 ${tagList.length - TAG_PREVIEW} 个`}
                            </button>
                        )}
                        {view.tags.length > 1 && (
                            <button
                                type="button"
                                className={styles.linkBtn}
                                onClick={() => setView((v) => ({ ...v, match: v.match === "all" ? "any" : "all" }))}
                            >
                                {view.match === "all" ? "标签: 全部命中" : "标签: 任一命中"}
                            </button>
                        )}
                    </div>
                )}
                <div className={styles.filterFoot}>
                    <span className={styles.muted}>当前显示 {visible.length} 部, 其中看过 {visibleMine} 部</span>
                    <span className={styles.spacer} />
                    <button type="button" className={styles.linkBtn} disabled={visible.length === 0} onClick={() => setMany(visibleIds, true)}>全选当前</button>
                    <button type="button" className={styles.linkBtn} disabled={visibleMine === 0} onClick={() => setMany(visibleIds, false)}>取消当前</button>
                    {hasPick && <button type="button" className={styles.linkBtn} onClick={copyTheirs}>照抄 TA 的勾选</button>}
                    <button type="button" className={styles.linkBtn} disabled={mine.size === 0} onClick={clearMine}>清空我的</button>
                    {viewDirty && <button type="button" className={styles.linkBtn} onClick={resetView}>重置筛选</button>}
                </div>
            </section>

            <aside className={styles.side}>
            <section className={styles.sharePanel} aria-label="分享">
                <h2 className={styles.shareTitle}>分享我的勾选</h2>
                <p className={styles.muted}>
                    链接里带着这份番单和你勾的 {myPick.length} 部. 别人打开能看到你看过哪些 (标"TA 看过"), 再勾自己的、生成自己的链接.
                </p>
                <div className={styles.shareRow}>
                    <button type="button" className="button button--primary" onClick={() => { void makeLink(); }}>
                        {flash === "link" ? "已复制 ✓" : "生成我的链接"}
                    </button>
                    {myLink && <Link to={withView(sharePath(base, myPick), mode)} className={styles.linkBtn}>在本页打开</Link>}
                </div>
                {myLink && (
                    <label className={styles.linkBox}>
                        <span className={styles.srOnly}>我的分享链接</span>
                        <input type="text" readOnly value={myLink} onFocus={(e) => e.currentTarget.select()} />
                        <span className={styles.muted}>{myLink.length} 字符</span>
                    </label>
                )}
                <div className={styles.shareRow}>
                    <span className={styles.filterLabel}>复制清单</span>
                    <button type="button" className={styles.linkBtn} disabled={imageSorted.length === 0} onClick={() => { void copyList("line"); }}>
                        {flash === "line" ? "已复制 ✓" : "一行一部"}
                    </button>
                    <button type="button" className={styles.linkBtn} disabled={imageSorted.length === 0} onClick={() => { void copyList("comma"); }}>
                        {flash === "comma" ? "已复制 ✓" : "逗号分隔"}
                    </button>
                </div>
                <ShareImagePreview
                    title={imageTitle}
                    conditions={baseLabels}
                    groups={imageGroups}
                    siteUrl={siteHost}
                    contentKey={imageKey}
                    summary={imageSummary}
                    empty={imageSource.length === 0}
                />
                {message && <p className={styles.warn} role="status">{message}</p>}
            </section>
            </aside>

            <div className={styles.listArea}>
            {visible.length === 0 ? (
                <p className={styles.empty}>{total === 0 ? "这份番单是空的." : "没有满足当前筛选的番, 换个条件试试."}</p>
            ) : mode === "table" ? (
                <div className={styles.yearTable}>
                    {yearRows.map((row) => (
                        <section key={row.key} className={styles.yearRow} aria-label={row.label}>
                            <h2 className={styles.yearHead}>{row.label}</h2>
                            <div className={styles.nameFlow}>
                                {row.records.map((r) => {
                                    const id = r.anime_data.id;
                                    return (
                                        <button
                                            key={id}
                                            type="button"
                                            className={styles.nameTag}
                                            aria-pressed={mine.has(id)}
                                            title={itemTitle(r)}
                                            onClick={() => toggle(id)}
                                        >
                                            {theirs.has(id) && <span className={styles.nameTheirs} aria-label="TA 看过">TA</span>}
                                            {displayName(r)}
                                        </button>
                                    );
                                })}
                            </div>
                        </section>
                    ))}
                </div>
            ) : (
                groups.map((g) => (
                    <section key={g.key} className={styles.group}>
                        {g.label && <h2 className={styles.groupTitle}>{g.label}<span className={styles.count}>{g.records.length} 部</span></h2>}
                        <ul className={styles.grid}>
                            {g.records.map((r) => {
                                const id = r.anime_data.id;
                                const on = mine.has(id);
                                const year = airYearOf(r);
                                return (
                                    <li key={id}>
                                        <button
                                            type="button"
                                            className={styles.item}
                                            aria-pressed={on}
                                            title={itemTitle(r)}
                                            onClick={() => toggle(id)}
                                        >
                                            <span className={styles.cover}>
                                                <img src={toAnimeCdnUrl(`/data/anime/${id}.jpg`)} alt="" loading="lazy" decoding="async" />
                                                <span className={styles.check} aria-hidden>✓</span>
                                                {theirs.has(id) && <span className={styles.theirs}>TA 看过</span>}
                                            </span>
                                            <span className={styles.name}>{displayName(r)}</span>
                                            <span className={styles.year}>{year ?? "未知年份"}</span>
                                        </button>
                                    </li>
                                );
                            })}
                        </ul>
                    </section>
                ))
            )}
            </div>
            </div>

        </div>
    );
}
