import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import { useHistory, useLocation } from "@docusaurus/router";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { useBaseUrlUtils } from "@docusaurus/useBaseUrl";
import PosterCard, { AnimeView } from "@site/src/components/anime/AnimeCard/PosterCard";
import SeriesCard from "@site/src/components/anime/AnimeCard/SeriesCard";
import { groupSeries, seriesHits, useAnimeSeries } from "@site/src/utils/anime/series";
import { FilterBar } from "@site/src/components/anime/Tag/FilterBar";
import AnimeShareBoard from "@site/src/components/anime/AnimeShareBoard";
import SusumeEntry from "@site/src/components/anime/susume/SusumeEntry";
import { useAnimeLabelsStatus, useAnimeRecordsState } from "@site/src/utils/anime/animeStore";
import { ANIME_TAG_QUERY, susumeDefOfTag, susumePath } from "@site/src/utils/anime/tags";
import {
    activeConditions,
    AnimeFilters,
    applyFilters,
    clearConditions,
    facetCounts,
    groupRecords,
    parseFilters,
    serializeFilters,
} from "@site/src/utils/anime/filters";
import { WatchStatus } from "@site/src/utils/anime/types";
import { usePersistent } from "@site/src/utils/hook/usePersistent";
import { runListTransition, useListFlip } from "@site/src/utils/anime/listMotion";
import BlogWithCats from "@site/src/components/BlogWithCats";
import styles from "./AnimeIndexPage.module.css";

const SKELETON_COUNT = 12;

/**
 * 旧式链接 /anime?tag=物凄い (以及 ?tag=ロリ / シスコン / 萝莉 / 妹控, 多选里带上任一个也算) -> 对应 /anime/susume/<slug>.
 * 推荐分类只有专属页一个入口, 不再作为 ?tag= 筛选 (hx-anime-tag-refactor-design.md 第 3.1 节).
 * 站点是静态构建, 查询串上的跳转做不了服务端 301, 这里用客户端 replace (不留历史, 后退不会再弹回来).
 */
function legacySusumeTarget (search: string): string | null {
    const raw = new URLSearchParams(search).get(ANIME_TAG_QUERY) ?? "";
    for (const t of raw.split(",")) {
        const def = susumeDefOfTag(t.trim());
        if (def) return susumePath(def);
    }
    return null;
}

/**
 * 番剧库. 筛选状态全部在 URL 上 (见 utils/anime/filters.ts), 默认显示全部状态、按最近更新排序;
 * 只有视图密度 (海报 / 列表) 存本地. 点选类改动 push 一条历史 (后退能撤回上一步筛选), 搜索输入用 replace.
 * 加载中显示骨架, 拉取失败显示错误与重试, 筛空时给出"去掉哪个条件能看到几部"的一键撤销.
 * .agents/notes/implemented/feature/2026-11-01-anime-page-refactor.md
 * 分享番单: 工具栏最右"分享"按钮打开 <dialog>, 里面挂与 /anime/share 同一个 AnimeShareBoard (embedded): 读当前 URL 筛选作番单,
 * 能切卡片 / 表格、勾选、生成链接、生成并预览分享图; 弹窗里不改地址栏. 只在打开时挂载, 关着不画图.
 * 真模态: showModal 让页面其余部分 inert (焦点留在弹窗内); 点遮罩与 Esc 都不关 (onCancel 拦掉), 只能点右上角 ✕ (弹窗自身不滚, 内容在 .shareBody 里滚, 按钮不会被盖住).
 * .agents/notes/implemented/feature/2026-11-01-anime-share-on-index.md
 * 2026-11-04 tag 重构: 顶部三张推荐分类大入口卡 (物凄い / ロリ / シスコン, 封面拼贴 + 数量, 进 /anime/susume/<slug>);
 * 标签只读人工标注 data/labels (utils/anime/tags.ts), 普通九项标注仍走 ?tag= 筛选; 旧式 ?tag=物凄い 等跳到专属页 (见 legacySusumeTarget).
 * 「系列」视图 (?fold=series): 筛选仍按单季, 命中的季按系列聚成一张 SeriesCard, 分组 / 排序用 utils/anime/series.ts 的 groupSeries.
 * 2026-11-06 第三版: 页头是标题 + 一行等宽副标 + 下划线式子导航 (全部 / おすすめ 三类 / キャラ / 关系图), 去掉胶囊按钮、渐变字与光晕; 状态计数只留在工具栏里.
 * /anime/kyara 路由由 plugins/kyara-gallery-plugin.mjs 注册.
 */
export default function AnimeIndexPage (): React.ReactNode {
    const { siteConfig } = useDocusaurusContext();
    const { records, status, error, retry } = useAnimeRecordsState(siteConfig.baseUrl);
    const location = useLocation();
    const history = useHistory();
    const labelsStatus = useAnimeLabelsStatus();
    const { withBaseUrl } = useBaseUrlUtils();

    const legacyTarget = legacySusumeTarget(location.search);
    React.useEffect(() => {
        if (legacyTarget) history.replace(withBaseUrl(legacyTarget));
    }, [legacyTarget, history, withBaseUrl]);

    const filters = React.useMemo(() => parseFilters(location.search), [location.search]);
    const [view, setView] = usePersistent<AnimeView>(`${siteConfig.baseUrl}/anime/:view`, "grid");

    const setFilters = React.useCallback((next: AnimeFilters) => {
        const search = serializeFilters(next);
        const to = { ...history.location, search: search ? `?${search}` : "" };
        if (to.search === history.location.search) return;
        const onlyQuery = serializeFilters({ ...next, q: filters.q }) === serializeFilters(filters);
        // 筛选切换的过渡 (FLIP 与 View Transitions 的取舍见 utils/anime/listMotion.ts)
        runListTransition(() => {
            if (onlyQuery) history.replace(to);
            else history.push(to);
        });
    }, [history, filters]);

    const filtered = React.useMemo(() => applyFilters(records, filters), [records, filters]);
    const counts = React.useMemo(() => facetCounts(records, filters), [records, filters]);
    const groups = React.useMemo(() => groupRecords(filtered, filters.sort), [filtered, filters.sort]);
    const { byId: seriesById } = useAnimeSeries(records);
    const seriesGroups = React.useMemo(
        () => (filters.fold === "series" ? groupSeries(seriesHits(filtered, seriesById), filters.sort) : []),
        [filters.fold, filters.sort, filtered, seriesById],
    );
    const seriesShown = React.useMemo(
        () => (filters.fold === "series" ? seriesGroups.reduce((n, g) => n + g.items.length, 0) : undefined),
        [filters.fold, seriesGroups],
    );

    // 不走 View Transition 的变化 (不支持的浏览器、后退 / 前进) 用 FLIP 兜底
    const listRef = React.useRef<HTMLDivElement | null>(null);
    useListFlip(listRef, filters.fold === "series" ? seriesGroups : groups);

    const toggleTag = React.useCallback((tag: string) => {
        setFilters({
            ...filters,
            tags: filters.tags.includes(tag) ? filters.tags.filter((t) => t !== tag) : [...filters.tags, tag],
        });
    }, [filters, setFilters]);

    const statusTotals = React.useMemo(() => {
        const m = new Map<number, number>();
        for (const r of records) m.set(r.user_status.watch_status, (m.get(r.user_status.watch_status) ?? 0) + 1);
        return m;
    }, [records]);

    const shareRef = React.useRef<HTMLDialogElement | null>(null);
    const [shareOpen, setShareOpen] = React.useState(false);
    const openShare = React.useCallback(() => {
        setShareOpen(true);
        shareRef.current?.showModal();
    }, []);
    const closeShare = React.useCallback(() => shareRef.current?.close(), []);

    const listClass = view === "grid" ? styles.grid : styles.list;

    let content: React.ReactNode;
    if (status === "loading" && records.length === 0) {
        content = (
            <ul className={styles.grid} aria-busy="true" aria-label="正在加载追番记录" style={{ marginTop: 28 }}>
                {Array.from({ length: SKELETON_COUNT }, (_, i) => (
                    <li key={i} className={styles.skeletonTile}>
                        <div className={styles.skeletonCover} />
                        <div className={styles.skeletonLine} />
                        <div className={styles.skeletonLine} />
                    </li>
                ))}
            </ul>
        );
    } else if (status === "error" && records.length === 0) {
        content = (
            <div className={styles.notice} role="alert">
                <h2>追番记录没能加载出来</h2>
                <p>数据从 HXLoLi-ANiMe 的 CDN 拉取 (约 2.4 MB gzip), 可能是网络或 CDN 节点问题{error ? `: ${error}` : ""}.</p>
                <div className={styles.noticeActions}>
                    <button type="button" className={styles.suggest} onClick={retry}>重试</button>
                </div>
            </div>
        );
    } else if (filtered.length === 0) {
        // 逐个去掉一个条件, 看哪个能救回最多结果
        const fixes = activeConditions(filters)
            .map((c) => ({ c, n: applyFilters(records, c.without).length }))
            .filter((x) => x.n > 0)
            .sort((a, b) => b.n - a.n)
            .slice(0, 3);
        content = (
            <div className={styles.notice}>
                <h2>没有同时满足这些条件的番</h2>
                <p>{fixes.length ? "去掉其中一个条件试试:" : "换个条件试试."}</p>
                <div className={styles.noticeActions}>
                    {fixes.map(({ c, n }) => (
                        <button key={c.key} type="button" className={styles.suggest} onClick={() => setFilters(c.without)}>
                            去掉 {c.label}, 有 {n} 部
                        </button>
                    ))}
                    <button type="button" className={styles.suggest} onClick={() => setFilters(clearConditions(filters))}>
                        清空全部条件
                    </button>
                </div>
            </div>
        );
    } else if (filters.fold === "series") {
        content = seriesGroups.map((g) => (
            <section key={g.key} className={styles.group} aria-labelledby={g.label ? `g-${g.key}` : undefined}>
                {g.label && (
                    <h2 id={`g-${g.key}`} className={styles.groupTitle}>
                        {g.label}<span className={styles.groupCount}>{g.items.length} 个系列</span>
                    </h2>
                )}
                <ul className={listClass}>
                    {g.items.map((h) => (
                        <SeriesCard
                            key={h.series.key}
                            hit={h}
                            view={view}
                            activeTags={filters.tags}
                            onTagClick={toggleTag}
                        />
                    ))}
                </ul>
            </section>
        ));
    } else {
        content = groups.map((g) => (
            <section key={g.key} className={styles.group} aria-labelledby={g.label ? `g-${g.key}` : undefined}>
                {g.label && (
                    <h2 id={`g-${g.key}`} className={styles.groupTitle}>
                        {g.label}<span className={styles.groupCount}>{g.records.length} 部</span>
                    </h2>
                )}
                <ul className={listClass}>
                    {g.records.map((r) => (
                        <PosterCard
                            key={r.anime_data.id}
                            record={r}
                            view={view}
                            activeTags={filters.tags}
                            onTagClick={toggleTag}
                        />
                    ))}
                </ul>
            </section>
        ));
    }

    return (
        <Layout title="アニメ" description="HXLoLi 的追番记录: 按观看状态、标签、年份、评分组合筛选">
            {/* 两侧立绘藏在正文墙后, 与系列页 / 博客文章页同一套 (墙宽 1200) */}
            <BlogWithCats contentWidth={1200}>
            <main className={styles.page}>
                <header className={styles.header}>
                    <h1 className={styles.title}>追番记录</h1>
                    <p className={styles.titleSub}>
                        BanGuMi 收藏{records.length > 0 && <> · <b>{records.length}</b> 部 · 看过 <b>{statusTotals.get(WatchStatus.WATCHED) ?? 0}</b> · 在看 <b>{statusTotals.get(WatchStatus.WATCHING) ?? 0}</b></>}
                    </p>
                </header>
                <nav className={styles.subnav} aria-label="番剧页面">
                    <Link to="/anime" aria-current="page">全部</Link>
                    <span className={styles.subnavSep} aria-hidden />
                    <SusumeEntry records={records} labelsStatus={labelsStatus} countClass={styles.subnavCount} />
                    <span className={styles.subnavSep} aria-hidden />
                    <Link to="/anime/kyara">キャラ</Link>
                    <Link to="/anime/graph" className={styles.subnavEnd}>关系图 →</Link>
                </nav>

                <FilterBar
                    filters={filters}
                    onChange={setFilters}
                    counts={counts}
                    shown={filtered.length}
                    total={records.length}
                    view={view}
                    onViewChange={setView}
                    onShare={openShare}
                    seriesShown={seriesShown}
                />

                <dialog
                    ref={shareRef}
                    className={styles.shareDialog}
                    aria-label="分享番单"
                    onClose={() => setShareOpen(false)}
                    onCancel={(e) => e.preventDefault()}
                >
                    <button type="button" className={styles.shareClose} onClick={closeShare} aria-label="关闭分享弹窗" title="关闭">✕</button>
                    <div className={styles.shareBody}>{shareOpen && <AnimeShareBoard embedded />}</div>
                </dialog>

                <div ref={listRef} className={styles.results}>{content}</div>
            </main>
            </BlogWithCats>
        </Layout>
    );
}
