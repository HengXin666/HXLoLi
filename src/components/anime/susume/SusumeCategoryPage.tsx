import React from "react";
import Layout from "@theme/Layout";
import { useHistory, useLocation } from "@docusaurus/router";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import type { AnimeSusumeKey } from "@site/data/animeTags";
import PosterCard, { type AnimeView } from "@site/src/components/anime/AnimeCard/PosterCard";
import SeriesCard from "@site/src/components/anime/AnimeCard/SeriesCard";
import { FilterBar } from "@site/src/components/anime/Tag/FilterBar";
import { StagePaper } from "@site/src/components/anime/stage/Stage";
import { groupSeries, seriesHits, useAnimeSeries } from "@site/src/utils/anime/series";
import {
    activeConditions,
    applyFilters,
    clearConditions,
    facetCounts,
    groupRecords,
    type AnimeFilters,
} from "@site/src/utils/anime/filters";
import { runListTransition, useListFlip } from "@site/src/utils/anime/listMotion";
import { usePersistent } from "@site/src/utils/hook/usePersistent";
import { listOf, parseSusumeFilters, serializeSusumeFilters, useSusumeData } from "./useSusume";
import { AnimeSubnav, GridSkeleton } from "./parts";
import styles from "@site/src/pages/anime/AnimeIndexPage.module.css";

/**
 * 推荐分类页 /anime/susume/<slug>: 就是番剧库 (/anime) 限定在这一类里, 交互和动效全部照搬番剧库:
 * 同一个页头 + 子导航、同一条 FilterBar (搜索 / 状态 / 排序 / 单部·系列 / 海报·列表 / 筛选面板), 同一张 PosterCard / SeriesCard,
 * 筛选切换走 listMotion 的 View Transitions, 不支持时 FLIP 兜底. 视图密度与番剧库共用一个本地键.
 * 和番剧库唯一的不同: 默认排序是「名称」, 一面 A-Z 墙连着排, 不分组、不分字母段.
 * 画面上不放名次、观后感说明、旧名提示. 没有分享按钮.
 * .agents/notes/implemented/feature/2026-11-08-anime-susume-wall.md
 */
export default function SusumeCategoryPage ({ susume }: { susume: AnimeSusumeKey }): React.ReactElement {
    const { siteConfig } = useDocusaurusContext();
    const data = useSusumeData();
    const { def, records } = listOf(data, susume);
    const location = useLocation();
    const history = useHistory();

    const filters = React.useMemo(() => parseSusumeFilters(location.search), [location.search]);
    const [view, setView] = usePersistent<AnimeView>(`${siteConfig.baseUrl}/anime/:view`, "grid");

    const setFilters = React.useCallback((next: AnimeFilters) => {
        const search = serializeSusumeFilters(next);
        const to = { ...history.location, search: search ? `?${search}` : "" };
        if (to.search === history.location.search) return;
        const onlyQuery = serializeSusumeFilters({ ...next, q: filters.q }) === serializeSusumeFilters(filters);
        runListTransition(() => {
            if (onlyQuery) history.replace(to);
            else history.push(to);
        });
    }, [history, filters]);

    const filtered = React.useMemo(() => applyFilters(records, filters), [records, filters]);
    const counts = React.useMemo(() => facetCounts(records, filters), [records, filters]);
    const groups = React.useMemo(() => groupRecords(filtered, filters.sort), [filtered, filters.sort]);
    const { byId: seriesById } = useAnimeSeries(data.all);
    const seriesGroups = React.useMemo(
        () => (filters.fold === "series" ? groupSeries(seriesHits(filtered, seriesById), filters.sort) : []),
        [filters.fold, filters.sort, filtered, seriesById],
    );
    const seriesShown = filters.fold === "series" ? seriesGroups.reduce((n, g) => n + g.items.length, 0) : undefined;

    const listRef = React.useRef<HTMLDivElement | null>(null);
    useListFlip(listRef, filters.fold === "series" ? seriesGroups : groups);

    const toggleTag = React.useCallback((tag: string) => {
        setFilters({
            ...filters,
            tags: filters.tags.includes(tag) ? filters.tags.filter((t) => t !== tag) : [...filters.tags, tag],
        });
    }, [filters, setFilters]);

    const listClass = view === "grid" ? styles.grid : styles.list;

    let content: React.ReactNode;
    if (data.loading) {
        content = <GridSkeleton />;
    } else if (data.error && records.length === 0) {
        content = (
            <div className={styles.notice} role="alert">
                <h2>{data.error}</h2>
                <div className={styles.noticeActions}>
                    <button type="button" className={styles.suggest} onClick={data.retry}>重试</button>
                </div>
            </div>
        );
    } else if (records.length === 0) {
        content = <div className={styles.notice}><h2>还没有番</h2></div>;
    } else if (filtered.length === 0) {
        const fixes = activeConditions(filters)
            .map((c) => ({ c, n: applyFilters(records, c.without).length }))
            .filter((x) => x.n > 0)
            .sort((a, b) => b.n - a.n)
            .slice(0, 3);
        content = (
            <div className={styles.notice}>
                <h2>没有同时满足这些条件的番</h2>
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
                        <SeriesCard key={h.series.key} hit={h} view={view} activeTags={filters.tags} onTagClick={toggleTag} />
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
                        <PosterCard key={r.anime_data.id} record={r} view={view} activeTags={filters.tags} onTagClick={toggleTag} />
                    ))}
                </ul>
            </section>
        ));
    }

    return (
        <Layout title={`${def.display} · おすすめ`} description={`HXLoLi 的推荐: ${def.display}`}>
            {/* 墙宽 1200 + 两侧立绘, 与番剧库 / 系列页同一套 (StagePaper 展开 BlogWithCats 的 DOM, 不用其默认导出) */}
            <StagePaper className={styles.page}>
                <header className={styles.header}>
                    <h1 className={styles.title}>{def.display}</h1>
                    <p className={styles.titleSub}>
                        おすすめ{!data.loading && <> · <b>{records.length}</b> 部</>}
                    </p>
                </header>
                <AnimeSubnav records={data.all} current={susume} />

                <FilterBar
                    filters={filters}
                    onChange={setFilters}
                    counts={counts}
                    shown={filtered.length}
                    total={records.length}
                    view={view}
                    onViewChange={setView}
                    seriesShown={seriesShown}
                />

                <div ref={listRef} className={styles.results}>{content}</div>
            </StagePaper>
        </Layout>
    );
}
