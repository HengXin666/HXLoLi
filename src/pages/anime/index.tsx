import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import { useHistory, useLocation } from "@docusaurus/router";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import PosterCard, { AnimeView } from "@site/src/components/anime/AnimeCard/PosterCard";
import { FilterBar } from "@site/src/components/anime/Tag/FilterBar";
import AnimeShareBoard from "@site/src/components/anime/AnimeShareBoard";
import { useAnimeRecordsState } from "@site/src/utils/anime/animeStore";
import {
    activeConditions,
    AnimeFilters,
    applyFilters,
    clearConditions,
    facetCounts,
    groupRecords,
    parseFilters,
    serializeFilters,
    STATUS_OPTIONS,
} from "@site/src/utils/anime/filters";
import { usePersistent } from "@site/src/utils/hook/usePersistent";
import { runListTransition, useListFlip } from "@site/src/utils/anime/listMotion";
import styles from "./AnimeIndexPage.module.css";

const SKELETON_COUNT = 12;

/**
 * 番剧库. 筛选状态全部在 URL 上 (见 utils/anime/filters.ts), 默认显示全部状态、按最近更新排序;
 * 只有视图密度 (海报 / 列表) 存本地. 点选类改动 push 一条历史 (后退能撤回上一步筛选), 搜索输入用 replace.
 * 加载中显示骨架, 拉取失败显示错误与重试, 筛空时给出"去掉哪个条件能看到几部"的一键撤销.
 * .agents/notes/implemented/feature/2026-11-01-anime-page-refactor.md
 * 分享番单: 工具栏最右"分享"按钮打开 <dialog>, 里面挂与 /anime/share 同一个 AnimeShareBoard (embedded): 读当前 URL 筛选作番单,
 * 能切卡片 / 表格、勾选、生成链接、生成并预览分享图; 弹窗里不改地址栏. 只在打开时挂载, 关着不画图.
 * .agents/notes/implemented/feature/2026-11-01-anime-share-on-index.md
 */
export default function AnimeIndexPage (): React.ReactNode {
    const { siteConfig } = useDocusaurusContext();
    const { records, status, error, retry } = useAnimeRecordsState(siteConfig.baseUrl);
    const location = useLocation();
    const history = useHistory();

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

    // 不走 View Transition 的变化 (不支持的浏览器、后退 / 前进) 用 FLIP 兜底
    const listRef = React.useRef<HTMLDivElement | null>(null);
    useListFlip(listRef, groups);

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
            <main className={styles.page}>
                <header className={styles.header}>
                    <h1 className={styles.title}>追番记录</h1>
                    <div className={styles.links}>
                        <Link to="/anime/graph">番剧关系图 →</Link>
                    </div>
                </header>
                {records.length > 0 && (
                    <ul className={styles.stats} aria-label="各状态部数">
                        <li>共 <b>{records.length}</b> 部</li>
                        {STATUS_OPTIONS.map((o) => (
                            <li key={o.value}>{o.label} <b>{statusTotals.get(o.value) ?? 0}</b></li>
                        ))}
                    </ul>
                )}

                <FilterBar
                    filters={filters}
                    onChange={setFilters}
                    counts={counts}
                    shown={filtered.length}
                    total={records.length}
                    view={view}
                    onViewChange={setView}
                    onShare={openShare}
                />

                <dialog
                    ref={shareRef}
                    className={styles.shareDialog}
                    aria-label="分享番单"
                    onClose={() => setShareOpen(false)}
                    onClick={(e) => { if (e.target === e.currentTarget) closeShare(); }}
                >
                    <button type="button" className={styles.shareClose} onClick={closeShare} aria-label="关闭">×</button>
                    {shareOpen && <AnimeShareBoard embedded />}
                </dialog>

                <div ref={listRef} className={styles.results}>{content}</div>
            </main>
        </Layout>
    );
}
