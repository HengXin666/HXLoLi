import React from "react";
import {
    activeConditions,
    AnimeFilters,
    clearConditions,
    FacetCounts,
    SCORE_OPTIONS,
    SORT_OPTIONS,
    SortKey,
    STATUS_OPTIONS,
} from "@site/src/utils/anime/filters";
import type { AnimeView } from "@site/src/components/anime/AnimeCard/PosterCard";
import Select from "@site/src/components/common/Tooltip/Select";
import { TagFilter } from "./TagFilter";
import styles from "./FilterBar.module.css";

/** 排序下拉左侧的图标, 跟着当前排序方式换: 最近更新 = 时钟, 首播日期 = 日历, 评分 = 星, 名称 = 字母序 */
const SORT_ICONS: Record<SortKey, React.ReactElement> = {
    update: (
        <svg viewBox="0 0 24 24" width="15" height="15" aria-hidden>
            <path fill="currentColor" d="M12 2a10 10 0 1 1 0 20 10 10 0 0 1 0-20m0 2a8 8 0 1 0 0 16 8 8 0 0 0 0-16m1 3v4.6l3.4 2-1 1.7L11 12.7V7z" />
        </svg>
    ),
    air: (
        <svg viewBox="0 0 24 24" width="15" height="15" aria-hidden>
            <path fill="currentColor" d="M7 2h2v2h6V2h2v2h2a2 2 0 0 1 2 2v13a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V6a2 2 0 0 1 2-2h2zM5 10v9h14v-9zm2 2h4v4H7z" />
        </svg>
    ),
    score: (
        <svg viewBox="0 0 24 24" width="15" height="15" aria-hidden>
            <path fill="currentColor" d="m12 2.5 2.9 6 6.6.9-4.8 4.6 1.2 6.5L12 17.4l-5.9 3.1 1.2-6.5L2.5 9.4l6.6-.9z" />
        </svg>
    ),
    name: (
        <svg viewBox="0 0 24 24" width="15" height="15" aria-hidden>
            <path fill="currentColor" d="M6.2 4h1.6L12 15h-2l-1-2.8H5L4 15H2zm-.5 6.4h2.6L7 6.8zM14 4h7v1.6L16.3 13H21v2h-7v-1.6L18.7 6H14zm2 13h4l-2 3z" />
        </svg>
    ),
};

export interface FilterBarProps {
    filters: AnimeFilters;
    onChange: (next: AnimeFilters) => void;
    counts: FacetCounts;
    /** 当前结果数 / 总数 */
    shown: number;
    total: number;
    view: AnimeView;
    onViewChange: (view: AnimeView) => void;
    /** 给了就在工具区最右放"分享"按钮 (番剧库用它打开分享番单弹层) */
    onShare?: () => void;
    /** 「系列」视图下命中的系列数 (单部视图不传) */
    seriesShown?: number;
}

/**
 * 番剧页工具栏: 一行放常用维度 (搜索、观看状态多选、排序、视图), 标签 / 年份 / 评分收进"筛选"面板
 * (原生 <dialog>: 桌面是居中面板, 窄屏是底部抽屉, Esc 与点遮罩关闭). 下面一行列出所有生效条件, 每个可单独去掉, 外加"清空".
 * 按 "/" 聚焦搜索. 所有改动立刻回调, 状态由番剧页写进 URL.
 * 排序 / 首播年份 / 评分三个下拉用 common/Tooltip/Select (复用 Tooltip 的定位层), 不再用原生 select 元素.
 * 下拉组件用的是 src/components/common/Tooltip/Select.tsx; 排序下拉带随排序方式变化的 svg 图标 (SORT_ICONS).
 * .agents/notes/implemented/feature/2026-11-01-anime-page-refactor.md
 * 分享按钮: .agents/notes/implemented/feature/2026-11-01-anime-share-on-index.md
 * 「单部 | 系列」切换写 ?fold= (与排序正交): .agents/notes/implemented/feature/2026-11-05-anime-series-fold.md
 */
export const FilterBar: React.FC<FilterBarProps> = ({ filters, onChange, counts, shown, total, view, onViewChange, onShare, seriesShown }) => {
    const searchRef = React.useRef<HTMLInputElement>(null);
    const dialogRef = React.useRef<HTMLDialogElement>(null);
    const [q, setQ] = React.useState(filters.q);

    // URL 被外部改了 (后退 / 点 chip 清掉搜索) 时同步输入框
    React.useEffect(() => { setQ(filters.q); }, [filters.q]);

    // 输入防抖 200ms 再写 URL, 免得每个按键一条 history
    React.useEffect(() => {
        if (q.trim() === filters.q) return;
        const id = window.setTimeout(() => onChange({ ...filters, q: q.trim() }), 200);
        return () => window.clearTimeout(id);
    }, [q]); // eslint-disable-line react-hooks/exhaustive-deps

    React.useEffect(() => {
        const onKey = (e: KeyboardEvent) => {
            if (e.key !== "/" || e.metaKey || e.ctrlKey || e.altKey) return;
            const el = e.target as HTMLElement | null;
            if (el && (el.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(el.tagName))) return;
            e.preventDefault();
            searchRef.current?.focus();
        };
        document.addEventListener("keydown", onKey);
        return () => document.removeEventListener("keydown", onKey);
    }, []);

    const toggleStatus = (s: (typeof STATUS_OPTIONS)[number]["value"]) =>
        onChange({ ...filters, status: filters.status.includes(s) ? filters.status.filter((x) => x !== s) : [...filters.status, s] });

    const conditions = activeConditions(filters);
    const panelCount = filters.tags.length + (filters.year !== null ? 1 : 0) + (filters.minScore !== null ? 1 : 0);

    const openPanel = () => dialogRef.current?.showModal();
    const closePanel = () => dialogRef.current?.close();

    return (
        <div className={styles.bar}>
            <div className={styles.row}>
                <label className={styles.search}>
                    <span className={styles.srOnly}>搜索番剧</span>
                    <svg viewBox="0 0 24 24" width="16" height="16" aria-hidden><path fill="currentColor" d="M10 4a6 6 0 1 1 0 12 6 6 0 0 1 0-12m0-2a8 8 0 1 0 4.9 14.3l5.4 5.4 1.4-1.4-5.4-5.4A8 8 0 0 0 10 2" /></svg>
                    <input
                        ref={searchRef}
                        type="search"
                        value={q}
                        placeholder="搜索中文名 / 原名 / 短评"
                        onChange={(e) => setQ(e.target.value)}
                        onKeyDown={(e) => { if (e.key === "Escape") { setQ(""); onChange({ ...filters, q: "" }); } }}
                    />
                    <kbd className={styles.kbd} aria-hidden>/</kbd>
                </label>

                <div className={styles.statusGroup} role="group" aria-label="观看状态 (可多选)">
                    {STATUS_OPTIONS.map((o) => {
                        const on = filters.status.includes(o.value);
                        const n = counts.status.get(o.value) ?? 0;
                        return (
                            <button
                                key={o.value}
                                type="button"
                                className={styles.chip}
                                aria-pressed={on}
                                disabled={!on && n === 0}
                                onClick={() => toggleStatus(o.value)}
                            >
                                {o.label}<span className={styles.count}>{n}</span>
                            </button>
                        );
                    })}
                </div>

                <div className={styles.tools}>
                    <button type="button" className={styles.panelBtn} onClick={openPanel} aria-haspopup="dialog">
                        <svg viewBox="0 0 24 24" width="15" height="15" aria-hidden><path fill="currentColor" d="M3 5h18v2H3zm4 6h10v2H7zm3 6h4v2h-4z" /></svg>
                        标签与更多{panelCount > 0 && <span className={styles.badge}>{panelCount}</span>}
                    </button>
                    <Select<SortKey>
                        ariaLabel="排序"
                        icon={SORT_ICONS[filters.sort]}
                        value={filters.sort}
                        options={SORT_OPTIONS.map((o) => ({ value: o.value, label: `按${o.label}` }))}
                        onChange={(sort) => onChange({ ...filters, sort })}
                    />
                    <div className={styles.segment} role="radiogroup" aria-label="卡片单位">
                        <button type="button" role="radio" aria-checked={filters.fold === "none"} onClick={() => onChange({ ...filters, fold: "none" })} title="一部一张卡">
                            单部
                        </button>
                        <button type="button" role="radio" aria-checked={filters.fold === "series"} onClick={() => onChange({ ...filters, fold: "series" })} title="同一番剧的各季聚成一张卡">
                            系列
                        </button>
                    </div>
                    <div className={styles.segment} role="radiogroup" aria-label="显示方式">
                        <button type="button" role="radio" aria-checked={view === "grid"} onClick={() => onViewChange("grid")} title="海报">
                            <svg viewBox="0 0 24 24" width="15" height="15" aria-hidden><path fill="currentColor" d="M3 3h8v8H3zm10 0h8v8h-8zM3 13h8v8H3zm10 0h8v8h-8z" /></svg>
                            <span className={styles.srOnly}>海报</span>
                        </button>
                        <button type="button" role="radio" aria-checked={view === "list"} onClick={() => onViewChange("list")} title="列表">
                            <svg viewBox="0 0 24 24" width="15" height="15" aria-hidden><path fill="currentColor" d="M3 4h18v3H3zm0 6.5h18v3H3zM3 17h18v3H3z" /></svg>
                            <span className={styles.srOnly}>列表</span>
                        </button>
                    </div>
                    {onShare && (
                        <button type="button" className={styles.panelBtn} onClick={onShare} aria-haspopup="dialog">
                            <svg viewBox="0 0 24 24" width="15" height="15" aria-hidden><path fill="currentColor" d="M18 16a3 3 0 0 0-2.2 1l-7-4a3 3 0 0 0 0-2l7-4A3 3 0 1 0 15 5l-7 4a3 3 0 1 0 0 6l7 4a3 3 0 1 0 3-3" /></svg>
                            分享
                        </button>
                    )}
                </div>
            </div>

            <div className={styles.activeRow} aria-live="polite">
                <span className={styles.resultCount}>
                    {seriesShown !== undefined && <><b>{seriesShown}</b> 个系列 · </>}<b>{shown}</b> / {total} 部
                </span>
                {conditions.length > 0 && (
                    <>
                        <ul className={styles.activeList} aria-label="生效中的条件">
                            {conditions.map((c, i) => {
                                const prev = conditions[i - 1];
                                // 标签之间写出组合方式, 不让人猜是"且"还是"或"
                                const joiner = prev && prev.key.startsWith("tag-") && c.key.startsWith("tag-")
                                    ? (filters.match === "all" ? "且" : "或")
                                    : null;
                                return (
                                    <li key={c.key}>
                                        {joiner && <span className={styles.joiner}>{joiner}</span>}
                                        <button type="button" className={styles.activeChip} onClick={() => onChange(c.without)} title={`去掉 ${c.label}`}>
                                            {c.label}<span aria-hidden>×</span>
                                        </button>
                                    </li>
                                );
                            })}
                        </ul>
                        <button type="button" className={styles.clear} onClick={() => onChange(clearConditions(filters))}>
                            清空全部
                        </button>
                    </>
                )}
            </div>

            <dialog
                ref={dialogRef}
                className={styles.dialog}
                aria-label="标签与更多筛选"
                onClick={(e) => { if (e.target === e.currentTarget) closePanel(); }}
            >
                <div className={styles.dialogInner}>
                    <header className={styles.dialogHead}>
                        <h2>标签与更多筛选</h2>
                        <button type="button" className={styles.iconBtn} onClick={closePanel} aria-label="关闭">×</button>
                    </header>

                    <div className={styles.dialogBody}>
                        <TagFilter
                            value={filters.tags}
                            match={filters.match}
                            counts={counts.tags}
                            onChange={(tags) => onChange({ ...filters, tags })}
                            onMatchChange={(match) => onChange({ ...filters, match })}
                        />

                        <div className={styles.moreRow}>
                            <div className={styles.field}>
                                <span id="anime-filter-year">首播年份</span>
                                <Select<string>
                                    ariaLabelledBy="anime-filter-year"
                                    value={filters.year === null ? "" : String(filters.year)}
                                    options={[
                                        { value: "", label: "不限" },
                                        ...counts.years.map((y) => ({ value: String(y.year), label: String(y.year), hint: `${y.count} 部` })),
                                    ]}
                                    onChange={(v) => onChange({ ...filters, year: v ? Number(v) : null })}
                                />
                            </div>
                            <div className={styles.field}>
                                <span id="anime-filter-score">Bangumi 评分</span>
                                <Select<string>
                                    ariaLabelledBy="anime-filter-score"
                                    value={filters.minScore === null ? "" : String(filters.minScore)}
                                    options={[
                                        { value: "", label: "不限" },
                                        ...SCORE_OPTIONS.map((s) => ({ value: String(s), label: `≥ ${s}` })),
                                    ]}
                                    onChange={(v) => onChange({ ...filters, minScore: v ? Number(v) : null })}
                                />
                            </div>
                        </div>
                    </div>

                    <footer className={styles.dialogFoot}>
                        <button
                            type="button"
                            className={styles.linkBtn}
                            disabled={panelCount === 0}
                            onClick={() => onChange({ ...filters, tags: [], match: "all", year: null, minScore: null })}
                        >
                            重置面板
                        </button>
                        <button type="button" className={styles.primaryBtn} onClick={closePanel}>
                            显示 {shown} 部
                        </button>
                    </footer>
                </div>
            </dialog>
        </div>
    );
};
