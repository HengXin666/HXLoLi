import React from "react";
import Link from "@docusaurus/Link";
import { type SusumeList, type SusumePick } from "./useSusume";
import styles from "./susume.module.css";

const cx = (...xs: (string | false | null | undefined)[]): string => xs.filter(Boolean).join(" ");

/** 一行等宽元信息: 我的评分 · Bangumi · 年份 · 话数 */
export function PickMeta ({ p, long }: { p: SusumePick; long?: boolean }): React.ReactElement {
    const bits: React.ReactNode[] = [];
    if (p.rate) bits.push(<span key="r" className={styles.metaRate} title="我的评分">♥ {p.rate}</span>);
    if (p.score > 0) bits.push(<span key="b" title="Bangumi 评分">bgm {p.score.toFixed(1)}</span>);
    if (p.year) bits.push(<span key="y">{p.year}</span>);
    if (long && p.eps > 0) bits.push(<span key="e">{p.eps} 话</span>);
    if (long) bits.push(<span key="s">{p.status}</span>);
    return <span className={styles.meta}>{bits.flatMap((b, i) => (i ? [<i key={`d${i}`} aria-hidden>·</i>, b] : [b]))}</span>;
}

/** 三个推荐分类的切换 (首页「最近更新」那种分段按钮, 带数量) */
export function CategoryTabs ({ lists, current, loading }: { lists: SusumeList[]; current?: string; loading: boolean }): React.ReactElement {
    return (
        <nav className={styles.tabs} aria-label="推荐分类">
            {lists.map((l) => {
                const on = l.def.key === current;
                return (
                    <Link
                        key={l.def.key}
                        to={l.path}
                        className={cx(styles.tab, on && styles.tabOn)}
                        aria-current={on ? "page" : undefined}
                    >
                        {l.def.display}
                        <span className={styles.tabCount}>{loading ? "…" : l.picks.length}</span>
                    </Link>
                );
            })}
        </nav>
    );
}

/** 一排封面 (首页「在追的番」的封面行) */
export function CoverRow ({ picks, max = 6 }: { picks: SusumePick[]; max?: number }): React.ReactElement {
    return (
        <ul className={styles.covers}>
            {picks.slice(0, max).map((p) => (
                <li key={p.id}>
                    <Link to={p.href} className={styles.cover}>
                        <img src={p.cover} alt={p.title} loading="lazy" decoding="async" width={120} height={170} />
                        <span className={styles.coverName}>{p.title}</span>
                        <PickMeta p={p} />
                    </Link>
                </li>
            ))}
        </ul>
    );
}

/** 还没有标注时: 一块细线面板, 说清楚怎么上榜 */
export function EmptyBoard ({ list }: { list: SusumeList }): React.ReactElement {
    return (
        <div className={styles.panel} role="status">
            <p className={styles.panelTitle}>「{list.def.display}」还没有番</p>
            <p className={styles.panelText}>
                这里只放亲手标注过的番. 在 HXLoLi-ANiMe 的标注里给番勾上「{list.def.display}」并打分, 合入后按评分出现在这里.
            </p>
        </div>
    );
}

/** 加载失败: 一行说明 + 重试 */
export function ErrorLine ({ text, onRetry }: { text: string; onRetry: () => void }): React.ReactElement {
    return (
        <div className={styles.error} role="alert">
            <span>{text}</span>
            <button type="button" className={styles.textBtn} onClick={onRetry}>重试</button>
        </div>
    );
}
