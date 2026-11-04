import React from "react";
import Link from "@docusaurus/Link";
import { ANiMeRecord, WatchStatus } from "@site/src/utils/anime/types";
import { getRecordTags, susumeDefOfTag, susumePath } from "@site/src/utils/anime/tags";
import { normalEpisodeCount, statusLabel } from "@site/src/utils/anime/filters";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";
import styles from "./PosterCard.module.css";

export type AnimeView = "grid" | "list";

export interface PosterCardProps {
    record: ANiMeRecord;
    view: AnimeView;
    /** 当前已选的标签, 卡片上对应标签高亮 */
    activeTags: readonly string[];
    /** 点卡片上的标签: 加进 (或移出) 当前筛选 */
    onTagClick: (tag: string) => void;
}

const STATUS_CLASS: Record<WatchStatus, string> = {
    [WatchStatus.WATCHING]: styles.sWatching,
    [WatchStatus.WATCHED]: styles.sWatched,
    [WatchStatus.WANT_TO_WATCH]: styles.sWish,
    [WatchStatus.ON_HOLD]: styles.sHold,
    [WatchStatus.DROPPED]: styles.sDropped,
};

function progressOf (record: ANiMeRecord): { ratio: number; text: string } {
    const total = normalEpisodeCount(record);
    const watched = record.user_status.watched_eps || 0;
    const s = record.user_status.watch_status;
    if (s === WatchStatus.WATCHED) return { ratio: 1, text: total ? `全 ${total} 话` : "看完" };
    if (s === WatchStatus.WANT_TO_WATCH) return { ratio: 0, text: total ? `共 ${total} 话` : "未开始" };
    if (!total) return { ratio: 0, text: watched ? `看到第 ${watched} 话` : "未开始" };
    return { ratio: Math.min(1, watched / total), text: `${watched} / ${total} 话` };
}

/**
 * 番剧库的一张卡, 同一份数据两种密度: 海报 (2:3 封面 + 状态角标 / 评分 / 进度条, 悬停或聚焦时浮出短评)
 * 与列表行 (小封面 + 名称 / 进度 / 评分 / 标签 / 短评一行). 整卡不是一个大链接: 封面与标题链到详情,
 * 标签是独立按钮 (点了加进当前筛选), 避免链接里套按钮.
 * 标签只读人工标注 (data/labels, 见 utils/anime/tags.ts); 未标注不显示. 推荐分类 (物凄い / ロリ / シスコン) 渲染成徽章链接,
 * 点了进 /anime/susume/<slug> 专属页, 不加进筛选; 普通标注仍是筛选按钮.
 * data-anime-id 供筛选切换过渡定位卡片: .agents/notes/implemented/feature/2026-11-01-anime-filter-motion.md
 * .agents/notes/implemented/feature/2026-11-01-anime-page-refactor.md
 */
export default function PosterCard ({ record, view, activeTags, onTagClick }: PosterCardProps): React.ReactElement {
    const { anime_data, user_status } = record;
    const href = `/anime/details?id=${anime_data.id}`;
    const title = anime_data.name_cn || anime_data.name;
    const { ratio, text } = progressOf(record);
    const tags = getRecordTags(record);
    const shownTags = view === "grid" ? tags.slice(0, 3) : tags;
    const year = (anime_data.date ?? "").slice(0, 4);

    const tagButtons = shownTags.map((t) => {
        const susume = susumeDefOfTag(t);
        if (susume) {
            return (
                <Link
                    key={t}
                    to={susumePath(susume)}
                    className={`${styles.tag} ${styles.susumeTag}`}
                    title={`推荐分类「${t}」: 去专属页`}
                >
                    ★{t}
                </Link>
            );
        }
        return (
        <button
            key={t}
            type="button"
            className={styles.tag}
            aria-pressed={activeTags.includes(t)}
            onClick={() => onTagClick(t)}
            title={activeTags.includes(t) ? `从筛选中去掉「${t}」` : `加入筛选「${t}」`}
        >
            #{t}
        </button>
        );
    });

    const badge = (
        <span className={`${styles.badge} ${STATUS_CLASS[user_status.watch_status] ?? ""}`}>
            {statusLabel(user_status.watch_status)}
        </span>
    );
    const score = anime_data.score > 0 ? <span className={styles.score}>★ {anime_data.score.toFixed(1)}</span> : null;

    if (view === "list") {
        return (
            <li className={styles.row} data-anime-id={anime_data.id}>
                <Link to={href} className={styles.rowCover} tabIndex={-1} aria-hidden>
                    <img src={toAnimeCdnUrl(`/data/anime/${anime_data.id}.jpg`)} alt="" loading="lazy" decoding="async" />
                </Link>
                <div className={styles.rowMain}>
                    <div className={styles.rowHead}>
                        <Link to={href} className={styles.rowTitle}>{title}</Link>
                        {badge}
                        {score}
                    </div>
                    <div className={styles.rowMeta}>
                        <span className={styles.rowOriginal}>{anime_data.name}</span>
                        {year && <span>{year}</span>}
                        <span>{text}</span>
                    </div>
                    <div className={styles.progress} aria-hidden><span style={{ width: `${ratio * 100}%` }} /></div>
                    {user_status.comment && <p className={styles.rowComment}>{user_status.comment}</p>}
                    {tagButtons.length > 0 && <div className={styles.tags}>{tagButtons}</div>}
                </div>
            </li>
        );
    }

    return (
        <li className={styles.tile} data-anime-id={anime_data.id}>
            <Link to={href} className={styles.cover} aria-label={`${title} 详情`}>
                <img src={toAnimeCdnUrl(`/data/anime/${anime_data.id}.jpg`)} alt="" loading="lazy" decoding="async" />
                {badge}
                {score}
                {user_status.comment && <span className={styles.peek}>{user_status.comment}</span>}
                <span className={styles.progress} aria-hidden><span style={{ width: `${ratio * 100}%` }} /></span>
            </Link>
            <div className={styles.body}>
                <Link to={href} className={styles.title} title={`${title}「${anime_data.name}」`}>{title}</Link>
                <div className={styles.meta}>
                    <span>{text}</span>
                    {year && <span>{year}</span>}
                </div>
                {tagButtons.length > 0 && <div className={styles.tags}>{tagButtons}</div>}
            </div>
        </li>
    );
}
