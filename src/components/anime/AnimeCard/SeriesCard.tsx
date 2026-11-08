import React from "react";
import Link from "@docusaurus/Link";
import { WatchStatus } from "@site/src/utils/anime/types";
import { susumeDefOfTag, susumePath } from "@site/src/utils/anime/tags";
import { normalEpisodeCount } from "@site/src/utils/anime/filters";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";
import {
    AnimeSeason,
    AnimeSeries,
    isMultiSeason,
    mainSeasons,
    seasonCount,
    seasonLabel,
    seasonShort,
    seriesCoverId,
    SeriesHit,
    seriesProgress,
    seriesScoreRange,
    seriesStatus,
    seriesTags,
    seriesYearRange,
    specialSeasons,
} from "@site/src/utils/anime/series";
import PosterCard, { AnimeView } from "./PosterCard";
import posterStyles from "./PosterCard.module.css";
import styles from "./SeriesCard.module.css";

export interface SeriesCardProps {
    hit: SeriesHit;
    view: AnimeView;
    activeTags: readonly string[];
    onTagClick: (tag: string) => void;
}

const STATUS_CLASS: Record<WatchStatus, string> = {
    [WatchStatus.WATCHING]: posterStyles.sWatching,
    [WatchStatus.WATCHED]: posterStyles.sWatched,
    [WatchStatus.WANT_TO_WATCH]: posterStyles.sWish,
    [WatchStatus.ON_HOLD]: posterStyles.sHold,
    [WatchStatus.DROPPED]: posterStyles.sDropped,
};

/** 叠图最多画几层: 再多卡片太碎, 多出来的在最后一层角标写 "+N" */
const MAX_LAYERS = 5;
/** 静止时每层向右上错开的像素上限, 总错位不超过 DECK_ROOM (= .deck 的 padding) */
const DECK_ROOM = 10;

interface DeckLayer {
    season: AnimeSeason;
    /** 静止时的叠放深度, 封面那季为 0 (最上层) */
    depth: number;
    style: React.CSSProperties;
}

/**
 * 叠图的每一层: 取系列前 MAX_LAYERS 季 (正篇在前, 特别篇在后, 已按首播排好), 保证封面那季在内.
 * 静止: 封面在最上, 其余按顺序往右上错开, 越深越暗, 一眼看出"一叠几部".
 * 展开 (悬停 / 键盘聚焦): 各层按首播顺序从左到右扇形摊开, 以底边中点为轴, 越靠外转得越多, 弹簧曲线 + 由中心向外错峰.
 * 位移全部算成 CSS 变量, 动画只动 transform / opacity / filter, 不触发布局.
 */
function deckLayers (series: AnimeSeries, coverId: number, mini: boolean): { layers: DeckLayer[]; extra: number } {
    const all = series.seasons;
    let picked = all.slice(0, mini ? 3 : MAX_LAYERS);
    if (!picked.some((x) => x.record.anime_data.id === coverId)) {
        const cover = all.find((x) => x.record.anime_data.id === coverId);
        if (cover) picked = [...picked.slice(0, picked.length - 1), cover];
    }
    const n = picked.length;
    const coverIndex = Math.max(0, picked.findIndex((x) => x.record.anime_data.id === coverId));
    const step = Math.min(mini ? 3 : 5, (mini ? 6 : DECK_ROOM) / Math.max(1, n - 1));
    const half = (n - 1) / 2 || 1;
    // 扇形幅度随层数收敛, 两层时更紧凑
    const spread = n <= 2 ? 16 : n === 3 ? 20 : 24;   // 外侧卡中心相对自身宽度的平移 (%)
    const tilt = n <= 2 ? 6 : 8;          // 外侧卡的旋转 (deg)
    let rank = 0;
    const layers = picked.map((season, i) => {
        const depth = i === coverIndex ? 0 : ++rank;
        const o = (i - (n - 1) / 2) / half;   // -1 .. 1, 左外 .. 右外
        const style = mini
            ? {
                "--rx": `${depth * step}px`,
                "--ry": `${-depth * step}px`,
                "--rr": "0deg",
                "--fx": `${i * 12}%`,
                "--fy": "0%",
                "--fr": `${i * 3}deg`,
                "--fs": "0.92",
                "--z": n - depth,
                "--fz": i + 1,
                "--dim": 1 - depth * 0.16,
                "--delay": `${i * 30}ms`,
            }
            : {
                "--rx": `${depth * step}px`,
                "--ry": `${-depth * step}px`,
                "--rr": `${depth * 1.2}deg`,
                "--fx": `${o * spread}%`,
                "--fy": `${o * o * 3 - 4}%`,
                "--fr": `${o * tilt}deg`,
                "--fs": "0.64",
                "--z": n - depth,
                "--fz": i + 1,
                "--dim": 1 - depth * 0.14,
                "--delay": `${Math.abs(o) * half * 35}ms`,
            };
        return { season, depth, style: style as React.CSSProperties };
    });
    return { layers, extra: all.length - n };
}

/** 叠图本体: 层按首播顺序输出 (DOM 顺序即展开后的左右顺序), 叠放靠 z-index */
function Deck ({ series, coverId, mini }: { series: AnimeSeries; coverId: number; mini: boolean }): React.ReactElement {
    const { layers, extra } = deckLayers(series, coverId, mini);
    return (
        <>
            {layers.map(({ season, depth, style }, i) => {
                const id = season.record.anime_data.id;
                const last = i === layers.length - 1;
                return (
                    <span key={id} className={`${styles.layer} ${depth === 0 ? styles.layerFront : ""}`} style={style}>
                        <img src={toAnimeCdnUrl(`/data/anime/${id}.jpg`)} alt="" loading="lazy" decoding="async" draggable={false} />
                        {!mini && (
                            <span className={`${styles.layerTag} ${STATUS_CLASS[season.record.user_status.watch_status] ?? ""}`}>
                                {seasonShort(season)}{last && extra > 0 ? ` +${extra}` : ""}
                            </span>
                        )}
                    </span>
                );
            })}
        </>
    );
}

/** 一季在列表行里的小进度: "S2 ✓" / "S3-2 7/12" / "FS 想看" (head 为 seasonShort, 分割放送带 -部分) */
function seasonChip (head: string, watched: number, total: number, status: WatchStatus): string {
    if (status === WatchStatus.WATCHED) return `${head} ✓`;
    if (status === WatchStatus.WANT_TO_WATCH) return `${head} 想看`;
    if (status === WatchStatus.DROPPED) return `${head} 弃`;
    return total ? `${head} ${watched}/${total}` : `${head} ${watched}`;
}

/**
 * 番剧库「系列」视图的一张卡 (?fold=series). 一个系列只有一季时直接渲染 PosterCard (链到详情, 不多一层空过渡页);
 * 多季时: 把系列每一部的封面真实地叠成一叠 (最多 5 层, 封面那季在最上, 见 deckLayers), 悬停 / 聚焦时扇形摊开并给每层标季号;
 * 角标为聚合状态, 左下 "N 季", 进度为正篇合计,
 * 评分为各季 Bangumi 分区间, 链到 /anime/series?id=<系列 key>. 筛选按单季算, 只命中部分季时标 "命中 x/N 季".
 * data-anime-id 填系列 key, 接入现有筛选切换过渡 (utils/anime/listMotion.ts).
 * .agents/notes/implemented/feature/2026-11-05-anime-series-fold.md
 * 推荐分类徽章不带悬停提示: .agents/notes/implemented/feature/2026-11-08-anime-susume-wall.md
 */
export default function SeriesCard ({ hit, view, activeTags, onTagClick }: SeriesCardProps): React.ReactElement {
    const { series, hits } = hit;
    if (!isMultiSeason(series)) {
        return <PosterCard record={series.seasons[0].record} view={view} activeTags={activeTags} onTagClick={onTagClick} />;
    }

    const main = mainSeasons(series);
    const specials = specialSeasons(series);
    const { status, label } = seriesStatus(series);
    const { watched, total, ratio } = seriesProgress(series);
    const range = seriesScoreRange(series);
    const years = seriesYearRange(series);
    const tags = seriesTags(series);
    const shownTags = view === "grid" ? tags.slice(0, 3) : tags;
    const partial = hits.size < series.seasons.length;
    // 只命中部分季时把命中的季带进过渡页高亮
    const href = `/anime/series?id=${series.key}${partial ? `&hit=${[...hits].join(",")}` : ""}`;
    const countText = `${seasonCount(series)} 季${specials.length ? ` + ${specials.length} 特别篇` : ""}`;
    const progressText = total ? `${watched} / ${total} 话` : countText;
    const coverId = seriesCoverId(series);

    const badge = <span className={`${posterStyles.badge} ${STATUS_CLASS[status] ?? ""}`}>{label}</span>;
    const score = range
        ? <span className={posterStyles.score}>★ {range.min === range.max ? range.max.toFixed(1) : `${range.min.toFixed(1)}–${range.max.toFixed(1)}`}</span>
        : null;

    const tagButtons = shownTags.map((t) => {
        const susume = susumeDefOfTag(t);
        if (susume) {
            return (
                <Link key={t} to={susumePath(susume)} className={`${posterStyles.tag} ${posterStyles.susumeTag}`}>
                    ★{t}
                </Link>
            );
        }
        return (
            <button
                key={t}
                type="button"
                className={posterStyles.tag}
                aria-pressed={activeTags.includes(t)}
                onClick={() => onTagClick(t)}
                title={activeTags.includes(t) ? `从筛选中去掉「${t}」` : `加入筛选「${t}」`}
            >
                #{t}
            </button>
        );
    });

    const hitNote = partial ? <span className={styles.hitNote}>命中 {hits.size}/{series.seasons.length}</span> : null;

    if (view === "list") {
        return (
            <li className={`${posterStyles.row} ${styles.deckRow}`} data-anime-id={series.key}>
                <Link to={href} className={`${styles.deck} ${styles.deckMini}`} tabIndex={-1} aria-hidden>
                    <span className={styles.stage}>
                        <Deck series={series} coverId={coverId} mini />
                    </span>
                </Link>
                <div className={posterStyles.rowMain}>
                    <div className={posterStyles.rowHead}>
                        <Link to={href} className={posterStyles.rowTitle}>{series.title}</Link>
                        {badge}
                        <span className={styles.countPill}>{countText}</span>
                        {score}
                        {hitNote}
                    </div>
                    <div className={posterStyles.rowMeta}>
                        {series.originalTitle && series.originalTitle !== series.title && <span className={posterStyles.rowOriginal}>{series.originalTitle}</span>}
                        {years && <span>{years}</span>}
                        <span>{progressText}</span>
                    </div>
                    <div className={posterStyles.progress} aria-hidden><span style={{ width: `${ratio * 100}%` }} /></div>
                    <div className={styles.chips}>
                        {main.map((season) => (
                            <Link
                                key={season.record.anime_data.id}
                                to={`/anime/details?id=${season.record.anime_data.id}`}
                                className={`${styles.chip} ${hits.has(season.record.anime_data.id) && partial ? styles.chipHit : ""}`}
                                title={`${seasonLabel(season)} · ${season.record.anime_data.name_cn || season.record.anime_data.name}`}
                            >
                                {seasonChip(seasonShort(season), season.record.user_status.watched_eps || 0, normalEpisodeCount(season.record), season.record.user_status.watch_status)}
                            </Link>
                        ))}
                    </div>
                    {tagButtons.length > 0 && <div className={posterStyles.tags}>{tagButtons}</div>}
                </div>
            </li>
        );
    }

    return (
        <li className={`${posterStyles.tile} ${styles.deckTile}`} data-anime-id={series.key}>
            <Link to={href} className={styles.deck} aria-label={`${series.title} 系列, ${countText}`}>
                <span className={styles.stage}>
                    <span className={styles.glow} aria-hidden />
                    <Deck series={series} coverId={coverId} mini={false} />
                    <span className={styles.overlay}>
                        {badge}
                        {score}
                        <span className={styles.countBadge}>{countText}</span>
                        <span className={`${posterStyles.progress} ${styles.overlayProgress}`} aria-hidden><span style={{ width: `${ratio * 100}%` }} /></span>
                    </span>
                </span>
            </Link>
            <div className={posterStyles.body}>
                <Link to={href} className={posterStyles.title} title={`${series.title}「${series.originalTitle}」`}>{series.title}</Link>
                <div className={posterStyles.meta}>
                    <span>{progressText}</span>
                    {years && <span>{years}</span>}
                    {hitNote}
                </div>
                {tagButtons.length > 0 && <div className={posterStyles.tags}>{tagButtons}</div>}
            </div>
        </li>
    );
}
