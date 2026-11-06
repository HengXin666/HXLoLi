import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAnimeRecords } from '@site/src/utils/anime/animeStore';
import { ANIME_TAG_QUERY, countTags } from '@site/src/utils/anime/tags';
import { ANiMeRecord, WatchStatus } from '@site/src/utils/anime/types';
import { toAnimeCdnUrl } from '@site/src/utils/cdn/linkJsDelivr';
import React from 'react';

import styles from './page.module.css';

const STATUS_ORDER: { status: WatchStatus; label: string }[] = [
    { status: WatchStatus.WATCHED, label: '看过' },
    { status: WatchStatus.WATCHING, label: '在看' },
    { status: WatchStatus.ON_HOLD, label: '搁置' },
    { status: WatchStatus.WANT_TO_WATCH, label: '想看' },
    { status: WatchStatus.DROPPED, label: '抛弃' },
];

/** 首页列最近更新进度的几部在看, 与"最近更新"同样的条数 */
const ROW_COUNT = 6;


function totalEps (r: ANiMeRecord): number {
    return r.anime_data.eps || r.anime_data.total_episodes || 0;
}

/**
 * 首页"在追的番", 与"最近更新"是同一种写法: 不套面板, 一行文字计数 + 1px 分隔的列表 + 页脚.
 * 列表每行: 小封面 / 中文名 + 原名 / 细进度条与 已看/总话数 / 评分 / 最后更新日期. 点行进详情页.
 * 标签行只在有计数时出现 (标注表为空时整行不渲染, 不留一个孤零零的"标签"字).
 * 数据与番剧页同源 (useAnimeRecords, 经 CDN 拉 ANiMeRecord.json), 由 LazyMount 滚到附近才挂载.
 * .agents/notes/implemented/feature/2026-11-01-anime-tags-and-home-anime.md
 * .agents/notes/implemented/feature/2026-11-06-home-plain-profile-and-anime.md
 */
export default function HomeAnime (): React.ReactElement {
    const { siteConfig } = useDocusaurusContext();
    const records = useAnimeRecords(siteConfig.baseUrl);

    if (records.length === 0) {
        return <div className={styles.animeSkeleton} aria-busy="true" aria-label="追番记录加载中" />;
    }

    const byStatus = new Map<WatchStatus, number>();
    for (const r of records) byStatus.set(r.user_status.watch_status, (byStatus.get(r.user_status.watch_status) ?? 0) + 1);

    const watching = records
        .filter((r) => r.user_status.watch_status === WatchStatus.WATCHING)
        .sort((a, b) => b.user_status.last_update.localeCompare(a.user_status.last_update))
        .slice(0, ROW_COUNT);

    const tags = countTags(records).filter((t) => t.count > 0).slice(0, 10);

    return (
        <div className={styles.anime}>
            <dl className={styles.animeStats}>
                <div>
                    <dt>记录</dt>
                    <dd>{records.length}</dd>
                </div>
                {STATUS_ORDER.map(({ status, label }) => (
                    <div key={status}>
                        <dt>{label}</dt>
                        <dd>{byStatus.get(status) ?? 0}</dd>
                    </div>
                ))}
            </dl>

            {watching.length > 0 && (
                <ol className={styles.animeList} aria-label="最近在看">
                    {watching.map((r) => {
                        const total = totalEps(r);
                        const seen = r.user_status.watched_eps;
                        const pct = total ? Math.min(100, Math.round((seen / total) * 100)) : 0;
                        const title = r.anime_data.name_cn || r.anime_data.name;
                        return (
                            <li key={r.anime_data.id}>
                                <Link to={`/anime/details?id=${r.anime_data.id}`} className={styles.animeRow}>
                                    <img
                                        className={styles.animeThumb}
                                        src={toAnimeCdnUrl(`/data/anime/${r.anime_data.id}.jpg`)}
                                        alt=""
                                        loading="lazy"
                                        decoding="async"
                                        width={36}
                                        height={51}
                                    />
                                    <span className={styles.animeTitle}>
                                        <span className={styles.animeName}>{title}</span>
                                        {r.anime_data.name_cn && r.anime_data.name !== r.anime_data.name_cn && (
                                            <span className={styles.animeOrig}>{r.anime_data.name}</span>
                                        )}
                                    </span>
                                    <span className={styles.animeProg}>
                                        <span className={styles.animeBar} aria-hidden="true">
                                            <span style={{ width: `${pct}%` }} />
                                        </span>
                                        <span className={styles.animeEps}>
                                            {total ? `${seen}/${total}` : `${seen} 话`}
                                        </span>
                                    </span>
                                    <span className={styles.animeScore}>{r.anime_data.score ? r.anime_data.score.toFixed(1) : '-'}</span>
                                    <time className={styles.animeDate}>{r.user_status.last_update.slice(0, 10)}</time>
                                </Link>
                            </li>
                        );
                    })}
                </ol>
            )}

            <div className={styles.animeFoot}>
                {tags.length > 0 && (
                    <div className={styles.animeTags}>
                        {tags.map((t) => (
                            <Link
                                key={t.tag}
                                to={`/anime?${ANIME_TAG_QUERY}=${encodeURIComponent(t.tag)}`}
                                className={styles.animeTag}
                            >
                                {t.tag}
                                <span className={styles.animeTagCount}>{t.count}</span>
                            </Link>
                        ))}
                    </div>
                )}
                <Link className={styles.feedAll} to="/anime">全部追番 →</Link>
            </div>
        </div>
    );
}
