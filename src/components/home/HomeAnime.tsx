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

/** 首页只放最近更新进度的几部在看, 一行放得下 */
const COVER_COUNT = 6;

function progress (r: ANiMeRecord): string {
    const total = r.anime_data.eps || r.anime_data.total_episodes;
    return total ? `${r.user_status.watched_eps} / ${total} 话` : `看到第 ${r.user_status.watched_eps} 话`;
}

/**
 * 首页"在追的番": 一行状态计数 + 最近更新进度的在看封面 + 标签入口. 数据与番剧页同源 (useAnimeRecords,
 * 经 CDN 拉 HXLoLi-ANiMe 的 ANiMeRecord.json, 约 12 MB / gzip 约 2.4 MB), 由 LazyMount 滚到附近才挂载, 首屏不下载.
 * 标签计数走 countTags (本站标签表 + user_status.tags), 点标签进 /anime?tag=..., 萝莉排第一并高亮.
 * 不画分布图: 160 部里看过占 134, 状态分布画出来就是一根长条, 一行数字说得更清楚, 也不用再拉一份 recharts.
 * .agents/notes/implemented/feature/2026-11-01-anime-tags-and-home-anime.md
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
        .slice(0, COVER_COUNT);

    const tags = countTags(records).filter((t) => t.count > 0).slice(0, 10);

    return (
        <div className={styles.anime}>
            <dl className={styles.animeStats}>
                <div className={styles.animeStat}>
                    <dd>{records.length}</dd>
                    <dt>记录</dt>
                </div>
                {STATUS_ORDER.map(({ status, label }) => (
                    <div key={status} className={styles.animeStat}>
                        <dd>{byStatus.get(status) ?? 0}</dd>
                        <dt>{label}</dt>
                    </div>
                ))}
            </dl>

            {watching.length > 0 && (
                <ul className={styles.animeCovers} aria-label="最近在看">
                    {watching.map((r) => (
                        <li key={r.anime_data.id}>
                            <Link to={`/anime/details?id=${r.anime_data.id}`} className={styles.animeCover}>
                                <img
                                    src={toAnimeCdnUrl(`/data/anime/${r.anime_data.id}.jpg`)}
                                    alt={r.anime_data.name_cn || r.anime_data.name}
                                    loading="lazy"
                                    decoding="async"
                                    width={120}
                                    height={170}
                                />
                                <span className={styles.animeName}>{r.anime_data.name_cn || r.anime_data.name}</span>
                                <span className={styles.animeMeta}>
                                    {progress(r)}
                                    {r.anime_data.score ? ` · ${r.anime_data.score}` : ''}
                                </span>
                            </Link>
                        </li>
                    ))}
                </ul>
            )}

            <div className={styles.animeTags}>
                <span className={styles.label}>标签</span>
                {tags.map((t, i) => (
                    <Link
                        key={t.tag}
                        to={`/anime?${ANIME_TAG_QUERY}=${encodeURIComponent(t.tag)}`}
                        className={i === 0 && t.tag === '萝莉' ? styles.animeTagHot : styles.chip}
                    >
                        {t.tag}
                        <span className={styles.animeTagCount}>{t.count}</span>
                    </Link>
                ))}
                <Link className={styles.animeAll} to="/anime">全部追番 →</Link>
            </div>
        </div>
    );
}

