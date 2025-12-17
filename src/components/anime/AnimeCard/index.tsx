import React, { JSX } from 'react';

import Heading from '@theme/Heading';
import { ANiMeRecord, WatchStatus } from '@site/src/utils/anime/types';
import styles from './AnimeCard.module.css';
import { toJsDelivrUrl } from '@site/src/utils/cdn/linkJsDelivr';
import Link from '@docusaurus/Link';
import MDXA from '@site/src/theme/MDXComponents/A';
// 一个辅助函数, 用于将 WatchStatus 枚举转换为可读的文本
function getWatchStatusText (status: WatchStatus): string {
    switch (status) {
        case WatchStatus.WATCHED: return '已看';
        case WatchStatus.WATCHING: return '在看';
        case WatchStatus.WANT_TO_WATCH: return '想看';
        case WatchStatus.ON_HOLD: return '搁置';
        case WatchStatus.DROPPED: return '抛弃';
        default: return '未知';
    }
}

export default function AnimeCard ({ record }: { record: ANiMeRecord }): JSX.Element {
    const { anime_data, user_status } = record;
    const bangumiUrl = `https://bgm.tv/subject/${anime_data.id}`;

    return (
        <div className={styles.animeCard}>
            {/* 左侧的番剧图片 */}
            <div className={styles.cardImageColumn}>
                <Link to={`/anime/details/${anime_data.id}`}>
                    <img src={toJsDelivrUrl(`/py/anime/data/anime/${anime_data.id}.jpg`)} alt={anime_data.name_cn} />
                </Link>
            </div>

            {/* 右侧的详细信息 */}
            <div className={styles.cardDetailsColumn}>
                {/* 第一行: 标题 */}
                <Heading as="h3" className={styles.cardTitle}>
                    <MDXA href={bangumiUrl}>{anime_data.name_cn}</MDXA>
                    <br />
                    <span className={styles.originalTitle}>{`「${anime_data.name}」`}</span>
                </Heading>

                {/* 第二行: 元数据 */}
                <div className={styles.metaRow}>
                    |
                    <span>共 {anime_data.eps} 话</span>
                    |
                    <span>评分: {anime_data.score}</span>
                    |
                    {user_status.tags.map((tag, idx) => (
                        <>
                            <span key={idx} className={styles.tag}>{tag}</span>
                            |
                        </>
                    ))}
                </div>

                {/* 第三行: 个人评价 */}
                <div className={styles.commentRow}>
                    <div className={styles.commentText}>
                        {/* 如果没有就不显示 */}
                        {user_status.comment &&
                            <blockquote>{user_status.comment}</blockquote>
                        }
                    </div>
                    <Link
                        to={`/anime/details/${anime_data.id}`}
                        className="button button--secondary button--md"
                    >
                        详情
                    </Link>
                </div>

                {/* 第四行: 简介 */}
                <p className={styles.summary}>{anime_data.short_summary ? anime_data.short_summary : "暂无简介"}</p>
            </div>
        </div>
    );
}
