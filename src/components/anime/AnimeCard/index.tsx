import React, { JSX } from 'react';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import MDXA from '@site/src/theme/MDXComponents/A';
import { ANiMeRecord } from '@site/src/utils/anime/types';
import { toJsDelivrUrl } from '@site/src/utils/cdn/linkJsDelivr';
import styles from './AnimeCard.module.css';
import { SortMode } from '../SortTabs';

// format date from YYYY-MM-DD to MM-DD
const formatDate = (dateString: string) => {
    if (!dateString) return '';
    const date = new Date(dateString);
    const month = String(date.getMonth() + 1).padStart(2, '0');
    const day = String(date.getDate()).padStart(2, '0');
    return `${month}-${day}`;
};

export default function AnimeCard({ record, sortMode }: { record: ANiMeRecord, sortMode: SortMode }): JSX.Element {
    const { anime_data, user_status } = record;
    const bangumiUrl = `https://bgm.tv/subject/${anime_data.id}`;

    // 根是"li", 它在我们的时间线上充当单个项目。
    return (
        <li className={styles.archiveItem}>
            {/* 时间线上显示的格式化日期 */}
            <time className={styles.archiveTime}>{formatDate(
                sortMode === SortMode.BY_ANIME_DATE ? anime_data.date : user_status.last_update)}</time>

            <div className={styles.animeCard}>
                <div className={styles.cardImageColumn}>
                    <img src={toJsDelivrUrl(`/py/anime/data/anime/${anime_data.id}.jpg`)} alt={anime_data.name_cn} />
                </div>
                <div className={styles.cardDetailsColumn}>
                    <Heading as="h3" className={styles.cardTitle}>
                        <MDXA href={bangumiUrl}>{anime_data.name_cn}</MDXA>
                        <br />
                        <span className={styles.originalTitle}>{`「${anime_data.name}」`}</span>
                    </Heading>
                    <div className={styles.metaRow}>
                        |<span>{anime_data.eps ? `${anime_data.eps} 话` : "? 话"}</span>|
                        <span>评分: {anime_data.score}</span>|
                        {user_status.tags.map((tag, idx) => (
                            <React.Fragment key={idx}>
                                <span className={styles.tag}>{tag}</span>|
                            </React.Fragment>
                        ))}
                    </div>
                    <div className={styles.commentRow}>
                        {user_status.comment && (
                            <div className={styles.commentText}>
                                <blockquote>{user_status.comment}</blockquote>
                            </div>
                        )}
                        {/* 按钮在右边 */}
                        <div className={styles.buttonContainer}>
                            <Link to={`/anime/details/${anime_data.id}`} className="button button--primary button--md">
                                详情
                            </Link>
                        </div>
                    </div>
                    <p className={styles.summary}>
                        <b style={{ fontSize: 'medium' }}>简介: </b>
                        {anime_data.short_summary ?  anime_data.short_summary  + "..." : "暂无简介"}
                    </p>
                </div>
            </div>
        </li>
    );
}
