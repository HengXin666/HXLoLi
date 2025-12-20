import React from "react";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";
import { useLocation } from "react-router-dom";
import useBaseUrl from "@docusaurus/useBaseUrl";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import BlogWithCats from "@site/src/components/BlogWithCats";
import { useActorMap, useAnimeRecords } from "@site/src/utils/anime/animeStore";
import { toJsDelivrUrl } from "@site/src/utils/cdn/linkJsDelivr";
import { WatchStatus } from "@site/src/utils/anime/types";
import styles from './AnimeDetailPage.module.css';

function useQuery (): URLSearchParams {
    const location = useLocation();
    return new URLSearchParams(location.search);
}

// 将观看状态的枚举值转换为可读的文本
function getWatchStatusText (status: WatchStatus): string {
    switch (status) {
        case WatchStatus.WANT_TO_WATCH:
            return "想看";
        case WatchStatus.WATCHED:
            return "看过";
        case WatchStatus.WATCHING:
            return "在看";
        case WatchStatus.ON_HOLD:
            return "搁置";
        case WatchStatus.DROPPED:
            return "抛弃";
        default:
            return "未知";
    }
}

export default function AnimeDetailPage (): React.ReactElement {
    const query = useQuery();
    const id = query.get("id");
    const { siteConfig } = useDocusaurusContext();
    const records = useAnimeRecords(siteConfig.baseUrl);
    const actorMap = useActorMap(siteConfig.baseUrl);
    const notFoundImageUrl = useBaseUrl("/anime/img/misaka-404.png");

    const record =
        id && /^\d+$/.test(id)
            ? records.find((r) => r.anime_data.id === Number(id))
            : undefined;

    if (!record) {
        return (
            <Layout title={"アニメ 404 NOT FOUND"}>
                <div style={{ height: "60px" }} />
                <BlogWithCats
                    style={{
                        backgroundColor: "#2b2b2b",
                        padding: "20px",
                        display: "flex",
                        flexDirection: "column",
                    }}
                >
                    <div style={{ textAlign: "center" }}>
                        <Heading as="h2" id="404">
                            404 NOT FOUND
                        </Heading>
                        <img src={notFoundImageUrl} alt="404" />
                    </div>
                </BlogWithCats>
                <div style={{ height: "60px" }} />
            </Layout>
        );
    }

    return (
        <Layout title={record.anime_data.name_cn}>
            <div style={{ height: "60px" }} />
            <BlogWithCats
                style={{
                    backgroundColor: "#2b2b2b",
                    padding: "20px",
                }}
            >
                <div className={styles.pageContainer}>

                    <header className={styles.headerSection}>
                        <h1 className={styles.mainTitle}>{record.anime_data.name_cn}</h1>
                        <p className={styles.subTitle}>「{record.anime_data.name}」</p>
                    </header>

                    <main className={styles.mainContent}>
                        <aside>
                            <img src={record.anime_data.image_url} alt={record.anime_data.name_cn} className={styles.coverImage} />
                        </aside>

                        <section className={styles.infoColumn}>
                            <div className={styles.infoBox}>
                                <Heading as="h4" className={styles.sectionTitle} id="basic">基本信息</Heading>
                                <p><strong>评分:</strong> {record.anime_data.score} / 10</p>
                                <p><strong>放送日期:</strong> {record.anime_data.date}</p>
                                <p><strong>集数:</strong> {record.anime_data.eps} 话, <strong>OVA:</strong> {record.anime_data.total_episodes - record.anime_data.eps} 话</p>
                            </div>

                            <div className={styles.infoBox}>
                                <Heading as="h4" className={styles.sectionTitle} id="status">我的状态</Heading>
                                <p><strong>观看状态:</strong> {getWatchStatusText(record.user_status.watch_status)}</p>
                                <p><strong>观看进度:</strong> {record.user_status.watched_eps} / {record.anime_data.total_episodes}</p>
                                {record.user_status.comment && <p><strong>简评:</strong> {record.user_status.comment}</p>}
                                <div>
                                    <strong>标签:</strong>
                                    <div style={{ marginTop: '5px' }}>
                                        {record.user_status.tags.length > 0
                                            ? record.user_status.tags.map(tag => <span key={tag} className={styles.tag}>{tag}</span>)
                                            : " 暂无标签"
                                        }
                                    </div>
                                </div>
                            </div>
                        </section>
                    </main>

                    <section>
                        <Heading as="h3" className={styles.sectionTitle} id="summary">剧情简介</Heading>
                        <p className={styles.summaryText}>{record.anime_data.summary}</p>
                    </section>

                    {/* --- 登场角色 (修改后) --- */}
                    <section>
                        <Heading as="h3" className={styles.sectionTitle} id="characters">登场角色</Heading>
                        <div className={styles.characterScrollContainer}>
                            {record.anime_data.characters.map(char => (
                                <div key={char.id} className={styles.characterCard}>
                                    <a href="#" className={styles.characterImageLink} onClick={(e) => e.preventDefault()} title={char.name}>
                                        <span
                                            className={styles.characterImageSpan}
                                            style={{ backgroundImage: `url(${char.image_url})` }}
                                        ></span>
                                    </a>
                                    <p className={styles.characterName}>{char.name}</p>
                                    <p className={styles.characterRelation}>{char.relation}</p>
                                    <p className={styles.characterActors}>
                                        CV {char.actor_ids
                                            .map(actorId => actorMap.get(actorId)?.name)
                                            .filter(Boolean) // 过滤掉未找到的声优
                                            .join('\n / ') || '?'}
                                    </p>
                                </div>
                            ))}
                        </div>
                    </section>

                    <section>
                        <Heading as="h3" className={styles.sectionTitle} id="relations">关联作品</Heading>
                        <div className={styles.relationList}>
                            {record.anime_data.relations.map(rel => (
                                <div key={rel.id} className={styles.relationItem}>
                                    <img src={rel.image_url} alt={rel.name} className={styles.relationImage} />
                                    <div>
                                        <p className={styles.relationInfoName}>{rel.name_cn || rel.name}</p>
                                        <p className={styles.relationInfoRelation}>{rel.relation}</p>
                                    </div>
                                </div>
                            ))}
                        </div>
                    </section>
                </div>
            </BlogWithCats>
            <div style={{ height: "60px" }} />
        </Layout>
    );
}
