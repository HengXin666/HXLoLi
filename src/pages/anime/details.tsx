import React from "react";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";
import { useLocation } from "react-router-dom";
import useBaseUrl from "@docusaurus/useBaseUrl";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import BlogWithCats from "@site/src/components/BlogWithCats";
import { useActorMap, useAnimeRecords } from "@site/src/utils/anime/animeStore";
import { toJsDelivrUrl } from "@site/src/utils/cdn/linkJsDelivr";
import { ANiMeRecord, WatchStatus } from "@site/src/utils/anime/types";
import styles from './AnimeDetailPage.module.css';
import MDXA from "@site/src/theme/MDXComponents/A";

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

const RelationList = ({ record }: { record: ANiMeRecord }) => {
    const { siteConfig } = useDocusaurusContext();

    // 1. 定义期望的排序顺序
    const sortOrder = [
        "前传",
        "续集",
        "番外篇",
        "片头曲",
        "片尾曲",
        "书籍",
        "游戏",
        "角色曲",
        "广播剧",
    ];

    // 2. 数据分组 (与之前相同)
    const groupedRelations = record.anime_data.relations.reduce((acc: Record<string, typeof record.anime_data.relations>, rel) => {
        const key = rel.relation;
        if (!acc[key]) {
            acc[key] = [];
        }
        acc[key].push(rel);
        return acc;
    }, {});

    // 3. 核心：获取分组的 key，并根据我们定义的 sortOrder 进行排序
    const sortedGroupKeys = Object.keys(groupedRelations).sort((a, b) => {
        const indexA = sortOrder.indexOf(a);
        const indexB = sortOrder.indexOf(b);

        // 如果某个 key 不在我们的排序数组中，就把它放到最后
        const effectiveIndexA = indexA === -1 ? Infinity : indexA;
        const effectiveIndexB = indexB === -1 ? Infinity : indexB;

        return effectiveIndexA - effectiveIndexB;
    });

    // 4. 渲染：遍历排序后的 keys 数组，而不是直接遍历对象
    return (
        <div className={styles.relationContainer}>
            {sortedGroupKeys.map(relation => {
                const items = groupedRelations[relation];
                return (
                    <section key={relation} className={styles.relationGroup}>
                        <h3 className={styles.relationGroupTitle}>{relation}</h3>
                        <div className={styles.relationList}>
                            {items.map(rel => (
                                <div key={rel.id} className={styles.relationItem}>
                                    {rel.name_cn && (
                                        <span className={styles.tooltip}>{rel.name_cn}</span>
                                    )}
                                    <img src={toJsDelivrUrl(`/py/anime/data/relation/${rel.id}.jpg`)} alt={rel.name} className={styles.relationImage} />
                                    <div className={styles.relationInfo}>
                                        <MDXA href={`${siteConfig.baseUrl}anime/details?id=${rel.id}`}>{rel.name}</MDXA>
                                    </div>
                                </div>
                            ))}
                        </div>
                    </section>
                );
            })}
        </div>
    );
};

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
                        <p>请尝试: <MDXA href={`https://bgm.tv/subject/${id}`}>{`bgm.tv/subject/${id}`}</MDXA></p>
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
                            <img src={toJsDelivrUrl(`/py/anime/data/anime/${record.anime_data.id}.jpg`)} alt={record.anime_data.name_cn} className={styles.coverImage} />
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

                    <div style={{ height: "20px" }}></div>

                    <section>
                        <Heading as="h3" className={styles.sectionTitle} id="summary">剧情简介</Heading>
                        <p className={styles.summaryText}>{record.anime_data.summary}</p>
                    </section>

                    {/* --- 角色介绍 --- */}
                    <section>
                        <Heading as="h3" className={styles.sectionTitle} id="characters">角色介绍</Heading>
                        <div className={styles.characterScrollContainer}>
                            {record.anime_data.characters.map(char => (
                                <div key={char.id} className={styles.characterCard}>
                                    <a href="#" className={styles.characterImageLink} onClick={(e) => e.preventDefault()} title={char.name}>
                                        <span
                                            className={styles.characterImageSpan}
                                            style={{ backgroundImage: `url(${toJsDelivrUrl(`/py/anime/data/character/${char.id}.jpg`)})` }}
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

                    <div style={{ height: "20px" }}></div>

                    <section>
                        <Heading as="h3" className={styles.sectionTitle} id="relations">关联条目</Heading>
                        <RelationList record={record} />
                    </section>
                </div>
            </BlogWithCats>
            <div style={{ height: "60px" }} />
        </Layout>
    );
}
