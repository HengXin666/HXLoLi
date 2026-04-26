import useBaseUrl from "@docusaurus/useBaseUrl";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import AnimeCard from "@site/src/components/anime/AnimeCard";
import { ANiMeRecord, WatchStatus } from "@site/src/utils/anime/types";
import Heading from "@theme/Heading";
import Layout from "@theme/Layout";
import React from "react";

import styles from "@site/src/components/anime/AnimeCard/AnimeCard.module.css";
import { SortMode, SortTabs } from "@site/src/components/anime/SortTabs";
import { WatchStatusTabs } from "@site/src/components/anime/WatchStatusTabs";
import '@site/src/components/BlogWithCats/BlogWithCats.css';
import { useAnimeRecords } from "@site/src/utils/anime/animeStore";
import { usePersistent } from "@site/src/utils/hook/usePersistent";

type GroupedAnime = {
    quarter: string;
    records: ANiMeRecord[];
};

function getYearAndQuarter (dateString: string): string {
    if (!dateString) return "未知季度";
    const date = new Date(dateString);
    const year = date.getFullYear();
    const month = date.getMonth() + 1;

    if (month <= 3) return `${year} 春`;
    if (month <= 6) return `${year} 夏`;
    if (month <= 9) return `${year} 秋`;
    return `${year} 冬`;
}

function getRecordDateBySortMode (
    record: ANiMeRecord,
    sortMode: SortMode
): string {
    if (sortMode === SortMode.BY_LAST_UPDATE) {
        return record.user_status.last_update;
    }
    return record.anime_data.date ?? "9999-12-31";
}

function groupByQuarter (
    records: ANiMeRecord[],
    sortMode: SortMode
): GroupedAnime[] {
    const grouped = records.reduce((acc, record) => {
        const date = getRecordDateBySortMode(record, sortMode);
        const quarter = getYearAndQuarter(date);

        if (!acc.has(quarter)) {
            acc.set(quarter, []);
        }
        acc.get(quarter)!.push(record);
        return acc;
    }, new Map<string, ANiMeRecord[]>());

    for (const recordsInGroup of grouped.values()) {
        recordsInGroup.sort((a, b) => {
            const dateA = getRecordDateBySortMode(a, sortMode);
            const dateB = getRecordDateBySortMode(b, sortMode);
            return dateB.localeCompare(dateA);
        });
    }

    const seasonOrder: Record<string, number> = {
        冬: 4,
        秋: 3,
        夏: 2,
        春: 1,
    };

    return Array.from(grouped, ([quarter, records]) => ({
        quarter,
        records,
    })).sort((a, b) => {
        const [yearA, seasonA] = a.quarter.split(" ");
        const [yearB, seasonB] = b.quarter.split(" ");

        if (yearA !== yearB) {
            return Number(yearB) - Number(yearA);
        }
        return seasonOrder[seasonB] - seasonOrder[seasonA];
    });
}

export default function Home (): React.ReactNode {
    const { siteConfig } = useDocusaurusContext();

    const records = useAnimeRecords(siteConfig.baseUrl);

    const [watchStatus, setWatchStatus] = usePersistent<WatchStatus>(
        `${siteConfig.baseUrl}/anime/:watchStatus`,
        WatchStatus.WATCHING
    );

    const [sortMode, setSortMode] = usePersistent<SortMode>(
        `${siteConfig.baseUrl}/anime/:sortMode`,
        SortMode.BY_LAST_UPDATE
    );

    const filtered = records.filter(
        (r) => r.user_status.watch_status === watchStatus
    );

    const groupedAnime = groupByQuarter(filtered, sortMode);

    const nekoLeftSrc = useBaseUrl('/default-img/neko_left.png');
    const nekoRightSrc = useBaseUrl('/default-img/neko_right.png');

    return (
        <Layout title={"アニメ"}>
            <WatchStatusTabs value={watchStatus} onChange={setWatchStatus} />
            <SortTabs value={sortMode} onChange={setSortMode} />

            <div className="blog-container">
                <div style={{
                    display: 'flex',
                    justifyContent: 'center',
                    maxWidth: '1340px',
                    margin: '0 auto',
                    position: 'relative',
                }}>
                    <div className="neko-sticky" style={{ alignSelf: 'flex-start' }}>
                        <img
                            src={nekoLeftSrc}
                            alt="左猫娘"
                            style={{ width: '200px' }}
                        />
                    </div>

                    <div style={{
                        flex: '1 1 auto',
                        maxWidth: '960px',
                        minHeight: '600px',
                        backgroundColor: '#2b2b2b',
                        padding: '20px',
                        borderRadius: '25px',
                        boxSizing: 'border-box',
                    }}>
                        <div style={{ marginTop: "2rem" }}>
                            {groupedAnime.length === 0 ? (
                                <>
                                    <section key={1} className="margin-vert--lg">
                                        <Heading as="h2" id={"0"}>
                                            <span className={styles.archiveListTitle}>追番记录</span>
                                        </Heading>

                                        <ul className={styles.archiveList}>
                                            <AnimeCard
                                                key={0}
                                                record={{
                                                    anime_data: {
                                                        id: 0,
                                                        name: "とあるアニメ",
                                                        name_cn: "未知番剧",
                                                        score: 9.9,
                                                        eps: 24,
                                                        date: "9999-12-31",
                                                        short_summary: "这是一个占位用的番剧数据",
                                                        summary: "",
                                                        image_url: "",
                                                        total_episodes: 0,
                                                        characters: [],
                                                        relations: [],
                                                        episodes: [],
                                                    },
                                                    user_status: {
                                                        watch_status: WatchStatus.WATCHING,
                                                        last_update: "2077-10-11",
                                                        tags: ["萝莉"],
                                                        comment: "如果存在评论, 这里会显示评论内容",
                                                        watched_eps: 0,
                                                    },
                                                }}
                                                sortMode={sortMode}
                                            />
                                        </ul>
                                    </section>
                                </>
                            ) : (
                                groupedAnime.map(({ quarter, records }) => (
                                    <section key={quarter} className="margin-vert--lg">
                                        <Heading as="h2" id={quarter.replace(" ", "-")}>
                                            <span className={styles.archiveListTitle}>{quarter.split(" ")[0] === "9999" ? "未知日期" : quarter}</span>
                                        </Heading>

                                        <ul className={styles.archiveList}>
                                            {records.map((record) => (
                                                <AnimeCard
                                                    key={record.anime_data.id}
                                                    record={record}
                                                    sortMode={sortMode}
                                                />
                                            ))}
                                        </ul>
                                    </section>
                                ))
                            )}
                        </div>
                    </div>

                    <div className="neko-sticky" style={{ alignSelf: 'flex-start' }}>
                        <img
                            src={nekoRightSrc}
                            alt="右猫娘"
                            style={{ width: '200px' }}
                        />
                    </div>
                </div>
            </div>
            <div style={{ height: "60px" }}></div>
        </Layout>
    );
}
