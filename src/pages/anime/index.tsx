import React, { useEffect, useState } from "react";
import Layout from "@theme/Layout";
import AnimeCard from "@site/src/components/anime/AnimeCard";
import { parse } from "@site/src/utils/anime/parser";
import { ANiMeRecord, WatchStatus } from "@site/src/utils/anime/types";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Heading from "@theme/Heading";

import styles from "@site/src/components/anime/AnimeCard/AnimeCard.module.css";
import BlogWithCats from "@site/src/components/BlogWithCats";
import { WatchStatusTabs } from "@site/src/components/anime/WatchStatusTabs";
import { SortMode, SortTabs } from "@site/src/components/anime/SortTabs";

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
    return record.anime_data.date;
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

    const [records, setRecords] = useState<ANiMeRecord[]>([]);
    const [isLoading, setIsLoading] = useState(true);

    const [watchStatus, setWatchStatus] = useState<WatchStatus>(
        WatchStatus.WATCHING
    );

    const [sortMode, setSortMode] = useState<SortMode>(SortMode.BY_LAST_UPDATE);

    useEffect(() => {
        async function fetchRecords () {
            try {
                const url = `${siteConfig.baseUrl}anime/ANiMeRecord.json`;
                const response = await fetch(url);
                if (!response.ok) {
                    throw new Error(response.statusText);
                }
                const data = parse<ANiMeRecord[]>(await response.text());
                setRecords(data);
            } finally {
                setIsLoading(false);
            }
        }
        fetchRecords();
    }, [siteConfig.baseUrl]);

    const filtered = records.filter(
        (r) => r.user_status.watch_status === watchStatus
    );

    const groupedAnime = groupByQuarter(filtered, sortMode);

    return (
        <Layout title={"アニメ"}>
            <WatchStatusTabs value={watchStatus} onChange={setWatchStatus} />
            <SortTabs value={sortMode} onChange={setSortMode} />

            <BlogWithCats
                style={{
                    minHeight: "1000px",
                    height: "600px",
                    backgroundColor: "#2b2b2b",
                    padding: "20px",
                    display: "flex",
                    flexDirection: "column",
                }}
            >
                <div
                    style={{
                        flex: 1,
                        minHeight: 0,
                        overflowY: "auto",
                    }}
                >
                    <div style={{ marginTop: "2rem" }}>
                        {isLoading
                            ? null
                            : groupedAnime.map(({ quarter, records }) => (
                                <section key={quarter} className="margin-vert--lg">
                                    <Heading as="h2" id={quarter.replace(" ", "-")}>
                                        <span className={styles.archiveListTitle}>{quarter}</span>
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
                            ))}
                    </div>
                </div>
            </BlogWithCats>
            <div style={{ height: "60px" }}></div>
        </Layout>
    );
}
