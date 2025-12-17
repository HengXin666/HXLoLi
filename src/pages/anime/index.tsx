import React, { useEffect, useState } from "react";
import Layout from "@theme/Layout";
import AnimeCard from "@site/src/components/anime/AnimeCard";
import { parse } from "@site/src/utils/anime/parser";
import { ANiMeRecord } from "@site/src/utils/anime/types";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Heading from "@theme/Heading";

import styles from '@site/src/components/anime/AnimeCard/AnimeCard.module.css';

// 为按季度分组后的数据结构定义类型
type GroupedAnime = {
    quarter: string;      // 例如: "2024 夏"
    records: ANiMeRecord[]; // 该季度下的所有番剧记录
};

// 辅助函数: 从日期字符串 (例如 "2023-04-01") 获取其所属的年份和季节
function getYearAndQuarter(dateString: string): string {
    if (!dateString) {
        return "未知季度";
    }
    const date = new Date(dateString);
    const year = date.getFullYear();
    const month = date.getMonth() + 1; // getMonth() 返回的月份是 0-11

    if (month <= 3) return `${year} 春`;
    if (month <= 6) return `${year} 夏`;
    if (month <= 9) return `${year} 秋`;
    return `${year} 冬`;
}

// 辅助函数: 按季度对记录进行分组和排序
function groupRecordsByQuarter(records: ANiMeRecord[]): GroupedAnime[] {
    // 1. 使用 reduce 将所有记录按 "年+季节" 分组到 Map 中
    const grouped = records.reduce((acc, record) => {
        const quarter = getYearAndQuarter(record.anime_data.date);
        if (!acc.has(quarter)) {
            acc.set(quarter, []); // 如果是新的季度, 则初始化一个空数组
        }
        acc.get(quarter)!.push(record);
        return acc;
    }, new Map<string, ANiMeRecord[]>());

    // 2. 对每个季度分组内部的记录按日期进行降序排序(新->旧)
    for (const recordsInGroup of grouped.values()) {
        recordsInGroup.sort((a, b) =>
            b.anime_data.date.localeCompare(a.anime_data.date)
        );
    }

    // 3. 定义季节的 chronological (时间先后) 顺序, 用于排序
    const seasonOrder: { [key: string]: number } = { '冬': 4, '秋': 3, '夏': 2, '春': 1 };

    // 4. 将 Map 转换为数组, 并对季度本身进行降序排序
    return Array.from(grouped, ([quarter, records]) => ({ quarter, records }))
        .sort((a, b) => {
            // 从 "YYYY 季" 中拆分出年份和季节
            const [yearA, seasonA] = a.quarter.split(' ');
            const [yearB, seasonB] = b.quarter.split(' ');

            // 首先, 按年份降序比较
            if (yearA !== yearB) {
                return Number(yearB) - Number(yearA);
            }

            // 如果年份相同, 则按季节的先后顺序降序比较
            return seasonOrder[seasonB] - seasonOrder[seasonA];
        });
}

// 页面主组件
export default function Home(): React.ReactNode {
    const { siteConfig } = useDocusaurusContext();
    // State: 用于存储分组和排序后的番剧数据
    const [groupedAnime, setGroupedAnime] = useState<GroupedAnime[]>([]);
    // State: 用于控制加载状态的显示
    const [isLoading, setIsLoading] = useState(true);

    // 使用 useEffect 在组件首次加载时获取和处理数据
    useEffect(() => {
        async function fetchAndGroupRecords() {
            try {
                // 构建数据文件的 URL
                const url = `${siteConfig.baseUrl}anime/ANiMeRecord.json`;
                const response = await fetch(url);
                if (!response.ok) {
                    throw new Error(`数据请求失败: ${response.statusText}`);
                }
                const data = parse<ANiMeRecord[]>(await response.text());

                // 使用我们强大的辅助函数对数据进行分组和排序
                setGroupedAnime(groupRecordsByQuarter(data));
            } catch (error) {
                console.error("获取或处理番剧记录时出错:", error);
            } finally {
                // 无论成功还是失败, 都结束加载状态
                setIsLoading(false);
            }
        }
        fetchAndGroupRecords();
    }, [siteConfig.baseUrl]); // 依赖项数组为空, 确保此 effect 仅运行一次

    // 根据加载状态和数据渲染最终的 UI
    return (
        <Layout title={"アニメ"}>
            <div className="container" style={{ marginTop: '2rem' }}>
                {isLoading ? (
                    <p>正在加载...</p>
                ) : (
                    // 遍历每个季度分组并渲染
                    groupedAnime.map(({ quarter, records }) => (
                        <section key={quarter} className="margin-vert--lg">
                            {/* 季度标题 (例如: 2024 夏) */}
                            <Heading as="h2" id={quarter.replace(' ', '-')}>
                                {quarter}
                            </Heading>
                            {/* 该季度下的时间线列表 */}
                            <ul className={styles.archiveList}>
                                {records.map((record) => (
                                    <AnimeCard key={record.anime_data.id} record={record} />
                                ))}
                            </ul>
                        </section>
                    ))
                )}
            </div>
        </Layout>
    );
}