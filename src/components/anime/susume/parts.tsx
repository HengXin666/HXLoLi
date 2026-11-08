import React from "react";
import Link from "@docusaurus/Link";
import type { AnimeSusumeKey } from "@site/data/animeTags";
import type { ANiMeRecord } from "@site/src/utils/anime/types";
import { useAnimeLabelsStatus } from "@site/src/utils/anime/animeStore";
import SusumeEntry from "./SusumeEntry";
import styles from "@site/src/pages/anime/AnimeIndexPage.module.css";

/** 与番剧库 (/anime) 同一条子导航: 全部 | 四个推荐分类 | キャラ, 右侧关系图; current 是当前推荐分类 */
export function AnimeSubnav ({ records, current }: { records: readonly ANiMeRecord[]; current?: AnimeSusumeKey }): React.ReactElement {
    const labelsStatus = useAnimeLabelsStatus();
    return (
        <nav className={styles.subnav} aria-label="番剧页面">
            <Link to="/anime">全部</Link>
            <span className={styles.subnavSep} aria-hidden />
            <SusumeEntry records={records} labelsStatus={labelsStatus} countClass={styles.subnavCount} current={current} />
            <span className={styles.subnavSep} aria-hidden />
            <Link to="/anime/kyara">キャラ</Link>
            <Link to="/anime/graph" className={styles.subnavEnd}>关系图 →</Link>
        </nav>
    );
}

/** 番剧库同款骨架: n 张封面 + 两行占位, 带 shimmer */
export function GridSkeleton ({ n = 12 }: { n?: number }): React.ReactElement {
    return (
        <ul className={styles.grid} aria-busy="true" aria-label="正在加载" style={{ marginTop: 28 }}>
            {Array.from({ length: n }, (_, i) => (
                <li key={i} className={styles.skeletonTile}>
                    <div className={styles.skeletonCover} />
                    <div className={styles.skeletonLine} />
                    <div className={styles.skeletonLine} />
                </li>
            ))}
        </ul>
    );
}
