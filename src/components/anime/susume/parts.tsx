import React from "react";
import Link from "@docusaurus/Link";
import type { AnimeSusumeKey } from "@site/data/animeTags";
import type { ANiMeRecord } from "@site/src/utils/anime/types";
import { useAnimeLabelsStatus } from "@site/src/utils/anime/animeStore";
import { useLocation } from "@docusaurus/router";
import SusumeEntry from "./SusumeEntry";
import { carryFilterSearch, type AnimeListKind } from "./useSusume";
import styles from "@site/src/pages/anime/AnimeIndexPage.module.css";

/**
 * 番剧库 (/anime) 与推荐分类页共用的子导航: 全部 | 四个推荐分类 | キャラ, 右侧关系图.
 * current 是当前推荐分类 (番剧库传 "all"). from 是当前页的筛选语义: 给了就把地址栏上的筛选带到「全部」与各分类链接上,
 * 切 tab 不再重置筛选 / 排序 / 系列视图; 总览页 (/anime/susume) 没有筛选, 不传.
 */
export function AnimeSubnav ({ records, current, from }: {
    records: readonly ANiMeRecord[];
    current?: AnimeSusumeKey | "all";
    from?: AnimeListKind;
}): React.ReactElement {
    const labelsStatus = useAnimeLabelsStatus();
    const { search } = useLocation();
    const toIndex = from ? carryFilterSearch(search, from, "index") : "";
    const toSusume = from ? carryFilterSearch(search, from, "susume") : "";
    return (
        <nav className={styles.subnav} aria-label="番剧页面">
            <Link to={`/anime${toIndex}`} aria-current={current === "all" ? "page" : undefined}>全部</Link>
            <span className={styles.subnavSep} aria-hidden />
            <SusumeEntry
                records={records}
                labelsStatus={labelsStatus}
                countClass={styles.subnavCount}
                current={current === "all" ? undefined : current}
                search={toSusume}
            />
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
