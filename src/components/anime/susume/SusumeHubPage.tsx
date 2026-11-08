import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import { useHistory } from "@docusaurus/router";
import { useBaseUrlUtils } from "@docusaurus/useBaseUrl";
import PosterCard from "@site/src/components/anime/AnimeCard/PosterCard";
import { StagePaper } from "@site/src/components/anime/stage/Stage";
import { ANIME_TAG_QUERY } from "@site/src/utils/anime/tags";
import { useSusumeData } from "./useSusume";
import { AnimeSubnav, GridSkeleton } from "./parts";
import styles from "@site/src/pages/anime/AnimeIndexPage.module.css";

/** 总览里每个分类露出的卡片数 */
const PREVIEW = 12;
const NO_TAGS: readonly string[] = [];

/**
 * /anime/susume 总览: 番剧库同款页头与子导航, 每个推荐分类一节 (番剧库的分组标题 + 全部 →),
 * 节内是按番名 A-Z 的前 12 张 PosterCard (海报密度, 悬停浮出短评与番剧库一致). 点卡片上的普通标签去番剧库按该标签筛选.
 */
export default function SusumeHubPage (): React.ReactElement {
    const data = useSusumeData();
    const history = useHistory();
    const { withBaseUrl } = useBaseUrlUtils();
    const total = data.lists.reduce((s, l) => s + l.records.length, 0);
    const openTag = React.useCallback(
        (tag: string) => history.push(withBaseUrl(`/anime?${ANIME_TAG_QUERY}=${encodeURIComponent(tag)}`)),
        [history, withBaseUrl],
    );

    return (
        <Layout title="おすすめ · 推荐" description="HXLoLi 的推荐番">
            {/* 墙宽 1200 + 两侧立绘, 与番剧库 / 系列页同一套 (StagePaper 展开 BlogWithCats 的 DOM, 不用其默认导出) */}
            <StagePaper className={styles.page}>
                <header className={styles.header}>
                    <h1 className={styles.title}>おすすめ</h1>
                    <p className={styles.titleSub}>
                        <b>{data.lists.length}</b> 类{!data.loading && <> · <b>{total}</b> 部</>}
                    </p>
                </header>
                <AnimeSubnav records={data.all} />

                {data.error && (
                    <div className={styles.notice} role="alert">
                        <h2>{data.error}</h2>
                        <div className={styles.noticeActions}>
                            <button type="button" className={styles.suggest} onClick={data.retry}>重试</button>
                        </div>
                    </div>
                )}

                <div className={styles.results}>
                    {data.lists.map((l) => (
                        <section key={l.def.key} className={styles.group} aria-labelledby={`susume-${l.def.key}`}>
                            <h2 id={`susume-${l.def.key}`} className={styles.groupTitle}>
                                <Link to={l.path} style={{ color: "inherit" }}>{l.def.display}</Link>
                                <span className={styles.groupCount}>{data.loading ? "…" : `${l.records.length} 部`}</span>
                            </h2>
                            {data.loading
                                ? <GridSkeleton n={6} />
                                : l.records.length === 0
                                    ? <div className={styles.notice}><h2>还没有番</h2></div>
                                    : (
                                        <ul className={styles.grid}>
                                            {l.records.slice(0, PREVIEW).map((r) => (
                                                <PosterCard key={r.anime_data.id} record={r} view="grid" activeTags={NO_TAGS} onTagClick={openTag} />
                                            ))}
                                        </ul>
                                    )}
                            {!data.loading && l.records.length > PREVIEW && (
                                <p style={{ margin: "16px 0 0", textAlign: "right" }}>
                                    <Link to={l.path}>全部 {l.records.length} 部 →</Link>
                                </p>
                            )}
                        </section>
                    ))}
                </div>
            </StagePaper>
        </Layout>
    );
}
