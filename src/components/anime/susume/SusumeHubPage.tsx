import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import { Rise, StageHead, StageHero, paperClass } from "@site/src/components/anime/stage/Stage";
import { useSusumeData, type SusumePick } from "./useSusume";
import { CoverRow, ErrorLine } from "./parts";
import styles from "./susume.module.css";

/**
 * /anime/susume 总览. 第二版 (2026-11-06): 首页语汇.
 * 页头 → 每个分类一节 (节标题 + 说明 + 前六部封面行) → 全部分类里评分最高的六部.
 */
export default function SusumeHubPage (): React.ReactElement {
    const data = useSusumeData();
    const total = data.lists.reduce((s, l) => s + l.picks.length, 0);
    const seen = new Set<number>();
    const best: SusumePick[] = data.lists
        .flatMap((l) => l.picks)
        .filter((p) => p.rate)
        .sort((a, b) => (b.rate ?? 0) - (a.rate ?? 0) || b.score - a.score)
        .filter((p) => (seen.has(p.id) ? false : (seen.add(p.id), true)))
        .slice(0, 6);

    return (
        <Layout title="おすすめ · 推荐" description="HXLoLi 亲手标注的推荐番, 分三类, 按评分排序">
            <main className={`${styles.page} ${paperClass}`}>
                <StageHero
                    crumbs={[["追番记录", "/anime"], ["おすすめ"]]}
                    kicker="HXLoLi のおすすめ"
                    title="推荐"
                    romaji="osusume"
                    sub="不是筛选结果, 是一部部亲手标注、打过分的番. 分三类, 每类按我的评分排."
                    stats={[
                        { label: "分类", value: data.lists.length },
                        { label: "收录", value: data.loading ? null : total, suffix: "部" },
                    ]}
                />

                {data.error && <ErrorLine text={data.error} onRetry={data.retry} />}

                {data.lists.map((l, i) => (
                    <Rise as="section" key={l.def.key} className={styles.section} delay={i * 0.04}>
                        <StageHead
                            title={<Link to={l.path} className={styles.headLink}>{l.def.display}</Link>}
                            count={data.loading ? "…" : `${l.picks.length} 部`}
                            action={<Link to={l.path}>全部 →</Link>}
                        />
                        <p className={styles.lede}>{l.def.subtitle}</p>
                        {data.loading
                            ? <div className={styles.coverSkeleton} aria-busy="true" />
                            : l.picks.length > 0
                                ? <CoverRow picks={l.picks} />
                                : <p className={styles.none}>还没有标注过的番.</p>}
                    </Rise>
                ))}

                {best.length > 0 && (
                    <section className={styles.section} aria-labelledby="susume-best">
                        <StageHead title="评分最高" count={`${best.length} 部`} id="susume-best" action="三个分类合在一起" />
                        <CoverRow picks={best} />
                    </section>
                )}
            </main>
        </Layout>
    );
}
