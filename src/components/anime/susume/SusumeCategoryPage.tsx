import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import type { AnimeSusumeKey } from "@site/data/animeTags";
import { ANIME_SUSUME_DEFS } from "@site/data/animeTags";
import { Numeral, Rise, StageHead, StageHero, accentStyle, paperClass } from "@site/src/components/anime/stage/Stage";
import { SUSUME_COPY, listOf, susumeAccent, useSusumeData, type SusumePick } from "./useSusume";
import { CategoryTabs, EmptyBoard, ErrorLine, PickMeta } from "./parts";
import styles from "./susume.module.css";

/** 前三名: 三张等宽封面卡, 封面下是名次 / 标题 / 元信息 / 一句观后感 */
function TopCard ({ p }: { p: SusumePick }): React.ReactElement {
    return (
        <Rise as="li" className={styles.top} delay={p.rank * 0.04}>
            <Link to={p.href} className={styles.topCover} aria-label={`第 ${p.rank} 名 ${p.title}`}>
                <img src={p.cover} alt="" loading={p.rank === 1 ? "eager" : "lazy"} decoding="async" width={240} height={340} />
            </Link>
            <div className={styles.topBody}>
                <Numeral n={p.rank} className={styles.topRank} />
                <Link to={p.href} className={styles.topTitle}>{p.title}</Link>
                {p.original && <span className={styles.original}>{p.original}</span>}
                <PickMeta p={p} long />
                {p.review && <p className={styles.review}>{p.review}</p>}
            </div>
        </Rise>
    );
}

/** 第 4 名起: 一行一部, 1px 分隔线 (首页「最近更新」的行) */
function RankRow ({ p }: { p: SusumePick }): React.ReactElement {
    return (
        <li>
            <Link to={p.href} className={styles.row}>
                <Numeral n={p.rank} className={styles.rowRank} />
                <img className={styles.rowThumb} src={p.cover} alt="" loading="lazy" decoding="async" width={36} height={51} />
                <span className={styles.rowText}>
                    <span className={styles.rowTitle}>{p.title}</span>
                    {p.review ? <span className={styles.rowReview}>{p.review}</span> : p.original ? <span className={styles.rowReview}>{p.original}</span> : null}
                </span>
                <PickMeta p={p} />
            </Link>
        </li>
    );
}

function Skeleton (): React.ReactElement {
    return (
        <ol className={styles.tops} aria-busy="true" aria-label="正在加载">
            {[1, 2, 3].map((n) => <li key={n} className={styles.skeleton} />)}
        </ol>
    );
}

/**
 * 推荐分类页 /anime/susume/<slug>. 第二版 (2026-11-06): 首页语汇.
 * 页头 (分类名 + 统计) → 分类切换 → 前三 (封面卡) → 第 4 名起 (细线列表) → 收录口径.
 * 数据只读人工标注 (useSusumeData), 按本人评分降序.
 * .agents/notes/implemented/feature/2026-11-06-anime-susume-neko-redesign.md
 */
export default function SusumeCategoryPage ({ susume }: { susume: AnimeSusumeKey }): React.ReactElement {
    const data = useSusumeData();
    const list = listOf(data, susume);
    const { def, picks } = list;
    const idx = ANIME_SUSUME_DEFS.findIndex((d) => d.key === susume);
    const top = picks.slice(0, 3);
    const rest = picks.slice(3);

    return (
        <Layout title={`${def.display} · おすすめ`} description={`HXLoLi 的推荐「${def.display}」: ${def.subtitle}, 按我的评分排序`}>
            <main className={`${styles.page} ${paperClass}`} style={accentStyle(susumeAccent(susume))}>
                <StageHero
                    crumbs={[["追番记录", "/anime"], ["おすすめ", "/anime/susume"], [def.display]]}
                    kicker={`おすすめ · ${String(idx + 1).padStart(2, "0")} / ${String(ANIME_SUSUME_DEFS.length).padStart(2, "0")}`}
                    title={def.display}
                    romaji={SUSUME_COPY[susume].romaji}
                    sub={<>{def.subtitle} · {SUSUME_COPY[susume].pitch}</>}
                    accent={susumeAccent(susume)}
                    stats={[
                        { label: "收录", value: data.loading ? null : picks.length, suffix: "部" },
                        { label: "我的平均分", value: list.avgRate, decimals: 1 },
                        { label: "最早", value: list.years ? list.years[0] : null },
                    ]}
                    aside={<CategoryTabs lists={data.lists} current={susume} loading={data.loading} />}
                />

                {data.error && <ErrorLine text={data.error} onRetry={data.retry} />}

                {data.loading ? <Skeleton /> : picks.length === 0 ? <EmptyBoard list={list} /> : (
                    <>
                        <section className={styles.section} aria-labelledby="susume-top">
                            <StageHead title="前三" id="susume-top" action="按我的评分排, 同分看 Bangumi" />
                            <ol className={styles.tops}>
                                {top.map((p) => <TopCard key={p.id} p={p} />)}
                            </ol>
                        </section>

                        {rest.length > 0 && (
                            <section className={styles.section} aria-labelledby="susume-rest">
                                <StageHead title="其余" count={`${rest.length} 部`} id="susume-rest" />
                                <ol className={styles.rows} start={4}>
                                    {rest.map((p) => <RankRow key={p.id} p={p} />)}
                                </ol>
                            </section>
                        )}
                    </>
                )}

                <section className={styles.section} aria-labelledby="susume-rule">
                    <StageHead title="收录口径" id="susume-rule" />
                    <p className={styles.rule}>{def.rule}</p>
                </section>
            </main>
        </Layout>
    );
}
