import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { ANIME_SUSUME_DEFS, AnimeSusumeKey } from "@site/data/animeTags";
import { useAnimeLabelsStatus, useAnimeRecordsState } from "@site/src/utils/anime/animeStore";
import { getRecordRate, getRecordReview, recordsOfSusume, susumePath } from "@site/src/utils/anime/tags";
import { statusLabel } from "@site/src/utils/anime/filters";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";
import styles from "./Susume.module.css";

/**
 * 推荐分类专属页 (/anime/susume/monosugoi | loli | imouto), 三个路由共用这一个组件, 由 src/pages/anime/susume/<slug>.tsx 传 key.
 * 这不是 /anime?tag= 的筛选结果, 而是独立页面 (hx-anime-tag-refactor-design.md 第 3.1 节):
 *   - 数据只读人工标注 HXLoLi-ANiMe/data/labels/<id>.json (utils/anime/labels.ts), 不读 user_status.tags / animeOverlay.json.tags / ANiMeTags.json.
 *   - 按本人评分 (labels 的 rate, 其次 overlay 的 rate) 降序, 未评分垫底; 同分按 Bangumi 评分.
 *   - 每部附一句话观后感 (labels 的 review, 其次追番短评), 没有就不显示.
 *   - 封面取 data/anime/<id>.jpg (番剧 CDN), 点封面 / 标题进详情页.
 * 三态: 记录或标注还在拉 -> 骨架; 拉失败 -> 错误与重试; 拉到了但没有 -> 空态说明怎么标注.
 */
export default function SusumePage ({ susume }: { susume: AnimeSusumeKey }): React.ReactElement {
    const def = ANIME_SUSUME_DEFS.find((d) => d.key === susume)!;
    const { siteConfig } = useDocusaurusContext();
    const { records, status, error, retry } = useAnimeRecordsState(siteConfig.baseUrl);
    const labelsStatus = useAnimeLabelsStatus();
    const list = React.useMemo(() => recordsOfSusume(records, def), [records, def]);

    const loading = (status === "loading" && records.length === 0) || labelsStatus === "loading";
    const failed = (status === "error" && records.length === 0) || labelsStatus === "error";

    let body: React.ReactNode;
    if (loading) {
        body = (
            <ol className={styles.rankList} aria-busy="true" aria-label="正在加载">
                {Array.from({ length: 4 }, (_, i) => <li key={i} className={styles.skeletonRow} />)}
            </ol>
        );
    } else if (failed) {
        body = (
            <div className={styles.notice} role="alert">
                <h2>{labelsStatus === "error" ? "标注没能加载出来" : "追番记录没能加载出来"}</h2>
                <p>数据从 HXLoLi-ANiMe 的 CDN 拉取, 可能是网络或 CDN 节点问题{error ? `: ${error}` : ""}.</p>
                <button type="button" className={styles.retry} onClick={retry}>重试</button>
            </div>
        );
    } else if (list.length === 0) {
        body = (
            <div className={styles.notice}>
                <h2>「{def.display}」还没有番</h2>
                <p>这里只收人工标注过的番: 在 HXLoLi-ANiMe 的标注 PR 里勾上「{def.display}」, 合入后就会出现在这里.</p>
            </div>
        );
    } else {
        body = (
            <ol className={styles.rankList}>
                {list.map((r, i) => {
                    const a = r.anime_data;
                    const href = `/anime/details?id=${a.id}`;
                    const rate = getRecordRate(r);
                    const review = getRecordReview(r);
                    const year = (a.date ?? "").slice(0, 4);
                    return (
                        <li key={a.id} className={styles.rankRow}>
                            <span className={styles.rankNo} aria-label={`第 ${i + 1} 名`}>{i + 1}</span>
                            <Link to={href} className={styles.rankCover} tabIndex={-1} aria-hidden>
                                <img src={toAnimeCdnUrl(`/data/anime/${a.id}.jpg`)} alt="" loading="lazy" decoding="async" />
                            </Link>
                            <div className={styles.rankMain}>
                                <Link to={href} className={styles.rankTitle}>{a.name_cn || a.name}</Link>
                                {a.name_cn && a.name !== a.name_cn && <span className={styles.rankOriginal}>{a.name}</span>}
                                <div className={styles.rankMeta}>
                                    <span className={styles.myRate}>{rate ? <>我的评分 <b>{rate}</b> / 10</> : "未评分"}</span>
                                    {a.score > 0 && <span>Bangumi ★ {a.score.toFixed(1)}</span>}
                                    {year && <span>{year}</span>}
                                    <span>{statusLabel(r.user_status.watch_status)}</span>
                                </div>
                                {review && <blockquote className={styles.review}>{review}</blockquote>}
                            </div>
                        </li>
                    );
                })}
            </ol>
        );
    }

    return (
        <Layout title={`${def.display} · アニメ`} description={`HXLoLi 的推荐分类「${def.display}」: ${def.subtitle}, 按我的评分排序`}>
            <main className={`${styles.page} ${styles[`entry_${def.key}`] ?? ""}`}>
                <nav className={styles.crumbs} aria-label="面包屑">
                    <Link to="/anime">追番记录</Link>
                    <span aria-hidden> / </span>
                    <span>推荐分类</span>
                </nav>
                <header className={styles.hero}>
                    <h1 className={styles.heroTitle}>{def.display}</h1>
                    <p className={styles.heroSub}>
                        {def.subtitle}
                        {!loading && !failed && <> · <b>{list.length}</b> 部 · 按我的评分排序</>}
                    </p>
                    <p className={styles.heroRule}>{def.rule}</p>
                </header>

                <ul className={styles.siblings} aria-label="其他推荐分类">
                    {ANIME_SUSUME_DEFS.map((d) => (
                        <li key={d.key}>
                            <Link to={susumePath(d)} aria-current={d.key === def.key ? "page" : undefined}>{d.display}</Link>
                        </li>
                    ))}
                </ul>

                {body}
            </main>
        </Layout>
    );
}
