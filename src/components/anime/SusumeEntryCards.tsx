import React from "react";
import Link from "@docusaurus/Link";
import { ANIME_SUSUME_DEFS } from "@site/data/animeTags";
import { ANiMeRecord } from "@site/src/utils/anime/types";
import { recordsOfSusume, susumePath } from "@site/src/utils/anime/tags";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";
import type { AnimeLabelsStatus } from "@site/src/utils/anime/animeStore";
import styles from "./Susume.module.css";

const COLLAGE = 4;

/**
 * 番剧首页 /anime 顶部的三张推荐分类大入口卡 (物凄い / ロリ / シスコン), 一眼可见.
 * 每张卡: 封面拼贴 (该分类按本人评分降序的前 4 部封面, data/anime/<id>.jpg 经番剧 CDN) + 数量 + 名称, 整卡链到 /anime/susume/<slug>.
 * 数据只读人工标注 data/labels (utils/anime/tags.ts); 标注还在拉时数量显示"…", 没有标注时封面位显示空态而不是隐藏卡片 (入口要稳定可见).
 * 设计: hx-anime-tag-refactor-design.md 第 3.1 节.
 */
export default function SusumeEntryCards ({ records, labelsStatus }: { records: readonly ANiMeRecord[]; labelsStatus: AnimeLabelsStatus }): React.ReactElement {
    const bySusume = React.useMemo(
        () => ANIME_SUSUME_DEFS.map((def) => ({ def, list: recordsOfSusume(records, def) })),
        [records],
    );
    const loading = labelsStatus === "loading" || records.length === 0;

    return (
        <nav className={styles.entryGrid} aria-label="推荐分类">
            {bySusume.map(({ def, list }) => {
                const covers = list.slice(0, COLLAGE);
                return (
                    <Link key={def.key} to={susumePath(def)} className={`${styles.entryCard} ${styles[`entry_${def.key}`] ?? ""}`}>
                        <div className={styles.collage} data-n={covers.length} aria-hidden>
                            {covers.map((r) => (
                                <img
                                    key={r.anime_data.id}
                                    src={toAnimeCdnUrl(`/data/anime/${r.anime_data.id}.jpg`)}
                                    alt=""
                                    loading="lazy"
                                    decoding="async"
                                />
                            ))}
                            {covers.length === 0 && <span className={styles.collageEmpty}>{loading ? "…" : "还没有标注"}</span>}
                        </div>
                        <div className={styles.entryText}>
                            <span className={styles.entryName}>{def.display}</span>
                            <span className={styles.entrySub}>{def.subtitle}</span>
                            <span className={styles.entryCount}>
                                {loading ? "…" : <><b>{list.length}</b> 部</>}
                            </span>
                        </div>
                    </Link>
                );
            })}
        </nav>
    );
}
