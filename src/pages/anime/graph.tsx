import React from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import {
    AnimeForceGraph,
    AnimeGraphController,
    GraphBreadcrumb,
    focusTitleOf,
} from "@site/src/components/anime/AnimeGraphPage";
import { GraphEmpty, GraphError, GraphLoading } from "@site/src/components/anime/AnimeGraphPage/GraphStatus";
import { GraphSearch } from "@site/src/components/anime/AnimeGraphPage/SearchBox";
import { useGraphUrlState } from "@site/src/components/anime/AnimeGraphPage/useGraphUrlState";
import styles from "@site/src/components/anime/AnimeGraphPage/graph.module.css";
import { useAnimeRecordsState } from "@site/src/utils/anime/animeStore";


/**
 * 番剧关系图路由: 套站点 Layout, 按加载状态切换 加载 / 错误 / 空态 / 图; 状态全在 URL (id 有值 = 聚焦层)
 * .agents/notes/implemented/feature/2026-11-01-anime-graph-refactor.md
 */
export default function AnimeGraphPage () {
    const { siteConfig } = useDocusaurusContext();
    const { records, status, error, retry } = useAnimeRecordsState(siteConfig.baseUrl);
    const [state, update] = useGraphUrlState();
    const { focusId, hiddenRelations, preferCn, showImages, chargeStrength, linkDistance, minShared } = state;

    const focusMissing = status === "ready" && focusId !== undefined
        && !records.some(r => r.anime_data.id === focusId);
    const layer = focusId !== undefined && !focusMissing ? "focus" : "overview";
    const setFocus = (id: number | undefined) => update({ focusId: id });

    let overlay: React.ReactNode = null;
    if (status === "loading") {
        overlay = <GraphLoading stage="正在下载番剧数据" detail="约 12 MB, 含角色与声优" />;
    } else if (status === "error") {
        overlay = <GraphError message={error} onRetry={retry} />;
    } else if (records.length === 0) {
        overlay = <GraphEmpty title="还没有番剧数据" detail="番剧库为空, 关系图无内容可画." />;
    } else if (focusMissing) {
        overlay = (
            <GraphEmpty
                title={`找不到 id=${focusId} 的番剧`}
                detail="链接里的番剧不在库中, 可能已被移除."
                actionLabel="返回全部番剧"
                onAction={() => setFocus(undefined)}
            />
        );
    }

    return (
        <Layout title="番剧关系图" description="番剧、角色与声优的关系图" noFooter>
            <main className={styles.shell}>
                {overlay ?? (
                    <>
                        <AnimeForceGraph
                            baseUrl={siteConfig.baseUrl}
                            targetAnimeId={focusId}
                            hiddenRelations={hiddenRelations}
                            preferCn={preferCn}
                            showImages={showImages}
                            chargeStrength={chargeStrength}
                            linkDistance={linkDistance}
                            minShared={minShared}
                            onFocusAnime={setFocus}
                        />
                        <div className={styles.topbar}>
                            <GraphBreadcrumb
                                focusTitle={layer === "focus" ? focusTitleOf(records, focusId, preferCn) : undefined}
                                onOverview={() => setFocus(undefined)}
                            />
                            <GraphSearch records={records} preferCn={preferCn} onPick={setFocus} />
                        </div>
                        <AnimeGraphController
                            layer={layer}
                            hiddenRelations={hiddenRelations}
                            setHiddenRelations={v => update({ hiddenRelations: v })}
                            preferCn={preferCn}
                            setPreferCn={v => update({ preferCn: v })}
                            showImages={showImages}
                            setShowImages={v => update({ showImages: v })}
                            minShared={minShared}
                            setMinShared={v => update({ minShared: v })}
                            chargeStrength={chargeStrength}
                            setChargeStrength={v => update({ chargeStrength: v })}
                            linkDistance={linkDistance}
                            setLinkDistance={v => update({ linkDistance: v })}
                        />
                    </>
                )}
            </main>
        </Layout>
    );
}
