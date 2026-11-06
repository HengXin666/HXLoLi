import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import { useBaseUrlUtils } from "@docusaurus/useBaseUrl";
import { AnimatePresence, motion } from "framer-motion";
import { HxPills, HxState, useHxReducedMotion } from "@site/src/components/anime/ui";
import { StageHead, StageHero, paperClass } from "@site/src/components/anime/stage/Stage";
import { useKyaraLocal } from "@site/src/utils/kyara/localStore";
import type { KyaraData, KyaraImage } from "@site/src/utils/kyara/types";
import Lightbox, { SourceIcon, cssVars, keyOf, ratioOf, seqLabel } from "./Lightbox";
import LocalPanel from "./LocalPanel";
import KyaraGuide from "./KyaraGuide";
import styles from "./Kyara.module.css";

type Scope = "all" | "static" | "local";

interface ViewProps {
    kyara: KyaraData;
    /** 「＋ 新角色」进来的纯本地角色: 仓库里还没有目录, 强制打开本地模式 */
    forceLocal?: boolean;
}

/**
 * 角色配图页主体. 路由页 KyaraPage 与索引页的「新角色」视图共用.
 * 结构: StageHero (该角色配图做封面墙 + accent 光晕 + 本地模式开关) -> HxPills (全部/仓库/本地) -> [LocalPanel] -> 瀑布流 -> Lightbox.
 */
export function KyaraView ({ kyara, forceLocal = false }: ViewProps): React.ReactElement {
    const { meta } = kyara;
    const { withBaseUrl } = useBaseUrlUtils();
    const reduced = useHxReducedMotion();
    const local = useKyaraLocal(meta.name);
    const localOn = forceLocal || local.enabled;

    const staticImages = React.useMemo<KyaraImage[]>(
        () => kyara.images.map((img: KyaraImage) => ({ ...img, source: img.source ?? "static", src: withBaseUrl(img.src) })),
        [kyara.images, withBaseUrl],
    );
    const localImages = React.useMemo<KyaraImage[]>(
        () => (localOn ? local.images.map((img: KyaraImage) => ({ ...img, source: "local" as const })) : []),
        [localOn, local.images],
    );

    const [scope, setScope] = React.useState<Scope>("all");
    React.useEffect(() => {
        if (!localOn && scope === "local") setScope("all");
    }, [localOn, scope]);

    // 本地条目排在静态条目后面 (契约)
    const shown = React.useMemo(() => {
        if (scope === "static") return staticImages;
        if (scope === "local") return localImages;
        return [...staticImages, ...localImages];
    }, [scope, staticImages, localImages]);

    const [open, setOpen] = React.useState<number | null>(null);
    const total = staticImages.length + localImages.length;
    const authors = new Set([...staticImages, ...localImages].map((img) => img.author).filter(Boolean)).size;
    const coverSrc = meta.cover ? withBaseUrl(meta.cover) : (staticImages[0]?.src ?? localImages[0]?.src ?? null);
    const accent = meta.accent || "var(--hx-accent)";

    const pills: { key: Scope; label: React.ReactNode; hint?: React.ReactNode }[] = [
        { key: "all", label: "全部", hint: total },
        { key: "static", label: "仓库", hint: staticImages.length },
    ];
    if (localOn) pills.push({ key: "local", label: "本地", hint: localImages.length });

    const toggle = forceLocal ? null : (
        <button
            type="button"
            role="switch"
            aria-checked={local.enabled}
            className={styles.localSwitch}
            onClick={() => local.setEnabled(!local.enabled)}
        >
            <span className={styles.switchTrack} aria-hidden><span className={styles.switchThumb} /></span>
            本地模式
        </button>
    );

    let body: React.ReactNode;
    if (shown.length === 0) {
        body = scope === "local" ? (
            <HxState kind="empty" title="本地还没有图" body="把图片拖进上面的面板, 或者「从链接导入」pixiv / x.com 的作品." />
        ) : (
            <KyaraGuide name={meta.name} />
        );
    } else {
        body = (
            <div className={styles.masonry} aria-label={`${meta.displayName} 的配图`}>
                {shown.map((img, i) => (
                    <figure
                        key={keyOf(img)}
                        className={styles.tile}
                        style={cssVars({ "--kyara-ratio": ratioOf(img), "--r": String(img.w && img.h ? img.w / img.h : 0.75), "--i": String(i % 12) })}
                    >
                        <button
                            type="button"
                            className={styles.tileBtn}
                            onClick={() => setOpen(i)}
                            aria-label={`查看 ${seqLabel(img.seq)} ${img.title}${img.author ? ` · @${img.author}` : ""}`}
                        >
                            <motion.img
                                layoutId={reduced ? undefined : `kyara-${keyOf(img)}`}
                                src={img.src}
                                alt=""
                                loading={i < 4 ? "eager" : "lazy"}
                                decoding="async"
                                className={styles.tileImg}
                            />
                            <span className={styles.seq}>{seqLabel(img.seq)}</span>
                            <figcaption className={styles.caption}>
                                <span className={styles.capTitle}>{img.title}</span>
                                {img.author && <span className={styles.capAuthor}>@{img.author}</span>}
                                <SourceIcon image={img} />
                            </figcaption>
                        </button>
                    </figure>
                ))}
            </div>
        );
    }

    return (
        <main
            className={`${styles.page} ${paperClass}`}
            style={cssVars({ "--kyara-accent": accent, "--stage-accent": accent, "--kyara-bg": coverSrc ? `url("${coverSrc}")` : "none" })}
        >
            <StageHero
                crumbs={[["追番记录", "/anime"], ["キャラ", "/anime/kyara"], [meta.displayName]]}
                kicker={`キャラ · ${meta.work || "kyara"}`}
                title={meta.displayName}
                romaji={meta.name !== meta.displayName ? meta.name : "kyara"}
                sub={total > 0 ? "一人一面墙. 图按 001、002… 的序号排好, 点开看大图." : "这面墙还空着, 先给她挂上第一张."}
                stats={[
                    { label: "配图", value: total, suffix: "张" },
                    { label: "画师", value: authors, suffix: "位" },
                ]}
                accent={accent}
                backdrop={[...staticImages, ...localImages].map((img) => img.src)}
                say={total > 0 ? "点开看大图, ← → 翻页哦~" : "还没有图, 快给她配一张!"}
                aside={
                    <div className={styles.heroTools}>
                        {toggle}
                        {meta.bangumiCharacterId ? (
                            <a className={styles.heroLink} href={`https://bgm.tv/character/${meta.bangumiCharacterId}`} target="_blank" rel="noreferrer noopener">
                                Bangumi 角色页
                            </a>
                        ) : null}
                    </div>
                }
            />

            <div className={styles.toolbar}>
                <HxPills<Scope> items={pills} current={scope} onPick={setScope} aria-label="图片来源" />
            </div>

            {localOn && (
                <LocalPanel
                    name={meta.name}
                    local={local}
                    nextSeq={staticImages.reduce((m, img) => Math.max(m, img.seq), 0) + 1}
                />
            )}

            <section className={styles.gallery}>
                <StageHead jp="ギャラリー" title="配图" count={`${shown.length} 张`} id="gallery" />
                {body}
            </section>

            <AnimatePresence>
                {open !== null && shown[open] && (
                    <Lightbox key="lightbox" images={shown} index={open} onIndex={setOpen} onClose={() => setOpen(null)} />
                )}
            </AnimatePresence>
        </main>
    );
}

/**
 * /anime/kyara/{name}: 某角色的配图墙, 由 plugins/kyara-gallery-plugin.mjs 按 static/img/kyara/{name}/ 生成路由并注入 kyara 数据.
 * 文件名 001-标题@作者.png -> 按序号升序渲染; 本地模式把浏览器 IndexedDB 里的图排在仓库图之后, 可导入 pixiv / x.com 链接、导出 zip, dev 下可写进仓库.
 * 视觉与动效全部来自 @site/src/components/anime/ui (HX おすすめ Glass), 颜色只取 --hx-* 令牌与角色 accent.
 * .agents/notes/implemented/feature/2026-11-06-anime-kyara-pages.md
 */
export default function KyaraPage ({ kyara }: { kyara: KyaraData }): React.ReactElement {
    const { meta, images } = kyara;
    return (
        <Layout
            title={`${meta.displayName} · キャラ`}
            description={`${meta.displayName}${meta.work ? ` (${meta.work})` : ""} 的配图 ${images.length} 张`}
        >
            <KyaraView kyara={kyara} />
        </Layout>
    );
}
