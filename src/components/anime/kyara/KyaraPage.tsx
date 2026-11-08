import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { useBaseUrlUtils } from "@docusaurus/useBaseUrl";
import { AnimatePresence } from "framer-motion";
import { HxState } from "@site/src/components/anime/ui";
import { StageHead, StagePaper } from "@site/src/components/anime/stage/Stage";
import { useKyaraLocal } from "@site/src/utils/kyara/localStore";
import { kyaraHref, useKyaraCatalog } from "@site/src/utils/kyara/link";
import type { KyaraData, KyaraImage, KyaraIndexEntry, KyaraMeta } from "@site/src/utils/kyara/types";
import Lightbox, { cssVars } from "./Lightbox";
import { ImageStage } from "./Spotlight";
import Gallery from "./Gallery";
import KyaraCard, { LocalSwitch } from "./KyaraCard";
import LocalDrawer from "./LocalDrawer";
import { useKyaraDev } from "./devChunks";
import styles from "./Kyara.module.css";

interface ViewProps {
    kyara: KyaraData;
    /** 纯本地角色 (仓库里还没有目录): 无论本地模式开关如何都读本地库 */
    localOnly?: boolean;
    /** 纯本地角色被删除后 */
    onLocalDeleted?: () => void;
}

const sameWork = (a: KyaraMeta, b: KyaraMeta): boolean => {
    const ids = new Set(a.bangumiSubjectIds ?? []);
    if ((b.bangumiSubjectIds ?? []).some((id) => ids.has(id))) return true;
    return !!a.work && a.work === b.work;
};

/**
 * 角色画集主体. 路由页 KyaraPage 与纯本地角色页 KyaraLocalPage 共用.
 * 结构: 面包屑 → 产品式首屏 (左: 主图轮换 + 胶片条; 右: 名字 / 出处 / 规格 / 画师 / 操作) → 画廊 (按画师过滤) → 同作品角色 → 上一位 / 下一位;
 * 本地模式开启 (或纯本地角色) 时吸底出现本地操作台, 本机 dev 下多出写进仓库与仓库管理.
 * 主题色 = meta.accent, 缺省回落 --hx-accent.
 */
export function KyaraView ({ kyara, localOnly = false, onLocalDeleted }: ViewProps): React.ReactElement {
    const { withBaseUrl } = useBaseUrlUtils();
    const { siteConfig } = useDocusaurusContext();
    const dev = useKyaraDev();
    const catalog = useKyaraCatalog();

    // dev 下写盘后先用 dev 清单的最新内容, 等热重扫换掉 props 时再回到 props
    const [fresh, setFresh] = React.useState<KyaraData | null>(null);
    React.useEffect(() => setFresh(null), [kyara]);
    const data = fresh ?? kyara;

    const staticImages = React.useMemo<KyaraImage[]>(
        () => data.images.map((img) => ({ ...img, source: img.source ?? "static", src: withBaseUrl(img.src) })),
        [data.images, withBaseUrl],
    );
    const staticMax = staticImages.reduce((m, img) => Math.max(m, img.seq), 0);
    const active = catalog.localOn || localOnly;
    const local = useKyaraLocal(data.meta.name, active, staticMax);

    const meta: KyaraMeta = React.useMemo(() => {
        const c = local.character;
        if (!localOnly || !c) return data.meta;
        return {
            ...data.meta,
            displayName: c.displayName,
            ...(c.work ? { work: c.work } : {}),
            ...(c.accent ? { accent: c.accent } : {}),
            ...(c.bangumiCharacterId ? { bangumiCharacterId: c.bangumiCharacterId } : {}),
            ...(c.bangumiSubjectIds ? { bangumiSubjectIds: c.bangumiSubjectIds } : {}),
        };
    }, [data.meta, local.character, localOnly]);

    const localImages = active ? local.images : [];
    const all = React.useMemo(() => [...staticImages, ...localImages], [staticImages, localImages]);

    const coverSrc = meta.cover
        ? withBaseUrl(meta.cover)
        : (localOnly && local.character?.cover ? localImages.find((i) => i.file === local.character?.cover)?.src : undefined) ?? all[0]?.src ?? null;
    // 首屏轮换: 封面打头, 其余按序号
    const stageImages = React.useMemo(() => {
        const head = all.find((i) => i.src === coverSrc);
        return head ? [head, ...all.filter((i) => i !== head)] : all;
    }, [all, coverSrc]);

    const accent = meta.accent || "var(--hx-accent)";
    const authors = React.useMemo(() => {
        const m = new Map<string, number>();
        for (const img of all) if (img.author) m.set(img.author, (m.get(img.author) ?? 0) + 1);
        return [...m.entries()].sort((a, b) => b[1] - a[1]);
    }, [all]);

    const [artist, setArtist] = React.useState<string | null>(null);
    const [box, setBox] = React.useState<{ list: KyaraImage[]; index: number } | null>(null);

    const entries = catalog.entries;
    const pos = entries.findIndex((e) => e.meta.name === meta.name);
    const prev: KyaraIndexEntry | null = pos > 0 ? entries[pos - 1] : entries.length > 1 && pos === 0 ? entries[entries.length - 1] : null;
    const next: KyaraIndexEntry | null = pos >= 0 && entries.length > 1 ? entries[(pos + 1) % entries.length] : null;
    const siblings = entries.filter((e) => e.meta.name !== meta.name && sameWork(meta, e.meta));
    const subjectId = meta.bangumiSubjectIds?.[0];

    const refreshRepo = React.useCallback(() => {
        if (!dev) return;
        void dev.kyaraRepo.read(siteConfig.baseUrl, meta.name).then((d) => d && setFresh(d));
    }, [dev, siteConfig.baseUrl, meta.name]);

    const repoTools = dev && !localOnly
        ? <dev.DevRepoTools name={meta.name} images={staticImages} meta={data.meta} baseUrl={siteConfig.baseUrl} onData={setFresh} />
        : null;

    const scrollToGallery = (): void => {
        document.getElementById("gallery")?.scrollIntoView({ behavior: "smooth", block: "start" });
    };

    return (
        <StagePaper
            className={styles.page}
            style={cssVars({ "--kyara-accent": accent, "--stage-accent": accent, "--kyara-bg": coverSrc ? `url("${coverSrc}")` : "none" })}
        >
            <nav className={styles.crumbs} aria-label="面包屑">
                <Link to="/anime">追番记录</Link>
                <span aria-hidden>/</span>
                <Link to="/anime/kyara">キャラ</Link>
                <span aria-hidden>/</span>
                <span aria-current="page">{meta.displayName}</span>
            </nav>

            <section className={styles.product}>
                <ImageStage images={stageImages} onOpen={(i) => setBox({ list: stageImages, index: i })} />

                <div className={styles.info}>
                    <p className={styles.infoKicker}>
                        キャラ{meta.work ? <> · {meta.work}</> : null}
                        {localOnly && <span className={styles.localTag}>本地</span>}
                    </p>
                    <h1 className={styles.infoTitle}>{meta.displayName}</h1>
                    {meta.name !== meta.displayName && <p className={styles.infoAlt}>{meta.name}</p>}
                    {meta.work && (
                        <p className={styles.infoFrom}>
                            出自{" "}
                            {subjectId ? <Link to={`/anime/details?id=${subjectId}`}>《{meta.work}》</Link> : <span>《{meta.work}》</span>}
                        </p>
                    )}

                    <dl className={styles.specs}>
                        <div className={styles.spec}><dd>{all.length}</dd><dt>配图</dt></div>
                        <div className={styles.spec}><dd>{authors.length}</dd><dt>画师</dt></div>
                        <div className={styles.spec}><dd>{(meta.bangumiSubjectIds ?? []).length || (meta.work ? 1 : 0)}</dd><dt>作品</dt></div>
                    </dl>

                    {authors.length > 0 && (
                        <div className={styles.artists} aria-label="画师">
                            {authors.slice(0, 10).map(([a, n]) => (
                                <button
                                    key={a}
                                    type="button"
                                    className={styles.artistChip}
                                    aria-pressed={artist === a}
                                    onClick={() => {
                                        setArtist(artist === a ? null : a);
                                        scrollToGallery();
                                    }}
                                >
                                    <span className={styles.artistInitial} aria-hidden>{Array.from(a)[0]}</span>
                                    @{a}<small>{n}</small>
                                </button>
                            ))}
                        </div>
                    )}

                    <div className={styles.actions}>
                        <button type="button" className={styles.btnPrimary} onClick={scrollToGallery} disabled={all.length === 0}>看全部配图</button>
                        {meta.bangumiCharacterId ? (
                            <a className={styles.btnGhost} href={`https://bgm.tv/character/${meta.bangumiCharacterId}`} target="_blank" rel="noreferrer noopener">
                                Bangumi 角色页 ↗
                            </a>
                        ) : null}
                        {!localOnly && <LocalSwitch on={catalog.localOn} onChange={catalog.setLocalOn} />}
                    </div>
                </div>
            </section>

            <section className={styles.section}>
                <StageHead title="配图" count={`${all.length} 张`} id="gallery" />
                {all.length === 0 ? (
                    active ? (
                        <HxState kind="empty" title="这面墙还空着" body="把图片拖到页面底部的操作台, 或者粘贴 pixiv / x.com 链接." />
                    ) : (
                        <HxState kind="empty" title="这面墙还空着" body="配图正在准备中." />
                    )
                ) : (
                    <Gallery images={all} artist={artist} onArtist={setArtist} onOpen={(list, index) => setBox({ list, index })} />
                )}
                {dev && !localOnly && staticImages.length === 0 && <dev.KyaraGuide name={meta.name} />}
            </section>

            {siblings.length > 0 && (
                <section className={styles.section}>
                    <StageHead title={meta.work ? `《${meta.work}》里的其他角色` : "同一部作品"} count={`${siblings.length} 位`} />
                    <div className={styles.strip}>
                        {siblings.map((e) => <KyaraCard key={e.meta.name} entry={e} />)}
                    </div>
                </section>
            )}

            {(prev || next) && (
                <nav className={styles.pager} aria-label="其他角色">
                    {prev && prev !== next ? <PagerCard entry={prev} dir="prev" /> : <span />}
                    {next ? <PagerCard entry={next} dir="next" /> : <span />}
                </nav>
            )}

            {active && (
                <LocalDrawer
                    name={meta.name}
                    meta={meta}
                    local={local}
                    dev={dev}
                    localOnly={localOnly}
                    repoTools={repoTools}
                    onRepoChanged={refreshRepo}
                    onDeleted={onLocalDeleted}
                />
            )}

            <AnimatePresence>
                {box && box.list[box.index] && (
                    <Lightbox
                        key="lightbox"
                        images={box.list}
                        index={box.index}
                        onIndex={(i) => setBox((b) => (b ? { ...b, index: i } : b))}
                        onClose={() => setBox(null)}
                    />
                )}
            </AnimatePresence>
        </StagePaper>
    );
}

function PagerCard ({ entry, dir }: { entry: KyaraIndexEntry; dir: "prev" | "next" }): React.ReactElement {
    return (
        <Link
            to={kyaraHref(entry)}
            className={styles.pagerCard}
            data-dir={dir}
            style={cssVars({ "--kyara-accent": entry.meta.accent || "var(--hx-accent)" })}
        >
            {entry.cover ? <img src={entry.cover} alt="" loading="lazy" decoding="async" /> : <span className={styles.pagerEmpty} aria-hidden />}
            <span>
                <small>{dir === "prev" ? "‹ 上一位" : "下一位 ›"}</small>
                <b>{entry.meta.displayName}</b>
            </span>
        </Link>
    );
}

/**
 * /anime/kyara/{name}: 某角色的画集, 由 plugins/kyara-gallery-plugin.mjs 按 static/img/kyara/{name}/ 生成路由并注入 kyara 数据.
 * 文件名 001-标题@作者.png -> 按序号升序渲染; 本地模式把浏览器 IndexedDB 里的图排在仓库图之后, 可导入 pixiv / x.com 链接、导出 zip, dev 下可写进仓库.
 * 视觉沿用 HX おすすめ Glass (src/components/anime/ui/DESIGN.md), 颜色只取 --hx-* 令牌与角色 accent.
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
