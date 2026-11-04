import React from "react";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";
import useBaseUrl from "@docusaurus/useBaseUrl";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import useIsBrowser from "@docusaurus/useIsBrowser";
import Link from "@docusaurus/Link";
import { PageMetadata } from "@docusaurus/theme-common";
import { FaCodeBranch, FaExternalLinkAlt, FaShareAlt, FaStar } from "react-icons/fa";
import { useActorMap, useAnimeRecordsState } from "@site/src/utils/anime/animeStore";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";
import { Actor, ANiMeRecord, Character, Episode, EpisodeType, Relation, WatchStatus } from "@site/src/utils/anime/types";
import { ANIME_TAG_QUERY, getRecordTags, susumeDefOfTag, susumePath } from "@site/src/utils/anime/tags";
import { animeSeasonOf, normalEpisodeCount, statusLabel } from "@site/src/utils/anime/filters";
import { formatMmSs } from "@site/src/utils/formatTime";
import Tooltip from "@site/src/components/common/Tooltip";
import ZoomImage from "@site/src/components/common/ZoomImage";
import HXGiscus from "@site/src/components/Giscus";
import useQuery from "@site/src/utils/url/useQuery";
import styles from "./AnimeDetailPage.module.css";
import { loadAnimeOverlayEditor } from "@site/src/components/home/lazyChunks";
import { CharacterMenuAnime, CharacterMenuScope, useCharacterMenuTrigger } from "@site/src/components/anime/CharacterMenu/CharacterContextMenu";

/**
 * 本人标注编辑器: 只在 dev 下有 (生产构建里 loader 是 null, 整块不渲染).
 * .agents/notes/implemented/feature/2026-11-01-anime-local-overlay.md
 */
const OverlayEditor = loadAnimeOverlayEditor ? React.lazy(loadAnimeOverlayEditor) : null;

// ---------------------------------------------------------------- 常量与小工具

const STATUS_CLASS: Record<number, string> = {
    [WatchStatus.WATCHING]: styles.statusWatching,
    [WatchStatus.WATCHED]: styles.statusWatched,
    [WatchStatus.WANT_TO_WATCH]: styles.statusWish,
    [WatchStatus.ON_HOLD]: styles.statusOnHold,
    [WatchStatus.DROPPED]: styles.statusDropped,
};

const EPISODE_GROUPS: readonly { label: string; types: readonly EpisodeType[] }[] = [
    { label: "正片", types: [EpisodeType.NORMAL] },
    { label: "SP", types: [EpisodeType.SP] },
    { label: "OP / ED", types: [EpisodeType.OP, EpisodeType.ED] },
    { label: "PV 与其他", types: [EpisodeType.PROMO, EpisodeType.MAD, EpisodeType.OTHER] },
];

const EPISODE_TYPE_TEXT: Record<number, string> = {
    [EpisodeType.NORMAL]: "正片",
    [EpisodeType.SP]: "SP",
    [EpisodeType.OP]: "OP",
    [EpisodeType.ED]: "ED",
    [EpisodeType.PROMO]: "PV",
    [EpisodeType.MAD]: "MAD",
    [EpisodeType.OTHER]: "其他",
};

const RELATION_ORDER = ["前传", "续集", "番外篇", "总集篇", "不同演绎", "主线故事", "片头曲", "片尾曲", "插入歌", "角色曲", "原声集", "书籍", "游戏", "广播剧"];
const CHARACTER_ORDER = ["主角", "配角", "客串"];
const CHARACTER_PREVIEW = 12;
const SUMMARY_FOLD = 220;

const orderIndex = (order: readonly string[], key: string): number => {
    const i = order.indexOf(key);
    return i === -1 ? order.length : i;
};

/** last_update 形如 "Mon 2026-03-30 1:07 PM CST (UTC+08:00)", 只取日期 */
const updateDate = (s: string | null | undefined): string => (s ?? "").match(/\d{4}-\d{2}-\d{2}/)?.[0] ?? "";

/** 页内目录: 宽屏是左侧竖排侧栏, 窄屏是内容上方一条可横滑的细条 (都不吸顶盖内容) */
const SECTIONS: readonly { id: string; label: string }[] = [
    { id: "summary", label: "简介" },
    { id: "eps", label: "剧集" },
    { id: "characters", label: "角色" },
    { id: "relations", label: "关联" },
    { id: "similar", label: "相似" },
    { id: "comments", label: "讨论" },
];

// ---------------------------------------------------------------- Hero

function Hero ({ record, coverUrl }: { record: ANiMeRecord; coverUrl: string }): React.ReactElement {
    const { anime_data: a, user_status: u } = record;
    const season = animeSeasonOf(a.date);
    const total = normalEpisodeCount(record) || a.total_episodes;
    const watched = Math.max(0, u.watched_eps);
    const pct = total > 0 ? Math.min(1, watched / total) : (u.watch_status === WatchStatus.WATCHED ? 1 : 0);
    const tags = getRecordTags(record);
    const updated = updateDate(u.last_update);

    return (
        <header className={styles.hero} style={{ "--cover": `url("${coverUrl}")` } as React.CSSProperties}>
            <div className={styles.heroGrid}>
                <div className={styles.coverFrame}>
                    <ZoomImage src={coverUrl} alt={a.name_cn || a.name} className={styles.cover} />
                </div>

                <div className={styles.heroHead}>
                    <h1 className={styles.title}>{a.name_cn || a.name}</h1>
                    {a.name_cn && a.name !== a.name_cn && <p className={styles.originalTitle}>{a.name}</p>}
                    <ul className={styles.metaRow} aria-label="基本信息">
                        {a.score > 0 && (
                            <li className={styles.score}>
                                <FaStar aria-hidden /> <strong>{a.score.toFixed(1)}</strong> <span className={styles.scoreSrc}>Bangumi</span>
                            </li>
                        )}
                        {season && <li>{season.year} {season.season} · {season.index * 3 + 1} 月番</li>}
                        <li>{a.date ? `${a.date} 首播` : "首播日期未知"}</li>
                        <li>{total > 0 ? `全 ${total} 话` : "集数未知"}</li>
                    </ul>
                </div>

                <div className={styles.heroBody}>
                    <div className={styles.statusCard}>
                        <div className={styles.statusLine}>
                            <span className={`${styles.statusBadge} ${STATUS_CLASS[u.watch_status] ?? ""}`}>{statusLabel(u.watch_status)}</span>
                            <span className={styles.progressText}>已看 <strong>{watched}</strong>{total > 0 ? ` / ${total}` : ""} 话</span>
                            {updated && <span className={styles.updated}>更新于 {updated}</span>}
                        </div>
                        <div
                            className={styles.progressBar}
                            role="progressbar"
                            aria-label="观看进度"
                            aria-valuemin={0}
                            aria-valuemax={total || 1}
                            aria-valuenow={Math.min(watched, total || 1)}
                        >
                            <span style={{ width: `${Math.round(pct * 100)}%` }} />
                        </div>
                    </div>

                    {tags.length > 0 && (
                        <div className={styles.tagRow}>
                            {/* 标签只读人工标注 data/labels (utils/anime/tags.ts); 推荐分类渲染成徽章进专属页, 普通标注走 ?tag= 筛选 */}
                            {tags.map((tag) => {
                                const susume = susumeDefOfTag(tag);
                                return susume ? (
                                    <Link key={tag} to={susumePath(susume)} className={styles.susumeBadge} title={`${susume.subtitle}: 进入「${tag}」专属页`}>
                                        ★ {tag}
                                    </Link>
                                ) : (
                                    <Link key={tag} to={`/anime?${ANIME_TAG_QUERY}=${encodeURIComponent(tag)}`} className={styles.tag}>#{tag}</Link>
                                );
                            })}
                        </div>
                    )}

                    <div className={styles.actions}>
                        <a className={styles.action} href={`https://bgm.tv/subject/${a.id}`} target="_blank" rel="noopener noreferrer">
                            <FaExternalLinkAlt aria-hidden /> Bangumi
                        </a>
                        <Link className={styles.action} to={`/anime/graph?id=${a.id}`}>
                            <FaCodeBranch aria-hidden /> 关系图
                        </Link>
                        <Link className={styles.action} to="/anime/share?status=watched,watching">
                            <FaShareAlt aria-hidden /> 分享番单
                        </Link>
                    </div>
                </div>
            </div>
        </header>
    );
}

// ---------------------------------------------------------------- 页内目录

/** 当前读到哪一段: 观察各 section, 取视口上部 (navbar 下沿到 45% 高) 里排最前的那个; 不支持 IO 时停在第一个 */
function useActiveSection (ids: readonly string[]): [string, (id: string) => void] {
    const key = ids.join(",");
    const [active, setActive] = React.useState(ids[0] ?? "");
    React.useEffect(() => {
        const order = key.split(",").filter(Boolean);
        if (order.length === 0 || typeof IntersectionObserver === "undefined") return undefined;
        const byEl = new Map<Element, string>();
        for (const id of order) {
            const el = document.getElementById(id)?.closest("section");
            if (el) byEl.set(el, id);
        }
        const visible = new Set<string>();
        const io = new IntersectionObserver((entries) => {
            for (const e of entries) {
                const id = byEl.get(e.target);
                if (!id) continue;
                if (e.isIntersecting) visible.add(id);
                else visible.delete(id);
            }
            const first = order.find((id) => visible.has(id));
            if (first) setActive(first);
        }, { rootMargin: "-80px 0px -55% 0px", threshold: 0 });
        byEl.forEach((_, el) => io.observe(el));
        return () => io.disconnect();
    }, [key]);
    return [active, setActive];
}

function SectionNav ({ sections }: { sections: readonly { id: string; label: string }[] }): React.ReactElement {
    const [active, setActive] = useActiveSection(sections.map((s) => s.id));
    return (
        <nav className={styles.toc} aria-label="页内导航">
            <p className={styles.tocTitle}>目录</p>
            <ul className={styles.tocList}>
                {sections.map((s) => (
                    <li key={s.id}>
                        <a
                            href={`#${s.id}`}
                            className={`${styles.tocLink} ${active === s.id ? styles.tocActive : ""}`}
                            aria-current={active === s.id ? "location" : undefined}
                            onClick={() => setActive(s.id)}
                        >
                            {s.label}
                        </a>
                    </li>
                ))}
            </ul>
        </nav>
    );
}

// ---------------------------------------------------------------- 简介

function Summary ({ text }: { text: string }): React.ReactElement {
    const [open, setOpen] = React.useState(false);
    const long = text.length > SUMMARY_FOLD;
    return (
        <div>
            <p className={`${styles.summary} ${long && !open ? styles.summaryFolded : ""}`}>{text || "暂无简介"}</p>
            {long && (
                <button type="button" className={styles.linkButton} onClick={() => setOpen((v) => !v)} aria-expanded={open}>
                    {open ? "收起" : "展开全部"}
                </button>
            )}
        </div>
    );
}

// ---------------------------------------------------------------- 剧集

function EpisodeCard ({ ep }: { ep: Episode }): React.ReactElement {
    return (
        <div className={styles.epCard}>
            <p className={styles.epCardTitle}>第 {ep.sort} 集{ep.name_cn ? ` · ${ep.name_cn}` : ""}</p>
            {ep.name && ep.name !== ep.name_cn && <p className={styles.epCardSub}>「{ep.name}」</p>}
            <p className={styles.epCardMeta}>
                {EPISODE_TYPE_TEXT[ep.type] ?? "其他"} · 放送 {ep.air_date || "未知"} · 时长 {formatMmSs(ep.duration_seconds) || "未知"}
            </p>
            <p className={styles.epCardDesc}>{ep.desc || "暂无简介"}</p>
        </div>
    );
}

function Episodes ({ record }: { record: ANiMeRecord }): React.ReactElement | null {
    // 复制后再排: 记录对象是冻结的, 原地 sort 会抛错, 也会改写其他页面共享的数据
    const sorted = React.useMemo(() => [...record.anime_data.episodes].sort((x, y) => x.sort - y.sort), [record]);
    const watched = record.user_status.watched_eps;
    const groups = EPISODE_GROUPS
        .map((g) => ({ ...g, eps: sorted.filter((e) => g.types.includes(e.type)) }))
        .filter((g) => g.eps.length > 0);
    if (groups.length === 0) return <p className={styles.muted}>暂无剧集信息</p>;

    return (
        <div className={styles.epGroups}>
            {groups.map((g) => (
                <div key={g.label} className={styles.epGroup}>
                    <div className={styles.epGroupHead}>
                        <span>{g.label}</span>
                        <span className={styles.muted}>{g.eps.length} 个</span>
                    </div>
                    <div className={styles.epGrid}>
                        {g.eps.map((ep, i) => {
                            const isSeen = g.types.includes(EpisodeType.NORMAL) && i < watched;
                            return (
                                <Tooltip key={ep.id} selectable width={420} content={<EpisodeCard ep={ep} />}>
                                    <button
                                        type="button"
                                        className={`${styles.ep} ${isSeen ? styles.epSeen : ""}`}
                                        aria-label={`第 ${ep.sort} 集 ${ep.name_cn || ep.name}${isSeen ? ", 已看" : ""}`}
                                    >
                                        {ep.sort}
                                    </button>
                                </Tooltip>
                            );
                        })}
                    </div>
                </div>
            ))}
        </div>
    );
}

// ---------------------------------------------------------------- 角色

function ActorCard ({ actor, cvTo }: { actor: Actor; cvTo: string }): React.ReactElement {
    return (
        <div className={styles.actorCard}>
            <div className={styles.actorSide}>
                <ZoomImage
                    src={toAnimeCdnUrl(`/data/cv/${actor.id}.jpg`)}
                    alt={actor.name}
                    loading="lazy"
                    decoding="async"
                    className={styles.actorImg}
                />
                <a href={`https://bgm.tv/person/${actor.id}`} target="_blank" rel="noopener noreferrer">{actor.name}</a>
                <Link to={cvTo}>出演番剧</Link>
            </div>
            <p className={styles.actorDesc}>{actor.short_summary || "暂无简介"}</p>
        </div>
    );
}

/**
 * 一张角色卡. 角色图挂着右键 / 长按菜单 (复制生图提示词、下载角色图), 本地模式多出开发者用途的探头立绘提示词.
 * .agents/notes/implemented/feature/2026-11-03-anime-character-context-menu.md
 */
function CharacterCard ({ char, actorMap }: { char: Character; actorMap: Map<number, Actor> }): React.ReactElement {
    const birthday = char.birth_month && char.birth_day ? `${char.birth_month}月${char.birth_day}日` : "";
    const actors = char.actor_ids.map((id) => actorMap.get(id)).filter((x): x is Actor => !!x);
    const imageUrl = toAnimeCdnUrl(`/data/kyara/${char.id}.jpg`);
    const { className: menuClass, ...menuHandlers } = useCharacterMenuTrigger({
        id: char.id,
        name: char.name_cn || char.name,
        originalName: char.name,
        imageUrl,
        imageFallbackUrl: char.image_url || undefined,
        relation: char.relation,
    });
    return (
        <li className={styles.charCard}>
            <Tooltip
                width={360}
                content={
                    <div className={styles.charTip}>
                        <strong>{char.name_cn || char.name}</strong>
                        {birthday && <span className={styles.muted}>生日 {birthday}</span>}
                        <p>{char.summary || "暂无简介"}</p>
                    </div>
                }
            >
                <a
                    href={`https://bgm.tv/character/${char.id}`}
                    target="_blank"
                    rel="noopener noreferrer"
                    className={`${styles.charImg} ${menuClass}`}
                    style={{ backgroundImage: `url(${imageUrl})` }}
                    aria-label={char.name_cn || char.name}
                    {...menuHandlers}
                />
            </Tooltip>
            <div className={styles.charBody}>
                <p className={styles.charName}>{char.name_cn || char.name}</p>
                {char.name_cn && char.name !== char.name_cn && <p className={styles.charOrig}>{char.name}</p>}
                <span className={`${styles.roleBadge} ${char.relation === "主角" ? styles.roleMain : ""}`}>{char.relation}</span>
                {actors.length > 0 && (
                    <p className={styles.charCv}>
                        CV{" "}
                        {actors.map((actor, i) => (
                            <React.Fragment key={actor.id}>
                                {i > 0 && " / "}
                                <Tooltip selectable width={480} content={<ActorCard actor={actor} cvTo={`/anime/cv?id=${actor.id}`} />}>
                                    <span className={styles.cvName} tabIndex={0}>{actor.name}</span>
                                </Tooltip>
                            </React.Fragment>
                        ))}
                    </p>
                )}
            </div>
        </li>
    );
}

function Characters ({ record, actorMap, anime }: { record: ANiMeRecord; actorMap: Map<number, Actor>; anime: CharacterMenuAnime }): React.ReactElement {
    const [all, setAll] = React.useState(false);
    const chars = React.useMemo(
        () => [...record.anime_data.characters].sort((x, y) => orderIndex(CHARACTER_ORDER, x.relation) - orderIndex(CHARACTER_ORDER, y.relation)),
        [record],
    );
    if (chars.length === 0) return <p className={styles.muted}>暂无角色信息</p>;
    const shown = all ? chars : chars.slice(0, CHARACTER_PREVIEW);
    return (
        <CharacterMenuScope anime={anime}>
            <ul className={styles.charGrid}>
                {shown.map((c) => <CharacterCard key={c.id} char={c} actorMap={actorMap} />)}
            </ul>
            {chars.length > CHARACTER_PREVIEW && (
                <button type="button" className={styles.linkButton} onClick={() => setAll((v) => !v)} aria-expanded={all}>
                    {all ? "收起" : `展开全部 ${chars.length} 个角色`}
                </button>
            )}
        </CharacterMenuScope>
    );
}

// ---------------------------------------------------------------- 关联与相似

function PosterLink ({ id, name, img, inLibrary, sub }: { id: number; name: string; img: string; inLibrary: boolean; sub?: string }): React.ReactElement {
    const body = (
        <>
            <span className={styles.posterImg} style={{ backgroundImage: `url(${img})` }} />
            <span className={styles.posterName}>{name}</span>
            {sub && <span className={styles.posterSub}>{sub}</span>}
        </>
    );
    return inLibrary
        ? <Link to={`/anime/details?id=${id}`} className={styles.poster}>{body}</Link>
        : <a href={`https://bgm.tv/subject/${id}`} target="_blank" rel="noopener noreferrer" className={styles.poster}>{body}<span className={styles.posterExt}>bgm ↗</span></a>;
}

function Relations ({ record, libraryIds }: { record: ANiMeRecord; libraryIds: ReadonlySet<number> }): React.ReactElement {
    const grouped = new Map<string, Relation[]>();
    for (const rel of record.anime_data.relations) {
        const list = grouped.get(rel.relation) ?? [];
        list.push(rel);
        grouped.set(rel.relation, list);
    }
    const keys = [...grouped.keys()].sort((x, y) => orderIndex(RELATION_ORDER, x) - orderIndex(RELATION_ORDER, y));
    if (keys.length === 0) return <p className={styles.muted}>暂无关联条目</p>;
    return (
        <div className={styles.relGroups}>
            {keys.map((k) => (
                <div key={k}>
                    <div className={styles.epGroupHead}><span>{k}</span></div>
                    <div className={styles.rail}>
                        {(grouped.get(k) ?? []).map((rel) => (
                            <PosterLink
                                key={rel.id}
                                id={rel.id}
                                name={rel.name_cn || rel.name}
                                img={toAnimeCdnUrl(`/data/relation/${rel.id}.jpg`)}
                                inLibrary={libraryIds.has(rel.id)}
                            />
                        ))}
                    </div>
                </div>
            ))}
        </div>
    );
}

/** 我库里与这部共享标签最多的 6 部 (同分按 Bangumi 评分) */
function similarRecords (record: ANiMeRecord, records: readonly ANiMeRecord[]): { r: ANiMeRecord; shared: string[] }[] {
    const own = new Set(getRecordTags(record));
    if (own.size === 0) return [];
    return records
        .filter((r) => r.anime_data.id !== record.anime_data.id)
        .map((r) => ({ r, shared: getRecordTags(r).filter((t) => own.has(t)) }))
        .filter((x) => x.shared.length > 0)
        .sort((x, y) => y.shared.length - x.shared.length || (y.r.anime_data.score || 0) - (x.r.anime_data.score || 0))
        .slice(0, 6);
}

// ---------------------------------------------------------------- 页面

function Section ({ id, title, children, extra }: { id: string; title: string; children: React.ReactNode; extra?: React.ReactNode }): React.ReactElement {
    return (
        <section className={styles.section} aria-labelledby={id}>
            <div className={styles.sectionHead}>
                <Heading as="h2" id={id} className={styles.sectionTitle}>{title}</Heading>
                {extra}
            </div>
            {children}
        </section>
    );
}

/**
 * 番剧详情. 冷启动 (直接打开 /anime/details?id=...) 的骨架 / 错误 / 404 三态保留.
 * 信息架构按"这是什么 -> 我看到哪 -> 我怎么评 -> 深挖"排: Hero (封面、标题、季度 / 集数 / 评分、观看状态与进度条、标签、操作),
 * 短评引用卡, 页内目录 (宽屏左侧竖排侧栏并高亮当前段, 窄屏横滑细条, 不再吸顶横在内容上), 简介 (长文折叠), 剧集 (按类型分组, 已看正片高亮, 点开看单集详情), 角色 (主角在前, 默认 12 个),
 * 关联条目 (在库里的走站内详情, 不在的去 bgm), 相似推荐 (库里共享标签最多的 6 部), Giscus (term 不变, 旧评论不丢).
 * 颜色走 Infima 变量, 明暗主题都可读. 剧集复制后再排序, 不再原地 sort 冻结的记录.
 * Hero 改为主题色卡片 (封面 2:3 定宽 220px, 虚化封面只剩淡光晕), 标题 / 原名 / 元信息三级层次.
 * .agents/notes/implemented/feature/2026-11-01-anime-detail-redesign.md
 * .agents/notes/implemented/feature/2026-11-01-anime-detail-sidebar-and-hero.md
 */
export default function AnimeDetailPage (): React.ReactElement {
    const query = useQuery();
    const id = query.get("id");
    const { siteConfig } = useDocusaurusContext();
    const isBrowser = useIsBrowser();
    const { records, status, error, retry } = useAnimeRecordsState(siteConfig.baseUrl);
    const actorMap = useActorMap(siteConfig.baseUrl);
    const notFoundImageUrl = useBaseUrl("/anime/img/misaka-404.png");

    const validId = !!id && /^\d+$/.test(id);
    const record = validId ? records.find((r) => r.anime_data.id === Number(id)) : undefined;
    const libraryIds = React.useMemo(() => new Set(records.map((r) => r.anime_data.id)), [records]);
    const similar = React.useMemo(() => (record ? similarRecords(record, records) : []), [record, records]);

    if (!record && (!isBrowser || (validId && status === "loading"))) {
        return (
            <Layout title={"アニメ"}>
                <div className={styles.shell} aria-busy="true" aria-label="正在加载番剧详情">
                    <div className={styles.skeletonHero}>
                        <div className={styles.skeletonCover} />
                        <div className={styles.skeletonLines}>
                            <div className={`${styles.skeletonLine} ${styles.skeletonTitle}`} />
                            <div className={`${styles.skeletonLine} ${styles.skeletonW40}`} />
                            <div className={`${styles.skeletonLine} ${styles.skeletonW75}`} />
                            <div className={`${styles.skeletonLine} ${styles.skeletonW55}`} />
                        </div>
                    </div>
                </div>
            </Layout>
        );
    }

    if (!record && validId && status === "error") {
        return (
            <Layout title={"アニメ"}>
                <div className={`${styles.shell} ${styles.center}`} role="alert">
                    <Heading as="h2" id="load-error">追番记录没能加载出来</Heading>
                    <p>数据从 HXLoLi-ANiMe 的 CDN 拉取, 可能是网络或 CDN 节点问题{error ? `: ${error}` : ""}.</p>
                    <button type="button" className="button button--primary" onClick={retry}>重试</button>
                </div>
            </Layout>
        );
    }

    if (!record) {
        return (
            <Layout title={"アニメ 404 NOT FOUND"}>
                <div className={`${styles.shell} ${styles.center}`}>
                    <Heading as="h2" id="404">404 NOT FOUND</Heading>
                    <p>
                        追番记录里没有这部
                        {validId ? <>, 请尝试: <a href={`https://bgm.tv/subject/${id}`} target="_blank" rel="noopener noreferrer">{`bgm.tv/subject/${id}`}</a></> : null}
                    </p>
                    <p><Link to="/anime">← 回番剧库</Link></p>
                    <ZoomImage src={notFoundImageUrl} alt="404" className={styles.notFoundImg} />
                </div>
            </Layout>
        );
    }

    const a = record.anime_data;
    const coverUrl = toAnimeCdnUrl(`/data/anime/${a.id}.jpg`);
    const comment = (record.user_status.comment ?? "").trim();
    const sections = SECTIONS.filter((s) => s.id !== "similar" || similar.length > 0);
    const menuAnime: CharacterMenuAnime = {
        name: a.name_cn || a.name,
        originalName: a.name,
        coverUrl,
        coverFallbackUrl: a.image_url || undefined,
    };

    return (
        <>
            <PageMetadata title={a.name_cn || a.name} description={a.short_summary || a.summary} />
            <Layout title={a.name_cn || a.name}>
                <div className={styles.shell}>
                    <Link to="/anime" className={styles.backLink}>← 番剧库</Link>
                    <Hero record={record} coverUrl={coverUrl} />

                    {comment && (
                        <blockquote className={styles.comment}>
                            <span className={styles.commentLabel}>我的短评</span>
                            {comment}
                        </blockquote>
                    )}

                    {OverlayEditor && isBrowser && (
                        <div className={styles.overlayEditor}>
                            <React.Suspense fallback={null}>
                                <OverlayEditor record={record} />
                            </React.Suspense>
                        </div>
                    )}

                    <div className={styles.layout}>
                        <SectionNav sections={sections} />

                        <div className={styles.main}>
                            <Section id="summary" title="剧情简介">
                                <Summary text={a.summary || a.short_summary} />
                            </Section>

                            <Section id="eps" title="剧集" extra={<span className={styles.muted}>悬停或点开看单集详情</span>}>
                                <Episodes record={record} />
                            </Section>

                            <Section id="characters" title="角色" extra={<span className={styles.muted}>{a.characters.length} 个</span>}>
                                <Characters record={record} actorMap={actorMap} anime={menuAnime} />
                            </Section>

                            <Section id="relations" title="关联条目">
                                <Relations record={record} libraryIds={libraryIds} />
                            </Section>

                            {similar.length > 0 && (
                                <Section id="similar" title="相似推荐" extra={<span className={styles.muted}>库里标签最接近的番</span>}>
                                    <div className={styles.rail}>
                                        {similar.map(({ r, shared }) => (
                                            <PosterLink
                                                key={r.anime_data.id}
                                                id={r.anime_data.id}
                                                name={r.anime_data.name_cn || r.anime_data.name}
                                                img={toAnimeCdnUrl(`/data/anime/${r.anime_data.id}.jpg`)}
                                                inLibrary
                                                sub={shared.map((t) => `#${t}`).join(" ")}
                                            />
                                        ))}
                                    </div>
                                </Section>
                            )}

                            <Section id="comments" title="讨论">
                                <HXGiscus term={`anime/details?id=${a.id}「${a.name}」`} />
                            </Section>

                            <p className={styles.source}>
                                数据来源 <a href={`https://bgm.tv/subject/${a.id}`} target="_blank" rel="noopener noreferrer">Bangumi.tv</a>
                            </p>
                        </div>
                    </div>
                </div>
            </Layout>
        </>
    );
}
