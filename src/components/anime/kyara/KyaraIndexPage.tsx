import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import { useBaseUrlUtils } from "@docusaurus/useBaseUrl";
import { useLocation } from "@docusaurus/router";
import { AnimatePresence } from "framer-motion";
import { HxPills, HxState, RevealList } from "@site/src/components/anime/ui";
import { StageHead, StagePaper } from "@site/src/components/anime/stage/Stage";
import { kyaraWorkKey, mergeKyaraEntries } from "@site/src/utils/kyara/link";
import { useKyaraLocalCharacters, useKyaraLocalMode } from "@site/src/utils/kyara/localStore";
import type { KyaraIndex, KyaraIndexEntry } from "@site/src/utils/kyara/types";
import { Spotlight } from "./Spotlight";
import KyaraCard, { LocalSwitch } from "./KyaraCard";
import NewKyaraDialog from "./NewKyaraDialog";
import { useKyaraDev } from "./devChunks";
import styles from "./Kyara.module.css";

interface Group {
    key: string;
    /** 锚点: 有番剧条目时 work-{subjectId}, 详情页的「キャラ画集」入口跳这里 */
    anchor: string;
    title: string;
    subjectId?: number;
    entries: KyaraIndexEntry[];
}

const ALL = "__all";

function groupByWork (entries: KyaraIndexEntry[]): Group[] {
    const map = new Map<string, Group>();
    for (const e of entries) {
        const key = kyaraWorkKey(e);
        let g = map.get(key);
        if (!g) {
            const sid = e.meta.bangumiSubjectIds?.[0];
            g = {
                key,
                anchor: sid ? `work-${sid}` : `work-${encodeURIComponent(e.meta.work ?? "other")}`,
                title: e.meta.work || "其他角色",
                ...(sid ? { subjectId: sid } : {}),
                entries: [],
            };
            map.set(key, g);
        }
        g.entries.push(e);
    }
    return [...map.values()].sort((a, b) => (a.key === "other" ? 1 : b.key === "other" ? -1 : b.entries.length - a.entries.length || a.title.localeCompare(b.title, "zh")));
}

interface Props {
    /** 由 plugins/kyara-gallery-plugin.mjs 经 modules 注入 */
    index: KyaraIndex;
}

/**
 * /anime/kyara: 产品页式的角色画集索引.
 * 主视觉 Spotlight (6s 轮换一位角色, 主题色随之变化) → 统计 → 作品筛选 + 按番剧分组的角色卡 (悬停轮换配图).
 * 仓库角色来自插件注入; 本地模式开启时并入这个浏览器里的本地角色 (带「本地」角标), 末尾多一张「＋ 新角色」.
 */
export default function KyaraIndexPage ({ index }: Props): React.ReactElement {
    const { withBaseUrl } = useBaseUrlUtils();
    const location = useLocation();
    const dev = useKyaraDev();
    const [localOn, setLocalOn] = useKyaraLocalMode();
    const { characters: locals } = useKyaraLocalCharacters(localOn);
    const [creating, setCreating] = React.useState(false);

    const statics = React.useMemo<KyaraIndexEntry[]>(
        () => index.characters.map((c) => ({
            ...c,
            cover: c.cover ? withBaseUrl(c.cover) : null,
            previews: (c.previews ?? []).map((p) => withBaseUrl(p)),
        })),
        [index.characters, withBaseUrl],
    );
    const entries = React.useMemo(() => mergeKyaraEntries(statics, locals), [statics, locals]);
    const groups = React.useMemo(() => groupByWork(entries), [entries]);

    const [work, setWork] = React.useState<string>(ALL);
    // 从番剧详情页带着 #work-{id} 进来时预选该作品
    React.useEffect(() => {
        const hash = decodeURIComponent(location.hash.replace(/^#/, ""));
        const hit = groups.find((g) => g.anchor === hash || encodeURIComponent(g.anchor) === location.hash.replace(/^#/, ""));
        if (hit) setWork(hit.key);
    }, [location.hash, groups]);

    const shownGroups = work === ALL ? groups : groups.filter((g) => g.key === work);
    const totalImages = entries.reduce((s, c) => s + c.count, 0);
    const totalAuthors = entries.reduce((s, c) => s + (c.authors ?? 0), 0);
    const workCount = groups.filter((g) => g.key !== "other").length;

    const newCard = localOn ? (
        <button type="button" className={styles.newCard} onClick={() => setCreating(true)}>
            <span className={styles.newPlus} aria-hidden>＋</span>
            <b>新角色</b>
            <small>在这个浏览器里先配起来</small>
        </button>
    ) : null;

    let body: React.ReactNode;
    if (entries.length === 0) {
        body = (
            <div className={styles.emptyBlock}>
                <HxState
                    kind="empty"
                    title="画集还在准备中"
                    body={localOn ? "点「新角色」, 在这个浏览器里给喜欢的角色配一面墙." : "打开「本地模式」, 就能在这个浏览器里给喜欢的角色配图, 不会上传."}
                />
                {localOn ? <div className={styles.cardGrid}>{newCard}</div> : (
                    <div className={styles.emptyActions}><LocalSwitch on={localOn} onChange={setLocalOn} /></div>
                )}
                {dev && <dev.KyaraGuide />}
            </div>
        );
    } else {
        body = (
            <>
                {groups.length > 1 && (
                    <div className={styles.filterRow}>
                        <HxPills<string>
                            aria-label="按作品筛选"
                            current={work}
                            onPick={setWork}
                            items={[
                                { key: ALL, label: "全部", hint: entries.length },
                                ...groups.map((g) => ({ key: g.key, label: g.title, hint: g.entries.length })),
                            ]}
                        />
                    </div>
                )}
                {shownGroups.map((g, gi) => (
                    <section key={g.key} className={styles.workSection} aria-labelledby={g.anchor}>
                        <div className={styles.workHead}>
                            <h3 id={g.anchor} className={styles.workTitle}>{g.title}</h3>
                            <span className={styles.workCount}>{g.entries.length} 位</span>
                            {g.subjectId && <Link className={styles.workLink} to={`/anime/details?id=${g.subjectId}`}>番剧详情 →</Link>}
                        </div>
                        <RevealList as="div" className={styles.cardGrid} aria-label={g.title}>
                            {g.entries.map((e, i) => <KyaraCard key={e.meta.name} entry={e} eager={gi === 0 && i < 4} />)}
                        </RevealList>
                    </section>
                ))}
                {newCard && <div className={styles.cardGrid}>{newCard}</div>}
            </>
        );
    }

    return (
        <Layout title="キャラ · 角色画集" description="HXLoLi 的角色画集: 喜欢的角色, 一人一面墙">
            <StagePaper className={styles.page}>
                <nav className={styles.crumbs} aria-label="面包屑">
                    <Link to="/anime">追番记录</Link>
                    <span aria-hidden>/</span>
                    <span aria-current="page">キャラ</span>
                </nav>

                <header className={styles.indexHead}>
                    <div>
                        <p className={styles.infoKicker}>HXLoLi のキャラ</p>
                        <h1 className={styles.indexTitle}>キャラ<span>角色画集</span></h1>
                        <p className={styles.indexSub}>喜欢的角色, 一人一面墙. 点开她, 一张张看过去.</p>
                    </div>
                    <LocalSwitch on={localOn} onChange={setLocalOn} />
                </header>

                <Spotlight entries={entries} />

                <dl className={styles.specs} data-wide>
                    <div className={styles.spec}><dd>{entries.length}</dd><dt>角色</dt></div>
                    <div className={styles.spec}><dd>{totalImages}</dd><dt>配图</dt></div>
                    <div className={styles.spec}><dd>{totalAuthors}</dd><dt>画师</dt></div>
                    <div className={styles.spec}><dd>{workCount}</dd><dt>作品</dt></div>
                </dl>

                <section className={styles.section}>
                    <StageHead title="角色" count={`${entries.length} 位`} id="kyara" />
                    {body}
                </section>

                <AnimatePresence>
                    {creating && <NewKyaraDialog key="new" entries={entries} dev={dev} onClose={() => setCreating(false)} />}
                </AnimatePresence>
            </StagePaper>
        </Layout>
    );
}
