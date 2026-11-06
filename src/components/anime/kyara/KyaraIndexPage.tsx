import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import useBaseUrl, { useBaseUrlUtils } from "@docusaurus/useBaseUrl";
import { useHistory, useLocation } from "@docusaurus/router";
import { motion } from "framer-motion";
import { HxCard, HxSkeleton, HxState, RevealList, hxEase, useHxReducedMotion } from "@site/src/components/anime/ui";
import { StageHead, StageHero, paperClass } from "@site/src/components/anime/stage/Stage";
import KyaraGuide from "./KyaraGuide";
import type { KyaraIndex } from "@site/src/utils/kyara/types";
import { KyaraView } from "./KyaraPage";
import { cssVars } from "./Lightbox";
import styles from "./Kyara.module.css";

/** 索引页的本地模式开关: 只决定「＋ 新角色」卡是否出现, 记在 localStorage */
type KyaraCharacter = KyaraIndex["characters"][number];

const LOCAL_KEY = "hx-kyara-local-index";

function useIndexLocal (): [boolean, (v: boolean) => void] {
    const [on, setOn] = React.useState(false);
    React.useEffect(() => {
        try {
            setOn(window.localStorage.getItem(LOCAL_KEY) === "1");
        } catch {
            /* 隐私模式等拿不到 localStorage, 保持关闭 */
        }
    }, []);
    const set = React.useCallback((v: boolean) => {
        setOn(v);
        try {
            window.localStorage.setItem(LOCAL_KEY, v ? "1" : "0");
        } catch {
            /* 同上 */
        }
    }, []);
    return [on, set];
}

type Load = { status: "loading" } | { status: "error"; error: string } | { status: "ok"; index: KyaraIndex };

/** 插件没有经 modules 注入 index 时 (或 dev 下想拿最新), 回落到运行时清单 /kyara/index.json */
function useKyaraIndex (injected?: KyaraIndex): [Load, () => void] {
    const url = useBaseUrl("/kyara/index.json");
    const [state, setState] = React.useState<Load>(injected ? { status: "ok", index: injected } : { status: "loading" });
    const [nonce, setNonce] = React.useState(0);
    React.useEffect(() => {
        if (injected && nonce === 0) return undefined;
        let alive = true;
        setState({ status: "loading" });
        fetch(url, { cache: "no-cache" })
            .then((r) => {
                if (!r.ok) throw new Error(`${r.status} ${r.statusText}`);
                return r.json() as Promise<KyaraIndex>;
            })
            .then((index) => alive && setState({ status: "ok", index }))
            .catch((e: Error) => alive && setState({ status: "error", error: e.message }));
        return () => {
            alive = false;
        };
    }, [url, injected, nonce]);
    return [state, () => setNonce((n) => n + 1)];
}

function NewKyaraCard (): React.ReactElement {
    const history = useHistory();
    const location = useLocation();
    const [name, setName] = React.useState("");
    const clean = name.trim().replace(/[/\\?#%]/g, "");
    return (
        <HxCard accent="var(--hx-accent)" className={styles.newCard}>
            <form
                className={styles.newForm}
                onSubmit={(e) => {
                    e.preventDefault();
                    if (clean) history.push(`${location.pathname}?new=${encodeURIComponent(clean)}`);
                }}
            >
                <span className={styles.newPlus} aria-hidden>＋</span>
                <label htmlFor="kyara-new-name">新角色</label>
                <input
                    id="kyara-new-name"
                    value={name}
                    onChange={(e) => setName(e.target.value)}
                    placeholder="目录名, 例: 春日野穹"
                    autoComplete="off"
                />
                <button type="submit" className={styles.btnPrimary} disabled={!clean}>开始配图</button>
                <small className={styles.dim}>先在本地攒图, 再导出 zip 或在 dev 下写进 static/img/kyara/</small>
            </form>
        </HxCard>
    );
}

interface Props {
    /** 由 plugins/kyara-gallery-plugin.mjs 经 modules 注入时直接用; 否则运行时拉 /kyara/index.json */
    index?: KyaraIndex;
}

/**
 * /anime/kyara: 角色 bento 网格 (封面 + 名字 + 作品 + 张数). 首个角色占 2×2 大格, 其余 1×1; 窄屏单列.
 * 本地模式下末尾多一张「＋ 新角色」卡, 提交后在 ?new={name} 渲染纯本地的 KyaraView (仓库里还没有目录, 没有静态路由).
 */
export default function KyaraIndexPage ({ index }: Props): React.ReactElement {
    const location = useLocation();
    const fresh = new URLSearchParams(location.search).get("new");
    const [state, retry] = useKyaraIndex(index);
    const [localOn, setLocalOn] = useIndexLocal();
    const reduced = useHxReducedMotion();
    const { withBaseUrl } = useBaseUrlUtils();

    if (fresh) {
        return (
            <Layout title={`${fresh} · キャラ`} description={`本地配图: ${fresh}`}>
                <KyaraView kyara={{ meta: { name: fresh, displayName: fresh }, images: [] }} forceLocal />
            </Layout>
        );
    }

    const chars = state.status === "ok" ? state.index.characters : [];
    const totalImages = chars.reduce((s: number, c: KyaraCharacter) => s + c.count, 0);

    let body: React.ReactNode;
    if (state.status === "loading") {
        body = (
            <div aria-busy="true" aria-label="正在加载">
                <HxSkeleton shape="tile" count={6} className={styles.bento} />
            </div>
        );
    } else if (state.status === "error") {
        body = <HxState kind="error" title="角色清单没能加载出来" body={`kyara/index.json: ${state.error}`} onRetry={retry} />;
    } else if (chars.length === 0 && !localOn) {
        body = (
            <KyaraGuide />
        );
    } else {
        body = (
            <RevealList as="div" className={styles.bento} aria-label="角色">
                {chars.map((c: KyaraCharacter, i: number) => {
                    const cover = c.cover ? withBaseUrl(c.cover) : null;
                    return (
                        <div key={c.meta.name} className={styles.bentoCell} data-big={i === 0 && chars.length > 2 ? "true" : undefined}>
                            <Link
                                to={`/anime/kyara/${encodeURIComponent(c.meta.name)}`}
                                className={styles.kyaraCard}
                                style={cssVars({ "--kyara-accent": c.meta.accent || "var(--hx-accent)" })}
                            >
                                <motion.span
                                    className={styles.kyaraCover}
                                    whileHover={reduced ? undefined : { scale: 1.04 }}
                                    transition={{ duration: 0.32, ease: hxEase }}
                                >
                                    {cover ? <img src={cover} alt="" loading={i < 3 ? "eager" : "lazy"} decoding="async" /> : <span className={styles.coverEmpty} aria-hidden>?</span>}
                                </motion.span>
                                <span className={styles.kyaraMeta}>
                                    <span className={styles.kyaraName}>{c.meta.displayName}</span>
                                    {c.meta.work && <span className={styles.kyaraWork}>{c.meta.work}</span>}
                                    <span className={styles.kyaraCount}><b>{c.count}</b> 张</span>
                                </span>
                            </Link>
                        </div>
                    );
                })}
                {localOn && (
                    <div key="__new" className={styles.bentoCell}>
                        <NewKyaraCard />
                    </div>
                )}
            </RevealList>
        );
    }

    return (
        <Layout title="キャラ · 角色配图" description="HXLoLi 的角色配图墙: 按目录约定 static/img/kyara/{name}/001-标题@作者.png 渲染">
            <main className={`${styles.page} ${paperClass}`}>
                <StageHero
                    crumbs={[["追番记录", "/anime"], ["キャラ"]]}
                    kicker="HXLoLi のキャラ"
                    title="キャラ"
                    romaji="kyara · 角色配图"
                    sub="喜欢的角色, 一人一面墙. 放进 static/img/kyara/{角色}/ 的图按 001、002… 的序号排好."
                    stats={[
                        { label: "角色", value: state.status === "ok" ? chars.length : null, suffix: "位" },
                        { label: "配图", value: state.status === "ok" ? totalImages : null, suffix: "张" },
                    ]}
                    backdrop={chars.map((c: KyaraCharacter) => (c.cover ? withBaseUrl(c.cover) : "")).filter(Boolean)}
                    say="想给谁配图? 点开她就好~"
                    aside={
                        <div className={styles.heroTools}>
                            <button
                                type="button"
                                role="switch"
                                aria-checked={localOn}
                                className={styles.localSwitch}
                                onClick={() => setLocalOn(!localOn)}
                            >
                                <span className={styles.switchTrack} aria-hidden><span className={styles.switchThumb} /></span>
                                本地模式
                            </button>
                        </div>
                    }
                />

                <section className={styles.gallery}>
                    <StageHead jp="キャラクター" title="角色" count={state.status === "ok" ? `${chars.length} 位` : undefined} id="kyara" />
                    {body}
                </section>
            </main>
        </Layout>
    );
}
