import React from "react";
import Link from "@docusaurus/Link";
import { AnimatePresence, motion } from "framer-motion";
import { hxEase, useHxReducedMotion } from "@site/src/components/anime/ui";
import type { KyaraImage, KyaraIndexEntry } from "@site/src/utils/kyara/types";
import { kyaraHref } from "@site/src/utils/kyara/link";
import { cssVars, keyOf, seqLabel } from "./Lightbox";
import styles from "./Kyara.module.css";

const INTERVAL = 6000;

/**
 * 自动轮换: 每 interval 前进一格; 悬停 / 聚焦在舞台内 / 标签页不可见 / reduced-motion 时暂停.
 * 返回 [当前序号, 跳到某格, 舞台要挂的事件].
 */
export function useRotation (count: number, interval = INTERVAL): {
    index: number;
    go: (i: number) => void;
    step: (d: number) => void;
    paused: boolean;
    bind: Pick<React.HTMLAttributes<HTMLElement>, "onMouseEnter" | "onMouseLeave" | "onFocus" | "onBlur">;
} {
    const reduced = useHxReducedMotion();
    const [index, setIndex] = React.useState(0);
    const [hover, setHover] = React.useState(false);
    const [focus, setFocus] = React.useState(false);
    const [hidden, setHidden] = React.useState(false);

    React.useEffect(() => {
        const on = (): void => setHidden(document.visibilityState === "hidden");
        document.addEventListener("visibilitychange", on);
        return () => document.removeEventListener("visibilitychange", on);
    }, []);

    React.useEffect(() => {
        if (index >= count && count > 0) setIndex(0);
    }, [count, index]);

    const paused = reduced || hover || focus || hidden || count < 2;
    React.useEffect(() => {
        if (paused) return undefined;
        const t = window.setTimeout(() => setIndex((i) => (i + 1) % count), interval);
        return () => window.clearTimeout(t);
    }, [paused, index, count, interval]);

    const go = React.useCallback((i: number) => setIndex(((i % count) + count) % count), [count]);
    const step = React.useCallback((d: number) => setIndex((i) => (((i + d) % count) + count) % count), [count]);

    return {
        index: count > 0 ? Math.min(index, count - 1) : 0,
        go,
        step,
        paused,
        bind: {
            onMouseEnter: () => setHover(true),
            onMouseLeave: () => setHover(false),
            onFocus: () => setFocus(true),
            onBlur: (e) => {
                if (!e.currentTarget.contains(e.relatedTarget as Node | null)) setFocus(false);
            },
        },
    };
}

function Dots ({ count, index, onPick, label }: { count: number; index: number; onPick: (i: number) => void; label: (i: number) => string }): React.ReactElement | null {
    if (count < 2) return null;
    return (
        <div className={styles.spotDots} role="group" aria-label="切换">
            {Array.from({ length: count }, (_, i) => (
                <button
                    key={i}
                    type="button"
                    className={styles.spotDot}
                    aria-current={i === index ? "true" : undefined}
                    aria-label={label(i)}
                    onClick={() => onPick(i)}
                />
            ))}
        </div>
    );
}

/**
 * 索引页主视觉: 一次展示一位角色 (封面大图 + 名字 + 作品 + 张数 + 进入画集), 6s 交叉淡入换下一位,
 * 主题色 (--kyara-accent) 跟着当前角色走. 只展示有封面的角色, 最多 6 位.
 */
export function Spotlight ({ entries }: { entries: KyaraIndexEntry[] }): React.ReactElement | null {
    const reduced = useHxReducedMotion();
    const list = React.useMemo(() => entries.filter((e) => e.cover).slice(0, 6), [entries]);
    const rot = useRotation(list.length);
    const cur = list[rot.index];
    if (!cur) return null;
    const accent = cur.meta.accent || "var(--hx-accent)";
    const fade = reduced ? { duration: 0 } : { duration: 0.9, ease: hxEase };

    return (
        <section
            className={styles.spot}
            style={cssVars({ "--kyara-accent": accent })}
            aria-roledescription="carousel"
            aria-label="角色轮播"
            {...rot.bind}
        >
            <div className={styles.spotBack} aria-hidden>
                <AnimatePresence initial={false}>
                    <motion.img
                        key={cur.meta.name}
                        src={cur.cover ?? undefined}
                        alt=""
                        className={styles.spotBlur}
                        initial={{ opacity: 0 }}
                        animate={{ opacity: 1 }}
                        exit={{ opacity: 0 }}
                        transition={fade}
                    />
                </AnimatePresence>
            </div>

            <div className={styles.spotInner}>
                <div className={styles.spotBody} aria-live={rot.paused ? "polite" : "off"}>
                    <AnimatePresence mode="wait" initial={false}>
                        <motion.div
                            key={cur.meta.name}
                            initial={reduced ? false : { opacity: 0, y: 14 }}
                            animate={{ opacity: 1, y: 0 }}
                            exit={reduced ? { opacity: 0 } : { opacity: 0, y: -10 }}
                            transition={reduced ? { duration: 0 } : { duration: 0.45, ease: hxEase }}
                        >
                            <p className={styles.spotKicker}>
                                <span>{String(rot.index + 1).padStart(2, "0")}</span> / {String(list.length).padStart(2, "0")}
                                {cur.meta.work ? <> · {cur.meta.work}</> : null}
                            </p>
                            <h2 className={styles.spotName}>{cur.meta.displayName}</h2>
                            {cur.meta.displayName !== cur.meta.name && <p className={styles.spotAlt}>{cur.meta.name}</p>}
                            <p className={styles.spotMeta}>
                                <b>{cur.count}</b> 张配图{cur.authors ? <> · <b>{cur.authors}</b> 位画师</> : null}
                                {cur.local ? <span className={styles.localTag}>本地</span> : null}
                            </p>
                            <Link to={kyaraHref(cur)} className={styles.btnPrimary}>进入画集 →</Link>
                        </motion.div>
                    </AnimatePresence>
                </div>

                <div className={styles.spotFigure}>
                    <AnimatePresence initial={false}>
                        <motion.div
                            key={cur.meta.name}
                            className={styles.spotFrame}
                            initial={reduced ? { opacity: 0 } : { opacity: 0, scale: 1.04 }}
                            animate={{ opacity: 1, scale: 1 }}
                            exit={{ opacity: 0 }}
                            transition={fade}
                        >
                            <Link to={kyaraHref(cur)} tabIndex={-1} aria-hidden>
                                <img src={cur.cover ?? undefined} alt="" className={styles.spotImg} decoding="async" />
                            </Link>
                        </motion.div>
                    </AnimatePresence>
                    {list.length > 1 && (
                        <div className={styles.spotArrows}>
                            <button type="button" className={styles.spotArrow} onClick={() => rot.step(-1)} aria-label="上一位">‹</button>
                            <button type="button" className={styles.spotArrow} onClick={() => rot.step(1)} aria-label="下一位">›</button>
                        </div>
                    )}
                </div>
            </div>

            <Dots count={list.length} index={rot.index} onPick={rot.go} label={(i) => `第 ${i + 1} 位: ${list[i].meta.displayName}`} />
            {!rot.paused && <span key={rot.index} className={styles.spotProgress} aria-hidden />}
        </section>
    );
}

/**
 * 角色页首屏左栏: 主图交叉淡入轮换 + 缩略胶片条; 点主图打开灯箱.
 */
export function ImageStage ({ images, onOpen }: { images: KyaraImage[]; onOpen: (i: number) => void }): React.ReactElement {
    const reduced = useHxReducedMotion();
    const shown = images.slice(0, 12);
    const rot = useRotation(shown.length);
    const cur = shown[rot.index];
    const stripRef = React.useRef<HTMLOListElement>(null);

    React.useEffect(() => {
        const el = stripRef.current?.querySelector<HTMLElement>(`[data-i="${rot.index}"]`);
        const strip = stripRef.current;
        if (!el || !strip) return;
        // 只滚胶片条自己, 不带动整页
        strip.scrollTo({ left: el.offsetLeft - strip.clientWidth / 2 + el.clientWidth / 2, behavior: reduced ? "auto" : "smooth" });
    }, [rot.index, reduced]);

    if (!cur) {
        return (
            <div className={styles.stage} data-empty>
                <div className={styles.stageMain}>
                    <span className={styles.stageEmpty} aria-hidden>キャラ</span>
                </div>
            </div>
        );
    }

    return (
        <div className={styles.stage} aria-roledescription="carousel" aria-label="配图轮播" {...rot.bind}>
            <div className={styles.stageMain}>
                <AnimatePresence initial={false}>
                    <motion.img
                        key={`b-${keyOf(cur)}`}
                        src={cur.src}
                        alt=""
                        aria-hidden
                        className={styles.stageBlur}
                        initial={{ opacity: 0 }}
                        animate={{ opacity: 1 }}
                        exit={{ opacity: 0 }}
                        transition={reduced ? { duration: 0 } : { duration: 0.8 }}
                    />
                </AnimatePresence>
                <AnimatePresence initial={false}>
                    <motion.button
                        key={keyOf(cur)}
                        type="button"
                        className={styles.stageBtn}
                        onClick={() => onOpen(rot.index)}
                        aria-label={`查看大图 ${seqLabel(cur.seq)} ${cur.title}`}
                        initial={reduced ? { opacity: 0 } : { opacity: 0, scale: 1.03 }}
                        animate={{ opacity: 1, scale: 1 }}
                        exit={{ opacity: 0 }}
                        transition={reduced ? { duration: 0 } : { duration: 0.8, ease: hxEase }}
                    >
                        <img src={cur.src} alt="" className={styles.stageImg} decoding="async" />
                    </motion.button>
                </AnimatePresence>
                <span className={styles.stageCaption}>
                    <span className={styles.seq} data-static>{seqLabel(cur.seq)}</span>
                    <span className={styles.stageTitle}>{cur.title}</span>
                    {cur.author && <span className={styles.capAuthor}>@{cur.author}</span>}
                </span>
                {shown.length > 1 && (
                    <div className={styles.stageArrows}>
                        <button type="button" className={styles.spotArrow} onClick={() => rot.step(-1)} aria-label="上一张">‹</button>
                        <button type="button" className={styles.spotArrow} onClick={() => rot.step(1)} aria-label="下一张">›</button>
                    </div>
                )}
                {!rot.paused && <span key={rot.index} className={styles.spotProgress} aria-hidden />}
            </div>
            {shown.length > 1 && (
                <ol ref={stripRef} className={styles.stageThumbs} aria-label="缩略图">
                    {shown.map((img, i) => (
                        <li key={keyOf(img)} data-i={i}>
                            <button
                                type="button"
                                className={styles.stageThumb}
                                aria-current={i === rot.index ? "true" : undefined}
                                aria-label={`第 ${i + 1} 张 ${img.title}`}
                                onClick={() => rot.go(i)}
                            >
                                <img src={img.src} alt="" loading="lazy" decoding="async" />
                            </button>
                        </li>
                    ))}
                </ol>
            )}
        </div>
    );
}
