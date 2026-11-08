import React from "react";
import Link from "@docusaurus/Link";
import { useHxReducedMotion } from "@site/src/components/anime/ui";
import type { KyaraIndexEntry } from "@site/src/utils/kyara/types";
import { kyaraHref } from "@site/src/utils/kyara/link";
import { cssVars } from "./Lightbox";
import styles from "./Kyara.module.css";

const CYCLE_MS = 1400;

/**
 * 角色卡: 3:4 竖卡, 静止时显示封面; 悬停或键盘聚焦时在前几张图之间缓慢轮换 (只在那时才加载后几张),
 * 底部玻璃条 = 名字 / 作品 / 张数, 描边与光晕取角色 accent.
 */
export default function KyaraCard ({ entry, eager = false }: { entry: KyaraIndexEntry; eager?: boolean }): React.ReactElement {
    const reduced = useHxReducedMotion();
    const frames = React.useMemo(() => {
        const list = [entry.cover, ...(entry.previews ?? [])].filter((x): x is string => !!x);
        return [...new Set(list)].slice(0, 4);
    }, [entry.cover, entry.previews]);
    const [active, setActive] = React.useState(false);
    const [i, setI] = React.useState(0);

    React.useEffect(() => {
        if (!active || reduced || frames.length < 2) {
            setI(0);
            return undefined;
        }
        const t = window.setInterval(() => setI((x) => (x + 1) % frames.length), CYCLE_MS);
        return () => window.clearInterval(t);
    }, [active, reduced, frames.length]);

    const accent = entry.meta.accent || "var(--hx-accent)";
    return (
        <Link
            to={kyaraHref(entry)}
            className={styles.card}
            style={cssVars({ "--kyara-accent": accent })}
            onMouseEnter={() => setActive(true)}
            onMouseLeave={() => setActive(false)}
            onFocus={() => setActive(true)}
            onBlur={() => setActive(false)}
        >
            <span className={styles.cardMedia}>
                {frames.length === 0 ? (
                    <span className={styles.cardEmpty} aria-hidden>キャラ</span>
                ) : (
                    frames.map((src, k) => (k === 0 || active) && (
                        <img
                            key={src}
                            src={src}
                            alt=""
                            className={styles.cardImg}
                            data-on={k === i ? "true" : undefined}
                            loading={eager && k === 0 ? "eager" : "lazy"}
                            decoding="async"
                        />
                    ))
                )}
                {entry.local && <span className={styles.cardBadge}>本地</span>}
                {active && frames.length > 1 && (
                    <span className={styles.cardDots} aria-hidden>
                        {frames.map((src, k) => <i key={src} data-on={k === i ? "true" : undefined} />)}
                    </span>
                )}
            </span>
            <span className={styles.cardBody}>
                <span className={styles.cardName}>{entry.meta.displayName}</span>
                {entry.meta.work && <span className={styles.cardWork}>{entry.meta.work}</span>}
                <span className={styles.cardCount}><b>{entry.count}</b> 张</span>
            </span>
        </Link>
    );
}

/** 本地模式开关 (全站一个状态, 见 src/utils/kyara/localStore.ts 的 useKyaraLocalMode) */
export function LocalSwitch ({ on, onChange }: { on: boolean; onChange: (v: boolean) => void }): React.ReactElement {
    return (
        <button
            type="button"
            role="switch"
            aria-checked={on}
            className={styles.localSwitch}
            onClick={() => onChange(!on)}
            title="在这个浏览器里给角色配图, 不会上传"
        >
            <span className={styles.switchTrack} aria-hidden><span className={styles.switchThumb} /></span>
            本地模式
        </button>
    );
}
