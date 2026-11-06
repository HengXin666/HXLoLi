import React from "react";
import { AnimatePresence, motion } from "framer-motion";
import { hxEase, useHxReducedMotion } from "@site/src/components/anime/ui";
import type { KyaraImage } from "@site/src/utils/kyara/types";
import styles from "./Kyara.module.css";

/** 来源图标: 仓库 (static) / 本地 (IndexedDB) / pixiv / x, 只看 sourceUrl 的域名, 不做网络请求 */
export function SourceIcon ({ image }: { image: KyaraImage }): React.ReactElement {
    const url = image.sourceUrl ?? "";
    if (/pixiv\.net/i.test(url)) {
        return <span className={styles.srcIcon} data-src="pixiv" title="pixiv" aria-label="pixiv">P</span>;
    }
    if (/(?:^|\/\/)(?:www\.|mobile\.)?(?:x|twitter)\.com/i.test(url)) {
        return <span className={styles.srcIcon} data-src="x" title="X" aria-label="X">𝕏</span>;
    }
    if (image.source === "local") {
        return <span className={styles.srcIcon} data-src="local" title="本地" aria-label="本地">本</span>;
    }
    return <span className={styles.srcIcon} data-src="static" title="仓库" aria-label="仓库">庫</span>;
}

/** CSS 自定义属性 (--kyara-*) 进 style; React.CSSProperties 不收自定义键, 这里集中做一次断言 */
export const cssVars = (vars: Record<string, string>): React.CSSProperties => vars as unknown as React.CSSProperties;

export const seqLabel = (seq: number): string => String(seq).padStart(3, "0");

/** 用 w/h 预留比例防止瀑布流跳动; 没有尺寸时按 3:4 竖图占位 */
export const ratioOf = (img: KyaraImage): string => (img.w && img.h ? `${img.w} / ${img.h}` : "3 / 4");

/** 本地图 key 用 local: 前缀, 与同名仓库文件不撞 layoutId */
export const keyOf = (img: KyaraImage): string => `${img.source ?? "static"}:${img.file}`;

const FOCUSABLE = 'a[href], button:not([disabled]), input, select, textarea, [tabindex]:not([tabindex="-1"])';

/** 打开对话框期间: 锁滚动 + 焦点困在框内, 关闭时把焦点还给打开它的元素 */
export function useDialogFocus (ref: React.RefObject<HTMLElement | null>, onClose: () => void): void {
    const closeRef = React.useRef(onClose);
    closeRef.current = onClose;
    React.useEffect(() => {
        const root = ref.current;
        if (!root) return undefined;
        const back = document.activeElement as HTMLElement | null;
        const prevOverflow = document.body.style.overflow;
        document.body.style.overflow = "hidden";
        (root.querySelector<HTMLElement>("[data-autofocus]") ?? root.querySelector<HTMLElement>(FOCUSABLE) ?? root).focus();
        const onKey = (e: KeyboardEvent): void => {
            if (e.key === "Escape") {
                e.preventDefault();
                closeRef.current();
                return;
            }
            if (e.key !== "Tab") return;
            const items = Array.from(root.querySelectorAll<HTMLElement>(FOCUSABLE)).filter((el) => el.offsetParent !== null);
            if (items.length === 0) {
                e.preventDefault();
                return;
            }
            const first = items[0];
            const last = items[items.length - 1];
            if (e.shiftKey && document.activeElement === first) {
                e.preventDefault();
                last.focus();
            } else if (!e.shiftKey && document.activeElement === last) {
                e.preventDefault();
                first.focus();
            }
        };
        document.addEventListener("keydown", onKey);
        return () => {
            document.removeEventListener("keydown", onKey);
            document.body.style.overflow = prevOverflow;
            back?.focus?.();
        };
    }, [ref]);
}

interface LightboxProps {
    images: KyaraImage[];
    index: number;
    onIndex: (i: number) => void;
    onClose: () => void;
}

/**
 * 灯箱: 与瀑布流缩略图共享 layoutId (kyara-{source}:{file}) 从原位放大; ←/→ 翻页, Esc 关闭, 底部胶片条可点选, 显示来源链接.
 * reduced-motion 下不做共享布局与滑动, 直接出现.
 */
export default function Lightbox ({ images, index, onIndex, onClose }: LightboxProps): React.ReactElement {
    const reduced = useHxReducedMotion();
    const ref = React.useRef<HTMLDivElement>(null);
    const stripRef = React.useRef<HTMLOListElement>(null);
    const [dir, setDir] = React.useState(0);
    const img = images[index];
    const n = images.length;

    const go = React.useCallback((step: number) => {
        if (n < 2) return;
        setDir(step);
        onIndex((index + step + n) % n);
    }, [index, n, onIndex]);

    useDialogFocus(ref, onClose);

    React.useEffect(() => {
        const onKey = (e: KeyboardEvent): void => {
            if (e.key === "ArrowLeft") {
                e.preventDefault();
                go(-1);
            } else if (e.key === "ArrowRight") {
                e.preventDefault();
                go(1);
            }
        };
        document.addEventListener("keydown", onKey);
        return () => document.removeEventListener("keydown", onKey);
    }, [go]);

    // 当前帧滚进胶片条可视区
    React.useEffect(() => {
        const el = stripRef.current?.querySelector<HTMLElement>(`[data-i="${index}"]`);
        el?.scrollIntoView({ block: "nearest", inline: "center", behavior: reduced ? "auto" : "smooth" });
    }, [index, reduced]);

    // 预取相邻两张, 翻页不闪白
    React.useEffect(() => {
        if (n < 2) return;
        for (const j of [(index + 1) % n, (index - 1 + n) % n]) {
            const pre = new Image();
            pre.src = images[j].src;
        }
    }, [index, images, n]);

    const t = reduced ? { duration: 0 } : { duration: 0.42, ease: hxEase };
    const label = `${seqLabel(img.seq)} ${img.title}${img.author ? ` · @${img.author}` : ""}`;

    return (
        <motion.div
            ref={ref}
            className={styles.lightbox}
            role="dialog"
            aria-modal="true"
            aria-label={`大图: ${label}`}
            tabIndex={-1}
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={reduced ? { duration: 0 } : { duration: 0.24 }}
        >
            <button type="button" className={styles.lbBackdrop} aria-label="关闭" tabIndex={-1} onClick={onClose} />

            <div className={styles.lbTop}>
                <span className={styles.lbCounter}>{index + 1} / {n}</span>
                <button type="button" className={styles.lbClose} onClick={onClose} aria-label="关闭 (Esc)" data-autofocus>✕</button>
            </div>

            <div className={styles.lbStage}>
                {n > 1 && (
                    <button type="button" className={styles.lbNav} data-dir="prev" onClick={() => go(-1)} aria-label="上一张 (←)">‹</button>
                )}
                <AnimatePresence initial={false} mode="popLayout" custom={dir}>
                    <motion.img
                        key={keyOf(img)}
                        layoutId={reduced ? undefined : `kyara-${keyOf(img)}`}
                        src={img.src}
                        alt={label}
                        className={styles.lbImg}
                        style={{ aspectRatio: ratioOf(img) }}
                        custom={dir}
                        initial={reduced ? false : { opacity: 0, x: dir * 48 }}
                        animate={{ opacity: 1, x: 0 }}
                        exit={reduced ? { opacity: 0 } : { opacity: 0, x: dir * -48 }}
                        transition={t}
                        drag={reduced || n < 2 ? false : "x"}
                        dragConstraints={{ left: 0, right: 0 }}
                        dragElastic={0.25}
                        onDragEnd={(_, info) => {
                            if (info.offset.x < -80) go(1);
                            else if (info.offset.x > 80) go(-1);
                        }}
                    />
                </AnimatePresence>
                {n > 1 && (
                    <button type="button" className={styles.lbNav} data-dir="next" onClick={() => go(1)} aria-label="下一张 (→)">›</button>
                )}
            </div>

            <div className={styles.lbInfo}>
                <span className={styles.seq} data-static>{seqLabel(img.seq)}</span>
                <span className={styles.lbTitle}>{img.title}</span>
                {img.author && <span className={styles.capAuthor}>@{img.author}</span>}
                <SourceIcon image={img} />
                {img.sourceUrl && (
                    <a className={styles.lbSource} href={img.sourceUrl} target="_blank" rel="noreferrer noopener">来源</a>
                )}
                <a className={styles.lbSource} href={img.src} target="_blank" rel="noreferrer noopener">原图</a>
            </div>

            {n > 1 && (
                <ol ref={stripRef} className={styles.lbStrip} aria-label="胶片条">
                    {images.map((it, i) => (
                        <li key={keyOf(it)} data-i={i}>
                            <button
                                type="button"
                                className={styles.lbThumb}
                                aria-current={i === index ? "true" : undefined}
                                aria-label={`第 ${i + 1} 张 ${it.title}`}
                                onClick={() => {
                                    setDir(i > index ? 1 : -1);
                                    onIndex(i);
                                }}
                            >
                                <img src={it.src} alt="" loading="lazy" decoding="async" />
                            </button>
                        </li>
                    ))}
                </ol>
            )}
        </motion.div>
    );
}
