import React from "react";
import { motion } from "framer-motion";
import { HxPills, useHxReducedMotion } from "@site/src/components/anime/ui";
import type { KyaraImage } from "@site/src/utils/kyara/types";
import { SourceIcon, cssVars, keyOf, ratioOf, seqLabel } from "./Lightbox";
import styles from "./Kyara.module.css";

const ALL = "__all";
const NONE = "__none";

/**
 * 画廊: 等高行瀑布 (按序号从左到右、从上到下, w/h 预留比例防跳动), 顶部按画师过滤.
 * 本地图排在仓库图后面, 带「本地」来源章. 点开把当前过滤后的列表交给灯箱.
 */
export default function Gallery ({ images, artist, onArtist, onOpen }: {
    images: KyaraImage[];
    artist: string | null;
    onArtist: (a: string | null) => void;
    onOpen: (list: KyaraImage[], index: number) => void;
}): React.ReactElement {
    const reduced = useHxReducedMotion();
    const artists = React.useMemo(() => {
        const m = new Map<string, number>();
        let none = 0;
        for (const img of images) {
            if (img.author) m.set(img.author, (m.get(img.author) ?? 0) + 1);
            else none += 1;
        }
        return { list: [...m.entries()].sort((a, b) => b[1] - a[1] || a[0].localeCompare(b[0])), none };
    }, [images]);

    const shown = React.useMemo(() => {
        if (artist === null) return images;
        if (artist === NONE) return images.filter((i) => !i.author);
        return images.filter((i) => i.author === artist);
    }, [images, artist]);

    const pills = [
        { key: ALL, label: "全部", hint: images.length },
        ...artists.list.map(([a, n]) => ({ key: a, label: `@${a}`, hint: n })),
        ...(artists.none > 0 && artists.list.length > 0 ? [{ key: NONE, label: "未署名", hint: artists.none }] : []),
    ];

    return (
        <>
            {artists.list.length > 1 && (
                <div className={styles.filterRow}>
                    <HxPills<string>
                        items={pills}
                        current={artist ?? ALL}
                        onPick={(k) => onArtist(k === ALL ? null : k)}
                        aria-label="按画师筛选"
                    />
                </div>
            )}
            <div className={styles.masonry} aria-label="配图">
                {shown.map((img, i) => (
                    <figure
                        key={keyOf(img)}
                        className={styles.tile}
                        style={cssVars({ "--kyara-ratio": ratioOf(img), "--r": String(img.w && img.h ? img.w / img.h : 0.75), "--i": String(i % 12) })}
                    >
                        <button
                            type="button"
                            className={styles.tileBtn}
                            onClick={() => onOpen(shown, i)}
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
        </>
    );
}
