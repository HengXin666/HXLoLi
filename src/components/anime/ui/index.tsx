import React from "react";
import type { JSX } from "react";
import Link from "@docusaurus/Link";
import useBaseUrl from "@docusaurus/useBaseUrl";
import { motion, useReducedMotion, useScroll, useTransform } from "framer-motion";
import styles from "./ui.module.css";

/*
 * 「HX おすすめ Glass」原子组件: 番剧类页面 (susume / 系列 / kyara) 的共用设计语言.
 * 接口冻结 (见 anime-redesign 计划 1.5): 其他路只引用, 不修改导出签名.
 * 颜色 / 圆角 / 曲线一律取 src/css/custom.css 的 --hx-* 令牌; 用法与参数见同目录 DESIGN.md.
 */

/* framer 的 motion.<tag> 是带缓存的代理, 按字符串取标签不会每次渲染新建组件 */
type AnyMotion = React.ComponentType<Record<string, unknown> & { children?: React.ReactNode }>;
const motionTag = (tag: string): AnyMotion => (motion as unknown as Record<string, AnyMotion>)[tag];

const cx = (...xs: (string | false | null | undefined)[]): string => xs.filter(Boolean).join(" ");

/** 卡片 / 气泡 / chip 的弹簧参数 */
const SPRING = { type: "spring", stiffness: 420, damping: 26 } as const;
/** 入场位移与时长 */
const ENTER_Y = 16;
const ENTER_DUR = 0.55;
/** 列表依次入场的间隔 (秒), 与 --hx-stagger 同值; 超过 8 个循环, 避免长列表底部等太久 */
const STAGGER = 0.04;
const STAGGER_WRAP = 8;
/** Hero 光晕视差的最大偏移 (px) */
const PARALLAX_PX = 24;

/**
 * 用户是否要求减少动效 (prefers-reduced-motion: reduce).
 * SSR 与首帧拿不到媒体查询时按 false 处理; 所有动效组件都先读它, 为 true 时一律瞬时出现、不做视差、不做 lean.
 * .agents/notes/implemented/feature/2026-11-06-anime-ui-kit.md
 */
export function useHxReducedMotion(): boolean {
    return useReducedMotion() === true;
}

export const hxEase: [number, number, number, number] = [0.22, 1, 0.36, 1];

const enterTransition = (delay = 0) => ({ duration: ENTER_DUR, ease: hxEase, delay });

/** 进入视口时淡入上移一次 */
export function Reveal(p: { as?: keyof JSX.IntrinsicElements; delay?: number; className?: string; children: React.ReactNode }): React.ReactElement {
    const reduced = useHxReducedMotion();
    const tag = (p.as ?? "div") as string;
    if (reduced) {
        return React.createElement(tag, { className: p.className }, p.children);
    }
    const M = motionTag(tag);
    return (
        <M
            className={p.className}
            initial={{ opacity: 0, y: ENTER_Y }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true, amount: 0.2 }}
            transition={enterTransition(p.delay ?? 0)}
        >
            {p.children}
        </M>
    );
}

/**
 * 子项依次入场. 每个子节点会被包进一个 <li> (as="ol"|"ul") 或 <div>, 调用方只传内容, 不要自己再包 li.
 * 每一项自己观察视口 (长列表整体 20% 进视口可能永远达不到), 延迟按序号对 8 取模 x 40ms.
 */
export function RevealList(p: { as?: "ol" | "ul" | "div"; className?: string; children: React.ReactNode; "aria-label"?: string }): React.ReactElement {
    const reduced = useHxReducedMotion();
    const as = p.as ?? "ul";
    const itemTag = as === "div" ? "div" : "li";
    const items = React.Children.toArray(p.children);
    const M = motionTag(itemTag);
    return React.createElement(
        as,
        { className: cx(styles.revealList, p.className), "aria-label": p["aria-label"] },
        items.map((child, i) => {
            const key = React.isValidElement(child) && child.key != null ? child.key : i;
            if (reduced) {
                return React.createElement(itemTag, { key, className: styles.revealItem }, child);
            }
            return (
                <M
                    key={key}
                    className={styles.revealItem}
                    initial={{ opacity: 0, y: ENTER_Y }}
                    whileInView={{ opacity: 1, y: 0 }}
                    viewport={{ once: true, amount: 0.2 }}
                    transition={enterTransition((i % STAGGER_WRAP) * STAGGER)}
                >
                    {child}
                </M>
            );
        }),
    );
}

const accentVar = (name: string, accent?: string): React.CSSProperties | undefined =>
    accent ? ({ [name]: accent } as React.CSSProperties) : undefined;

/** 页面 Hero: kicker 小标 + 圆体大标题 + 副标 + 计数 + 猫娘台词气泡, 背景是随滚动轻微视差的三色光晕 */
export function HxHero(p: { kicker: React.ReactNode; title: React.ReactNode; sub?: React.ReactNode; count?: React.ReactNode; say?: React.ReactNode; accent?: string; neko?: "left" | "right" | false; children?: React.ReactNode }): React.ReactElement {
    const reduced = useHxReducedMotion();
    const ref = React.useRef<HTMLElement>(null);
    const { scrollYProgress } = useScroll({ target: ref, offset: ["start start", "end start"] });
    const meshY = useTransform(scrollYProgress, [0, 1], [0, reduced ? 0 : PARALLAX_PX]);
    const neko = p.neko ?? "right";
    /* hero 里的猫娘贴着 hero 边探进来: 右边用切口在右的 neko_left, 左边反之 */
    const nekoSrc = useBaseUrl(neko === "left" ? "/default-img/neko_right.png" : "/default-img/neko_left.png");

    return (
        <section
            ref={ref}
            className={styles.hero}
            data-neko={neko || undefined}
            style={accentVar("--hx-hero-accent", p.accent)}
        >
            <motion.div className={styles.heroMesh} style={reduced ? undefined : { y: meshY }} aria-hidden />
            <div className={styles.heroBody}>
                <Reveal as="p" className={styles.kicker}>{p.kicker}</Reveal>
                <Reveal as="h1" className={styles.heroTitle} delay={0.05}>{p.title}</Reveal>
                {p.sub || p.count ? (
                    <Reveal as="p" className={styles.heroSub} delay={0.1}>
                        {p.sub ? <span>{p.sub}</span> : null}
                        {p.count ? <span className={styles.heroCount}>{p.count}</span> : null}
                    </Reveal>
                ) : null}
                {p.say ? (
                    reduced ? (
                        <p className={styles.say}>{p.say}</p>
                    ) : (
                        <motion.p
                            className={styles.say}
                            initial={{ opacity: 0, scale: 0.92, y: 6 }}
                            whileInView={{ opacity: 1, scale: 1, y: 0 }}
                            viewport={{ once: true }}
                            transition={{ ...SPRING, delay: 0.18 }}
                        >
                            {p.say}
                        </motion.p>
                    )
                ) : null}
                {p.children ? <div className={styles.heroExtra}>{p.children}</div> : null}
            </div>
            {neko ? (
                <img
                    className={styles.heroNeko}
                    data-side={neko}
                    data-reduced={reduced ? "true" : undefined}
                    src={nekoSrc}
                    alt=""
                    aria-hidden
                    decoding="async"
                />
            ) : null}
        </section>
    );
}

/** 分类 / 筛选切换: 有 href 渲染成链接 (aria-current), 否则按钮 (aria-pressed); 选中底块用 layoutId 滑动 */
export function HxPills<K extends string>(p: { items: { key: K; label: React.ReactNode; hint?: React.ReactNode; href?: string; accent?: string }[]; current: K; onPick?: (k: K) => void; "aria-label": string }): React.ReactElement {
    const reduced = useHxReducedMotion();
    const layoutId = `hx-pill-${React.useId()}`;
    return (
        <nav className={styles.pills} aria-label={p["aria-label"]}>
            {p.items.map((it) => {
                const on = it.key === p.current;
                const inner = (
                    <>
                        {on ? (
                            reduced ? (
                                <span className={styles.pillBg} aria-hidden />
                            ) : (
                                <motion.span className={styles.pillBg} layoutId={layoutId} transition={SPRING} aria-hidden />
                            )
                        ) : null}
                        <span className={styles.pillLabel}>{it.label}</span>
                        {it.hint != null ? <small className={styles.pillHint}>{it.hint}</small> : null}
                    </>
                );
                const common = {
                    className: styles.pill,
                    "data-on": on ? "true" : undefined,
                    style: accentVar("--hx-pill-accent", it.accent),
                };
                return it.href ? (
                    <Link key={it.key} to={it.href} {...common} aria-current={on ? "page" : undefined} onClick={() => p.onPick?.(it.key)}>
                        {inner}
                    </Link>
                ) : (
                    <button key={it.key} type="button" {...common} aria-pressed={on} onClick={() => p.onPick?.(it.key)}>
                        {inner}
                    </button>
                );
            })}
        </nav>
    );
}

/** 玻璃卡片: interactive 时悬浮上移 --hx-lift + 阴影, 强调色写在 --hx-card-accent */
export function HxCard(p: { accent?: string; interactive?: boolean; className?: string; children: React.ReactNode }): React.ReactElement {
    const reduced = useHxReducedMotion();
    return (
        <div
            className={cx(styles.card, p.className)}
            data-interactive={p.interactive ? "true" : undefined}
            data-reduced={reduced ? "true" : undefined}
            style={accentVar("--hx-card-accent", p.accent)}
        >
            {p.children}
        </div>
    );
}

/** 区块标题: 圆体标题 + 计数 + 右侧操作 */
export function HxSectionHead(p: { title: React.ReactNode; count?: React.ReactNode; action?: React.ReactNode; id?: string }): React.ReactElement {
    return (
        <header className={styles.sectionHead}>
            <h2 id={p.id} className={styles.sectionTitle}>
                {p.title}
                {p.count != null ? <span className={styles.sectionCount}>{p.count}</span> : null}
            </h2>
            {p.action ? <div className={styles.sectionAction}>{p.action}</div> : null}
        </header>
    );
}

const HeartIcon = (): React.ReactElement => (
    <svg viewBox="0 0 24 24" width="14" height="14" aria-hidden>
        <path fill="currentColor" d="M12 21s-7.5-4.6-9.6-9.3C.9 8.4 3 4.5 6.7 4.5c2.1 0 3.6 1.1 4.3 2.4.7-1.3 2.2-2.4 4.3-2.4 3.7 0 5.8 3.9 4.3 7.2C19.5 16.4 12 21 12 21z" />
    </svg>
);

const StarIcon = (): React.ReactElement => (
    <svg viewBox="0 0 24 24" width="14" height="14" aria-hidden>
        <path fill="currentColor" d="M12 2.8l2.8 5.8 6.4.9-4.6 4.5 1.1 6.3L12 17.3l-5.7 3 1.1-6.3L2.8 9.5l6.4-.9z" />
    </svg>
);

/** 我的评分 (1-10, 粉色心) */
export function HeartRate(p: { value: number }): React.ReactElement {
    return (
        <span className={styles.heart} title="我的评分" aria-label={`我的评分 ${p.value} / 10`}>
            <HeartIcon />
            <b>{p.value}</b>
        </span>
    );
}

/** Bangumi 评分 (金色星, 一位小数) */
export function BgmScore(p: { value: number }): React.ReactElement {
    const v = p.value.toFixed(1);
    return (
        <span className={styles.bgm} title="Bangumi 评分" aria-label={`Bangumi 评分 ${v}`}>
            <StarIcon />
            <b>{v}</b>
        </span>
    );
}

const SKELETON_CLASS: Record<"cover" | "card" | "line" | "tile", string> = {
    cover: styles.skCover,
    card: styles.skCard,
    line: styles.skLine,
    tile: styles.skTile,
};

/** 骨架: 尺寸跟真实卡一致, 外层由调用方的网格类排版 */
export function HxSkeleton(p: { shape: "cover" | "card" | "line" | "tile"; count?: number; className?: string }): React.ReactElement {
    const reduced = useHxReducedMotion();
    const n = Math.max(1, p.count ?? 1);
    return (
        <div className={cx(styles.skeletons, p.className)} data-shape={p.shape} role="status" aria-label="加载中" aria-busy="true">
            {Array.from({ length: n }, (_, i) => (
                <span key={i} className={cx(styles.sk, SKELETON_CLASS[p.shape])} data-reduced={reduced ? "true" : undefined} aria-hidden />
            ))}
        </div>
    );
}

/** 错误 (标题 + 原因 + 重试) / 空态 (猫娘 + 怎么填数据) */
export function HxState(p: { kind: "error" | "empty"; title: string; body?: React.ReactNode; onRetry?: () => void }): React.ReactElement {
    const nekoSrc = useBaseUrl("/default-img/neko_right.png");
    return (
        <Reveal className={styles.state}>
            <div data-kind={p.kind} role={p.kind === "error" ? "alert" : undefined} className={styles.stateInner}>
                {p.kind === "empty" ? <img className={styles.stateNeko} src={nekoSrc} alt="" aria-hidden decoding="async" /> : null}
                <h2 className={styles.stateTitle}>{p.title}</h2>
                {p.body ? <div className={styles.stateBody}>{p.body}</div> : null}
                {p.kind === "error" && p.onRetry ? (
                    <button type="button" className={styles.retry} onClick={p.onRetry}>重试</button>
                ) : null}
            </div>
        </Reveal>
    );
}

/**
 * 探头猫娘轨道: 放进一个 position: relative 的舞台里, 贴在舞台外侧, 切口压进舞台边 6%.
 * sticky 时随滚动一直探头; lean 时往外歪头 (弹簧). 宽度不足 1280px 时整条轨道隐藏, 由 HxHero 里那一只代替.
 */
export function NekoPeek(p: { side: "left" | "right"; lean?: boolean; sticky?: boolean }): React.ReactElement {
    const reduced = useHxReducedMotion();
    const src = useBaseUrl(p.side === "left" ? "/default-img/neko_left.png" : "/default-img/neko_right.png");
    const lean = !!p.lean && !reduced;
    const sign = p.side === "left" ? -1 : 1;
    return (
        <div className={styles.nekoRail} data-side={p.side} aria-hidden>
            <div className={styles.nekoHold} data-sticky={p.sticky ? "true" : undefined}>
                <motion.img
                    className={styles.neko}
                    data-reduced={reduced ? "true" : undefined}
                    src={src}
                    alt=""
                    decoding="async"
                    animate={{ rotate: lean ? 6 * sign : 0, x: lean ? 6 * sign : 0 }}
                    transition={reduced ? { duration: 0 } : SPRING}
                />
            </div>
        </div>
    );
}
