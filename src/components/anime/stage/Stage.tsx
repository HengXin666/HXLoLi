import React from "react";
import Link from "@docusaurus/Link";
import { motion, useReducedMotion } from "framer-motion";
import { SideCats } from "@site/src/components/BlogWithCats";
import styles from "./stage.module.css";

/*
 * 番剧专题页的页面骨架: kyara 角色配图用全套; susume 推荐页 2026-11-08 起照搬番剧库, 只用这里的 StagePaper 当墙.
 * 2026-11-06 第二版: 照首页 (src/components/home/page.module.css) 的语汇重做 ——
 * 1px 细线、12 / 16px 圆角、20px 节标题 + 右侧灰色说明、等宽数字、150ms 悬停, 不要封面墙 / 光晕 / 点阵 / 逐字弹入 / 舞台里的猫娘.
 * 正文是一张有左右边线的「纸」(paperClass), 与两侧探头立绘的留白分开, 学博客文章页的正文 / 侧栏分界.
 */

export const EASE: [number, number, number, number] = [0.2, 0.8, 0.2, 1];

const cx = (...xs: (string | false | null | undefined)[]): string => xs.filter(Boolean).join(" ");

/** 正文纸面: 加在页面 main 上, 左右 1px 边线 + 实底, 与两侧立绘区分开 (kyara 仍在用; susume 已改用 StagePaper) */
export const paperClass = styles.paper;

/** 正文墙宽, 与系列页 / 番剧详情 / 番剧库的 BlogWithCats contentWidth 一致 */
export const PAPER_WIDTH = 1200;

/**
 * 正文墙 + 两侧墙后探头立绘, 与系列页 (src/pages/anime/series.tsx) 完全同一套: BlogWithCats pair 变体, 墙宽 1200.
 * 2026-11-06 晚: susume 原来只挂 paperClass, 立绘走 RouteCats 的兜底 gutter, 墙选择器 'main .container' 量不到纸,
 * 只能按 1140 / 1320 推算 —— 纸实际 1180, 立绘离纸要么隔一条缝要么压边, 宽度上限 260 也比系列页 (380, 藏进墙后) 小一圈, 两侧比例不对.
 * 取舍记在 StageHero 上方引用的 note (StagePaper 一节).
 */
export function StagePaper ({ className, style, children }: { className?: string; style?: React.CSSProperties; children: React.ReactNode }): React.ReactElement {
    // 2026-11-06 21:1x: 不再用 BlogWithCats 的默认导出 (SusumeHubPage 报 Element type is invalid / undefined),
    // 直接按 BlogWithCats 的 DOM 结构展开, 只用已在 share.tsx 验证过的具名导出 SideCats; 结构不变, pair 立绘照样找 .blog-content 当墙.
    return (
        <div className="blog-container">
            <SideCats contentWidth={PAPER_WIDTH} />
            <div className="blog-content-wrapper" style={{ maxWidth: PAPER_WIDTH }}>
                <div className="blog-content">
                    <main className={cx(styles.sheet, className)} style={style}>{children}</main>
                </div>
            </div>
        </div>
    );
}

export function useCalm (): boolean {
    return useReducedMotion() === true;
}

export function accentStyle (accent?: string, extra?: React.CSSProperties): React.CSSProperties | undefined {
    if (!accent && !extra) return undefined;
    return { ...(accent ? { ["--stage-accent" as string]: accent } : {}), ...extra } as React.CSSProperties;
}

/** 数字: 等宽, 不滚动 */
export function CountUp ({ value, decimals = 0 }: { value: number; decimals?: number }): React.ReactElement {
    return <span className={styles.num}>{value.toFixed(decimals)}</span>;
}

/** 名次: 小号等宽灰字 01 / 02 */
export function Numeral ({ n, className, pad = true }: { n: number; className?: string; pad?: boolean }): React.ReactElement {
    return <span className={cx(styles.numeral, className)} aria-hidden>{pad ? String(n).padStart(2, "0") : String(n)}</span>;
}

/** 进入视口时轻微淡入 (8px, 200ms), 与首页最近更新的 hxReveal 同一手感 */
export function Rise ({ children, className, delay = 0, as = "div" }: { children: React.ReactNode; className?: string; delay?: number; as?: "div" | "li" | "section" | "article" }): React.ReactElement {
    const calm = useCalm();
    if (calm) return React.createElement(as, { className }, children);
    const M = motion[as];
    return (
        <M
            className={className}
            initial={{ opacity: 0, y: 8 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true, amount: 0.1 }}
            transition={{ duration: 0.2, ease: EASE, delay: Math.min(delay, 0.2) }}
        >
            {children}
        </M>
    );
}

export interface StageStat { label: string; value: number | null; decimals?: number; suffix?: string }

export interface StageHeroProps {
    /** 面包屑: [文字, 链接?], 最后一项不带链接 */
    crumbs: [string, string?][];
    kicker?: React.ReactNode;
    title: string;
    romaji?: string;
    sub?: React.ReactNode;
    stats?: StageStat[];
    accent?: string;
    /** 旧版封面墙用, 现已不渲染 (保留字段免得调用方报错) */
    backdrop?: readonly string[];
    /** 旧版舞台猫娘台词, 现已不渲染 */
    say?: React.ReactNode;
    /** 标题行右侧的操作 */
    aside?: React.ReactNode;
    children?: React.ReactNode;
}

/**
 * 页头: 面包屑 → 小标 / 标题 + 罗马音 / 一句说明 (右侧操作) → 一行细线分格的统计.
 * .agents/notes/implemented/feature/2026-11-06-anime-susume-neko-redesign.md
 */
export function StageHero (p: StageHeroProps): React.ReactElement {
    return (
        <header className={styles.hero} style={accentStyle(p.accent)}>
            <nav className={styles.crumbs} aria-label="面包屑">
                {p.crumbs.map(([label, href], i) => (
                    <React.Fragment key={label}>
                        {i > 0 && <span aria-hidden>/</span>}
                        {href ? <Link to={href}>{label}</Link> : <span aria-current="page">{label}</span>}
                    </React.Fragment>
                ))}
            </nav>
            <div className={styles.heroRow}>
                <div className={styles.heroMain}>
                    {p.kicker && <p className={styles.kicker}>{p.kicker}</p>}
                    <h1 className={styles.title}>
                        {p.title}
                        {p.romaji && <span className={styles.romaji}>{p.romaji}</span>}
                    </h1>
                    {p.sub && <div className={styles.sub}>{p.sub}</div>}
                </div>
                {p.aside && <div className={styles.heroAside}>{p.aside}</div>}
            </div>
            {p.stats && p.stats.length > 0 && (
                <dl className={styles.stats} style={{ ["--n" as string]: p.stats.length } as React.CSSProperties}>
                    {p.stats.map((s) => (
                        <div key={s.label} className={styles.stat}>
                            <dd>
                                {s.value == null ? "—" : <CountUp value={s.value} decimals={s.decimals} />}
                                {s.suffix && s.value != null ? <small>{s.suffix}</small> : null}
                            </dd>
                            <dt>{s.label}</dt>
                        </div>
                    ))}
                </dl>
            )}
            {p.children}
        </header>
    );
}

/** 节标题: 20px 标题 + 灰色计数, 右侧说明或操作 (首页 .head) */
export function StageHead ({ title, count, action, id }: { jp?: string; title: React.ReactNode; count?: React.ReactNode; action?: React.ReactNode; id?: string }): React.ReactElement {
    return (
        <div className={styles.head}>
            <h2 id={id}>
                {title}
                {count != null && <span className={styles.headCount}>{count}</span>}
            </h2>
            {action && <div className={styles.headAction}>{action}</div>}
        </div>
    );
}
