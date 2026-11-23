import React, { type ComponentType, type ReactNode, type SVGProps } from 'react';
import './kb-note.css';
import { Chart } from './Chart';

/**
 * 知识库笔记组件 (KBNote)
 *
 * 用法: 笔记 frontmatter 写 `mdx:` 下一行缩进 `format: mdx`, 正文直接写 <Lead /> <Layers /> 等,
 * 不用 import (已在 src/theme/MDXComponents 注册).
 * 视觉跟首页 (src/components/home/page.module.css) 同一套: 1px 细线, 小圆角, 粉色是唯一强调色, 数字等宽.
 *
 * 约束:
 *   - 只吃数据, 不吃样式. 没有 style / className 参数, 写笔记的模型改不了外观
 *   - 颜色只取 Infima 令牌 (--ifm-*), 跟随明暗
 *   - 数字型的宽度都按数据比例算, 不手调
 * 每个组件的范例见同目录 CATALOG.md, 检查脚本 scripts/check-kb-note.mjs
 */

type Tone = 'good' | 'ok' | 'bad' | 'last';

/* ---------- Lead: 开头三行 ---------- */

export interface LeadProps {
    /** 这篇为什么存在: 我碰到了什么问题 */
    why: ReactNode;
    /** 一句话结论 */
    claim: ReactNode;
    /** 读完能做什么 */
    use: ReactNode;
}

export function Lead ({ why, claim, use }: LeadProps): ReactNode {
    return (
        <div className="hxkb hxkb-panel hxkb-lead">
            <div className="hxkb-lead-row"><span className="hxkb-k">为什么有这篇</span><div>{why}</div></div>
            <div className="hxkb-lead-row hxkb-lead-claim"><span className="hxkb-k">一句话</span><div>{claim}</div></div>
            <div className="hxkb-lead-row"><span className="hxkb-k">读完你能</span><div>{use}</div></div>
        </div>
    );
}

/* ---------- Stats: 关键数字 ---------- */

export interface StatItem {
    value: string;
    label: ReactNode;
    /** 出处, 必填. 例: "演讲 0:44" */
    source: string;
    /** false = 未核实 / 自述, 会自动标 [不确定] */
    verified?: boolean;
}

export function Stats ({ items }: { items: StatItem[] }): ReactNode {
    return (
        <dl className="hxkb hxkb-stats">
            {items.map((s, i) => (
                <div className="hxkb-stat" key={i}>
                    <dd>{s.value}</dd>
                    <dt>{s.label}</dt>
                    <div className="hxkb-stat-s">
                        {s.verified === false && <span className="hxkb-unsure">[不确定]</span>}
                        {s.source}
                    </div>
                </div>
            ))}
        </dl>
    );
}

/* ---------- Layers: 分层 / 金字塔 ---------- */

export interface LayerItem {
    name: ReactNode;
    desc?: ReactNode;
    /** 投入或体量, 决定条宽 (按最大值等比) */
    weight: number;
    /** 角标, 例: "确定性" */
    tag?: string;
    tone?: Tone;
}

export function Layers ({ items, caption }: { items: LayerItem[]; caption?: ReactNode }): ReactNode {
    const max = Math.max(...items.map((x) => x.weight), 1);
    return (
        <figure className="hxkb hxkb-panel hxkb-layers">
            <ol>
                {items.map((it, i) => (
                    <li className={`hxkb-layer hxkb-tone-${it.tone || 'ok'}`} key={i}>
                        <div className="hxkb-layer-text">
                            <div className="hxkb-layer-name">
                                {it.name}
                                {it.tag && <span className="hxkb-tag">{it.tag}</span>}
                            </div>
                            {it.desc && <div className="hxkb-layer-desc">{it.desc}</div>}
                        </div>
                        <div className="hxkb-bar" aria-hidden>
                            <span style={{ width: `${Math.round(100 * it.weight / max)}%` }} />
                        </div>
                    </li>
                ))}
            </ol>
            {caption && <figcaption>{caption}</figcaption>}
        </figure>
    );
}

/* ---------- Fork: 一个起点, 按顺序尝试的几条路 ---------- */

export interface ForkOption {
    name: ReactNode;
    desc?: ReactNode;
    tone?: Tone;
}

export function Fork ({ from, options, caption }: { from: ReactNode; options: ForkOption[]; caption?: ReactNode }): ReactNode {
    return (
        <figure className="hxkb hxkb-panel hxkb-fork">
            <div className="hxkb-fork-from">{from}</div>
            <ol className="hxkb-fork-opts">
                {options.map((o, i) => (
                    <li className={`hxkb-fork-opt hxkb-tone-${o.tone || 'ok'}`} key={i}>
                        <span className="hxkb-n">{String(i + 1).padStart(2, '0')}</span>
                        <span className="hxkb-fork-name">{o.name}</span>
                        {o.desc && <span className="hxkb-fork-desc">{o.desc}</span>}
                    </li>
                ))}
            </ol>
            {caption && <figcaption>{caption}</figcaption>}
        </figure>
    );
}

/* ---------- Flow: 线性流程 ---------- */

export interface FlowStep {
    name: ReactNode;
    desc?: ReactNode;
    tone?: Tone;
}

export function Flow ({ steps, caption }: { steps: FlowStep[]; caption?: ReactNode }): ReactNode {
    return (
        <figure className="hxkb hxkb-panel hxkb-flow">
            <ol>
                {steps.map((s, i) => (
                    <li className={`hxkb-flow-step hxkb-tone-${s.tone || 'ok'}`} key={i}>
                        <div className="hxkb-flow-name">{s.name}</div>
                        {s.desc && <div className="hxkb-flow-desc">{s.desc}</div>}
                    </li>
                ))}
            </ol>
            {caption && <figcaption>{caption}</figcaption>}
        </figure>
    );
}

/* ---------- Compare: 两边对照 ---------- */

export interface CompareSide {
    title: ReactNode;
    items: ReactNode[];
    tone?: Tone;
}

export function Compare ({ left, right, caption }: { left: CompareSide; right: CompareSide; caption?: ReactNode }): ReactNode {
    const side = (s: CompareSide, def: Tone) => (
        <div className={`hxkb-cmp-side hxkb-tone-${s.tone || def}`}>
            <div className="hxkb-cmp-title">{s.title}</div>
            <ul>{s.items.map((x, i) => <li key={i}>{x}</li>)}</ul>
        </div>
    );
    return (
        <figure className="hxkb hxkb-panel hxkb-cmp">
            <div className="hxkb-cmp-grid">
                {side(left, 'bad')}
                                {side(right, 'good')}
            </div>
            {caption && <figcaption>{caption}</figcaption>}
        </figure>
    );
}

/* ---------- Evidence: 折叠的原文 ---------- */

export function Evidence ({ source, at, children }: { source: string; at?: string; children: ReactNode }): ReactNode {
    return (
        <details className="hxkb hxkb-ev">
            <summary><span className="hxkb-k">原文</span>{source}{at && <span className="hxkb-ev-at">{at}</span>}</summary>
            <div className="hxkb-ev-body">{children}</div>
        </details>
    );
}

/* ---------- Apply: 落到我的项目 ---------- */
/*
 * 两种画法:
 *   不给 tiers: 一行一条 (路径 + 现在 / 下一步), 旧写法, 兼容已有笔记.
 *   给 tiers:   升档图. 每条是一行, 虚线胶囊在现在那一档, 粉框胶囊在要升到的那一档,
 *               中间一根箭头. 路径和完整的现在/下一步放在行下的小字和悬停提示里.
 */

export interface ApplyItem {
    /** 仓库内路径 (相对 HXLoLis 根), 检查脚本会核对存在 */
    path: string;
    now: ReactNode;
    next: ReactNode;
    /** 升档图: 现在在第几档 (从 0 数, 对应 tiers) */
    from?: number;
    /** 升档图: 要升到第几档, 必须大于 from */
    to?: number;
    /** 升档图: 现在那颗胶囊上的短字 (16 字以内) */
    label?: string;
    /** 升档图: 目标那颗胶囊上的短字 (16 字以内) */
    target?: string;
}

function plain (x: ReactNode): string {
    return typeof x === 'string' || typeof x === 'number' ? String(x) : '';
}

function ApplyLadder ({ tiers, items }: { tiers: string[]; items: ApplyItem[] }): ReactNode {
    const n = tiers.length;
    return (
        <div className="hxkb hxkb-ladder" style={{ ['--hxkb-n' as string]: n }}>
            <div className="hxkb-ladder-grid hxkb-ladder-head">
                {tiers.map((t, i) => <span key={i} className={i === n - 1 ? 'hxkb-ladder-top' : undefined}>{t}</span>)}
            </div>
            {items.map((a, i) => {
                const from = a.from ?? 0;
                const to = a.to ?? n - 1;
                const tip = [a.path, plain(a.now) && `现在: ${plain(a.now)}`, plain(a.next) && `下一步: ${plain(a.next)}`].filter(Boolean).join('\n');
                return (
                    <div className="hxkb-ladder-item" key={i} title={tip}>
                        <div className="hxkb-ladder-grid">
                            {tiers.map((_, c) => {
                                if (c < from || c > to) return <span key={c} />;
                                if (c === from) {
                                    return (
                                        <span key={c} className="hxkb-ladder-cell">
                                            <i className="hxkb-ladder-gap" />
                                            <span className="hxkb-pill hxkb-pill-now">{a.label ?? plain(a.now)}</span>
                                            <i className="hxkb-ladder-ln" />
                                        </span>
                                    );
                                }
                                if (c === to) {
                                    return (
                                        <span key={c} className="hxkb-ladder-cell">
                                            <i className="hxkb-ladder-ln hxkb-ladder-ln-end" />
                                            <span className="hxkb-pill hxkb-pill-to">{a.target ?? plain(a.next)}</span>
                                            <i className="hxkb-ladder-gap" />
                                        </span>
                                    );
                                }
                                return <span key={c} className="hxkb-ladder-cell"><i className="hxkb-ladder-ln" /></span>;
                            })}
                        </div>
                        <div className="hxkb-ladder-note"><code>{a.path}</code><span>{a.next}</span></div>
                    </div>
                );
            })}
        </div>
    );
}

export function Apply ({ items, tiers }: { items: ApplyItem[]; tiers?: string[] }): ReactNode {
    if (tiers && tiers.length >= 2) return <ApplyLadder tiers={tiers} items={items} />;
    return (
        <div className="hxkb hxkb-apply">
            {items.map((a, i) => (
                <div className="hxkb-apply-row" key={i}>
                    <code className="hxkb-apply-path">{a.path}</code>
                    <div className="hxkb-apply-cols">
                        <div><span className="hxkb-k">现在</span>{a.now}</div>
                        <div><span className="hxkb-k">下一步</span>{a.next}</div>
                    </div>
                </div>
            ))}
        </div>
    );
}

/* ---------- Boundary: 什么时候不成立 ---------- */

export function Boundary ({ items }: { items: ReactNode[] }): ReactNode {
    return (
        <div className="hxkb hxkb-panel hxkb-bound" role="list">
            {items.map((x, i) => <div className="hxkb-bound-item" role="listitem" key={i}>{x}</div>)}
        </div>
    );
}

/* ---------- Figure: 结构图片 ---------- */
/*
 * src 两种:
 *   require('./x.png').default -> 图片地址, 用 <img>
 *   require('./x.svg').default -> Docusaurus 的 svgr 把 svg 变成 React 组件, 直接内联.
 * 内联 svg 里的 class (box / ln / pk ...) 由 kb-note.css 按站点令牌上色, 所以亮暗主题跟着走.
 */
type FigureSrc = string | ComponentType<SVGProps<SVGSVGElement> & { title?: string }>;

export function Figure ({ src, alt, caption }: { src: FigureSrc; alt: string; caption: ReactNode }): ReactNode {
    const Svg = typeof src === 'string' ? null : src;
    return (
        <figure className={`hxkb hxkb-fig${Svg ? ' hxkb-fig-svg' : ''}`}>
            {Svg ? <Svg role="img" aria-label={alt} title={alt} /> : <img src={src as string} alt={alt} loading="lazy" />}
            <figcaption>{caption}</figcaption>
        </figure>
    );
}

/* ---------- Red / Gold: 行内标注 ----------
 * 只标一句话里最该记住的那几个字, 一节最多一处, 不要整段上色.
 * Red: 判断和警示 (要记住的结论, 踩坑点). Gold: 关键事实 (数字, 原话里的要点).
 */
export function Red ({ children }: { children: ReactNode }): ReactNode {
    return <strong className="hxkb-mark hxkb-mark-red">{children}</strong>;
}

export function Gold ({ children }: { children: ReactNode }): ReactNode {
    return <strong className="hxkb-mark hxkb-mark-gold">{children}</strong>;
}

export { Chart };

export const kbNoteComponents = { Lead, Stats, Layers, Fork, Flow, Compare, Evidence, Apply, Boundary, Figure, Chart, Red, Gold };
