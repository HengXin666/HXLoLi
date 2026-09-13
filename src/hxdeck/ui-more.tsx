import React from 'react';
import { useDeckTheme } from './theme/context';
import { Rise } from './blocks';

/**
 * 控件补充集 (v3) —— 补齐演示页常见但此前缺失的形态.
 *
 * 命名与语义参考:
 *   Material 3 的 Progress indicator / Data table 语义角色
 *   Apple HIG 的"用留白与层级表达重点"
 */

/* ==========================================================================
   时间轴 —— 讲演进过程
   ========================================================================== */

export function Timeline({
    items,
    orientation = 'vertical',
    i,
}: {
    items: { time: React.ReactNode; title: React.ReactNode; desc?: React.ReactNode; tone?: 'default' | 'brand' | 'success' | 'warn' }[];
    orientation?: 'vertical' | 'horizontal';
    i?: number;
}): React.ReactElement {
    const body = (
        <ol className="hxd-timeline" data-orient={orientation}>
            {items.map((it, k) => (
                <li className="hxd-timeline__item" data-tone={it.tone ?? 'default'} key={k}>
                    <span className="hxd-timeline__time">{it.time}</span>
                    <span className="hxd-timeline__dot" aria-hidden="true" />
                    <span className="hxd-timeline__main">
                        <span className="hxd-timeline__title">{it.title}</span>
                        {it.desc ? <span className="hxd-timeline__desc">{it.desc}</span> : null}
                    </span>
                </li>
            ))}
        </ol>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/* ==========================================================================
   进度 / 占比条 —— 比饼图更直观的单值展示
   ========================================================================== */

export function Meter({
    items,
    max = 100,
    showValue = true,
    i,
}: {
    items: { label: React.ReactNode; value: number; hint?: React.ReactNode; tone?: 'brand' | 'success' | 'warn' | 'danger' | 'accent' }[];
    max?: number;
    showValue?: boolean;
    i?: number;
}): React.ReactElement {
    const body = (
        <div className="hxd-meters">
            {items.map((it, k) => (
                <div className="hxd-meter" data-tone={it.tone ?? 'brand'} key={k}>
                    <div className="hxd-meter__head">
                        <span className="hxd-meter__label">{it.label}</span>
                        {showValue ? (
                            <span className="hxd-meter__value">
                                {it.value}
                                <small>/{max}</small>
                            </span>
                        ) : null}
                    </div>
                    <div className="hxd-meter__track">
                        <span
                            className="hxd-meter__fill"
                            style={{ width: `${Math.max(0, Math.min(100, (it.value / max) * 100))}%` }}
                        />
                    </div>
                    {it.hint ? <div className="hxd-meter__hint">{it.hint}</div> : null}
                </div>
            ))}
        </div>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/** 单个环形指标 —— 一个数字要突出时用, 不要用饼图凑 */
export function Gauge({
    value,
    max = 100,
    label,
    unit,
    size = 160,
    i,
}: {
    value: number;
    max?: number;
    label?: React.ReactNode;
    unit?: React.ReactNode;
    size?: number;
    i?: number;
}): React.ReactElement {
    const t = useDeckTheme();
    const pct = Math.max(0, Math.min(1, value / max));
    const R = 54;
    const C = 2 * Math.PI * R;
    const body = (
        <div className="hxd-gauge" style={{ width: size }}>
            <svg viewBox="0 0 128 128" role="img" aria-label={typeof label === 'string' ? label : 'gauge'}>
                <circle cx="64" cy="64" r={R} fill="none" stroke={t.colors.border} strokeWidth="10" />
                <circle
                    cx="64"
                    cy="64"
                    r={R}
                    fill="none"
                    stroke={t.colors.primary}
                    strokeWidth="10"
                    strokeLinecap="round"
                    strokeDasharray={`${C * pct} ${C}`}
                    transform="rotate(-90 64 64)"
                />
            </svg>
            <div className="hxd-gauge__mid">
                <span className="hxd-gauge__num">
                    {value}
                    {unit ? <small>{unit}</small> : null}
                </span>
                {label ? <span className="hxd-gauge__label">{label}</span> : null}
            </div>
        </div>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/* ==========================================================================
   对照表 —— A/B 方案对比 (Columns 拼不出来)
   ========================================================================== */

export function CompareTable({
    columns,
    rows,
    highlight,
    i,
}: {
    columns: React.ReactNode[];
    rows: { label: React.ReactNode; values: React.ReactNode[]; tone?: 'default' | 'good' | 'bad' }[];
    /** 高亮第几列 (0-based) */
    highlight?: number;
    i?: number;
}): React.ReactElement {
    const body = (
        <div className="hxd-ctable__wrap">
            <table className="hxd-ctable">
                <thead>
                    <tr>
                        <th className="hxd-ctable__corner" />
                        {columns.map((c, k) => (
                            <th key={k} data-hl={k === highlight ? 'true' : undefined}>
                                {c}
                            </th>
                        ))}
                    </tr>
                </thead>
                <tbody>
                    {rows.map((r, k) => (
                        <tr key={k}>
                            <th scope="row">{r.label}</th>
                            {r.values.map((v, j) => (
                                <td key={j} data-hl={j === highlight ? 'true' : undefined} data-tone={r.tone ?? 'default'}>
                                    {v}
                                </td>
                            ))}
                        </tr>
                    ))}
                </tbody>
            </table>
        </div>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/* ==========================================================================
   通栏大图 —— 一张图铺满, 配一句话
   ========================================================================== */

export function FullBleed({
    src,
    headline,
    sub,
    overlay = 0.45,
    i,
}: {
    src: string;
    headline?: React.ReactNode;
    sub?: React.ReactNode;
    /** 压暗程度 0~1, 保证文字可读 */
    overlay?: number;
    i?: number;
}): React.ReactElement {
    const body = (
        <div className="hxd-bleed">
            <img className="hxd-bleed__img" src={src} alt="" loading="lazy" />
            <span className="hxd-bleed__scrim" style={{ opacity: overlay }} aria-hidden="true" />
            {headline || sub ? (
                <div className="hxd-bleed__text">
                    {headline ? <div className="hxd-bleed__headline">{headline}</div> : null}
                    {sub ? <div className="hxd-bleed__sub">{sub}</div> : null}
                </div>
            ) : null}
        </div>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/* ==========================================================================
   代码 diff —— 显示增删, 而不只是高亮
   ========================================================================== */

export interface DiffLine {
    type: 'add' | 'del' | 'ctx';
    text: string;
}

/** 把 unified diff 文本解析成行 */
export function parseDiff(diff: string): DiffLine[] {
    return diff
        .replace(/\r\n/g, '\n')
        .split('\n')
        .filter((l) => !/^(---|\+\+\+|@@)/.test(l))
        .map((l) => {
            if (l.startsWith('+')) return { type: 'add' as const, text: l.slice(1) };
            if (l.startsWith('-')) return { type: 'del' as const, text: l.slice(1) };
            return { type: 'ctx' as const, text: l.startsWith(' ') ? l.slice(1) : l };
        });
}

export function CodeDiff({
    lines,
    filename,
    i,
}: {
    lines: DiffLine[] | string;
    filename?: string;
    i?: number;
}): React.ReactElement {
    const t = useDeckTheme();
    const rows: DiffLine[] = typeof lines === 'string' ? parseDiff(lines) : lines;
    const body = (
        <div className="hxd-diff" style={{ fontFamily: t.fonts.mono }}>
            {filename ? (
                <div className="hxd-diff__bar">
                    <span style={{ display: 'flex', gap: 6 }}>
                        {['#ff5f57', '#febc2e', '#28c840'].map((c) => (
                            <span key={c} className="hxd-diff__dot" style={{ background: c }} />
                        ))}
                    </span>
                    <span>{filename}</span>
                </div>
            ) : null}
            <pre className="hxd-diff__pre">
                {rows.map((l, k) => (
                    <div className="hxd-diff__line" data-type={l.type} key={k}>
                        <span className="hxd-diff__sign" aria-hidden="true">
                            {l.type === 'add' ? '+' : l.type === 'del' ? '-' : ' '}
                        </span>
                        <span className="hxd-diff__text">{l.text || ' '}</span>
                    </div>
                ))}
            </pre>
        </div>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}
