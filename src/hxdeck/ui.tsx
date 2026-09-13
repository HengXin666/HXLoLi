import React from 'react';
import { useDeckTheme } from './theme/context';
import { Rise } from './blocks';

/**
 * hxdeck 控件层 (v2)
 *
 * 设计参考:
 *   · Apple HIG  —— clarity (内容优先, 装饰让位) / deference (控件不抢戏) / depth (层级表达)
 *   · Material 3 —— elevation 五级 + state layer 交互反馈 + 语义色角色
 *   · 现代实践   —— 同心圆角 (concentric corners): 内圆角 = 外圆角 - 内边距
 *
 * 一条统一约束: 所有颜色/圆角/阴影/动效都取自主题 token, 控件内不出现字面值.
 * 这样"换主题"才是真的换主题, 而不是换一半.
 */

/* ==========================================================================
   1. 布局原语
   ========================================================================== */

/** 页头: 章节标识 + 标题 + 可选说明. 用于每页统一的开场结构. */
export function PageHeader({
    title,
    eyebrow,
    desc,
    i = 0,
}: {
    title: React.ReactNode;
    eyebrow?: React.ReactNode;
    desc?: React.ReactNode;
    i?: number;
}): React.ReactElement {
    return (
        <Rise i={i} className="hxd-pageheader">
            {eyebrow ? <div className="hxd-pageheader__eyebrow">{eyebrow}</div> : null}
            <h2 className="hxd-pageheader__title">{title}</h2>
            {desc ? <p className="hxd-pageheader__desc">{desc}</p> : null}
        </Rise>
    );
}

/**
 * 两栏布局.
 *
 * 两种写法都支持 —— 这是刻意的宽容设计:
 *   <Split left={<A/>} right={<B/>} />
 *   <Split><A/><B/></Split>
 * 单一 API 容易让人写错且**静默丢内容**, 所以这里同时接受 props 与 children.
 */
export function Split({
    left,
    right,
    children,
    ratio = '1fr 1fr',
    gap,
    align = 'center',
    i,
}: {
    left?: React.ReactNode;
    right?: React.ReactNode;
    children?: React.ReactNode;
    ratio?: string;
    gap?: string;
    align?: 'start' | 'center' | 'stretch';
    i?: number;
}): React.ReactElement {
    let l = left;
    let r = right;
    if (l === undefined && r === undefined && children !== undefined) {
        const arr = React.Children.toArray(children);
        l = arr[0] ?? null;
        r = arr.slice(1);
    }
    const body = (
        <div className="hxd-split" style={{ gridTemplateColumns: ratio, gap, alignItems: align }}>
            <div className="hxd-split__col">{l}</div>
            <div className="hxd-split__col">{r}</div>
        </div>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/** 等分栏 (2~4 项最佳). 用于并列对比. */
export function Columns({
    children,
    cols,
    gap,
    i,
}: {
    children: React.ReactNode;
    cols?: number;
    gap?: string;
    i?: number;
}): React.ReactElement {
    const items = React.Children.toArray(children);
    const n = cols ?? Math.min(items.length, 4);
    const body = (
        <div
            className="hxd-columns"
            style={{ gridTemplateColumns: `repeat(${n}, minmax(0, 1fr))`, gap }}
        >
            {items.map((c, k) => (
                <div className="hxd-columns__col" key={k}>
                    {c}
                </div>
            ))}
        </div>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/* ==========================================================================
   2. 内容块
   ========================================================================== */

/** 要点列表: 支持图标位, 支持强调项. 比裸 <ul> 更适合投屏阅读. */
export function Bullets({
    items,
    marker = 'dot',
    i,
}: {
    items: (React.ReactNode | { text: React.ReactNode; strong?: boolean })[];
    marker?: 'dot' | 'check' | 'num' | 'arrow';
    i?: number;
}): React.ReactElement {
    const body = (
        <ul className="hxd-bullets" data-marker={marker}>
            {items.map((it, k) => {
                const isObj = it !== null && typeof it === 'object' && 'text' in (it as object);
                const text = isObj ? (it as { text: React.ReactNode }).text : (it as React.ReactNode);
                const strong = isObj ? (it as { strong?: boolean }).strong : false;
                return (
                    <li className="hxd-bullets__item" data-strong={strong ? 'true' : undefined} key={k}>
                        <span className="hxd-bullets__marker" aria-hidden="true">
                            {marker === 'num' ? k + 1 : marker === 'check' ? '✓' : marker === 'arrow' ? '→' : ''}
                        </span>
                        <span className="hxd-bullets__text">{text}</span>
                    </li>
                );
            })}
        </ul>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/** 提示条: 4 种语义 (tip / info / warn / danger). 颜色取主题语义色. */
export function Callout({
    kind = 'tip',
    title,
    children,
    i,
}: {
    kind?: 'tip' | 'info' | 'warn' | 'danger';
    title?: React.ReactNode;
    children: React.ReactNode;
    i?: number;
}): React.ReactElement {
    const body = (
        <div className="hxd-callout" data-kind={kind}>
            <div className="hxd-callout__icon" aria-hidden="true">
                {kind === 'warn' ? '!' : kind === 'danger' ? '×' : kind === 'info' ? 'i' : '★'}
            </div>
            <div className="hxd-callout__main">
                {title ? <div className="hxd-callout__title">{title}</div> : null}
                <div className="hxd-callout__body">{children}</div>
            </div>
        </div>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/** 引用块: 带出处. */
export function Quote({
    children,
    cite,
    i,
}: {
    children: React.ReactNode;
    cite?: React.ReactNode;
    i?: number;
}): React.ReactElement {
    const body = (
        <figure className="hxd-quoteblock">
            <blockquote className="hxd-quoteblock__text">{children}</blockquote>
            {cite ? <figcaption className="hxd-quoteblock__cite">— {cite}</figcaption> : null}
        </figure>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/** 步骤条: 横向流程. 用于"从 A 到 B"的过程说明. */
export function Steps({
    steps,
    i,
}: {
    steps: { title: React.ReactNode; desc?: React.ReactNode }[];
    i?: number;
}): React.ReactElement {
    const body = (
        <ol className="hxd-steps">
            {steps.map((s, k) => (
                <li className="hxd-steps__item" key={k}>
                    <span className="hxd-steps__num">{k + 1}</span>
                    <span className="hxd-steps__main">
                        <span className="hxd-steps__title">{s.title}</span>
                        {s.desc ? <span className="hxd-steps__desc">{s.desc}</span> : null}
                    </span>
                </li>
            ))}
        </ol>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/** 徽章: 状态/标签. */
export function Badge({
    children,
    tone = 'neutral',
}: {
    children: React.ReactNode;
    tone?: 'neutral' | 'brand' | 'success' | 'warn' | 'danger';
}): React.ReactElement {
    return <span className="hxd-badge" data-tone={tone}>{children}</span>;
}

/** 分隔线 (可带文字). */
export function Divider({ label }: { label?: React.ReactNode }): React.ReactElement {
    return label ? (
        <div className="hxd-divider hxd-divider--labeled">
            <span>{label}</span>
        </div>
    ) : (
        <hr className="hxd-divider" />
    );
}

/** 键值对列表: 适合参数/规格说明. */
export function KeyValues({
    items,
    i,
}: {
    items: { key: React.ReactNode; value: React.ReactNode }[];
    i?: number;
}): React.ReactElement {
    const body = (
        <dl className="hxd-kv">
            {items.map((it, k) => (
                <div className="hxd-kv__row" key={k}>
                    <dt className="hxd-kv__k">{it.key}</dt>
                    <dd className="hxd-kv__v">{it.value}</dd>
                </div>
            ))}
        </dl>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/* ==========================================================================
   3. 图片容器 —— 图片"融入"而不是"贴上去"
   ========================================================================== */

/**
 * 图片框 (Figure).
 *
 * 关键设计 —— 为什么图片要有"容器":
 *   演示页里直接放 <img> 会显得突兀 (背景色/圆角/比例都与版面无关).
 *   Figure 提供统一的框: 背景、圆角(同心)、内边距、说明位、可选适应方式.
 *   于是任意图片放进来都像是"版面的一部分", 换图不改版面.
 */
export function Figure({
    src,
    alt = '',
    caption,
    /** cover 填满 (会裁切) / contain 完整显示 / width 按宽自适应 */
    fit = 'cover',
    /** 宽高比, 如 '16/9' '4/3' '1/1'; 不传则按内容 */
    ratio,
    tone = 'plain',
    i,
}: {
    src: string;
    alt?: string;
    caption?: React.ReactNode;
    fit?: 'cover' | 'contain';
    ratio?: string;
    /** plain 无框 / card 卡片底 / glass 磨砂 */
    tone?: 'plain' | 'card' | 'glass';
    i?: number;
}): React.ReactElement {
    const body = (
        <figure className="hxd-figure" data-tone={tone}>
            <div className="hxd-figure__frame" style={ratio ? { aspectRatio: ratio } : undefined}>
                <img className="hxd-figure__img" data-fit={fit} src={src} alt={alt} loading="lazy" />
            </div>
            {caption ? <figcaption className="hxd-figure__caption">{caption}</figcaption> : null}
        </figure>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

export { Quote as QuoteBlock };