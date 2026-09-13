import React from 'react';

/** 逐元素级联进场的包装: 按 index 决定延迟, 让一页里的元素依次浮现 */
export function Rise({
    i = 0,
    as: Tag = 'div',
    className,
    style,
    children,
    ...rest
}: React.HTMLAttributes<HTMLElement> & { i?: number; as?: keyof React.JSX.IntrinsicElements }): React.ReactElement {
    const Comp = Tag as React.ElementType;
    return (
        <Comp
            className={['hxd-rise', className].filter(Boolean).join(' ')}
            style={{ ['--hxd-i' as string]: i, ...style }}
            {...rest}
        >
            {children}
        </Comp>
    );
}

/** 封面页: 大标题 + 副题 + 可选角标 */
export function Cover({
    title,
    subtitle,
    eyebrow,
}: {
    title: React.ReactNode;
    subtitle?: React.ReactNode;
    eyebrow?: React.ReactNode;
}): React.ReactElement {
    return (
        <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center', height: '100%', gap: 18 }}>
            {eyebrow ? (
                <Rise i={0} className="hxd-eyebrow">
                    {eyebrow}
                </Rise>
            ) : null}
            <Rise i={1} as="h1" className="hxd-title">
                {title}
            </Rise>
            {subtitle ? (
                <Rise i={2} as="p" className="hxd-subtitle">
                    {subtitle}
                </Rise>
            ) : null}
        </div>
    );
}

/** 章节页头 */
export function Head({ children, i = 0 }: { children: React.ReactNode; i?: number }): React.ReactElement {
    return (
        <Rise i={i} as="h2" className="hxd-h2">
            {children}
        </Rise>
    );
}

/**
 * 通用卡片.
 *
 * 注意 (踩坑): 带 i 时会被 <Rise> 包一层, 此时**外层**才是 flex/grid 的直接子项.
 * 因此 className 与 style 必须落在包装层, 否则 `hxd-fill` / `flex` 这类布局属性
 * 会作用在错误的元素上, 导致卡片宽度塌陷 (内部图表因拿不到宽度而空白).
 */
/**
 * 通用卡片.
 *
 * 命名注意: 角落装饰位的 prop 叫 **skin** 而不是 slot ——
 * 因为 `React.HTMLAttributes` 本身已有 `slot?: string` (Web Components 属性),
 * 与 ReactNode 相交会退化成一个谁都传不进去的畸形类型.
 */
export function Card({
    children,
    style,
    i,
    className,
    tex = false,
    skin,
    ...rest
}: Omit<React.HTMLAttributes<HTMLDivElement>, 'slot'> & {
    i?: number;
    /** 让主题纹样融入本卡片右下角 (皮肤槽, 非内容图) */
    tex?: boolean;
    /** 卡片角落的吉祥物槽 */
    skin?: React.ReactNode;
}): React.ReactElement {
    const inner = (
        <div
            className="hxd-card"
            data-tex={tex ? 'true' : undefined}
            style={{ height: '100%', boxSizing: 'border-box', ...style }}
            {...rest}
        >
            {children}
            {skin}
        </div>
    );
    return i === undefined ? inner : <Rise i={i} className={className}>{inner}</Rise>;
}