import React from 'react';
import { Highlight, themes } from 'prism-react-renderer';
import { useDeckTheme } from './theme/context';

/**
 * 代码块控件 (自建, 不复用博客的 src/theme/CodeBlock).
 *
 * 与博客代码块的区别:
 *   - 博客那个面向"可编辑 + 全屏 Monaco", 重;
 *   - 这个面向演示页, 只要"静态高亮 + 主题跟随 + 行号 + 逐行入场".
 * 因此单独实现, 互不牵连.
 */

/** 三套配色 —— 都是 VS Code 系, 随主题明暗自动选 */
export type CodeScheme = 'onedark' | 'nightowl' | 'plain';

export interface CodeBlockProps {
    code: string;
    language?: string;
    filename?: string;
    /** 高亮行 (1-based) */
    highlight?: number[];
    showLineNumbers?: boolean;
    /** 明暗主题下用哪套色 */
    scheme?: CodeScheme | 'auto';
    maxHeight?: number;
}

export function CodeBlock({
    code,
    language = 'tsx',
    filename,
    highlight = [],
    showLineNumbers = false,
    scheme = 'auto',
    maxHeight,
}: CodeBlockProps): React.ReactElement {
    const t = useDeckTheme();

    const prismTheme = React.useMemo(() => {
        const s = scheme === 'auto' ? 'onedark' : scheme;
        if (s === 'nightowl') return themes.nightOwl;
        if (s === 'plain') return themes.oneLight;
        return themes.vsDark;
    }, [scheme]);

    const body = code.replace(/^\n/, '').replace(/\s+$/, '');

    return (
        <div
            className="hxd-code"
            style={{
                borderRadius: t.shape.radius,
                border: `1px solid ${t.colors.border}`,
                background: prismTheme.plain.backgroundColor ?? t.colors.bg,
                overflow: 'hidden',
                boxShadow: t.shape.shadow,
                fontFamily: t.fonts.mono,
            }}
        >
            {filename ? (
                <div
                    style={{
                        display: 'flex',
                        alignItems: 'center',
                        gap: 8,
                        padding: '8px 14px',
                        fontSize: 13,
                        fontFamily: t.fonts.mono,
                        color: t.colors.textMuted,
                        borderBottom: `1px solid ${t.colors.border}`,
                        background: 'rgba(127,127,127,0.08)',
                    }}
                >
                    <span style={{ display: 'flex', gap: 6 }}>
                        {['#ff5f57', '#febc2e', '#28c840'].map((c) => (
                            <span key={c} style={{ width: 10, height: 10, borderRadius: '50%', background: c }} />
                        ))}
                    </span>
                    <span>{filename}</span>
                </div>
            ) : null}

            <Highlight theme={prismTheme} code={body} language={language}>
                {({ className, style, tokens, getLineProps, getTokenProps }) => (
                    <pre
                        className={className}
                        style={{
                            ...style,
                            margin: 0,
                            padding: '14px 16px',
                            fontSize: 14,
                            lineHeight: 1.65,
                            overflow: 'auto',
                            maxHeight,
                            background: 'transparent',
                        }}
                    >
                        {tokens.map((line, i) => {
                            const lineNo = i + 1;
                            const isHit = highlight.includes(lineNo);
                            const p = getLineProps({ line, key: i });
                            return (
                                <div
                                    {...p}
                                    key={i}
                                    style={{
                                        ...p.style,
                                        display: 'flex',
                                        gap: 14,
                                        padding: '0 8px',
                                        margin: '0 -8px',
                                        borderRadius: 4,
                                        background: isHit ? `${t.colors.primary}22` : undefined,
                                        borderLeft: isHit ? `2px solid ${t.colors.primary}` : '2px solid transparent',
                                    }}
                                >
                                    {showLineNumbers ? (
                                        <span
                                            style={{
                                                userSelect: 'none',
                                                textAlign: 'right',
                                                minWidth: 24,
                                                color: t.colors.textMuted,
                                                opacity: 0.6,
                                                fontVariantNumeric: 'tabular-nums',
                                            }}
                                        >
                                            {lineNo}
                                        </span>
                                    ) : null}
                                    <span style={{ flex: 1 }}>
                                        {line.map((token, key) => (
                                            <span {...getTokenProps({ token, key })} key={key} />
                                        ))}
                                    </span>
                                </div>
                            );
                        })}
                    </pre>
                )}
            </Highlight>
        </div>
    );
}

/** 行内代码 —— 跟正文同一行用 */
export function Code({ children }: { children: React.ReactNode }): React.ReactElement {
    const t = useDeckTheme();
    return (
        <code
            style={{
                fontFamily: t.fonts.mono,
                fontSize: '0.92em',
                padding: '2px 7px',
                borderRadius: 6,
                background: t.colors.primarySoft,
                border: `1px solid ${t.colors.border}`,
            }}
        >
            {children}
        </code>
    );
}