import React, { createContext, useContext } from 'react';
import type { DeckTheme } from './types';

/**
 * 主题上下文.
 * 存在的理由: 图表 / 画布类控件必须拿到**真实色值** (SVG fill 不能吃 CSS 变量),
 * 所以它们不能只靠 CSS 变量, 需要读当前主题对象.
 */
const DeckThemeContext = createContext<DeckTheme | null>(null);

export function DeckThemeProvider({
    theme,
    children,
}: {
    theme: DeckTheme;
    children: React.ReactNode;
}): React.ReactElement {
    return <DeckThemeContext.Provider value={theme}>{children}</DeckThemeContext.Provider>;
}

export function useDeckTheme(): DeckTheme {
    const t = useContext(DeckThemeContext);
    if (!t) throw new Error('useDeckTheme 必须在 <Deck> 内部使用 —— 图表控件需要主题色值.');
    return t;
}

/** 图表系列色: 以主题主色为中心派生出 N 个协调色, 避免调用方硬编码调色板 */
export function seriesColors(theme: DeckTheme, n: number): string[] {
    const base = [
        theme.colors.primary,
        theme.colors.accent,
        theme.colors.success,
        theme.colors.warn,
        theme.colors.danger,
    ];
    const out: string[] = [];
    for (let i = 0; i < n; i++) out.push(base[i % base.length]);
    return out;
}
