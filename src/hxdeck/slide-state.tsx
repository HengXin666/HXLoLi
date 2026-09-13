import React, { createContext, useContext } from 'react';

/**
 * 当前幻灯片是否处于活动态.
 * 需要它是因为 visibility:hidden 的容器尺寸为 0, 任何依赖测量的控件 (图表/画布)
 * 都必须等激活后再挂载, 否则会静默渲染成空白.
 */
const SlideActiveContext = createContext(false);

export function SlideActiveProvider({
    active,
    children,
}: {
    active: boolean;
    children: React.ReactNode;
}): React.ReactElement {
    return <SlideActiveContext.Provider value={active}>{children}</SlideActiveContext.Provider>;
}

export function useIsSlideActive(): boolean {
    return useContext(SlideActiveContext);
}

/**
 * 当前 deck 是否处于"接管交互"状态 (放大 / 独立播放页为 true, 正文卡片预览为 false).
 *
 * 为什么需要单独开一条通道:
 *   卡片预览是嵌在 16:9 卡片里、外面还套着一层舞台 scale() 的。任何"把浮层铺满
 *   视口"的控件 (如 Diagram 的页内全屏) 在这种祖先 transform 下会把 position:fixed
 *   的包含块算错 —— 实测在预览卡片里点放大, 整块被裁在卡片高度内且退不出去,
 *   正是"放大到全屏网页、退出不明显、特别是在预览模式"那个隐患。
 *
 *   所以预览态干脆不提供这类控件: 要缩放/全屏, 点开卡片 (此时 interactive=true)。
 *   默认 true —— 不在 <Deck> 内单独使用控件时保持原有能力。
 */
const DeckInteractiveContext = createContext(true);

export function DeckInteractiveProvider({
    interactive,
    children,
}: {
    interactive: boolean;
    children: React.ReactNode;
}): React.ReactElement {
    return <DeckInteractiveContext.Provider value={interactive}>{children}</DeckInteractiveContext.Provider>;
}

export function useDeckInteractive(): boolean {
    return useContext(DeckInteractiveContext);
}

/**
 * 当前演示页的页码 (端口).
 *
 * 用途: 页内控件要生成"能回到此刻视角"的分享链接 —— 少了页码, 别人打开只会
 * 落在第 1 页, 还得自己翻。这里把 deck 的当前页下发, 控件不必猜.
 *
 * index 是 0-based (对内约定), total 是总屏数.
 */
export interface DeckPage {
    index: number;
    total: number;
}

const DeckPageContext = createContext<DeckPage>({ index: 0, total: 1 });

export function DeckPageProvider({
    value,
    children,
}: {
    value: DeckPage;
    children: React.ReactNode;
}): React.ReactElement {
    return <DeckPageContext.Provider value={value}>{children}</DeckPageContext.Provider>;
}

export function useDeckPage(): DeckPage {
    return useContext(DeckPageContext);
}
