import React, { createContext, useContext } from 'react';

/**
 * 演示页"层"端口 —— 页内浮层 (放大/全屏类控件) 的挂载点与占用信号.
 *
 * 为什么需要它 (一个真实踩出来的层级 bug):
 *   Diagram 的"放大"曾经把浮层 Portal 到 document.body, 于是:
 *     1. 浮层脱离了 .hxd-deck 的 CSS 变量 (--hxd-color-bg 等) ——
 *        背景色解析不出值, 浮层是透明的, 后面的正文直接透出来;
 *     2. 它按 100vw/100vh 铺满**整个网页**, 而不是演示页 ——
 *        内部布局比父布局还大, 视觉上就是"整页变成了演示页";
 *     3. 宿主在浏览器全屏里时, 全屏元素处于 top layer,
 *        body 上的浮层被压在它下面 —— 点了全屏反而什么也看不见.
 *
 *   正解: 浮层挂在**演示页根节点**里. inset:0 天然等于"恰好铺满演示页",
 *   永远不可能超过父布局; 变量、字体、圆角也都还在; 站点 UI 与卡片外框一概不受影响.
 */
export interface DeckLayer {
    /** 演示页根节点 (.hxd-deck); 未挂载时为 null */
    host: HTMLElement | null;
    /** 告诉演示页"有控件正在占用整页", 让它自己的装饰暂时让位 */
    setLayerFull: (on: boolean) => void;
}

const DeckLayerContext = createContext<DeckLayer>({ host: null, setLayerFull: () => {} });

/**
 * 卡片身份 (端口) —— 图表生成分享链接时要知道"我属于哪张卡片".
 *
 * 为什么用 context 而不是 prop: 演示页内容是**预先渲染好的 React 节点**
 * (registry 里的 slides()), 从 PptCard 一路把 cardId 传到 Diagram 需要
 * 穿过这棵不由我们构造的树. context 是这条链上唯一稳定的通道.
 */
const PptCardIdContext = createContext<string>('');

export function PptCardIdProvider({
    value,
    children,
}: {
    value: string;
    children: React.ReactNode;
}): React.ReactElement {
    return <PptCardIdContext.Provider value={value}>{children}</PptCardIdContext.Provider>;
}

/** 当前卡片在 URL 里的标识; 不在卡片内时为空串 */
export function usePptCardId(): string {
    return useContext(PptCardIdContext);
}

export function DeckLayerProvider({
    value,
    children,
}: {
    value: DeckLayer;
    children: React.ReactNode;
}): React.ReactElement {
    return <DeckLayerContext.Provider value={value}>{children}</DeckLayerContext.Provider>;
}

/** 默认值 (host=null) 让控件可以脱离 Deck 单独使用 —— 那时退回视口浮层 */
export function useDeckLayer(): DeckLayer {
    return useContext(DeckLayerContext);
}
