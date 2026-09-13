import React from 'react';

/**
 * 架构图"语义护照" —— 把 archify viewer 的节点探查能力搬进我们的 React 控件.
 *
 * archify 的产物里, 每个节点/连线都带着稳定的语义钩子:
 *   <g data-node-id data-node-label data-node-sublabel data-node-context data-node-kind>
 *   <path data-edge-from data-edge-to data-edge-label data-edge-key>
 * 交互全部建立在这份钩子上, 与几何无关 —— 所以缩放/拖拽之后依然准确.
 *
 * 这里只做两件事:
 *   1. 从 SVG 文本里解析出这张语义图 (节点 + 有向边)
 *   2. 渲染悬浮信息卡 (archify 叫 Semantic Passport)
 *
 * 高亮本身不走 JS: 直接给元素打 data-focus-* 属性, 由 archify 自带的语义
 * CSS (extracted-diagram 的 css 字段) 负责变暗/提亮 —— 这样视觉语言与
 * 原始 archify 产物完全一致, 我们只是"重新接线".
 */

export interface DiagramNodeFact {
    id: string;
    label: string;
    sublabel?: string;
    context?: string;
    kind?: string;
}

export interface DiagramEdgeFact {
    from: string;
    to: string;
    label?: string;
    key: string;
}

export interface DiagramFacts {
    nodes: Map<string, DiagramNodeFact>;
    /** 出边 (本节点 -> 下游) */
    out: Map<string, DiagramEdgeFact[]>;
    /** 入边 (上游 -> 本节点) */
    into: Map<string, DiagramEdgeFact[]>;
}

const EMPTY: DiagramFacts = { nodes: new Map(), out: new Map(), into: new Map() };

/** 从属性串里取一个属性值 (含反转义) */
function attr(tag: string, name: string): string | undefined {
    const m = new RegExp(name + '="([^"]*)"').exec(tag);
    if (!m) return undefined;
    return m[1]
        .replace(/&quot;/g, '"')
        .replace(/&#39;/g, "'")
        .replace(/&lt;/g, '<')
        .replace(/&gt;/g, '>')
        .replace(/&amp;/g, '&');
}

/** 解析 SVG 里的语义图. 结果按 svg 文本缓存 (调用方用 useMemo) */
export function parseFacts(svg: string): DiagramFacts {
    if (!svg) return EMPTY;
    const nodes = new Map<string, DiagramNodeFact>();
    for (const m of svg.matchAll(/<g\b[^>]*data-node-id="([^"]+)"[^>]*>/g)) {
        const tag = m[0];
        const id = m[1];
        nodes.set(id, {
            id,
            label: attr(tag, 'data-node-label') || id,
            sublabel: attr(tag, 'data-node-sublabel'),
            context: attr(tag, 'data-node-context'),
            kind: attr(tag, 'data-node-kind'),
        });
    }

    const out = new Map<string, DiagramEdgeFact[]>();
    const into = new Map<string, DiagramEdgeFact[]>();
    for (const m of svg.matchAll(/<path\b[^>]*data-edge-from="([^"]+)"[^>]*data-edge-to="([^"]+)"[^>]*>/g)) {
        const tag = m[0];
        const from = m[1];
        const to = m[2];
        // 同一条边可能被拆成多段路径, 用 edge-key 去重
        const key = attr(tag, 'data-edge-key') || from + '\u0000' + to + '\u0000' + (attr(tag, 'data-edge-label') || '');
        const edge: DiagramEdgeFact = { from, to, label: attr(tag, 'data-edge-label'), key };
        const o = out.get(from) || [];
        if (!o.some((e) => e.key === edge.key)) o.push(edge);
        out.set(from, o);
        const i = into.get(to) || [];
        if (!i.some((e) => e.key === edge.key)) i.push(edge);
        into.set(to, i);
    }
    return { nodes, out, into };
}

/** 聚焦时可视化的邻域: 邻居节点 + 直接相连的边 (与 archify set() 的 neighborhood 一致) */
export function neighborhood(facts: DiagramFacts, id: string): { nodes: Set<string>; edgeKeys: Set<string> } {
    const nodes = new Set<string>([id]);
    const edgeKeys = new Set<string>();
    for (const e of facts.out.get(id) || []) { nodes.add(e.to); edgeKeys.add(e.key); }
    for (const e of facts.into.get(id) || []) { nodes.add(e.from); edgeKeys.add(e.key); }
    return { nodes, edgeKeys };
}

/**
 * 生成"聚焦版"的 SVG 文本 (archify 的 data-focus-* 协议).
 *
 * 为什么在**文本层**做, 而不是拿到 DOM 后 setAttribute:
 *   我们的 SVG 是用 dangerouslySetInnerHTML 注进去的, React 每次重渲染都会
 *   重建这棵子树 —— 命令式打上的属性会被下一次渲染抹掉 (实测: 点节点后
 *   setFocusId 触发重渲染 -> 属性生效 -> 紧接着 setPassportPos 又一次重渲染
 *   -> 属性全没了). 改成 useMemo 派生字符串, 属性就是 React 渲染结果的一部分,
 *   任何重渲染都只会把它画回正确的样子.
 */
export function focusSvg(svg: string, facts: DiagramFacts, id: string | null): string {
    if (!svg || !id || !facts.nodes.has(id)) return svg;
    const { nodes, edgeKeys } = neighborhood(facts, id);

    /** 在标签收尾处插入属性 (自闭合与非自闭合都照顾到) */
    const withAttr = (tag: string, attrText: string): string => {
        const cleaned = tag
            .replace(/\s*data-focus-(?:match|selected)=""/g, '')
            .replace(/\s*aria-pressed="[^"]*"/g, '');
        return cleaned.replace(/\s*\/?>$/, (tail) => ' ' + attrText + (tail.includes('/') ? '/>' : '>'));
    };

    // 节点: 邻域内标 data-focus-match, 被选中的再标 data-focus-selected
    let out = svg.replace(/<g\b[^>]*data-node-id="[^"]+"[^>]*>/g, (tag) => {
        const nodeId = /data-node-id="([^"]+)"/.exec(tag)?.[1] || '';
        if (!nodes.has(nodeId)) return tag;
        const extra = 'data-focus-match=""'
            + (nodeId === id ? ' data-focus-selected=""' : '')
            + ' aria-pressed="' + (nodeId === id ? 'true' : 'false') + '"';
        return withAttr(tag, extra);
    });

    // 边: 只看直接相连的那些
    out = out.replace(/<path\b[^>]*data-edge-from="[^"]+"[^>]*>/g, (tag) => {
        const from = /data-edge-from="([^"]+)"/.exec(tag)?.[1] || '';
        const to = /data-edge-to="([^"]+)"/.exec(tag)?.[1] || '';
        const label = /data-edge-label="([^"]*)"/.exec(tag)?.[1] || '';
        const key = /data-edge-key="([^"]+)"/.exec(tag)?.[1] || (from + '\u0000' + to + '\u0000' + label);
        if (!edgeKeys.has(key)) return tag;
        return withAttr(tag, 'data-focus-match=""');
    });

    // 根: 标记"当前处于聚焦态" (CSS 据此变暗非邻域元素)
    return out.replace(/<svg\b[^>]*>/, (tag) => withAttr(tag, 'data-focus-active="' + id.replace(/"/g, '&quot;') + '"'));
}

/**
 * 把"当前语义视图"统一写进 SVG 字符串.
 *
 * archify 用一组 data-* 属性表达"读者此刻在干什么" (聚焦 / 透镜 / 路径 / 邻域预览).
 * 它们互相**互斥且优先级明确**, 这里集中在一个函数里算, 避免各处零散拼接导致
 * 两个状态同时挂在 svg 上 (archify 的 CSS 会因此出现互相矛盾的变暗规则).
 *
 * 必须在**文本层**做: SVG 走 dangerouslySetInnerHTML, 命令式改 DOM 会被下次重渲染抹掉.
 */
export type LensKind = string;

export interface DiagramView {
    /** 聚焦节点 (邻域高亮) */
    focusId?: string | null;
    /** 语义透镜: 选中的节点类型 */
    lensKind?: LensKind | null;
    /** 图例邻域预览: 悬停在图例项上 */
    previewKind?: LensKind | null;
    /** 路径探测: 已完成的有向路径节点序列 */
    route?: string[] | null;
    /** 视觉风格 (archify preset): classic / signal-flow / blueprint / editorial */
    preset?: string | null;
    /** 路径探测: 正在选择起点/终点 */
    routePicking?: 'source' | 'target' | null;
    /** 路径探测: 已选起点 */
    routeStart?: string | null;
    /** 路径探测: 已选起点, 可选终点集合 */
    routeCandidates?: Set<string> | null;
}

/** 在标签收尾处插入属性 (自闭合与非自闭合都照顾到) */
function withAttr(tag: string, attrText: string): string {
    const cleaned = tag
        .replace(/\s*data-(?:focus|lens|route|legend-preview|preset)[a-z-]*="[^"]*"/g, '')
        .replace(/\s*aria-pressed="[^"]*"/g, '');
    return cleaned.replace(/\s*\/?>$/, (tail) => ' ' + attrText + (tail.includes('/') ? '/>' : '>'));
}

/** 路径上每一跳的边 key (from->to), 用于把边标成 on-route */
function routeEdgeKeys(facts: DiagramFacts, route: string[]): Set<string> {
    const keys = new Set<string>();
    for (let i = 0; i < route.length - 1; i += 1) {
        const from = route[i];
        const to = route[i + 1];
        const edge = (facts.out.get(from) || []).find((e) => e.to === to);
        if (edge) keys.add(edge.key);
    }
    return keys;
}

/** 按节点类型取集合 (语义透镜 / 图例预览共用) */
function byKind(facts: DiagramFacts, kind: LensKind): Set<string> {
    const out = new Set<string>();
    for (const n of facts.nodes.values()) if (n.kind === kind) out.add(n.id);
    return out;
}

export function applyView(svg: string, facts: DiagramFacts, view: DiagramView): string {
    if (!svg) return svg;

    const focusId = view.focusId && facts.nodes.has(view.focusId) ? view.focusId : null;
    const route = (view.route || []).filter((id) => facts.nodes.has(id));
    const hasRoute = route.length > 1;
    const lensSet = view.lensKind ? byKind(facts, view.lensKind) : null;
    const previewSet = view.previewKind ? byKind(facts, view.previewKind) : null;

    // 互斥优先级: 路径 > 透镜 > 图例预览 > 聚焦 (同一个 svg 只允许一种"主动语义视图")
    const mode: 'route' | 'lens' | 'preview' | 'focus' | 'none' = hasRoute ? 'route'
        : lensSet ? 'lens' : previewSet ? 'preview' : focusId ? 'focus' : 'none';

    const focusSet = focusId ? neighborhood(facts, focusId).nodes : new Set<string>();
    const focusEdges = focusId ? neighborhood(facts, focusId).edgeKeys : new Set<string>();
    const routeNodes = new Set(route);
    const routeEdges = hasRoute ? routeEdgeKeys(facts, route) : new Set<string>();

    // 节点
    let out = svg.replace(/<g\b[^>]*data-node-id="[^"]+"[^>]*>/g, (tag) => {
        const id = /data-node-id="([^"]+)"/.exec(tag)?.[1] || '';
        let extra = '';
        if (mode === 'route') {
            if (routeNodes.has(id)) {
                extra = 'data-route-match="" data-focus-match=""'
                    + (id === route[0] ? ' data-route-start=""' : '')
                    + (id === route[route.length - 1] ? ' data-route-end=""' : '');
            }
        } else if (mode === 'lens') {
            if (lensSet!.has(id)) extra = 'data-lens-match="" data-lens-selected="" data-focus-match=""';
        } else if (mode === 'preview') {
            if (previewSet!.has(id)) extra = 'data-legend-preview-match="" data-focus-match=""';
        } else if (mode === 'focus') {
            if (focusSet.has(id)) {
                extra = 'data-focus-match=""'
                    + (id === focusId ? ' data-focus-selected=""' : '');
            }
        }
        // 路径选择态: 起点已定, 候选终点高亮, 其余压暗
        if (view.routePicking) {
            if (id === view.routeStart) extra += ' data-route-start=""';
            if (view.routeCandidates && view.routeCandidates.has(id)) extra += ' data-route-candidate=""';
        }
        if (!extra) return tag;
        extra += ' aria-pressed="' + (
            mode === 'focus' && id === focusId ? 'true' : 'false') + '"';
        return withAttr(tag, extra);
    });

    // 边
    out = out.replace(/<path\b[^>]*data-edge-from="[^"]+"[^>]*>/g, (tag) => {
        const from = /data-edge-from="([^"]+)"/.exec(tag)?.[1] || '';
        const to = /data-edge-to="([^"]+)"/.exec(tag)?.[1] || '';
        const label = /data-edge-label="([^"]*)"/.exec(tag)?.[1] || '';
        const key = /data-edge-key="([^"]+)"/.exec(tag)?.[1] || (from + String.fromCharCode(0) + to + String.fromCharCode(0) + label);
        let extra = '';
        if (mode === 'route') {
            if (routeEdges.has(key)) extra = 'data-route-match="" data-focus-match=""';
        } else if (mode === 'focus') {
            if (focusEdges.has(key)) extra = 'data-focus-match=""';
        } else if (mode === 'lens') {
            if (lensSet!.has(from) && lensSet!.has(to)) extra = 'data-lens-match="" data-focus-match=""';
            else if (lensSet!.has(from) || lensSet!.has(to)) extra = 'data-lens-peer="" data-focus-match=""';
        } else if (mode === 'preview') {
            if (previewSet!.has(from) && previewSet!.has(to)) extra = 'data-legend-preview-match="" data-focus-match=""';
            else if (previewSet!.has(from) || previewSet!.has(to)) extra = 'data-legend-preview-peer="" data-focus-match=""';
        }
        if (!extra) return tag;
        return withAttr(tag, extra);
    });

    // 根: 声明当前模式
    const rootAttrs = [
        mode === 'focus' ? 'data-focus-active="' + focusId + '"' : '',
        mode === 'lens' ? 'data-lens-active="' + view.lensKind + '"' : '',
        mode === 'preview' ? 'data-legend-preview-active="' + view.previewKind + '"' : '',
        mode === 'route' ? 'data-route-active="' + route.join(' ') + '"' : '',
        view.routePicking ? 'data-route-picking="' + view.routePicking + '"' : '',
        view.preset ? 'data-preset="' + view.preset + '"' : '',
    ].filter(Boolean).join(' ');
    if (!rootAttrs) return out;
    return out.replace(/<svg\b[^>]*>/, (tag) => withAttr(tag, rootAttrs));
}

/**
 * 把"当前语义视图"应用到**已挂载的 DOM** 上.
 *
 * 为什么不用字符串版 (focusSvg / 早先的 applyView(svg,...)):
 *   字符串版意味着 svg 内容随视图状态变化, React 每次重渲染都会重建整棵 SVG 子树.
 *   实测两个后果 (用户直接报的):
 *     1. **动效被打回起点**: 任何交互 (点节点/开面板) 都重启动画,
 *        观感上"不按箭头顺序""有时候倒着来" —— 因为那条边被重置后又从头跑;
 *     2. **点击失效**: mousedown 触发了重渲染 (setPanning), 光标下的节点被销毁,
 *        浏览器于是不会派发 click —— 路径探测"点了没用"就是这么来的.
 *
 *   所以现在 SVG 只注入一次 (dangerouslySetInnerHTML 用稳定的 a.svg),
 *   视图属性走这里**命令式**设置. React 不再碰这棵子树, 属性与动画都不会被打断.
 */
const VIEW_ATTRS = [
    'data-focus-match', 'data-focus-selected', 'data-focus-active',
    'data-lens-match', 'data-lens-selected', 'data-lens-peer', 'data-lens-active',
    'data-route-match', 'data-route-start', 'data-route-end', 'data-route-active',
    'data-route-picking', 'data-route-candidate',
    'data-legend-preview-match', 'data-legend-preview-peer', 'data-legend-preview-active',
];

export function applyViewToDom(root: HTMLElement | null, facts: DiagramFacts, view: DiagramView): void {
    if (!root) return;
    const svg = root.querySelector('svg');
    if (!svg) return;

    // 先全部清掉 (包括 aria-pressed), 再按当前视图重新打 —— 保证互斥
    for (const el of Array.from(svg.querySelectorAll('*')) as Element[]) {
        for (const a of VIEW_ATTRS) el.removeAttribute(a);
        if (el.hasAttribute('aria-pressed')) el.setAttribute('aria-pressed', 'false');
    }
    for (const a of VIEW_ATTRS) svg.removeAttribute(a);

    const focusId = view.focusId && facts.nodes.has(view.focusId) ? view.focusId : null;
    const route = (view.route || []).filter((id) => facts.nodes.has(id));
    const hasRoute = route.length > 1;
    const lensSet = view.lensKind ? byKind(facts, view.lensKind) : null;
    const previewSet = view.previewKind ? byKind(facts, view.previewKind) : null;

    // 互斥优先级: 路径 > 透镜 > 图例预览 > 聚焦
    const mode: 'route' | 'lens' | 'preview' | 'focus' | 'none' = hasRoute ? 'route'
        : lensSet ? 'lens' : previewSet ? 'preview' : focusId ? 'focus' : 'none';

    const focusNb = focusId ? neighborhood(facts, focusId) : null;
    const routeNodes = new Set(route);
    const routeEdges = hasRoute ? routeEdgeKeys(facts, route) : new Set<string>();

    // ---- 节点 ----
    for (const el of Array.from(svg.querySelectorAll('[data-node-id]')) as Element[]) {
        const id = el.getAttribute('data-node-id') || '';
        if (mode === 'route' && routeNodes.has(id)) {
            el.setAttribute('data-route-match', '');
            el.setAttribute('data-focus-match', '');
            if (id === route[0]) el.setAttribute('data-route-start', '');
            if (id === route[route.length - 1]) el.setAttribute('data-route-end', '');
        } else if (mode === 'lens' && lensSet!.has(id)) {
            el.setAttribute('data-lens-match', '');
            el.setAttribute('data-lens-selected', '');
            el.setAttribute('data-focus-match', '');
        } else if (mode === 'preview' && previewSet!.has(id)) {
            el.setAttribute('data-legend-preview-match', '');
            el.setAttribute('data-focus-match', '');
        } else if (mode === 'focus' && focusNb!.nodes.has(id)) {
            el.setAttribute('data-focus-match', '');
            if (id === focusId) {
                el.setAttribute('data-focus-selected', '');
                el.setAttribute('aria-pressed', 'true');
            }
        }
    }

    // ---- 边 ----
    for (const el of Array.from(svg.querySelectorAll('[data-edge-from]')) as Element[]) {
        const from = el.getAttribute('data-edge-from') || '';
        const to = el.getAttribute('data-edge-to') || '';
        const label = el.getAttribute('data-edge-label') || '';
        const key = el.getAttribute('data-edge-key') || (from + '\u0000' + to + '\u0000' + label);
        if (mode === 'route' && routeEdges.has(key)) {
            el.setAttribute('data-route-match', '');
            el.setAttribute('data-focus-match', '');
        } else if (mode === 'focus' && focusNb!.edgeKeys.has(key)) {
            el.setAttribute('data-focus-match', '');
        } else if (mode === 'lens') {
            if (lensSet!.has(from) && lensSet!.has(to)) {
                el.setAttribute('data-lens-match', '');
                el.setAttribute('data-focus-match', '');
            } else if (lensSet!.has(from) || lensSet!.has(to)) {
                el.setAttribute('data-lens-peer', '');
                el.setAttribute('data-focus-match', '');
            }
        } else if (mode === 'preview') {
            if (previewSet!.has(from) && previewSet!.has(to)) {
                el.setAttribute('data-legend-preview-match', '');
                el.setAttribute('data-focus-match', '');
            } else if (previewSet!.has(from) || previewSet!.has(to)) {
                el.setAttribute('data-legend-preview-peer', '');
                el.setAttribute('data-focus-match', '');
            }
        }
    }

    // ---- 根 ----
    if (mode === 'focus' && focusId) svg.setAttribute('data-focus-active', focusId);
    else if (mode === 'lens' && view.lensKind) svg.setAttribute('data-lens-active', view.lensKind);
    else if (mode === 'preview' && view.previewKind) svg.setAttribute('data-legend-preview-active', view.previewKind);
    else if (mode === 'route') svg.setAttribute('data-route-active', route.join(' '));
    if (view.routePicking) svg.setAttribute('data-route-picking', view.routePicking);
}
export interface PassportPos { left: number; top: number }

/**
 * 语义护照卡片.
 *
 * 内容刻意保持"事实陈述": 只列作者写进图里的事实 (标签/子标签/上下文/稳定 ID
 * /直接上下游), 不推断因果, 不猜运行时行为.
 *
 * 卡片上**不放任何动作按钮**: 点节点是"我想看这个节点是什么", 不是"我要对这个
 * 节点做点什么". 复制链接属于"带走这张图"的通用操作, 归工具条的分享菜单 ——
 * 放在节点卡里既与本意无关, 又和分享菜单重复.
 */
export function DiagramPassport({
    facts, id, pos, onClose, onGo,
}: {
    facts: DiagramFacts;
    id: string;
    pos: PassportPos;
    onClose: () => void;
    /** 点上下游条目 -> 聚焦到那个节点 */
    onGo: (next: string) => void;
}): React.ReactElement | null {
    const node = facts.nodes.get(id);
    if (!node) return null;
    const ups = facts.into.get(id) || [];
    const downs = facts.out.get(id) || [];

    return (
        <div className="hxd-passport" style={{ left: pos.left, top: pos.top }} role="dialog" aria-label="节点信息">
            <div className="hxd-passport__head">
                <span className="hxd-passport__eyebrow">语义护照</span>
                <button type="button" className="hxd-passport__close" onClick={onClose} aria-label="关闭" title="关闭 (Esc)">×</button>
            </div>
            <strong className="hxd-passport__title">{node.label}</strong>
            {node.sublabel ? <span className="hxd-passport__sub">{node.sublabel}</span> : null}
            <div className="hxd-passport__meta">
                {node.kind ? <span className="hxd-passport__chip" data-kind={node.kind}>{node.kind}</span> : null}
                {node.context ? <span className="hxd-passport__chip">{node.context}</span> : null}
                <code>{node.id}</code>
            </div>

            {ups.length ? (
                <div className="hxd-passport__rel">
                    <span className="hxd-passport__rel-title">上游 · {ups.length}</span>
                    {ups.map((e) => (
                        <button key={'u' + e.key} type="button" onClick={() => onGo(e.from)}>
                            ← {facts.nodes.get(e.from)?.label || e.from}{e.label ? ' · ' + e.label : ''}
                        </button>
                    ))}
                </div>
            ) : null}
            {downs.length ? (
                <div className="hxd-passport__rel">
                    <span className="hxd-passport__rel-title">下游 · {downs.length}</span>
                    {downs.map((e) => (
                        <button key={'d' + e.key} type="button" onClick={() => onGo(e.to)}>
                            → {facts.nodes.get(e.to)?.label || e.to}{e.label ? ' · ' + e.label : ''}
                        </button>
                    ))}
                </div>
            ) : null}
        </div>
    );
}
